/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * CUSTOMIZED, DMA and RX only !!!!
 */

#include <string.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/lldesc.h"
#include "soc/soc_caps.h"
#include "soc/spi_periph.h"
#include "soc/soc_memory_layout.h"
#include "hal/spi_ll.h"
#include "hal/spi_slave_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "esp_private/spi_common_internal.h"
#include "c_spi_slave.h"
#include "hal/spi_slave_hal.h"

//This GDMA related part will be introduced by GDMA dedicated APIs in the future. Here we temporarily use macros.
#if SOC_GDMA_SUPPORTED
#include "soc/gdma_struct.h"
#include "hal/gdma_ll.h"

#define spi_dma_ll_rx_enable_burst_data(dev, chan, enable)         gdma_ll_rx_enable_data_burst(&GDMA, chan, enable);
#define spi_dma_ll_tx_enable_burst_data(dev, chan, enable)         gdma_ll_tx_enable_data_burst(&GDMA, chan, enable);
#define spi_dma_ll_rx_enable_burst_desc(dev, chan, enable)         gdma_ll_rx_enable_descriptor_burst(&GDMA, chan, enable);
#define spi_dma_ll_tx_enable_burst_desc(dev, chan, enable)         gdma_ll_tx_enable_descriptor_burst(&GDMA, chan, enable);
#endif

static const char *SPI_TAG = "c_spi_slave";
#define SPI_CHECK(a, str, ret_val) \
    if (!(a)) { \
        ESP_LOGE(SPI_TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val); \
    }

#ifdef CONFIG_SPI_SLAVE_ISR_IN_IRAM
#define SPI_SLAVE_ISR_ATTR IRAM_ATTR
#else
#define SPI_SLAVE_ISR_ATTR
#endif

#ifdef CONFIG_SPI_SLAVE_IN_IRAM
#define SPI_SLAVE_ATTR IRAM_ATTR
#else
#define SPI_SLAVE_ATTR
#endif

typedef struct {
    int id;
    spi_bus_config_t bus_config;
    spi_slave_interface_config_t cfg;
    intr_handle_t intr;
    intr_handle_t intr_dma;
    spi_slave_hal_context_t hal;
    spi_slave_transaction_t *cur_trans;
    uint32_t flags;
    int max_transfer_sz;
    QueueHandle_t trans_queue;
    QueueHandle_t ret_queue;
    bool dma_enabled;
    bool cs_iomux;
    uint32_t tx_dma_chan;
    uint32_t rx_dma_chan;
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_handle_t pm_lock;
#endif
} c_spi_slave_t;

static c_spi_slave_t *spihost[SOC_SPI_PERIPH_NUM];

static void spi_dma_intr(void *arg);

void c_spi_slave_hal_prepare_data_rx(const spi_slave_hal_context_t *hal, void *buffer2);
void c_spi_slave_fill_descriptors(
    lldesc_t *dmadesc, const spi_slave_hal_context_t *hal, int len, void *buffer2_p);
void c_spi_slave_hal_init(
    spi_slave_hal_context_t *hal, const spi_slave_hal_config_t *hal_config);
static void c_spi_complete_init(void *arg);
//********************************************************************

static inline bool c_is_valid_host(spi_host_device_t host)
{
//SPI1 can be used as GPSPI only on ESP32
#if CONFIG_IDF_TARGET_ESP32
    return host >= SPI1_HOST && host <= SPI3_HOST;
#elif (SOC_SPI_PERIPH_NUM == 2)
    return host == SPI2_HOST;
#elif (SOC_SPI_PERIPH_NUM == 3)
    return host >= SPI2_HOST && host <= SPI3_HOST;
#endif
}

static inline bool SPI_SLAVE_ISR_ATTR c_bus_is_iomux(c_spi_slave_t *host)
{
    return host->flags&SPICOMMON_BUSFLAG_IOMUX_PINS;
}

void SPI_SLAVE_ISR_ATTR c_freeze_cs(c_spi_slave_t *host)
{
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, spi_periph_signal[host->id].spics_in, false);
}

// Use this function instead of cs_initial to avoid overwrite the output config
// This is used in test by internal gpio matrix connections
static inline void SPI_SLAVE_ISR_ATTR c_restore_cs(c_spi_slave_t *host)
{
    if (host->cs_iomux) {
        gpio_iomux_in(host->cfg.spics_io_num, spi_periph_signal[host->id].spics_in);
    } else {
        esp_rom_gpio_connect_in_signal(host->cfg.spics_io_num, spi_periph_signal[host->id].spics_in, false);
    }
}

esp_err_t c_spi_slave_initialize(spi_host_device_t host, const spi_bus_config_t *bus_config, const spi_slave_interface_config_t *slave_config, spi_dma_chan_t dma_chan)
{
    bool spi_chan_claimed;
    uint32_t actual_tx_dma_chan = 0;
    uint32_t actual_rx_dma_chan = 0;
    esp_err_t ret = ESP_OK;
    esp_err_t err;
    SPI_CHECK(c_is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
#ifdef CONFIG_IDF_TARGET_ESP32
    SPI_CHECK(dma_chan >= SPI_DMA_DISABLED && dma_chan <= SPI_DMA_CH_AUTO, "invalid dma channel", ESP_ERR_INVALID_ARG );
#elif CONFIG_IDF_TARGET_ESP32S2
    SPI_CHECK( dma_chan == SPI_DMA_DISABLED || dma_chan == (int)host || dma_chan == SPI_DMA_CH_AUTO, "invalid dma channel", ESP_ERR_INVALID_ARG );
#elif SOC_GDMA_SUPPORTED
    SPI_CHECK( dma_chan == SPI_DMA_DISABLED || dma_chan == SPI_DMA_CH_AUTO, "invalid dma channel, chip only support spi dma channel auto-alloc", ESP_ERR_INVALID_ARG );
#endif
    SPI_CHECK((bus_config->intr_flags & (ESP_INTR_FLAG_HIGH|ESP_INTR_FLAG_EDGE|ESP_INTR_FLAG_INTRDISABLED))==0, "intr flag not allowed", ESP_ERR_INVALID_ARG);
#ifndef CONFIG_SPI_SLAVE_ISR_IN_IRAM
    SPI_CHECK((bus_config->intr_flags & ESP_INTR_FLAG_IRAM)==0, "ESP_INTR_FLAG_IRAM should be disabled when CONFIG_SPI_SLAVE_ISR_IN_IRAM is not set.", ESP_ERR_INVALID_ARG);
#endif
    SPI_CHECK(slave_config->spics_io_num < 0 || GPIO_IS_VALID_GPIO(slave_config->spics_io_num), "spics pin invalid", ESP_ERR_INVALID_ARG);

    spi_chan_claimed=spicommon_periph_claim(host, "spi slave");
    SPI_CHECK(spi_chan_claimed, "host already in use", ESP_ERR_INVALID_STATE);

    spihost[host] = malloc(sizeof(c_spi_slave_t));
    if (spihost[host] == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    memset(spihost[host], 0, sizeof(c_spi_slave_t));
    memcpy(&spihost[host]->cfg, slave_config, sizeof(spi_slave_interface_config_t));
    memcpy(&spihost[host]->bus_config, bus_config, sizeof(spi_bus_config_t));
    spihost[host]->id = host;

    bool use_dma = (dma_chan != SPI_DMA_DISABLED);
    spihost[host]->dma_enabled = use_dma;
    if (use_dma) {
        ret = spicommon_dma_chan_alloc(host, dma_chan, &actual_tx_dma_chan, &actual_rx_dma_chan);
        if (ret != ESP_OK) {
            goto cleanup;
        }
    }

    err = spicommon_bus_initialize_io(host, bus_config, SPICOMMON_BUSFLAG_SLAVE|bus_config->flags, &spihost[host]->flags);
    if (err!=ESP_OK) {
        ret = err;
        goto cleanup;
    }
    if (slave_config->spics_io_num >= 0) {
        spicommon_cs_initialize(host, slave_config->spics_io_num, 0, !c_bus_is_iomux(spihost[host]));
        // check and save where cs line really route through
        spihost[host]->cs_iomux = (slave_config->spics_io_num == spi_periph_signal[host].spics0_iomux_pin) && c_bus_is_iomux(spihost[host]);
    }

    // The slave DMA suffers from unexpected transactions. Forbid reading if DMA is enabled by disabling the CS line.
    if (use_dma) c_freeze_cs(spihost[host]);

    int dma_desc_ct = 0;
    spihost[host]->tx_dma_chan = actual_tx_dma_chan;
    spihost[host]->rx_dma_chan = actual_rx_dma_chan;
    if (use_dma) {
        //See how many dma descriptors we need and allocate them
        dma_desc_ct = (bus_config->max_transfer_sz + SPI_MAX_DMA_LEN - 1) / SPI_MAX_DMA_LEN;
        if (dma_desc_ct == 0) dma_desc_ct = 1; //default to 4k when max is not given
        spihost[host]->max_transfer_sz = dma_desc_ct * SPI_MAX_DMA_LEN;
    } else {
        //We're limited to non-DMA transfers: the SPI work registers can hold 64 bytes at most.
        spihost[host]->max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;
    }
#ifdef CONFIG_PM_ENABLE
    err = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "spi_slave",
            &spihost[host]->pm_lock);
    if (err != ESP_OK) {
        ret = err;
        goto cleanup;
    }
    // Lock APB frequency while SPI slave driver is in use
    esp_pm_lock_acquire(spihost[host]->pm_lock);
#endif //CONFIG_PM_ENABLE

    //Create queues
    spihost[host]->trans_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_transaction_t *));
    spihost[host]->ret_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_transaction_t *));
    if (!spihost[host]->trans_queue || !spihost[host]->ret_queue) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // Link handler to SPI DMA IRQ
    int flags_dma = ESP_INTR_FLAG_INTRDISABLED;
    err = esp_intr_alloc(
        spicommon_irqdma_source_for_host(host), flags_dma, spi_dma_intr, (void *)spihost[host], &spihost[host]->intr_dma);
    if (err != ESP_OK) {
        ret = err;
        goto cleanup;
    }

    spi_slave_hal_context_t *hal = &spihost[host]->hal;
    //assign the SPI, RX DMA and TX DMA peripheral registers beginning address
    spi_slave_hal_config_t hal_config = {
        .host_id = host,
        .dma_in = SPI_LL_GET_HW(host),
        .dma_out = SPI_LL_GET_HW(host)
    };
    c_spi_slave_hal_init(hal, &hal_config);

    if (dma_desc_ct) {
        hal->dmadesc_tx = heap_caps_malloc(sizeof(lldesc_t) * dma_desc_ct, MALLOC_CAP_DMA);
        hal->dmadesc_rx = heap_caps_malloc(sizeof(lldesc_t) * dma_desc_ct, MALLOC_CAP_DMA);
        if (!hal->dmadesc_tx || !hal->dmadesc_rx) {
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
    }
    hal->dmadesc_n = dma_desc_ct;
    hal->rx_lsbfirst = (slave_config->flags & SPI_SLAVE_RXBIT_LSBFIRST) ? 1 : 0;
    hal->tx_lsbfirst = (slave_config->flags & SPI_SLAVE_TXBIT_LSBFIRST) ? 1 : 0;
    hal->mode = slave_config->mode;
    hal->use_dma = use_dma;
    hal->tx_dma_chan = actual_tx_dma_chan;
    hal->rx_dma_chan = actual_rx_dma_chan;

    spi_slave_hal_setup_device(hal);

    return ESP_OK;

cleanup:
    if (spihost[host]) {
        if (spihost[host]->trans_queue) vQueueDelete(spihost[host]->trans_queue);
        if (spihost[host]->ret_queue) vQueueDelete(spihost[host]->ret_queue);
        free(spihost[host]->hal.dmadesc_tx);
        free(spihost[host]->hal.dmadesc_rx);
#ifdef CONFIG_PM_ENABLE
        if (spihost[host]->pm_lock) {
            esp_pm_lock_release(spihost[host]->pm_lock);
            esp_pm_lock_delete(spihost[host]->pm_lock);
        }
#endif
    }
    spi_slave_hal_deinit(&spihost[host]->hal);
    if (spihost[host]->dma_enabled) {
        spicommon_dma_chan_free(host);
    }

    free(spihost[host]);
    spihost[host] = NULL;
    spicommon_periph_free(host);

    return ret;
}
//Taken from "spi_slave_hal.c"
void c_spi_slave_hal_init(
    spi_slave_hal_context_t *hal, const spi_slave_hal_config_t *hal_config)
{
    memset(hal, 0, sizeof(spi_slave_hal_context_t));
    spi_dev_t *hw = SPI_LL_GET_HW(hal_config->host_id);
    hal->hw = hw;
    hal->dma_in = hal_config->dma_in;
    hal->dma_out = hal_config->dma_out;

    spi_dma_ll_rx_enable_burst_data(hal->dma_in, hal->rx_dma_chan, 1);
    spi_dma_ll_tx_enable_burst_data(hal->dma_out, hal->tx_dma_chan, 1);
    spi_dma_ll_rx_enable_burst_desc(hal->dma_in, hal->rx_dma_chan, 1);
    spi_dma_ll_tx_enable_burst_desc(hal->dma_out, hal->tx_dma_chan, 1);

    spi_ll_slave_init(hal->hw);

    //But without enabling SPI IRQ
}

esp_err_t c_spi_slave_free(spi_host_device_t host)
{
    SPI_CHECK(c_is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    if (spihost[host]->trans_queue) vQueueDelete(spihost[host]->trans_queue);
    if (spihost[host]->ret_queue) vQueueDelete(spihost[host]->ret_queue);
    if (spihost[host]->dma_enabled) {
        spicommon_dma_chan_free(host);
    }
    spicommon_bus_free_io_cfg(&spihost[host]->bus_config);
    free(spihost[host]->hal.dmadesc_tx);
    free(spihost[host]->hal.dmadesc_rx);
    esp_intr_free(spihost[host]->intr);
    esp_intr_free(spihost[host]->intr_dma);
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_release(spihost[host]->pm_lock);
    esp_pm_lock_delete(spihost[host]->pm_lock);
#endif //CONFIG_PM_ENABLE
    free(spihost[host]);
    spihost[host] = NULL;
    spicommon_periph_free(host);
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ATTR c_spi_slave_queue_trans(spi_host_device_t host, const spi_slave_transaction_t *trans_desc, TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(c_is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]->dma_enabled == 0 || trans_desc->tx_buffer==NULL || esp_ptr_dma_capable(trans_desc->tx_buffer),
			"txdata not in DMA-capable memory", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]->dma_enabled == 0 || trans_desc->rx_buffer==NULL ||
        (esp_ptr_dma_capable(trans_desc->rx_buffer) && esp_ptr_word_aligned(trans_desc->rx_buffer) &&
            (trans_desc->length%4==0)),
        "rxdata not in DMA-capable memory or not WORD aligned", ESP_ERR_INVALID_ARG);

    SPI_CHECK(trans_desc->length <= spihost[host]->max_transfer_sz * 8, "data transfer > host maximum", ESP_ERR_INVALID_ARG);
    r = xQueueSend(spihost[host]->trans_queue, (void *)&trans_desc, ticks_to_wait);
    if (!r) 
        return ESP_ERR_TIMEOUT;
    c_spi_complete_init(spihost[host]);
    return ESP_OK;
}


esp_err_t SPI_SLAVE_ATTR c_spi_slave_get_trans_result(spi_host_device_t host, spi_slave_transaction_t **trans_desc, TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(c_is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    r = xQueueReceive(spihost[host]->ret_queue, (void *)trans_desc, ticks_to_wait);
    if (!r) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}


//Start transmit, leave without waiting
//Copied from "spi_slave_transmit"
esp_err_t SPI_SLAVE_ATTR c_spi_slave_start_transmit(
    spi_host_device_t host, spi_slave_transaction_t *trans_desc, TickType_t ticks_to_wait)
{
    esp_err_t ret;
    //xQueueSend
    ret = c_spi_slave_queue_trans(host, trans_desc, ticks_to_wait);
    return ret;
}

//This not an interrupt now!
//static void SPI_SLAVE_ISR_ATTR spi_intr(void *arg)
static void c_spi_complete_init(void *arg)
{
    BaseType_t r;
    BaseType_t do_yield = pdFALSE;
    spi_slave_transaction_t *trans = NULL;
    c_spi_slave_t *host = (c_spi_slave_t *)arg;
    spi_slave_hal_context_t *hal = &host->hal;

    // Disable SPI interrupts
    esp_intr_disable(host->intr);

#if CONFIG_IDF_TARGET_ESP32
    // This workaround is only for esp32
    // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
    spicommon_dmaworkaround_idle(host->tx_dma_chan);
    if (spicommon_dmaworkaround_reset_in_progress())
    {
        // We need to wait for the reset to complete. Disable int (will be re-enabled on reset callback) and exit isr.
        esp_intr_disable(host->intr);
        if (do_yield)
            portYIELD_FROM_ISR();
        return;
    }
#endif // #if CONFIG_IDF_TARGET_ESP32

    r = xQueueReceive(host->trans_queue, &trans, portMAX_DELAY);
    if (r)
    {
        // sanity check
        assert(trans);

        // We have a transaction. Send it.
        host->cur_trans = trans;

        hal->bitlen = trans->length;
        hal->rx_buffer = trans->rx_buffer;
        void *rx_buffer2 = trans->rx_buffer2;
        hal->tx_buffer = trans->tx_buffer;

#if CONFIG_IDF_TARGET_ESP32
        // This workaround is only for esp32
        // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
        spicommon_dmaworkaround_transfer_active(host->tx_dma_chan);
#endif // #if CONFIG_IDF_TARGET_ESP32

        c_spi_slave_hal_prepare_data_rx(hal, rx_buffer2);
        hal->hw->dma_conf.dma_continue = 1;
        hal->hw->dma_int_clr.val = 0xFFFF; // Clear SPI DMA irq
        hal->hw->dma_int_ena.val = 0;
        hal->hw->dma_int_ena.in_suc_eof = 1; // enable SPI DMA IRQ
        esp_intr_enable(host->intr_dma);

        // The slave rx dma get disturbed by unexpected transaction. Only connect the CS when slave is ready.
        c_restore_cs(host);

        // Kick off transfer
        spi_slave_hal_user_start(hal);
        if (host->cfg.post_setup_cb)
            host->cfg.post_setup_cb(trans);
    }

    if (do_yield)
        portYIELD_FROM_ISR();

    return;
}

/// @brief SPI DMA IRQ handler
/// @param arg 
/// @return 
static void SPI_SLAVE_ISR_ATTR spi_dma_intr(void *arg)
{
    c_spi_slave_t *host = (c_spi_slave_t *)arg;
    spi_slave_hal_context_t *hal = &host->hal;

    hal->hw->dma_int_clr.in_suc_eof = 1;//Clear SPI DMA IRQ
    //c_freeze_cs(host);//stop

    if (host->cfg.post_trans_cb) 
        host->cfg.post_trans_cb(host->cur_trans);
}

//Copied from "spi_slave_hal_iram.c"
//DMA only, RX only
//Called from c_spi_complete_init()
void c_spi_slave_hal_prepare_data_rx(const spi_slave_hal_context_t *hal, void *buffer2_p)
{
    // Fill DMA descriptors
    if (hal->rx_buffer)
    {
        int len_bytes = ((hal->bitlen + 7) / 8);
        //lldesc_setup_link(hal->dmadesc_rx, hal->rx_buffer, len_bytes, true);
        c_spi_slave_fill_descriptors(hal->dmadesc_rx, hal, len_bytes, buffer2_p);

        // reset dma inlink, this should be reset before spi related reset
        spi_dma_ll_rx_reset(hal->dma_in, hal->rx_dma_chan);
        spi_ll_dma_rx_fifo_reset(hal->dma_in);
        spi_ll_slave_reset(hal->hw);
        spi_ll_infifo_full_clr(hal->hw);

        spi_ll_dma_rx_enable(hal->hw, 1);
        spi_dma_ll_rx_start(hal->dma_in, hal->rx_dma_chan, &hal->dmadesc_rx[0]);
    }

    //This value is realy used, but I don't know how
    //From documentation - needed in half-duplex mode
    spi_ll_slave_set_rx_bitlen(hal->hw, hal->bitlen);
    spi_ll_slave_set_tx_bitlen(hal->hw, hal->bitlen);

#ifdef CONFIG_IDF_TARGET_ESP32
    //SPI Slave mode on ESP32 requires MOSI/MISO enable
    spi_ll_enable_mosi(hal->hw, (hal->rx_buffer == NULL) ? 0 : 1);
    spi_ll_enable_miso(hal->hw, 0);
#endif
}

//Based on "lldesc_setup_link_constrained" (or "lldesc_setup_link"), RX only
//len - in bytes
// This is used for DMA configuring
void c_spi_slave_fill_descriptors(
    lldesc_t *dmadesc, const spi_slave_hal_context_t *hal, int len, void *buffer2_p)
{
    int n = 0;

    int dmachunklen = len;
    if (dmachunklen > LLDESC_MAX_NUM_PER_DESC) 
    {
        dmachunklen = LLDESC_MAX_NUM_PER_DESC;
    }
    //N = 0

    //Receive needs DMA length rounded to next 32-bit boundary
    dmadesc[n].size = (dmachunklen + 3) & (~3);
    dmadesc[n].length = (dmachunklen + 3) & (~3);
    
    dmadesc[n].buf = (uint8_t *)hal->rx_buffer;
    dmadesc[n].sosf = 0;
    dmadesc[n].owner = 1;
    dmadesc[n].qe.stqe_next = &dmadesc[n + 1];
    dmadesc[n].eof = 0;

    n = 1;
    dmadesc[n].size = (dmachunklen + 3) & (~3);
    dmadesc[n].length = (dmachunklen + 3) & (~3);
    
    dmadesc[n].buf = (uint8_t *)buffer2_p;
    dmadesc[n].sosf = 0;
    dmadesc[n].owner = 1;
    dmadesc[n].qe.stqe_next = &dmadesc[0]; //go to descriptor 0
    dmadesc[n].eof = 0;

    //dmadesc[n - 1].qe.stqe_next = NULL;
    //dmadesc[n].eof = 1;//Mark last DMA desc as end of stream.
}
