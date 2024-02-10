#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <esp_log.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "c_spi_slave.h" //CUSTOMIZED!!
//#include "esp_timer.h"
#include "timer_u32.h"
#include "config.h"
#include "signal_capture.h"

//SPI is working in 8-bit mode, slave, receive only
//Data is written to the RAM using circular DMA
//One 16-bit word is one GPS PRN "chip"

static const char *TAG = "s_capture";

// Circular buffer, updated by DMA in realtime
//8bit type here!!!
WORD_ALIGNED_ATTR uint8_t spi_rx_buffer0[GPS_DATA_WORDS_CNT * 2];//2048 bytes
WORD_ALIGNED_ATTR uint8_t spi_rx_buffer1[GPS_DATA_WORDS_CNT * 2];

// Software can read from this pointer
volatile uint16_t* spi_curr_ready_rx_buf = (uint16_t*)spi_rx_buffer1;

// TMP buffer for long processing
uint16_t spi_rx_copy_buffer[GPS_DATA_WORDS_CNT];

uint8_t signal_capture_need_copy_flag = 0;
volatile uint8_t signal_capture_irq_unprocessed_flag = 0;

/// @brief main time counter, 1 packet = 1ms, 1 PRN chip
volatile uint32_t signal_capture_packet_cnt = 0;

/// @brief Thais value is updated when DMA IRQ happens, 1ms period, in timer ticks
volatile uint32_t signal_capture_irq_timestamp = 0;//DWT timer

// Number of copied for slow reading
volatile uint16_t test_cnt = 0;

void signal_capture_spi_init(void);

/* Stores the handle of the task that will be notified when the
transmission is complete. */
static TaskHandle_t signal_capture_task_to_notify = NULL;

//****************************************

// Called from IRQ handler, when filling one buffer is done
// Called every 1ms (see PRN_SPEED_HZ)
void gps_spi_dma_transfer_done(spi_slave_transaction_t *trans)
{
    static uint8_t buffer_idx = 0;

    if (buffer_idx == 0)
        spi_curr_ready_rx_buf = (uint16_t*)&spi_rx_buffer0[0];
    else
        spi_curr_ready_rx_buf = (uint16_t*)&spi_rx_buffer1[0];
    buffer_idx ^= 1; //swap

    signal_capture_packet_cnt++;
    signal_capture_irq_timestamp = timer_u32();
    signal_capture_irq_unprocessed_flag = 1;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    //Notify high priority task that we had IRQ
    //MCU will switch to High priority task instantly after portYIELD_FROM_ISR()
    assert(signal_capture_task_to_notify != NULL);
    vTaskNotifyGiveIndexedFromISR(signal_capture_task_to_notify,
                                   (UBaseType_t)0,
                                   &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/// @brief Must be called at the startup from high priority task
/// @param  
void signal_capture_init_task(void)
{
    assert(signal_capture_task_to_notify == NULL);
    signal_capture_task_to_notify = xTaskGetCurrentTaskHandle();
}

/// @brief Return count of received data blocks, a kind of of system time
/// @param  
/// @return Count of received packets, or time in ms
uint32_t signal_capture_get_packet_cnt(void)
{
    return signal_capture_packet_cnt;
}


/// @brief Return pointer to the buffer with copied data, fixed address
/// @param  
/// @return Return pointer to the buffer with copied data, fixed address
uint8_t* signal_capture_get_copy_buf(void)
{
    return (uint8_t*)spi_rx_copy_buffer;
}


/// @brief Return pointer to a "fast" buffer, that is already filled
/// Data can be overwritten by DMA in 1ms! Need to be processed fast.
/// Also reset have new data flag
/// @return Pointer to a "fast" buffer
uint8_t* signal_capture_get_ready_buf(void)
{
  signal_capture_irq_unprocessed_flag = 0;
  return (uint8_t*)spi_curr_ready_rx_buf;
}



/// @brief Must be called periodically, used for data copy
/// @param  
void signal_capture_handling(void)
{
    if (signal_capture_irq_unprocessed_flag == 0)
    {
        return;
    }

    //Copy data for slow processing
    if (signal_capture_need_copy_flag)
    {
        uint32_t time_now = timer_u32();
        uint32_t time_diff = time_now - signal_capture_irq_timestamp;
        if (timer_delta_us(time_diff) > 900)    // PRN period is 1ms
            return;                             // to much time from IRQ, possibly do not have time to copy

        //NVIC_DisableIRQ(SPI_DMA_IRQ);
        memcpy(spi_rx_copy_buffer,
               (void *)spi_curr_ready_rx_buf,
               sizeof(spi_rx_copy_buffer));
        test_cnt++;
        signal_capture_need_copy_flag = 0;
        signal_capture_irq_unprocessed_flag = 0;
    }
}

/// @brief Notify signal capturing part that we need copied data.
/// Called by external code
/// @param  
void signal_capture_need_data_copy(void)
{
  signal_capture_need_copy_flag = 1;
}

/// @brief Check copy data state
/// @param  
/// @return Return 1 if data was copied
uint8_t signal_capture_check_copied(void)
{
  return (signal_capture_need_copy_flag == 0);
}


//********************************************************

void signal_capture_init(void)
{
    signal_capture_spi_init();

    gpio_set_direction(GPS_SPI_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_SPI_CS_PIN, 0);
    gpio_set_pull_mode(GPS_SPI_CS_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPS_SPI_MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPS_SPI_CLK_PIN, GPIO_PULLUP_ONLY);

    memset(spi_rx_buffer0, 0, sizeof(spi_rx_buffer0));
    memset(spi_rx_buffer1, 0, sizeof(spi_rx_buffer1));

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = BITS_IN_PRN;//2046bytes*8bit = 1023words*16bit
    //t.length = 2048 * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = &spi_rx_buffer0;
    t.rx_buffer2 = &spi_rx_buffer1;
    c_spi_slave_start_transmit(GPS_SPI_NAME, &t, portMAX_DELAY);
}

//Callback called after the SPI registers are loaded with new data.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    ESP_LOGI(TAG, "SPI+DMA Init done");
}

void signal_capture_spi_init(void)
{
    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPS_SPI_MOSI_PIN,//slave input
        .sclk_io_num = GPS_SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_DMA_LEN * 2,//to allocate descriptors for two buffers
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 3,
        .spics_io_num = GPS_SPI_CS_PIN,
        .queue_size = 3,
        .flags = SPI_SLAVE_BIT_LSBFIRST,
        .post_setup_cb = my_post_setup_cb,//called from  c_spi_slave_hal_prepare_data_rx()
        .post_trans_cb = gps_spi_dma_transfer_done//called from IRQ spi_dma_intr()
    };

    //Initialize SPI slave interface
    esp_err_t ret = c_spi_slave_initialize(GPS_SPI_NAME, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

    ESP_LOGI(TAG, "RES: %d, 0 is OK", ret);

    assert(ret == ESP_OK);
}