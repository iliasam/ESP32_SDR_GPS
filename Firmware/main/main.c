/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "gui.h"
#include "hardware.h"
#include "config.h"
#include <rom/ets_sys.h>
#include "timer_u32.h"

#include "signal_capture.h"
#include "gps_misc.h"
#include "acquisition.h"
#include "tracking.h"
#include "gps_master.h"

#if (ENABLE_CALC_POSITION)
  #include "solving.h"
#endif

gps_ch_t gps_channels[GPS_SAT_CNT];

//***************************************************************

void main_task(void *pvParameter);
void fast_task(void *pvParameter);

uint8_t need_slow_data_proc(void);
void main_slow_data_proc(void);
void main_fast_data_proc(void);
void gps_new_slow_data_handling(void);

//***************************************************************
//***************************************************************

void app_main(void)
{
    printf("MCU SDR GPS - ILIASAM 2024\n");

    pcb_hardware_init();
    gps_fill_summ_table();
    memset(&gps_channels[0], 0, sizeof(gps_channels));

    // user can enter known doppler frequency to make acquisition much faster
    gps_channels[0].prn = 5;
    gps_channels[0].acq_data.given_freq_offset_hz = 900;
    gps_channell_prepare(&gps_channels[0]);

    gps_channels[1].prn = 14;
    gps_channels[1].acq_data.given_freq_offset_hz = 4000;
    gps_channell_prepare(&gps_channels[1]);

    gps_channels[2].prn = 20;
    gps_channels[2].acq_data.given_freq_offset_hz = -1000;
    gps_channell_prepare(&gps_channels[2]);

    gps_channels[3].prn = 30;
    gps_channels[3].acq_data.given_freq_offset_hz = 2000;
    gps_channell_prepare(&gps_channels[3]);

#if (ENABLE_CALC_POSITION)
    gps_pos_solve_init(gps_channels);
#endif

    xTaskCreatePinnedToCore(fast_task, "fast_tsk", 4096*2, NULL, 3, NULL, 0); //Core 0
    xTaskCreatePinnedToCore(main_task, "main", 4096*2, NULL, 2, NULL, 0); //Core 0
    xTaskCreatePinnedToCore(gui_task, "gui", 4096*2, NULL, 0, NULL, 1);//Core 1
}

//High priority task, core 0
void fast_task(void *pvParameter)
{
  static uint16_t blink_cnt = 0;
  signal_capture_init_task();

  while (1)
  {
    //Wait for DMA IRQ, IRQ happens with 1ms period
    uint32_t res = ulTaskNotifyTakeIndexed(
      (UBaseType_t)0, pdTRUE, pdMS_TO_TICKS(10));

    if (res == 1)
    {
      blink_cnt++;
      if (blink_cnt >= 256)
        blink_cnt = 0;

      gpio_set_level(LED_R_PIN, (blink_cnt > 128));
      

      uint8_t need_slow = gps_master_need_acq();
      if (need_slow)
      {
        // Process IRQ, data copy if needed
        // Data will be processed slowly in "main_task"
        signal_capture_handling(); 
      }
      else
      {
        // Tracking!
        main_fast_data_proc();
      }
    }
    else
    {
      // timeout
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

//Less priority task, core 0
void main_task(void *pvParameter)
{
    signal_capture_init();
    signal_capture_need_data_copy();
    while (1)
    {
        uint8_t need_slow = gps_master_need_acq();
        if (need_slow)
        {
            // Not realtime
            main_slow_data_proc();
        }
        else
        {
            vTaskDelay(1); //Give time for RTOS
        }
    }

    vTaskDelete(NULL);
}

//Tracking
void main_fast_data_proc(void)
{
  uint8_t* signal_p = signal_capture_get_ready_buf();
  uint32_t time_cnt = signal_capture_get_packet_cnt();
  
  //4 channel multiplexing
  uint32_t index_big = time_cnt % (TRACKING_CH_LENGTH * GPS_SAT_CNT + 1); //real index, 0-16 range
  
  uint8_t sat_index = index_big / TRACKING_CH_LENGTH;
  if (sat_index >= GPS_SAT_CNT)
    sat_index = 0;
  
  uint32_t index = 0;
  if (index_big == (TRACKING_CH_LENGTH * GPS_SAT_CNT))
    index = 0xFF;
  else
  {
    index = index_big % TRACKING_CH_LENGTH;
  }
  
  //"index" can be 0-3 or 0xFF
  gps_tracking_process(&gps_channels[sat_index], signal_p, (uint8_t)index);
  
  gps_master_handling(gps_channels, index);
}

// Acqusition data capure and process - not realtime
void main_slow_data_proc(void)
{
  uint8_t have_data = signal_capture_check_copied();
  if (have_data == 0)
  {
    vTaskDelay(1); //Give time for RTOS
    return;
  }
  else
  {
    //have new data
    gps_new_slow_data_handling();
    signal_capture_need_data_copy();
    vTaskDelay(1);//Give time for RTOS
  }
}

// Acquisition data process
// Can be long!
void gps_new_slow_data_handling(void)
{
  uint8_t* signal_p = signal_capture_get_copy_buf();
  acquisition_process(&gps_channels[0], signal_p);
  gps_master_handling(gps_channels, 0);
}





