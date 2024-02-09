/*
By ILIASAM 2024.
Based on gt911.c/h driver
*/

#include <esp_log.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif
#include "cst820.h"

#include "lvgl_i2c/i2c_manager.h"
#include "driver/gpio.h"
#include <rom/ets_sys.h>

#define TAG "CST820"

#define CST820_REG_GESTURE_ID       0x01
#define CST820_REG_CHIP_ID          0xA7
#define CST820_REG_DIS_AUTO_SLEEP   0xFE

uint8_t cst820_i2c_dev_addr = CST820_I2C_SLAVE_ADDR;

//****************************************************************

esp_err_t cst820_i2c_read(
  uint8_t slave_addr, uint16_t register_addr, uint8_t *data_buf, uint8_t len) 
{
    return lvgl_i2c_read(
        CONFIG_LV_I2C_TOUCH_PORT, slave_addr, register_addr, data_buf, len);
}

esp_err_t cst820_i2c_write8(uint8_t slave_addr, uint16_t register_addr, uint8_t data) 
{
    uint8_t buffer = data;
    return lvgl_i2c_write(
        CONFIG_LV_I2C_TOUCH_PORT, slave_addr, register_addr, &buffer, 1);
}

/**
  * @brief  Initialize for CST820 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C address of CST820).
  * @retval None
  */
void cst820_init(uint8_t dev_addr) 
{
    cst820_i2c_dev_addr = dev_addr;

    if (CONFIG_LV_CST820_RESET_PIN >= 0)
    {
	    gpio_set_direction(CONFIG_LV_CST820_RESET_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(CONFIG_LV_CST820_RESET_PIN, 0);//reset
        ets_delay_us(1000*30);
        gpio_set_level(CONFIG_LV_CST820_RESET_PIN, 1);//on
        ets_delay_us(1000*200);
        ESP_LOGI(TAG, "CST820 reset done");
    }


    uint8_t version;
    uint8_t versionInfo[3];
    esp_err_t ret;
    ESP_LOGI(TAG, "Init CST820 addr: %d", dev_addr);
    if ((ret = cst820_i2c_read(dev_addr, 0x15, &version, 1) != ESP_OK))
    {
        ESP_LOGE(TAG, "Error reading version from device: %d", ret);
        return;
    }
    if ((ret = cst820_i2c_read(dev_addr, CST820_REG_CHIP_ID, versionInfo, 3) != ESP_OK))
    {
        ESP_LOGE(TAG, "Error reading versionInfo from device: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "CST820 version %d, versionInfo: %d.%d.%d", version, versionInfo[0], versionInfo[1], versionInfo[2]);

    cst820_i2c_write8(dev_addr, CST820_REG_DIS_AUTO_SLEEP, 0XFF); //Disable entering low power

}

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool cst820_read(lv_indev_drv_t *drv, lv_indev_data_t *data) 
{
    uint8_t data_raw[6] ;
    cst820_i2c_read(cst820_i2c_dev_addr, CST820_REG_GESTURE_ID, data_raw, 6);

    uint8_t points_cnt = data_raw[1];
    uint16_t point_x = 0;
    uint16_t point_y = 0;

    if (points_cnt >= 1)
    {
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    
    point_x = ((data_raw[2] & 0x0f) << 8) | data_raw[3];
    point_y = ((data_raw[4] & 0x0f) << 8) | data_raw[5];

#if CONFIG_LV_CST820_SWAPXY
    int temp;
    temp = point_y;
    point_y = point_x;
    point_x = temp;
#endif

#if CONFIG_LV_CST820_INVERT_Y
    point_y = CONFIG_LV_TOUCH_Y_MAX_CST820 - point_y;
#endif

#if CONFIG_LV_CST820_INVERT_X  
    point_x = CONFIG_LV_TOUCH_X_MAX_CST820 - point_x;
#endif


#if CONFIG_LV_TOUCH_X_MAX_CST820
    if (point_x >= CONFIG_LV_TOUCH_X_MAX_CST820) //to remove LVGL warnings
        point_x = CONFIG_LV_TOUCH_X_MAX_CST820 - 1;
#endif

#if CONFIG_LV_TOUCH_Y_MAX_CST820
    if (point_y >= CONFIG_LV_TOUCH_Y_MAX_CST820) //to remove LVGL warnings
        point_y = CONFIG_LV_TOUCH_Y_MAX_CST820 - 1;
#endif

    data->point.x = point_x;
    data->point.y = point_y;

#if (0)
    uint8_t gestureID = data_raw[0];
    uint8_t event = data_raw[2] >> 6;
    ESP_LOGI(TAG, "gestureID %d, points %d, event %d X=%u Y=%u", gestureID, points_cnt, event, point_x, point_y);
#endif

    return false;
}
