#ifndef __CST820_H
/*
By ILIASAM 2024.
Based on gt911.c/h driver
*/

#define __CST820_H

#include <stdint.h>
#include <stdbool.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CST820_I2C_SLAVE_ADDR   0x15




/**
  * @brief  Initialize for CST820 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C slave address of CST820).
  * @retval None
  */
void cst820_init(uint8_t dev_addr);

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool cst820_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

#ifdef __cplusplus
}
#endif
#endif /* __CST820_H */
