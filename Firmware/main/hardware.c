//Controling some PCB hardware from here

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "config.h"

void pcb_hardware_init(void)
{
    gpio_set_direction(LED_R_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B_PIN, GPIO_MODE_OUTPUT);

    //LEDs are inverted
    gpio_set_level(LED_R_PIN, 1);
    gpio_set_level(LED_G_PIN, 1);
    gpio_set_level(LED_B_PIN, 1);

}