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
#include "GUI/ui.h"
#include <esp_log.h>

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "lvgl_helpers.h"
#include "touch_driver.h"

#include "ui.h"

#define LV_TICK_PERIOD_MS 1
#define GUI_TAG "LV_GUI"

//Touchscreen driver
static lv_indev_drv_t indev_drv;
lv_indev_t * indev_touchpad;

static void lv_tick_task(void *arg);
void lvgl_touchscreen_init(void);
void change_keyboard1(void);
void startup_actions(void);
void create_state_panel(void);

lv_obj_t * table_state;

//*************************************************************

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;
static void lv_tick_task(void *arg) 
{
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

void gui_task(void *pvParameter) 
{
    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();
    lv_init();
    lvgl_driver_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
     /* Use double buffered when not working with monochrome displays */
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);

    static lv_disp_draw_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;

    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    disp_drv.hor_res = 320;
    disp_drv.ver_res = 240;

    #if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
    #endif

    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    lvgl_touchscreen_init();//must be called after display driver init
    //lv_port_indev_init(); //templates input

    ui_init();
    startup_actions();

    while (1) 
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) 
        {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }

    /* A task should NEVER return */
    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}

void lvgl_touchscreen_init(void)
{
    //HW is initialized in gui_task() => lvgl_driver_init() => touch_driver_init()

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_driver_read;//from "touch_driver.h"
    indev_touchpad = lv_indev_drv_register(&indev_drv);
    if (indev_touchpad == NULL)
        ESP_LOGE(GUI_TAG, "Touchscreen init fail");
}

void change_keyboard1(void)
{
    /*Create a keyboard map*/
    static const char * kb_map[] = {
        "1", "2", "3", "\n", 
        "4", "5", "6", "\n", 
        "7", "8", "9", "\n", 
        LV_SYMBOL_BACKSPACE, "0", LV_SYMBOL_OK, NULL };

    /*Set the relative width of the buttons and other controls*/
    static const lv_btnmatrix_ctrl_t kb_ctrl[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    lv_keyboard_set_map(ui_Keyboard1, LV_KEYBOARD_MODE_USER_1, kb_map, kb_ctrl);
    lv_keyboard_set_mode(ui_Keyboard1, LV_KEYBOARD_MODE_USER_1);
}

void startup_actions(void)
{
    lv_obj_add_state(ui_btnStop, LV_STATE_DISABLED);
    lv_obj_add_state(ui_btnRestartAcq, LV_STATE_DISABLED);

    create_state_panel();
    change_keyboard1();
}

void create_state_panel(void)
{
    table_state = lv_table_create(ui_ScreenState);
    lv_obj_set_size(table_state, 309, 136);
    lv_obj_set_x(table_state, 6);
    lv_obj_set_y(table_state, 26);

    lv_obj_set_style_pad_top(table_state, 5, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(table_state, 5, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(table_state, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(table_state, 2, LV_PART_ITEMS);

    lv_table_set_cell_value(table_state, 0, 0, "LINE 1");
    lv_table_set_cell_value(table_state, 1, 0, "LINE 2");
    lv_table_set_cell_value(table_state, 2, 0, "LINE 3");
    lv_table_set_cell_value(table_state, 3, 0, "LINE 4 **********************");
    lv_table_set_col_width(table_state, 0, 300);
    _ui_flag_modify(table_state, LV_OBJ_FLAG_SCROLLABLE, _UI_MODIFY_FLAG_REMOVE);
    _ui_flag_modify(table_state, LV_OBJ_FLAG_CLICKABLE, _UI_MODIFY_FLAG_REMOVE);

    lv_table_set_cell_value(table_state, 0, 0, "### LINE 1 NEW");
}


