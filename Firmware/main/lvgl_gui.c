#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
//#include "esp_flash.h"
//#include "esp_system.h"
//#include "driver/gpio.h"
#include "esp_timer.h"
#include "GUI/ui.h"
#include "lvgl_gui.h"
#include <esp_log.h>

#include "signal_capture.h"
#include "config.h"

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
#define USER_GUI_UPDATE_PERIOD_MS   150

#define USER_GUI_MAX_LINE_LENGTH    150

//Touchscreen driver
static lv_indev_drv_t indev_drv;
lv_indev_t * indev_touchpad;
static lv_color_t* disp_buf1;
static lv_color_t* disp_buf2;

lv_obj_t *table_state;
gps_gui_ch_t gps_channels_gui[GPS_SAT_CNT];

/// @brief Set be fast task at core 0, cleared by slow gui_task at core 1.
uint8_t gui_state_have_data_copied = 0;

#if (ENABLE_CALC_POSITION)
  sol_t gui_gps_sol = {0};
  double gui_final_pos[3];
#endif

//*************************************************************

static void lv_tick_task_cb(void *arg);
static void user_gui_update_cb(lv_timer_t * timer);

void lvgl_touchscreen_init(void);
void change_keyboard1(void);
void startup_actions(void);
void create_state_table(void);
void lvgl_gui_display_init(void);

void lvgl_redraw_state_screen(void);
void lvgl_generate_state_table_line(uint8_t sat_idx, char *line_txt);
uint16_t print_state_acquisition_channel(
    gps_gui_ch_t *channel, char *line_txt, uint16_t max_len);
uint16_t print_state_tracking_channel(
    gps_gui_ch_t *channel, char *line_txt, uint16_t max_len);
//*************************************************************
//*************************************************************

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;


void gui_task(void *pvParameter) 
{
    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();
   
   lvgl_gui_display_init();

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
       lv_timer_handler();
    }

    /* A task should NEVER return */
    free(disp_buf1);
    free(disp_buf2);
    vTaskDelete(NULL);
}

//Used for LVGL update
static void lv_tick_task_cb(void *arg) 
{
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void user_gui_update_cb(lv_timer_t * timer)
{
    lv_obj_t *activeScreen = lv_scr_act();

    if (activeScreen == ui_ScreenState)
        lvgl_redraw_state_screen();

}

void lvgl_gui_display_init(void)
{
    lv_init();
    lvgl_driver_init();

    disp_buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(disp_buf1 != NULL);
     /* Use double buffered when not working with monochrome displays */
    disp_buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(disp_buf2 != NULL);

    static lv_disp_draw_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;

    lv_disp_draw_buf_init(&disp_buf, disp_buf1, disp_buf2, size_in_px);

    static lv_disp_drv_t disp_drv;
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
        .callback = &lv_tick_task_cb,
        .name = "periodic_gui"
    };
    static esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    static uint32_t user_data = 0;
    lv_timer_create(
        user_gui_update_cb, USER_GUI_UPDATE_PERIOD_MS,  &user_data);

    lvgl_touchscreen_init();//must be called after display driver init
    //lv_port_indev_init(); //templates for input

    ui_init();
    startup_actions();
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
    memset(gps_channels_gui, 0, sizeof(gps_channels_gui));

    lv_obj_add_state(ui_btnStop, LV_STATE_DISABLED);
    lv_obj_add_state(ui_btnRestartAcq, LV_STATE_DISABLED);

    create_state_table();
    change_keyboard1();
}

void create_state_table(void)
{
    table_state = lv_table_create(ui_ScreenState);
    lv_obj_set_size(table_state, 309, 156);
    lv_obj_set_x(table_state, 6);
    lv_obj_set_y(table_state, 22);

    lv_obj_set_style_pad_top(table_state, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(table_state, 3, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(table_state, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(table_state, 2, LV_PART_ITEMS);

    lv_table_set_cell_value(table_state, 0, 0, "LINE 1");
    lv_table_set_cell_value(table_state, 1, 0, "LINE 2");
    lv_table_set_cell_value(table_state, 2, 0, "LINE 3");
    lv_table_set_cell_value(table_state, 3, 0, "LINE 4");
    lv_table_set_col_width(table_state, 0, 300);
    _ui_flag_modify(table_state, LV_OBJ_FLAG_SCROLLABLE, _UI_MODIFY_FLAG_REMOVE);
    _ui_flag_modify(table_state, LV_OBJ_FLAG_CLICKABLE, _UI_MODIFY_FLAG_REMOVE);
}

//####################################################

/// @brief Store GPS data for future dsplaying
/// Need to be called from GPS master
/// @param channels 
void lvgl_store_gps_state(gps_ch_t *channels)
{
    if (gui_state_have_data_copied) //Previoous data is not processed
        return;

    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
        gps_channels_gui[i].prn = channels[i].prn;
        gps_channels_gui[i].acq_state = channels[i].acq_data.state;
        gps_channels_gui[i].track_state = channels[i].tracking_data.state;
        gps_channels_gui[i].acq_found_freq_offset_hz = 
            channels[i].acq_data.found_freq_offset_hz;
        gps_channels_gui[i].acq_freq_index = channels[i].acq_data.freq_index;
        gps_channels_gui[i].track_code_phase_fine = 
            channels[i].tracking_data.code_phase_fine;

        gps_channels_gui[i].track_snr = 
            channels[i].tracking_data.snr_value;

        gps_channels_gui[i].nav_word_cnt = 
            channels[i].nav_data.word_cnt_test;

        gps_channels_gui[i].track_if_freq_offset_hz = 
            (int16_t)channels[i].tracking_data.if_freq_offset_hz;

        gps_channels_gui[i].nav_pol_found = channels[i].nav_data.polarity_found;
        gps_channels_gui[i].nav_subframe_cnt = channels[i].eph_data.sub_cnt;
        gps_channels_gui[i].eph_time = channels[i].eph_data.eph.ttr.time;
    }

    gui_state_have_data_copied = 1;
}

#if (ENABLE_CALC_POSITION)
/// @brief Save new position for later processing
/// Need to be called from GPS master
/// @param gps_sol_p - solution
/// @param position - 3 elements, geodetic position {lat,lon,h} (deg,m)
void lvgl_store_new_position(sol_t *gps_sol_p, double *position)
{
    if (gps_sol_p->stat != SOLQ_NONE) //Copy only good values
    {
        memcpy(&gui_gps_sol, gps_sol_p, sizeof(sol_t));
        memcpy(gui_final_pos, position, sizeof(gui_final_pos));
    }
}
#endif

//Called periodically from user_gui_update_cb <= TIMER <= lv_timer_handler() in gui_task()
void lvgl_redraw_state_screen(void)
{
    char tmp_txt[USER_GUI_MAX_LINE_LENGTH];
    char *write_prt = tmp_txt;

    if (gui_state_have_data_copied)
    {
        for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
        {
            lvgl_generate_state_table_line(i, tmp_txt);
            lv_table_set_cell_value(table_state, i, 0, tmp_txt);
        }
        gui_state_have_data_copied = 0;
    }

    time_t tmp_time = 0;
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
        if (gps_channels_gui[i].eph_time > 0)
            tmp_time = gps_channels_gui[i].eph_time;
    }

    //Time label
    memset(tmp_txt, 0, USER_GUI_MAX_LINE_LENGTH);
    float time = (float)signal_capture_get_packet_cnt() / 1000.0f;
    sprintf(tmp_txt, "SYS RUNTIME: %.1f s", time);
    lv_label_set_text(ui_lblSysTime1, tmp_txt);

    //Info label
    memset(tmp_txt, 0, USER_GUI_MAX_LINE_LENGTH);
    write_prt = tmp_txt;
    if (tmp_time > 0)
    {
        tmp_time = tmp_time - GPS_UTC_TIME_OFFSET_S;
        write_prt += sprintf(write_prt, "EPH UTC TIME: %s", ctime(&tmp_time));
    }

#if (ENABLE_CALC_POSITION)
  if (gui_gps_sol.stat != SOLQ_NONE)
  {
    tmp_time = gui_gps_sol.time.time - GPS_UTC_TIME_OFFSET_S;
    write_prt += sprintf(write_prt, "UTC TIME: %s", ctime(&tmp_time));
    write_prt += sprintf(write_prt, LV_SYMBOL_GPS "POS: %2.5f %2.5f", gui_final_pos[0], gui_final_pos[1]);
  }
#endif

    lv_label_set_text(ui_lblStateCommon, tmp_txt);
}

void lvgl_generate_state_table_line(uint8_t sat_idx, char *line_txt)
{
    memset(line_txt, 0, USER_GUI_MAX_LINE_LENGTH);
    if (sat_idx >= GPS_SAT_CNT)
        return;

    uint16_t len = USER_GUI_MAX_LINE_LENGTH;
    uint16_t char_cnt = 0;

    char_cnt = snprintf(line_txt, len, "PRN=%2d ", gps_channels_gui[sat_idx].prn);
    line_txt += char_cnt;
    len -= char_cnt;

    if (gps_channels_gui[sat_idx].track_state > GPS_TRACKNG_IDLE)
    {
        char_cnt = print_state_tracking_channel(
            &gps_channels_gui[sat_idx], line_txt, len);
    }
    else
    {
        char_cnt = print_state_acquisition_channel(
            &gps_channels_gui[sat_idx], line_txt, len);
    }
    line_txt += char_cnt;
    len -= char_cnt;
}

uint16_t print_state_tracking_channel(
    gps_gui_ch_t *channel, char *line_txt, uint16_t max_len)
{
    uint16_t char_cnt = 0;
    uint16_t len = max_len;

    switch (channel->track_state)
    {
    case GPS_NEED_PRE_TRACK:
        char_cnt = snprintf(line_txt, len, "NEED PRE_TRACK");
        break;
    case GPS_PRE_TRACK_RUN:
        char_cnt = snprintf(line_txt, len, "PRE_TRACK RUN");
        break;
    case GPS_PRE_TRACK_DONE:
        char_cnt = snprintf(line_txt, len, "PRE_TRACK DONE");
        break;
    case GPS_TRACKING_RUN:
        char_cnt = snprintf(line_txt, len, "*TRK* | ");
        break;
    default:
        break;
    }
    line_txt += char_cnt;
    len -= char_cnt;

    uint16_t code = (uint16_t)channel->track_code_phase_fine / 16;
    char_cnt = snprintf(line_txt, len, "SNR=%2.0fdB Freq=%5d Hz \nCode=%4d chips | wrd=%3d ", 
        channel->track_snr,
        channel->track_if_freq_offset_hz,
        code, 
        channel->nav_word_cnt);

    line_txt += char_cnt;
    len -= char_cnt;

    if (channel->nav_pol_found)
    {
        char_cnt = snprintf(line_txt, len, "|SUBF=%d", channel->nav_subframe_cnt);
        line_txt += char_cnt;
        len -= char_cnt;
    }

    return (max_len - len);
}

uint16_t print_state_acquisition_channel(
    gps_gui_ch_t *channel, char *line_txt, uint16_t max_len)
{
    uint16_t char_cnt = 0;
    uint16_t len = max_len;

    switch (channel->acq_state)
    {
    case GPS_ACQ_NEED_FREQ_SEARCH:
        char_cnt = snprintf(line_txt, len, "ACQ NEED FREQ_SEARCH ");
        break;
    case GPS_ACQ_FREQ_SEARCH_RUN:
        char_cnt = snprintf(line_txt, len, "ACQ FREQ_SEARCH_RUN << ");
        break;

    case GPS_ACQ_FREQ_SEARCH_DONE:
        char_cnt = snprintf(line_txt, len, "ACQ FREQ_SEARCH_DONE ");
        break;

    case GPS_ACQ_CODE_PHASE_SEARCH1:
        char_cnt = snprintf(line_txt, len, "ACQ CODE_SEARCH_1 ");
        break;
    case GPS_ACQ_CODE_PHASE_SEARCH2:
        char_cnt = snprintf(line_txt, len, "ACQ CODE_SEARCH_2 ");
        break;
    case GPS_ACQ_CODE_PHASE_SEARCH3:
        char_cnt = snprintf(line_txt, len, "ACQ CODE_SEARCH_3 ");
        break;

    case GPS_ACQ_CODE_PHASE_SEARCH1_DONE:
        char_cnt = snprintf(line_txt, len, "ACQ CODE_SEARCH_1_DONE ");
        break;
    case GPS_ACQ_CODE_PHASE_SEARCH2_DONE:
        char_cnt = snprintf(line_txt, len, "ACQ CODE_SEARCH_2_DONE ");
        break;
    case GPS_ACQ_CODE_PHASE_SEARCH3_DONE:
        char_cnt = snprintf(line_txt, len, "ACQ CODE_SEARCH_3_DONE ");
        break;
    default:
        break;
    }
    line_txt += char_cnt;
    len -= char_cnt;

    if (channel->acq_state == GPS_ACQ_FREQ_SEARCH_RUN)
    {
        uint16_t curr_f_idx = channel->acq_freq_index;
        uint16_t acq_percent = curr_f_idx * 100 / ACQ_COUNT;
        char_cnt = snprintf(line_txt, len, "SCAN: %d%%", acq_percent);

        line_txt += char_cnt;
        len -= char_cnt;
    }

    if (channel->acq_state >= GPS_ACQ_FREQ_SEARCH_DONE)
    {
        char_cnt = snprintf(line_txt, len, "\nFREQ=%5d Hz", channel->acq_found_freq_offset_hz);

        line_txt += char_cnt;
        len -= char_cnt;
    }


    return (max_len - len);
}
