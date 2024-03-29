// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.3
// Project name: SquareLine_Project-GPS

#include "../ui.h"

void ui_ScreenState_screen_init(void)
{
    ui_ScreenState = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScreenState, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label2 = lv_label_create(ui_ScreenState);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, -136);
    lv_obj_set_y(ui_Label2, -108);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "State");

    ui_lblStateCommon = lv_label_create(ui_ScreenState);
    lv_obj_set_width(ui_lblStateCommon, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblStateCommon, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblStateCommon, 10);
    lv_obj_set_y(ui_lblStateCommon, 181);
    lv_label_set_text(ui_lblStateCommon, "INFO");

    ui_lblSysTime1 = lv_label_create(ui_ScreenState);
    lv_obj_set_width(ui_lblSysTime1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblSysTime1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblSysTime1, 108);
    lv_obj_set_y(ui_lblSysTime1, 5);
    lv_label_set_text(ui_lblSysTime1, "SYS RUNTIME");
    lv_obj_set_style_text_color(ui_lblSysTime1, lv_color_hex(0x0704BC), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblSysTime1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_ScreenState, ui_event_ScreenState, LV_EVENT_ALL, NULL);

}
