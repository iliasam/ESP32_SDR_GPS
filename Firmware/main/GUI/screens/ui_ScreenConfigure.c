// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.3
// Project name: SquareLine_Project-GPS

#include "../ui.h"

void ui_ScreenConfigure_screen_init(void)
{
    ui_ScreenConfigure = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScreenConfigure, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label1 = lv_label_create(ui_ScreenConfigure);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, 5);
    lv_obj_set_y(ui_Label1, 5);
    lv_obj_set_align(ui_Label1, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(ui_Label1, "Configuration");

    ui_btnStartStop = lv_btn_create(ui_ScreenConfigure);
    lv_obj_set_width(ui_btnStartStop, 70);
    lv_obj_set_height(ui_btnStartStop, 70);
    lv_obj_set_x(ui_btnStartStop, 117);
    lv_obj_set_y(ui_btnStartStop, -77);
    lv_obj_set_align(ui_btnStartStop, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_btnStartStop, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_btnStartStop, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_btnStartStop, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_btnStartStop, lv_color_hex(0x120DB1), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_btnStartStop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_btnStartStop, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label7 = lv_label_create(ui_btnStartStop);
    lv_obj_set_width(ui_Label7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label7, -1);
    lv_obj_set_y(ui_Label7, 0);
    lv_obj_set_align(ui_Label7, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label7, "START");

    ui_PanelConfigureSats = lv_obj_create(ui_ScreenConfigure);
    lv_obj_set_width(ui_PanelConfigureSats, 170);
    lv_obj_set_height(ui_PanelConfigureSats, 210);
    lv_obj_set_x(ui_PanelConfigureSats, -69);
    lv_obj_set_y(ui_PanelConfigureSats, 10);
    lv_obj_set_align(ui_PanelConfigureSats, LV_ALIGN_CENTER);

    ui_TextAreaPRN1 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaPRN1, 55);
    lv_obj_set_height(ui_TextAreaPRN1, LV_SIZE_CONTENT);    /// 30
    lv_obj_set_x(ui_TextAreaPRN1, 52);
    lv_obj_set_y(ui_TextAreaPRN1, -9);
    lv_textarea_set_max_length(ui_TextAreaPRN1, 2);
    lv_textarea_set_text(ui_TextAreaPRN1, "1");
    lv_textarea_set_placeholder_text(ui_TextAreaPRN1, "PRN Num");
    lv_textarea_set_one_line(ui_TextAreaPRN1, true);



    ui_TextAreaPRN2 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaPRN2, 55);
    lv_obj_set_height(ui_TextAreaPRN2, LV_SIZE_CONTENT);    /// 30
    lv_obj_set_x(ui_TextAreaPRN2, 52);
    lv_obj_set_y(ui_TextAreaPRN2, 80);
    lv_textarea_set_max_length(ui_TextAreaPRN2, 2);
    lv_textarea_set_text(ui_TextAreaPRN2, "2");
    lv_textarea_set_placeholder_text(ui_TextAreaPRN2, "PRN Num");
    lv_textarea_set_one_line(ui_TextAreaPRN2, true);



    ui_TextAreaPRN3 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaPRN3, 71);
    lv_obj_set_height(ui_TextAreaPRN3, LV_SIZE_CONTENT);    /// 30
    lv_obj_set_x(ui_TextAreaPRN3, 52);
    lv_obj_set_y(ui_TextAreaPRN3, 172);
    lv_textarea_set_max_length(ui_TextAreaPRN3, 2);
    lv_textarea_set_text(ui_TextAreaPRN3, "3");
    lv_textarea_set_placeholder_text(ui_TextAreaPRN3, "PRN Num");
    lv_textarea_set_one_line(ui_TextAreaPRN3, true);



    ui_TextAreaPRN4 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaPRN4, 71);
    lv_obj_set_height(ui_TextAreaPRN4, LV_SIZE_CONTENT);    /// 30
    lv_obj_set_x(ui_TextAreaPRN4, 52);
    lv_obj_set_y(ui_TextAreaPRN4, 269);
    lv_textarea_set_max_length(ui_TextAreaPRN4, 2);
    lv_textarea_set_text(ui_TextAreaPRN4, "4");
    lv_textarea_set_placeholder_text(ui_TextAreaPRN4, "PRN Num");
    lv_textarea_set_one_line(ui_TextAreaPRN4, true);



    ui_Label5 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label5, 0);
    lv_obj_set_y(ui_Label5, 2);
    lv_label_set_text(ui_Label5, "PRN1:");

    ui_TextAreaFreq1 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaFreq1, 79);
    lv_obj_set_height(ui_TextAreaFreq1, LV_SIZE_CONTENT);    /// 70
    lv_obj_set_x(ui_TextAreaFreq1, 52);
    lv_obj_set_y(ui_TextAreaFreq1, 32);
    lv_textarea_set_text(ui_TextAreaFreq1, "0");
    lv_textarea_set_placeholder_text(ui_TextAreaFreq1, "Freq. Hz");
    lv_textarea_set_one_line(ui_TextAreaFreq1, true);



    ui_Label8 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label8, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label8, 0);
    lv_obj_set_y(ui_Label8, 43);
    lv_label_set_text(ui_Label8, "Freq1:");
    lv_obj_set_style_text_color(ui_Label8, lv_color_hex(0x1203F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label9 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label9, 2);
    lv_obj_set_y(ui_Label9, 92);
    lv_label_set_text(ui_Label9, "PRN2:");

    ui_TextAreaFreq2 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaFreq2, 79);
    lv_obj_set_height(ui_TextAreaFreq2, LV_SIZE_CONTENT);    /// 70
    lv_obj_set_x(ui_TextAreaFreq2, 52);
    lv_obj_set_y(ui_TextAreaFreq2, 121);
    lv_textarea_set_text(ui_TextAreaFreq2, "0");
    lv_textarea_set_placeholder_text(ui_TextAreaFreq2, "Freq. Hz");
    lv_textarea_set_one_line(ui_TextAreaFreq2, true);



    ui_Label6 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label6, 1);
    lv_obj_set_y(ui_Label6, 132);
    lv_label_set_text(ui_Label6, "Freq2:");
    lv_obj_set_style_text_color(ui_Label6, lv_color_hex(0x1203F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label10 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label10, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label10, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label10, 1);
    lv_obj_set_y(ui_Label10, 184);
    lv_label_set_text(ui_Label10, "PRN3:");

    ui_TextAreaFreq3 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaFreq3, 79);
    lv_obj_set_height(ui_TextAreaFreq3, LV_SIZE_CONTENT);    /// 70
    lv_obj_set_x(ui_TextAreaFreq3, 52);
    lv_obj_set_y(ui_TextAreaFreq3, 215);
    lv_textarea_set_text(ui_TextAreaFreq3, "0");
    lv_textarea_set_placeholder_text(ui_TextAreaFreq3, "Freq. Hz");
    lv_textarea_set_one_line(ui_TextAreaFreq3, true);



    ui_Label11 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label11, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label11, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label11, -2);
    lv_obj_set_y(ui_Label11, 226);
    lv_label_set_text(ui_Label11, "Freq3:");
    lv_obj_set_style_text_color(ui_Label11, lv_color_hex(0x1203F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label11, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label12 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label12, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label12, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label12, -2);
    lv_obj_set_y(ui_Label12, 281);
    lv_label_set_text(ui_Label12, "PRN4:");

    ui_TextAreaFreq4 = lv_textarea_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_TextAreaFreq4, 79);
    lv_obj_set_height(ui_TextAreaFreq4, LV_SIZE_CONTENT);    /// 70
    lv_obj_set_x(ui_TextAreaFreq4, 52);
    lv_obj_set_y(ui_TextAreaFreq4, 309);
    lv_textarea_set_text(ui_TextAreaFreq4, "0");
    lv_textarea_set_placeholder_text(ui_TextAreaFreq4, "Freq. Hz");
    lv_textarea_set_one_line(ui_TextAreaFreq4, true);



    ui_Label13 = lv_label_create(ui_PanelConfigureSats);
    lv_obj_set_width(ui_Label13, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label13, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label13, -2);
    lv_obj_set_y(ui_Label13, 319);
    lv_label_set_text(ui_Label13, "Freq4:");
    lv_obj_set_style_text_color(ui_Label13, lv_color_hex(0x1203F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label13, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_btnStop = lv_btn_create(ui_ScreenConfigure);
    lv_obj_set_width(ui_btnStop, 106);
    lv_obj_set_height(ui_btnStop, 38);
    lv_obj_set_x(ui_btnStop, 96);
    lv_obj_set_y(ui_btnStop, 40);
    lv_obj_set_align(ui_btnStop, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_btnStop, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_btnStop, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_btnStop, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_btnStop, lv_color_hex(0xD4DF05), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_btnStop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_btnStop, lv_color_hex(0x3D3D3D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_btnStop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label14 = lv_label_create(ui_btnStop);
    lv_obj_set_width(ui_Label14, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label14, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label14, -1);
    lv_obj_set_y(ui_Label14, 0);
    lv_obj_set_align(ui_Label14, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label14, "STOP");

    ui_btnRestartAcq = lv_btn_create(ui_ScreenConfigure);
    lv_obj_set_width(ui_btnRestartAcq, 106);
    lv_obj_set_height(ui_btnRestartAcq, 38);
    lv_obj_set_x(ui_btnRestartAcq, 96);
    lv_obj_set_y(ui_btnRestartAcq, 92);
    lv_obj_set_align(ui_btnRestartAcq, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_btnRestartAcq, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_btnRestartAcq, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label15 = lv_label_create(ui_btnRestartAcq);
    lv_obj_set_width(ui_Label15, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label15, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label15, -1);
    lv_obj_set_y(ui_Label15, 0);
    lv_obj_set_align(ui_Label15, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label15, "CODE SRCH");

    ui_Keyboard1 = lv_keyboard_create(ui_ScreenConfigure);
    lv_keyboard_set_mode(ui_Keyboard1, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_width(ui_Keyboard1, 130);
    lv_obj_set_height(ui_Keyboard1, 232);
    lv_obj_set_x(ui_Keyboard1, 92);
    lv_obj_set_y(ui_Keyboard1, 0);
    lv_obj_set_align(ui_Keyboard1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN);     /// Flags

    lv_obj_add_event_cb(ui_btnStartStop, ui_event_btnStartStop, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaPRN1, ui_event_TextAreaPRN1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaPRN2, ui_event_TextAreaPRN2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaPRN3, ui_event_TextAreaPRN3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaPRN4, ui_event_TextAreaPRN4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaFreq1, ui_event_TextAreaFreq1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaFreq2, ui_event_TextAreaFreq2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaFreq3, ui_event_TextAreaFreq3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextAreaFreq4, ui_event_TextAreaFreq4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_btnStop, ui_event_btnStop, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_btnRestartAcq, ui_event_btnRestartAcq, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Keyboard1, ui_event_Keyboard1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ScreenConfigure, ui_event_ScreenConfigure, LV_EVENT_ALL, NULL);

}
