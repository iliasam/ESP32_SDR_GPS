// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.3
// Project name: SquareLine_Project-GPS

#include "ui.h"

static lv_obj_tree_walk_res_t disable_object_cb(lv_obj_t * obj, void * user_data)
{
    lv_obj_add_state(obj, LV_STATE_DISABLED);
    return LV_OBJ_TREE_WALK_NEXT;
}

static lv_obj_tree_walk_res_t enable_object_cb(lv_obj_t * obj, void * user_data)
{
    lv_obj_clear_state(obj, LV_STATE_DISABLED);
    return LV_OBJ_TREE_WALK_NEXT;
}

//***************************

//Button is placed at ui_ScreenConfigure
void btnStartStopPressed(lv_event_t * e)
{
	//lv_obj_set_click(ui_btnStartStop, false);
	lv_obj_add_state(ui_btnStartStop, LV_STATE_DISABLED);

	lv_obj_tree_walk(ui_PanelConfigureSats, disable_object_cb, NULL); //lock panel elements
	lv_obj_clear_state(ui_PanelConfigureSats, LV_STATE_DISABLED);

	lv_obj_clear_state(ui_btnStop, LV_STATE_DISABLED);
    lv_obj_clear_state(ui_btnRestartAcq, LV_STATE_DISABLED);

	lv_indev_wait_release(lv_indev_get_act());
  	_ui_screen_change(&ui_ScreenState, LV_SCR_LOAD_ANIM_MOVE_LEFT, 0, 0, &ui_ScreenState_screen_init);
}

void funcShowKeyboard(lv_event_t * e)
{
	_ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
}



void btnStopClick(lv_event_t * e)
{
	lv_obj_add_state(ui_btnStop, LV_STATE_DISABLED);
    lv_obj_add_state(ui_btnRestartAcq, LV_STATE_DISABLED);
	//Set active
	lv_obj_clear_state(ui_btnStartStop, LV_STATE_DISABLED);

	lv_obj_tree_walk(ui_PanelConfigureSats, enable_object_cb, NULL);//activate panel elements
}

void btnCodeSearchClick(lv_event_t * e)
{
	lv_indev_wait_release(lv_indev_get_act());
  	_ui_screen_change(&ui_ScreenState, LV_SCR_LOAD_ANIM_MOVE_LEFT, 0, 0, &ui_ScreenState_screen_init);
}

void func_IQSatChanged(lv_event_t * e)
{
	// Your code here
}