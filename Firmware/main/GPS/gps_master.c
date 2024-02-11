//GPS master is a code that controllig acquisition in all channels, start tracking
//and controllling sending RTCM and position calculation

#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "signal_capture.h"
#include "gps_master.h"
#include "config.h"
#include "acquisition.h"
#include "rtk_common.h"
#include "math.h"
#include "time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lvgl_gui.h"


#if (ENABLE_CALC_POSITION)
  #include "solving.h"
  extern sol_t gps_sol;
  extern double final_pos[3];//geodetic position {lat,lon,h} (deg,m)
#endif


#define GPS_OFFSET_TIME_MS    (68.802)
#define CLIGHT		            299792458.0      /* speed of light (m/s) */
#define CLIGHT_NORM	          (299792458.0 / PRN_SPEED_HZ)
#define SUBFRAME_DURATION_MS	(6000)

#define GPS_RTCM_SEND_PERIOD_MS 200

#define GPS_CALC_POS_PERIOD_MS  200 //Code filter can make calculations slower


//******************************************************************
obsd_t obsd[GPS_SAT_CNT];

//Flag that at least one sat. need acquisition
uint8_t gps_common_need_acq = 1;

/// Need position solving
uint8_t gps_common_need_solve = 0;

uint8_t gps_start_flag = 1;

void gps_master_nav_handling(gps_ch_t* channels);
void gps_master_transmit_obs(gps_ch_t* channels);
void gps_master_calculate_pos_handling(gps_ch_t* channels);

void gps_master_final_pseudorange_calc(
  gps_ch_t* channels, uint32_t curr_tick_time, 
  uint32_t ref_time_diff_ms, uint32_t ref_time_ms, uint8_t ref_idx);

#if (ENABLE_CODE_FILTER)
uint16_t gps_master_filter_code_phase(
  gps_ch_t* channels, uint32_t curr_tick_time);
void gps_master_code_phase_filter_reset(
  gps_ch_t* channels, uint32_t curr_tick_time);
#endif

//****************************************************
//****************************************************

void gps_master_handling(gps_ch_t* channels, uint8_t index)
{
  //gps_ch_t* curr_ch = channels;
  
  if (gps_start_flag)
  {
    gps_start_flag = 0;
    acquisition_start_channel(&channels[0]);
  }
  
  gps_common_need_acq = 0;
  uint8_t need_f_search = 0;//freq. search
  uint8_t code_search3_cnt = 0;
  
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].acq_data.state != GPS_ACQ_DONE)
      gps_common_need_acq = 1;
    if (channels[i].acq_data.state < GPS_ACQ_FREQ_SEARCH_DONE)
      need_f_search = 1;
    if (channels[i].acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH2_DONE)
      code_search3_cnt++;
  }
  
  //Starting code search - one by one
  if (gps_common_need_acq == 1)
  {
    for (uint8_t i = 0; i < (GPS_SAT_CNT - 1); i++)
    {
      //If next channel need freq. search
      if ((channels[i].acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE) &&
          (channels[i + 1].acq_data.state == GPS_ACQ_NEED_FREQ_SEARCH))
      {
        //Start freq. search at the new channel
        acquisition_start_channel(&channels[i+1]);
        return;
      }
    }
  }
  
  //Start acq. code search for all channels
  if ((need_f_search == 0) && (gps_common_need_acq == 1))
  {
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      if (channels[i].acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE)
        acquisition_start_code_search_channel(&channels[i]);
      
      // Start simultaneously code search 3
      if (code_search3_cnt == GPS_SAT_CNT)
      {
        acquisition_start_code_search3_channel(&channels[i]);
      }
    }
  }
  
  if (gps_common_need_acq == 0)
  {
    //acq is done at all sats. -> start tracking!
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      if (channels[i].tracking_data.state == GPS_TRACKNG_IDLE)
        channels[i].tracking_data.state = GPS_NEED_PRE_TRACK; //start tracking for this sat.
    }
  }

  if (gps_common_need_acq)
  {
    lvgl_store_gps_state(channels);
  }
  else
  {
    //Tracking running
    if (index == 0xFF) // dummy tracking
    {
      gps_master_nav_handling(channels);

      lvgl_store_gps_state(channels);
    }
  }
}


//Called every 17ms, when there is no tracking working
void gps_master_nav_handling(gps_ch_t* channels)
{
  //Set first subframe detection time for all channels
  uint8_t has_subframe_time_cnt = 0;
  uint8_t first_time_not_set_cnt = 0;
  
  uint8_t ref_idx = 0;
  uint32_t min_subframe_time = 0xFFFFFFFF;
  uint32_t max_subframe_time = 0;
  
  uint16_t min_subframe_cnt = 0xFFFF;
  uint16_t max_subframe_cnt = 0;
  
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].nav_data.last_subframe_time != 0)
      has_subframe_time_cnt++;
    
    if (channels[i].nav_data.first_subframe_time == 0)
      first_time_not_set_cnt++;
    
    if (channels[i].nav_data.last_subframe_time < min_subframe_time)
    {
      min_subframe_time = channels[i].nav_data.last_subframe_time;
      ref_idx = i;//Reference sat. is a sat. with min. time - closest to receiver
    }
    
    if (channels[i].nav_data.last_subframe_time > max_subframe_time)
      max_subframe_time = channels[i].nav_data.last_subframe_time;

    if (channels[i].nav_data.subframe_cnt < min_subframe_cnt)
      min_subframe_cnt = channels[i].nav_data.subframe_cnt;
    
    if (channels[i].nav_data.subframe_cnt > max_subframe_cnt)
      max_subframe_cnt = channels[i].nav_data.subframe_cnt;

    if (channels[i].tracking_data.snr_value < OBS_SNR_THRESHOLD_DB)
      return;
  }
  
  if (min_subframe_time == 0)
    return;
  
  uint32_t diff_ms = max_subframe_time - min_subframe_time;
  if (diff_ms > 100)//wait untill all subframes of this epoch get received (they will have similiar times)
    return;
  
  if ((has_subframe_time_cnt == GPS_SAT_CNT) && 
      (first_time_not_set_cnt == GPS_SAT_CNT))
  {
    //This works once!
    //Lock "first_subframe_time"
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      channels[i].nav_data.first_subframe_time = 
        channels[i].nav_data.last_subframe_time;
      channels[i].nav_data.subframe_cnt = 0;
    }
  }
  
  //************************************************
  //Pseudorange calculation
  
  if (channels[0].nav_data.first_subframe_time == 0)
    return;
  
  uint32_t ref_time_ms = channels[ref_idx].nav_data.first_subframe_time + 
    max_subframe_cnt * SUBFRAME_DURATION_MS;
  
  //Detecting code phase swap - may happen more longer after new subframe event
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].tracking_data.code_phase_swap_flag && 
        channels[i].nav_data.new_subframe_flag)
    {
      channels[i].nav_data.new_subframe_flag = 0;
      channels[i].tracking_data.code_phase_swap_flag = 0;
    }
    
    float diff_f = fabs(channels[i].tracking_data.old_code_phase_fine - 
                        channels[i].tracking_data.code_phase_fine);
    
    if (diff_f > ((float)PRN_LENGTH * 16.0f / 2.0f))//half of range
    {
      //code phase swap detected
      channels[i].tracking_data.code_phase_swap_flag = 1;
    }
    
    channels[i].tracking_data.old_code_phase_fine = channels[i].tracking_data.code_phase_fine;
  }
  
  uint32_t curr_tick_time = signal_capture_get_packet_cnt();//system time, ms
  
  //Time from last subframe of REF sat.
  int32_t ref_time_diff_ms = (int32_t)curr_tick_time - 
    (int32_t)channels[ref_idx].nav_data.last_subframe_time;
  if ((ref_time_diff_ms < 0))
    ref_time_diff_ms = ref_time_diff_ms % SUBFRAME_DURATION_MS;
  
  uint8_t need_calc_pseuodranges = 1;
#if (ENABLE_CODE_FILTER)
  uint16_t filter_duration_ms = gps_master_filter_code_phase(channels, curr_tick_time);
  if (filter_duration_ms < 1)
    need_calc_pseuodranges = 0;//can't just return - we need to go below
  
  //Filter time averaging
  ref_time_diff_ms = ref_time_diff_ms - filter_duration_ms / 2; 
#endif
  
  if (need_calc_pseuodranges)
  {
    gps_master_final_pseudorange_calc(
      channels, curr_tick_time, 
      ref_time_diff_ms, ref_time_ms, ref_idx);
    
#if (ENABLE_CODE_FILTER)
    gps_master_code_phase_filter_reset(channels, curr_tick_time);
#endif

#if (ENABLE_CALC_POSITION)
    gps_master_calculate_pos_handling(channels);
#endif
  }

  #if (ENABLE_RTCM_SEND)
    //gps_master_transmit_obs(channels);
    #error "NOT IMPLEMENTED FOR THIS MCU!"
  #endif
}

// Final pseudoranges and observaltions calculation
// channels - receiver channels
// curr_tick_time - current receiving time in ms
// ref_time_diff_ms - 
// ref_time_ms - 
// ref_idx - index of reference satellite in ms
void gps_master_final_pseudorange_calc(
  gps_ch_t* channels, uint32_t curr_tick_time, 
  uint32_t ref_time_diff_ms, uint32_t ref_time_ms, uint8_t ref_idx)
{
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    int32_t diff_prn_ms = channels[i].nav_data.last_subframe_time - ref_time_ms;
    
#if (ENABLE_CODE_FILTER)
    double ch_diff_time_ms = (double)diff_prn_ms + 
      channels[i].tracking_data.code_phase_fine_filt / ((double)PRN_LENGTH * 16.0f);
#else
    double ch_diff_time_ms = (double)diff_prn_ms + 
      channels[i].tracking_data.code_phase_fine / ((double)PRN_LENGTH * 16.0f);
#endif
    
    //Comepensating state when code phase swapped, but new subframe has not come yet
    if (channels[i].tracking_data.code_phase_swap_flag == 1)
    {
      double corr_ms = 1.0f;
      if (channels[i].tracking_data.if_freq_offset_hz < 0.0f)
        corr_ms = -1.0f;
      //Add correction
      ch_diff_time_ms = ch_diff_time_ms - corr_ms;
    }
    double pseudo_range_m = (GPS_OFFSET_TIME_MS + ch_diff_time_ms) * CLIGHT_NORM;
    channels[i].obs_data.pseudorange_m = pseudo_range_m;
    channels[i].obs_data.tow_s = channels[ref_idx].eph_data.tow_gpst + 
      ((float)(ref_time_diff_ms + i * TRACKING_CH_LENGTH) / PRN_SPEED_HZ); //to seconds
  }
}


#if (ENABLE_CODE_FILTER)
//Return 0 if not ready, or duration (time from last filtering in ms) if OK
uint16_t gps_master_filter_code_phase(
  gps_ch_t* channels, uint32_t curr_tick_time)
{
  //Count channels that have enough captured pointes
  uint8_t filter_ready_cnt = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].tracking_data.code_filt_cnt > CODE_FILTER_LENGTH)
      filter_ready_cnt++;
  }
  if (filter_ready_cnt < GPS_SAT_CNT)
    return 0;
  
  uint8_t code_swap_flag = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].tracking_data.code_phase_fine_filt < -0.5f) //code swap detected
      code_swap_flag++;
  }
  if (code_swap_flag)
  {
    gps_master_code_phase_filter_reset(channels, curr_tick_time);
    return 0;
  }
  
  //Expected that all channels have same time
  uint32_t filt_time_duration_ms = curr_tick_time - 
    channels[0].tracking_data.filt_start_time_ms;
  
  if (filt_time_duration_ms > 1000)
  {
    gps_master_code_phase_filter_reset(channels, curr_tick_time);
    return 0;
  }
    
  //Calculate average value
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    channels[i].tracking_data.code_phase_fine_filt =
      channels[i].tracking_data.code_phase_fine_filt / 
      channels[i].tracking_data.code_filt_cnt;
  }
  return filt_time_duration_ms;
}

void gps_master_code_phase_filter_reset(
  gps_ch_t* channels, uint32_t curr_tick_time)
{
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    channels[i].tracking_data.code_phase_fine_filt = 0.0f;
    channels[i].tracking_data.code_filt_cnt = 0;
    channels[i].tracking_data.filt_start_time_ms = curr_tick_time;
  }
}
#endif


#if (ENABLE_CALC_POSITION)
//Calculate receiver positon - handling, do not run solving here!
//Called from high priority task
void gps_master_calculate_pos_handling(gps_ch_t* channels)
{
  static uint32_t prev_calc_time_ms = 0;
  
  if (solving_is_busy() || gps_common_need_solve)
  {
    //Processing already runing solving
    return;
  }
  
  uint32_t curr_time_ms = signal_capture_get_packet_cnt();
  if ((curr_time_ms - prev_calc_time_ms) > GPS_CALC_POS_PERIOD_MS)
  {
    //Check that all sats have received ephemeris
    uint8_t  eph_ok_cnt = 0;
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      if ((channels[i].eph_data.received_mask_proc & 0x7) == 0x7)
        eph_ok_cnt++;
    }
    
    if (eph_ok_cnt == GPS_SAT_CNT)
    {
      //Copy data and set a flag for less priority task
      sdrobs2obsd(channels, GPS_SAT_CNT, obsd);
      gps_common_need_solve = 1;
      prev_calc_time_ms = curr_time_ms;
    }
  }
}

/// @brief Process position solving - can be long, called from less priority task
/// @param  
void gps_master_run_solving(void)
{
  //static uint32_t prev_time = 0;

  if (gps_common_need_solve == 0) //No request to solve
    return;

  //uint32_t start = signal_capture_get_packet_cnt();

  // Take 20-40ms at ESP32 at 240MHz (notice that this is low priority task!)
  gps_pos_solve_direct(obsd);

  //uint32_t diff_ms = signal_capture_get_packet_cnt() - start;

  gps_common_need_solve = 0;

  lvgl_store_new_position(&gps_sol, &final_pos[0]);
}
#endif //ENABLE_CALC_POSITION

uint8_t gps_master_need_freq_search(gps_ch_t* channels)
{
  uint8_t need_search = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels->acq_data.state < GPS_ACQ_FREQ_SEARCH_DONE)
      need_search = 1;
    channels++;
  }
  return need_search;
}

// All in mode > GPS_ACQ_CODE_PHASE_SEARCH2
uint8_t gps_master_is_code_search3(gps_ch_t* channels)
{
  uint8_t test_cnt = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels->acq_data.state > GPS_ACQ_CODE_PHASE_SEARCH2)
      test_cnt++;
    channels++;
  }
  return (test_cnt == GPS_SAT_CNT);
}

uint8_t gps_master_need_acq(void)
{
  return gps_common_need_acq;
}

uint8_t gps_master_need_solve(void)
{
  return gps_common_need_solve;
}


//Reset all channels to acquisition code search 
void gps_master_reset_to_aqc_start(gps_ch_t* channels)
{
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].acq_data.state < GPS_ACQ_FREQ_SEARCH_DONE)
      return;
  }
  
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].nav_data.word_cnt_test > 1)
    {
      //copy good value
      channels[i].acq_data.found_freq_offset_hz = 
        (int16_t)channels[i].tracking_data.if_freq_offset_hz;
    }
    channels[i].acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
    memset(&channels[i].tracking_data, 0, sizeof(gps_tracking_t));
    memset(&channels[i].nav_data, 0, sizeof(gps_nav_data_t)); 
  }
}


