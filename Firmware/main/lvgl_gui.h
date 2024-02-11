#ifndef _GUI_H
#define _GUI_H

#include <time.h>
#include "gps_misc.h"

#if (ENABLE_CALC_POSITION)
  #include "solving.h"
#endif

/// @brief Used to store data from GPS channels for displaying
typedef struct
{
    gps_acq_state_t         acq_state;
    int16_t                 acq_found_freq_offset_hz;
    uint8_t                 acq_freq_index;
    gps_tracking_state_t	track_state;
    float                   track_code_phase_fine;
    float                   track_snr;
    int16_t                 track_if_freq_offset_hz;
    uint16_t                nav_word_cnt;
    uint8_t                 nav_pol_found;
    uint16_t                nav_subframe_cnt;
    time_t                  eph_time;


    uint8_t	                prn; //Sat PRN code
} gps_gui_ch_t;



void gui_task(void *pvParameter);

void lvgl_store_gps_state(gps_ch_t *channels);
void lvgl_store_new_position(sol_t *gps_sol_p, double *position);

void lvgl_update_configure_controls(gps_ch_t *channels);

#endif
