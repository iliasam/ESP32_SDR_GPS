#ifndef _GUI_H
#define _GUI_H

#include "gps_misc.h"

/// @brief Used too store data from GPS channels for displaying
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
    uint8_t	                prn; //Sat PRN code
} gps_gui_ch_t;



void gui_task(void *pvParameter);
void lvgl_store_gps_state(gps_ch_t *channels);

#endif
