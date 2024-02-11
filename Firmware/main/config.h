#ifndef _CONFIG_H
#define _CONFIG_H

#define LED_R_PIN        4
#define LED_G_PIN        16
#define LED_B_PIN        17

#define GPS_SPI_NAME        SPI3_HOST //SPI3
#define GPS_SPI_CLK_PIN     18
#define GPS_SPI_MOSI_PIN    23
#define GPS_SPI_CS_PIN      5

//*****************************************************

#define IF_FREQ_HZ              (int)(4092000)
#define SPI_BAUDRATE_HZ         (int)(16368000)
#define PRN_SPEED_HZ            1000 //1ms period
#define BITS_IN_PRN             (SPI_BAUDRATE_HZ / PRN_SPEED_HZ) //16Kbit
#define PRN_SPI_WORDS_CNT       (BITS_IN_PRN / 16) //1023 16bit words
#define PRN_LENGTH              1023 //in chips

// Increase buffer size 1023+1=1024 to make space for shifting data
// So now it is possible to work 
// with buffers as (GPS_DATA_WORDS_CNT/2=512) 32-bit words
#define GPS_DATA_WORDS_CNT      (PRN_SPI_WORDS_CNT + 1)

//******************************************************


#define ENABLE_RTCM_SEND        0

//Calculate receiver position by observations
#define ENABLE_CALC_POSITION   1

// Accurate code phase averaging
#define ENABLE_CODE_FILTER      1
//Number of measurements, speed is 25 measurements in one channel per 100ms
#define CODE_FILTER_LENGTH      100

//Draw IQ plot
#define ENABLE_IQ_PLOT          1
#define IQ_PLOT_POINTS_CNT      50
#define IQ_PLOT_MAX             100 //Axis Max
#define IQ_PLOT_SIGNAL_MAX      1000

#define ACQ_SEARCH_FREQ_HZ      (7000) //Search zone is x2
#define ACQ_SEARCH_STEP_HZ      (500)
/// Number of freq steps
#define ACQ_COUNT       (ACQ_SEARCH_FREQ_HZ * 2 / ACQ_SEARCH_STEP_HZ + 1)

/// 1 step is 0.5 of chip
#define ACQ_PHASE1_HIST_STEP	(64)
#define ACQ_PHASE1_HIST_SIZE	((PRN_LENGTH + 1) * 2 / ACQ_PHASE1_HIST_STEP) //32

#define PRE_TRACK_POINTS_MAX_CNT        30

//#define IF_NCO_STEP_HZ        ((float)SPI_BAUDRATE_HZ / (float)(1 << 32)) //NCO accumulator is 32bit
#define IF_NCO_STEP_HZ	        (0.003810972f)

//Channel SNR must be bigger that this value for producing observations
#define OBS_SNR_THRESHOLD_DB    (3.0f)

// Number of analysed PRN periods in one channal
#define TRACKING_CH_LENGTH      4

//Number of used satellites
#define GPS_SAT_CNT	            4

#define TRACKING_DLL1_C1        (1.0f)
#define TRACKING_DLL1_C2        (300.0f)

#define TRACKING_PLL1_C1        (4.0f)
#define TRACKING_PLL1_C2        (3000.0f)

#define TRACKING_PLL2_C1        (8.0f)
#define TRACKING_PLL2_C2        (5000.0f)

#define TRACKING_FLL1_C1        (200.0f)
#define TRACKING_FLL1_C2        (2000.0f)

#define GPS_BUILD_WEEK          2290 //NOV 2023

#endif
