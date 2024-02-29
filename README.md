# ESP32_SDR_GPS
Software GPS receiver for ESP32  
See article (in Russian): https://habr.com/ru/articles/789382/  
  
This is a demo project of ESP32 based SDR GPS receiver.  
No additional factory/commercial "black box" GPS receiver is needed here - just connect MAX2769 to the ESP32 MCU.  
All "raw" GPS signal processing is done at the ESP32.  
Results of the receiving are displayed at the LCD.   
<img src="https://github.com/iliasam/ESP32_SDR_GPS/blob/main/Images/Drawing_esp32b.png" width="800">  
  
**Video about this project:**  
https://youtu.be/1hBnaDsQgMc  
  
Configured for ESP32-2432S024C board (SPI ILI9341 + CST820 capacitive touchscreen) - CYD variant.  
-O2 optimization is used here  
RTOS tick period - 1ms  
LCD+Touch pinout is set by menuconfig  
LVGL 8.3.0 is a managed component, must be downloaded by ESP-IDF  
ESP-IDF 5.0 is used here  

User must enter PRN codes of 4 satellites in GUI before start.  
User can enter Doppler frequency offset to make acquisition much faster. "-" symbol is entered by holding "delete" GUI key.  
  
RF frontend pinout (See config.h):  
#define GPS_SPI_CLK_PIN     18 //MAX2769 - CLKOUT  
#define GPS_SPI_MOSI_PIN    23 //MAX2769 - I0  
#define GPS_SPI_CS_PIN      5  //To GND  
  
Part of the code is taken from these projects:
https://github.com/tomojitakasu/RTKLIB  
https://github.com/taroz/GNSS-SDRLIB  
https://github.com/VitorAlho/lvgl_esp32_drivers  

