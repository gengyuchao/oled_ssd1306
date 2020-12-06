#ifndef __SPI_OLED_DRIVER_H__
#define __SPI_OLED_DRIVER_H__


#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "string.h"



// D5 -> CLK(OLED_D0)           GPIO14         
// D7 -> MOSI(OLED_D1) (DOUT)   GPIO13 
// D0 -> RES                    GPIO16
// D2 -> DC                     GPIO4
// D8 -> CS                     GPIO15

//ESP32
#define OLED_HOST    HSPI_HOST
#define DMA_CHAN    2

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5

//8266
#define OLED_DC_GPIO     PIN_NUM_DC
#define OLED_RST_GPIO    PIN_NUM_RST
// #define OLED_CS_GPIO     15
// #define OLED_PIN_SEL  (1ULL<<OLED_DC_GPIO) | (1ULL<<OLED_RST_GPIO) | (1ULL<<OLED_CS_GPIO)

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

#define _max(a,b) ((a) > (b) ? (a) : (b))
#define _min(a,b) ((a) < (b) ? (a) : (b))

#ifdef DEBUG_OLEDDISPLAY
    #define DEBUG_OLEDDISPLAY printf
#else
    #define DEBUG_OLEDDISPLAY(...)
#endif


// Display commands
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2


 esp_err_t oled_delay_ms(uint32_t time);

 esp_err_t oled_set_dc(uint8_t dc);

// Write an 8-bit cmd
 esp_err_t oled_write_cmd(uint8_t data);

// Write an 8-bit data
 esp_err_t oled_write_byte(uint8_t data);

 esp_err_t oled_write_data(uint8_t *data,int length);

 esp_err_t oled_rst();

 esp_err_t oled_init();

 //esp_err_t oled_Fully_fill(uint8_t data);

 void IRAM_ATTR spi_event_callback(int event, void *arg);

 esp_err_t oled_gpio_init();


#endif