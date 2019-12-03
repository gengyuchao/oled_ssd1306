/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */

#ifndef SSD1306Spi_h
#define SSD1306Spi_h

#include "OLEDDisplay.hpp"

extern "C"{
  #include "user_oled_config.h"

  #include "driver/spi_master.h"
  #include "driver/gpio.h"

  //ESP32 - special
  #define OLED_HOST    HSPI_HOST
  #define DMA_CHAN    2

  #define PIN_NUM_MISO 25
  #define PIN_NUM_MOSI 23
  #define PIN_NUM_CLK  19
  #define PIN_NUM_CS   22

  #define PIN_NUM_DC   21
  #define PIN_NUM_RST  18
  #define PIN_NUM_BCKL 5

  #define _max(a,b) ((a) > (b) ? (a) : (b))
  #define _min(a,b) ((a) < (b) ? (a) : (b))

}

#define OLEDDISPLAY_DOUBLE_BUFFER

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);

}

class SSD1306Spi : public OLEDDisplay {
  private:
      uint8_t             _rst;
      uint8_t             _dc;
      uint8_t             _cs;
      spi_device_handle_t oled_spi;

      // Write an 8-bit data
      esp_err_t oled_write_byte(uint8_t data)
      {
          return oled_write_data(&data,1);;
      }

      // Write many-bit data
      esp_err_t oled_write_data(uint8_t *data,int len)
      {
          esp_err_t ret;
          spi_transaction_t t;
          if (len==0) return -1;             //no need to send anything
          memset(&t, 0, sizeof(t));       //Zero out the transaction
          t.length=len*8;                 //Len is in bytes, transaction length is in bits.
          t.tx_buffer=data;               //Data
          t.user=(void*)1;                //D/C needs to be set to 1
          ret=spi_device_polling_transmit(oled_spi, &t);  //Transmit!
          assert(ret==ESP_OK);            //Should have had no issues.

          return ESP_OK;
      }

  public:
    SSD1306Spi(uint8_t _rst, uint8_t _dc, uint8_t _cs, OLEDDISPLAY_GEOMETRY g = GEOMETRY_128_64) {
        setGeometry(g);

      this->_rst = _rst;
      this->_dc  = _dc;
      this->_cs  = _cs;
    }

    bool connect(){
      esp_err_t ret;
      // spi_device_handle_t spi;
      spi_bus_config_t buscfg;
      memset(&buscfg,0,sizeof(buscfg));

      buscfg.mosi_io_num=PIN_NUM_MOSI;
      buscfg.miso_io_num=PIN_NUM_MISO;
      buscfg.sclk_io_num=PIN_NUM_CLK;
      buscfg.quadwp_io_num=-1;
      buscfg.quadhd_io_num=-1;
      buscfg.max_transfer_sz=128*64+8;

      spi_device_interface_config_t devcfg;
      memset(&devcfg,0,sizeof(devcfg));
      
  #ifdef CONFIG_LCD_OVERCLOCK
      devcfg.clock_speed_hz=26*1000*1000;           //Clock out at 26 MHz
  #else
      devcfg.clock_speed_hz=10*1000*1000;           //Clock out at 10 MHz
  #endif
      devcfg.mode=0;                                //SPI mode 0
      devcfg.spics_io_num=_cs;               //CS pin
      devcfg.queue_size=7;                          //We want to be able to queue 7 transactions at a time
      devcfg.pre_cb=lcd_spi_pre_transfer_callback;  //Specify pre-transfer callback to handle D/C line
     
      //Initialize the SPI bus
      ret=spi_bus_initialize(OLED_HOST, &buscfg, DMA_CHAN);
      ESP_ERROR_CHECK(ret);
      //Attach the LCD to the SPI bus
      ret=spi_bus_add_device(OLED_HOST, &devcfg, &oled_spi);
      ESP_ERROR_CHECK(ret);

      //Initialize non-SPI GPIOs
      gpio_set_direction(_dc, GPIO_MODE_OUTPUT);
      gpio_set_direction(_rst, GPIO_MODE_OUTPUT);


      // Pulse Reset low for 10ms
      gpio_set_level(_rst, 1);
      delay(1);
      gpio_set_level(_rst, 0);
      delay(10);
      gpio_set_level(_rst, 1);

      return true;
      
    }

    void display(void) {
    #ifdef OLEDDISPLAY_DOUBLE_BUFFER
       uint8_t minBoundY = UINT8_MAX;
       uint8_t maxBoundY = 0;

       uint8_t minBoundX = UINT8_MAX;
       uint8_t maxBoundX = 0;

       uint8_t x, y;

       // Calculate the Y bounding box of changes
       // and copy buffer[pos] to buffer_back[pos];
       for (y = 0; y < (displayHeight / 8); y++) {
         for (x = 0; x < displayWidth; x++) {
          uint16_t pos = x + y * displayWidth;
          if (buffer[pos] != buffer_back[pos]) {
            minBoundY = _min(minBoundY, y);
            maxBoundY = _max(maxBoundY, y);
            minBoundX = _min(minBoundX, x);
            maxBoundX = _max(maxBoundX, x);
          }
          buffer_back[pos] = buffer[pos];
        }
        yield();
       }

       // If the minBoundY wasn't updated
       // we can savely assume that buffer_back[pos] == buffer[pos]
       // holdes true for all values of pos
       if (minBoundY == UINT8_MAX) return;

       sendCommand(COLUMNADDR);
       sendCommand(minBoundX);
       sendCommand(maxBoundX);

       sendCommand(PAGEADDR);
       sendCommand(minBoundY);
       sendCommand(maxBoundY);

       for (y = minBoundY; y <= maxBoundY; y++) {
         //for (x = minBoundX; x <= maxBoundX; x++) {
           oled_write_data(&buffer[minBoundX + y * displayWidth],maxBoundX-minBoundX+1);
         //}
         yield();
       }

      #else
        // No double buffering
        sendCommand(COLUMNADDR);
        sendCommand(0x0);
        sendCommand(0x7F);

        sendCommand(PAGEADDR);
        sendCommand(0x0);

        if (geometry == GEOMETRY_128_64) {
          sendCommand(0x7);
        } else if (geometry == GEOMETRY_128_32) {
          sendCommand(0x3);
        }

        oled_write_data(buffer,displayBufferSize);

    
      #endif
    }

  private:
	int getBufferOffset(void) {
		return 0;
	}
    inline void sendCommand(uint8_t com) __attribute__((always_inline)){
      esp_err_t ret;
      spi_transaction_t t;
      memset(&t, 0, sizeof(t));       //Zero out the transaction
      t.length=8;                     //Command is 8 bits
      t.tx_buffer=&com;               //The data is the cmd itself
      t.user=(void*)0;                //D/C needs to be set to 0
      ret=spi_device_polling_transmit(oled_spi, &t);  //Transmit!
      assert(ret==ESP_OK);            //Should have had no issues.
    }
};

#endif
