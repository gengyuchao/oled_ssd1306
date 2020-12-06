
#include "spi_oled_driver.h"


/* 需要修改说明
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

spi_device_handle_t oled_spi;


//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);

}



static const char *TAG = "spi_oled_driver";

/*------OLED Base function------*/

static uint8_t oled_dc_level = 0;

esp_err_t oled_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
    return ESP_OK;
}

esp_err_t oled_set_dc(uint8_t dc)
{
    oled_dc_level = dc;
    return ESP_OK;
}

// Write an 8-bit cmd
esp_err_t oled_write_cmd(uint8_t cmd)
{    
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(oled_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    return ESP_OK;
}

// Write an 8-bit data
esp_err_t oled_write_byte(uint8_t data)
{

    oled_write_data(&data,1);

    return ESP_OK;
}

// Write many-bit data
esp_err_t oled_write_data(uint8_t *data,int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(oled_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    return ESP_OK;
}

esp_err_t oled_rst()
{
    gpio_set_level(OLED_RST_GPIO, 0);
    oled_delay_ms(200);
    gpio_set_level(OLED_RST_GPIO, 1);
    oled_delay_ms(100);
    return ESP_OK;
}

esp_err_t oled_gpio_init()
{
    esp_err_t ret;
    // spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=128*64+8
    };
    spi_device_interface_config_t devcfg={
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
#else
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
#endif
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(OLED_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(OLED_HOST, &devcfg, &oled_spi);
    ESP_ERROR_CHECK(ret);

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);


    ESP_LOGI(TAG, "\nOLED_DC_GPIO:%d  OLED_RST_GPIO:%d",OLED_DC_GPIO,OLED_RST_GPIO);

  oled_rst(); // Reset OLED
  return ESP_OK;
}

