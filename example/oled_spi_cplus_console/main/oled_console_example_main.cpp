/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"

#include "SSD1306I2C.hpp"

SSD1306I2C display(0x3c,GPIO_NUM_25,GPIO_NUM_26);// OLED_Address SCL SDA 


// #include "SSD1306Spi.hpp"
    
// SSD1306Spi display(PIN_NUM_RST,PIN_NUM_DC,PIN_NUM_CS);


void task_oled_console(void *);
void task_oled_console_history(void *);


static void spp_uart_init();

extern "C" void app_main(void)       
{
    esp_err_t ret;

    ets_printf("start\n");
    
    xTaskCreate(&task_oled_console_history, "task_oled_console_history", 4096, NULL, 5, NULL);
    vTaskDelay(20 / portTICK_RATE_MS);
    spp_uart_init();

    ets_printf("Hello world\n");

}


char Log_buffer[10][90];
uint8_t point_x=0,point_y=0;
uint8_t oled_font_size = 10;


#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_attr.h"

int oled_printf(const char* format, ...)
{
    char buffer[50]={0};
    int val;
    va_list arg;
    va_start (arg, format); 
    val = vsprintf(buffer,format,arg);
    va_end(arg);

    String log_line((char*)buffer);
    display.draw_to_console(log_line);
    display.display();
    return val;
}


void task_oled_console_history(void *)
{


    for(int i=0;i<10;i++)
        memset(Log_buffer[i],0,90);


    display.resetOrientation();
    display.init();


    display.flipScreenVertically();

    // display.mirrorScreen();

    display.setTextAlignment(TEXT_ALIGN_LEFT);

    display.clear();
    display.display();

    display.drawString(10,10, "Hello world");

    display.display();
    vTaskDelay(1000 / portTICK_RATE_MS);

    int count = 0;

    while(1)
    {
        // display.clear();
        // for(int i=0;i<7;i++)
        // {
        //     oled_printf("%d:Hello world!\n",count++);
        // }
        // display.display();
        vTaskDelay(100 / portTICK_RATE_MS);
    }

}

#include "driver/uart.h"

QueueHandle_t spp_uart_queue = NULL;

void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if (event.size) {
                    uint8_t * temp = NULL;
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size+1);//Add '\0' after
                    if(temp == NULL){
                        ESP_LOGE("OLED spp", "malloc failed,%s L#%d\n", __func__, __LINE__);
                        break;
                    }
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                    //此事件中无法获得 \n 字符
                    for(int evt_len = 0;evt_len<event.size;evt_len++)
                    {
 
                        if(temp[evt_len]=='\r')
                        {
                            temp[evt_len]='\n';
                            ets_printf("\r\n\n~:");
                            oled_printf("%c",temp[evt_len]);   
                            oled_printf("~:");   
                        }
                        else
                        {
                            oled_printf("%c",temp[evt_len]); 
                            ets_printf("%c",temp[evt_len]);
                        }
                                             
                    }
                 
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 5120, (void*)UART_NUM_0, 8, NULL);
}