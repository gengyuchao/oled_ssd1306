//user_oled_config.h

#ifndef __USER_OLED_CONFIG_H__
#define __USER_OLED_CONFIG_H__

#define __USE_FreeRTOS__ //FreeRTOS系统支持

#define __ESP8266_IDF__ //ESP-IDF系统支持

// #define __STM32F103__ //Stm32支持

#ifdef __ESP8266_IDF__
    #include "esp_system.h"
#endif


#endif