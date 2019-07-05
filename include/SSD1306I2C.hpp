/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 by Helmut Tschemernjak - www.radioshuttle.de
 * Copyright (c) 2019 by gengyuchao - https://github.com/gengyuchao
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

 /*
  * TODO gengyuchao
  * - Migrate the program to stm32f103 for FreeRTOS and NO_OS
  * - Migrate the program to the esp8266-IDF SDK
  * - Added a simple Hardware IIC program for stm32 
  * - Added a simple Software IIC program for esp8266
  */

#ifndef SSD1306I2C_h
#define SSD1306I2C_h

#include "user_oled_config.h"

#include "OLEDDisplay.hpp"


#if defined(__STM32F103__) 

	#define PinName uint32_t

	#ifndef UINT8_MAX
	#define UINT8_MAX 0xff
	#endif

	class I2C
	{
	public:
		//在STM32F103系列中 只有端口B有硬件IIC功能 故默认使用GPIOB
		I2C(PinName scl, PinName sda)
		{
			_sda = sda;
			_scl = scl;
		}

		void frequency(uint32_t freq)
		{
			I2C_InitTypeDef I2C_InitStructure;
			GPIO_InitTypeDef GPIO_InitStructure;

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

			/*STM32F103C8T6 硬件 I2C2: PB10 -- SCL; PB11 -- SDA */
			/*STM32F103C8T6 硬件 I2C1: PB6  -- SCL; PB7  -- SDA */
			/*STM32F103C8T6 硬件 I2C1: PB8  -- SCL; PB9  -- SDA */

			if (_scl == GPIO_Pin_6 && _sda == GPIO_Pin_7)
			{
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
				I2Cx = I2C1;
			}
			else if (_scl == GPIO_Pin_8 && _sda == GPIO_Pin_9)
			{
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
				I2Cx = I2C1;
			}
			else if (_scl == GPIO_Pin_10 && _sda == GPIO_Pin_11)
			{
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
				I2Cx = I2C2;
			}
			else
			{
				printf("STM32 do not have this Hardware IIC.\n");
				I2Cx = I2C2;
				return;
			}

			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //I2C必须开漏输出
			GPIO_Init(GPIOB, &GPIO_InitStructure);

			I2C_DeInit(I2Cx); //使用I2Cx
			I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
			I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
			I2C_InitStructure.I2C_OwnAddress1 = 0x30; //主机的I2C地址,随便写的
			I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
			I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
			I2C_InitStructure.I2C_ClockSpeed = freq; //400K

			I2C_Cmd(I2Cx, ENABLE);
			I2C_Init(I2Cx, &I2C_InitStructure);
		}
		uint8_t write(uint8_t _address, char *data, uint8_t length)
		{
			while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
				;

			I2C_GenerateSTART(I2Cx, ENABLE); //开启I2Cx
			while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
				; /*EV5,主模式*/

			I2C_Send7bitAddress(I2Cx, _address, I2C_Direction_Transmitter); //器件地址 -- 默认0x78
			while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
				;

			while (length-- > 0)
			{
				I2C_SendData(I2Cx, *(data++)); //发送数据
				while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
					;
			}

			I2C_GenerateSTOP(I2Cx, ENABLE); //关闭I2Cx总线

			return 0;
		}

	private:
		PinName _sda;
		PinName _scl;
		I2C_TypeDef *I2Cx;
	};

#elif defined(__ESP8266_IDF__) 

#include "driver/gpio.h"
#define PinName gpio_num_t

#define I2C_MASTER_GPIO_OUT(pin,val)  gpio_set_level(pin, val)

#ifndef UINT8_MAX
 #define UINT8_MAX 0xff
#endif

class I2C
{
public:
	//在STM32F103系列中 只有端口B有硬件IIC功能 故默认使用GPIOB
	I2C(PinName scl, PinName sda)
	{
		_sda = sda;
		_scl = scl;
		i2c_gpio_init(_sda, _scl);

	}

	esp_err_t i2c_gpio_init(PinName sda_io_num, PinName scl_io_num)
	{
		gpio_config_t io_conf;

		printf("init oled i2c\n");
		// disable interrupt
		io_conf.intr_type = GPIO_INTR_DISABLE;
		// set as output mode
		io_conf.mode = GPIO_MODE_OUTPUT_OD;
		// bit mask of the pins that you want to set
		io_conf.pin_bit_mask = (1ULL << sda_io_num)|(1ULL << scl_io_num);
		// disable pull-down mode
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		// disable pull-up mode
		io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

		// configure GPIO with the given settings
		ESP_ERROR_CHECK(gpio_config(&io_conf));
		ESP_ERROR_CHECK(gpio_set_level(sda_io_num, 1));
		ESP_ERROR_CHECK(gpio_set_level(scl_io_num, 1));

		printf("\nOLED_SDA_GPIO:%d  OLED_SCL_GPIO:%d",sda_io_num,scl_io_num);

		return ESP_OK;
	}

	void frequency(uint32_t freq)
	{
		printf("ESP8266 I2C Frequency has always been the fastest.\n");
	}

/*------IIC Base function------*/
// #define delay_us os_delay_us //慢速模式，确保I2C稳定
	void delay_us(int xus)//超高速模式,减少延时时间
	{
	// volatile uint8_t i;
	// for(i=0;i<xus;i++);
	}
	
	//开始信号
	void IIC_Start(void)
	{
		gpio_set_direction(_sda,GPIO_MODE_OUTPUT);//SDA_OUT();
		I2C_MASTER_GPIO_OUT(_sda,1);//IIC_SDA=1;
		I2C_MASTER_GPIO_OUT(_scl,1);//IIC_SCL=1;
		delay_us(2);
		I2C_MASTER_GPIO_OUT(_sda,0);//IIC_SDA=0;
		delay_us(2);
		I2C_MASTER_GPIO_OUT(_scl,0);//IIC_SCL=0;
		delay_us(2);
	}
	
	void IIC_Stop(void)
	{
		I2C_MASTER_GPIO_OUT(_scl,1);//IIC_SCL=1;
		I2C_MASTER_GPIO_OUT(_sda,0);//IIC_SDA=0;
		delay_us(2);
		I2C_MASTER_GPIO_OUT(_sda,1);//IIC_SDA=1;
		delay_us(2);
	}
	
	/*
	*   返回1--应答出错
	*   返回0--应答正确
	*/
	uint8_t IIC_Wait_Ask(void)
	{
		int count=0;
	
		gpio_set_direction(_sda,GPIO_MODE_INPUT);//    SDA_IN();
	
		I2C_MASTER_GPIO_OUT(_scl,1);//IIC_SCL=1;
		delay_us(2);
		while(gpio_get_level(_sda)) //
		{
			count++;
			if(count>250)
			{
				IIC_Stop();
				return 1;
			}
		}
		I2C_MASTER_GPIO_OUT(_scl,0);//IIC_SCL=0;
		delay_us(2);
		return 0;
	}
	
	//写一个字节
	void IIC_WriteByte(uint8_t data)
	{
		uint8_t i;
		gpio_set_direction(_sda,GPIO_MODE_OUTPUT);//SDA_OUT();
		for(i=0;i<8;i++)
		{
			I2C_MASTER_GPIO_OUT(_scl,0);//IIC_SCL=0;
			delay_us(2);
			if(data & 0x80)     //MSB,从高位开始一位一位传输
				I2C_MASTER_GPIO_OUT(_sda,1);//IIC_SDA=1;
			else
				I2C_MASTER_GPIO_OUT(_sda,0);//IIC_SDA=0;
			I2C_MASTER_GPIO_OUT(_scl,1);//IIC_SCL=1;
			delay_us(2);
			I2C_MASTER_GPIO_OUT(_scl,0);//IIC_SCL=0;
			data<<=1;
	
		}
	}
	
	uint8_t IIC_ReadByte(void)
	{
		uint8_t data=0,i=0;
		I2C_MASTER_GPIO_OUT(_sda,1);//IIC_SDA=1;
		delay_us(2);
		for(i=0;i<8;i++)
		{
			data<<=1;
			I2C_MASTER_GPIO_OUT(_scl,0);//IIC_SCL=0;
			delay_us(2);
			I2C_MASTER_GPIO_OUT(_scl,1);//IIC_SCL=1;
			delay_us(2);
			if(gpio_get_level(_sda))//(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))
				data=data | 0x01;
			else
				data=data & 0xFE;
	
		}
		I2C_MASTER_GPIO_OUT(_scl,0);//IIC_SCL=0;
		delay_us(2);
		return data;
	
	}

	uint8_t write(uint8_t _address, char *data, uint8_t length)
	{
		IIC_Start();
		IIC_WriteByte(_address);//OLED地址
		IIC_Wait_Ask();
		for(int i=0;i<length;i++)
		{
			IIC_WriteByte(*(data+i));
			IIC_Wait_Ask();
		}
		IIC_Stop();
		return ESP_OK;

	}

private:
	PinName _sda;
	PinName _scl;

};


#endif


class SSD1306I2C : public OLEDDisplay
{
public:
	SSD1306I2C(uint8_t _address, PinName _sda, PinName _scl, OLEDDISPLAY_GEOMETRY g = GEOMETRY_128_64)
	{
		setGeometry(g);

		this->_address = _address << 1; // convert from 7 to 8 bit for mbed.
		this->_sda = _sda;
		this->_scl = _scl;
		_i2c = new I2C(_sda, _scl);
	}

	bool connect()
	{
		// mbed supports 100k and 400k some device maybe 1000k
#ifdef TARGET_STM32L4
		_i2c->frequency(1000000);
#else
		_i2c->frequency(400000);
#endif
		return true;
	}

	void display(void)
	{
		const int x_offset = (128 - this->width()) / 2;
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
		uint8_t minBoundY = UINT8_MAX;
		uint8_t maxBoundY = 0;

		uint8_t minBoundX = UINT8_MAX;
		uint8_t maxBoundX = 0;
		uint8_t x, y;

		// Calculate the Y bounding box of changes
		// and copy buffer[pos] to buffer_back[pos];
		for (y = 0; y < (this->height() / 8); y++)
		{
			for (x = 0; x < this->width(); x++)
			{
				uint16_t pos = x + y * this->width();
				if (buffer[pos] != buffer_back[pos])
				{
					minBoundY = min(minBoundY, y);
					maxBoundY = max(maxBoundY, y);
					minBoundX = min(minBoundX, x);
					maxBoundX = max(maxBoundX, x);
				}
				buffer_back[pos] = buffer[pos];
			}
			yield();
		}

		// If the minBoundY wasn't updated
		// we can savely assume that buffer_back[pos] == buffer[pos]
		// holdes true for all values of pos

		if (minBoundY == UINT8_MAX)
			return;

		sendCommand(COLUMNADDR);
		sendCommand(x_offset + minBoundX); // column start address (0 = reset)
		sendCommand(x_offset + maxBoundX); // column end address (127 = reset)

		sendCommand(PAGEADDR);
		sendCommand(minBoundY); // page start address
		sendCommand(maxBoundY); // page end address

		for (y = minBoundY; y <= maxBoundY; y++)
		{
			uint8_t *start = &buffer[(minBoundX + y * this->width()) - 1];
			uint8_t save = *start;

			*start = 0x40; // control
			_i2c->write(_address, (char *)start, (maxBoundX - minBoundX) + 1 + 1);
			*start = save;
		}
#else

		sendCommand(COLUMNADDR);
		sendCommand(x_offset);						 // column start address (0 = reset)
		sendCommand(x_offset + (this->width() - 1)); // column end address (127 = reset)

		sendCommand(PAGEADDR);
		sendCommand(0x0); // page start address (0 = reset)

		if (geometry == GEOMETRY_128_64)
		{
			sendCommand(0x7);
		}
		else if (geometry == GEOMETRY_128_32)
		{
			sendCommand(0x3);
		}

		buffer[-1] = 0x40; // control
		_i2c->write(_address, (char *)&buffer[-1], displayBufferSize + 1);
#endif
	}

private:
	int getBufferOffset(void)
	{
		return 0;
	}

	inline void sendCommand(uint8_t command) __attribute__((always_inline))
	{
		char _data[2];
		_data[0] = 0x80; // control
		_data[1] = command;
		_i2c->write(_address, _data, sizeof(_data));
	}

	uint8_t _address;
	PinName _sda;
	PinName _scl;
	I2C *_i2c;
};

#endif
