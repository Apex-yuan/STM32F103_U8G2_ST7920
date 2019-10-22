#ifndef _U8G_ARM_H
#define _U8G_ARM_H

#include "u8g2.h"
#include "u8x8.h"

#include "stm32f10x_conf.h"

#define MODE_DATA 0XFA
#define MODE_CMD  0XF8



/*******软件SPI********/
#define SWSPI //软件SPI


#ifdef SWSPI 

#define SWSPI_GPIO_CLK    RCC_APB2Periph_GPIOE
#define SWSPI_GPIO_PORT   GPIOE
#define SWSPI_CS_PIN      GPIO_Pin_12
#define SWSPI_DIN_PIN     GPIO_Pin_13
#define SWSPI_CLK_PIN     GPIO_Pin_14
#define SWSPI_PSB_PIN     GPIO_Pin_15

#define SET_CS(status) (PEout(12) = status)    //CS-->RS 片选引脚
#define SET_DIN(status) (PEout(13) = status)  //  MOSI-->R/W 数据传输引脚
#define SET_CLK(status) (PEout(14) = status)  // SCLK-->E 时钟引脚

#endif /* SWSPI */

/*******硬件SPI********/
//#define HW_SPI //硬件SPI

#ifdef HWSPI



#endif

//*************************************************************************
void WS2811_delay(unsigned int delay_num);
void SW_SPI_Init(void);
void HW_SPI_Init(void);
void SPI_SendData(uint8_t data);
void SPI_WriteByte(uint8_t data, uint8_t mode);


uint8_t u8x8_byte_stm32_sw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif

