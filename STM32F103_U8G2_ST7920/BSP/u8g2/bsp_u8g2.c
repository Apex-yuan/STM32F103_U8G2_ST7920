/**
  ******************************************************************************
  * @file    bsp_u8g2.c 
  * @author  Apex yuan
  * @version V1.0.0
  * @date    2019-10-19
  * @brief   提供u8g2库需要的回调函数
  ******************************************************************************
  * @attention
  * 1. 使用软件SPI时只需实现1个回调函数即可，spi接口的回调函数可以直接使用库提供的(u8x8_byte文件内)，当然也可以自己实现
  *    例如：u8g2_Setup_st7920_s_128x64_2(&u8g2, U8G2_R0, u8x8_byte_4wire_sw_spi, u8g2_gpio_and_delay_stm32)
  ******************************************************************************
  */
#include "bsp_u8g2.h"
#include "bsp.h"


/**
  * @brief  为u8g2提供GPIO配置和延时函数配置的回调函数
  * @param  None
  * @retval 0: fault 
  *         1: success
  */
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
      #ifdef HW_SPI		
			   HW_SPI_Init() ;//初始化硬件SPI
      #endif //HW_SPI
      #ifdef SWSPI	  
		     SW_SPI_Init(); //初始化软件SPI
		  #endif //SWSPI
      break;							// can be used to setup pins
    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
      break;    
    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
      __NOP();
      break;
    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
      delay_us(arg_int * 10);
      break;
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
      delay_ms(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
      break;							// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
    //case U8X8_MSG_GPIO_SPI_CLOCK:
      SET_CLK(arg_int);
      break;
    case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
    //case U8X8_MSG_GPIO_SPI_DATA:
      SET_DIN(arg_int);
      break;
    case U8X8_MSG_GPIO_D2:				// D2 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D3:				// D3 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D4:				// D4 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D5:				// D5 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D6:				// D6 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D7:				// D7 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
      SET_CS(arg_int);
      break;
    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS1:				// CS1 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS2:				// CS2 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:		// arg_int=0: Output low at I2C clock pin
      break;							// arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:			// arg_int=0: Output low at I2C data pin
      break;							// arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
      break;
    default:
      u8x8_SetGPIOResult(u8x8, 1);			// default return value
      break;
  }
  return 1;
}



/**
  * @brief  为u8g2提供软件SPI通信的回调函数
  * @param  None
  * @retval 0: fault 
  *         1: success
  */
uint8_t u8x8_byte_stm32_sw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t b;
  uint8_t *data;
 
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while( arg_int > 0 )
      {
        b = *data;
        data++;
        arg_int--;
        SPI_SendData(b);
      }
      break;
      
    case U8X8_MSG_BYTE_INIT:
      /* disable chipselect */
      SET_CS(0);
      
      break;
    case U8X8_MSG_BYTE_SET_DC:

      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      SET_CS(1);

      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      SET_CS(0);

      break;
    default:
      return 0;
  }
  return 1;
}


//软件SPI初始化
void SW_SPI_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;	
  RCC_APB2PeriphClockCmd(SWSPI_GPIO_CLK, ENABLE);	 //使能PE端口时钟

  GPIO_InitStructure.GPIO_Pin = SWSPI_CS_PIN | SWSPI_CLK_PIN | SWSPI_DIN_PIN | SWSPI_PSB_PIN;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(SWSPI_GPIO_PORT, &GPIO_InitStructure);					 //根据设定参数初始化GPIOE
  //GPIO_SetBits(GPIOE,GPIO_Pin_4 | GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);						 //PA.8 输出高
  GPIO_ResetBits(SWSPI_GPIO_PORT, SWSPI_PSB_PIN); //串行模式，要拉低PSB引脚
}

//硬件SPI初始化
void HW_SPI_Init(void)
{
	 /* configure pins used by SPI1
	 * PA5 = SCK--E（CLK）串行的同步时钟信号
	 * PA6 = MISO--RS（CS）串行的片选信号
	 * PA7 = MOSI--R/W（SID）串行的数据口
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_7);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;//SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//选择了串行时钟的稳态:时钟悬空高
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	//SPI1_ReadWriteByte(0xff);//启动传输	
}


//data:要发送的数据
void SPI_SendData(uint8_t data) //发送一个数据
{
#ifdef HW_SPI //硬件SPI
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {};//检查指定的SPI标志位设置与否:发送缓存空标志位
  SPI_I2S_SendData(SPI1, data); //通过外设SPIx发送一个数据
#endif
    
#ifdef SWSPI //软件模拟SPI	 
  for(uint8_t j=0;j<8;j++)        
  { 
    if((data<<j)&0x80)
      SET_DIN(1);
    else 
      SET_DIN(0); 
    SET_CLK(1); 
    delay_us(2);
    SET_CLK(0); 
  }    
#endif
}

////data：写入的数据/命令
////mode：写入的是数据还是命令. 0XF8:写入的是命令， 0XFA：写入的是数据
//void SPI_WriteByte(uint8_t data, uint8_t mode) //写一个字节
//{
//	CS_ON(); //打开片选                 
//	SPI_SendData(mode);           
//  SPI_SendData(data & 0xF0);   
//  SPI_SendData(data << 4);     
//  CS_OFF(); //关闭片选
//}

