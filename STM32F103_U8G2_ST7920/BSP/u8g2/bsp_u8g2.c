/**
  ******************************************************************************
  * @file    bsp_u8g2.c 
  * @author  Apex yuan
  * @version V1.0.0
  * @date    2019-10-19
  * @brief   �ṩu8g2����Ҫ�Ļص�����
  ******************************************************************************
  * @attention
  * 1. ʹ�����SPIʱֻ��ʵ��1���ص��������ɣ�spi�ӿڵĻص���������ֱ��ʹ�ÿ��ṩ��(u8x8_byte�ļ���)����ȻҲ�����Լ�ʵ��
  *    ���磺u8g2_Setup_st7920_s_128x64_2(&u8g2, U8G2_R0, u8x8_byte_4wire_sw_spi, u8g2_gpio_and_delay_stm32)
  ******************************************************************************
  */
#include "bsp_u8g2.h"
#include "bsp.h"


/**
  * @brief  Ϊu8g2�ṩGPIO���ú���ʱ�������õĻص�����
  * @param  None
  * @retval 0: fault 
  *         1: success
  */
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
	switch(msg){
		//Initialize SPI peripheral
		case U8X8_MSG_GPIO_AND_DELAY_INIT:
		  //��ʼ��Ӳ���ӿڣ���ʱ����ͨ��IO��
      #ifdef HW_SPI		
			   HW_SPI_Init() ;//��ʼ��Ӳ��SPI
      #endif //HW_SPI
      #ifdef SWSPI	  
		     SW_SPI_Init(); //��ʼ�����SPI
		  #endif //SWSPI
		  break;

		//Function which implements a delay, arg_int contains the amount of ms
		case U8X8_MSG_DELAY_MILLI:;
      delay_ms(arg_int);
      break;
    
		//Function which delays 10us
		case U8X8_MSG_DELAY_10MICRO:
      delay_us(10);
      break;
    
		//Function which delays 100ns
		case U8X8_MSG_DELAY_100NANO:
      __NOP();
      break;
    
		//Function to define the logic level of the clockline
		case U8X8_MSG_GPIO_SPI_CLOCK:
      SET_CLK(arg_int);
      break;
    
		//Function to define the logic level of the data line to the display
		case U8X8_MSG_GPIO_SPI_DATA:
			SET_DIN(arg_int);
		break;
    
		// Function to define the logic level of the CS line
		case U8X8_MSG_GPIO_CS:
      SET_CS(arg_int);
      break;
    
		//Function to define the logic level of the Data/ Command line
		case U8X8_MSG_GPIO_DC:
      break;
    
		//Function to define the logic level of the RESET line
		case U8X8_MSG_GPIO_RESET:
      break;
    
		default:
			return 0; //A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}

/**
  * @brief  Ϊu8g2�ṩ���SPIͨ�ŵĻص�����
  * @param  None
  * @retval 0: fault 
  *         1: success
  */
uint8_t u8x8_byte_sw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
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


//���SPI��ʼ��
void SW_SPI_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;	
  RCC_APB2PeriphClockCmd(SWSPI_GPIO_CLK, ENABLE);	 //ʹ��PE�˿�ʱ��

  GPIO_InitStructure.GPIO_Pin = SWSPI_CS_PIN | SWSPI_CLK_PIN | SWSPI_DIN_PIN | SWSPI_PSB_PIN;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(SWSPI_GPIO_PORT, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOE
  //GPIO_SetBits(GPIOE,GPIO_Pin_4 | GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);						 //PA.8 �����
  GPIO_ResetBits(SWSPI_GPIO_PORT, SWSPI_PSB_PIN); //����ģʽ��Ҫ����PSB����
}

//Ӳ��SPI��ʼ��
void HW_SPI_Init(void)
{
	 /* configure pins used by SPI1
	 * PA5 = SCK--E��CLK�����е�ͬ��ʱ���ź�
	 * PA6 = MISO--RS��CS�����е�Ƭѡ�ź�
	 * PA7 = MOSI--R/W��SID�����е����ݿ�
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_7);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;//SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
	//SPI1_ReadWriteByte(0xff);//��������	
}


//data:Ҫ���͵�����
void SPI_SendData(uint8_t data) //����һ������
{
#ifdef HW_SPI //Ӳ��SPI
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {};//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
  SPI_I2S_SendData(SPI1, data); //ͨ������SPIx����һ������
#endif
    
#ifdef SWSPI //���ģ��SPI	 
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

////data��д�������/����
////mode��д��������ݻ�������. 0XF8:д�������� 0XFA��д���������
//void SPI_WriteByte(uint8_t data, uint8_t mode) //дһ���ֽ�
//{
//	CS_ON(); //��Ƭѡ                 
//	SPI_SendData(mode);           
//  SPI_SendData(data & 0xF0);   
//  SPI_SendData(data << 4);     
//  CS_OFF(); //�ر�Ƭѡ
//}

