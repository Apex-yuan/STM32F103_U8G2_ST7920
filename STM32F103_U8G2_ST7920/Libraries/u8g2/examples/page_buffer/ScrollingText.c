/**
  ******************************************************************************
  * @file    bsp_u8g2.c 
  * @author  Apex yuan
  * @version V1.0.0
  * @date    2019-10-19
  * @brief   Main program body
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "bsp.h"
#include "bsp_u8g2.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8g2_t u8g2;

u8g2_uint_t offset1;			// current offset for the scrolling text
u8g2_uint_t width;			// pixel width of the scrolling text (must be lesser than 128 unless U8G2_16BIT is defined
const char *text = "U8g2 ";	// scroll this text from right to left

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  bsp_init();
  
  u8g2_Setup_st7920_s_128x64_2(&u8g2, U8G2_R0, u8x8_byte_sw_spi, u8g2_gpio_and_delay_stm32); //注册屏幕的回调函数
  u8g2_InitDisplay(&u8g2); //初始化屏幕的显示设置
 
  u8g2_SetFont(&u8g2, u8g2_font_inb30_mr);
  width = u8g2_GetUTF8Width(&u8g2, text);
  u8g2_SetFontMode(&u8g2, 0);
  
  /* Infinite loop */
  while (1)
  {
    u8g2_uint_t x;
    
    u8g2_FirstPage(&u8g2);
    do
    {    
      x = offset1;
      u8g2_SetFont(&u8g2, u8g2_font_inb30_mr);
      do{
        u8g2_DrawUTF8(&u8g2, x, 30, text);
        x += width;
      }while(x < u8g2_GetDisplayWidth(&u8g2));
      
      u8g2_SetFont(&u8g2, u8g2_font_inb16_mr);
      u8g2_DrawUTF8(&u8g2, 0, 58, text);
     
    }while( u8g2_NextPage(&u8g2) );

    offset1-=1;	// scroll by one pixel
    if ( (u8g2_uint_t)offset1 < (u8g2_uint_t)-width )	
      offset1 = 0; // start over again

    delay_ms(100);
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
