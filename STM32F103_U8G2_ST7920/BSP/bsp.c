#include "bsp.h"

void bsp_init(void)
{ 
  /* �����ж����ȼ����� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  systick_init();
  usart1_init(115200);
  TIM2_Int_Init(999,719); //10ms
  KEY_Init();
//  ultrasonic_init();  //��ʼ��������ģ��
}


