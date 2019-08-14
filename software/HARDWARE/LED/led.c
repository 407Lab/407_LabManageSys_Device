/******************************************************************************
* @ File name --> led.c
* @ Author    --> By@ ³��
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> led��������
*
* @ Copyright (C) 2019
* @ All rights reserved	www.cqutlab.club
*******************************************************************************
*
*                                  File Update
* @ Version   --> V1.0
* @ Author    --> By@³��
* @ Date      --> 07 - 16 - 2019
* @ Revise    --> �½��ļ�
*
******************************************************************************/
#include "led.h" 
 
//LED IO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
	
	GPIO_SetBits(GPIOE,GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_6);//GPIOF4,F5,F6���øߣ�����

}






