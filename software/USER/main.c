/******************************************************************************
* @ File name --> main.c
* @ Author    --> By@ ³��
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> ϵͳ�������ļ�
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

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);      //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	
	LED_Init();					  //��ʼ��LED
 	LCD_Init();           //��ʼ��LCD FSMC�ӿ�
	POINT_COLOR=RED;      //������ɫ����ɫ
	POINT_COLOR=RED;	  
	LCD_ShowString(30,40,210,24,24,(u8 *)"Smart Lab");	
	LCD_ShowString(30,70,200,16,16,(u8 *)"407Lab");
	LCD_ShowString(30,90,200,16,16,(u8 *)"www.cqutlab.club");    					 
	LCD_ShowString(30,110,200,12,12,(u8 *)"2019/07/16");
  while(1)
	{
		LED0=!LED0;	 
		delay_ms(1000);	
	} 
}
