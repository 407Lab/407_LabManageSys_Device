/******************************************************************************
* @ File name --> main.c
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> 系统主函数文件
*
* @ Copyright (C) 2019
* @ All rights reserved	www.cqutlab.club
*******************************************************************************
*
*                                  File Update
* @ Version   --> V1.0
* @ Author    --> By@鲁亮
* @ Date      --> 07 - 16 - 2019
* @ Revise    --> 新建文件
*
******************************************************************************/

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	
	LED_Init();					  //初始化LED
 	LCD_Init();           //初始化LCD FSMC接口
	POINT_COLOR=RED;      //画笔颜色：红色
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
