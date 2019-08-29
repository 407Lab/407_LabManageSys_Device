#ifndef __LED_H
#define __LED_H
#include "sys.h"

/******************************************************************************
* @ File name --> led.h
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> led灯驱动头文件
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

//LED端口定义
#define LED1	PEout(4)	// DS1
#define LED2	PEout(5)	// DS2
#define LED3	PEout(6)	// DS3

void LED_Init(void);//初始化		 				    
#endif
