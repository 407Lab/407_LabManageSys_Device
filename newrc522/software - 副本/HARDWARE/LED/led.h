#ifndef __LED_H
#define __LED_H
#include "sys.h"

/******************************************************************************
* @ File name --> led.h
* @ Author    --> By@ ³��
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> led������ͷ�ļ�
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

//LED�˿ڶ���
#define LED1	PEout(4)	// DS1
#define LED2	PEout(5)	// DS2
#define LED3	PEout(6)	// DS3

void LED_Init(void);//��ʼ��		 				    
#endif
