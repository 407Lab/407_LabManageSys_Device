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
#define LED0 PFout(9)	// DS0
#define LED1 PFout(10)	// DS1	 

void LED_Init(void);//��ʼ��		 				    
#endif
