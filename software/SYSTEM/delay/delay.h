#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>

/******************************************************************************
* @ File name --> delay.h
* @ Author    --> By@ ³��
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> ��ʱ����ͷ�ļ�
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

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























