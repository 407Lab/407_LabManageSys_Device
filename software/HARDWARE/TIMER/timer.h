#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
/******************************************************************************
* @ File name --> timer.h
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 08 - 14 - 2019
* @ Brief     --> 定时器驱动程序头文件
*
* @ Copyright (C) 2019
* @ All rights reserved	www.cqutlab.club
*******************************************************************************
*
*                                  File Update
* @ Version   --> V1.0
* @ Author    --> By@鲁亮
* @ Date      --> 08 - 14 - 2019
* @ Revise    --> 新建文件
*
******************************************************************************/

void TIM7_Int_Init(u16 arr,u16 psc);

#endif


