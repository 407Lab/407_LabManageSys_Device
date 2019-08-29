#ifndef _BEEP_H
#define _BEEP_H
#include "sys.h"

/******************************************************************************
* @ File name --> beep.h
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 08 - 14 - 2019
* @ Brief     --> 蜂鸣器驱动程序头文件
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

#define BEEP PFout(8)	// 蜂鸣器控制IO

void BEEP_Init(void);//初始化

#endif


