/******************************************************************************
* @ File name --> beep.c
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 08 - 14 - 2019
* @ Brief     --> 蜂鸣器驱动程序
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

#include "beep.h"

void BEEP_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟

    //初始化蜂鸣器对应引脚GPIOF8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIO

    GPIO_ResetBits(GPIOF,GPIO_Pin_8);  //蜂鸣器对应引脚GPIOF8拉低，
}


