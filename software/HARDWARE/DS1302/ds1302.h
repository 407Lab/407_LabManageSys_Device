#ifndef _DS1302_H_
#define _DS1302_H_
#include "stm32f4xx.h"
#include "sys.h"
/******************************************************************************
* @ File name --> ds1302.h
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 08 - 14 - 2019
* @ Brief     --> ds1302时钟模块驱动程序头文件
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

//IO方向设置
#define DS1302_IO_IN()  {GPIOB->MODER&=~(3<<(13*2));GPIOB->MODER|=0<<13*2;}
#define DS1302_IO_OUT() {GPIOB->MODER&=~(3<<(13*2));GPIOB->MODER|=1<<13*2;}
//IO操作函数
#define	DS1302_DATA_OUT PBout(13) //数据端口	PB13
#define	DS1302_DATA_IN  PBin(13)  //数据端口	PB13

#define	DS1302_SCK  PBout(12)
#define	DS1302_RST  PBout(14)

//DS1302地址定义
#define ds1302_sec_add			  0x80		//秒数据地址
#define ds1302_min_add			  0x82		//分数据地址
#define ds1302_hr_add			    0x84		//时数据地址
#define ds1302_date_add			  0x86		//日数据地址
#define ds1302_month_add		  0x88		//月数据地址
#define ds1302_day_add			  0x8a		//星期数据地址
#define ds1302_year_add			  0x8c		//年数据地址
#define ds1302_control_add		0x8e		//控制数据地址
#define ds1302_charger_add		0x90
#define ds1302_clkburst_add		0xbe

void DS1302_Init(void);
void DS1302_Write_Byte(u8 addr, u8 data);
u8 DS1302_Read_Byte(u8 addr);
void DS1302_Write_Time(void);
void DS1302_Read_Time(void);
void DS1302_Get_Time(u8 *time);

#endif

