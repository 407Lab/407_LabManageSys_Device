#ifndef _SPI_H_
#define _SPI_H_
#include "sys.h"

/******************************************************************************
* @ File name --> spi.h
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 07 - 17 - 2019
* @ Brief     --> spi总线驱动头文件
*
* @ Copyright (C) 2019
* @ All rights reserved	www.cqutlab.club
*******************************************************************************
*
*                                  File Update
* @ Version   --> V1.0
* @ Author    --> By@鲁亮
* @ Date      --> 07 - 17 - 2019
* @ Revise    --> 新建文件
*
******************************************************************************/

#define SPIReadByte() SPIWriteByte(0) 

#define MYRC522_CS   PAout(4)
#define MYRC522_RST  PAout(6)

void delay_ns(u32 ns);
u8 SPIWriteByte(u8 Byte);
void SPI3_Init(void);

#endif












