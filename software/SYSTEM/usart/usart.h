#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

/******************************************************************************
* @ File name --> usart.h
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> 系统串口驱动头文件
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

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define	USART_SEND_LEN			255		//定义最大发送字节数 255
//串口1数据缓存
extern u16 USART1_RX_STA;         		//接收状态标记
extern u8  USART1_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
extern u8  USART1_TX_BUF[USART_SEND_LEN];
//串口2数据缓存
extern u16 USART2_RX_STA;
extern u8  USART2_RX_BUF[USART_REC_LEN];
extern u8  USART2_TX_BUF[USART_SEND_LEN];
//串口3数据缓存
extern u16 USART3_RX_STA;
extern u8  USART3_RX_BUF[USART_REC_LEN];
extern u8  USART3_TX_BUF[USART_SEND_LEN];
//串口4数据缓存
extern u16 USART4_RX_STA;
extern u8  USART4_RX_BUF[USART_REC_LEN];
extern u8  USART4_TX_BUF[USART_SEND_LEN];
//串口6数据缓存
extern u16 USART6_RX_STA;
extern u8  USART6_RX_BUF[USART_REC_LEN];
extern u8  USART6_TX_BUF[USART_SEND_LEN];


void usart_init(u32 bound);			//串口1初始化
void usart2_init(u32 bound);		//串口2初始化
void usart3_init(u32 bound);		//串口3初始化
void uart4_init(u32 bound);			//串口4初始化
void usart6_init(u32 bound);		//串口6初始化

void u2_printf(char* fmt,...);	//串口2printf 函数
void u3_printf(char* fmt,...);	//串口3printf 函数
void u4_printf(char* fmt,...);	//串口4printf 函数
void u6_printf(char* fmt,...);	//串口6printf 函数

#endif


