#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

/******************************************************************************
* @ File name --> usart.h
* @ Author    --> By@ ³��
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> ϵͳ��������ͷ�ļ�
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

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define	USART_SEND_LEN			255		//����������ֽ��� 255
//����1���ݻ���
extern u16 USART1_RX_STA;         		//����״̬���
extern u8  USART1_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern u8  USART1_TX_BUF[USART_SEND_LEN];
//����2���ݻ���
extern u16 USART2_RX_STA;
extern u8  USART2_RX_BUF[USART_REC_LEN];
extern u8  USART2_TX_BUF[USART_SEND_LEN];
//����3���ݻ���
extern u16 USART3_RX_STA;
extern u8  USART3_RX_BUF[USART_REC_LEN];
extern u8  USART3_TX_BUF[USART_SEND_LEN];
//����4���ݻ���
extern u16 USART4_RX_STA;
extern u8  USART4_RX_BUF[USART_REC_LEN];
extern u8  USART4_TX_BUF[USART_SEND_LEN];
//����6���ݻ���
extern u16 USART6_RX_STA;
extern u8  USART6_RX_BUF[USART_REC_LEN];
extern u8  USART6_TX_BUF[USART_SEND_LEN];


void usart_init(u32 bound);			//����1��ʼ��
void usart2_init(u32 bound);		//����2��ʼ��
void usart3_init(u32 bound);		//����3��ʼ��
void uart4_init(u32 bound);			//����4��ʼ��
void usart6_init(u32 bound);		//����6��ʼ��

void u2_printf(char* fmt,...);	//����2printf ����
void u3_printf(char* fmt,...);	//����3printf ����
void u4_printf(char* fmt,...);	//����4printf ����
void u6_printf(char* fmt,...);	//����6printf ����

#endif


