/******************************************************************************
* @ File name --> hc05.c
* @ Author    --> By@ ³��
* @ Version   --> V1.0
* @ Date      --> 08 - 14 - 2019
* @ Brief     --> ����ģ����������
*
* @ Copyright (C) 2019
* @ All rights reserved	www.cqutlab.club
*******************************************************************************
*
*                                  File Update
* @ Version   --> V1.0
* @ Author    --> By@³��
* @ Date      --> 08 - 14 - 2019
* @ Revise    --> �½��ļ�
*
******************************************************************************/

#include "hc05.h"
#include "usart.h"
#include "string.h"
#include "math.h"
#include "delay.h"


//��ʼ��HC05ģ��
//����ֵ:0,�ɹ�;1,ʧ��.
u8 HC05_Init(void)
{
		u8 retry=10,t;
		u8 temp=1;

		usart3_init(9600);	//��ʼ������3Ϊ:9600,������.

		while(retry--)
		{
				delay_ms(10);
				u3_printf("AT\r\n");		//����AT����ָ��
				for(t=0;t<10;t++) 			//��ȴ�50ms,������HC05ģ��Ļ�Ӧ
				{
						if(USART3_RX_STA&0X8000)break;
						delay_ms(5);
				}
				if(USART3_RX_STA&0X8000)	//���յ�һ��������
				{
						temp=USART3_RX_STA&0X7FFF;	//�õ����ݳ���
						USART3_RX_STA=0;
						if(temp==4&&USART3_RX_BUF[0]=='O'&&USART3_RX_BUF[1]=='K')
						{
								temp=0;//���յ�OK��Ӧ
								break;
						}
				}
		}
		if(retry==0)temp=1;	//���ʧ��
		return temp;
}	 

