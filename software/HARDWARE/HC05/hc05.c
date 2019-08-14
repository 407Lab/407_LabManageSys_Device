/******************************************************************************
* @ File name --> hc05.c
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 08 - 14 - 2019
* @ Brief     --> 蓝牙模块驱动程序
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

#include "hc05.h"
#include "usart.h"
#include "string.h"
#include "math.h"
#include "delay.h"


//初始化HC05模块
//返回值:0,成功;1,失败.
u8 HC05_Init(void)
{
		u8 retry=10,t;
		u8 temp=1;

		usart3_init(9600);	//初始化串口3为:9600,波特率.

		while(retry--)
		{
				delay_ms(10);
				u3_printf("AT\r\n");		//发送AT测试指令
				for(t=0;t<10;t++) 			//最长等待50ms,来接收HC05模块的回应
				{
						if(USART3_RX_STA&0X8000)break;
						delay_ms(5);
				}
				if(USART3_RX_STA&0X8000)	//接收到一次数据了
				{
						temp=USART3_RX_STA&0X7FFF;	//得到数据长度
						USART3_RX_STA=0;
						if(temp==4&&USART3_RX_BUF[0]=='O'&&USART3_RX_BUF[1]=='K')
						{
								temp=0;//接收到OK响应
								break;
						}
				}
		}
		if(retry==0)temp=1;	//检测失败
		return temp;
}	 

