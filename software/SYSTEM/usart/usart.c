/******************************************************************************
* @ File name --> usart.c
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> 系统串口驱动程序
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
#include "usart.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"


#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
		int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x)
{
		x = x;
		return x;
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
		USART1->DR = (u8) ch;
		return ch;
}
#endif

//串口接收缓存区
__align(8) u8 USART1_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
__align(8) u8 USART2_RX_BUF[USART_REC_LEN];
__align(8) u8 USART3_RX_BUF[USART_REC_LEN];
__align(8) u8 USART4_RX_BUF[USART_REC_LEN];
__align(8) u8 USART6_RX_BUF[USART_REC_LEN];
//串口发送缓存区
__align(8) u8 USART1_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART2_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART3_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART4_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART6_TX_BUF[USART_SEND_LEN];
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART1_RX_STA = 0;       //接收状态标记
u16 USART2_RX_STA = 0;
u16 USART3_RX_STA = 0;
u16 USART4_RX_STA = 0;
u16 USART6_RX_STA = 0;

//初始化IO 串口1 
//bound:波特率
void usart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART1_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART1_RX_STA=0;//接收错误,重新开始
				else USART1_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{
				if(Res==0x0d)USART1_RX_STA|=0x4000;
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=Res ;
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART_REC_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}
			}
		}
  }
}


void usart2_init(u32 bound)
{
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟

		//串口2对应引脚复用映射
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9复用为USART2
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10复用为USART2

		//USART2端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3

		//USART2 初始化设置
		USART_InitStructure.USART_BaudRate = bound;//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
		USART_Init(USART2, &USART_InitStructure); //初始化串口2

		USART_Cmd(USART2, ENABLE);  //使能串口2 

		//USART_ClearFlag(USART2, USART_FLAG_TC);

		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

		//Usart2 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

void USART2_IRQHandler(void)                	//串口1中断服务程序
{
		u8 Res;
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
		{
				Res =USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
		}
}

//串口2printf 函数
//确保一次发送数据不超过USART2_SEND_LEN字节
void u2_printf(char* fmt,...)
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART2_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART2_TX_BUF);//此次发送数据的长度
		for(j=0;j<i;j++)//循环发送数据
		{
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  //等待上次传输完成
				USART_SendData(USART2,(uint8_t)USART2_TX_BUF[j]); 	 //发送数据到串口2
		}
}

void usart3_init(u32 bound)
{
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART1时钟

		//串口3对应引脚复用映射
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART1); //GPIOB10复用为USART3
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3

		//USART3端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB10，PB11

		//USART3 初始化设置
		USART_InitStructure.USART_BaudRate = bound;//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
		USART_Init(USART3, &USART_InitStructure); //初始化串口3

		USART_Cmd(USART3, ENABLE);  //使能串口3

		//USART_ClearFlag(USART3, USART_FLAG_TC);

		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

		//Usart3 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

void USART3_IRQHandler(void)                	//串口3中断服务程序
{
		u8 Res;
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
		{
				Res =USART_ReceiveData(USART3);
				if((USART3_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
				{
						if(USART3_RX_STA<USART_REC_LEN)		//还可以接收数据
						{
								TIM_SetCounter(TIM7,0);//计数器清空
								if(USART3_RX_STA==0)
								TIM_Cmd(TIM7, ENABLE);  //使能定时器7
								USART3_RX_BUF[USART3_RX_STA++] = Res;		//记录接收到的值
						}else
						{
								USART3_RX_STA|=1<<15;					//强制标记接收完成
						}
				}
		}										 
}

//串口3printf 函数
//确保一次发送数据不超过USART3_MAX_SEND_LEN字节
void u3_printf(char* fmt,...)  
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART3_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART3_TX_BUF);//此次发送数据的长度
		for(j=0;j<i;j++)//循环发送数据
		{
				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);  //等待上次传输完成 
				USART_SendData(USART3,(uint8_t)USART3_TX_BUF[j]); 	 //发送数据到串口3 
		}
}


void uart4_init(u32 bound)
{
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART4时钟

		//串口4对应引脚复用映射
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0复用为UART4
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1复用为UART4

		//USART4端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //GPIOA0与GPIOA1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

		//USART4 初始化设置
		USART_InitStructure.USART_BaudRate = bound;//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
		USART_Init(UART4, &USART_InitStructure); //初始化串口4

		USART_Cmd(UART4, ENABLE);  //使能串口4

		//USART_ClearFlag(UART4, USART_FLAG_TC);

		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

		//Usart4 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

void USART4_IRQHandler(void)                	//串口4中断服务程序
{
		u8 Res;
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断
		{
				Res = USART_ReceiveData(UART4);//(UART4->DR);	//读取接收到的数据
		}
}

//串口4printf 函数
//确保一次发送数据不超过USART4_SEND_LEN字节
void u4_printf(char* fmt,...)  
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART4_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART3_TX_BUF);//此次发送数据的长度
		for(j=0;j<i;j++)//循环发送数据
		{
				while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);  //等待上次传输完成 
				USART_SendData(UART4,(uint8_t)USART4_TX_BUF[j]); 	 //发送数据到串口4 
		}
}

void uart6_init(u32 bound)
{
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOG时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟

		//串口1对应引脚复用映射
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART1); //GPIOG9复用为USART6
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART1); //GPIOG14复用为USART6

		//USART6端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9与GPIOG14
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PA9，PA10

		//USART6 初始化设置
		USART_InitStructure.USART_BaudRate = bound;//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
		USART_Init(USART6, &USART_InitStructure); //初始化串口6

		USART_Cmd(USART6, ENABLE);  //使能串口6

		//USART_ClearFlag(USART6, USART_FLAG_TC);

		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

		//Usart1 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口6中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

void USART6_IRQHandler(void)                	//串口6中断服务程序
{
		u8 Res;
		if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断
		{
				Res =USART_ReceiveData(USART6);//(USART6->DR);	//读取接收到的数据
		}
}

//串口6printf 函数
//确保一次发送数据不超过USART6_SEND_LEN字节
void u6_printf(char* fmt,...)  
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART6_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART6_TX_BUF);//此次发送数据的长度
		for(j=0;j<i;j++)//循环发送数据
		{
				while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);  //等待上次传输完成 
				USART_SendData(USART6,(uint8_t)USART6_TX_BUF[j]); 	 //发送数据到串口6
		}
}




