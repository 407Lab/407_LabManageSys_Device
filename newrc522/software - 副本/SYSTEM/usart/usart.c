/******************************************************************************
* @ File name --> usart.c
* @ Author    --> By@ ³��
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> ϵͳ������������
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
#include "usart.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"


#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
		int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x)
{
		x = x;
		return x;
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{
		while((USART1->SR&0X40)==0);//ѭ������,ֱ���������
		USART1->DR = (u8) ch;
		return ch;
}
#endif

//���ڽ��ջ�����
__align(8) u8 USART1_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
__align(8) u8 USART2_RX_BUF[USART_REC_LEN];
__align(8) u8 USART3_RX_BUF[USART_REC_LEN];
__align(8) u8 USART4_RX_BUF[USART_REC_LEN];
__align(8) u8 USART6_RX_BUF[USART_REC_LEN];
//���ڷ��ͻ�����
__align(8) u8 USART1_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART2_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART3_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART4_TX_BUF[USART_SEND_LEN];
__align(8) u8 USART6_TX_BUF[USART_SEND_LEN];
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART1_RX_STA = 0;       //����״̬���
u16 USART2_RX_STA = 0;
u16 USART3_RX_STA = 0;
u16 USART4_RX_STA = 0;
u16 USART6_RX_STA = 0;

//��ʼ��IO ����1 
//bound:������
void usart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART1_RX_STA&0x8000)==0)//����δ���
		{
			if(USART1_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART1_RX_STA=0;//���մ���,���¿�ʼ
				else USART1_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{
				if(Res==0x0d)USART1_RX_STA|=0x4000;
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=Res ;
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART_REC_LEN-1))USART1_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}
			}
		}
  }
}


void usart2_init(u32 bound)
{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��

		//����2��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9����ΪUSART2
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10����ΪUSART2

		//USART2�˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2��GPIOA3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2��PA3

		//USART2 ��ʼ������
		USART_InitStructure.USART_BaudRate = bound;//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
		USART_Init(USART2, &USART_InitStructure); //��ʼ������2

		USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2 

		//USART_ClearFlag(USART2, USART_FLAG_TC);

		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

		//Usart2 NVIC ����
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

void USART2_IRQHandler(void)                	//����1�жϷ������
{
		u8 Res;
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
		{
				Res =USART_ReceiveData(USART2);//(USART2->DR);	//��ȡ���յ�������
		}
}

//����2printf ����
//ȷ��һ�η������ݲ�����USART2_SEND_LEN�ֽ�
void u2_printf(char* fmt,...)
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART2_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART2_TX_BUF);//�˴η������ݵĳ���
		for(j=0;j<i;j++)//ѭ����������
		{
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������
				USART_SendData(USART2,(uint8_t)USART2_TX_BUF[j]); 	 //�������ݵ�����2
		}
}

void usart3_init(u32 bound)
{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART1ʱ��

		//����3��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART1); //GPIOB10����ΪUSART3
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3

		//USART3�˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10��GPIOB11
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PB10��PB11

		//USART3 ��ʼ������
		USART_InitStructure.USART_BaudRate = bound;//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
		USART_Init(USART3, &USART_InitStructure); //��ʼ������3

		USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3

		//USART_ClearFlag(USART3, USART_FLAG_TC);

		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

		//Usart3 NVIC ����
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

void USART3_IRQHandler(void)                	//����3�жϷ������
{
		u8 Res;
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
		{
				Res =USART_ReceiveData(USART3);
				if((USART3_RX_STA&(1<<15))==0)//�������һ������,��û�б�����,���ٽ�����������
				{
						if(USART3_RX_STA<USART_REC_LEN)		//�����Խ�������
						{
								TIM_SetCounter(TIM7,0);//���������
								if(USART3_RX_STA==0)
								TIM_Cmd(TIM7, ENABLE);  //ʹ�ܶ�ʱ��7
								USART3_RX_BUF[USART3_RX_STA++] = Res;		//��¼���յ���ֵ
						}else
						{
								USART3_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
						}
				}
		}										 
}

//����3printf ����
//ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
void u3_printf(char* fmt,...)  
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART3_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART3_TX_BUF);//�˴η������ݵĳ���
		for(j=0;j<i;j++)//ѭ����������
		{
				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
				USART_SendData(USART3,(uint8_t)USART3_TX_BUF[j]); 	 //�������ݵ�����3 
		}
}


void uart4_init(u32 bound)
{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��USART4ʱ��

		//����4��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0����ΪUART4
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1����ΪUART4

		//USART4�˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //GPIOA0��GPIOA1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

		//USART4 ��ʼ������
		USART_InitStructure.USART_BaudRate = bound;//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
		USART_Init(UART4, &USART_InitStructure); //��ʼ������4

		USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���4

		//USART_ClearFlag(UART4, USART_FLAG_TC);

		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�

		//Usart4 NVIC ����
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����4�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

void USART4_IRQHandler(void)                	//����4�жϷ������
{
		u8 Res;
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //�����ж�
		{
				Res = USART_ReceiveData(UART4);//(UART4->DR);	//��ȡ���յ�������
		}
}

//����4printf ����
//ȷ��һ�η������ݲ�����USART4_SEND_LEN�ֽ�
void u4_printf(char* fmt,...)  
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART4_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART3_TX_BUF);//�˴η������ݵĳ���
		for(j=0;j<i;j++)//ѭ����������
		{
				while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
				USART_SendData(UART4,(uint8_t)USART4_TX_BUF[j]); 	 //�������ݵ�����4 
		}
}

void uart6_init(u32 bound)
{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //ʹ��GPIOGʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��

		//����1��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART1); //GPIOG9����ΪUSART6
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART1); //GPIOG14����ΪUSART6

		//USART6�˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9��GPIOG14
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOG,&GPIO_InitStructure); //��ʼ��PA9��PA10

		//USART6 ��ʼ������
		USART_InitStructure.USART_BaudRate = bound;//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
		USART_Init(USART6, &USART_InitStructure); //��ʼ������6

		USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���6

		//USART_ClearFlag(USART6, USART_FLAG_TC);

		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�

		//Usart1 NVIC ����
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����6�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

void USART6_IRQHandler(void)                	//����6�жϷ������
{
		u8 Res;
		if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //�����ж�
		{
				Res =USART_ReceiveData(USART6);//(USART6->DR);	//��ȡ���յ�������
		}
}

//����6printf ����
//ȷ��һ�η������ݲ�����USART6_SEND_LEN�ֽ�
void u6_printf(char* fmt,...)  
{
		u16 i,j;
		va_list ap;
		va_start(ap,fmt);
		vsprintf((char*)USART6_TX_BUF,fmt,ap);
		va_end(ap);
		i=strlen((const char*)USART6_TX_BUF);//�˴η������ݵĳ���
		for(j=0;j<i;j++)//ѭ����������
		{
				while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
				USART_SendData(USART6,(uint8_t)USART6_TX_BUF[j]); 	 //�������ݵ�����6
		}
}




