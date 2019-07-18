#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK��ӢSTM32������
//������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//������ʼ������
void KEY_Init(void) //IO��ʼ��
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;//KEY0||KEY1||KEY2
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
}
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY3���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY2==0||KEY1==0||WK_UP==1))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY0==0)return KEY0_PRES;
		else if(KEY1==0)return KEY1_PRES;
		else if(KEY2==0)return KEY2_PRES;
		else if(WK_UP==1)return WKUP_PRES;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1; 	    
 	return 0;// �ް�������
}
//int KEY3_Scan(void) //��������
//{
//	static int flag;
//if(KEY3  == 1&&flag==0)
//		{
//			delay_ms(50);		  							
//			flag=1;			
//		}else{
//		        if(KEY3!=1)
//						{
//							flag=0;							
//						}
//				 }
//		return flag;
//}
///****************************************************
//��֧��������
//****************************************************/
//int Key_Test(void)
//{
//	static u8 key_value=0,flag=1;
//	//OLED_ShowString(35,-1,"Test Key");
//	if((KEY1==0||KEY2==0||KEY3==0||WK_UP==1)&&flag==1)
//	{
//		delay_ms(5);
//		flag=0;
//		if(KEY1==0)   key_value=KEY1_PRES;
//		if(KEY2==0)   key_value=KEY2_PRES;
//		if(KEY3==0)   key_value=KEY3_PRES;
//		//if(KEY4==1)   key_value=KEY4_PRES;
//		if(WK_UP==1)  key_value=WKUP_PRES;
//	}
//else 
//			{
//				if(KEY1==1&&KEY2==1&&KEY3==1&&WK_UP==0)
//				{
//				   flag=1;
//					 key_value=0;
//				}
//			}			
//  return  key_value;	
//}

