#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板
//按键驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//按键初始化函数
void KEY_Init(void) //IO初始化
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
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY3按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY2==0||KEY1==0||WK_UP==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY0==0)return KEY0_PRES;
		else if(KEY1==0)return KEY1_PRES;
		else if(KEY2==0)return KEY2_PRES;
		else if(WK_UP==1)return WKUP_PRES;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1; 	    
 	return 0;// 无按键按下
}
//int KEY3_Scan(void) //触摸按键
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
//不支持连续按
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

