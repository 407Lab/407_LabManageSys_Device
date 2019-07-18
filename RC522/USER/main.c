#include "stm32f10x.h"
#include "rfid.h"
#include "lcd.h"
#include "key.h"
#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"


extern char Card_flag;			//内读卡器检测到卡标志  0 ：无，1：有
//extern unsigned char SN;
extern char num[9];
int main(void)
{
	delay_init();	    							 //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //设置NVIC中断分组2:2位抢占优先级，2位响应优先级	
  LED_Init();                      //初始化LED
	uart_init(115200);	 	//串口初始化为115200
  LCD_Init();				//LCD初始化	 
	KEY_Init();       //KEY初始化
	InitRC522();
	POINT_COLOR=RED;
	LCD_ShowString(30,40,210,24,24,"cccFFFF");
	delay_ms(1000);
  while(1)
  {
		if(KEY0==0)//进入用户录入方式
	  {
			GPIO_ResetBits(GPIOE,GPIO_Pin_4);
			LCD_Clear(WHITE);    //清屏
		  POINT_COLOR=BLUE;
		  LCD_ShowString(30,70,170,12,16,"welcome to entry mode");	
			delay_ms(500);
			Compare_ID();	//对比两个卡号是否相同			
	  }
    else delay_ms(1);
		
	}
}

