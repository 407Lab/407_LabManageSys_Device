#include "stm32f10x.h"
#include "rfid.h"
#include "lcd.h"
#include "key.h"
#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
/************************************************
 ALIENTEK 精英STM32F103开发板实验0
 工程模板
 注意，这是手册中的新建工程章节使用的main文件 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


extern char Card_flag;			//内读卡器检测到卡标志  0 ：无，1：有
//extern unsigned char SN;

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
	  if(KEY0==0||KEY1==0)//进入用户录入方式
	  {
			LCD_Clear(WHITE);    //清屏
		  POINT_COLOR=BLUE;
		  LCD_ShowString(30,70,210,24,24,"ccc欢迎进入录入模式");		
				
		  ReadID();  //读卡
			if(Card_flag==1)	//如果检测到IC卡
		  {
		  	POINT_COLOR=BLUE;
			  LCD_ShowString(30,100,210,24,24,"*****************");			
				delay_ms(500);
			  POINT_COLOR=BLUE;
			  LCD_ShowString(30,130,200,16,16,"111111111111");
				LCD_ShowString(156,130,200,16,16,(u8 *)num);
		  	LED1=~LED1;
        Card_flag=0;      //内刷卡清零				
				delay_ms(500);
				
				
				ReadID();  //读卡
				if(Card_flag==1)  //第二次录卡
		    {				
			    POINT_COLOR=BLUE;
		      LCD_ShowString(30,150,180,16,16,"22222222222");
			    LCD_ShowString(156,150,200,16,16,(u8 *)num);
          Card_flag=0;      //内刷卡清零     				
		      delay_ms(500);
	      }    
		  }
		
//		  if(RFID_ICID==RFID_ID)
//		  {
//				LCD_Clear(YELLOW);
//		  	POINT_COLOR=BLUE;
//		    LCD_ShowString(30,180,200,16,16,"MMM注册成功");	
//		  }
//	    else
//		  {

//		  	POINT_COLOR=BLUE;
//			  LCD_ShowString(30,210,200,16,16,"LLL两次刷卡信息不同，请重试");
//		  }
	   }

    else delay_ms(10);

	}
		
}


