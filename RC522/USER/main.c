#include "stm32f10x.h"
#include "rfid.h"
#include "lcd.h"
#include "key.h"
#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"


extern char Card_flag;			//�ڶ�������⵽����־  0 ���ޣ�1����
//extern unsigned char SN;
extern char num[9];
int main(void)
{
	delay_init();	    							 //��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�	
  LED_Init();                      //��ʼ��LED
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
  LCD_Init();				//LCD��ʼ��	 
	KEY_Init();       //KEY��ʼ��
	InitRC522();
	POINT_COLOR=RED;
	LCD_ShowString(30,40,210,24,24,"cccFFFF");
	delay_ms(1000);
  while(1)
  {
		if(KEY0==0)//�����û�¼�뷽ʽ
	  {
			GPIO_ResetBits(GPIOE,GPIO_Pin_4);
			LCD_Clear(WHITE);    //����
		  POINT_COLOR=BLUE;
		  LCD_ShowString(30,70,170,12,16,"welcome to entry mode");	
			delay_ms(500);
			Compare_ID();	//�Ա����������Ƿ���ͬ			
	  }
    else delay_ms(1);
		
	}
}

