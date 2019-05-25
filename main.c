#include "stm32f10x.h"
#include "rfid.h"
#include "lcd.h"
#include "key.h"
#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
/************************************************
 ALIENTEK ��ӢSTM32F103������ʵ��0
 ����ģ��
 ע�⣬�����ֲ��е��½������½�ʹ�õ�main�ļ� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


extern char Card_flag;			//�ڶ�������⵽����־  0 ���ޣ�1����
//extern unsigned char SN;

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
	  if(KEY0==0||KEY1==0)//�����û�¼�뷽ʽ
	  {
			LCD_Clear(WHITE);    //����
		  POINT_COLOR=BLUE;
		  LCD_ShowString(30,70,210,24,24,"ccc��ӭ����¼��ģʽ");		
				
		  ReadID();  //����
			if(Card_flag==1)	//�����⵽IC��
		  {
		  	POINT_COLOR=BLUE;
			  LCD_ShowString(30,100,210,24,24,"*****************");			
				delay_ms(500);
			  POINT_COLOR=BLUE;
			  LCD_ShowString(30,130,200,16,16,"111111111111");
				LCD_ShowString(156,130,200,16,16,(u8 *)num);
		  	LED1=~LED1;
        Card_flag=0;      //��ˢ������				
				delay_ms(500);
				
				
				ReadID();  //����
				if(Card_flag==1)  //�ڶ���¼��
		    {				
			    POINT_COLOR=BLUE;
		      LCD_ShowString(30,150,180,16,16,"22222222222");
			    LCD_ShowString(156,150,200,16,16,(u8 *)num);
          Card_flag=0;      //��ˢ������     				
		      delay_ms(500);
	      }    
		  }
		
//		  if(RFID_ICID==RFID_ID)
//		  {
//				LCD_Clear(YELLOW);
//		  	POINT_COLOR=BLUE;
//		    LCD_ShowString(30,180,200,16,16,"MMMע��ɹ�");	
//		  }
//	    else
//		  {

//		  	POINT_COLOR=BLUE;
//			  LCD_ShowString(30,210,200,16,16,"LLL����ˢ����Ϣ��ͬ��������");
//		  }
	   }

    else delay_ms(10);

	}
		
}


