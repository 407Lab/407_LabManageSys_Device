#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h" 		 	 
#include "lcd.h"  
#include "key.h"     
#include "usmart.h" 
#include "malloc.h"
#include "sdio_sdcard.h"    
#include "ff.h"  
//#include "exfuns.h"    
 

 int main(void)
 {	 
 	u32 total,free;
	u8 t=0;	
  u8 key;
	FIL *fp;
	FATFS *fs;	
	 FRESULT res;
	 BYTE buff[4096]="name    number     ";
	 UINT btw,bw;

	delay_init();	    	 //��ʱ������ʼ��	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
 	usmart_dev.init(72);		//��ʼ��USMART		
 	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD   
 	//my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
 	POINT_COLOR=BLUE;			//��������Ϊ��ɫ 
  
	while(SD_Init())//��ⲻ��SD��
	{
		LCD_ShowString(30,150,200,16,16,"SD Card Error!");
		delay_ms(500);					
		LCD_ShowString(30,150,200,16,16,"Please Check! ");
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	}

 //	exfuns_init();							//Ϊfatfs��ر��������ڴ�				 
  f_mount(fs,"0:",1); 					//����SD�� 
	res=f_mkdir("menjin");        //����һ���ԡ�menjin�����ļ���
	if(res)
		printf("%s\r\n",res);
	res=f_open(fp,"/menjin/member.txt",FA_CREATE_NEW |FA_WRITE );     //��"menjin"�ļ��������½���member.txt���ı��ĵ�
	if(FR_OK==res)
	{
		res=f_write(fp,buff,btw,&bw);
	
	
		
		f_close(fp);
		
	}
	
	
	
	
//	LCD_ShowString(30,130,200,16,16,);
//	LCD_ShowString(30,130,200,16,16,);
LCD_ShowString(30,130,200,16,16,buff);

		
	
}
















//}
