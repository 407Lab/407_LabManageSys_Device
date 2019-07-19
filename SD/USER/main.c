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

	delay_init();	    	 //延时函数初始化	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
 	usmart_dev.init(72);		//初始化USMART		
 	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD   
 	//my_mem_init(SRAMIN);		//初始化内部内存池
 	POINT_COLOR=BLUE;			//设置字体为红色 
  
	while(SD_Init())//检测不到SD卡
	{
		LCD_ShowString(30,150,200,16,16,"SD Card Error!");
		delay_ms(500);					
		LCD_ShowString(30,150,200,16,16,"Please Check! ");
		delay_ms(500);
		LED0=!LED0;//DS0闪烁
	}

 //	exfuns_init();							//为fatfs相关变量申请内存				 
  f_mount(fs,"0:",1); 					//挂载SD卡 
	res=f_mkdir("menjin");        //建立一个以“menjin”的文件夹
	if(res)
		printf("%s\r\n",res);
	res=f_open(fp,"/menjin/member.txt",FA_CREATE_NEW |FA_WRITE );     //在"menjin"文件夹下面新建“member.txt”文本文档
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
