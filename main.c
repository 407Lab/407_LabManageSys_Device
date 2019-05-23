#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "key.h"
#include "malloc.h"  
#include "MMC_SD.h"   

u8 t=0;	 

void SD_Read_Sectorx(u32 sec)
{
	u8 *buf;
	u16 i;
	buf=mymalloc(512);				//申请内存
	if(SD_ReadDisk(buf,sec,1)==0)	//读取0扇区的内容
	{	
		LCD_ShowString(60,190,200,16,16,"USART1 Sending Data...");
		printf("SECTOR 0 DATA:\r\n");
		for(i=0;i<512;i++)printf("%x ",buf[i]);//打印sec扇区数据    	   
		printf("\r\nDATA ENDED\r\n");
		LCD_ShowString(60,190,200,16,16,"USART1 Send Data Over!");
	}
	myfree(buf);//释放内存	
}

 int main(void)
 { 
	u8 key;		 
	u32 sd_size;
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	uart_init(9600);	 	//串口初始化为9600
	LED_Init();		  		//初始化与LED连接的硬件接口
	LCD_Init();			   	//初始化LCD	
 	KEY_Init();				//按键初始化  
 	mem_init();				//初始化内存池	 
 	POINT_COLOR=BLUE;//设置字体为蓝色 
	LCD_ShowString(60,50,200,16,16,"Mini STM32");	
	LCD_ShowString(60,70,200,16,16,"SD CARD TEST");	
	LCD_ShowString(60,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(60,110,200,16,16,"2019");  
	LCD_ShowString(60,130,200,16,16,"KEY1:Read Sector 0");	   
 	while(SD_Initialize())//检测不到SD卡
	{
		LCD_ShowString(60,150,200,16,16,"SD Card Error!");
		delay_ms(300);					
		LCD_ShowString(60,150,200,16,16,"Please Check! ");
		delay_ms(300);
		LED0=!LED0;//DS0闪烁
	}
 	POINT_COLOR=GREEN;//设置字体为绿色 
	//检测SD卡成功 											    
	LCD_ShowString(60,150,200,16,16,"SD Card OK    ");
	LCD_ShowString(60,170,200,16,16,"SD Card Size:     MB");
	sd_size=SD_GetSectorCount();//得到扇区数
	LCD_ShowNum(164,170,sd_size>>11,5,16);//显示SD卡容量
	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY1_PRES)SD_Read_Sectorx(0);//KEY1按下,读取SD卡扇区0的内容
		t++;
		delay_ms(10);
//		delay_ms(1000);
//		delay_ms(1000);
		if(t==20)
		{
			LED0=!LED0;
			t=0;
		}
	}    	   
}