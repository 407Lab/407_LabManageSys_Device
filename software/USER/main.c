/******************************************************************************
* @ File name --> main.c
* @ Author    --> By@ 鲁亮
* @ Version   --> V1.0
* @ Date      --> 07 - 16 - 2019
* @ Brief     --> 系统主函数文件
*
* @ Copyright (C) 2019
* @ All rights reserved	www.cqutlab.club
*******************************************************************************
*
*                                  File Update
* @ Version   --> V1.0
* @ Author    --> By@鲁亮
* @ Date      --> 07 - 16 - 2019
* @ Revise    --> 新建文件
*
* @ Version   --> V1.0
* @ Author    --> By@鲁亮
* @ Date      --> 08 - 11 - 2019
* @ Revise    --> 新建测试工程，检测基础模块
*
******************************************************************************/

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "beep.h"
#include "led.h"
#include "lcd.h"
#include "hc05.h"
#include "timer.h"
#include "rc522.h"
#include "sdio_sdcard.h"
#include "ff.h"
#include "exfuns.h"
#include "ds1302.h"

FIL fdsts;

void Create_file(char *filename);
void Read_file(char *filename);

int main(void)
{
		u8 time[15];
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
		delay_init(168);      //初始化延时函数
		usart_init(115200);		//初始化串口波特率为115200
		
		LED_Init();					  //初始化LED
		BEEP_Init();
		LCD_Init();           //初始化LCD FSMC接口
		HC05_Init();
		InitRc522();
		POINT_COLOR=RED;      //画笔颜色：红色
		POINT_COLOR=RED;
		LCD_ShowString(30,40,210,24,24,(u8 *)"Smart Lab");	
		LCD_ShowString(30,70,200,16,16,(u8 *)"407Lab");
		LCD_ShowString(30,90,200,16,16,(u8 *)"www.cqutlab.club");    					 
		LCD_ShowString(30,110,200,12,12,(u8 *)"2019/07/16");
		printf("初始化完成！！！\r\n");
		
		SD_Init();						//SD卡驱动初始化
		exfuns_init();				//为fatfs相关变量申请内存
		f_mount(fs[0],"0:",1); 		//挂载SD卡
	
		DS1302_Init();
		delay_ms(10);
		DS1302_Write_Time();
		
//		Create_file("132.txt");
//		Read_file("132.txt");
	
		while(1)
		{
//				ReadID();
//				DS1302_Get_Time(time);
//				printf("%d%d%d%d年%d%d月%d%d日 ", time[0],time[1],time[2],time[3],time[4],time[5],time[6],time[7]);
//				printf("%d%d时%d%d分%d%d秒 星期%d\n", time[9],time[10],time[11],time[12],time[13],time[14],time[8]);
				LED1=!LED1;
				LED2=!LED2;
				LED3=!LED3;
//				BEEP=1;
				delay_ms(200);
//				BEEP=0;
				delay_ms(200);
				u3_printf("test bluetooth\r\n");
		}
}

void Create_file(char *filename)
{
		UINT bw;
		char test[10] = "123456";
		if(f_open(&fdsts,filename,FA_CREATE_NEW|FA_WRITE)==FR_OK)
		{
				printf("写入文件!\r\n");
				f_write(&fdsts,test,6,&bw);
				f_close(&fdsts);
				printf("写入成功\r\n");
		}
}

void Read_file(char *filename)
{
		UINT bw;
		char data_buf[15] = {0};
		if(f_open(&fdsts,filename,FA_READ)==FR_OK)
		{
				printf("读取文件！\r\n");
				f_read(&fdsts,data_buf,6,&bw);
				f_close(&fdsts);
				printf("读取成功！\r\n");
				printf("读取数据为：%s\r\n",data_buf);
		}
}
