#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"  
#include "sram.h"   
#include "malloc.h" 
#include "usmart.h"  
#include "sdio_sdcard.h"    
#include "malloc.h" 
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h"    
#include "fontupd.h"
#include "text.h"	
#include "piclib.h"	
#include "string.h"	
#include "math.h"	

u16 pic_get_tnum(u8 *path)
{	  
	u8 res;
	u16 rval=0;
 	DIR tdir;	 		//��ʱĿ¼
	FILINFO tfileinfo;	//��ʱ�ļ���Ϣ	
	u8 *fn;	 			 			   			     
  res=f_opendir(&tdir,(const TCHAR*)path); 	//��Ŀ¼
  tfileinfo.lfsize=_MAX_LFN*2+1;				//���ļ�����󳤶�
	tfileinfo.lfname=mymalloc(SRAMIN,tfileinfo.lfsize);//Ϊ���ļ������������ڴ�
	if(res==FR_OK&&tfileinfo.lfname!=NULL)
	{
		while(1)//��ѯ�ܵ���Ч�ļ���
		{
	        res=f_readdir(&tdir,&tfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
	        if(res!=FR_OK||tfileinfo.fname[0]==0)
						break;	//������/��ĩβ��,�˳�		  
     		fn=(u8*)(*tfileinfo.lfname?tfileinfo.lfname:tfileinfo.fname);			 
			res=f_typetell(fn);	
			if((res&0XF0)==0X50)//ȡ����λ,�����ǲ���ͼƬ�ļ�	
			{
				rval++;//��Ч�ļ�������1
			}	    
		}  
	} 
	return rval;
}


int main(void)
{        
	u8 *bname;//��·���ı���ͼƬ���ļ���
	DIR bpicdir;//����ͼƬ��ͼƬĿ¼
	u16 bcurindex;//����ͼƬ�ĵ�ǰ����
	
	u8 res;
 	DIR picdir;	 		//ͷ��ͼƬĿ¼
	FILINFO picfileinfo;//ͷ���ļ���Ϣ
	u8 *fn;   			//���ļ���
	u8 *pname;			//��·�����ļ�����ͷ��
	u16 totpicnum; 		//ͼƬ�ļ�����
	u16 curindex;		//ͼƬ��ǰ����
	u8 key;				//��ֵ
	u8 pause=0;			//��ͣ���
	u8 t;
	u16 temp;
	u16 *picindextbl;	//ͼƬ������ ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
	usmart_dev.init(84);		//��ʼ��USMART
 	LCD_Init();					//LCD��ʼ��  
	LCD_DrawRectangle(0,0,0,0);
	LCD_Fill(0,0,0,0,0);
	LCD_Color_Fill(0,0,0,0,0);
 	KEY_Init();					//������ʼ��  
	W25QXX_Init();				//��ʼ��W25Q128
	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ�� 
	my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ�� 
	exfuns_init();			//Ϊfatfs��ر��������ڴ�  
  f_mount(fs[0],"0:",1); 	//����SD�� 
 	f_mount(fs[1],"1:",1); 	//����FLASH.
	POINT_COLOR=RED;      
	while(font_init()) 		//����ֿ�
	{	    
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}  	 
 	Show_Str(30,50,200,16,"Explorer STM32F4������",16,0);				    	 
	Show_Str(30,70,200,16,"ͼƬ��ʾ����",16,0);				    	 
	Show_Str(30,90,200,16,"KEY0:NEXT KEY1:PREV",16,0);				    	 
	Show_Str(30,110,200,16,"WK_UP:PAUSE",16,0);				    	 
	
	
	while(f_opendir(&bpicdir,"0:/Hand_PICTURE"))//�򿪱���ͼƬ�ļ���
 	{	    
		Show_Str(30,170,240,16,"Hand_PICTURE�ļ��д���!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}  
	totpicnum=pic_get_tnum("0:/Hand_PICTURE"); //�õ�����Ч�ļ���
  	while(totpicnum==NULL)//ͼƬ�ļ�Ϊ0		
 	{	    
		Show_Str(30,170,240,16,"û��ͼƬ�ļ�!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}
  picfileinfo.lfsize=_MAX_LFN*2+1;						//���ļ�����󳤶�
	picfileinfo.lfname=mymalloc(SRAMIN,picfileinfo.lfsize);	//Ϊ���ļ������������ڴ�
 	bname=mymalloc(SRAMIN,picfileinfo.lfsize);				//Ϊ��·�����ļ��������ڴ�
 	picindextbl=mymalloc(SRAMIN,2*totpicnum);				//����2*totpicnum���ֽڵ��ڴ�,���ڴ��ͼƬ����
 	while(picfileinfo.lfname==NULL||bname==NULL||picindextbl==NULL)//�ڴ�������
 	{	    
		Show_Str(30,170,240,16,"�ڴ����ʧ��!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}  	
	//��¼����
    res=f_opendir(&bpicdir,"0:/Hand_PICTURE"); //��Ŀ¼
	if(res==FR_OK)
	{
		bcurindex=0;//��ǰ����Ϊ0
		while(1)//ȫ����ѯһ��
		{
			temp=bpicdir.index;								//��¼��ǰindex
	    res=f_readdir(&bpicdir,&picfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
	        if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//������/��ĩβ��,�˳�		  
     		fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
			res=f_typetell(fn);	
			if((res&0XF0)==0X50)//ȡ����λ,�����ǲ���ͼƬ�ļ�	
			{
				picindextbl[bcurindex]=temp;//��¼����
				bcurindex++;
			}	    
		} 
	}   
	Show_Str(30,170,240,16,"��ʼ��ʾ...",16,0); 
	delay_ms(1500);
	piclib_init();										//��ʼ����ͼ	   	   
	bcurindex=0;											//��0��ʼ��ʾ
   	res=f_opendir(&bpicdir,(const TCHAR*)"0:/Hand_PICTURE"); 	//��Ŀ¼
	while(res==FR_OK)//�򿪳ɹ�
	{	
	  dir_sdi(&bpicdir,picindextbl[bcurindex]);			//�ı䵱ǰĿ¼����	   
    res=f_readdir(&bpicdir,&picfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
    if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//������/��ĩβ��,�˳�
    fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
		strcpy((char*)bname,"0:/Hand_PICTURE/");				//����·��(Ŀ¼)
		strcat((char*)bname,(const char*)fn);  			//���ļ������ں���
 		LCD_Clear(BLACK);
	
	while(f_opendir(&picdir,"0:/PICTURE"))//��ͷ��ͼƬ�ļ���
 	{	    
		Show_Str(30,170,240,16,"PICTURE�ļ��д���!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}  
	totpicnum=pic_get_tnum("0:/PICTURE"); //�õ�����Ч�ļ���

  picfileinfo.lfsize=_MAX_LFN*2+1;						//���ļ�����󳤶�
	picfileinfo.lfname=mymalloc(SRAMIN,picfileinfo.lfsize);	//Ϊ���ļ������������ڴ�
 	pname=mymalloc(SRAMIN,picfileinfo.lfsize);				//Ϊ��·���ı����ļ��������ڴ�
 	picindextbl=mymalloc(SRAMIN,2*totpicnum);				//����2*totpicnum���ֽڵ��ڴ�,���ڴ��ͼƬ����
 	while(picfileinfo.lfname==NULL||pname==NULL||picindextbl==NULL)//�ڴ�������
 	{	    
		Show_Str(30,170,240,16,"�ڴ����ʧ��!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}  	
	//��¼����
  res=f_opendir(&picdir,"0:/PICTURE"); //��Ŀ¼
	if(res==FR_OK)
	{
		curindex=0;//��ǰ����Ϊ0
		while(1)//ȫ����ѯһ��
		{
			temp=picdir.index;								//��¼��ǰindex
	        res=f_readdir(&picdir,&picfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
	        if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//������/��ĩβ��,�˳�		  
     		fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
			res=f_typetell(fn);	
			if((res&0XF0)==0X50)//ȡ����λ,�����ǲ���ͼƬ�ļ�	
			{
				picindextbl[curindex]=temp;//��¼����
				curindex++;
			}	    
		} 
	}     	   
	curindex=0;											//��0��ʼ��ʾ
  res=f_opendir(&picdir,(const TCHAR*)"0:/PICTURE"); 	//��Ŀ¼
	while(res==FR_OK)//�򿪳ɹ�
	{	
		dir_sdi(&picdir,picindextbl[curindex]);			//�ı䵱ǰĿ¼����	   
    res=f_readdir(&picdir,&picfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
    if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//������/��ĩβ��,�˳�
    fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
		strcpy((char*)pname,"0:/PICTURE/");				//����·��(Ŀ¼)
		strcat((char*)pname,(const char*)fn);  			//���ļ������ں���
 		LCD_Clear(BLACK);
		ai_load_picfile( bname,0,0,lcddev.width,lcddev.height,1);//��ʾ����ͼƬ   
	  ai_load_picfile(pname,300,30,150,200,1);//��ʾͷ��ͼƬ

	  Show_Str(30,100,240,24,"407ʵ����",24,0);//��ʾʵ��������
		LCD_Fill(90,170,150,190,BROWN);//�����ĵ�ɫ
		LCD_Fill(90,210,250,230,BROWN);//ѧ�ŵĵ�ɫ
		
		POINT_COLOR=BLUE;//��������Ϊ��ɫ 
	 Show_Str(30,170,240,24,"����",24,0);
	 LCD_DrawRectangle(90,170,150,190);//�������淽��Ĵ�С
   Show_Str(30,210,240,24,"ѧ��",24,0);		
	 LCD_DrawRectangle(90,210,250,230);//ѧ�ź��淽��Ĵ�С
	 LCD_DrawRectangle(300,30,450,230);//ͷ�񷽿�Ĵ�С
		//t=0;
		while(1) 
		{
			key=KEY_Scan(0);		//ɨ�谴��
			if(t>250)key=1;			//ģ��һ�ΰ���KEY0    
			if((t%20)==0)LED0=!LED0;//LED0��˸,��ʾ������������.
			if(key==KEY1_PRES)		//��һ��
			{
				if(curindex)curindex--;
				else curindex=totpicnum-1;
				break;
			}else if(key==KEY0_PRES)//��һ��
			{
				curindex++;		   	
				if(curindex>=totpicnum)curindex=0;//��ĩβ��ʱ��,�Զ���ͷ��ʼ
				break;
			}else if(key==WKUP_PRES)
			{
				pause=!pause;
				LED1=!pause; 	//��ͣ��ʱ��LED1��.  
			}
			delay_ms(10); 
		}					    
		res=0;  
	} 											  
	myfree(SRAMIN,picfileinfo.lfname);	//�ͷ��ڴ�			    
	myfree(SRAMIN,pname);				//�ͷ��ڴ�			    
	myfree(SRAMIN,picindextbl);			//�ͷ��ڴ�		
}
}
