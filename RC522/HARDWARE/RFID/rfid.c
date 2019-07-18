#include "sys.h"
#include "rfid.h"
#include "lcd.h"
#include "delay.h"
#include "string.h"			//字符串操作
/*******************************
*SPI1连线说明：         门内
*1--SS  <----->PA4			
*2--SCK <----->PA5
*3--MOSI<----->PA7
*4--MISO<----->PA6
*5--悬空
*6--GND <----->GND
*7--RST <----->PC4
*8--VCC <----->VCC
************************************/
/*******************************
*SPI3连线说明：         门外
*1--SS  <----->PG15			
*2--SCK <----->PB3
*3--MOSI<----->PB5
*4--MISO<----->PB4
*5--悬空
*6--GND <----->GND
*7--RST <----->PB6
*8--VCC <----->VCC·
************************************/
/*全局变量*/
char num[9]={0};
unsigned char CT[2];//卡类型
unsigned char SN[4]; //卡号
unsigned char P_CT[2];//卡类型
unsigned char P_SN[4]; //卡号
char Card_flag=0;			//内读卡器检测到卡标志  0 ：无，1：有
char P_Card_flag=0;			//外读卡器检测到卡标志  0 ：无，1：有


void delay_ns(u32 ns)
{
  u32 i;
  for(i=0;i<ns;i++)
  {
    __nop();
    __nop();
    __nop();
  }
}

u8 SPIWriteByte(u8 Byte)
{
	while((SPI1->SR&0X02)==0);		//等待发送区空	  
	SPI1->DR=Byte;	 	            //发送一个byte   
	while((SPI1->SR&0X01)==0);      //等待接收完一个byte  
	return SPI1->DR;          	    //返回收到的数据			
}

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}


//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频  
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1&=0XFFC7;
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI2速度 
	SPI_Cmd(SPI1,ENABLE); 

} 




void SPI1_Init(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;
 	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_SPI1, ENABLE );//PORTB时钟使能 


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	 //IO-->	PC4  NSS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化PF0、PF1
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	 //IO-->PC4  RST					PA4  NSS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化PF0、PF1
    GPIO_ResetBits(GPIOC,GPIO_Pin_4);			             //PC4输出低

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOB

 	GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7);  

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第一个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
 	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	
	//SPI2_ReadWriteByte(0xff);//启动传输	
}

void InitRC522(void)
{
  SPI1_Init();               //门内
  PcdReset();               //功能：复位RC522
  PcdAntennaOff();          //关闭天线
  delay_ms(2);               
  PcdAntennaOn();           //开启天线
  M500PcdConfigISOType( 'A' );
}
void Reset_RC522(void)
{
  PcdReset();              //功能：复位RC522 
  PcdAntennaOff();         //关闭天线
  delay_ms(2);             
  PcdAntennaOn();          
}                         
/////////////////////////////////////////////////////////////////////
//功    能：寻卡
//参数说明: req_code[IN]:寻卡方式
//                0x52 = 寻感应区内所有符合14443A标准的卡
//                0x26 = 寻未进入休眠状态的卡
//         		  pTagType[OUT]：卡片类型代码
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(u8   req_code,u8 *pTagType)
{
	char   status;  
	u8   unLen;
	u8   ucComMF522Buf[MAXRLEN]; 

	ClearBitMask(Status2Reg,0x08);
	WriteRawRC(BitFramingReg,0x07);
	SetBitMask(TxControlReg,0x03);
 
	ucComMF522Buf[0] = req_code;

	status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);

	if ((status == MI_OK) && (unLen == 0x10))
	{    
		*pTagType     = ucComMF522Buf[0];
		*(pTagType+1) = ucComMF522Buf[1];
	}
	else
	{   status = MI_ERR;   }
   
	return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////  
char PcdAnticoll(u8 *pSnr)
{
    char   status;
    u8   i,snr_check=0;
    u8   unLen;
    u8   ucComMF522Buf[MAXRLEN]; 
    

    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }
    
    SetBitMask(CollReg,0x80);
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(u8 *pSnr)
{
    char   status;
    u8   i;
    u8   unLen;
    u8   ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥 
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////               
char PcdAuthState(u8   auth_mode,u8   addr,u8 *pKey,u8 *pSnr)
{
    char   status;
    u8   unLen;
    u8   ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
//    for (i=0; i<6; i++)
//    {    ucComMF522Buf[i+2] = *(pKey+i);   }
//    for (i=0; i<6; i++)
//    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
    memcpy(&ucComMF522Buf[2], pKey, 6); 
    memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          p [OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
char PcdRead(u8   addr,u8 *p )
{
    char   status;
    u8   unLen;
    u8   i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
 //   {   memcpy(p , ucComMF522Buf, 16);   }
    {
        for (i=0; i<16; i++)
        {    *(p +i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          p [IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                  
char PcdWrite(u8   addr,u8 *p )
{
    char   status;
    u8   unLen;
    u8   i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, p , 16);
        for (i=0; i<16; i++)
        {    
        	ucComMF522Buf[i] = *(p +i);   
        }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdHalt(void)
{
    u8   status;
    u8   unLen;
    u8   ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//用MF522计算CRC16函数
/////////////////////////////////////////////////////////////////////
void CalulateCRC(u8 *pIn ,u8   len,u8 *pOut )
{
    u8   i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIn +i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOut [0] = ReadRawRC(CRCResultRegL);
    pOut [1] = ReadRawRC(CRCResultRegM);
}

/////////////////////////////////////////////////////////////////////
//功    能：复位RC522
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdReset(void)
{
	//PORTD|=(1<<RC522RST);
	SET_RC522RST;
    delay_ns(10);
	//PORTD&=~(1<<RC522RST);
	CLR_RC522RST;
    delay_ns(10);
	//PORTD|=(1<<RC522RST);
	SET_RC522RST;
    delay_ns(10);
    WriteRawRC(CommandReg,PCD_RESETPHASE);
	WriteRawRC(CommandReg,PCD_RESETPHASE);
    delay_ns(10);
    
    WriteRawRC(ModeReg,0x3D);            //和Mifare卡通讯，CRC初始值0x6363
    WriteRawRC(TReloadRegL,30);           
    WriteRawRC(TReloadRegH,0);
    WriteRawRC(TModeReg,0x8D);
    WriteRawRC(TPrescalerReg,0x3E);
	
	WriteRawRC(TxAutoReg,0x40);//必须要
   
    return MI_OK;
}
//////////////////////////////////////////////////////////////////////
//设置RC632的工作方式 
//////////////////////////////////////////////////////////////////////
char M500PcdConfigISOType(u8   type)
{
   if (type == 'A')                     //ISO14443_A
   { 
       ClearBitMask(Status2Reg,0x08);
       WriteRawRC(ModeReg,0x3D);//3F
       WriteRawRC(RxSelReg,0x86);//84
       WriteRawRC(RFCfgReg,0x7F);   //4F
   	   WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
	   WriteRawRC(TReloadRegH,0);
       WriteRawRC(TModeReg,0x8D);
	   WriteRawRC(TPrescalerReg,0x3E);
	   delay_ns(1000);
       PcdAntennaOn();
   }
   else{ return 1; }
   
   return MI_OK;
}
/////////////////////////////////////////////////////////////////////
//功    能：读RC632寄存器
//参数说明：Address[IN]:寄存器地址
//返    回：读出的值
/////////////////////////////////////////////////////////////////////
u8 ReadRawRC(u8   Address)
{
    u8   ucAddr;
    u8   ucResult=0;
	CLR_SPI_CS;
    ucAddr = ((Address<<1)&0x7E)|0x80;
	
	SPIWriteByte(ucAddr);
	ucResult=SPIReadByte();
	SET_SPI_CS;
   return ucResult;
}

/////////////////////////////////////////////////////////////////////
//功    能：写RC632寄存器
//参数说明：Address[IN]:寄存器地址
//          value[IN]:写入的值
/////////////////////////////////////////////////////////////////////
void WriteRawRC(u8   Address, u8   value)
{  
    u8   ucAddr;
//	u8 tmp;

	CLR_SPI_CS;
    ucAddr = ((Address<<1)&0x7E);

	SPIWriteByte(ucAddr);
	SPIWriteByte(value);
	SET_SPI_CS;

//	tmp=ReadRawRC(Address);
//
//	if(value!=tmp)
//		printf("wrong\n");
}
/////////////////////////////////////////////////////////////////////
//功    能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:置位值
/////////////////////////////////////////////////////////////////////
void SetBitMask(u8   reg,u8   mask)  
{
    char   tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//功    能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:清位值
/////////////////////////////////////////////////////////////////////
void ClearBitMask(u8   reg,u8   mask)  
{
    char   tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pIn [IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOut [OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
char PcdComMF522(u8   Command, 
                 u8 *pIn , 
                 u8   InLenByte,
                 u8 *pOut , 
                 u8 *pOutLenBit)
{
    char   status = MI_ERR;
    u8   irqEn   = 0x00;
    u8   waitFor = 0x00;
    u8   lastBits;
    u8   n;
    u16   i;
    switch (Command)
    {
        case PCD_AUTHENT:
			irqEn   = 0x12;
			waitFor = 0x10;
			break;
		case PCD_TRANSCEIVE:
			irqEn   = 0x77;
			waitFor = 0x30;
			break;
		default:
			break;
    }
   
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);	//清所有中断位
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);	 	//清FIFO缓存
    
    for (i=0; i<InLenByte; i++)
    {   WriteRawRC(FIFODataReg, pIn [i]);    }
    WriteRawRC(CommandReg, Command);	  
//   	 n = ReadRawRC(CommandReg);
    
    if (Command == PCD_TRANSCEIVE)
    {    SetBitMask(BitFramingReg,0x80);  }	 //开始传送
    										 
    //i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms
	i = 2000;
    do 
    {
        n = ReadRawRC(ComIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);

    if (i!=0)
    {    
        if(!(ReadRawRC(ErrorReg)&0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {   status = MI_NOTAGERR;   }
            if (Command == PCD_TRANSCEIVE)
            {
               	n = ReadRawRC(FIFOLevelReg);
              	lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOut [i] = ReadRawRC(FIFODataReg);    }
            }
        }
        else
        {   status = MI_ERR;   }
        
    }
   

    SetBitMask(ControlReg,0x80);           // stop timer now
    WriteRawRC(CommandReg,PCD_IDLE); 
    return status;
}

/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn(void)
{
    u8   i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}


/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff(void)
{
	ClearBitMask(TxControlReg, 0x03);
}



///**********************************************************************************************
//									P_RC522函数
//**********************************************************************************************/

///////////////////////////////////////////////////////////////////////////////////
///*********************************************************/
///*********************************************************/

//u8 P_SPIWriteByte(u8 Byte)
//{
//	while((SPI3->SR&0X02)==0);		//等待发送区空	  
//	SPI3->DR=Byte;	 	            //发送一个byte   
// 	while((SPI3->SR&0X01)==0);      //等待接收完一个byte  
//	return SPI3->DR;          	    //返回收到的数据			
//}
////SPIx 读写一个字节
////TxData:要写入的字节
////返回值:读取到的字节
//u8 P_SPI_ReadWriteByte(u8 TxData)
//{		
//	u8 retry=0;				 	
//	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
//	{			
//		retry++;
//		if(retry>200)return 0;
//	}			  
//	SPI_I2S_SendData(SPI3, TxData); //通过外设SPIx发送一个数据
//	retry=0;

//	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
//		{
//		retry++;
//		if(retry>200)return 0;
//		}	  						    
//	return SPI_I2S_ReceiveData(SPI3); //返回通过SPIx最近接收的数据					    
//}


////SPI 速度设置函数
////SpeedSet:
////SPI_BaudRatePrescaler_2   2分频   
////SPI_BaudRatePrescaler_8   8分频   
////SPI_BaudRatePrescaler_16  16分频  
////SPI_BaudRatePrescaler_256 256分频 
//  
//void P_SPI_SetSpeed(u8 SPI_BaudRatePrescaler)
//{
//  	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
//	SPI3->CR1&=0XFFC7;
//	SPI3->CR1|=SPI_BaudRatePrescaler;	//设置SPI3速度 
//	SPI_Cmd(SPI3,ENABLE); 
//} 

//void P_SPI_Init(void)	
//{
//	
//	#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */
//	


//	GPIO_InitTypeDef GPIO_InitStructure;
//	SPI_InitTypeDef  SPI_InitStructure;
//	
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);      /*??SWD ??JTAG*/
// 	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOG, ENABLE );//PORTB时钟使能 
//	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI3,  ENABLE );//SPI2时钟使能

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//IO-->PG15  NSS
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOG, &GPIO_InitStructure);					
//    
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	 //IO-->PB6  RST
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化PF0、PF1
//	GPIO_ResetBits(GPIOB,GPIO_Pin_6);			             //PF3输出低

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB3/4/5复用推挽输出 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB

// 	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);  //PB3/4/5上拉

//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为低电平
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第一个跳变沿（上升或下降）数据被采样
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
//	SPI_Init(SPI3, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
// 
//	SPI_Cmd(SPI3, ENABLE); //使能SPI外设	
//}

//void P_InitRC522(void)
//{
//  P_SPI_Init();             //门外
//  P_PcdReset();
//  P_PcdAntennaOff();
//  delay_ms(2);  
//  P_PcdAntennaOn();
//  P_M500PcdConfigISOType( 'A' );
//}
//void P_Reset_RC522(void)
//{
//  P_PcdReset();
//  P_PcdAntennaOff();
//  delay_ms(2);  
//  P_PcdAntennaOn();
//}                         
///////////////////////////////////////////////////////////////////////
////功    能：寻卡
////参数说明: req_code[IN]:寻卡方式
////                0x52 = 寻感应区内所有符合14443A标准的卡
////                0x26 = 寻未进入休眠状态的卡
////          pTagType[OUT]：卡片类型代码
////                0x4400 = Mifare_UltraLight
////                0x0400 = Mifare_One(S50)
////                0x0200 = Mifare_One(S70)
////                0x0800 = Mifare_Pro(X)
////                0x4403 = Mifare_DESFire
////返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////////
//char P_PcdRequest(u8   req_code,u8 *pTagType)
//{
//	char   status;  
//	u8   unLen;
//	u8   ucComMF522Buf[MAXRLEN]; 

//	P_ClearBitMask(Status2Reg,0x08);
//	P_WriteRawRC(BitFramingReg,0x07);
//	P_SetBitMask(TxControlReg,0x03); 
//	ucComMF522Buf[0] = req_code;
//	status = P_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
//	if ((status == MI_OK) && (unLen == 0x10))
//	{    
//		*pTagType     = ucComMF522Buf[0];
//		*(pTagType+1) = ucComMF522Buf[1];
//	}
//	else
//	{   status = MI_ERR;   }   
//	return status;
//}

///////////////////////////////////////////////////////////////////////
////功    能：防冲撞
////参数说明: pSnr[OUT]:卡片序列号，4字节
////返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////////  
//char P_PcdAnticoll(u8 *pSnr)
//{
//    char   status;
//    u8   i,snr_check=0;
//    u8   unLen;
//    u8   ucComMF522Buf[MAXRLEN];    

//    P_ClearBitMask(Status2Reg,0x08);
//    P_WriteRawRC(BitFramingReg,0x00);
//    P_ClearBitMask(CollReg,0x80); 
//    ucComMF522Buf[0] = PICC_ANTICOLL1;
//    ucComMF522Buf[1] = 0x20;
//    status = P_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);
//    if (status == MI_OK)
//    {
//    	 for (i=0; i<4; i++)
//         {   
//             *(pSnr+i)  = ucComMF522Buf[i];
//             snr_check ^= ucComMF522Buf[i];
//         }
//         if (snr_check != ucComMF522Buf[i])
//         {   status = MI_ERR;    }
//    }    
//    P_SetBitMask(CollReg,0x80);
//    return status;
//}

///////////////////////////////////////////////////////////////////////
////功    能：选定卡片
////参数说明: pSnr[IN]:卡片序列号，4字节
////返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////////
//char P_PcdSelect(u8 *pSnr)
//{
//    char   status;
//    u8   i;
//    u8   unLen;
//    u8   ucComMF522Buf[MAXRLEN]; 
//    
//    ucComMF522Buf[0] = PICC_ANTICOLL1;
//    ucComMF522Buf[1] = 0x70;
//    ucComMF522Buf[6] = 0;
//    for (i=0; i<4; i++)
//    {
//    	ucComMF522Buf[i+2] = *(pSnr+i);
//    	ucComMF522Buf[6]  ^= *(pSnr+i);
//    }
//    P_CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
//  
//    P_ClearBitMask(Status2Reg,0x08);

//    status = P_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
//    
//    if ((status == MI_OK) && (unLen == 0x18))
//    {   status = MI_OK;  }
//    else
//    {   status = MI_ERR;    }

//    return status;
//}

///////////////////////////////////////////////////////////////////////
////功    能：验证卡片密码
////参数说明: auth_mode[IN]: 密码验证模式
////                 0x60 = 验证A密钥
////                 0x61 = 验证B密钥 
////          addr[IN]：块地址
////          pKey[IN]：密码
////          pSnr[IN]：卡片序列号，4字节
////返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////////               
//char P_PcdAuthState(u8   auth_mode,u8   addr,u8 *pKey,u8 *pSnr)
//{
//    char   status;
//    u8   unLen;
//    u8   ucComMF522Buf[MAXRLEN]; 

//    ucComMF522Buf[0] = auth_mode;
//    ucComMF522Buf[1] = addr;
//    memcpy(&ucComMF522Buf[2], pKey, 6); 
//    memcpy(&ucComMF522Buf[8], pSnr, 4);     
//    status = P_PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
//    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
//    {   status = MI_ERR;   }    
//    return status;
//}

///////////////////////////////////////////////////////////////////////
////功    能：读取M1卡一块数据
////参数说明: addr[IN]：块地址
////          p [OUT]：读出的数据，16字节
////返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////// 
//char P_PcdRead(u8   addr,u8 *p )
//{
//    char   status;
//    u8   unLen;
//    u8   i,ucComMF522Buf[MAXRLEN]; 

//    ucComMF522Buf[0] = PICC_READ;
//    ucComMF522Buf[1] = addr;
//    P_CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);   
//    status = P_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
//    if ((status == MI_OK) && (unLen == 0x90))
//    {
//        for (i=0; i<16; i++)
//        {    *(p +i) = ucComMF522Buf[i];   }
//    }
//    else
//    {   status = MI_ERR;   }    
//    return status;
//}

///////////////////////////////////////////////////////////////////////
////功    能：写数据到M1卡一块
////参数说明: addr[IN]：块地址
////          p [IN]：写入的数据，16字节
////返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////////                  
//char P_PcdWrite(u8   addr,u8 *p )
//{
//    char   status;
//    u8   unLen;
//    u8   i,ucComMF522Buf[MAXRLEN]; 
//    
//    ucComMF522Buf[0] = PICC_WRITE;
//    ucComMF522Buf[1] = addr;
//    P_CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
//    status = P_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
//    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
//    {   status = MI_ERR;   }        
//    if (status == MI_OK)
//    {
//        for (i=0; i<16; i++)
//        {    
//        	ucComMF522Buf[i] = *(p +i);   
//        }
//        P_CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);
//        status = P_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
//        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
//        {   status = MI_ERR;   }
//    }    
//    return status;
//}

///////////////////////////////////////////////////////////////////////
////功    能：命令卡片进入休眠状态
////返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////////
//char P_PcdHalt(void)
//{
//    u8   status;
//    u8   unLen;
//    u8   ucComMF522Buf[MAXRLEN]; 
//    ucComMF522Buf[0] = PICC_HALT;
//    ucComMF522Buf[1] = 0;
//    P_CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
//    status = P_PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
//    return MI_OK;
//}

///////////////////////////////////////////////////////////////////////
////用MF522计算CRC16函数
///////////////////////////////////////////////////////////////////////
//void P_CalulateCRC(u8 *pIn ,u8   len,u8 *pOut )
//{
//    u8   i,n;
//    P_ClearBitMask(DivIrqReg,0x04);
//    P_WriteRawRC(CommandReg,PCD_IDLE);
//    P_SetBitMask(FIFOLevelReg,0x80);
//    for (i=0; i<len; i++)
//    {   P_WriteRawRC(FIFODataReg, *(pIn +i));   }
//    P_WriteRawRC(CommandReg, PCD_CALCCRC);
//    i = 0xFF;
//    do 
//    {
//        n = P_ReadRawRC(DivIrqReg);
//        i--;
//    }
//    while ((i!=0) && !(n&0x04));
//    pOut [0] = P_ReadRawRC(CRCResultRegL);
//    pOut [1] = P_ReadRawRC(CRCResultRegM);
//}

///////////////////////////////////////////////////////////////////////
////功    能：复位RC522out
////返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////////
//char P_PcdReset(void)
//{
//	P_SET_RC522RST;
//	delay_ns(10);
//	P_CLR_RC522RST;
//	delay_ns(10);
//	P_SET_RC522RST;
//	delay_ns(10);
//	P_WriteRawRC(CommandReg,PCD_RESETPHASE);
//	P_WriteRawRC(CommandReg,PCD_RESETPHASE);
//	delay_ns(10);
//	P_WriteRawRC(ModeReg,0x3D);            //和Mifare卡通讯，CRC初始值0x6363
//	P_WriteRawRC(TReloadRegL,30);           
//	P_WriteRawRC(TReloadRegH,0);
//	P_WriteRawRC(TModeReg,0x8D);
//	P_WriteRawRC(TPrescalerReg,0x3E);
//	P_WriteRawRC(TxAutoReg,0x40);//必须要
//	return MI_OK;
//}
////////////////////////////////////////////////////////////////////////
////设置RC632的工作方式 
////////////////////////////////////////////////////////////////////////
//char P_M500PcdConfigISOType(u8   type)
//{
//   if (type == 'A')                     //ISO14443_A
//   { 
//       P_ClearBitMask(Status2Reg,0x08);
//       P_WriteRawRC(ModeReg,0x3D);//3F
//       P_WriteRawRC(RxSelReg,0x86);//84
//       P_WriteRawRC(RFCfgReg,0x7F);   //4F
//   	   P_WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
//	   P_WriteRawRC(TReloadRegH,0);
//       P_WriteRawRC(TModeReg,0x8D);
//	   P_WriteRawRC(TPrescalerReg,0x3E);
//	   delay_ns(1000);
//       P_PcdAntennaOn();
//   }
//   else{ return 1; }   
//   return MI_OK;
//}

///////////////////////////////////////////////////////////////////////
////功    能：读RC632寄存器
////参数说明：Address[IN]:寄存器地址
////返    回：读出的值
///////////////////////////////////////////////////////////////////////
//u8 P_ReadRawRC(u8   Address)
//{
//    u8   ucAddr;
//    u8   ucResult=0;
//	P_CLR_SPI_CS;
//    ucAddr = ((Address<<1)&0x7E)|0x80;
//	
//	P_SPIWriteByte(ucAddr);
//	ucResult=P_SPIReadByte();
//	P_SET_SPI_CS;
//   return ucResult;
//}

///////////////////////////////////////////////////////////////////////
////功    能：写RC632寄存器
////参数说明：Address[IN]:寄存器地址
////          value[IN]:写入的值
///////////////////////////////////////////////////////////////////////
//void P_WriteRawRC(u8   Address, u8   value)
//{  
//    u8   ucAddr;
//	P_CLR_SPI_CS;
//    ucAddr = ((Address<<1)&0x7E);
//	P_SPIWriteByte(ucAddr);
//	P_SPIWriteByte(value);
//	P_SET_SPI_CS;
//}

///////////////////////////////////////////////////////////////////////
////功    能：置RC522寄存器位
////参数说明：reg[IN]:寄存器地址
////          mask[IN]:置位值
///////////////////////////////////////////////////////////////////////
//void P_SetBitMask(u8   reg,u8   mask)  
//{
//    char   tmp = 0x0;
//    tmp = P_ReadRawRC(reg);
//    P_WriteRawRC(reg,tmp | mask);  // set bit mask
//}

///////////////////////////////////////////////////////////////////////
////功    能：清RC522寄存器位
////参数说明：reg[IN]:寄存器地址
////          mask[IN]:清位值
///////////////////////////////////////////////////////////////////////
//void P_ClearBitMask(u8   reg,u8   mask)  
//{
//    char   tmp = 0x0;
//    tmp = P_ReadRawRC(reg);
//    P_WriteRawRC(reg, tmp & ~mask);  // clear bit mask
//} 

///////////////////////////////////////////////////////////////////////
////功    能：通过RC522和ISO14443卡通讯
////参数说明：Command[IN]:RC522命令字
////          pIn [IN]:通过RC522发送到卡片的数据
////          InLenByte[IN]:发送数据的字节长度
////          pOut [OUT]:接收到的卡片返回数据
////          *pOutLenBit[OUT]:返回数据的位长度
///////////////////////////////////////////////////////////////////////
//char P_PcdComMF522(u8   Command, 
//                 u8 *pIn , 
//                 u8   InLenByte,
//                 u8 *pOut , 
//                 u8 *pOutLenBit)
//{
//    char   status = MI_ERR;
//    u8   irqEn   = 0x00;
//    u8   waitFor = 0x00;
//    u8   lastBits;
//    u8   n;
//    u16   i;
//    switch (Command)
//    {
//        case PCD_AUTHENT:
//			irqEn   = 0x12;
//			waitFor = 0x10;
//			break;
//		case PCD_TRANSCEIVE:
//			irqEn   = 0x77;
//			waitFor = 0x30;
//			break;
//		default:
//			break;
//    }   
//    P_WriteRawRC(ComIEnReg,irqEn|0x80);
//    P_ClearBitMask(ComIrqReg,0x80);	//清所有中断位
//    P_WriteRawRC(CommandReg,PCD_IDLE);
//    P_SetBitMask(FIFOLevelReg,0x80);	 	//清FIFO缓存    
//    for (i=0; i<InLenByte; i++)
//    {   P_WriteRawRC(FIFODataReg, pIn [i]);    }
//    P_WriteRawRC(CommandReg, Command);	    
//    if (Command == PCD_TRANSCEIVE)
//    {    P_SetBitMask(BitFramingReg,0x80);  }	 //开始传送
//	i = 2000;
//    do 
//    {
//        n = P_ReadRawRC(ComIrqReg);
//        i--;
//    }
//    while ((i!=0) && !(n&0x01) && !(n&waitFor));
//    P_ClearBitMask(BitFramingReg,0x80);
//    if (i!=0)
//    {    
//        if(!(P_ReadRawRC(ErrorReg)&0x1B))
//        {
//            status = MI_OK;
//            if (n & irqEn & 0x01)
//            {   status = MI_NOTAGERR;   }
//            if (Command == PCD_TRANSCEIVE)
//            {
//               	n = P_ReadRawRC(FIFOLevelReg);
//              	lastBits = P_ReadRawRC(ControlReg) & 0x07;
//                if (lastBits)
//                {   *pOutLenBit = (n-1)*8 + lastBits;   }
//                else
//                {   *pOutLenBit = n*8;   }
//                if (n == 0)
//                {   n = 1;    }
//                if (n > MAXRLEN)
//                {   n = MAXRLEN;   }
//                for (i=0; i<n; i++)
//                {   pOut [i] = P_ReadRawRC(FIFODataReg);    }
//            }
//        }
//        else
//        {   status = MI_ERR;   }        
//    }
//    P_SetBitMask(ControlReg,0x80);           // stop timer now
//    P_WriteRawRC(CommandReg,PCD_IDLE); 
//    return status;
//}

///////////////////////////////////////////////////////////////////////
////开启天线  
////每次启动或关闭天险发射之间应至少有1ms的间隔
///////////////////////////////////////////////////////////////////////
//void P_PcdAntennaOn(void)
//{
//    u8   i;
//    i = P_ReadRawRC(TxControlReg);
//    if (!(i & 0x03))
//    {
//        P_SetBitMask(TxControlReg, 0x03);
//    }
//}

///////////////////////////////////////////////////////////////////////
////关闭天线
///////////////////////////////////////////////////////////////////////
//void P_PcdAntennaOff(void)
//{
//	P_ClearBitMask(TxControlReg, 0x03);
//}


///********************************************************************************************/
///*************************************
//*函数功能：显示卡的卡号，以十六进制显示
//*参数：x，y 坐标
//*		p 卡号的地址
//*		charcolor 字符的颜色
//*		bkcolor   背景的颜色
//***************************************/
void TurnID(u8 *p)	 //转换卡的卡号，以十六进制显示
{
	unsigned char i;
	for(i=0;i<4;i++)                              
	{
		num[i*2]=p[i]>>4;
		num[i*2]>9?(num[i*2]+='7'):(num[i*2]+='0');
		num[i*2+1]=p[i]&0x0f;
		num[i*2+1]>9?(num[i*2+1]+='7'):(num[i*2+1]+='0');
	}
	num[8]=0;
}

//void P_TurnID(u8 *p)	 //转换卡的卡号，以十六进制显示
//{
//	unsigned char i;
//	for(i=0;i<4;i++)
//	{
//		P_num[i*2]=p[i]>>4;
//		P_num[i*2]>9?(P_num[i*2]+='7'):(P_num[i*2]+='0');
//		P_num[i*2+1]=p[i]&0x0f;
//		P_num[i*2+1]>9?(P_num[i*2+1]+='7'):(P_num[i*2+1]+='0');
//	}
//	P_num[8]=0;
//}

void ReadID(void)								//读SPI2对应的卡
{
	unsigned char status;
	status = PcdRequest(PICC_REQALL,CT);/*尋卡*/
	if(status==MI_OK)//尋卡成功
	{
		status = PcdAnticoll(SN);/*防冲撞*/		 
	}
	if (status==MI_OK)//防衝撞成功
	{		
		Card_flag=1;			//内读卡器检测到卡标志  0 ：无，1：有
		status=MI_ERR;	
		TurnID(SN);	 //转换卡的卡号，存在数组num[]内				
	}
}

//void P_ReadID(void)			//读SPI3对应的卡
//{
//	unsigned char status;
//	status = PcdRequest(PICC_REQALL,P_CT);/*尋卡*/
//	if(status==MI_OK)//尋卡成功
//	{
//		status = PcdAnticoll(SN);/*防冲撞*/		 
//	}
//	if (status==MI_OK)//防衝撞成功
//	{	
//		P_Card_flag=1;			//外读卡器检测到卡标志  0 ：无，1：有		
//		status=MI_ERR;	
//		TurnID(SN);	 //转换卡的卡号，存在数组num[]内				
//	}
//}
u8 First(char *num)
{
	while(Card_flag==0)
	{
	  ReadID();
	}
	if(Card_flag==1)	//如果检测到IC卡
  {
		POINT_COLOR=BLUE;
	  LCD_ShowString(30,100,210,24,24,"*****************");			
	  LCD_ShowString(30,130,200,16,16,"111111111111");
		LCD_ShowString(156,130,200,16,16,(u8 *)num);
	 	delay_ms(1000);
	}
	Card_flag=0;      //内刷卡清零
	
}	

u8 Second(char *num)
{
	while(Card_flag==0)
	{
		ReadID();
	}
  if(Card_flag==1)
	{		
		POINT_COLOR=BLUE;
		LCD_ShowString(30,170,180,16,16,"222222222222");
	  LCD_ShowString(156,170,200,16,16,(u8 *)num);
	  delay_ms(1000);
	}
    Card_flag=0;      //内刷卡清零
	
}
void Compare_ID(void)
{
	u8 i;
	u8 state;//1为相同，0为不同
	int time=2000;
	char redata1[9];
  char redata2[9];
	First(num);
	for(i=0;i<sizeof(num)/sizeof(num[0]);i++)
	redata1[i] = num[i];
	
	time=2000;
	while(time)
		delay_ms(1),time--;
	LCD_ShowString(5,150,250,6,16,"Please start your second record");
	Second(num);
		for(i=0;i<sizeof(num)/sizeof(num[0]);i++)
	redata2[i] = num[i];
	
	
	for(i=0;i<sizeof(num)/sizeof(num[0]);i++)
	{
		if(redata1[i]==redata2[i])
			state=1;
		else
		{state=0;break;}
	}
	
	if(state==1)
		{ 
		LCD_ShowString(56,200,200,16,16,"OK");
		delay_ms(1000);
	}
	else 
	{
		LCD_ShowString(36,250,200,16,16,"please try again");
	  delay_ms(1000);
	}
	
//	if(redata1==redata2)
//	{ 
//		LCD_ShowString(56,200,200,16,16,"OK");
//		delay_ms(1000);
//	}
//	else 
//	{
//		LCD_ShowString(36,250,200,16,16,"please try again");
//	  delay_ms(1000);
//	}
}


