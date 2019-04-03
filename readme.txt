//使能I/O端口闹钟，初始化I/O端口为输入模式。首先，要使用I/O端口作为中断输入，所以要使能相应的I/O端口时钟，以及初始化相应的I/O端口为输入模式。首先，要使用I/O端口作为中断输入，所以要使能相应的I/O端口时钟，以及初始化相应的I/O端口为输入模式

RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);   //使能SYSCFG时钟，要配置GPIO与中断线的映射关系，首次需要使能SYSCFG时钟
void SYSCFG_EXTILineConfig(unit8_t EXTI_PortSourceGPIOx,unit8_EXTI_PinSourcex);   //在库函数中，配置GPIO与中断线的映射关系是通过函数SYSCFG_EXTILineConfig()来实现的
SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);   //将中断线0与GPIOA映射起来，GPIOA.0与EXTI1中断线连接起来

void EXTI_Init(EXTI_InitTypeDef*EXTI_InitStruct);   //在程序中设置该中断线上的中断初始化参数，如设置触发条件。中断线上的中断初始化是通过函数EXTI_Init()来实现的
EXTI_InitTypeDef EXTI_InitStructure;
EXTI_InitStructure.EXTI_Line=EXTI_Line4;
EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
EXTI_InitStructure.EXTI_LineCmd=ENABLE;
EXTI_Init(&EXTI_InitStructure);   //初始化外设EXTI寄存器
//以上的例子设置中断线4上的中断为下降沿触发

//结构体EXTI_InitTypeDef的成员变量（即参数）的定义如下
typedef struct
{
    unit32_t EXTI_Line;   //中断线的标号，对于外部中断，取值范围为EXTI_Line0~EXTI_Line15.线上的中断参数
	EXTIMode_TypeDef EXTI_Mode;   //中断模式，可选值为EXTI_Mode_Interrupt和EXTI_Mode_Event
	EXTITrigger_TypeDef EXTI_Trigger;   //触发方式，可以是下降沿触发(EXTI_Trigger_Falling)、上升沿触发(EXTI_Trigger_Rising)或者任意电平(上升沿和下降沿)触发(EXTI_Trigger_Rising_Falling)
	FunctionalState EXTI_LinCmd;   //中断线使能
	}EXTI_InitTypeDef;

//既然是外部中断，就必须配置NVIC中断优先级
NVIC_InitTypeDef NVIC_InitStructure;
NVIC_InitStructure.NVIC_IRQChannel=EXTI2_IRQn;   //使能按键外部中断通道
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;   //抢占优先级2
NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02;   //响应优先级2
NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;   //使能外部中断通道
NVIC_Init(&NVIC_InitStructure);   //中断优先级分组初始化

//配置完中断服务函数后，接下来就是要编写中断服务函数
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);   //此函数用于判断某个中断线上的中断是否发生(标志位是否置位)，一般应用于中断服务函数的开头
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);   //此函数用于清除某条中断线的中断标志位，一般应用于中断服务函数结束之前	

//常用的中断服务函数格式
void EXTI3_IRQHandleer(void)
{
   if(EXTI_GetITStatus(EXTI_Line3)!=RESET)   //判断某条中断线上是否发生了中断
   {...中断逻辑...
   EXTI_ClearITPendingBit(EXTI_Line3);   //清除中断线上的中断标志位
   }
}
	

