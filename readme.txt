void GPIO_Init(GPIO_TypeDef*GPIOx,GPIO_InitTypeDef*GPIO_InitStruct;//GPIO的初始化函数,函数有两个参数，第一个参数用来指定需要初始化的GPIO组，取值范围为GPIOA~GPIOK;第二个参数为初始化参数结构体指针，结构类型为GPIO_InitTypeDef,其结构体的定义为：
typedefstruct
{
    uint32_t GPIO_Pin;
	GPIOMode_TypeDef GPIO_Mode;
	GPIOSpeed_TypeDef GPIO_Speed;
	GPIOOType_TypeDef GPIO_OType;
	GPIOPuPd_TypeDef GPIO_PuPd;
}GPIO_InitTypeDef;
初始化GPIO的常用格式是：
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;  //GPIOF9
GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;   //普通输出模式
GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100 MHz;   //100MHz
GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;   //推挽模式
GPIO_InitStructure.GPIO_PuPb=GPIO_PuPb_UP;   //上拉模式
GPIO_Init(GPIOF,&GPIO_InitStructure);   //初始化GPIO
上面代码的意思是设置GPIO的第九个端口为推挽输出模式，同时速度为100MHz,上拉模式
GPIO_InitStructure的第一个成员变量GPIO_Pin用来设置要初始化的是哪个或者哪些I/O端口；第二个成员变量GPIO_Mode用来设置对应I/O端口的输入/输出模式

GPIOx_IDR:输入数据寄存器（该寄存器用于读取GPIOx的输入）
unit8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx,unit16_t GPIO_Pin);
unit16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
//第一个函数是用来读取一组GPIO的一个或者几个I/O端口输入电平，第二个函数用来一次读取一组GPIO所有I/O端口的输入电平
GPIO_ReadInputDataBit(GPIOF,GPIO_PIN_5);//读取GPIOF.5的输入电平

GPIOx_ODR:输出数据寄存器（该寄存器是GPIO输入/输出电平控制相关的寄存器，用于控制GPIOx的输出）
voidGPIO_Write(GPIO_TypeDef*GPIOx,unit16_t PortVal);//该函数一般用来一次性地往一个GPIO写入多个端口设值
GPIO_Write(GPIOA_0x0000);//往GPIO的A组端口输入设值
//同时读GPIOx_ODR还可以读出I/O端口的输出状态
unit16_tGPIO_ReadOutputData(GPIO_TypeDef*GPIOx);
unit8_t GPIO_ReadOutputDataBit(GPIO_TypeDef*GPIOx,unit16_t GPIO_Pin);

GPIOx_BSRR:置位/复位寄存器（该寄存器用来置位或者复位I/O端口）
//对于GPIOx_BSRR,不需要先读，可以直接设置
GPIOA-->BSRR=1<<1;   //设置GPIOA.1为高电平
GPIOA-->BSRR=1<<(16+1);   //设置GPIOA.1为低电平
void GPIO_SetBits(GPIO_TypeDef*GPIOx,unit16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef*GPIOx,unit16_t GPIO_Pin);
//通过库函数操作GPIOx_BSRR来设置I/O端口电平的函数
