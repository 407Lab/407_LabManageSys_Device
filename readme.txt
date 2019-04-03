//以TIM3定时器为例
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);   //使能TIM3定时器，TIM3定时器挂载在APB1总线下，所以通过APB1总线下的时钟函数可使能TIM3定时器
void TIM_TimeBaseInit(TIM_TypeDef*TIMx,TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);   //设置自动重装值、分频系数、计数方式等，定时器参数的初始化是通过初始化函数TIM_TimeBaseInit实现的，第一个参数确定为哪个定时器；第二个参数是定时器初始化参数结构体指针
//结构体类型为TIM_TimeBaseInitTypeDef;结构体的定义如下
typedef struct
{
   unit16_t TIM_Prescaler;   //设置分频系数
   unit16_t TIM_CounterMode;   //设置计数方式，可以设置为向上计数、向下计数方式和中心对齐计数模式，比较常用的是向上计数模式(TIM_CounterMode_Up)和向下计数模式(TIM_CounterMode_Down)
   unit16_t TIM_Period;   //设置自动重载计数周期值
   unit16_t TIM_ClockDivision;   //设置时钟分频因子
   unit8_t TIM_RepetitionCounter;   //此成员变量是高级控制器才会用到的
}TIM_TimeBaseInitTypeDef;

void TIM_ITConfig(TIM_TypeDef*TIMx,unit16_t TIM_IT,FouctionalState NewState);   //设置TIM3_DIER使能更新中断,第一个参数用于选择定时器，取值为TIM1~TIM17;第二个参数用来指明使能的定时器中断的类型，定时器中断的类型包括:更新中断(TIM_IT_Update)、触发中断(TIM_IT_Trigger),以及输入捕获中断等；第三个参数表示禁止还是使能更新中断

void TIM_Cmd(TIM_TypeDef*TIMx,FouctionalState NewState);   //使能TIM3定时器

ITStatus TIM_GetITStatus(TIM_TypeDef*TIMx,unit16_t);   //编写中断服务函数，中断服务函数用来处理定时器产生的中断，在中断产生后，通过状态寄存器的值来判断产生的中断属于什么类型，然后执行相关的操作，这里使用的是更新（溢出）中断（在状态寄存器中TIMx_SR的最低位），在处理完中断之后应该向TIMx_SR的最低位写0来清除该中断标志

void TIM_ClearITPendingBit(TIM_TypeDef*TIMx,unit16_t TIM_IT);   //清除中断标志位

//TIM3定时器初始化示例如下
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_TimeBaseStructure.TIM_Period=5000;
TIM_TimeBaseStructure.TIM_Prescaler=7199;
TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIVI;
TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

//要使能TIM3定时器
TIM_Cmd(TIM3,ENABLE);   //使能TIM3外设

//程序中判断TIM3定时器是否发生更新（溢出）中断
if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
{ }

//在TIM3定时器的更新中断发生后要清除中断标志位
TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
   
