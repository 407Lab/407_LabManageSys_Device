//USART初始化结构体
typedef struct
{
   uint32_t USART_BaudRate;   //用于设置波特率，一般设置为2400、9600、19200、115200，标准库函数会根据设定值计算得到USARTDIV值，并设置USART_BRR
   uint16_t USART_WordLength;   //用于设置数据帧字长，一般设置为8位或9位，它设定USART_CR1的M位的值，若没有使能奇偶校检控制，一般使用8数据位，若使能奇偶校检，一般使用9数据位
   uint16_t USART_StopBits;   //用于设置停止位，可选0.5位、1位、1.5位和2位停止位，它设定USART_CR2的STOP[1:0]位的值，一般选择1位停止位
   uint16_t USART_Parity;   //用于设置奇偶校检控制，可选USART_Parity_No(无校检)、USARTA_Parity_Even(奇校检)、USART_Parity_Odd(偶校检),它是设定USART_CR1的PCE位和PS位的值
   uint16_t USART_Mode;   //用于设置USART的工作模式，有USART_Mode_Rx和USART_Mode_Tx，允许使用逻辑或运算选择两个，它设定USART_CR1的RE位和TE位
   uint16_t USART_HardwareFlowControl;   //用于设置硬件流控制，只有在硬件流控制下有效，可选使能RTS、使能CTS、同时使能RTS和CTS、不使能硬件流
}USART_InitTypeDef;

//当使用同步模式时需要配置SCLK引脚输出脉冲的属性，标准库是通过一个时钟初始化结构体USART_ClockInitTypeDef来设置的，因此该结构体内容也只有在同步模式才需要设置
typdef struct
{
   uint16_t USART_Clock;   //用于同步模式下SCLK引脚上时钟输出使能控制，可选禁止时钟输出(USART_Clock_Disable)或开启时钟输出(USART_Clock_Enable);若使用同步模式发送，一般都需要开启时钟，它设定USART_CR2的CLKEN位的值
   uint16_t USART_CPOL;   //用于同步模式下SCLK引脚上的输出时钟极性设置，设置在空闲时SCLK引脚为低电平(USART_CPOL_Low)或高电平(USART_CPOL_High)，它设定USART_CR2的CPOL位的值
   uint16_t USART_CPHA;   //用于同步模式下SCLK引脚上的输出时钟相位设置，可设置在时钟第一个变化沿捕获数据(USART_CPHA_1Edge)或在时钟第二个变化沿捕获数据，它设定USART_CR2的CPHA位的值，USART_CPHA和USART_CPOL配合使用可以获得多种模式时钟关系
   uint16_t USART_LastBit;   //末位时钟脉冲，用于选择在发送最后一个数据位时时钟脉冲是否在SCLK引脚输出，可以是不输出脉冲(USART_LastBit_Disable)、输出脉冲(USART_LastBit_Enable),它设定USART_CR2的LBCL位的值
}USART_ClockInitTypeDef
   
   