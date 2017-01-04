/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_interrupt.c
;* Author             : 张力阵
;* 中断管理驱动程序
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "Interrupt.h"

//*****************************************************************************
//
// The processor vector table.
//
// This contains a list of the handlers for the various interrupt sources in
// the system.  The layout of this list is defined by the hardware; assertion
// of an interrupt causes the processor to start executing directly at the
// address given in the corresponding location in this list.
//
//*****************************************************************************
#if defined(ewarm)
static __no_init void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void) @ "VTABLE";
#elif defined(sourcerygxx)
static __attribute__((section(".cs3.region-head.ram")))
void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void);
#else
static __attribute__((section("vtable")))
void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void);	//函数指针
#endif
/*****************************************************************************
* 初始化中断 
* 中断优先级 0最高 255最低
* 标准晶振频率较高 中断有效级最高 防止丢脉冲
*****************************************************************************/
void Init_Int(void)
{  
    IntPrioritySet(FAULT_SYSTICK,INT_PRIORITY_5);   //系统节拍发生器 中断优先级  2 系统定时器中断
    IntPrioritySet(INT_UART0,INT_PRIORITY_6);       //UART0          中断优先级  6 UART0中断
    IntPrioritySet(INT_UART1,INT_PRIORITY_6);       //UART1          中断优先级  6 UART1中断
    IntPrioritySet(INT_TIMER0A,INT_PRIORITY_3);     //TIMER0-A       中断优先级  3 电子脉冲计数中断
//    IntPrioritySet(INT_TIMER0B,INT_PRIORITY_0);     //TIMER0-B       中断优先级  0 标准晶振高频计数溢出中断
//    IntPrioritySet(INT_TIMER1A,INT_PRIORITY_1);     //TIMER1-A       中断优先级  1 时钟脉冲计数溢出中断
//    IntPrioritySet(INT_TIMER1B,INT_PRIORITY_3);     //TIMER1-B       中断优先级  3 被检表高频计数溢出中断
    IntPrioritySet(INT_TIMER2A,INT_PRIORITY_2);     //TIMER2-A       中断优先级  2 标准表高频计数溢出中断 
//    IntPrioritySet(INT_TIMER2B,INT_PRIORITY_7);   //TIMER2-B       中断优先级  7 PWM 输出
    IntPrioritySet(INT_CAN0,INT_PRIORITY_4);        //CAN            中断优先级  4 CAN 中断
//    IntPrioritySet(INT_WATCHDOG,INT_PRIORITY_0);  //看门狗         中断优先级  0 看门狗 中断
    IntPrioritySet(INT_GPIOB,INT_PRIORITY_3);       //GPIOB          中断优先级  3 GPIOB中断 光电头脉冲输入 
    IntPrioritySet(INT_GPIOC,INT_PRIORITY_5);       //GPIOC          中断优先级  3 按键中断
    IntPrioritySet(INT_GPIOD,INT_PRIORITY_3);       //GPIOD          中断优先级  3 CS5460A_INT
//    IntPrioritySet(INT_GPIOF,INT_PRIORITY_3);       //GPIOF          中断优先级  3 需量脉冲 时段投切 合闸脉冲中断 
    
    IntEnable(FAULT_SYSTICK);   //系统节拍发生器NVIC中断使能  系统定时器中断
    IntEnable(INT_UART0);       //UART0         NVIC中断使能  UART0中断
    IntEnable(INT_UART1);       //UART1         NVIC中断使能  UART1中断
    IntEnable(INT_TIMER0A);     //TIMER0-A      NVIC中断使能  电子脉冲计数中断
//    IntEnable(INT_TIMER0B);     //TIMER0-B      NVIC中断使能  标准晶振高频计数溢出中断
//    IntEnable(INT_TIMER1A);     //TIMER1-A      NVIC中断使能  时钟脉冲计数溢出中断
#ifndef __TEST
//    IntEnable(INT_TIMER1B);     //TIMER1-B      NVIC中断使能  被检表高频计数溢出中断
#endif
    IntEnable(INT_TIMER2A);     //TIMER2-A      NVIC中断使能  标准表高频计数溢出中断 
//    IntEnable(INT_TIMER2B);   //TIMER2-B      NVIC中断使能  PWM 输出
    IntEnable(INT_CAN0);        //CAN           NVIC中断使能  CAN 中断
//    IntEnable(INT_WATCHDOG);    //看门狗        NVIC中断使能
    IntEnable(INT_GPIOB);       //GPIOB         NVIC中断使能  GPIOB中断 光电头脉冲输入
    IntEnable(INT_GPIOC);       //GPIOC         NVIC中断使能  GPIOC中断 按键输入
    IntEnable(INT_GPIOD);       //GPIOD         NVIC中断使能  GPIOD中断 CS5460A_INT
//    IntEnable(INT_GPIOF);       //GPIOF         NVIC中断使能  GPIOF中断 需量脉冲 时段投切 合闸脉冲
	IntEnable(INT_ADC0);							//使能ADC sequence0 NVIC中断
}
/*****************************************************************************
* CPU中断使能
*****************************************************************************/
u8 IntMasterEnable(void)
{
    return(CPUcpsie());
}

/*****************************************************************************
* CPU中断禁能
*****************************************************************************/
u8 IntMasterDisable(void)
{
    return(CPUcpsid());
}

/*****************************************************************************
* 设置中断入口子程序
* 入口ulInterrupt 中断ID号
* 入口pfnHandler 中断服务函数
*****************************************************************************/
void IntRegister(u32 ulInterrupt, void (*pfnHandler)(void))
{
    u32 ulIdx;
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    ASSERT(((u32)g_pfnRAMVectors & 0x000003ff) == 0);
    if(NVIC->VTABLE != (u32)g_pfnRAMVectors)
     {
        for(ulIdx = 0; ulIdx < NUM_INTERRUPTS; ulIdx++)
         g_pfnRAMVectors[ulIdx] = (void (*)(void))HWREG(ulIdx * 4);
        NVIC->VTABLE = (u32)g_pfnRAMVectors;
     }
    g_pfnRAMVectors[ulInterrupt] = pfnHandler;
}

/*****************************************************************************
* 撤销中断子程序
* 入口ulInterrupt 中断ID号
*****************************************************************************/
void IntUnregister(u32 ulInterrupt)
{
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    g_pfnRAMVectors[ulInterrupt] = IntDefaultHandler;
}

/*****************************************************************************
* 优先级分组设置子程序
* 入口ulBits 优先级分组 ulBits=0~7 
* ulBits 占先优先级位数
*****************************************************************************/
void IntPriorityGroupingSet(u32 ulBits)
{
    ASSERT(ulBits < NUM_PRIORITY);
    ulBits=((~(ulBits<<8)) & NVIC_APINT_PRIGROUP_M);
    NVIC->APINT = (NVIC_APINT_VECTKEY | ulBits);
}

/*****************************************************************************
* 获取优先级分组设置子程序
* 入口ulBits 优先级分组 ulBits=0~7 
*****************************************************************************/
u32	IntPriorityGroupingGet(void)
{
    u32 ulValue;

    ulValue = NVIC->APINT ;
    ulValue = ((~(ulValue>>8)) & NVIC_APINT_PRIGROUP_M);
    return(ulValue);
}

/*****************************************************************************
* 设置中断优先级
* 入口 ulInterrupt 中断号
* 入口 ucPriority  优先级
*****************************************************************************/
void IntPrioritySet(u32 ulInterrupt, u8 ucPriority)
{
    u32 ulTemp;
    ASSERT((ulInterrupt >= 4) && (ulInterrupt < NUM_INTERRUPTS));
	if(ulInterrupt<16)
	 {	//设置系统中断优先级
      ulTemp = NVIC->SYS_PRI[(ulInterrupt>>2)-1];
      ulTemp &= ~(0xFF << (8 * (ulInterrupt & 3)));
      ulTemp |= ucPriority << (8 * (ulInterrupt & 3));
      NVIC->SYS_PRI[(ulInterrupt>>2)-1] = ulTemp;
	 }
	else
	 {  //设置设备中断优先级
      ulTemp = NVIC->PRI[(ulInterrupt>>2)-4];
      ulTemp &= ~(0xFF << (8 * (ulInterrupt & 3)));
      ulTemp |= ucPriority << (8 * (ulInterrupt & 3));
      NVIC->PRI[(ulInterrupt>>2)-4] = ulTemp;
	 }
}

/*****************************************************************************
* 获取设备中断优先级
* 出口 优先级
*****************************************************************************/
u32 IntPriorityGet(u32 ulInterrupt)
{
	u32 Priority;
    ASSERT((ulInterrupt >= 4) && (ulInterrupt < NUM_INTERRUPTS));
	if(ulInterrupt<16)
	 {  //获取内核设备优先级
	  Priority=NVIC->SYS_PRI[(ulInterrupt>>2)-1];
	 }
	else
	 {
	  Priority=NVIC->PRI[(ulInterrupt>>2)-4];
	 }
	ulInterrupt &= 3;
	ulInterrupt <<=3;   //*8
    return((Priority >> ulInterrupt) &
           0xFF);
}

/*****************************************************************************
* 中断使能子程序
* 入口:ulInterrupt 中断ID号
*****************************************************************************/
void IntEnable(u32 ulInterrupt)
{
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    if(ulInterrupt == FAULT_MPU)	     //存储器保护中断
     NVIC->SYS_HND_CTRL |= NVIC_SYS_HND_CTRL_MEM;
    else if(ulInterrupt == FAULT_BUS)    //总线错误中断
     NVIC->SYS_HND_CTRL |= NVIC_SYS_HND_CTRL_BUS;
    else if(ulInterrupt == FAULT_USAGE)  //使用故障
     NVIC->SYS_HND_CTRL |= NVIC_SYS_HND_CTRL_USAGE;
    else if(ulInterrupt == FAULT_SYSTICK)//系统节拍定时器中断
     NVIC->ST_CTRL |= NVIC_ST_CTRL_INTEN;
    else
     NVIC->EN[((ulInterrupt-16)/32)] = 1 << ((ulInterrupt - 16)%32);
}

/*****************************************************************************
* 中断禁能子程序
* 入口:ulInterrupt 中断ID号
*****************************************************************************/
void IntDisable(u32 ulInterrupt)
{
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    if(ulInterrupt == FAULT_MPU)
     NVIC->SYS_HND_CTRL &= ~(NVIC_SYS_HND_CTRL_MEM);
    else if(ulInterrupt == FAULT_BUS)
     NVIC->SYS_HND_CTRL &= ~(NVIC_SYS_HND_CTRL_BUS);
    else if(ulInterrupt == FAULT_USAGE)
     NVIC->SYS_HND_CTRL &= ~(NVIC_SYS_HND_CTRL_USAGE);
    else if(ulInterrupt == FAULT_SYSTICK)
     NVIC->ST_CTRL &= ~(NVIC_ST_CTRL_INTEN);
    else
     NVIC->DIS[((ulInterrupt-16)/32)] = 1 << ((ulInterrupt - 16)%32);
}

