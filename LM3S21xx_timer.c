/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_timer.c
;* Author             : 张力阵
;* 定时/计数器驱动程序
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "Disp.h"
#include "define.h"
#include "vari.h"
/*****************************************************************************
* 初始化定时器
* Timer0 Timer1 Timer2 被分成六个计数器
* Timer0-A Timer0-B Timer1-A Timer1-B Timer2-A Timer2-B 
*****************************************************************************/
void Init_Timer(void)
{
//初始化Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	//使能设备时钟
    TimerConfigure(TIMER0,TIMER_CFG_16_BIT_PAIR|  //TIMER0分成两个16bit 计数器
                          TIMER_CFG_A_CAP_COUNT|  //Timer0-A 边沿计数模式	CCP0输入 电子脉冲
                          TIMER_CFG_B_CAP_COUNT); //Timer0-B 边沿计数模式	CCP1输入 高稳晶振脉冲
//初始化Timer0-A 电子脉冲输入
    TimerControlEvent(TIMER0,                     //定时器
                      TIMER_A,                    //通道 
                      TIMER_CTL_TAEVENT_NEG);     //脉冲输入方式
    TimerLoadSet(TIMER0,                          //定时器
                 TIMER_A,                         //通道
                 1);                              //设置(分频系数)
    TimerMatchSet(TIMER0,                         //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER0,                        //定时器
                   TIMER_TIMA_TIMEOUT);           //中断类型
//    TimerEnable(TIMER0,                           //定时器
//                TIMER_A);                         //定时器启动
//初始化Timer0-B 标准晶振脉冲输入                 //设置CCP1触发沿为下降沿	 JZ_IN
    TimerControlEvent(TIMER0,                     //定时器
                      TIMER_B,                    //通道 
                      TIMER_CTL_TBEVENT_NEG);     //脉冲输入方式
    TimerLoadSet(TIMER0,                          //定时器
                 TIMER_B,                         //通道
                 0xFFFF);                         //设置重装值(分频系数)
    TimerMatchSet(TIMER0,                         //定时器
                  TIMER_B,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER0,                        //定时器
                   TIMER_TIMB_TIMEOUT);           //中断类型
    TimerEnable(TIMER0,                           //定时器
                TIMER_B);                         //定时器启动
//初始化Timer1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	//使能设备时钟
#ifdef __TEST
    TimerConfigure(TIMER1,TIMER_CFG_16_BIT_PAIR|  //TIMER1分成两个16bit 计数器
                          TIMER_CFG_A_CAP_COUNT|  //Timer1-A 边沿计数模式	CCP2输入 时钟脉冲
                          TIMER_CFG_B_PWM);       //Timer1-B PWM	CCP3输出 测试 标准晶振脉冲1M
#else
    TimerConfigure(TIMER1,TIMER_CFG_16_BIT_PAIR|  //TIMER1分成两个16bit 计数器
                          TIMER_CFG_A_CAP_COUNT|  //Timer1-A 边沿计数模式	CCP2输入 时钟脉冲
                          TIMER_CFG_B_CAP_COUNT); //Timer1-B 边沿计数模式	CCP3输入 被检标准表脉冲
#endif
//初始化Timer1-A 时钟脉冲输入                     //设置CCP2触发沿为下降沿	 SZ_IN
    TimerControlEvent(TIMER1,                     //定时器
                      TIMER_A,                    //通道 
                      TIMER_CTL_TAEVENT_NEG);     //脉冲输入方式
    TimerLoadSet(TIMER1,                          //定时器
                 TIMER_A,                         //通道
                 60000);                          //设置重装值(分频系数)
    TimerMatchSet(TIMER1,                         //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER1,                        //定时器
                   TIMER_TIMA_TIMEOUT);           //中断类型
    TimerEnable(TIMER1,                           //定时器
                TIMER_A);                         //定时器启动
//初始化Timer1-B 被检表高频输入                   //设置CCP3触发沿为下降沿	GP_IN
#ifdef __TEST
    TimerControlLevel(TIMER1,                     //定时器
                      TIMER_B,                    //通道
                      true);                      //CCP5输出反相      
    TimerControlEvent(TIMER1,                     //定时器
                      TIMER_B,                    //通道
                      TIMER_CTL_TBEVENT_NEG);     //脉冲输入方式
    TimerLoadSet(TIMER1,                          //定时器
                 TIMER_B,                         //通道
                 24);                             //设置重装值1mHZ
    TimerMatchSet(TIMER1,                         //定时器 
                  TIMER_B,                        //通道
                  12);                            //设置比较值
    TimerIntEnable(TIMER1,                        //定时器
                   TIMER_TIMB_TIMEOUT);           //中断方式
    TimerEnable(TIMER1,                           //定时器
                TIMER_B);                         //定时器启动
#else
    TimerControlEvent(TIMER1,                     //定时器
                      TIMER_B,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER1,                          //定时器
                 TIMER_B,                         //通道
                 1);                              //设置重装值(分频系数)  2012.11.2 需按需要重做
    TimerMatchSet(TIMER1,                         //定时器
                  TIMER_B,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER1,                        //定时器
                   TIMER_TIMB_TIMEOUT);           //中断类型
#endif
//    TimerEnable(TIMER1,                           //定时器
//                TIMER_B);                         //定时器启动
//初始化Timer2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);   //使能设备时钟
    TimerConfigure(TIMER2,TIMER_CFG_16_BIT_PAIR|    //TIMER2分成两个16bit 计数器
                          TIMER_CFG_A_CAP_COUNT|    //Timer1-A 边沿计数模式	CCP4输入 标准表高频脉冲
                          TIMER_CFG_B_PWM);         //Timer1-B PWM输出模式	CCP5输出 PWM脉冲输出
//初始化Timer2-A 标准表高频输入
    TimerControlEvent(TIMER2,                     //定时器
                      TIMER_A,                    //通道 
                      TIMER_CTL_TAEVENT_NEG);     //脉冲输入方式
    TimerLoadSet(TIMER2,                          //定时器
                 TIMER_A,                         //通道
                 0xFFFF);                         //设置重装值(分频系数)
    TimerMatchSet(TIMER2,                         //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER2,                        //定时器
                   TIMER_TIMA_TIMEOUT);           //中断类型
    TimerEnable(TIMER2,                           //定时器
                TIMER_A);                         //定时器启动
//初始化Timer2-B PWM输出                          //设置CCP5
//初始化Timer1-B 被检表高频输入                   //设置CCP3触发沿为下降沿	GP_IN
#ifdef __TEST
    TimerControlLevel(TIMER2,                     //定时器
                      TIMER_B,                    //通道
                      true);                      //CCP5输出反相      
    TimerControlEvent(TIMER2,                     //定时器
                      TIMER_B,                    //通道
                      TIMER_CTL_TBEVENT_NEG);      //脉冲输入方式
    TimerLoadSet(TIMER2,                          //定时器
                 TIMER_B,                         //通道
                 2495);                           //设置重装值10kHZ
    TimerMatchSet(TIMER2,                         //定时器 
                  TIMER_B,                        //通道
                  1249);                          //设置比较值
    TimerIntEnable(TIMER2,                        //定时器
                   TIMER_TIMB_TIMEOUT);           //中断方式
    TimerEnable(TIMER2,                           //定时器
                TIMER_B);                         //定时器启动
#endif
}
/*****************************************************************************
* 检查TIMER设备基址是否正确
* TIMERx 定时器结构体
* 调试时有效
*****************************************************************************/
#ifdef DEBUG
u8 TimerBaseValid(TIMER_Typedef *TIMERx)
{
    return(((u32)TIMERx == TIMER0_BASE) || ((u32)TIMERx == TIMER1_BASE) ||
           ((u32)TIMERx == TIMER2_BASE) || ((u32)TIMERx == TIMER3_BASE));
}
#endif

/*****************************************************************************
* TIMER使能(TimerA TimerB)
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
*****************************************************************************/
void TimerEnable(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    TIMERx->CTL |= ulTimer & (TIMER_CTL_TAEN | TIMER_CTL_TBEN);
}

/*****************************************************************************
* TIMER禁能(TimerA TimerB)
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
*****************************************************************************/
void TimerDisable(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    TIMERx->CTL &= ~(ulTimer & (TIMER_CTL_TAEN | TIMER_CTL_TBEN));
}

/*****************************************************************************
* TIMER工作模式设置(TimerA TimerB)
* TIMERx 定时器结构体
* 人口:ulConfig bit0~bit7 设置TAMR寄存器 bit8~bit15 设置TBMR bit24~bit32 设置 CFG 寄存器 
* TIMER_CFG_32_BIT_OS TIMER_CFG_32_BIT_PER TIMER_CFG_32_RTC 设置成32位定时器模式
* TIMER_CFG_16_BIT_PAIR 设置成双16位定时器组
* TIMER_CFG_A_ONE_SHOT... TimerA工作模式
* TIMER_CFG_B_ONE_SHOT... TimerB工作模式
*****************************************************************************/
void TimerConfigure(TIMER_Typedef *TIMERx, u32 ulConfig)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulConfig == TIMER_CFG_32_BIT_OS) ||
           (ulConfig == TIMER_CFG_32_BIT_PER) ||
           (ulConfig == TIMER_CFG_32_RTC) ||
           ((ulConfig & 0xff000000) == TIMER_CFG_16_BIT_PAIR));
    ASSERT(((ulConfig & 0xff000000) != TIMER_CFG_16_BIT_PAIR) ||
           ((((ulConfig & 0x000000ff) == TIMER_CFG_A_ONE_SHOT) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_PERIODIC) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_CAP_COUNT) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_CAP_TIME) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_PWM)) &&
            (((ulConfig & 0x0000ff00) == TIMER_CFG_B_ONE_SHOT) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_PERIODIC) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_CAP_COUNT) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_CAP_TIME) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_PWM))));
    TIMERx->CTL &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN); //先禁能
    TIMERx->CFG = ulConfig >> 24;                      //
    TIMERx->TAMR = ulConfig & 255;
    TIMERx->TBMR = (ulConfig >> 8) & 255;
}

/*****************************************************************************
* 控制 TIMER PWM 输出电平
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 人口:bInvert输出电平 1 输出反相 0 保持不变
*****************************************************************************/
void TimerControlLevel(TIMER_Typedef *TIMERx, u32 ulTimer, u8 bInvert)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    ulTimer &= TIMER_CTL_TAPWML | TIMER_CTL_TBPWML;
    TIMERx->CTL = (bInvert ? (TIMERx->CTL | ulTimer) : (TIMERx->CTL & ~(ulTimer)));
}

/*****************************************************************************
* 控制 TIMER 触发器是否使能
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 人口:bEnable触发设置 1 触发使能 0 触发禁能
* 触发ADC采样 设置
*****************************************************************************/
void TimerControlTrigger(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bEnable)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    ulTimer &= TIMER_CTL_TAOTE | TIMER_CTL_TBOTE;
    TIMERx->CTL = (bEnable ? (TIMERx->CTL | ulTimer) : (TIMERx->CTL & ~(ulTimer)));
}

/*****************************************************************************
* 设置 TIMER 触发事件
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 人口:ulEvent触发事件设置
*      TIMER_EVENT_POS_EDGE 上升沿
*      TIMER_EVENT_NEG_EDGE 下降沿
*      TIMER_EVENT_BOTH_EDGES 双边沿
*****************************************************************************/
void TimerControlEvent(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulEvent)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    if(ulTimer == TIMER_A)
	 {
      ulEvent &= TIMER_CTL_TAEVENT_M;
      TIMERx->CTL = ((TIMERx->CTL & ~TIMER_CTL_TAEVENT_M)|ulEvent);
	 }							    
    else if(ulTimer == TIMER_B)
	 {
      ulEvent &= TIMER_CTL_TBEVENT_M;
      TIMERx->CTL = ((TIMERx->CTL & ~TIMER_CTL_TBEVENT_M)|ulEvent);
	 }
	else if(ulTimer == TIMER_BOTH)
	 {
      ulEvent &= TIMER_CTL_TBEVENT_M;
      TIMERx->CTL = ((TIMERx->CTL & ~(TIMER_CTL_TAEVENT_M|TIMER_CTL_TBEVENT_M))|ulEvent);
	 }
}

/*****************************************************************************
* 设置 调试模式 是否允许TIMER 停止
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 人口:bStall
*      true  使能Timer停止
*      false 禁止Timer停止
*****************************************************************************/
void TimerControlStall(TIMER_Typedef *TIMERx, u32 ulTimer,u8  bStall)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    ulTimer &= TIMER_CTL_TASTALL | TIMER_CTL_TBSTALL;
    TIMERx->CTL = (bStall ? (TIMERx->CTL | ulTimer) : (TIMERx->CTL & ~(ulTimer)));
}

/*****************************************************************************
* RTC 模式下Timer计数使能
* TIMERx 定时器结构体
*****************************************************************************/
void TimerRTCEnable(TIMER_Typedef *TIMERx)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->CTL |= TIMER_CTL_RTCEN;
}
/*****************************************************************************
* RTC 模式下Timer计数禁能(停止)
* TIMERx 定时器结构体
*****************************************************************************/
void TimerRTCDisable(TIMER_Typedef *TIMERx)
{
    ASSERT(TimerBaseValid(TIMERx));

    TIMERx->CTL &= ~(TIMER_CTL_RTCEN);
}
/*****************************************************************************
* 设置 Timer预分频 
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 人口:ulValue 分频值(0~255)
*****************************************************************************/
void TimerPrescaleSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    ASSERT(ulValue < 256);

    if(ulTimer & TIMER_A)
     TIMERx->TAPR = ulValue;
    if(ulTimer & TIMER_B)
     TIMERx->TBPR = ulValue;
}
/*****************************************************************************
* 获取 Timer预分频值
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 返回 分频值(0~255)
*****************************************************************************/
u32 TimerPrescaleGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    return((ulTimer == TIMER_A) ? TIMERx->TAPR :TIMERx->TBPR);
}
/*****************************************************************************
* 设置 Timer 间隔装载值
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* ulValue 间隔装载值
*****************************************************************************/
void TimerLoadSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    if(ulTimer & TIMER_A)
     TIMERx->TAILR = ulValue;
    if(ulTimer & TIMER_B)
     TIMERx->TBILR = ulValue;
}
/*****************************************************************************
* 获取 Timer 间隔装载值
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 返回 间隔装载值
*****************************************************************************/
u32 TimerLoadGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B));
    return((ulTimer == TIMER_A) ? TIMERx->TAILR :TIMERx->TBILR);
}
/*****************************************************************************
* 获取 Timer 当前计数值
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 返回 当前计数值
*****************************************************************************/
u32 TimerValueGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B));
    return((ulTimer == TIMER_A) ? TIMERx->TAR : TIMERx->TBR);
}
/*****************************************************************************
* 设置 Timer 匹配值
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* ulValue 匹配值
*****************************************************************************/
void TimerMatchSet(TIMER_Typedef *TIMERx, u32 ulTimer, u32 ulValue)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    if(ulTimer & TIMER_A)
     TIMERx->TAMATCHR = ulValue;
    if(ulTimer & TIMER_B)
     TIMERx->TBMATCHR = ulValue;
}
/*****************************************************************************
* 获取 Timer 匹配值
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 返回 匹配值
*****************************************************************************/
u32 TimerMatchGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B));
    return((ulTimer == TIMER_A) ? TIMERx->TAMATCHR :  TIMERx->TBMATCHR);
}
/*****************************************************************************
* 设置Timer中断复位程序入口
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* pfnHandler 服务程序入口
* 使能UART NVIC中断 中断向量表在RAM中时 使用
*****************************************************************************/
void TimerIntRegister(TIMER_Typedef *TIMERx, u32 ulTimer,void (*pfnHandler)(void))
{
    u32  Int;
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    Int = (((u32)TIMERx == TIMER0_BASE) ? INT_TIMER0A :
           (((u32)TIMERx == TIMER1_BASE) ? INT_TIMER1A :
            (((u32)TIMERx == TIMER2_BASE) ? INT_TIMER2A : INT_TIMER3A)));

    if(ulTimer & TIMER_A)
     {
      IntRegister(Int, pfnHandler);
      IntEnable(Int);
     }
    if(ulTimer & TIMER_B)
     {
      IntRegister(Int + 1, pfnHandler);
      IntEnable(Int + 1);
     }
}
/*****************************************************************************
* 撤销Timer中断复位程序入口
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 禁能UART NVIC中断 中断向量表在RAM中时 使用
*****************************************************************************/
void TimerIntUnregister(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    u32 Int;
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    Int = (((u32)TIMERx == TIMER0_BASE) ? INT_TIMER0A :
           (((u32)TIMERx == TIMER1_BASE) ? INT_TIMER1A :
           (((u32)TIMERx == TIMER2_BASE) ? INT_TIMER2A : INT_TIMER3A)));
    if(ulTimer & TIMER_A)
     {
      IntDisable(Int);
      IntUnregister(Int);
     }
    if(ulTimer & TIMER_B)
     {
      IntDisable(Int + 1);
      IntUnregister(Int + 1);
     }
}
/*****************************************************************************
* Timer中断源使能设置
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 人口:ulIntFlags 中断使能
*                 TIMER_CAPB_EVENT ...
*                 TIMER_CAPA_EVENT ...
*****************************************************************************/
void TimerIntEnable(TIMER_Typedef *TIMERx, u32 ulIntFlags)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->IMR |= ulIntFlags;
}

/*****************************************************************************
* Timer中断源禁能设置
* TIMERx 定时器结构体
* 人口:ulTimer 为TIMER_A TIMER_B 或TIMER_BOTH
* 人口:ulIntFlags 中断禁能
*                 TIMER_CAPB_EVENT ...
*                 TIMER_CAPA_EVENT ...
*****************************************************************************/
void TimerIntDisable(TIMER_Typedef *TIMERx, u32 ulIntFlags)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->IMR &= ~(ulIntFlags);
}
/*****************************************************************************
* 获取Timer中断状态
* TIMERx 定时器结构体
*  bMasked=1 读取屏蔽后的中断状态
*  bMasked=1 读取原始的中断状态
*****************************************************************************/
u32 TimerIntStatus(TIMER_Typedef *TIMERx, u8 bMasked)
{
    ASSERT(TimerBaseValid(TIMERx));
    return(bMasked ? TIMERx->MIS : TIMERx->RIS);
}

/*****************************************************************************
* 清除Timer中断状态
* TIMERx 定时器结构体
* ulIntFlags 要清除的中断标志
*****************************************************************************/
void TimerIntClear(TIMER_Typedef *TIMERx, u32 ulIntFlags)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->ICR = ulIntFlags;
}

/*****************************************************************************
* Timer 复位
* TIMERx 定时器结构体
* 复位后
*****************************************************************************/
void Reset_Timer(TIMER_Typedef *TIMERx)
{
    u32 Int;
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->CTL = TIMER_RV_CTL;
    TIMERx->IMR = TIMER_RV_IMR;
    TIMERx->ICR = 0xFFFFFFFF;
    Int = (((u32)TIMERx == TIMER0_BASE) ? INT_TIMER0A :
           (((u32)TIMERx == TIMER1_BASE) ? INT_TIMER1A :
           (((u32)TIMERx == TIMER2_BASE) ? INT_TIMER2A : INT_TIMER3A)));
    IntDisable(Int);
    IntDisable(Int + 1);
    TIMERx->CFG = TIMER_RV_CFG;
    TIMERx->TAMR = TIMER_RV_TAMR;
    TIMERx->TBMR = TIMER_RV_TBMR;
    TIMERx->RIS = TIMER_RV_RIS;
    TIMERx->MIS = TIMER_RV_MIS;
    TIMERx->TAILR = TIMER_RV_TAILR;
    TIMERx->TBILR = TIMER_RV_TBILR;
    TIMERx->TAMATCHR = TIMER_RV_TAMATCHR;
    TIMERx->TBMATCHR = TIMER_RV_TBMATCHR;
    TIMERx->TAPR = TIMER_RV_TAPR;
    TIMERx->TBPR = TIMER_RV_TBPR;
    TIMERx->TAPMR = TIMER_RV_TAPMR;
    TIMERx->TBPMR = TIMER_RV_TBPMR;
    TIMERx->TAR = TIMER_RV_TAR;
    TIMERx->TBR = TIMER_RV_TBR;
}

