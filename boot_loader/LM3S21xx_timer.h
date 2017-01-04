/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_timer.h
;* Author             : 张力阵
;* 定时/计数器驱动程序声明和预定义
*******************************************************************************/

#ifndef __TIMER_H__
#define __TIMER_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Values that can be passed to TimerConfigure as the ulConfig parameter.
//
//*****************************************************************************
#define TIMER_CFG_32_BIT_OS     0x00000001  // 32-bit one-shot timer
#define TIMER_CFG_32_BIT_PER    0x00000002  // 32-bit periodic timer
#define TIMER_CFG_32_RTC        0x01000000  // 32-bit RTC timer
#define TIMER_CFG_16_BIT_PAIR   0x04000000  // Two 16-bit timers
#define TIMER_CFG_A_ONE_SHOT    0x00000001  // Timer A 单次触发模式            TAAMS=0(CCPx 为捕获模式) TACMR=0(沿计数模式) TAMR=1(单次触发模式)
#define TIMER_CFG_A_PERIODIC    0x00000002  // Timer A 周期定时模式            TAAMS=0(CCPx 为捕获模式) TACMR=0(沿计数模式) TAMR=2(周期定时模式)
#define TIMER_CFG_A_CAP_COUNT   0x00000003  // Timer A 事件计数模式            TAAMS=0(CCPx 为捕获模式) TACMR=0(沿计数模式) TAMR=3(捕获模式)
#define TIMER_CFG_A_CAP_TIME    0x00000007  // Timer A 事件定时(捕获外部事件)  TAAMS=0(CCPx 为捕获模式) TACMR=1(沿定时模式) TAMR=3(捕获模式)
#define TIMER_CFG_A_PWM         0x0000000A  // Timer A PWM输出                 TAAMS=1(CCPx 为PWM 模式) TACMR=0(沿计数模式) TAMR=2(周期定时模式)
#define TIMER_CFG_B_ONE_SHOT    0x00000100  // Timer B one-shot timer
#define TIMER_CFG_B_PERIODIC    0x00000200  // Timer B periodic timer
#define TIMER_CFG_B_CAP_COUNT   0x00000300  // Timer B event counter
#define TIMER_CFG_B_CAP_TIME    0x00000700  // Timer B event timer
#define TIMER_CFG_B_PWM         0x00000A00  // Timer B PWM output

//*****************************************************************************
//
// Values that can be passed to TimerIntEnable, TimerIntDisable, and
// TimerIntClear as the ulIntFlags parameter, and returned from TimerIntStatus.
//
//*****************************************************************************
#define TIMER_CAPB_EVENT        0x00000400  // CaptureB event interrupt
#define TIMER_CAPB_MATCH        0x00000200  // CaptureB match interrupt
#define TIMER_TIMB_TIMEOUT      0x00000100  // TimerB time out interrupt
#define TIMER_RTC_MATCH         0x00000008  // RTC interrupt mask
#define TIMER_CAPA_EVENT        0x00000004  // CaptureA event interrupt
#define TIMER_CAPA_MATCH        0x00000002  // CaptureA match interrupt
#define TIMER_TIMA_TIMEOUT      0x00000001  // TimerA time out interrupt

//*****************************************************************************
//
// Values that can be passed to TimerControlEvent as the ulEvent parameter.
//
//*****************************************************************************
#define TIMER_EVENT_POS_EDGE    0x00000000  // Count positive edges
#define TIMER_EVENT_NEG_EDGE    0x00000404  // Count negative edges
#define TIMER_EVENT_BOTH_EDGES  0x00000C0C  // Count both edges

//*****************************************************************************
//
// Values that can be passed to most of the timer APIs as the ulTimer
// parameter.
//
//*****************************************************************************
#define TIMER_A                 0x000000ff  // Timer A
#define TIMER_B                 0x0000ff00  // Timer B
#define TIMER_BOTH              0x0000ffff  // Timer Both

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void TimerEnable(TIMER_Typedef *TIMERx, u32 ulTimer);
extern void TimerDisable(TIMER_Typedef *TIMERx, u32 ulTimer);
extern void TimerConfigure(TIMER_Typedef *TIMERx, u32 ulConfig);
extern void TimerControlLevel(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bInvert);
extern void TimerControlTrigger(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bEnable);
extern void TimerControlEvent(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulEvent);
extern void TimerControlStall(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bStall);
extern void TimerRTCEnable(TIMER_Typedef *TIMERx);
extern void TimerRTCDisable(TIMER_Typedef *TIMERx);
extern void TimerPrescaleSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue);
extern u32 TimerPrescaleGet(TIMER_Typedef *TIMERx,u32 ulTimer);
extern void TimerLoadSet(TIMER_Typedef *TIMERx, u32 ulTimer, u32 ulValue);
extern u32 TimerLoadGet(TIMER_Typedef *TIMERx, u32 ulTimer);
extern u32 TimerValueGet(TIMER_Typedef *TIMERx,u32 ulTimer);
extern void TimerMatchSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue);
extern u32 TimerMatchGet(TIMER_Typedef *TIMERx,u32 ulTimer);
extern void TimerIntRegister(TIMER_Typedef *TIMERx, u32 ulTimer, void (*pfnHandler)(void));
extern void TimerIntUnregister(TIMER_Typedef *TIMERx, u32 ulTimer);
extern void TimerIntEnable(TIMER_Typedef *TIMERx, u32 ulIntFlags);
extern void TimerIntDisable(TIMER_Typedef *TIMERx, u32 ulIntFlags);
extern u32 TimerIntStatus(TIMER_Typedef *TIMERx, u8 bMasked);
extern void TimerIntClear(TIMER_Typedef *TIMERx, u32 ulIntFlags);
extern void Init_Timer(void);      //初始化定时器

//*****************************************************************************
//
// TimerQuiesce() has been deprecated.  SysCtlPeripheralReset() should be used
// instead to return the timer to its reset state.
//
//*****************************************************************************
#ifndef DEPRECATED
extern void TimerQuiesce(TIMER_Typedef *TIMERx);
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __TIMER_H__
