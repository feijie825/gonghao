/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_Lib.h
;* Author             : 张力阵
;* 函数库声明预定义
*******************************************************************************/
#include "LM3S2139.h"

#define _ADC			 //2012.11.3 ZLZ 添加
#define _CAN
#define _FLASH
#define _GPIO
#define _INTERRUPT
#define _MPU
#define _SSI
#define _SYSCTL
#define _SYSTICK
#define _TIMER
#define _UART
#define _WATCHDOG
#define _DEBUG
#define _CPU

#ifdef _ADC
#include "LM3S21xx_adc.h"
#endif

#ifdef _CAN
#include "LM3S21xx_can.h"
#endif

#ifdef _FLASH
#include "LM3S21xx_flash.h"
#endif

#ifdef _GPIO
#include "LM3S21xx_gpio.h"
#endif

#ifdef _INTERRUPT
#include "LM3S21xx_interrupt.h"
#endif

#ifdef _MPU
#include "LM3S21xx_mpu.h"
#endif

#ifdef _SSI
#include "LM3S21xx_ssi.h"
#endif

#ifdef _SYSCTL
#include "LM3S21xx_sysctl.h"
#endif

#ifdef _SYSTICK
#include "LM3S21xx_systick.h"
#endif

#ifdef _TIMER
#include "LM3S21xx_timer.h"
#endif

#ifdef _UART
#include "LM3S21xx_uart.h"
#endif

#ifdef _WATCHDOG
#include "LM3S21xx_watchdog.h"
#endif

#ifdef _DEBUG
#include "LM3S21xx_debug.h"
#endif

#ifdef _CPU
#include "LM3S21xx_cpu.h"
#endif
