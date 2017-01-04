/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_interrupt.h
;* Author             : 张力阵
;* 中断管理驱动程序声明
*******************************************************************************/

#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

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
// Macro to generate an interrupt priority mask based on the number of bits
// of priority supported by the hardware.
//
//*****************************************************************************
#define INT_PRIORITY_MASK       ((0xFF << (8 - NUM_PRIORITY_BITS)) & 0xFF)
//中断优先级定义
#define INT_PRIORITY_0          0x00//优先级0 最高
#define INT_PRIORITY_1          0x20//优先级1 
#define INT_PRIORITY_2          0x40//优先级2 
#define INT_PRIORITY_3          0x60//优先级3 
#define INT_PRIORITY_4          0x80//优先级4
#define INT_PRIORITY_5          0xa0//优先级5
#define INT_PRIORITY_6          0xc0//优先级6
#define INT_PRIORITY_7          0xe0//优先级7 最低

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern u8 IntMasterEnable(void);
extern u8 IntMasterDisable(void);
extern void IntRegister(u32 ulInterrupt, void (*pfnHandler)(void));
extern void IntUnregister(u32 ulInterrupt);
extern void IntPriorityGroupingSet(u32 ulBits);
extern u32 IntPriorityGroupingGet(void);
extern void IntPrioritySet(u32 ulInterrupt,u8 ucPriority);
extern u32 IntPriorityGet(u32 ulInterrupt);
extern void IntEnable(u32 ulInterrupt);
extern void IntDisable(u32 ulInterrupt);
extern void Init_Int(void);      //初始化中断

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_H__
