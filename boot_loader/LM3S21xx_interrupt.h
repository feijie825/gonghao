/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_interrupt.h
;* Author             : ������
;* �жϹ���������������
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
//�ж����ȼ�����
#define INT_PRIORITY_0          0x00//���ȼ�0 ���
#define INT_PRIORITY_1          0x20//���ȼ�1 
#define INT_PRIORITY_2          0x40//���ȼ�2 
#define INT_PRIORITY_3          0x60//���ȼ�3 
#define INT_PRIORITY_4          0x80//���ȼ�4
#define INT_PRIORITY_5          0xa0//���ȼ�5
#define INT_PRIORITY_6          0xc0//���ȼ�6
#define INT_PRIORITY_7          0xe0//���ȼ�7 ���

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
extern void Init_Int(void);      //��ʼ���ж�

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_H__
