/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_cpu.h
;* Author             : 张力阵
;* cpu 预定义
*******************************************************************************/

#ifndef __CPU_H__
#define __CPU_H__

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
// Prototypes.
//
//*****************************************************************************
extern unsigned long CPUcpsid(void);
extern unsigned long CPUcpsie(void);
extern void CPUwfi(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __CPU_H__
