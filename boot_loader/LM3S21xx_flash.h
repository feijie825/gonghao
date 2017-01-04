/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_flash.h
;* Author             : 张力阵
;* flash 驱动库声明和预定义
*******************************************************************************/

#ifndef __FLASH_H__
#define __FLASH_H__

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
// Values that can be passed to FlashProtectSet(), and returned by
// FlashProtectGet().
//
//*****************************************************************************
typedef enum
{
    FlashReadWrite,                         // Flash can be read and written
    FlashReadOnly,                          // Flash can only be read
    FlashExecuteOnly                        // Flash can only be executed
}
tFlashProtection;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern u32 FlashUsecGet(void);
extern void FlashUsecSet(u32 ulClocks);
extern s32 FlashErase(u32 ulAddress);
extern s32 FlashProgram(u32 *pulData, u32 ulAddress,u32 ulCount);
extern tFlashProtection FlashProtectGet(u32 ulAddress);
extern s32 FlashProtectSet(u32 ulAddress,tFlashProtection eProtect);
extern s32 FlashProtectSave(void);
extern s32 FlashUserGet(u32 *pulUser0, u32 *pulUser1);
extern s32 FlashUserSet(u32 ulUser0, u32 ulUser1);
extern s32 FlashUserSave(void);
extern void FlashIntRegister(void (*pfnHandler)(void));
extern void FlashIntUnregister(void);
extern void FlashIntEnable(u32 ulIntFlags);
extern void FlashIntDisable(u32 ulIntFlags);
extern u32 FlashIntGetStatus(u8 bMasked);
extern void FlashIntClear(u32 ulIntFlags);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __FLASH_H__
