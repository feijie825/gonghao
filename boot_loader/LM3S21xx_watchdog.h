/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_watchdog.h
;* Author             : ������
;* ���Ź�����Ԥ����	
;* ����LM3S21xx_watchdog.c�ļ��еĺ�������
*******************************************************************************/
#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern u8 WatchdogRunning(void);
extern void WatchdogEnable(void);
extern void WatchdogResetEnable(void);
extern void WatchdogResetDisable(void);
extern void WatchdogLock(void);
extern void WatchdogUnlock(void);
extern u8 WatchdogLockState(void);
extern void WatchdogReloadSet(u32 ulLoadVal);
extern u32 WatchdogReloadGet(void);
extern u32 WatchdogValueGet(void);
extern void WatchdogIntRegister(void(*pfnHandler)(void));
extern void WatchdogIntUnregister(void);
extern void WatchdogIntEnable(void);
extern u32 WatchdogIntStatus(u8 bMasked);
extern void WatchdogIntClear(void);
extern void WatchdogStallEnable(void);
extern void WatchdogStallDisable(void);

extern void Init_Wdt(void);	   // ��ʼ�����Ź�
extern void WDTFeed(void);	   //ι������

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __WATCHDOG_H__
