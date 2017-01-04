/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_ssi.h
;* Author             : 张力阵
;* 同步串行接口声明和预定义
*******************************************************************************/

#ifndef __SSI_H__
#define __SSI_H__

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

//定义SSI位速率
#define SSI_BITRATE            16667        //16.667K

//*****************************************************************************
//
// Values that can be passed to SSIIntEnable, SSIIntDisable, and SSIIntClear
// as the ulIntFlags parameter, and returned by SSIIntStatus.
//
//*****************************************************************************
#define SSI_TXFF                0x00000008  // TX FIFO half empty or less
#define SSI_RXFF                0x00000004  // RX FIFO half full or less
#define SSI_RXTO                0x00000002  // RX timeout
#define SSI_RXOR                0x00000001  // RX overrun

//*****************************************************************************
//
// Values that can be passed to SSIConfigSetExpClk.
//
//*****************************************************************************
#define SSI_FRF_MOTO_MODE_0     0x00000000  // Moto fmt, polarity 0, phase 0
#define SSI_FRF_MOTO_MODE_1     0x00000002  // Moto fmt, polarity 0, phase 1
#define SSI_FRF_MOTO_MODE_2     0x00000001  // Moto fmt, polarity 1, phase 0
#define SSI_FRF_MOTO_MODE_3     0x00000003  // Moto fmt, polarity 1, phase 1
#define SSI_FRF_TI              0x00000010  // TI frame format
#define SSI_FRF_NMW             0x00000020  // National MicroWire frame format

#define SSI_MODE_MASTER         0x00000000  // SSI master
#define SSI_MODE_SLAVE          0x00000001  // SSI slave
#define SSI_MODE_SLAVE_OD       0x00000002  // SSI slave with output disabled

//*****************************************************************************
//
// Values that can be passed to SSIDMAEnable() and SSIDMADisable().
//
//*****************************************************************************
#define SSI_DMA_TX              0x00000002  // Enable DMA for transmit
#define SSI_DMA_RX              0x00000001  // Enable DMA for receive

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void SSIConfigSetExpClk(SSI_Typedef *SSIx, u32 ulSSIClk,
                               u32 ulProtocol, u32 ulMode,
                               u32 ulBitRate,u32 ulDataWidth);
extern void SSIDataGet(SSI_Typedef *SSIx, u32 *pulData);
extern u32 SSIDataGetNonBlocking(SSI_Typedef *SSIx,u32 *pulData);
extern void SSIDataPut(SSI_Typedef *SSIx, u32 ulData);
extern u32 SSIDataPutNonBlocking(SSI_Typedef *SSIx, u32 ulData);
extern void SSIDisable(SSI_Typedef *SSIx);
extern void SSIEnable(SSI_Typedef *SSIx);
extern void SSIIntClear(SSI_Typedef *SSIx, u32 ulIntFlags);
extern void SSIIntDisable(SSI_Typedef *SSIx, u32 ulIntFlags);
extern void SSIIntEnable(SSI_Typedef *SSIx, u32 ulIntFlags);
extern void SSIIntRegister(SSI_Typedef *SSIx, void(*pfnHandler)(void));
extern u32 SSIIntStatus(SSI_Typedef *SSIx, u8 bMasked);
extern void SSIIntUnregister(SSI_Typedef *SSIx);
extern void SSIDMAEnable(SSI_Typedef *SSIx, u32 ulDMAFlags);
extern void SSIDMADisable(SSI_Typedef *SSIx, u32 ulDMAFlags);
extern void Init_Ssi(void);    //初始化SSI
void SSIDataLen(SSI_Typedef *SSIx,u8 Len);//设置SSI数据长度

//*****************************************************************************
//
// Several SSI APIs have been renamed, with the original function name being
// deprecated.  These defines provide backward compatibility.
//
//*****************************************************************************
#ifndef DEPRECATED
#include "lm3s21xx_sysctl.h"
#define SSIConfig(a, b, c, d, e)                            \
        SSIConfigSetExpClk(a, SysCtlClockGet(), b, c, d, e)
#define SSIDataNonBlockingGet(a, b) \
        SSIDataGetNonBlocking(a, b)
#define SSIDataNonBlockingPut(a, b) \
        SSIDataPutNonBlocking(a, b)
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __SSI_H__
