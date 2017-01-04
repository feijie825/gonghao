/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_uart.h
;* Author             : 张力阵
;* 串口驱动程序声明和预定义
*******************************************************************************/

#ifndef __UART_H__
#define __UART_H__

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
// Values that can be passed to UARTIntEnable, UARTIntDisable, and UARTIntClear
// as the ulIntFlags parameter, and returned from UARTIntStatus.
//
//*****************************************************************************
#define UART_INT_OE             0x400       // Overrun Error Interrupt Mask
#define UART_INT_BE             0x200       // Break Error Interrupt Mask
#define UART_INT_PE             0x100       // Parity Error Interrupt Mask
#define UART_INT_FE             0x080       // Framing Error Interrupt Mask
#define UART_INT_RT             0x040       // Receive Timeout Interrupt Mask
#define UART_INT_TX             0x020       // Transmit Interrupt Mask
#define UART_INT_RX             0x010       // Receive Interrupt Mask

//*****************************************************************************
//
// Values that can be passed to UARTConfigSetExpClk as the ulConfig parameter
// and returned by UARTConfigGetExpClk in the pulConfig parameter.
// Additionally, the UART_CONFIG_PAR_* subset can be passed to
// UARTParityModeSet as the ulParity parameter, and are returned by
// UARTParityModeGet.
//
//*****************************************************************************
#define UART_CONFIG_WLEN_MASK   0x00000060  // Mask for extracting word length
#define UART_CONFIG_WLEN_8      0x00000060  // 8 bit data
#define UART_CONFIG_WLEN_7      0x00000040  // 7 bit data
#define UART_CONFIG_WLEN_6      0x00000020  // 6 bit data
#define UART_CONFIG_WLEN_5      0x00000000  // 5 bit data
#define UART_CONFIG_STOP_MASK   0x00000008  // Mask for extracting stop bits
#define UART_CONFIG_STOP_ONE    0x00000000  // One stop bit
#define UART_CONFIG_STOP_TWO    0x00000008  // Two stop bits
#define UART_CONFIG_PAR_MASK    0x00000086  // Mask for extracting parity
#define UART_CONFIG_PAR_NONE    0x00000000  // 无校验
#define UART_CONFIG_PAR_EVEN    0x00000006  // 偶校验
#define UART_CONFIG_PAR_ODD     0x00000002  // 奇校验
#define UART_CONFIG_PAR_ONE     0x00000086  // 校验位为1
#define UART_CONFIG_PAR_ZERO    0x00000082  // 校验位为0

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ulTxLevel parameter and
// returned by UARTFIFOLevelGet in the pulTxLevel.
//
//*****************************************************************************
#define UART_FIFO_TX1_8         0x00000000  // Transmit interrupt at 1/8 Full
#define UART_FIFO_TX2_8         0x00000001  // Transmit interrupt at 1/4 Full
#define UART_FIFO_TX4_8         0x00000002  // Transmit interrupt at 1/2 Full
#define UART_FIFO_TX6_8         0x00000003  // Transmit interrupt at 3/4 Full
#define UART_FIFO_TX7_8         0x00000004  // Transmit interrupt at 7/8 Full

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ulRxLevel parameter and
// returned by UARTFIFOLevelGet in the pulRxLevel.
//
//*****************************************************************************
#define UART_FIFO_RX1_8         0x00000000  // Receive interrupt at 1/8 Full
#define UART_FIFO_RX2_8         0x00000008  // Receive interrupt at 1/4 Full
#define UART_FIFO_RX4_8         0x00000010  // Receive interrupt at 1/2 Full
#define UART_FIFO_RX6_8         0x00000018  // Receive interrupt at 3/4 Full
#define UART_FIFO_RX7_8         0x00000020  // Receive interrupt at 7/8 Full

//*****************************************************************************
//
// Values that can be passed to UARTDMAEnable() and UARTDMADisable().
//
//*****************************************************************************
#define UART_DMA_ERR_RXSTOP     0x00000004  // Stop DMA receive if UART error
#define UART_DMA_TX             0x00000002  // Enable DMA for transmit
#define UART_DMA_RX             0x00000001  // Enable DMA for receive

//*****************************************************************************
//
// Values returned from UARTRxErrorGet().
//
//*****************************************************************************
#define UART_RXERROR_OVERRUN    0x00000008
#define UART_RXERROR_BREAK      0x00000004
#define UART_RXERROR_PARITY     0x00000002
#define UART_RXERROR_FRAMING    0x00000001

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern void UARTParityModeSet(UART_Typedef *UARTx, u32 ulParity);
extern u32 UARTParityModeGet(UART_Typedef *UARTx);
extern void UARTFIFOLevelSet(UART_Typedef *UARTx, u32 ulTxLevel,u32 ulRxLevel);
extern void UARTFIFOLevelGet(UART_Typedef *UARTx, u32 *pulTxLevel,u32 *pulRxLevel);
extern void UARTConfigSetExpClk(UART_Typedef *UARTx, u32 ulUARTClk,u32 ulBaud, u32 ulConfig);
extern void UARTConfigGetExpClk(UART_Typedef *UARTx, u32 ulUARTClk,u32 *pulBaud,u32 *pulConfig);
extern void UARTEnable(UART_Typedef *UARTx);
extern void UARTDisable(UART_Typedef *UARTx);
extern void UARTEnableSIR(UART_Typedef *UARTx, u8 bLowPower);
extern void UARTDisableSIR(UART_Typedef *UARTx);
extern u8 UARTCharsAvail(UART_Typedef *UARTx);
extern u8 UARTSpaceAvail(UART_Typedef *UARTx);
extern s32 UARTCharGetNonBlocking(UART_Typedef *UARTx);
extern s32 UARTCharGet(UART_Typedef *UARTx);
extern u8 UARTCharPutNonBlocking(UART_Typedef *UARTx,u8 ucData);
extern void UARTCharPut(UART_Typedef *UARTx, u8 ucData);
extern void UARTBreakCtl(UART_Typedef *UARTx, u8 bBreakState);
extern u8 UARTBusy(UART_Typedef *UARTx);
extern void UARTIntRegister(UART_Typedef *UARTx, void(*pfnHandler)(void));
extern void UARTIntUnregister(UART_Typedef *UARTx);
extern void UARTIntEnable(UART_Typedef *UARTx, u32 ulIntFlags);
extern void UARTIntDisable(UART_Typedef *UARTx, u32 ulIntFlags);
extern u32 UARTIntStatus(UART_Typedef *UARTx, u8 bMasked);
extern void UARTIntClear(UART_Typedef *UARTx, u32 ulIntFlags);
extern void UARTDMAEnable(UART_Typedef *UARTx, u32 ulDMAFlags);
extern void UARTDMADisable(UART_Typedef *UARTx, u32 ulDMAFlags);
extern u32 UARTRxErrorGet(UART_Typedef *UARTx);
extern void UARTRxErrorClear(UART_Typedef *UARTx);
extern void Init_Uart(void);              //串口初始化
/*****************************************************************************
* 串口初始化
*****************************************************************************/
extern void Init_Com(u8 COM);
/*****************************************************************************
* COM发送缓冲区处理
*****************************************************************************/
extern void Proc_Com_OBuf(u8 COM);
/*****************************************************************************
* COM0接收缓冲区处理
*****************************************************************************/
extern void Proc_Com_IBuf(u8 COM);   
/*****************************************************************************
* COM接收超时处理
*****************************************************************************/
extern void Proc_Com_Rx_OvTm(u8 COM);

//*****************************************************************************
//
// Several UART APIs have been renamed, with the original function name being
// deprecated.  These defines provide backward compatibility.
//
//*****************************************************************************
#ifndef DEPRECATED
#include "lm3s21xx_sysctl.h"
#define UARTConfigSet(a, b, c)                         \
        UARTConfigSetExpClk(a, SysCtlClockGet(), b, c)
#define UARTConfigGet(a, b, c)                         \
        UARTConfigGetExpClk(a, SysCtlClockGet(), b, c)
#define UARTCharNonBlockingGet(a) \
        UARTCharGetNonBlocking(a)
#define UARTCharNonBlockingPut(a, b) \
        UARTCharPutNonBlocking(a, b)
#endif
//CAN 接口地址表格
extern const u32 UART_PORT[];

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __UART_H__
