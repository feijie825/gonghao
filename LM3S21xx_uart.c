/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_uart.c
;* Author             : 张力阵
;* 串口驱动程序
;* UART 发送FIFO 使能 一次可向发送缓冲区写入7字节数据 发送缓冲区不超过1字节时申请中断 请求发送
;* UART 接收FIFO 使能 接收缓冲区不小于7字节时请求中断 或接收超时中断
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#define UART_CLK_DIVIDER 16
/*****************************************************************************
* 校验设置对应列表
*****************************************************************************/
const u8 Parity_Set_TAB[5]=
{
    0x10,           //无校验位 'N' FIFO 使能
    0x12,           //奇校验   'O' FIFO 使能
    0x16,           //偶校验   'E' FIFO 使能
    0x92,           //Mark校验 'M' FIFO 使能 校验位固定为1
    0x96            //Space校验'S' FIFO 使能 校验位固定为0
};
/*****************************************************************************
* 串口默认参数列表 MTRCOM LCTCOM
*****************************************************************************/
const UART_SET UART_PARA_TAB[UART_NUMB]=
{           
    {
     UART0_BAUD,  //波特率
     UART_8_BIT,  //数据位数
     UART_1_STOP, //停止位数
     UART_N_PARITY//校验位
    },
    {
     UART1_BAUD,  //波特率
     UART_8_BIT,  //数据位数
     UART_1_STOP, //停止位数
     UART_N_PARITY//校验位
    },
};
/*****************************************************************************
* 串口初始化
* 人口 Com:端口号
*****************************************************************************/
void Init_Com(u8 Com)
{
    UART_PARA UART_P;
    UART_Typedef *UARTx;
    UARTx = (UART_Typedef *)UART_PORT[Com];
    *((u32*)&UART_P) = Parity_Set_TAB[Uart_Para[Com].PARITY];//校验位
    UART_P.Data_Len=Uart_Para[Com].LEN;              //数据位数
    UART_P.Stop2=Uart_Para[Com].STOP;                //停止位	 
    UARTConfigSetExpClk(UARTx,Sysclk,Uart_Para[Com].BAUD,*((u32*)&UART_P));
    UARTFIFOLevelSet(UARTx,UART_FIFO_TX1_8,          //发送FIFO 中断触发深度1/8
                           UART_FIFO_RX7_8);         //接收FIFO 中断触发深度7/8
    UARTIntEnable(UARTx,UART_INT_RX |                //开启接收中断 
                        UART_INT_RT|                 //开启接收超时中断
                        UART_INT_TX );	              //开启发送中断
}
/*****************************************************************************
* 串口初始化
* 管脚已在Init_Gpio()中设置
*****************************************************************************/
void Init_Uart(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);       //UART0设备时钟使能
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);       //UART1设备时钟使能
    memcpy(&Uart_Para[MTRCOM],                         //MTRCOM参数
           &UART_PARA_TAB[MTRCOM],
           4);
    Init_Com(MTRCOM);                                  //初始化UART0
}
/*****************************************************************************
* 检查 UART设备地址是否正确
* 调试用
*****************************************************************************/
#ifdef DEBUG
static u8
UARTBaseValid(UART_Typedef *UARTx)
{
    return(((u32)UARTx == UART0_BASE) || ((u32)UARTx == UART1_BASE) ||
           ((u32)UARTx == UART2_BASE));
}
#endif

/*****************************************************************************
* 设置 奇偶校验 
* UARTx 串口设备结构体
* ulParity 校验设置 UART_CONFIG_PAR_NONE...
*****************************************************************************/
void UARTParityModeSet(UART_Typedef *UARTx, u32 ulParity)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    ASSERT((ulParity == UART_CONFIG_PAR_NONE) ||
           (ulParity == UART_CONFIG_PAR_EVEN) ||
           (ulParity == UART_CONFIG_PAR_ODD) ||
           (ulParity == UART_CONFIG_PAR_ONE) ||
           (ulParity == UART_CONFIG_PAR_ZERO));
    UARTx->LCRH = ((UARTx->LCRH & (~(UART_LCRH_SPS | UART_LCRH_EPS | UART_LCRH_PEN)))
	              | ulParity);
}
/*****************************************************************************
* 获取 奇偶校验方式 
* UARTx 串口设备结构体
* 返回 校验方式 UART_CONFIG_PAR_NONE...
*****************************************************************************/
u32 UARTParityModeGet(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));

    return(UARTx->LCRH &
           (UART_LCRH_SPS | UART_LCRH_EPS | UART_LCRH_PEN));
}

/*****************************************************************************
* FIFO 中断触发设置 
* UARTx 串口设备结构体
* ulTxLevel 发送FIFO触发 UART_FIFO_TX1_8...
* ulRxLevel 接收FIFO触发 UART_FIFO_RX1_8...
*****************************************************************************/
void UARTFIFOLevelSet(UART_Typedef *UARTx, u32 ulTxLevel,u32 ulRxLevel)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    ASSERT((ulTxLevel == UART_FIFO_TX1_8) ||
           (ulTxLevel == UART_FIFO_TX2_8) ||
           (ulTxLevel == UART_FIFO_TX4_8) ||
           (ulTxLevel == UART_FIFO_TX6_8) ||
           (ulTxLevel == UART_FIFO_TX7_8));
    ASSERT((ulRxLevel == UART_FIFO_RX1_8) ||
           (ulRxLevel == UART_FIFO_RX2_8) ||
           (ulRxLevel == UART_FIFO_RX4_8) ||
           (ulRxLevel == UART_FIFO_RX6_8) ||
           (ulRxLevel == UART_FIFO_RX7_8));
    UARTx->IFLS = ulTxLevel | ulRxLevel;
}
/*****************************************************************************
* 获取 FIFO 中断触发设置 
* UARTx 串口设备结构体
* pulTxLevel 返回指针 UART_FIFO_TX1_8...
* pulRxLevel 返回指针 UART_FIFO_RX1_8...
*****************************************************************************/
void
UARTFIFOLevelGet(UART_Typedef *UARTx, u32 *pulTxLevel,u32 *pulRxLevel)
{
    u32 ulTemp;
    ASSERT(UARTBaseValid((u32)UARTx));
    ulTemp = UARTx->IFLS;
    *pulTxLevel = (ulTemp & UART_IFLS_TX_M);
    *pulRxLevel = (ulTemp & UART_IFLS_RX_M);
}
/*****************************************************************************
* 串口配置 
* UARTx 串口设备结构体
* ulUARTClk UART时钟频率
* ulBaud 波特率
* ulConfig 串口线程控制
*****************************************************************************/
void UARTConfigSetExpClk(UART_Typedef *UARTx,u32 ulUARTClk,u32 ulBaud, u32 ulConfig)
{
    u32 ulDiv;
    ASSERT(UARTBaseValid((u32)UARTx));
    ASSERT(ulBaud != 0);
    ASSERT(ulUARTClk >= (ulBaud * UART_CLK_DIVIDER));

    UARTDisable(UARTx);
    ulDiv = (((ulUARTClk * 8) / ulBaud) + 1) / 2;  //0.5+64*(sysclk/16/baud)

    UARTx->IBRD = ulDiv / 64;
    UARTx->FBRD = ulDiv % 64;

    UARTx->LCRH = (ulConfig&0xFF);

    UARTx->FR = 0;

    UARTEnable(UARTx);
}

/*****************************************************************************
* 获取 串口配置 
* UARTx 串口设备结构体
* ulUARTClk UART时钟频率
* pulBaud 返回波特率
* pulConfig 串口线程控制
*****************************************************************************/
void UARTConfigGetExpClk(UART_Typedef *UARTx, u32 ulUARTClk,u32 *pulBaud, u32 *pulConfig)
{
    u32 ulInt, ulFrac;
    ASSERT(UARTBaseValid((u32)UARTx));
    ulInt = UARTx->IBRD;
    ulFrac = UARTx->FBRD;
    *pulBaud = (ulUARTClk * 4) / ((64 * ulInt) + ulFrac);
    *pulConfig = (UARTx->LCRH &
                  (UART_LCRH_SPS | UART_LCRH_WLEN_M | UART_LCRH_STP2 |
                   UART_LCRH_EPS | UART_LCRH_PEN));
}

/*****************************************************************************
* 使能串口收发 
* UARTx 串口设备结构体
* FIFO使能
*****************************************************************************/
void UARTEnable(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->LCRH |= UART_LCRH_FEN; //FIFO使能
    UARTx->CTL |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/*****************************************************************************
* 禁能串口收发 
* UARTx 串口设备结构体
* FIFO禁能
*****************************************************************************/
void UARTDisable(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    while(UARTx->FR & UART_FR_BUSY)
     {
     }

    UARTx->LCRH &= ~(UART_LCRH_FEN);
    UARTx->CTL &= ~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/*****************************************************************************
* 使能串口Irda SIR 模块 
* UARTx 串口设备结构体
* bLowPower=1 低功耗模式
* bLowPower=0 正常模式
*****************************************************************************/
void UARTEnableSIR(UART_Typedef *UARTx, u8 bLowPower)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    if(bLowPower)
     UARTx->CTL |= (UART_CTL_SIREN | UART_CTL_SIRLP);
    else
     UARTx->CTL |= (UART_CTL_SIREN);
}
/*****************************************************************************
* 禁能串口Irda 低功耗 
* UARTx 串口设备结构体
*****************************************************************************/
void UARTDisableSIR(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->CTL &= ~(UART_CTL_SIREN | UART_CTL_SIRLP);
}

/*****************************************************************************
* 检查接收FIFO是否有数据 
* UARTx 串口设备结构体
*****************************************************************************/
u8 UARTCharsAvail(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return((UARTx->FR & UART_FR_RXFE) ? false : true);
}

/*****************************************************************************
* 检查发送FIFO是否可用
* UARTx 串口设备结构体
* 返回 1 可用
* 返回 0 不可用
*****************************************************************************/
u8 UARTSpaceAvail(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return((UARTx->FR & UART_FR_TXFF) ? false : true);
}

/*****************************************************************************
* 读取数据
* UARTx 串口设备结构体
* 无数据返回 -1 
*****************************************************************************/
s32 UARTCharGetNonBlocking(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    if(!(UARTx->FR & UART_FR_RXFE))
     return(UARTx->DR);
    else
     return(-1);
}

/*****************************************************************************
* 等待返回有效数据
* UARTx 串口设备结构体
* 并返回数据
*****************************************************************************/
s32 UARTCharGet(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return(UARTx->DR);
}

/*****************************************************************************
* 发送1字节数据
* UARTx 串口设备结构体
* 返回1 发成功
* 返回0 发送失败
*****************************************************************************/
u8 UARTCharPutNonBlocking(UART_Typedef *UARTx, u8 ucData)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    if(!(UARTx->FR & UART_FR_TXFF))
     {
      UARTx->DR = ucData;
      return(true);
     }
    else
     return(false);
}
/*****************************************************************************
* 发送1字节数据
* UARTx 串口设备结构体
* 发送完成返回
*****************************************************************************/
void UARTCharPut(UART_Typedef *UARTx, u8 ucData)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    while(UARTx->FR & UART_FR_TXFF)
     { //发送缓冲区满 等待
     }
    UARTx->DR = ucData;
}

/*****************************************************************************
* UART发送终止控制
* UARTx 串口设备结构体
* bBreakState=1 发送终止
* bBreakState=0 取消发送终止
*****************************************************************************/
void UARTBreakCtl(UART_Typedef *UARTx, u8 bBreakState)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->LCRH = (bBreakState ?
                   (UARTx->LCRH | UART_LCRH_BRK) :
                   (UARTx->LCRH & ~(UART_LCRH_BRK)));
}

/*****************************************************************************
* UART是否正在发送数据
* UARTx 串口设备结构体
* 1 正在发送
* 0 发送空闲
*****************************************************************************/
u8 UARTBusy(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return((UARTx->FR & UART_FR_BUSY) ? true : false);
}
/*****************************************************************************
* 设置串口中断复位程序入口
* UARTx 串口设备结构体
* pfnHandler 服务程序入口
* 使能UART NVIC中断
*****************************************************************************/
void UARTIntRegister(UART_Typedef *UARTx, void (*pfnHandler)(void))
{
    u32 ulInt;
    ASSERT(UARTBaseValid((u32)UARTx));
    ulInt = (((u32)UARTx == UART0_BASE) ? INT_UART0 :
             (((u32)UARTx == UART1_BASE) ? INT_UART1 : INT_UART2));
    IntRegister(ulInt, pfnHandler);
    IntEnable(ulInt);
}
/*****************************************************************************
* 撤销串口中断复位程序入口
* UARTx 串口设备结构体
*****************************************************************************/
void UARTIntUnregister(UART_Typedef *UARTx)
{
    u32 ulInt;
    ASSERT(UARTBaseValid((u32)UARTx));
    ulInt = (((u32)UARTx == UART0_BASE) ? INT_UART0 :
             (((u32)UARTx == UART1_BASE) ? INT_UART1 : INT_UART2));
    IntDisable(ulInt);
    IntUnregister(ulInt);
}
/*****************************************************************************
* 设置串口中断使能状态
* UARTx 串口设备结构体
* ulIntFlags 中断使能位
*****************************************************************************/
void UARTIntEnable(UART_Typedef *UARTx, u32 ulIntFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->IM |= ulIntFlags;
}
/*****************************************************************************
* 设置串口中断禁能状态
* UARTx 串口设备结构体
* ulIntFlags 中断禁能位
*****************************************************************************/
void UARTIntDisable(UART_Typedef *UARTx, u32 ulIntFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->IM &= ~(ulIntFlags);
}

/*****************************************************************************
* 读取串口中断状态
* UARTx 串口设备结构体
* bMasked=1 读取屏蔽后的中断状态
* bMasked=0 读取原始的中断状态
*****************************************************************************/
u32 UARTIntStatus(UART_Typedef *UARTx, u8 bMasked)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    if(bMasked)
     return(UARTx->MIS);
    else
     return(UARTx->RIS);
}
/*****************************************************************************
* 清除串口中断状态
* UARTx 串口设备结构体
* ulIntFlags 清除中断相应位
*****************************************************************************/
void UARTIntClear(UART_Typedef *UARTx, u32 ulIntFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->ICR = ulIntFlags;
}

/*****************************************************************************
* UART DMA使能设置
* UARTx 串口设备结构体
* ulIntFlags DMA控制位 LM2139无效
*****************************************************************************/
void UARTDMAEnable(UART_Typedef *UARTx, u32 ulDMAFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->DMACTL |= ulDMAFlags;
}

/*****************************************************************************
* UART DMA禁能设置
* UARTx 串口设备结构体
* ulIntFlags DMA控制位 LM2139无效
*****************************************************************************/
void UARTDMADisable(UART_Typedef *UARTx, u32 ulDMAFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->DMACTL &= ~ulDMAFlags;
}

/*****************************************************************************
* 获取uart 接收错误状态
* UARTx 串口设备结构体
* 返回 UART接收错误状态
*****************************************************************************/
u32 UARTRxErrorGet(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return(UARTx->RSR & 0x0000000F);
}

/*****************************************************************************
* 清除uart 接收错误状态
* UARTx 串口设备结构体
*****************************************************************************/
void UARTRxErrorClear(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->RSR = 0;
}
