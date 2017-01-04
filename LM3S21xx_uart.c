/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_uart.c
;* Author             : ������
;* ������������
;* UART ����FIFO ʹ�� һ�ο����ͻ�����д��7�ֽ����� ���ͻ�����������1�ֽ�ʱ�����ж� ������
;* UART ����FIFO ʹ�� ���ջ�������С��7�ֽ�ʱ�����ж� ����ճ�ʱ�ж�
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#define UART_CLK_DIVIDER 16
/*****************************************************************************
* У�����ö�Ӧ�б�
*****************************************************************************/
const u8 Parity_Set_TAB[5]=
{
    0x10,           //��У��λ 'N' FIFO ʹ��
    0x12,           //��У��   'O' FIFO ʹ��
    0x16,           //żУ��   'E' FIFO ʹ��
    0x92,           //MarkУ�� 'M' FIFO ʹ�� У��λ�̶�Ϊ1
    0x96            //SpaceУ��'S' FIFO ʹ�� У��λ�̶�Ϊ0
};
/*****************************************************************************
* ����Ĭ�ϲ����б� MTRCOM LCTCOM
*****************************************************************************/
const UART_SET UART_PARA_TAB[UART_NUMB]=
{           
    {
     UART0_BAUD,  //������
     UART_8_BIT,  //����λ��
     UART_1_STOP, //ֹͣλ��
     UART_N_PARITY//У��λ
    },
    {
     UART1_BAUD,  //������
     UART_8_BIT,  //����λ��
     UART_1_STOP, //ֹͣλ��
     UART_N_PARITY//У��λ
    },
};
/*****************************************************************************
* ���ڳ�ʼ��
* �˿� Com:�˿ں�
*****************************************************************************/
void Init_Com(u8 Com)
{
    UART_PARA UART_P;
    UART_Typedef *UARTx;
    UARTx = (UART_Typedef *)UART_PORT[Com];
    *((u32*)&UART_P) = Parity_Set_TAB[Uart_Para[Com].PARITY];//У��λ
    UART_P.Data_Len=Uart_Para[Com].LEN;              //����λ��
    UART_P.Stop2=Uart_Para[Com].STOP;                //ֹͣλ	 
    UARTConfigSetExpClk(UARTx,Sysclk,Uart_Para[Com].BAUD,*((u32*)&UART_P));
    UARTFIFOLevelSet(UARTx,UART_FIFO_TX1_8,          //����FIFO �жϴ������1/8
                           UART_FIFO_RX7_8);         //����FIFO �жϴ������7/8
    UARTIntEnable(UARTx,UART_INT_RX |                //���������ж� 
                        UART_INT_RT|                 //�������ճ�ʱ�ж�
                        UART_INT_TX );	              //���������ж�
}
/*****************************************************************************
* ���ڳ�ʼ��
* �ܽ�����Init_Gpio()������
*****************************************************************************/
void Init_Uart(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);       //UART0�豸ʱ��ʹ��
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);       //UART1�豸ʱ��ʹ��
    memcpy(&Uart_Para[MTRCOM],                         //MTRCOM����
           &UART_PARA_TAB[MTRCOM],
           4);
    Init_Com(MTRCOM);                                  //��ʼ��UART0
}
/*****************************************************************************
* ��� UART�豸��ַ�Ƿ���ȷ
* ������
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
* ���� ��żУ�� 
* UARTx �����豸�ṹ��
* ulParity У������ UART_CONFIG_PAR_NONE...
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
* ��ȡ ��żУ�鷽ʽ 
* UARTx �����豸�ṹ��
* ���� У�鷽ʽ UART_CONFIG_PAR_NONE...
*****************************************************************************/
u32 UARTParityModeGet(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));

    return(UARTx->LCRH &
           (UART_LCRH_SPS | UART_LCRH_EPS | UART_LCRH_PEN));
}

/*****************************************************************************
* FIFO �жϴ������� 
* UARTx �����豸�ṹ��
* ulTxLevel ����FIFO���� UART_FIFO_TX1_8...
* ulRxLevel ����FIFO���� UART_FIFO_RX1_8...
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
* ��ȡ FIFO �жϴ������� 
* UARTx �����豸�ṹ��
* pulTxLevel ����ָ�� UART_FIFO_TX1_8...
* pulRxLevel ����ָ�� UART_FIFO_RX1_8...
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
* �������� 
* UARTx �����豸�ṹ��
* ulUARTClk UARTʱ��Ƶ��
* ulBaud ������
* ulConfig �����߳̿���
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
* ��ȡ �������� 
* UARTx �����豸�ṹ��
* ulUARTClk UARTʱ��Ƶ��
* pulBaud ���ز�����
* pulConfig �����߳̿���
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
* ʹ�ܴ����շ� 
* UARTx �����豸�ṹ��
* FIFOʹ��
*****************************************************************************/
void UARTEnable(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->LCRH |= UART_LCRH_FEN; //FIFOʹ��
    UARTx->CTL |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/*****************************************************************************
* ���ܴ����շ� 
* UARTx �����豸�ṹ��
* FIFO����
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
* ʹ�ܴ���Irda SIR ģ�� 
* UARTx �����豸�ṹ��
* bLowPower=1 �͹���ģʽ
* bLowPower=0 ����ģʽ
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
* ���ܴ���Irda �͹��� 
* UARTx �����豸�ṹ��
*****************************************************************************/
void UARTDisableSIR(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->CTL &= ~(UART_CTL_SIREN | UART_CTL_SIRLP);
}

/*****************************************************************************
* ������FIFO�Ƿ������� 
* UARTx �����豸�ṹ��
*****************************************************************************/
u8 UARTCharsAvail(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return((UARTx->FR & UART_FR_RXFE) ? false : true);
}

/*****************************************************************************
* ��鷢��FIFO�Ƿ����
* UARTx �����豸�ṹ��
* ���� 1 ����
* ���� 0 ������
*****************************************************************************/
u8 UARTSpaceAvail(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return((UARTx->FR & UART_FR_TXFF) ? false : true);
}

/*****************************************************************************
* ��ȡ����
* UARTx �����豸�ṹ��
* �����ݷ��� -1 
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
* �ȴ�������Ч����
* UARTx �����豸�ṹ��
* ����������
*****************************************************************************/
s32 UARTCharGet(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return(UARTx->DR);
}

/*****************************************************************************
* ����1�ֽ�����
* UARTx �����豸�ṹ��
* ����1 ���ɹ�
* ����0 ����ʧ��
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
* ����1�ֽ�����
* UARTx �����豸�ṹ��
* ������ɷ���
*****************************************************************************/
void UARTCharPut(UART_Typedef *UARTx, u8 ucData)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    while(UARTx->FR & UART_FR_TXFF)
     { //���ͻ������� �ȴ�
     }
    UARTx->DR = ucData;
}

/*****************************************************************************
* UART������ֹ����
* UARTx �����豸�ṹ��
* bBreakState=1 ������ֹ
* bBreakState=0 ȡ��������ֹ
*****************************************************************************/
void UARTBreakCtl(UART_Typedef *UARTx, u8 bBreakState)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->LCRH = (bBreakState ?
                   (UARTx->LCRH | UART_LCRH_BRK) :
                   (UARTx->LCRH & ~(UART_LCRH_BRK)));
}

/*****************************************************************************
* UART�Ƿ����ڷ�������
* UARTx �����豸�ṹ��
* 1 ���ڷ���
* 0 ���Ϳ���
*****************************************************************************/
u8 UARTBusy(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return((UARTx->FR & UART_FR_BUSY) ? true : false);
}
/*****************************************************************************
* ���ô����жϸ�λ�������
* UARTx �����豸�ṹ��
* pfnHandler ����������
* ʹ��UART NVIC�ж�
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
* ���������жϸ�λ�������
* UARTx �����豸�ṹ��
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
* ���ô����ж�ʹ��״̬
* UARTx �����豸�ṹ��
* ulIntFlags �ж�ʹ��λ
*****************************************************************************/
void UARTIntEnable(UART_Typedef *UARTx, u32 ulIntFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->IM |= ulIntFlags;
}
/*****************************************************************************
* ���ô����жϽ���״̬
* UARTx �����豸�ṹ��
* ulIntFlags �жϽ���λ
*****************************************************************************/
void UARTIntDisable(UART_Typedef *UARTx, u32 ulIntFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->IM &= ~(ulIntFlags);
}

/*****************************************************************************
* ��ȡ�����ж�״̬
* UARTx �����豸�ṹ��
* bMasked=1 ��ȡ���κ���ж�״̬
* bMasked=0 ��ȡԭʼ���ж�״̬
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
* ��������ж�״̬
* UARTx �����豸�ṹ��
* ulIntFlags ����ж���Ӧλ
*****************************************************************************/
void UARTIntClear(UART_Typedef *UARTx, u32 ulIntFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->ICR = ulIntFlags;
}

/*****************************************************************************
* UART DMAʹ������
* UARTx �����豸�ṹ��
* ulIntFlags DMA����λ LM2139��Ч
*****************************************************************************/
void UARTDMAEnable(UART_Typedef *UARTx, u32 ulDMAFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->DMACTL |= ulDMAFlags;
}

/*****************************************************************************
* UART DMA��������
* UARTx �����豸�ṹ��
* ulIntFlags DMA����λ LM2139��Ч
*****************************************************************************/
void UARTDMADisable(UART_Typedef *UARTx, u32 ulDMAFlags)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->DMACTL &= ~ulDMAFlags;
}

/*****************************************************************************
* ��ȡuart ���մ���״̬
* UARTx �����豸�ṹ��
* ���� UART���մ���״̬
*****************************************************************************/
u32 UARTRxErrorGet(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    return(UARTx->RSR & 0x0000000F);
}

/*****************************************************************************
* ���uart ���մ���״̬
* UARTx �����豸�ṹ��
*****************************************************************************/
void UARTRxErrorClear(UART_Typedef *UARTx)
{
    ASSERT(UARTBaseValid((u32)UARTx));
    UARTx->RSR = 0;
}
