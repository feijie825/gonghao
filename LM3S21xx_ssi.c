/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_ssi.c
;* Author             : ������
;* ͬ�����нӿ�
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"
/*****************************************************************************
* ��ʼ��SSI
*****************************************************************************/
void Init_Ssi(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);//ʹ��SSI0ʱ��	
    SSIConfigSetExpClk(SSI0,Sysclk,\
                            SSI_FRF_MOTO_MODE_0,\
                            SSI_MODE_MASTER,\
                            SSI_BITRATE,\
                            SSI_CR0_DSS_8);    //����SSI CR0
    SSIEnable(SSI0);                           //ʹ��SSI0
}
/*****************************************************************************
* ����SSI���ݳ���
*****************************************************************************/
void SSIDataLen(SSI_Typedef *SSIx,u8 Len)
{
    while(!(SSIx->SR & SSI_SR_TFE))
     { //���ͻ��������� �ȴ�
     }
    SSIx->CR0 = ((SSIx->CR0 & (~SSI_CR0_DSS_M)) | Len); 
}
/*****************************************************************************
* ����SSI����״̬ ����SSI CR0
* SSIx SSI�ṹ�� SSI0~SSI1
* ulSSIClk
* ulProtocol
* ulMode ����ģʽ
* ulBitRate
* ulDataWidth
*****************************************************************************/
void SSIConfigSetExpClk(SSI_Typedef *SSIx, u32 ulSSIClk,u32 ulProtocol, u32 ulMode,
                                           u32 ulBitRate, u32 ulDataWidth)
{
    u32 ulMaxBitRate;
    u32 ulRegVal;
    u32 ulPreDiv;
    u32 ulSCR;
    u32 ulSPH_SPO;

    ASSERT((ulProtocol == SSI_FRF_MOTO_MODE_0) ||
           (ulProtocol == SSI_FRF_MOTO_MODE_1) ||
           (ulProtocol == SSI_FRF_MOTO_MODE_2) ||
           (ulProtocol == SSI_FRF_MOTO_MODE_3) ||
           (ulProtocol == SSI_FRF_TI) ||
           (ulProtocol == SSI_FRF_NMW));
    ASSERT((ulMode == SSI_MODE_MASTER) ||
           (ulMode == SSI_MODE_SLAVE) ||
           (ulMode == SSI_MODE_SLAVE_OD));
    ASSERT(((ulMode == SSI_MODE_MASTER) && (ulBitRate <= (ulSSIClk / 2))) ||
           ((ulMode != SSI_MODE_MASTER) && (ulBitRate <= (ulSSIClk / 12))));
    ASSERT((ulSSIClk / ulBitRate) <= (254 * 256));
    ASSERT((ulDataWidth >= 4) && (ulDataWidth <= 16));

    ulRegVal = (ulMode == SSI_MODE_SLAVE_OD) ? SSI_CR1_SOD : 0;
    ulRegVal |= (ulMode == SSI_MODE_MASTER) ? 0 : SSI_CR1_MS;
    SSIx->CR1 = ulRegVal;

    ulMaxBitRate = ulSSIClk / ulBitRate;
    ulPreDiv = 0;
    do
     {
      ulPreDiv += 2;
      ulSCR = (ulMaxBitRate / ulPreDiv) - 1;
     }
    while(ulSCR > 255);
    SSIx->CPSR = ulPreDiv;

    ulSPH_SPO = ulProtocol << 6;
    ulProtocol &= SSI_CR0_FRF_M;
    ulRegVal = (ulSCR << 8) | ulSPH_SPO | ulProtocol | ulDataWidth ;
    SSIx->CR0 = ulRegVal;
}

/*****************************************************************************
* ʹ��SSIģ��
* SSIx SSI�ṹ�� SSI0~SSI1
*****************************************************************************/
void SSIEnable(SSI_Typedef *SSIx)
{
    SSIx->CR1 |= SSI_CR1_SSE;
}
/*****************************************************************************
* ����SSIģ��
* SSIx SSI�ṹ�� SSI0~SSI1
*****************************************************************************/
void SSIDisable(SSI_Typedef *SSIx)
{
    SSIx->CR1 &= ~(SSI_CR1_SSE);
}

/*****************************************************************************
* ����SSIģ���жϷ������
* SSIx SSI�ṹ�� SSI0~SSI1
* pfnHandler �жϷ���������
* �ж���RAM��ʹ��
*****************************************************************************/
void SSIIntRegister(SSI_Typedef *SSIx, void (*pfnHandler)(void))
{
    u32 ulInt;
    ulInt = ((u32)SSIx == SSI0_BASE) ? INT_SSI0 : INT_SSI1;
    IntRegister(ulInt, pfnHandler);
    IntEnable(ulInt);
}
/*****************************************************************************
* ����SSIģ���жϷ������
* SSIx SSI�ṹ�� SSI0~SSI1
*****************************************************************************/
void SSIIntUnregister(SSI_Typedef *SSIx)
{
    u32 ulInt;

    ulInt = ((u32)SSIx == SSI0_BASE) ? INT_SSI0 : INT_SSI1;
    IntDisable(ulInt);
    IntUnregister(ulInt);
}
/*****************************************************************************
* ����SSI�ж�ʹ��
* ulIntFlags �ж�ʹ��λ
* SSIx SSI�ṹ�� SSI0~SSI1
*****************************************************************************/
void SSIIntEnable(SSI_Typedef *SSIx, u32 ulIntFlags)
{
    SSIx->IM |= ulIntFlags;
}

/*****************************************************************************
* ����SSI�жϽ���
* ulIntFlags �жϽ���λ
* SSIx SSI�ṹ�� SSI0~SSI1
*****************************************************************************/
void
SSIIntDisable(SSI_Typedef *SSIx, u32 ulIntFlags)
{
    SSIx->IM &= ~(ulIntFlags);
}

/*****************************************************************************
* ��ȡSSI�ж�״̬
* SSIx SSI�ṹ�� SSI0~SSI1
* bMasked=1 �������κ���ж�״̬
* bMasked=0 ����ԭʼ���ж�״̬
*****************************************************************************/
u32 SSIIntStatus(SSI_Typedef *SSIx, u8 bMasked)
{
    if(bMasked)
     return(SSIx->MIS);
    else
     return(SSIx->RIS);
}
/*****************************************************************************
* ���SSI�ж�״̬
* SSIx SSI�ṹ�� SSI0~SSI1
* ulIntFlags ����ж�״̬λ
*****************************************************************************/
void SSIIntClear(SSI_Typedef *SSIx, u32 ulIntFlags)
{
    SSIx->ICR = ulIntFlags;
}
/*****************************************************************************
* ����SSI����
* SSIx SSI�ṹ�� SSI0~SSI1
* ulData ����
*****************************************************************************/
void SSIDataPut(SSI_Typedef *SSIx, u32 ulData)
{
    while(!(SSIx->SR & SSI_SR_TNF))
     { //�� �ȴ�
     }
    SSIx->DR = ulData;
}

/*****************************************************************************
* ����SSI����
* SSIx SSI�ṹ�� SSI0~SSI1
* ulData ����
* ���ͻ������� ����0
* ���ݷ��ͳɹ� ����1
*****************************************************************************/
u32 SSIDataPutNonBlocking(SSI_Typedef *SSIx, u32 ulData)
{
    if(SSIx->SR & SSI_SR_TNF)
     {
      SSIx->DR = ulData;
      return(1);
     }
    else
     return(0);
}

/*****************************************************************************
* ��ȡSSI����
* SSIx SSI�ṹ�� SSI0~SSI1
* pulData ��������
*****************************************************************************/
void SSIDataGet(SSI_Typedef *SSIx, u32 *pulData)
{
    while(!(SSIx->SR & SSI_SR_RNE))
     { //û������ �ȴ�
     }
    *pulData = SSIx->DR;
}
/*****************************************************************************
* ��ȡSSI����
* SSIx SSI�ṹ�� SSI0~SSI1
* pulData ��������
* ���ջ������� ����0
* ���ճɹ� ����1
*****************************************************************************/
u32 SSIDataGetNonBlocking(SSI_Typedef *SSIx, u32 *pulData)
{
    if(SSIx->SR & SSI_SR_RNE)
     {
      *pulData = SSIx->DR;
      return(1);
     }
    else
     return(0);
}
/*****************************************************************************
* ʹ��SSI DMA���� LM3S2139��Ч
* SSIx SSI�ṹ�� SSI0~SSI1
* ulDMAFlags DMA��������
*****************************************************************************/
void SSIDMAEnable(SSI_Typedef *SSIx, u32 ulDMAFlags)
{
    SSIx->DMACTL |= ulDMAFlags;
}
/*****************************************************************************
* ����SSI DMA���� LM3S2139��Ч
* SSIx SSI�ṹ�� SSI0~SSI1
* ulDMAFlags DMA��������
*****************************************************************************/
void SSIDMADisable(SSI_Typedef *SSIx, u32 ulDMAFlags)
{
    SSIx->DMACTL &= ~ulDMAFlags;
}

