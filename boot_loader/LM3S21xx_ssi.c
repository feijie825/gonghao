/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_ssi.c
;* Author             : ������
;* ͬ�����нӿ�
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "disp.h"
/*****************************************************************************
* ��ʼ��SSI
*****************************************************************************/
void Init_Ssi(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);//ʹ��SSI0ʱ��	
    SSIConfigSetExpClk(SSI0,8000000,\
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


