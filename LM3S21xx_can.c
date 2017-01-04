/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_can.c
;* Author             : ������
;* CAN���߽ӿ��������� 
;* ������CAN�ӿڵ�LM3S ϵ�д����� ����CANʱ�Ӻ�CPUʱ��֮��Ƶ�ʵĲ�һ��,��ɷ���CAN�Ĵ���ʱ
;* Ҫ��Ӷ�����ʱ CPU ʱ��Ƶ�ʸ���CANʱ��Ƶ��
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "stdlib.h"
#include "string.h"
#include "define.h"
#include "vari.h"

//CAN�Ĵ���������ʱ����
#define CAN_RW_DELAY            (1)
/******************************************************************************
* CAN���������ñ�
* ��CANBIT�Ĵ�������һ��
* λ˳�� TSEG1 TSEG2  SJW PRE(Ԥ��Ƶֵ)
* CANλʱ��=(TSEG1+1)+(TSEG2+1)+1 
* ������=((TSEG1+1)+1)/CAN �ݶ�ʱ�����
* CAN ��CLK λ8MHz �ڲ�����
* ����CAN��ʱ�Ӻ�CPUʱ�Ӽ�Ĳ��� 
* ��ϵͳʱ��Ƶ�ʴ���8MHzʱ ����CAN�Ĵ���ʱҪ���Ӷ�Ӧ����ʱ
******************************************************************************/
const u32 CANBitClkSettings[]=
{
    CAN_TIMING(5,2,2,10),  // CANBAUD_100K CANλʱ��=(4+1)+(1+1)+1=8��CAN�ݶ�ʱ��  �ݶ�ʱ��=8/10=0.8MHz ������6/8=75%
    CAN_TIMING(5,2,2,8),   // CANBAUD_125K CANλʱ��=(4+1)+(1+1)+1=8��CAN�ݶ�ʱ��  �ݶ�ʱ��=8/8=1MHz    ������6/8=75%
    CAN_TIMING(5,2,2,4),   // CANBAUD_250K CANλʱ��=(4+1)+(1+1)+1=8��CAN�ݶ�ʱ��  �ݶ�ʱ��=8/4=2MHz    ������6/8=75%
    CAN_TIMING(5,2,2,2),   // CANBAUD_500K CANλʱ��=(4+1)+(1+1)+1=8��CAN�ݶ�ʱ��  �ݶ�ʱ��=8/2=4MHz    ������6/8=75%
    CAN_TIMING(5,2,2,1)    // CANBAUD_1M   CANλʱ��=(4+1)+(1+1)+1=8��CAN�ݶ�ʱ��  �ݶ�ʱ��=8/1=8MHz    ������6/8=75%
};
/*****************************************************************************
* CAN ���Ķ��������ñ�� MSG RAM 
* ���ձ��Ķ�����Ҫ�����˲� 
* ���ͱ��Ķ����������˲�
* ����λ����� LM3S21xx_can.h �б��Ľṹ�嶨�� 
* ���ӱ��Ķ���ʱ �������
*****************************************************************************/
const CAN_MSG_SET  CAN_MSG_SET_TAB[]=
{    //������     �ٲ���     ��������
    {0x000103E0,0x10000002,0x1800FFFF},  //��չ֡ �������Ͷ̹㲥����֡ MST_SCDATA_TX   MSG OBJ ID1  ����
    {0x000203E0,0x10000002,0x1800FFFF},  //��չ֡ �������Ͷ̵�������֡ MST_SSDATA_TX   MSG OBJ ID2  ����
    {0x00030050,0x10000002,0x1FFFFFFF},  //��չ֡ �ӻ����Ͷ�����֡     SLV_SDATA_TX    MSG OBJ ID3  ����
};
/*****************************************************************************
* ��ʼ��CAN�ӿ�
*****************************************************************************/
void Init_CAN(void)
{
    CANBit_Timing  CANBit;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);  //ʹ��CAN�豸SYCTL_RCGC(ʹ��ʱ��) 
    CANDisable(CAN0);                            //�����ʼ��ģʽ
    CANBit.WORD = CANBitClkSettings[CANBAUD];
    Clr_MsgRam(CAN0);                            //���CAN MSG RAM
    CANBitTimingSet(CAN0,&CANBit);               //���ò�����
    CANIntEnable(CAN0, CAN_INT_MASTER |	         //ʹ���ж�
                       CAN_INT_ERROR );          //ʹ�ܴ�������ж� Boff Ewarn 
    Set_MsgRam(CAN0);                            //��ʼ��can���� ���ý��ձ��Ķ������
    CANEnable(CAN0);                             
    CANStatusGet(CAN0, CAN_STS_CONTROL);         //���״̬�ж�
}
/*****************************************************************************
* ���CAN MSG RAM��
* ���:CANx CAN�ṹ��
*****************************************************************************/
void Clr_MsgRam(CAN_Typedef *CANx)
{
    u32 iMsg;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    CANRegWrite((u32)(&CANx->CTL),CAN_CTL_INIT);//�����ʼ��ģʽ �������CAN RAM��
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {                                        //�ȴ����ʽӿڿ���
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK,      //�����������μĴ���
                      CAN_IF1CMSK_WRNRD |     //��д���� WRNRD 0 �� 1д ����д INF->����RAM
                      CAN_IF1CMSK_ARB |       //�����ٲ�λ ID+Dir+Xtd+MsgValλ->����RAM
                      CAN_IF1CMSK_CONTROL);   //�������λ ->����RAM
    CANRegWrite((u32)&CANx->INF[0].ARB2 , 0); //�ٲüĴ���2 ���� MsgVal Xtd Dir ID[28:16] ��׼֡ID ID[28:18]
    CANRegWrite((u32)&CANx->INF[0].MCTL , 0); //���Ŀ��ƼĴ���
    for(iMsg = 1; iMsg <= 32; iMsg++)
     {                                        //��ʼ������RAM
      while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
       {                                      //�ȴ����ʽӿڿ��� 
       }
      CANRegWrite((u32)&CANx->INF[0].CRQ , iMsg); //д�뱨��ID
     }

    CANRegWrite((u32)&CANx->INF[0].CMSK ,     //�����������μĴ��� WRNRD 0 �� ����RAM->INF
                      CAN_IF1CMSK_NEWDAT |    //���ʱ���RAM ���NewDat λ
                      CAN_IF1CMSK_CLRINTPND); //���ʱ���RAM ���IntPnd �жϹ���λ
    for(iMsg = 1; iMsg <= 32; iMsg++)
     {
      while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
       {                                      //�ȴ����ʽӿڿ���
       }
      CANRegWrite((u32)&CANx->INF[0].CRQ , iMsg); //������б��ĵ� NEWDAT λ�� IntPndλ
     }
    CANRegRead(IntNumber,(u32)&CANx->STS);    //��״̬�Ĵ��� ���CAN�жϼĴ��� CAN->INT
}
/*****************************************************************************
* ����CAN MSG ����RAM����������
* ���:CANx CAN�ṹ��
* ������ ���� CMD=3Ԥ��
*****************************************************************************/
void Set_MsgRam(CAN_Typedef *CANx)
{
    CAN_MSG_SET RX_MSG_SET;    //���ձ���
/*********�����������Ͷ̹㲥���ݴ� ����  MSG RAM ���MST_SCDATA_TX(8) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_SCDATA_TX-1],sizeof(CAN_MSG_SET));
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //���(CAN�ӿڻ�ַ,���Ķ�����,���Ľṹ���ַ,���Ķ�������)
/*********�����������ڷ��Ͷ̵������ݴ� ����  MSG RAM ���MST_SSDATA_TX(9) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_SSDATA_TX-1],sizeof(CAN_MSG_SET));
    RX_MSG_SET.ID.BIT.NUM = Mtr_Numb_ID;               //�ٲ���	��λ��
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //���(CAN�ӿڻ�ַ,���Ķ�����,���Ľṹ���ַ,���Ķ�������)
}
/*****************************************************************************
* ���CAN�豸��ַ�Ƿ���ȷ
* CANx �ṹ��
* ����ʱ��Ч
*****************************************************************************/
#ifdef DEBUG
u8 CANBaseValid(CAN_Typedef *CANx)
{
    return(((u32)CANx == CAN0_BASE) || ((u32)CANx == CAN1_BASE) ||
           ((u32)CANx == CAN2_BASE));
}
#endif
/*****************************************************************************
* ��ȡCAN�ӿ��ж�ID�� 
* CANx �ṹ��
* ����ID��
* 2008.12.5 Ϊ����ߴ����ٶ�zlz ����ֱ�ӷ����ж���NVIC->EN �е�λ��
*****************************************************************************/
u32 CANIntNumberGet(CAN_Typedef *CANx)
{
    return(((u32)CANx == CAN0_BASE) ? ~(1<<(INT_CAN0-48)) :
           (((u32)CANx == CAN1_BASE) ? ~(1<<(INT_CAN1-48)) :
           (((u32)CANx == CAN2_BASE) ? ~(1<<(INT_CAN2-48)) : ~(1<<(INT_CAN0-48)))));
}

/*****************************************************************************
* ��CAN�Ĵ���
* ���� �Ĵ���ֵ
*****************************************************************************/
u32	CANRegRead(u32 ulIntNumber,u32 ulRegAddress)
{
    u32 ulRetVal;
    u32 ReCANEnInt;                   //CAN�ж���ʹ��
    ReCANEnInt = HWREG(NVIC_EN1) ;
    HWREG(NVIC_EN1) = (ReCANEnInt & ulIntNumber);
    HWREG(ulRegAddress);              //��һ�ζ�CAN�Ĵ��� (���ݲ���ȷ ֪ͨCAN������������)
    SysCtlDelay(CAN_RW_DELAY);        //��ʱ
    ulRetVal = HWREG(ulRegAddress);		//�ڶ��ζ�CAN�Ĵ��� (������ȷ)
    HWREG(NVIC_EN1) = ReCANEnInt ;
    return(ulRetVal);
}

/*****************************************************************************
* дCAN�Ĵ���
* ���:ulRegAddress�Ĵ�����ַ
* ulRegValue �Ĵ���ֵ
*****************************************************************************/
void CANRegWrite(u32 ulRegAddress, u32 ulRegValue)
{
    HWREG(ulRegAddress) = ulRegValue;
    SysCtlDelay(CAN_RW_DELAY);
}
/*****************************************************************************
* дCAN���ݼĴ���
* ���:pucData ����������ָ��
* ���:pulRegister ���ݼĴ�����ַ
* ���:iSize ���ݳ���
* ʵ�ʷ������ݳ����ܷ���DLC����λ����
*****************************************************************************/
void CANDataRegWrite(u8 *pucData, u32 pulRegister, u32 iSize)
{
    u32 iIdx;
    u32 ulValue;
    for(iIdx = 0; iIdx < iSize; )
     {
      ulValue = pucData[iIdx++];
      if(iIdx < iSize)
       ulValue |= (pucData[iIdx++] << 8);
      CANRegWrite(pulRegister, ulValue);
      pulRegister += 4;   //CAN���ݼĴ�����ַ+4
     }
}
/*****************************************************************************
* ��CAN���ݼĴ���
* ���:IntNumber CAN�жϺ�
* ���:pucData ���������ݵ�ָ��
* ���:pulRegister ���ݼĴ�����ַָ��
* ���:iSize ���ݳ���
* ʵ�ʽ������ݳ�����DLC����λ����
*****************************************************************************/
void CANDataRegRead(u32 IntNumber,u8 *pucData, u32 pulRegister, u32 iSize)
{
    u32 iIdx;
    u32 ulValue;
    for(iIdx = 0; iIdx < iSize; )
     {
      ulValue = CANRegRead(IntNumber,pulRegister);
  	  pulRegister += 4;   //CAN���ݼĴ�����ַ+4
      pucData[iIdx++] = (u8)ulValue;
      if(iIdx < iSize)
       pucData[iIdx++] = (u8)(ulValue >> 8);
     }
}

/*****************************************************************************
* CAN������ʹ��
* CANx �ṹ��
*****************************************************************************/
void CANEnable(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) & ~CAN_CTL_INIT));
}

/*****************************************************************************
* CAN����������
*****************************************************************************/
void CANDisable(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) | CAN_CTL_INIT));
}

/*****************************************************************************
* ��ȡ CAN λ��ʱ����
* ���:CANx CAN�ṹ��
* ���� ָ��pClkParms
* 2008.11.24 �������޸� CANBPRE	Ĭ��Ϊ0 ����չ��Ƶ ��Ƶϵ����CANBIT����
*****************************************************************************/
void CANBitTimingGet(CAN_Typedef *CANx, CANBit_Timing *CANBit_Time)
{
    u32 IntNumber = CANIntNumberGet(CANx);           //��ȡCAN�ж�ID��
    ASSERT(CANBaseValid(CANx));
    ASSERT(pClkParms != 0);
    CANBit_Time->WORD = CANRegRead(IntNumber,(u32)&CANx->BIT); //��ȡCAN bittime���üĴ���
}

/*****************************************************************************
* ���� CAN λ��ʱ
* ���:CANx CAN�ṹ��
* ���: ָ��pClkParms
*****************************************************************************/
void CANBitTimingSet(CAN_Typedef *CANx, CANBit_Timing *CANBit_Time)
{
    u32 uSavedInit;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    uSavedInit = CANRegRead(IntNumber,(u32)&CANx->CTL); //�����ƼĴ���
    CANRegWrite((u32)&CANx->CTL , (uSavedInit |
                                   CAN_CTL_INIT |       //д���ʼ����ʼ
                                   CAN_CTL_CCE));       //�ı�ʹ��
    CANRegWrite((u32)&CANx->BIT , CANBit_Time->WORD);   //д��λ����
    CANRegWrite((u32)&CANx->BRPE , 0);                  //Ԥ��Ƶ=1
    uSavedInit &= ~CAN_CTL_CCE;                         //����ı�ʹ��λ
//    if(uSavedInit & CAN_CTL_INIT)                       //�жϽ�������ʱ�Ƿ��ڳ�ʼ��״̬
//     uSavedInit &= ~CAN_CTL_INIT;                       //�˳���ʼ��״̬
    CANRegWrite((u32)&CANx->CTL , uSavedInit);          //д���ƼĴ���
}
/*****************************************************************************
* ����CAN�жϸ�λ�������
* ���:CANx CAN�ṹ��
* pfnHandler ����������
* ʹ�� NVIC�ж� �ж���������RAM��ʱ ʹ��
*****************************************************************************/
void CANIntRegister(CAN_Typedef *CANx, void (*pfnHandler)(void))
{
    u32 IntNumber;
    ASSERT(CANBaseValid(CANx));
    IntNumber = CANIntNumberGet(CANx);
    IntRegister(IntNumber, pfnHandler);
    IntEnable(IntNumber);
}
/*****************************************************************************
* ����CAN�жϸ�λ�������
* ���:CANx CAN�ṹ��
* ���� NVIC�ж� �ж���������RAM��ʱ ʹ��
*****************************************************************************/
void CANIntUnregister(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    IntUnregister(IntNumber);
    IntDisable(IntNumber);
}
/*****************************************************************************
* ����CAN�ж�ʹ��λ
* ���:CANx CAN�ṹ��
* ���:ulIntFlags ��Ҫ�жϵ�λ
*****************************************************************************/
void CANIntEnable(CAN_Typedef *CANx, u32 ulIntFlags)
{
    u32 IntNumber = CANIntNumberGet(CANx);

    ASSERT(CANBaseValid(CANx));
    ASSERT((ulIntFlags & ~(CAN_CTL_EIE | CAN_CTL_SIE | CAN_CTL_IE)) == 0);

    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) | ulIntFlags));
}

/*****************************************************************************
* ����CAN�ж�λ
* ���:CANx CAN�ṹ��
* ���:ulIntFlags ��Ҫ�жϵ�λ
*****************************************************************************/
void CANIntDisable(CAN_Typedef *CANx, u32 ulIntFlags)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulIntFlags & ~(CAN_CTL_EIE | CAN_CTL_SIE | CAN_CTL_IE)) == 0);
    CANRegWrite((u32)&CANx->CTL , (CANRegRead(IntNumber,(u32)&CANx->CTL) & ~(ulIntFlags)));
}

/*****************************************************************************
* ��ȡCAN �����ж�״̬
* ���:CANx CAN�ṹ��
* ���:eIntStsReg ��Ҫ��ȡ�жϱ���ID ���� �жϱ���λ
*****************************************************************************/
u32	CANIntStatus(CAN_Typedef *CANx, CANIntSts eIntStsReg)
{
    u32 ulStatus;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    switch(eIntStsReg)
     {
      case CAN_INT_STS_CAUSE:
       {                //��ȡ�ж�ID��
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->INT);
        return(ulStatus);
       }
      case CAN_INT_STS_OBJECT:
       {                //��ȡ�жϱ�����Ϣ
        ulStatus = (CANRegRead(IntNumber,(u32)&CANx->MSG1INT) &
                    CAN_MSG1INT_INTPND_M);
        ulStatus |= (CANRegRead(IntNumber,(u32)&CANx->MSG2INT) << 16);
        return(ulStatus);
       }
      default:
       return(0);
     }
    
}
/*****************************************************************************
* ���CAN �����ж�״̬λ
* ���:CANx CAN�ṹ��
* ���:ulIntClr ����ж�״̬λѡ��
* ulIntClr= CAN_INT_INTID_STATUS ���״̬�ж�
* ulIntClr= 1~32 ��������ж�״̬
* 2011.3.22 zlz ������ж϶˿���1(�ж���) ��ֹ��CAN_Tx_Msg()������ʹ�ö˿ڳ�ͻ
*****************************************************************************/
void CANIntClear(CAN_Typedef *CANx, u32 ulIntClr)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulIntClr == CAN_INT_INTID_STATUS) ||
           ((ulIntClr>=1) && (ulIntClr <=32)));
    if(ulIntClr == CAN_INT_INTID_STATUS)
     CANRegRead(IntNumber,(u32)&CANx->STS);
    else
     {
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {              //�ȴ��ӿڼĴ�������
       }
      CANRegWrite((u32)&CANx->INF[1].CMSK , CAN_IF1CMSK_CLRINTPND);         //���ʱ���CLRINTPNDλ
      CANRegWrite((u32)&CANx->INF[1].CRQ , (ulIntClr & CAN_IF1CRQ_MNUM_M)); //����Ҫ���ʵı���
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {
       }
     }
}
/*****************************************************************************
* ����CAN��ֹ�Զ��ط�λ
* ���:CANx CAN�ṹ��
* ���:bAutoRetry=1 ʹ���Զ��ط� bAutoRetry=0�����Զ��ط�
*****************************************************************************/
void CANRetrySet(CAN_Typedef *CANx, u8 bAutoRetry)
{
    u32 ulCtlReg;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ulCtlReg = CANRegRead(IntNumber,(u32)&CANx->CTL);
    if(bAutoRetry)
     ulCtlReg &= ~CAN_CTL_DAR;               //�����Զ��ط�
    else
     ulCtlReg |= CAN_CTL_DAR;                //��ֹ�Զ��ط�    
    CANRegWrite((u32)&CANx->CTL , ulCtlReg); //д���ƼĴ���
}

/*****************************************************************************
* ��ȡCAN�Զ��ط�״̬
* ���:CANx CAN�ṹ��
* ���� true �Զ��ط�ʹ�� false�Զ��ط�����
*****************************************************************************/
u8 CANRetryGet(CAN_Typedef *CANx)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));

    if(CANRegRead(IntNumber,(u32)&CANx->CTL) & CAN_CTL_DAR)
     return(false);
    return(true);
}
/*****************************************************************************
* ��ȡCAN �ж�״̬
* ���:CANx CAN�ṹ��
* ���:eStatusReg ��Ҫ��ȡ״̬λ������ CAN_STS_CONTROL...
*****************************************************************************/
u32	CANStatusGet(CAN_Typedef *CANx, CANSts eStatusReg)
{
    u32 ulStatus;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    switch(eStatusReg)
     {
      case CAN_STS_CONTROL:
       {                 //��ȡϵͳ״̬�Ĵ��� ͬʱ���״̬λ
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->STS);
        CANRegWrite((u32)&CANx->STS , (~(CAN_STS_RXOK | CAN_STS_TXOK | CAN_STS_LEC_M)));
        return(ulStatus);
       }
      case CAN_STS_TXREQUEST:
       {                 //��ȡ��������״̬
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->TXRQ1);
        ulStatus |= CANRegRead(IntNumber,(u32)&CANx->TXRQ2) << 16;
        return(ulStatus);
       }
      case CAN_STS_NEWDAT:
       {                 //��ȡ�����ݽ���״̬
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->NWDA1);
        ulStatus |= CANRegRead(IntNumber,(u32)&CANx->NWDA2) << 16;
        return(ulStatus);
       }
      case CAN_STS_MSGVAL:
       {                 //��ȡ����RAM��Ч״̬
        ulStatus = CANRegRead(IntNumber,(u32)&CANx->MSG1VAL);
        ulStatus |= CANRegRead(IntNumber,(u32)&CANx->MSG2VAL) << 16;
        return(ulStatus);
       }
      default:
       return(0);       //�Ƿ� ����0
     }
}

/*****************************************************************************
* ��ȡCAN �������ֵ
* ���:CANx CAN�ṹ��
* �˿�*CanErr_Cnt �����������ָ��
* ����:true �ﵽ�����Ͽ����� falseδ�ﵽ
*****************************************************************************/
u8 CANErrCntrGet(CAN_Typedef *CANx, u16 *CanErr_Cnt)
{
    u32 ulCANError;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ulCANError = CANRegRead(IntNumber,(u32)&CANx->ERR); //����������Ĵ���
    *CanErr_Cnt = (ulCANError &(CAN_ERR_REC_M|CAN_ERR_TEC_M));  //�������
    if(ulCANError & CAN_ERR_RP)                         //�ж��Ƿ�ﵽ�����Ͽ�����
     return(true);
    return(false);
}

/*****************************************************************************
* ���� CAN ���ձ���RAM �� ���Ķ���
* ���:CANx CAN�ṹ��
* ���: CAN_MSG ����ָ�� 
* ���: eMsgType ��������
* �رս��յ�Զ��֡�Զ����͹��� ZLZ 2008.11.27
*****************************************************************************/
void CAN_RxMsg_Set(CAN_Typedef *CANx, CAN_MSG_SET *CAN_MSG, MsgType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usMaskReg[2]={0,0};
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_RX) ||
           (eMsgType == MSG_OBJ_TYPE_RX_REMOTE) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE) ||)
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {                                    //�ȴ��ӿڼĴ�������
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //���� INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //�������MSG ����A
                    CAN_IF1CMSK_DATAB |   //�������MSG ����B
                    CAN_IF1CMSK_CONTROL|  //�������MSG ����λ
					CAN_IF1CMSK_ARB);     //�������MSG �ٲ�λ
    switch(eMsgType)
     {                    //���ݱ���������ת
         case MSG_OBJ_TYPE_RX:
          {                //��������֡
           usArbReg[1] = 0;//���÷���λΪ��������֡
           break;
          }
         case MSG_OBJ_TYPE_RX_REMOTE:
          {                //����Զ��֡
           usArbReg[1] = CAN_IF1ARB2_DIR;    //���÷���λΪ����Զ��֡
           usMsgCtrl   = CAN_IF1MCTL_UMASK;  //��������λ
           usMaskReg[0] = 0xffff;            //Զ��֡Ĭ��ID����ʹ��
           usMaskReg[1] = 0x1fff;            //Զ��֡Ĭ��ID����ʹ��
           usCmdMaskReg |= CAN_IF1CMSK_MASK; //�����ٲ�λ
           break;
          }
         case MSG_OBJ_TYPE_RXTX_REMOTE:
          {                //����Զ��֡���Զ�����
           usArbReg[1] = CAN_IF1ARB2_DIR;
           usMsgCtrl = CAN_IF1MCTL_RMTEN | CAN_IF1MCTL_UMASK;
           break;
          }
         default:
          return;              //�Ƿ� �˳�
     }
     if(CAN_MSG->CTL.BIT.ID_FLT_EN )//�ж��Ƿ�ʹ��ID��������
      {                        //ʹ��ID��������
       if(CAN_MSG->CTL.BIT.EXD_ID)
        {                      //��չ֡	29bit ID
          usMaskReg[0] = CAN_MSG->IDMask.WORD & CAN_IF1MSK1_IDMSK_M;
          usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 16) & CAN_IF1MSK2_IDMSK_M);
        }
       else
        {                      //��׼֡  
         usMaskReg[0] = 0;
         usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 6 ) & CAN_IF1MSK2_IDMSK_M);
        }
      }
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.EXT_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MXTD;  //ʹ����չID����
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.DIR_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MDIR;  //���䷽�����ڹ���
    if(CAN_MSG->CTL.BIT.ID_FLT_EN  ||
       CAN_MSG->CTL.BIT.EXT_FLT_EN ||
       CAN_MSG->CTL.BIT.DIR_FLT_EN)
     {
      usMsgCtrl |= CAN_IF1MCTL_UMASK;
      usCmdMaskReg |= CAN_IF1CMSK_MASK;
     }
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //��չ֡ 29BIT ID
      usArbReg[0] |= CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD;
     }
    else
     {                      //��׼֡
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL;
     }
    usMsgCtrl |= (CAN_MSG->CTL.BIT.LEN  | CAN_IF1MCTL_EOB); //���ÿ���λ ���ݳ��� ���η���
    if(CAN_MSG->CTL.BIT.RX_INT_EN )                       //�жϽ����ж��Ƿ�ʹ��
     usMsgCtrl |= CAN_IF1MCTL_RXIE;                       //�����ж�ʹ��        
    CANRegWrite((u32)&CANx->INF[0].CMSK , usCmdMaskReg);  //д�������μĴ���
    CANRegWrite((u32)&CANx->INF[0].MSK1 , usMaskReg[0]);  //д���μĴ���1
    CANRegWrite((u32)&CANx->INF[0].MSK2 , usMaskReg[1]);  //д���μĴ���2
    CANRegWrite((u32)&CANx->INF[0].ARB1 , usArbReg[0]);   //д�ٲüĴ���1
    CANRegWrite((u32)&CANx->INF[0].ARB2 , usArbReg[1]);   //д�ٲüĴ���2
    CANRegWrite((u32)&CANx->INF[0].MCTL , usMsgCtrl);     //д���Ŀ��ƼĴ���
    CANRegWrite((u32)&CANx->INF[0].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));//INF->MSG RAM
}
/*****************************************************************************
* ���� CAN ����RAM �� ���Ķ���
* ���:CANx CAN�ṹ��
* ���: CAN_MSG ����ָ�� 
* ���: eMsgType �������� MSG_OBJ_TYPE_TX  MSG_OBJ_TYPE_TX_REMOTE
* 2008.12.5 Ϊ��ߴ����ٶ� ȥ��Զ��֡���� �����Ҫ����Զ��֡ Ҫ�޸ĳ���
*****************************************************************************/
void CAN_Tx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, MsgType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_TX) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE))
    
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {        //�ȴ��ӿڼĴ�������
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //���� INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //�������MSG ����A
                    CAN_IF1CMSK_DATAB |   //�������MSG ����B
                    CAN_IF1CMSK_CONTROL|  //�������MSG ����λ
                    CAN_IF1CMSK_ARB);     //�������MSG �ٲ�λ 
    if(eMsgType != MSG_OBJ_TYPE_TX)
     return;
    usArbReg[1] = (CAN_IF1ARB2_DIR|CAN_IF1ARB2_MSGVAL);    //���÷���λΪ���� ��������֡
    usMsgCtrl = (CAN_IF1MCTL_TXRQST| CAN_IF1MCTL_EOB);     //��λ��������  ��������֡
    usMsgCtrl |= CAN_MSG->CTL.BIT.LEN  ;                   //���ÿ���λ ���ݳ��� ���η���
    if(CAN_MSG->CTL.BIT.TX_INT_EN )                        //�жϷ����ж��Ƿ�ʹ��
     usMsgCtrl |= CAN_IF1MCTL_TXIE;                        //�����ж�ʹ��
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //��չ֡ 29BIT ID
      usArbReg[0]  = CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_XTD;
     }
    else
     {                      //��׼֡
      usArbReg[0] = 0;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
     }
    if(CAN_MSG->CTL.BIT.LEN!=0)
     {                                         //����֡ ���ݳ��Ȳ�Ϊ0��������
      CANDataRegWrite(&CAN_MSG->Data.BYTE[0],  //����������ָ��
                     (u32)&CANx->INF[0].DA1,   //CAN���ݼĴ�����ַ
                     CAN_MSG->CTL.BIT.LEN);    //���ݳ���)
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK , usCmdMaskReg);  //д�������μĴ���
    CANRegWrite((u32)&CANx->INF[0].ARB1 , usArbReg[0]);   //д�ٲüĴ���1
    CANRegWrite((u32)&CANx->INF[0].ARB2 , usArbReg[1]);   //д�ٲüĴ���2
    CANRegWrite((u32)&CANx->INF[0].MCTL , usMsgCtrl);     //д���Ŀ��ƼĴ���
    CANRegWrite((u32)&CANx->INF[0].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));//INF->MSG RAM
}
/*****************************************************************************
* ��ȡ CAN ����RAM �� ���Ķ���
* ���:CANx CAN�ṹ��
* ���: CAN_MSG ����ָ��
* ���: bClrPendingInt �Ƿ��������λ
* 2008.11.27 zlz ȡ�����������ȡ ������Զ��֡ 
*****************************************************************************/
void CAN_Rx_Msg(CAN_Typedef *CANx, CAN_MSG *CAN_MSG, u8 bClrPendingInt)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    usCmdMaskReg = (CAN_IF1CMSK_DATAA |   //�������DA
                    CAN_IF1CMSK_DATAB |   //�������DB
                    CAN_IF1CMSK_CONTROL | //���ʿ���λ
                    CAN_IF1CMSK_MASK |    //��������λ
                    CAN_IF1CMSK_ARB|      //�����ٲ�λ
                    CAN_IF1CMSK_NEWDAT);  //��������ݱ�־
    if(bClrPendingInt)                    //�ж��Ƿ����pendingλ
     usCmdMaskReg |= CAN_IF1CMSK_CLRINTPND;//���
    CANRegWrite((u32)&CANx->INF[1].CMSK , usCmdMaskReg); //д�������μĴ���
    CANRegWrite((u32)&CANx->INF[1].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M)); //MSG RAM->INF
    while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
     {                                     //�ȴ� MSG RAM->INF ���
     }
    usArbReg[0]  = CANRegRead(IntNumber,(u32)&CANx->INF[1].ARB1); //���ٲüĴ���1
    usArbReg[1]  = CANRegRead(IntNumber,(u32)&CANx->INF[1].ARB2); //���ٲüĴ���2
    usMsgCtrl    = CANRegRead(IntNumber,(u32)&CANx->INF[1].MCTL); //�����Ŀ��ƼĴ���
/*
    CAN_MSG->CTL.WORD = MSG_OBJ_NO_FLAGS;//������ı�־ 
    if((!(usMsgCtrl & CAN_IF1MCTL_TXRQST) && (usArbReg[1] & CAN_IF1ARB2_DIR))
     || ((usMsgCtrl & CAN_IF1MCTL_TXRQST) && (!(usArbReg[1] & CAN_IF1ARB2_DIR))))
     CAN_MSG->CTL.BIT.RMT_FRM = 1;       //Զ��֡��־
*/
    if(usArbReg[1] & CAN_IF1ARB2_XTD)
     {                                 //��չ֡
      CAN_MSG->ID.WORD = (((usArbReg[1] & CAN_IF1ARB2_ID_M) << 16) | usArbReg[0]);//ID
      CAN_MSG->CTL.BIT.EXD_ID = 1;  //����������չ֡��־
     }
    else							   //��׼֡
     CAN_MSG->ID.WORD = (usArbReg[1] & CAN_IF1ARB2_ID_M) << 6;//ID
/* 2008.12.5 zlz Ϊ��ߴ����ٶ�ȥ��
    if(usMsgCtrl & CAN_IF1MCTL_MSGLST)             //�ж��Ƿ��б��Ķ�ʧ ����Overrun 
     CAN_MSG->CTL.BIT.DATA_LOST = 1;            //���Ķ�ʧ��־ 
*/
    if(usMsgCtrl & CAN_IF1MCTL_NEWDAT)              //�ж��Ƿ���������
     {                                              //��������
      CAN_MSG->CTL.BIT.LEN = (usMsgCtrl & CAN_IF1MCTL_DLC_M);//���ݳ���
//      if(CAN_MSG->CTL.BIT.RMT_FRM == 0)
       {                                            //��ΪԶ��֡ 
        CANDataRegRead(IntNumber,					          //��CAN���ݼĴ��� �жϺ�
                         &CAN_MSG->Data.BYTE[0],	//��������ָ��
                         (u32)&CANx->INF[1].DA1,		//CAN���ݼĴ�����ַ
                         CAN_MSG->CTL.BIT.LEN);	//���ݳ���
       }
/*       
      CANRegWrite((u32)&CANx->INF[1].CMSK , CAN_IF1CMSK_NEWDAT);//��������ݱ�־
      CANRegWrite((u32)&CANx->INF[1].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {              //�ȴ�д�������
       }
      CAN_MSG->CTL.BIT.NEW_DATA = 1;              //��λ�������ݱ�־  
*/
     }
    else                                             //û������
     CAN_MSG->CTL.BIT.LEN = 0;                    //���ݳ���0 
}
/*****************************************************************************
* ��� CAN ����RAM �� ���Ķ���
* ���:CANx CAN�ṹ��
* ���: ulObjID ���ı��
*****************************************************************************/
void CANMessageClear(CAN_Typedef *CANx, u32 ulObjID)
{
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulObjID >= 1) && (ulObjID <= 32));
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {             //�ȴ��ӿڼĴ��������
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK , (CAN_IF1CMSK_WRNRD | //д INF->MSG RAM
                                           CAN_IF1CMSK_ARB));  //���� ARB�ٲ�λ
    CANRegWrite((u32)&CANx->INF[0].ARB1 , 0);                  // 
    CANRegWrite((u32)&CANx->INF[0].ARB2 , 0);                  //  
    CANRegWrite((u32)&CANx->INF[0].CRQ , (ulObjID & CAN_IF1CRQ_MNUM_M));
}
/*****************************************************************************
* ��� ���ķ���
* ���:CANx CAN�ṹ��
* ���: ulObjID ���ı��
*****************************************************************************/
void CAN_MSG_TX_CANCEL(CAN_Typedef *CANx, u32 ulObjID)
{
    u32 usMsgCtrl =	 CAN_IF1MCTL_EOB;                          //���Txrqst ��־
    u32 IntNumber = CANIntNumberGet(CANx);
    ASSERT(CANBaseValid(CANx));
    ASSERT((ulObjID >= 1) && (ulObjID <= 32));
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {             //�ȴ��ӿڼĴ��������
     }
    CANRegWrite((u32)&CANx->INF[0].CMSK , (CAN_IF1CMSK_WRNRD | //д INF->MSG RAM
                                           CAN_IF1CMSK_CONTROL|
										   CAN_IF1CMSK_CLRINTPND));//���ʿ���λ
    CANRegWrite((u32)&CANx->INF[0].MCTL , usMsgCtrl);          //д���Ŀ��ƼĴ���
    CANRegWrite((u32)&CANx->INF[0].CRQ , (ulObjID & CAN_IF1CRQ_MNUM_M));
    while(CANRegRead(IntNumber,(u32)&CANx->INF[0].CRQ) & CAN_IF1CRQ_BUSY)
     {             //�ȴ��ӿڼĴ��������
     }
}
