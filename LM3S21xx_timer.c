/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_timer.c
;* Author             : ������
;* ��ʱ/��������������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "Disp.h"
#include "define.h"
#include "vari.h"
/*****************************************************************************
* ��ʼ����ʱ��
* Timer0 Timer1 Timer2 ���ֳ�����������
* Timer0-A Timer0-B Timer1-A Timer1-B Timer2-A Timer2-B 
*****************************************************************************/
void Init_Timer(void)
{
//��ʼ��Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	//ʹ���豸ʱ��
    TimerConfigure(TIMER0,TIMER_CFG_16_BIT_PAIR|  //TIMER0�ֳ�����16bit ������
                          TIMER_CFG_A_CAP_COUNT|  //Timer0-A ���ؼ���ģʽ	CCP0���� ��������
                          TIMER_CFG_B_CAP_COUNT); //Timer0-B ���ؼ���ģʽ	CCP1���� ���Ⱦ�������
//��ʼ��Timer0-A ������������
    TimerControlEvent(TIMER0,                     //��ʱ��
                      TIMER_A,                    //ͨ�� 
                      TIMER_CTL_TAEVENT_NEG);     //�������뷽ʽ
    TimerLoadSet(TIMER0,                          //��ʱ��
                 TIMER_A,                         //ͨ��
                 1);                              //����(��Ƶϵ��)
    TimerMatchSet(TIMER0,                         //��ʱ��
                  TIMER_A,                        //ͨ��
                  0);                             //���ñȽ�ֵ
    TimerIntEnable(TIMER0,                        //��ʱ��
                   TIMER_TIMA_TIMEOUT);           //�ж�����
//    TimerEnable(TIMER0,                           //��ʱ��
//                TIMER_A);                         //��ʱ������
//��ʼ��Timer0-B ��׼������������                 //����CCP1������Ϊ�½���	 JZ_IN
    TimerControlEvent(TIMER0,                     //��ʱ��
                      TIMER_B,                    //ͨ�� 
                      TIMER_CTL_TBEVENT_NEG);     //�������뷽ʽ
    TimerLoadSet(TIMER0,                          //��ʱ��
                 TIMER_B,                         //ͨ��
                 0xFFFF);                         //������װֵ(��Ƶϵ��)
    TimerMatchSet(TIMER0,                         //��ʱ��
                  TIMER_B,                        //ͨ��
                  0);                             //���ñȽ�ֵ
    TimerIntEnable(TIMER0,                        //��ʱ��
                   TIMER_TIMB_TIMEOUT);           //�ж�����
    TimerEnable(TIMER0,                           //��ʱ��
                TIMER_B);                         //��ʱ������
//��ʼ��Timer1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	//ʹ���豸ʱ��
#ifdef __TEST
    TimerConfigure(TIMER1,TIMER_CFG_16_BIT_PAIR|  //TIMER1�ֳ�����16bit ������
                          TIMER_CFG_A_CAP_COUNT|  //Timer1-A ���ؼ���ģʽ	CCP2���� ʱ������
                          TIMER_CFG_B_PWM);       //Timer1-B PWM	CCP3��� ���� ��׼��������1M
#else
    TimerConfigure(TIMER1,TIMER_CFG_16_BIT_PAIR|  //TIMER1�ֳ�����16bit ������
                          TIMER_CFG_A_CAP_COUNT|  //Timer1-A ���ؼ���ģʽ	CCP2���� ʱ������
                          TIMER_CFG_B_CAP_COUNT); //Timer1-B ���ؼ���ģʽ	CCP3���� �����׼������
#endif
//��ʼ��Timer1-A ʱ����������                     //����CCP2������Ϊ�½���	 SZ_IN
    TimerControlEvent(TIMER1,                     //��ʱ��
                      TIMER_A,                    //ͨ�� 
                      TIMER_CTL_TAEVENT_NEG);     //�������뷽ʽ
    TimerLoadSet(TIMER1,                          //��ʱ��
                 TIMER_A,                         //ͨ��
                 60000);                          //������װֵ(��Ƶϵ��)
    TimerMatchSet(TIMER1,                         //��ʱ��
                  TIMER_A,                        //ͨ��
                  0);                             //���ñȽ�ֵ
    TimerIntEnable(TIMER1,                        //��ʱ��
                   TIMER_TIMA_TIMEOUT);           //�ж�����
    TimerEnable(TIMER1,                           //��ʱ��
                TIMER_A);                         //��ʱ������
//��ʼ��Timer1-B ������Ƶ����                   //����CCP3������Ϊ�½���	GP_IN
#ifdef __TEST
    TimerControlLevel(TIMER1,                     //��ʱ��
                      TIMER_B,                    //ͨ��
                      true);                      //CCP5�������      
    TimerControlEvent(TIMER1,                     //��ʱ��
                      TIMER_B,                    //ͨ��
                      TIMER_CTL_TBEVENT_NEG);     //�������뷽ʽ
    TimerLoadSet(TIMER1,                          //��ʱ��
                 TIMER_B,                         //ͨ��
                 24);                             //������װֵ1mHZ
    TimerMatchSet(TIMER1,                         //��ʱ�� 
                  TIMER_B,                        //ͨ��
                  12);                            //���ñȽ�ֵ
    TimerIntEnable(TIMER1,                        //��ʱ��
                   TIMER_TIMB_TIMEOUT);           //�жϷ�ʽ
    TimerEnable(TIMER1,                           //��ʱ��
                TIMER_B);                         //��ʱ������
#else
    TimerControlEvent(TIMER1,                     //��ʱ��
                      TIMER_B,                    //ͨ�� 
                      TIMER_EVENT_NEG_EDGE);      //�������뷽ʽ
    TimerLoadSet(TIMER1,                          //��ʱ��
                 TIMER_B,                         //ͨ��
                 1);                              //������װֵ(��Ƶϵ��)  2012.11.2 �谴��Ҫ����
    TimerMatchSet(TIMER1,                         //��ʱ��
                  TIMER_B,                        //ͨ��
                  0);                             //���ñȽ�ֵ
    TimerIntEnable(TIMER1,                        //��ʱ��
                   TIMER_TIMB_TIMEOUT);           //�ж�����
#endif
//    TimerEnable(TIMER1,                           //��ʱ��
//                TIMER_B);                         //��ʱ������
//��ʼ��Timer2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);   //ʹ���豸ʱ��
    TimerConfigure(TIMER2,TIMER_CFG_16_BIT_PAIR|    //TIMER2�ֳ�����16bit ������
                          TIMER_CFG_A_CAP_COUNT|    //Timer1-A ���ؼ���ģʽ	CCP4���� ��׼���Ƶ����
                          TIMER_CFG_B_PWM);         //Timer1-B PWM���ģʽ	CCP5��� PWM�������
//��ʼ��Timer2-A ��׼���Ƶ����
    TimerControlEvent(TIMER2,                     //��ʱ��
                      TIMER_A,                    //ͨ�� 
                      TIMER_CTL_TAEVENT_NEG);     //�������뷽ʽ
    TimerLoadSet(TIMER2,                          //��ʱ��
                 TIMER_A,                         //ͨ��
                 0xFFFF);                         //������װֵ(��Ƶϵ��)
    TimerMatchSet(TIMER2,                         //��ʱ��
                  TIMER_A,                        //ͨ��
                  0);                             //���ñȽ�ֵ
    TimerIntEnable(TIMER2,                        //��ʱ��
                   TIMER_TIMA_TIMEOUT);           //�ж�����
    TimerEnable(TIMER2,                           //��ʱ��
                TIMER_A);                         //��ʱ������
//��ʼ��Timer2-B PWM���                          //����CCP5
//��ʼ��Timer1-B ������Ƶ����                   //����CCP3������Ϊ�½���	GP_IN
#ifdef __TEST
    TimerControlLevel(TIMER2,                     //��ʱ��
                      TIMER_B,                    //ͨ��
                      true);                      //CCP5�������      
    TimerControlEvent(TIMER2,                     //��ʱ��
                      TIMER_B,                    //ͨ��
                      TIMER_CTL_TBEVENT_NEG);      //�������뷽ʽ
    TimerLoadSet(TIMER2,                          //��ʱ��
                 TIMER_B,                         //ͨ��
                 2495);                           //������װֵ10kHZ
    TimerMatchSet(TIMER2,                         //��ʱ�� 
                  TIMER_B,                        //ͨ��
                  1249);                          //���ñȽ�ֵ
    TimerIntEnable(TIMER2,                        //��ʱ��
                   TIMER_TIMB_TIMEOUT);           //�жϷ�ʽ
    TimerEnable(TIMER2,                           //��ʱ��
                TIMER_B);                         //��ʱ������
#endif
}
/*****************************************************************************
* ���TIMER�豸��ַ�Ƿ���ȷ
* TIMERx ��ʱ���ṹ��
* ����ʱ��Ч
*****************************************************************************/
#ifdef DEBUG
u8 TimerBaseValid(TIMER_Typedef *TIMERx)
{
    return(((u32)TIMERx == TIMER0_BASE) || ((u32)TIMERx == TIMER1_BASE) ||
           ((u32)TIMERx == TIMER2_BASE) || ((u32)TIMERx == TIMER3_BASE));
}
#endif

/*****************************************************************************
* TIMERʹ��(TimerA TimerB)
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
*****************************************************************************/
void TimerEnable(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    TIMERx->CTL |= ulTimer & (TIMER_CTL_TAEN | TIMER_CTL_TBEN);
}

/*****************************************************************************
* TIMER����(TimerA TimerB)
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
*****************************************************************************/
void TimerDisable(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    TIMERx->CTL &= ~(ulTimer & (TIMER_CTL_TAEN | TIMER_CTL_TBEN));
}

/*****************************************************************************
* TIMER����ģʽ����(TimerA TimerB)
* TIMERx ��ʱ���ṹ��
* �˿�:ulConfig bit0~bit7 ����TAMR�Ĵ��� bit8~bit15 ����TBMR bit24~bit32 ���� CFG �Ĵ��� 
* TIMER_CFG_32_BIT_OS TIMER_CFG_32_BIT_PER TIMER_CFG_32_RTC ���ó�32λ��ʱ��ģʽ
* TIMER_CFG_16_BIT_PAIR ���ó�˫16λ��ʱ����
* TIMER_CFG_A_ONE_SHOT... TimerA����ģʽ
* TIMER_CFG_B_ONE_SHOT... TimerB����ģʽ
*****************************************************************************/
void TimerConfigure(TIMER_Typedef *TIMERx, u32 ulConfig)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulConfig == TIMER_CFG_32_BIT_OS) ||
           (ulConfig == TIMER_CFG_32_BIT_PER) ||
           (ulConfig == TIMER_CFG_32_RTC) ||
           ((ulConfig & 0xff000000) == TIMER_CFG_16_BIT_PAIR));
    ASSERT(((ulConfig & 0xff000000) != TIMER_CFG_16_BIT_PAIR) ||
           ((((ulConfig & 0x000000ff) == TIMER_CFG_A_ONE_SHOT) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_PERIODIC) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_CAP_COUNT) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_CAP_TIME) ||
             ((ulConfig & 0x000000ff) == TIMER_CFG_A_PWM)) &&
            (((ulConfig & 0x0000ff00) == TIMER_CFG_B_ONE_SHOT) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_PERIODIC) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_CAP_COUNT) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_CAP_TIME) ||
             ((ulConfig & 0x0000ff00) == TIMER_CFG_B_PWM))));
    TIMERx->CTL &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN); //�Ƚ���
    TIMERx->CFG = ulConfig >> 24;                      //
    TIMERx->TAMR = ulConfig & 255;
    TIMERx->TBMR = (ulConfig >> 8) & 255;
}

/*****************************************************************************
* ���� TIMER PWM �����ƽ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* �˿�:bInvert�����ƽ 1 ������� 0 ���ֲ���
*****************************************************************************/
void TimerControlLevel(TIMER_Typedef *TIMERx, u32 ulTimer, u8 bInvert)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    ulTimer &= TIMER_CTL_TAPWML | TIMER_CTL_TBPWML;
    TIMERx->CTL = (bInvert ? (TIMERx->CTL | ulTimer) : (TIMERx->CTL & ~(ulTimer)));
}

/*****************************************************************************
* ���� TIMER �������Ƿ�ʹ��
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* �˿�:bEnable�������� 1 ����ʹ�� 0 ��������
* ����ADC���� ����
*****************************************************************************/
void TimerControlTrigger(TIMER_Typedef *TIMERx, u32 ulTimer,u8 bEnable)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    ulTimer &= TIMER_CTL_TAOTE | TIMER_CTL_TBOTE;
    TIMERx->CTL = (bEnable ? (TIMERx->CTL | ulTimer) : (TIMERx->CTL & ~(ulTimer)));
}

/*****************************************************************************
* ���� TIMER �����¼�
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* �˿�:ulEvent�����¼�����
*      TIMER_EVENT_POS_EDGE ������
*      TIMER_EVENT_NEG_EDGE �½���
*      TIMER_EVENT_BOTH_EDGES ˫����
*****************************************************************************/
void TimerControlEvent(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulEvent)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    if(ulTimer == TIMER_A)
	 {
      ulEvent &= TIMER_CTL_TAEVENT_M;
      TIMERx->CTL = ((TIMERx->CTL & ~TIMER_CTL_TAEVENT_M)|ulEvent);
	 }							    
    else if(ulTimer == TIMER_B)
	 {
      ulEvent &= TIMER_CTL_TBEVENT_M;
      TIMERx->CTL = ((TIMERx->CTL & ~TIMER_CTL_TBEVENT_M)|ulEvent);
	 }
	else if(ulTimer == TIMER_BOTH)
	 {
      ulEvent &= TIMER_CTL_TBEVENT_M;
      TIMERx->CTL = ((TIMERx->CTL & ~(TIMER_CTL_TAEVENT_M|TIMER_CTL_TBEVENT_M))|ulEvent);
	 }
}

/*****************************************************************************
* ���� ����ģʽ �Ƿ�����TIMER ֹͣ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* �˿�:bStall
*      true  ʹ��Timerֹͣ
*      false ��ֹTimerֹͣ
*****************************************************************************/
void TimerControlStall(TIMER_Typedef *TIMERx, u32 ulTimer,u8  bStall)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    ulTimer &= TIMER_CTL_TASTALL | TIMER_CTL_TBSTALL;
    TIMERx->CTL = (bStall ? (TIMERx->CTL | ulTimer) : (TIMERx->CTL & ~(ulTimer)));
}

/*****************************************************************************
* RTC ģʽ��Timer����ʹ��
* TIMERx ��ʱ���ṹ��
*****************************************************************************/
void TimerRTCEnable(TIMER_Typedef *TIMERx)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->CTL |= TIMER_CTL_RTCEN;
}
/*****************************************************************************
* RTC ģʽ��Timer��������(ֹͣ)
* TIMERx ��ʱ���ṹ��
*****************************************************************************/
void TimerRTCDisable(TIMER_Typedef *TIMERx)
{
    ASSERT(TimerBaseValid(TIMERx));

    TIMERx->CTL &= ~(TIMER_CTL_RTCEN);
}
/*****************************************************************************
* ���� TimerԤ��Ƶ 
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* �˿�:ulValue ��Ƶֵ(0~255)
*****************************************************************************/
void TimerPrescaleSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    ASSERT(ulValue < 256);

    if(ulTimer & TIMER_A)
     TIMERx->TAPR = ulValue;
    if(ulTimer & TIMER_B)
     TIMERx->TBPR = ulValue;
}
/*****************************************************************************
* ��ȡ TimerԤ��Ƶֵ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* ���� ��Ƶֵ(0~255)
*****************************************************************************/
u32 TimerPrescaleGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    return((ulTimer == TIMER_A) ? TIMERx->TAPR :TIMERx->TBPR);
}
/*****************************************************************************
* ���� Timer ���װ��ֵ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* ulValue ���װ��ֵ
*****************************************************************************/
void TimerLoadSet(TIMER_Typedef *TIMERx, u32 ulTimer,u32 ulValue)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    if(ulTimer & TIMER_A)
     TIMERx->TAILR = ulValue;
    if(ulTimer & TIMER_B)
     TIMERx->TBILR = ulValue;
}
/*****************************************************************************
* ��ȡ Timer ���װ��ֵ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* ���� ���װ��ֵ
*****************************************************************************/
u32 TimerLoadGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B));
    return((ulTimer == TIMER_A) ? TIMERx->TAILR :TIMERx->TBILR);
}
/*****************************************************************************
* ��ȡ Timer ��ǰ����ֵ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* ���� ��ǰ����ֵ
*****************************************************************************/
u32 TimerValueGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B));
    return((ulTimer == TIMER_A) ? TIMERx->TAR : TIMERx->TBR);
}
/*****************************************************************************
* ���� Timer ƥ��ֵ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* ulValue ƥ��ֵ
*****************************************************************************/
void TimerMatchSet(TIMER_Typedef *TIMERx, u32 ulTimer, u32 ulValue)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    if(ulTimer & TIMER_A)
     TIMERx->TAMATCHR = ulValue;
    if(ulTimer & TIMER_B)
     TIMERx->TBMATCHR = ulValue;
}
/*****************************************************************************
* ��ȡ Timer ƥ��ֵ
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* ���� ƥ��ֵ
*****************************************************************************/
u32 TimerMatchGet(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B));
    return((ulTimer == TIMER_A) ? TIMERx->TAMATCHR :  TIMERx->TBMATCHR);
}
/*****************************************************************************
* ����Timer�жϸ�λ�������
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* pfnHandler ����������
* ʹ��UART NVIC�ж� �ж���������RAM��ʱ ʹ��
*****************************************************************************/
void TimerIntRegister(TIMER_Typedef *TIMERx, u32 ulTimer,void (*pfnHandler)(void))
{
    u32  Int;
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));

    Int = (((u32)TIMERx == TIMER0_BASE) ? INT_TIMER0A :
           (((u32)TIMERx == TIMER1_BASE) ? INT_TIMER1A :
            (((u32)TIMERx == TIMER2_BASE) ? INT_TIMER2A : INT_TIMER3A)));

    if(ulTimer & TIMER_A)
     {
      IntRegister(Int, pfnHandler);
      IntEnable(Int);
     }
    if(ulTimer & TIMER_B)
     {
      IntRegister(Int + 1, pfnHandler);
      IntEnable(Int + 1);
     }
}
/*****************************************************************************
* ����Timer�жϸ�λ�������
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* ����UART NVIC�ж� �ж���������RAM��ʱ ʹ��
*****************************************************************************/
void TimerIntUnregister(TIMER_Typedef *TIMERx, u32 ulTimer)
{
    u32 Int;
    ASSERT(TimerBaseValid(TIMERx));
    ASSERT((ulTimer == TIMER_A) || (ulTimer == TIMER_B) ||
           (ulTimer == TIMER_BOTH));
    Int = (((u32)TIMERx == TIMER0_BASE) ? INT_TIMER0A :
           (((u32)TIMERx == TIMER1_BASE) ? INT_TIMER1A :
           (((u32)TIMERx == TIMER2_BASE) ? INT_TIMER2A : INT_TIMER3A)));
    if(ulTimer & TIMER_A)
     {
      IntDisable(Int);
      IntUnregister(Int);
     }
    if(ulTimer & TIMER_B)
     {
      IntDisable(Int + 1);
      IntUnregister(Int + 1);
     }
}
/*****************************************************************************
* Timer�ж�Դʹ������
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* �˿�:ulIntFlags �ж�ʹ��
*                 TIMER_CAPB_EVENT ...
*                 TIMER_CAPA_EVENT ...
*****************************************************************************/
void TimerIntEnable(TIMER_Typedef *TIMERx, u32 ulIntFlags)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->IMR |= ulIntFlags;
}

/*****************************************************************************
* Timer�ж�Դ��������
* TIMERx ��ʱ���ṹ��
* �˿�:ulTimer ΪTIMER_A TIMER_B ��TIMER_BOTH
* �˿�:ulIntFlags �жϽ���
*                 TIMER_CAPB_EVENT ...
*                 TIMER_CAPA_EVENT ...
*****************************************************************************/
void TimerIntDisable(TIMER_Typedef *TIMERx, u32 ulIntFlags)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->IMR &= ~(ulIntFlags);
}
/*****************************************************************************
* ��ȡTimer�ж�״̬
* TIMERx ��ʱ���ṹ��
*  bMasked=1 ��ȡ���κ���ж�״̬
*  bMasked=1 ��ȡԭʼ���ж�״̬
*****************************************************************************/
u32 TimerIntStatus(TIMER_Typedef *TIMERx, u8 bMasked)
{
    ASSERT(TimerBaseValid(TIMERx));
    return(bMasked ? TIMERx->MIS : TIMERx->RIS);
}

/*****************************************************************************
* ���Timer�ж�״̬
* TIMERx ��ʱ���ṹ��
* ulIntFlags Ҫ������жϱ�־
*****************************************************************************/
void TimerIntClear(TIMER_Typedef *TIMERx, u32 ulIntFlags)
{
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->ICR = ulIntFlags;
}

/*****************************************************************************
* Timer ��λ
* TIMERx ��ʱ���ṹ��
* ��λ��
*****************************************************************************/
void Reset_Timer(TIMER_Typedef *TIMERx)
{
    u32 Int;
    ASSERT(TimerBaseValid(TIMERx));
    TIMERx->CTL = TIMER_RV_CTL;
    TIMERx->IMR = TIMER_RV_IMR;
    TIMERx->ICR = 0xFFFFFFFF;
    Int = (((u32)TIMERx == TIMER0_BASE) ? INT_TIMER0A :
           (((u32)TIMERx == TIMER1_BASE) ? INT_TIMER1A :
           (((u32)TIMERx == TIMER2_BASE) ? INT_TIMER2A : INT_TIMER3A)));
    IntDisable(Int);
    IntDisable(Int + 1);
    TIMERx->CFG = TIMER_RV_CFG;
    TIMERx->TAMR = TIMER_RV_TAMR;
    TIMERx->TBMR = TIMER_RV_TBMR;
    TIMERx->RIS = TIMER_RV_RIS;
    TIMERx->MIS = TIMER_RV_MIS;
    TIMERx->TAILR = TIMER_RV_TAILR;
    TIMERx->TBILR = TIMER_RV_TBILR;
    TIMERx->TAMATCHR = TIMER_RV_TAMATCHR;
    TIMERx->TBMATCHR = TIMER_RV_TBMATCHR;
    TIMERx->TAPR = TIMER_RV_TAPR;
    TIMERx->TBPR = TIMER_RV_TBPR;
    TIMERx->TAPMR = TIMER_RV_TAPMR;
    TIMERx->TBPMR = TIMER_RV_TBPMR;
    TIMERx->TAR = TIMER_RV_TAR;
    TIMERx->TBR = TIMER_RV_TBR;
}

