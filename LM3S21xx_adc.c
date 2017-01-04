/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_adc.c
;* Author             : ������
;* ģ��ת��
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"

//*****************************************************************************
//
// The currently configured software oversampling factor for each of the ADC
// sequencers.
//
//*****************************************************************************
u8 g_pucOversampleFactor[3];
/*****************************************************************************
* ADC��ʼ��
*****************************************************************************/
void Init_Adc(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);      //ʹ��ADC�豸SYCTL_RCGC(ʹ��ʱ��) 
	   SysCtlADCSpeedSet(SYSCTL_RCGC0_ADCSPD125K);		   //����ADC��������
    ADCSequenceDisable(0);                          //���ܲ�������0
    ADCSequenceConfigure(0,ADC_TRIGGER_PROCESSOR,0);//����Ϊ������� ������ȼ�    
    ADCSequenceStepConfigure(0,0,ADC_CTL_CH0|       //��������0 ����ADC2   ADC_CTL_CH2
                                 ADC_CTL_IE|        //��������0 �ж�ʹ��
                                 ADC_CTL_END);      //��������0 ���һ������
                                 
    ADCHardwareOversampleConfigure(64);             //Ӳ��ƽ��64�β���ȡƽ��
	   ADCIntEnable(0);								//ʹ��ADC sequence0�ж�
    ADCSequenceEnable(0);							//ʹ�ܲ�������0
	   ADCProcessorTrigger(0);
}
/*****************************************************************************
* ����ADC�жϷ���������
* ulSequenceNum ͨ�����
* pfnHandler �жϷ�����
* �ж�����������SRAM ��ʱ��Ч
*****************************************************************************/
void ADCIntRegister(u32 ulSequenceNum,void (*pfnHandler)(void))
{
    u32 ulInt;
    
    ASSERT(ulSequenceNum < 4);
    ulInt = INT_ADC0 + ulSequenceNum;
    IntRegister(ulInt, pfnHandler);
    IntEnable(ulInt);
}
/*****************************************************************************
* ����ADC�жϷ���������
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCIntUnregister(u32 ulSequenceNum)
{
    u32 ulInt;
    ASSERT(ulSequenceNum < 4);
    ulInt = INT_ADC0 + ulSequenceNum;
    IntDisable(ulInt);
    IntUnregister(ulInt);
}
/*****************************************************************************
* ADC�жϽ���
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCIntDisable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->IM &= ~(1 << ulSequenceNum);
}
/*****************************************************************************
* ADC�ж�ʹ��
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCIntEnable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ISC = 1 << ulSequenceNum; //����ж�״̬
    ADC->IM |= 1 << ulSequenceNum; //ʹ���ж�
}
/*****************************************************************************
* ��ȡ�ж�״̬ 
* ulSequenceNum ͨ�����
* bMasked=1 ��ȡ���κ���ж�
* bMasked=0 ��ȡԭʼ�ж�
*****************************************************************************/
u32	ADCIntStatus(u32 ulSequenceNum,u8 bMasked)
{
    ASSERT(ulSequenceNum < 4);
    if(bMasked)
     return(ADC->ISC & (1 << ulSequenceNum));
    else
     return(ADC->RIS & (1 << ulSequenceNum));
}
/*****************************************************************************
* ���ADC�ж�״̬ 
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCIntClear(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ISC = 1 << ulSequenceNum;
}
/*****************************************************************************
* ADC ��������ʹ�� 
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCSequenceEnable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ACTSS |= 1 << ulSequenceNum;
}

/*****************************************************************************
* ADC �������н��� 
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCSequenceDisable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ACTSS &= ~(1 << ulSequenceNum);
}

/*****************************************************************************
* ����ADC����Դ  
* ulSequenceNum ͨ�����
* ulTrigger ����Դ ADC_TRIGGER_PROCESSOR ...
* ulPriority ���ȼ�
*****************************************************************************/
void ADCSequenceConfigure(u32 ulSequenceNum,u32 ulTrigger, u32 ulPriority)
{
    ASSERT(ulSequenceNum < 4);
    ASSERT((ulTrigger == ADC_TRIGGER_PROCESSOR) ||
           (ulTrigger == ADC_TRIGGER_COMP0) ||
           (ulTrigger == ADC_TRIGGER_COMP1) ||
           (ulTrigger == ADC_TRIGGER_COMP2) ||
           (ulTrigger == ADC_TRIGGER_EXTERNAL) ||
           (ulTrigger == ADC_TRIGGER_TIMER) ||
           (ulTrigger == ADC_TRIGGER_PWM0) ||
           (ulTrigger == ADC_TRIGGER_PWM1) ||
           (ulTrigger == ADC_TRIGGER_PWM2) ||
           (ulTrigger == ADC_TRIGGER_ALWAYS));
    ASSERT(ulPriority < 4);
    ulSequenceNum *= 4;
    ADC->EMUX = ((ADC->EMUX &    (~(0xf << ulSequenceNum))) |
                 ((ulTrigger & 0xf) << ulSequenceNum));
    ADC->SSPRI = ((ADC->SSPRI &  (~(0xf << ulSequenceNum))) |
                  ((ulPriority & 0x3) << ulSequenceNum));
}

/*****************************************************************************
* ADC����˳������ 
* ulSequenceNum ͨ�����
* ulStep ѡ�����õڼ������� 0~7 
* ulConfig ���ò��� ��4λ���� ��������Դ(����SSMUX) ��4λ���ò�������(����SSCTL) ADC_CTL_TS ...
*****************************************************************************/
void ADCSequenceStepConfigure(u32 ulSequenceNum,u32 ulStep, u32 ulConfig)
{
    ASSERT(ulSequenceNum < 4);
    ASSERT(((ulSequenceNum == 0) && (ulStep < 8)) ||
           ((ulSequenceNum == 1) && (ulStep < 4)) ||
           ((ulSequenceNum == 2) && (ulStep < 4)) ||
           ((ulSequenceNum == 3) && (ulStep < 1)));
    ulStep *= 4;
    ADC->SEQ[ulSequenceNum].SSMUX = ((ADC->SEQ[ulSequenceNum].SSMUX &(~(0x0000000f << ulStep))) |
                                     ((ulConfig & 0x0f) << ulStep));//���ò���ADCx ����

    ADC->SEQ[ulSequenceNum].SSCTL = ((ADC->SEQ[ulSequenceNum].SSCTL &(~(0x0000000f << ulStep))) |
                                     (((ulConfig & 0xf0) >> 4) << ulStep));
}

/*****************************************************************************
* ��ȡ����ͨ�������״̬(FIFO�� д)  
* ulSequenceNum ͨ�����
*****************************************************************************/
u32	ADCSequenceOverflow(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    return(ADC->OSTAT & (1 << ulSequenceNum));
}

/*****************************************************************************
* �������ͨ�������״̬  
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCSequenceOverflowClear(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->OSTAT = 1 << ulSequenceNum;
}

/*****************************************************************************
* ��ȡ����ͨ�������״̬(FIFO�� ��)  
* ulSequenceNum ͨ�����
*****************************************************************************/
u32	ADCSequenceUnderflow(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    return(ADC->USTAT & (1 << ulSequenceNum));
}

/*****************************************************************************
* �������ͨ�������״̬(FIFO�� ��)  
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCSequenceUnderflowClear(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->USTAT = 1 << ulSequenceNum;
}

/*****************************************************************************
* ��ȡ����ͨ���Ĳ������� 
* �������� pulBuffer
* ���ز�������
*****************************************************************************/
u32	ADCSequenceDataGet(u32 ulSequenceNum,u32 *pulBuffer)
{
    u32 ulCount;
    ASSERT(ulSequenceNum < 4);
    ulCount = 0;
    while((!(ADC->SEQ[ulSequenceNum].SSFSTAT & ADC_SSFSTAT0_EMPTY)) && (ulCount < 8))
     {
      *pulBuffer++ = ADC->SEQ[ulSequenceNum].SSFIFO;
      ulCount++;
     }
    return(ulCount);
}
/*****************************************************************************
* �����������ͨ������ 
* ulSequenceNum ͨ�����
*****************************************************************************/
void ADCProcessorTrigger(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->PSSI = 1 << ulSequenceNum;
}

/*****************************************************************************
* ���ù�����ϵ�� 
* ulSequenceNum ͨ�����
* ulFactor ����ϵ�� Ϊ2 4 ��8(ֻ��ͨ��0) 
* ͨ��3�����ڹ�����
*****************************************************************************/
void ADCSoftwareOversampleConfigure(u32 ulSequenceNum,u32 ulFactor)
{
    ASSERT(ulSequenceNum < 3);
    ASSERT(((ulFactor == 2) || (ulFactor == 4) || (ulFactor == 8)) &&
           ((ulSequenceNum == 0) || (ulFactor != 8)));
	if(ulFactor==2)
	 g_pucOversampleFactor[ulSequenceNum] =1;
	else if(ulFactor==4)
	 g_pucOversampleFactor[ulSequenceNum] =2;
	else if((ulFactor == 8)&& (ulSequenceNum == 0))
	 g_pucOversampleFactor[ulSequenceNum] =3;
}

/*****************************************************************************
* ADC����˳������ 
* ulSequenceNum ͨ�����
* ulStep ѡ�����õڼ������� 0~7 
* ulConfig ���ò��� ��4λ���� ��������Դ(����SSMUX) ��4λ���ò�������(����SSCTL) ADC_CTL_TS ...
*****************************************************************************/
void ADCSoftwareOversampleStepConfigure(u32 ulSequenceNum,u32 ulStep,u32 ulConfig)
{
    ASSERT(ulSequenceNum < 3);
    ASSERT(((ulSequenceNum == 0) && (ulStep < (8 >> g_pucOversampleFactor[ulSequenceNum]))) ||
           (ulStep < (4 >> g_pucOversampleFactor[ulSequenceNum])));

    ulStep *= (4 << g_pucOversampleFactor[ulSequenceNum]);
    ulSequenceNum = (1 << g_pucOversampleFactor[ulSequenceNum]);
    for(;ulSequenceNum; ulSequenceNum--)
     {
      ADC->SEQ[ulSequenceNum].SSMUX = ((ADC->SEQ[ulSequenceNum].SSMUX & (~(0x0000000f << ulStep))) |
                                       ((ulConfig & 0x0f) << ulStep));
      ADC->SEQ[ulSequenceNum].SSCTL = ((ADC->SEQ[ulSequenceNum].SSCTL & (~(0x0000000f << ulStep))) |
                                       (((ulConfig & 0xf0) >> 4) << ulStep));
      if(ulSequenceNum != 1)
       ADC->SEQ[ulSequenceNum].SSCTL &= (~((ADC_SSCTL0_IE0 |ADC_SSCTL0_END0) << ulStep));
      ulStep += 4;
     }
}

/*****************************************************************************
* ��ȡ������ֵ
* ulSequenceNum ͨ�����
* pulBuffer ����ֵ �ۼӺ�
* ulCount ��������
*****************************************************************************/
void ADCSoftwareOversampleDataGet(u32 ulSequenceNum,u32 *pulBuffer, u32 ulCount)
{
    u32 ulIdx, ulAccum;

    ASSERT(ulSequenceNum < 3);
    ASSERT(((ulSequenceNum == 0) &&(ulCount < (8 >> g_pucOversampleFactor[ulSequenceNum]))) ||
           (ulCount < (4 >> g_pucOversampleFactor[ulSequenceNum])));
    while(ulCount--)
     {
      ulAccum = 0;
      ulIdx = (1 << g_pucOversampleFactor[ulSequenceNum]);
      for(; ulIdx; ulIdx--)
       ulAccum += ADC->SEQ[ulSequenceNum].SSFIFO;
      *pulBuffer++ = (ulAccum >> g_pucOversampleFactor[ulSequenceNum]);
     }
}

/*****************************************************************************
* ����Ӳ��������
* ulFactor ������ϵ�� 2 n �η� 
*****************************************************************************/
void ADCHardwareOversampleConfigure(u32 ulFactor)
{
    u32 ulValue;
    ASSERT(((ulFactor == 0) || (ulFactor == 2) || (ulFactor == 4) ||
           (ulFactor == 8) || (ulFactor == 16) || (ulFactor == 32) ||
           (ulFactor == 64)));
    for(ulValue = 0, ulFactor >>= 1; ulFactor; ulValue++, ulFactor >>= 1)
     {
     }
    ADC->SAC = ulValue;
}
