/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_adc.c
;* Author             : 张力阵
;* 模数转换
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
* ADC初始化
*****************************************************************************/
void Init_Adc(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);      //使能ADC设备SYCTL_RCGC(使能时钟) 
	   SysCtlADCSpeedSet(SYSCTL_RCGC0_ADCSPD125K);		   //设置ADC采样速率
    ADCSequenceDisable(0);                          //禁能采样序列0
    ADCSequenceConfigure(0,ADC_TRIGGER_PROCESSOR,0);//设置为软件触发 最高优先级    
    ADCSequenceStepConfigure(0,0,ADC_CTL_CH0|       //采样序列0 采样ADC2   ADC_CTL_CH2
                                 ADC_CTL_IE|        //采样序列0 中断使能
                                 ADC_CTL_END);      //采样序列0 最后一个采样
                                 
    ADCHardwareOversampleConfigure(64);             //硬件平均64次采样取平均
	   ADCIntEnable(0);								//使能ADC sequence0中断
    ADCSequenceEnable(0);							//使能采样序列0
	   ADCProcessorTrigger(0);
}
/*****************************************************************************
* 设置ADC中断服务程序入口
* ulSequenceNum 通道序号
* pfnHandler 中断服务函数
* 中断向量表存放在SRAM 中时有效
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
* 撤销ADC中断服务程序入口
* ulSequenceNum 通道序号
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
* ADC中断禁能
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCIntDisable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->IM &= ~(1 << ulSequenceNum);
}
/*****************************************************************************
* ADC中断使能
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCIntEnable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ISC = 1 << ulSequenceNum; //清除中断状态
    ADC->IM |= 1 << ulSequenceNum; //使能中断
}
/*****************************************************************************
* 读取中断状态 
* ulSequenceNum 通道序号
* bMasked=1 读取屏蔽后的中断
* bMasked=0 读取原始中断
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
* 清除ADC中断状态 
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCIntClear(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ISC = 1 << ulSequenceNum;
}
/*****************************************************************************
* ADC 采样序列使能 
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCSequenceEnable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ACTSS |= 1 << ulSequenceNum;
}

/*****************************************************************************
* ADC 采样序列禁能 
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCSequenceDisable(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->ACTSS &= ~(1 << ulSequenceNum);
}

/*****************************************************************************
* 设置ADC触发源  
* ulSequenceNum 通道序号
* ulTrigger 触发源 ADC_TRIGGER_PROCESSOR ...
* ulPriority 优先级
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
* ADC采样顺序设置 
* ulSequenceNum 通道序号
* ulStep 选择设置第几个采样 0~7 
* ulConfig 设置采样 低4位设置 采样输入源(输入SSMUX) 高4位设置采样控制(输入SSCTL) ADC_CTL_TS ...
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
                                     ((ulConfig & 0x0f) << ulStep));//设置采样ADCx 输入

    ADC->SEQ[ulSequenceNum].SSCTL = ((ADC->SEQ[ulSequenceNum].SSCTL &(~(0x0000000f << ulStep))) |
                                     (((ulConfig & 0xf0) >> 4) << ulStep));
}

/*****************************************************************************
* 读取采样通道上溢出状态(FIFO满 写)  
* ulSequenceNum 通道序号
*****************************************************************************/
u32	ADCSequenceOverflow(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    return(ADC->OSTAT & (1 << ulSequenceNum));
}

/*****************************************************************************
* 清除采样通道上溢出状态  
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCSequenceOverflowClear(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->OSTAT = 1 << ulSequenceNum;
}

/*****************************************************************************
* 读取采样通道下溢出状态(FIFO空 读)  
* ulSequenceNum 通道序号
*****************************************************************************/
u32	ADCSequenceUnderflow(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    return(ADC->USTAT & (1 << ulSequenceNum));
}

/*****************************************************************************
* 清除采样通道下溢出状态(FIFO空 读)  
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCSequenceUnderflowClear(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->USTAT = 1 << ulSequenceNum;
}

/*****************************************************************************
* 读取采样通道的采样数据 
* 返回数据 pulBuffer
* 返回采样次数
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
* 软件触发采样通道采样 
* ulSequenceNum 通道序号
*****************************************************************************/
void ADCProcessorTrigger(u32 ulSequenceNum)
{
    ASSERT(ulSequenceNum < 4);
    ADC->PSSI = 1 << ulSequenceNum;
}

/*****************************************************************************
* 设置过采样系数 
* ulSequenceNum 通道序号
* ulFactor 设置系数 为2 4 或8(只有通道0) 
* 通道3不存在过采样
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
* ADC采样顺序设置 
* ulSequenceNum 通道序号
* ulStep 选择设置第几个采样 0~7 
* ulConfig 设置采样 低4位设置 采样输入源(输入SSMUX) 高4位设置采样控制(输入SSCTL) ADC_CTL_TS ...
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
* 读取过采样值
* ulSequenceNum 通道序号
* pulBuffer 采样值 累加和
* ulCount 采样次数
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
* 设置硬件过采样
* ulFactor 过采样系数 2 n 次方 
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
