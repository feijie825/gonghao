/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_adc.h
;* Author             : 张力阵
;* 模数转换声明
*******************************************************************************/

#ifndef __ADC_H__
#define __ADC_H__

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
// Values that can be passed to ADCSequenceConfigure as the ulTrigger
// parameter.
//
//*****************************************************************************
#define ADC_TRIGGER_PROCESSOR   0x00000000  // Processor event
#define ADC_TRIGGER_COMP0       0x00000001  // Analog comparator 0 event
#define ADC_TRIGGER_COMP1       0x00000002  // Analog comparator 1 event
#define ADC_TRIGGER_COMP2       0x00000003  // Analog comparator 2 event
#define ADC_TRIGGER_EXTERNAL    0x00000004  // External event
#define ADC_TRIGGER_TIMER       0x00000005  // Timer event
#define ADC_TRIGGER_PWM0        0x00000006  // PWM0 event
#define ADC_TRIGGER_PWM1        0x00000007  // PWM1 event
#define ADC_TRIGGER_PWM2        0x00000008  // PWM2 event
#define ADC_TRIGGER_ALWAYS      0x0000000F  // Always event

//*****************************************************************************
//
// Values that can be passed to ADCSequenceStepConfigure as the ulConfig
// parameter.
//
//*****************************************************************************
#define ADC_CTL_TS              0x00000080  // Temperature sensor select
#define ADC_CTL_IE              0x00000040  // Interrupt enable
#define ADC_CTL_END             0x00000020  // Sequence end select
#define ADC_CTL_D               0x00000010  // Differential select
#define ADC_CTL_CH0             0x00000000  // Input channel 0 采样ADC0
#define ADC_CTL_CH1             0x00000001  // Input channel 1 采样ADC1
#define ADC_CTL_CH2             0x00000002  // Input channel 2 采样ADC2
#define ADC_CTL_CH3             0x00000003  // Input channel 3 采样ADC3
#define ADC_CTL_CH4             0x00000004  // Input channel 4 采样ADC4
#define ADC_CTL_CH5             0x00000005  // Input channel 5 采样ADC5
#define ADC_CTL_CH6             0x00000006  // Input channel 6 采样ADC6
#define ADC_CTL_CH7             0x00000007  // Input channel 7 采样ADC7

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void ADCIntRegister(u32 ulSequenceNum,void (*pfnHandler)(void));
extern void ADCIntUnregister(u32 ulSequenceNum);
extern void ADCIntDisable(u32 ulSequenceNum);
extern void ADCIntEnable(u32 ulSequenceNum);
extern u32 ADCIntStatus(u32 ulSequenceNum,u8 bMasked);
extern void ADCIntClear(u32 ulSequenceNum);
extern void ADCSequenceEnable(u32 ulSequenceNum);
extern void ADCSequenceDisable(u32 ulSequenceNum);
extern void ADCSequenceConfigure(u32 ulSequenceNum,u32 ulTrigger,u32 ulPriority);
extern void	ADCSequenceStepConfigure(u32 ulSequenceNum,u32 ulStep, u32 ulConfig);
extern u32 ADCSequenceOverflow(u32 ulSequenceNum);
extern void ADCSequenceOverflowClear(u32 ulSequenceNum);
extern u32 ADCSequenceUnderflow(u32 ulSequenceNum);
extern void ADCSequenceUnderflowClear(u32 ulSequenceNum);
extern u32 ADCSequenceDataGet(u32 ulSequenceNum,u32 *pulBuffer);
extern void ADCProcessorTrigger(u32 ulSequenceNum);
extern void ADCSoftwareOversampleConfigure(u32 ulSequenceNum,u32 ulFactor);
extern void ADCSoftwareOversampleStepConfigure(u32 ulSequenceNum,u32 ulStep,u32 ulConfig);
extern void ADCSoftwareOversampleDataGet(u32 ulSequenceNum,u32 *pulBuffer,u32 ulCount);
extern void ADCHardwareOversampleConfigure(u32 ulFactor);
extern void Init_Adc(void);	// ADC初始化

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __ADC_H__
