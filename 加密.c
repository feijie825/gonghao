/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : ����.c
;* Author             : ������
;* ��������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "define.h"
/*********************************************************************************************************
* JTAG ������I/O�ڶԳ�����м���
*********************************************************************************************************/
void JtagToGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //  ʹ��PB7                     
    GPIOPinTypeGPIOOutput(GPIOB, GPIO_PIN_7);    //  ����GPIO                    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //  ʹ��PC[3:0]                 
    GPIOC->LOCK = 0x1ACCE551;                    //  ����                        
    GPIOC->CR   = 0x000000FF;                    //  ȷ�Ϲ���                    
    GPIOC->LOCK = 0;                             //  ����                        
    GPIOC->AFSEL = 0;                            
    GPIOPinTypeGPIOOutput(GPIOC, GPIO_PIN_0|     //  ����GPIO                    
                                 GPIO_PIN_1|
                                 GPIO_PIN_2|
                                 GPIO_PIN_3);
}

/*********************************************************************************************************
* JTAG �����ӵȴ� PC5 ����JTAG���ȴ�  CPU.25
*********************************************************************************************************/
void JtagWait(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //  ʹ��KEY���ڵ�GPIO�˿�       
    GPIOPinTypeGPIOInput(GPIOC , PWM_DAC);       //  ����KEY���ڹܽ�Ϊ����       
    if (GPIOPinRead(GPIOC , PWM_DAC) )           //  �����λʱΪ��   
     {                                           //
      for (;;);                                  //  ��ѭ�����Եȴ�JTAG����      
     }                                           //
    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOC);//  ��ֹKEY���ڵ�GPIO�˿�       
}
