/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : 加密.c
;* Author             : 张力阵
;* 变量定义
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "define.h"
/*********************************************************************************************************
* JTAG 口用作I/O口对程序进行加密
*********************************************************************************************************/
void JtagToGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //  使能PB7                     
    GPIOPinTypeGPIOOutput(GPIOB, GPIO_PIN_7);    //  用作GPIO                    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //  使能PC[3:0]                 
    GPIOC->LOCK = 0x1ACCE551;                    //  解锁                        
    GPIOC->CR   = 0x000000FF;                    //  确认功能                    
    GPIOC->LOCK = 0;                             //  锁定                        
    GPIOC->AFSEL = 0;                            
    GPIOPinTypeGPIOOutput(GPIOC, GPIO_PIN_0|     //  用作GPIO                    
                                 GPIO_PIN_1|
                                 GPIO_PIN_2|
                                 GPIO_PIN_3);
}

/*********************************************************************************************************
* JTAG 口连接等待 PC5 用于JTAG检测等待  CPU.25
*********************************************************************************************************/
void JtagWait(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //  使能KEY所在的GPIO端口       
    GPIOPinTypeGPIOInput(GPIOC , PWM_DAC);       //  设置KEY所在管脚为输入       
    if (GPIOPinRead(GPIOC , PWM_DAC) )           //  如果复位时为高   
     {                                           //
      for (;;);                                  //  死循环，以等待JTAG连接      
     }                                           //
    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOC);//  禁止KEY所在的GPIO端口       
}
