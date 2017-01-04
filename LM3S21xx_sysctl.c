/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_sysctl.c
;* Author             : ������
;* ϵͳ����
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"
//*********************ȡ�豸����DCx**************************************
#define SYSCTL_PERIPH_INDEX(a)  (((a) >> 8) & 0xf)

//*********************ȡ�豸����λ*******************************************
#define SYSCTL_PERIPH_MASK(a)   ( 1<< ((a) & 0xff))

/*****************************************************************************
* ϵͳ����Ƶ���б�
*****************************************************************************/
static const u32 g_pulXtals[] =
{
    1000000,
    1843200,
    2000000,
    2457600,
    3579545,
    3686400,
    4000000,
    4096000,
    4915200,
    5000000,
    5120000,
    6000000,
    6144000,
    7372800,
    8000000,
    8192000,
    10000000,
    12000000,
    12288000,
    13560000,
    14318180,
    16000000,
    16384000
};
/*****************************************************************************
* оƬ�豸��Ч �ж��ӳ���
*****************************************************************************/
#ifdef DEBUG
u8 SysCtlPeripheralValid(u32 ulPeripheral)
{
    return((ulPeripheral == SYSCTL_PERIPH_ADC) ||
           (ulPeripheral == SYSCTL_PERIPH_CAN0) ||
           (ulPeripheral == SYSCTL_PERIPH_CAN1) ||
           (ulPeripheral == SYSCTL_PERIPH_CAN2) ||
           (ulPeripheral == SYSCTL_PERIPH_COMP0) ||
           (ulPeripheral == SYSCTL_PERIPH_COMP1) ||
           (ulPeripheral == SYSCTL_PERIPH_COMP2) ||
           (ulPeripheral == SYSCTL_PERIPH_MAC) ||
           (ulPeripheral == SYSCTL_PERIPH_PHY) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOA) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOB) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOC) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOD) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOE) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOF) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOG) ||
           (ulPeripheral == SYSCTL_PERIPH_GPIOH) ||
           (ulPeripheral == SYSCTL_PERIPH_HIBERNATE) ||
           (ulPeripheral == SYSCTL_PERIPH_I2C0) ||
           (ulPeripheral == SYSCTL_PERIPH_I2C1) ||
           (ulPeripheral == SYSCTL_PERIPH_IEEE1588) ||
           (ulPeripheral == SYSCTL_PERIPH_MPU) ||
           (ulPeripheral == SYSCTL_PERIPH_PLL) ||
           (ulPeripheral == SYSCTL_PERIPH_PWM) ||
           (ulPeripheral == SYSCTL_PERIPH_QEI0) ||
           (ulPeripheral == SYSCTL_PERIPH_QEI1) ||
           (ulPeripheral == SYSCTL_PERIPH_SSI0) ||
           (ulPeripheral == SYSCTL_PERIPH_SSI1) ||
           (ulPeripheral == SYSCTL_PERIPH_TEMP) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER0) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER1) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER2) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER3) ||
           (ulPeripheral == SYSCTL_PERIPH_UART0) ||
           (ulPeripheral == SYSCTL_PERIPH_UART1) ||
           (ulPeripheral == SYSCTL_PERIPH_UART2) ||
           (ulPeripheral == SYSCTL_PERIPH_UDMA) ||
           (ulPeripheral == SYSCTL_PERIPH_USB0) ||
           (ulPeripheral == SYSCTL_PERIPH_WDOG));
}
#endif
/*****************************************************************************
* ��ʼ��ϵͳƵ���ӳ���
*****************************************************************************/
void Init_Pll(void)
{
    if(REVISION_IS_A2)	              //�ж���������
     SysCtlLDOSet(SYSCTL_LDO_2_75V);  //ϵͳ�ں˹�����ѹ2.75V
    SysCtlClockSet(SYSCTL_SYSDIV_8 |  //ϵͳ8��Ƶ ϵͳƵ��=200/8=25MHz 
                   SYSCTL_USE_PLL |   //ʹ�����໷PLL
                   SYSCTL_OSC_MAIN |  //ʹ��������ʱ��
                   SYSCTL_INT_OSC_DIS|//�����ڲ�����
                   SYSCTL_XTAL_8MHZ); //ϵͳ����Ƶ��8MHz
    Sysclk=SysCtlClockGet();          //��ȡϵͳƵ��
}
/*****************************************************************************
* ��ȡоƬSRAM��С�ӳ��� ����SRAM��С
*****************************************************************************/
u32 SysCtlSRAMSizeGet(void)
{
    return(((SYSCTL->DC0 & SYSCTL_DC0_SRAMSZ_M) >> 8) + 0x100);
}

/*****************************************************************************
* ��ȡоƬFLASH ��С�ӳ��� ����FLASH ��С
*****************************************************************************/
u32	SysCtlFlashSizeGet(void)
{
    return(((SYSCTL->DC0 & SYSCTL_DC0_FLASHSZ_M) << 11) + 0x800);
}

/*****************************************************************************
* ��� �ܽ��Ƿ�����ӳ���
* ���� ����TRUE ������ ���� FALSE
*****************************************************************************/
#ifdef DEBUG
u8 SysCtlPinPresent(u32 ulPin)
{
    ASSERT((ulPin == SYSCTL_PIN_PWM0) ||
           (ulPin == SYSCTL_PIN_PWM1) ||
           (ulPin == SYSCTL_PIN_PWM2) ||
           (ulPin == SYSCTL_PIN_PWM3) ||
           (ulPin == SYSCTL_PIN_PWM4) ||
           (ulPin == SYSCTL_PIN_PWM5) ||
           (ulPin == SYSCTL_PIN_C0MINUS) ||
           (ulPin == SYSCTL_PIN_C0PLUS) ||
           (ulPin == SYSCTL_PIN_C0O) ||
           (ulPin == SYSCTL_PIN_C1MINUS) ||
           (ulPin == SYSCTL_PIN_C1PLUS) ||
           (ulPin == SYSCTL_PIN_C1O) ||
           (ulPin == SYSCTL_PIN_C2MINUS) ||
           (ulPin == SYSCTL_PIN_C2PLUS) ||
           (ulPin == SYSCTL_PIN_C2O) ||
           (ulPin == SYSCTL_PIN_MC_FAULT0) ||
           (ulPin == SYSCTL_PIN_ADC0) ||
           (ulPin == SYSCTL_PIN_ADC1) ||
           (ulPin == SYSCTL_PIN_ADC2) ||
           (ulPin == SYSCTL_PIN_ADC3) ||
           (ulPin == SYSCTL_PIN_ADC4) ||
           (ulPin == SYSCTL_PIN_ADC5) ||
           (ulPin == SYSCTL_PIN_ADC6) ||
           (ulPin == SYSCTL_PIN_ADC7) ||
           (ulPin == SYSCTL_PIN_CCP0) ||
           (ulPin == SYSCTL_PIN_CCP1) ||
           (ulPin == SYSCTL_PIN_CCP2) ||
           (ulPin == SYSCTL_PIN_CCP3) ||
           (ulPin == SYSCTL_PIN_CCP4) ||
           (ulPin == SYSCTL_PIN_CCP5) ||
           (ulPin == SYSCTL_PIN_32KHZ));
    if(SYSCTL->DC[3] & ulPin)
     return(true);
    else
     return(false);
}
#endif

/*****************************************************************************
* ���оƬ�豸�Ƿ�����ӳ��� 
* ���:�豸��
* ���� ����TRUE ������ ���� FALSE
*****************************************************************************/
u8 SysCtlPeripheralPresent(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
    if(SYSCTL->DC[(SYSCTL_PERIPH_INDEX(ulPeripheral)+1)] & 	//��ȡоƬ�豸�Ĵ���
       SYSCTL_PERIPH_MASK(ulPeripheral))
     return(true);
    else
     return(false);
}

/*****************************************************************************
* оƬ�豸��λ�ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralReset(u32 ulPeripheral)
{
    u32 ulDelay;
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
    SYSCTL->SRCR[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);  //��λλ ��1
    for(ulDelay = 0; ulDelay < 16; ulDelay++)
     ;
    SYSCTL->SRCR[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral); //�����λ λ
}

/*****************************************************************************
* �豸ʱ��ʹ���ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralEnable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->RCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}
/*****************************************************************************
* �豸ʱ�ӽ����ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralDisable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->RCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* �豸˯��ʹ���ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralSleepEnable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->SCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* �豸˯�߽����ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralSleepDisable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->SCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* �豸���˯��ʹ���ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralDeepSleepEnable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->DCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* �豸���˯�߽����ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralDeepSleepDisable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->DCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* �豸ʱ���ſؿ��� 
* ��ڣ� bEnable=TRUE  ����˯��ģʽ����˯��ʱ�ӿ��ƼĴ��������豸ʱ��ʹ��
* ��ڣ� bEnable=FALSE ����˯��ģʽ��������ʱ�ӿ��ƼĴ��������豸ʱ��ʹ�� 
*****************************************************************************/
void SysCtlPeripheralClockGating(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_ACG;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_ACG);
}

/*****************************************************************************
* ����ϵͳ�жϷ��������ڲ�ʹ��ϵͳ�ж� INT_SYSCTL
* ����NVIC
*****************************************************************************/
void SysCtlIntRegister(void (*pfnHandler)(void))
{
    IntRegister(INT_SYSCTL, pfnHandler);
    IntEnable(INT_SYSCTL);
}

/*****************************************************************************
* ��ֹϵͳ�ж� ������ϵͳ�жϷ���������ΪIntDefaultHandler
* ����NVIC
*****************************************************************************/
void SysCtlIntUnregister(void)
{
    IntDisable(INT_SYSCTL);
    IntUnregister(INT_SYSCTL);
}

/*****************************************************************************
* ϵͳ�ж�ʹ�� ����ϵͳ�ж����μĴ���
* ��� ulInts �ж�ʹ�� PLLLIM(PLL LOCK �ж�ʹ��λ) BORIM(���縴λ�ж�ʹ��λ)
*****************************************************************************/
void SysCtlIntEnable(u32 ulInts)
{
    SYSCTL->IMC |= ulInts;
}

/*****************************************************************************
* ϵͳ�жϽ��� ����ϵͳ�ж����μĴ���
* ��� ulInts �жϽ��� PLLLIM(PLL LOCK �ж�ʹ��λ) BORIM(���縴λ�ж�ʹ��λ)
*****************************************************************************/
void SysCtlIntDisable(u32 ulInts)
{
    SYSCTL->IMC &= ~(ulInts);
}

/*****************************************************************************
* ����ж�״̬�Ĵ����� PLLLMIS BORMIS
* д1����ж�
*****************************************************************************/
void SysCtlIntClear(u32 ulInts)
{
    SYSCTL->MISC = ulInts;
}

/*****************************************************************************
* ��ϵͳ�ж�״̬�Ĵ���
* ��� bMasked=1�����κ���ж� bMasked=0 ��ԭʼ�ж�
*****************************************************************************/
u32	SysCtlIntStatus(u8 bMasked)
{
    if(bMasked)
     return(SYSCTL->MISC);
    else
     return(SYSCTL->RIS);
}

/*****************************************************************************
* ���� LDO�����ѹ LOW DROP-OUT
* ���ulVoltage LDO ��ѹֵ ����LM3S21xx_sysctl.h��ѡȡ
*****************************************************************************/
void SysCtlLDOSet(u32 ulVoltage)
{
    ASSERT((ulVoltage == SYSCTL_LDO_2_25V) ||
           (ulVoltage == SYSCTL_LDO_2_30V) ||
           (ulVoltage == SYSCTL_LDO_2_35V) ||
           (ulVoltage == SYSCTL_LDO_2_40V) ||
           (ulVoltage == SYSCTL_LDO_2_45V) ||
           (ulVoltage == SYSCTL_LDO_2_50V) ||
           (ulVoltage == SYSCTL_LDO_2_55V) ||
           (ulVoltage == SYSCTL_LDO_2_60V) ||
           (ulVoltage == SYSCTL_LDO_2_65V) ||
           (ulVoltage == SYSCTL_LDO_2_70V) ||
           (ulVoltage == SYSCTL_LDO_2_75V));
    SYSCTL->LDOPCTL = ulVoltage;
}

/*****************************************************************************
* ��ȡLDO������ƼĴ���
*****************************************************************************/
u32	SysCtlLDOGet(void)
{
    return(SYSCTL->LDOPCTL);
}
/*****************************************************************************
* ��LDO��λ���ƼĴ��� ����LM3S2XXX ��Ч
*****************************************************************************/
void SysCtlLDOConfigSet(u32 ulConfig)
{
    ASSERT((ulConfig == SYSCTL_LDOCFG_ARST) ||
           (ulConfig == SYSCTL_LDOCFG_NORST));
    SYSCTL->LDOARST = ulConfig;
}
/*****************************************************************************
* ����Ӧ���ж��븴λ���ƼĴ���
*****************************************************************************/
void SysCtlReset(void)
{
    HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    while(1)
     {
     }
}
/*****************************************************************************
* ��������ģʽ
*****************************************************************************/
void SysCtlSleep(void)
{
    CPUwfi();
}

/*****************************************************************************
* �����������ģʽ
*****************************************************************************/
void SysCtlDeepSleep(void)
{
    HWREG(NVIC_SYS_CTRL) |= NVIC_SYS_CTRL_SLEEPDEEP;
    CPUwfi();
    HWREG(NVIC_SYS_CTRL) &= ~(NVIC_SYS_CTRL_SLEEPDEEP);
}

/*****************************************************************************
* ��ȡ��λԴ
*****************************************************************************/
u32	SysCtlResetCauseGet(void)
{
    return(SYSCTL->RESC);
}

/*****************************************************************************
* �����λԴ״̬
*****************************************************************************/
void SysCtlResetCauseClear(u32 ulCauses)
{
    SYSCTL->RESC &= ~(ulCauses);
}

/*****************************************************************************
* ���õ��縴λ����
*****************************************************************************/
void SysCtlBrownOutConfigSet(u32 ulConfig, u32 ulDelay)
{
    ASSERT(!(ulConfig & ~(SYSCTL_BOR_RESET | SYSCTL_BOR_RESAMPLE)));
    ASSERT(ulDelay < 8192);
    SYSCTL->PBORCTL = (ulDelay << SYSCTL_PBORCTL_BORTIM_S) | ulConfig;
}

/*****************************************************************************
* ϵͳ������ʱ
*****************************************************************************/
#if defined(ewarm) || defined(DOXYGEN)
void SysCtlDelay(u32 ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
void __attribute__((naked))
SysCtlDelay(u32 ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(rvmdk) || defined(__ARMCC_VERSION)
__asm void
SysCtlDelay(u32 ulCount)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif

/*****************************************************************************
* ����ϵͳʱ�� ��������ο�LM3S2139.h��
*****************************************************************************/
void SysCtlClockSet(u32 ulConfig)
{
    u32 ulDelay, ulRCC, ulRCC2;
    if(CLASS_IS_SANDSTORM && (ulConfig & SYSCTL_RCC2_USERCC2))
     return;                          //SandStorm������RCC2�����滻RCC����ͬ��  
    SysRCC_CFG=ulConfig;              //��������ֵ ��ʱ���
    ulRCC = SYSCTL->RCC;              //��ȡRCC  ����ģʽʱ�����üĴ���
    ulRCC2 = SYSCTL->RCC2;            //��ȡRCC2 ����ģʽʱ�����üĴ���2
    ulRCC |= SYSCTL_RCC_BYPASS;       //����·PLL ��OSC����ϵͳʱ��
    ulRCC &= ~(SYSCTL_RCC_USESYSDIV); //�Ȳ���ϵͳʱ�ӷ�Ƶ��
    ulRCC2 |= SYSCTL_RCC2_BYPASS2;    //����·PLL ��OSC����ϵͳʱ��

    SYSCTL->RCC = ulRCC;              //����RCC  ����ģʽʱ�����üĴ���
    SysCtlDelay(16);                            //�ȴ��ȶ�
    ulRCC |= SYSCTL_RCC_PWRDN;        //����·PLL �ٵ���
    SYSCTL->RCC = ulRCC;              //����RCC  ����ģʽʱ�����üĴ���
    SYSCTL->RCC2 = ulRCC2;            //����RCC2 ����ģʽʱ�����üĴ���
    if(((ulRCC & SYSCTL_RCC_IOSCDIS) && !(ulConfig & SYSCTL_RCC_IOSCDIS))\
     ||((ulRCC & SYSCTL_RCC_MOSCDIS) && !(ulConfig & SYSCTL_RCC_MOSCDIS))) //�ڲ����������Ƿ�ı� ���������������Ƿ�ı�
     {                               //�ڲ��������ܸı� ���������������øı� ִ��
      ulRCC &= ((~(SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS))
               |(ulConfig & (SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS))); //��������ʹ��
      SYSCTL->RCC = ulRCC;    //����RCC��IOSCDIS ��	MOSCDISλ
      if(((ulRCC2 & SYSCTL_RCC2_USERCC2) &&							//ͨ��RCC2����
          (((ulRCC2 & SYSCTL_RCC2_OSCSRC2_M) == SYSCTL_RCC2_OSCSRC2_30) ||  //30KHz
           ((ulRCC2 & SYSCTL_RCC2_OSCSRC2_M) == SYSCTL_RCC2_OSCSRC2_32)))	  //32KHz
         ||(!(ulRCC2 & SYSCTL_RCC2_USERCC2) &&								  //��ͨ��RCC2����30KHz
           ((ulRCC & SYSCTL_RCC_OSCSRC_M) == SYSCTL_RCC_OSCSRC_30)))
       SysCtlDelay(4096);                      //ϵͳ��ʱ
      else                                     
       SysCtlDelay(524288);                    //ϵͳ��ʱ
     }                                           
    ulRCC &= ~(SYSCTL_RCC_XTAL_M |               //�������Ƶ������λ
               SYSCTL_RCC_OSCSRC_M |             //���ʱ��Դ����λ
               SYSCTL_RCC_PWRDN|                 //���PLL �ϵ�λ
               SYSCTL_RCC_OEN);                  //���PLL ���ʹ��λ
    ulRCC |= (ulConfig & (SYSCTL_RCC_XTAL_M |    //���þ���Ƶ������λ
                          SYSCTL_RCC_OSCSRC_M |  //����ʱ��Դ����λ  
                          SYSCTL_RCC_PWRDN |     //����PLL �ϵ�λ    
                          SYSCTL_RCC_OEN));      //����PLL ���ʹ��λ
    ulRCC2 &= ~(SYSCTL_RCC2_USERCC2|             //���RCC2ʹ��λ
                SYSCTL_RCC2_OSCSRC2_M|           //���RCC2ʱ��Դλ
                SYSCTL_RCC2_PWRDN2);             //���RCC2PLL �ϵ�λ
    ulRCC2 |= (ulConfig & (SYSCTL_RCC2_USERCC2|  //ʹ��RCC2�Ĵ���
                           SYSCTL_RCC_OSCSRC_M|  //����RCC2ʱ��Դλ
                           SYSCTL_RCC2_PWRDN2)); //����RCC2 PLL�ϵ�λ
    ulRCC2 |= ((ulConfig & 0x00000008) << 3);    //����RCC2.3
    SYSCTL->MISC = SYSCTL_INT_PLL_LOCK;          //���PLL����λ
    if(ulRCC2 & SYSCTL_RCC2_USERCC2)
     {
      SYSCTL->RCC2 = ulRCC2;
      SYSCTL->RCC = ulRCC;
     }
    else
     {
      SYSCTL->RCC = ulRCC;
      SYSCTL->RCC2 = ulRCC2;
     }
    SysCtlDelay(16);                            //�ȴ��ȶ�

    ulRCC &= ~(SYSCTL_RCC_SYSDIV_M|             //�����Ƶλ
               SYSCTL_RCC_USESYSDIV|            //�����Ƶʹ��λ
               SYSCTL_RCC_IOSCDIS|              //����ڲ���������λ
               SYSCTL_RCC_MOSCDIS);             //�������������λ
    ulRCC |= (ulConfig & (SYSCTL_RCC_SYSDIV_M|  //���÷�Ƶλ           
                          SYSCTL_RCC_USESYSDIV| //���÷�Ƶʹ��λ       
                          SYSCTL_RCC_IOSCDIS|   //�����ڲ���������λ 
                          SYSCTL_RCC_MOSCDIS)); //���ó�����������λ   
    ulRCC2 &= ~(SYSCTL_RCC2_SYSDIV2_M);         //���RCC2��Ƶλ
    ulRCC2 |= (ulConfig & SYSCTL_RCC2_SYSDIV2_M);//����RCC2��Ƶλ

    if(!(ulConfig & SYSCTL_RCC_BYPASS))         //�ж�PLL�Ƿ���·
     {                                          //ʹ��PLL
      for(ulDelay = 32768; ulDelay > 0; ulDelay--)
       {
        if(SYSCTL->RIS & SYSCTL_INT_PLL_LOCK)
         break;                                 //���໷���� ����ѭ��
       }
      ulRCC &= ~(SYSCTL_RCC_BYPASS);
      ulRCC2 &= ~(SYSCTL_RCC2_BYPASS2);
     }
    SYSCTL->RCC = ulRCC;
    SYSCTL->RCC2 = ulRCC2;
    SysCtlDelay(16);
}
/*****************************************************************************
* ��ȡϵͳʱ��Ƶ�� 
* ����ϵͳʱ��Ƶ��
*****************************************************************************/
u32	SysCtlClockGet(void)
{
    u32 ulRCC, ulRCC2, ulPLL, ulClk;
    ulRCC = SYSCTL->RCC;
    ulRCC2 = SYSCTL->RCC2;
    switch((ulRCC2 & SYSCTL_RCC2_USERCC2) ?
           (ulRCC2 & SYSCTL_RCC2_OSCSRC2_M) :
           (ulRCC & SYSCTL_RCC_OSCSRC_M))
    {
        case SYSCTL_RCC_OSCSRC_MAIN:
         {
          ulClk = g_pulXtals[(ulRCC & SYSCTL_RCC_XTAL_M) >>
                             SYSCTL_RCC_XTAL_S];
          break;
         }
        case SYSCTL_RCC_OSCSRC_INT:
         {
          if(CLASS_IS_SANDSTORM)
           ulClk = 15000000;
          else
           ulClk = 12000000;
          break;
         }
        case SYSCTL_RCC_OSCSRC_INT4:
         {
          if(CLASS_IS_SANDSTORM)
           ulClk = 15000000 / 4;
          else
           ulClk = 12000000 / 4;
          break;
         }
        case SYSCTL_RCC_OSCSRC_30:
         {
          ulClk = 30000;
          break;
         }
        case SYSCTL_RCC2_OSCSRC2_32:
         {
          ulClk = 32768;
          break;
         }
        default:
         return(0);
     }
    if(((ulRCC2 & SYSCTL_RCC2_USERCC2) && !(ulRCC2 & SYSCTL_RCC2_BYPASS2)) ||
       (!(ulRCC2 & SYSCTL_RCC2_USERCC2) && !(ulRCC & SYSCTL_RCC_BYPASS)))
     {
      ulPLL = SYSCTL->PLLCFG;
      if(CLASS_IS_SANDSTORM)
       ulClk = ((ulClk * (((ulPLL & SYSCTL_PLLCFG_F_M) >>
                           SYSCTL_PLLCFG_F_S) + 2)) /
                (((ulPLL & SYSCTL_PLLCFG_R_M) >>
                  SYSCTL_PLLCFG_R_S) + 2));
      else
       ulClk = ((ulClk * ((ulPLL & SYSCTL_PLLCFG_F_M) >>
                          SYSCTL_PLLCFG_F_S)) /
                ((((ulPLL & SYSCTL_PLLCFG_R_M) >>
                   SYSCTL_PLLCFG_R_S) + 1) * 2));
      if(ulPLL & SYSCTL_PLLCFG_OD_2)
       ulClk /= 2;
      if(ulPLL & SYSCTL_PLLCFG_OD_4)
       ulClk /= 4;
     }
    if(ulRCC & SYSCTL_RCC_USESYSDIV)
     {
      if(ulRCC2 & SYSCTL_RCC2_USERCC2)
       ulClk /= (((ulRCC2 & SYSCTL_RCC2_SYSDIV2_M) >>
                  SYSCTL_RCC2_SYSDIV2_S) + 1);
      else
       ulClk /= (((ulRCC & SYSCTL_RCC_SYSDIV_M) >> SYSCTL_RCC_SYSDIV_S) +
                 1);
     }
    return(ulClk);
}
/*****************************************************************************
* ����PWMϵͳʱ��Ƶ�� 
*****************************************************************************/
void SysCtlPWMClockSet(u32 ulConfig)
{
    ASSERT((ulConfig == SYSCTL_PWMDIV_1) ||
           (ulConfig == SYSCTL_PWMDIV_2) ||
           (ulConfig == SYSCTL_PWMDIV_4) ||
           (ulConfig == SYSCTL_PWMDIV_8) ||
           (ulConfig == SYSCTL_PWMDIV_16) ||
           (ulConfig == SYSCTL_PWMDIV_32) ||
           (ulConfig == SYSCTL_PWMDIV_64));
    ASSERT(SYSCTL->DC[1] & SYSCTL_DC1_PWM);
    SYSCTL->RCC = ((SYSCTL->RCC &
                          ~(SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_M)) |
                         ulConfig);
}
/*****************************************************************************
* ��ȡPWMϵͳʱ��Ƶ�� 
*****************************************************************************/
u32	SysCtlPWMClockGet(void)
{
    ASSERT(SYSCTL->DC[1] & SYSCTL_DC1_PWM);
    if(!(SYSCTL->RCC & SYSCTL_RCC_USEPWMDIV))
     return(SYSCTL_PWMDIV_1);
    else
     return(SYSCTL->RCC &
            (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_M));
}
/*****************************************************************************
* ����ADC�������� 
*****************************************************************************/
void SysCtlADCSpeedSet(u32 ulSpeed)
{
    ASSERT((ulSpeed == SYSCTL_ADCSPEED_1MSPS) ||
           (ulSpeed == SYSCTL_ADCSPEED_500KSPS) ||
           (ulSpeed == SYSCTL_ADCSPEED_250KSPS) ||
           (ulSpeed == SYSCTL_ADCSPEED_125KSPS));
    ASSERT(SYSCTL->DC[1] & SYSCTL_DC1_ADC);
    SYSCTL->RCGC[0] = ((SYSCTL->RCGC[0] & ~(SYSCTL_RCGC0_ADCSPD_M)) |
                           ulSpeed);
    SYSCTL->SCGC[0] = ((SYSCTL->SCGC[0] & ~(SYSCTL_SCGC0_ADCSPD_M)) |
                           ulSpeed);
    SYSCTL->DCGC[0] = ((SYSCTL->DCGC[0] & ~(SYSCTL_DCGC0_ADCSPD_M)) |
                           ulSpeed);
}
/*****************************************************************************
* ��ȡADC����Ƶ������ 
*****************************************************************************/
u32	SysCtlADCSpeedGet(void)
{
    ASSERT(SYSCTL->DC[1] & SYSCTL_DC1_ADC);
    return(SYSCTL->RCGC[0] & SYSCTL_RCGC0_ADCSPD_M);
}
/*****************************************************************************
* �ڲ�����У׼����
* bEnable=TRUE  У׼
* bEnable=FALSE ��У׼
*****************************************************************************/
void SysCtlIOSCVerificationSet(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_IOSCVER;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_IOSCVER);
}
/*****************************************************************************
* ������У׼����
* bEnable=TRUE  У׼
* bEnable=FALSE ��У׼
*****************************************************************************/
void SysCtlMOSCVerificationSet(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_MOSCVER;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_MOSCVER);
}
/*****************************************************************************
* PLLУ׼����
* bEnable=TRUE  У׼
* bEnable=FALSE ��У׼
*****************************************************************************/
void SysCtlPLLVerificationSet(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_PLLVER;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_PLLVER);
}
/*****************************************************************************
* ���ʱ��У׼
*****************************************************************************/
void SysCtlClkVerificationClear(void)
{
    SYSCTL->CLKVCLR = SYSCTL_CLKVCLR_VERCLR;
    SYSCTL->CLKVCLR = 0;
}
/*****************************************************************************
* GPIO AHB����ʹ��
*****************************************************************************/
void SysCtlGPIOAHBEnable(u32 ulGPIOPeripheral)
{
    ASSERT((ulGPIOPeripheral == SYSCTL_PERIPH_GPIOA) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOB) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOC) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOD) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOE) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOF) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOG) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOH));
    SYSCTL->GPIOHSCTL |= (ulGPIOPeripheral & 0xFFFF);
}
/*****************************************************************************
* GPIO AHB���ʽ���
*****************************************************************************/
void SysCtlGPIOAHBDisable(u32 ulGPIOPeripheral)
{
    ASSERT((ulGPIOPeripheral == SYSCTL_PERIPH_GPIOA) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOB) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOC) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOD) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOE) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOF) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOG) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOH));
    SYSCTL->GPIOHSCTL &= ~(ulGPIOPeripheral & 0xFFFF);
}
