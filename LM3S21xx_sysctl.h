/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_sysctl.h
;* Author             : 张力阵
;* 系统控制预定义
*******************************************************************************/

#ifndef __SYSCTL_H__
#define __SYSCTL_H__

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

/*****************************************************************************
* 
* The following are values that can be passed to the
* SysCtlPeripheralPresent(), SysCtlPeripheralEnable(),
* SysCtlPeripheralDisable(), and SysCtlPeripheralReset() APIs as the
* ulPeripheral parameter.  The peripherals in the fourth group (upper nibble
* is 3) can only be used with the SysCtlPeripheralPresent() API.
* 2008.11.7 ZLZ更改和LM3S2139.h中SYSCTL结构体定义一致
*****************************************************************************/
//BIT0~BIT4 低5位 设备所在DCx位置 BIT8~BIT15 设备所在DCx(DC1~DC4) ID
#define SYSCTL_PERIPH_JTAG      ((0<<8)|0 )  // DC1.0  JTAG	  
#define SYSCTL_PERIPH_SWD       ((0<<8)|1 )  // DC1.1  SWD
#define SYSCTL_PERIPH_SWO       ((0<<8)|2 )  // DC1.2  SWO
#define SYSCTL_PERIPH_WDOG      ((0<<8)|3 )  // DC1.3  Watchdog	  
#define SYSCTL_PERIPH_PLL       ((0<<8)|4 )  // DC1.4  PLL
#define SYSCTL_PERIPH_TEMP      ((0<<8)|5 )  // DC1.5  Temperature sensor
#define SYSCTL_PERIPH_HIBERNATE ((0<<8)|6 )  // DC1.6  Hibernation module
#define SYSCTL_PERIPH_MPU       ((0<<8)|7 )  // DC1.7  Cortex M3 MPU
#define SYSCTL_PERIPH_ADC       ((0<<8)|16)  // DC1.16 ADC
#define SYSCTL_PERIPH_PWM       ((0<<8)|20)  // DC1.20 PWM
#define SYSCTL_PERIPH_CAN0      ((0<<8)|24)  // DC1.24 CAN 0
#define SYSCTL_PERIPH_CAN1      ((0<<8)|25)  // DC1.25 CAN 1
#define SYSCTL_PERIPH_CAN2      ((0<<8)|26)  // DC1.26 CAN 2
#define SYSCTL_PERIPH_UART0     ((1<<8)|0 )  // DC2.0  UART 0
#define SYSCTL_PERIPH_UART1     ((1<<8)|1 )  // DC2.1  UART 1
#define SYSCTL_PERIPH_UART2     ((1<<8)|2 )  // DC2.2  UART 2
#define SYSCTL_PERIPH_SSI0      ((1<<8)|4 )  // DC2.4  SSI 0
#define SYSCTL_PERIPH_SSI1      ((1<<8)|5 )  // DC2.5  SSI 1
#define SYSCTL_PERIPH_QEI0      ((1<<8)|8 )  // DC2.8  QEI 0
#define SYSCTL_PERIPH_QEI1      ((1<<8)|9 )  // DC2.9  QEI 1
#define SYSCTL_PERIPH_I2C0      ((1<<8)|12)  // DC2.12 I2C 0
#define SYSCTL_PERIPH_I2C1      ((1<<8)|14)  // DC2.14 I2C 1
#define SYSCTL_PERIPH_TIMER0    ((1<<8)|16)  // DC2.16 Timer 0
#define SYSCTL_PERIPH_TIMER1    ((1<<8)|17)  // DC2.17 Timer 1
#define SYSCTL_PERIPH_TIMER2    ((1<<8)|18)  // DC2.18 Timer 2
#define SYSCTL_PERIPH_TIMER3    ((1<<8)|19)  // DC2.19 Timer 3
#define SYSCTL_PERIPH_COMP0     ((1<<8)|24)  // DC2.24 Analog comparator 0
#define SYSCTL_PERIPH_COMP1     ((1<<8)|25)  // DC2.25 Analog comparator 1
#define SYSCTL_PERIPH_COMP2     ((1<<8)|26)  // DC2.26 Analog comparator 2
#define SYSCTL_PERIPH_GPIOA     ((3<<8)|0 )  // DC4.0  GPIO A
#define SYSCTL_PERIPH_GPIOB     ((3<<8)|1 )  // DC4.1  GPIO B
#define SYSCTL_PERIPH_GPIOC     ((3<<8)|2 )  // DC4.2  GPIO C
#define SYSCTL_PERIPH_GPIOD     ((3<<8)|3 )  // DC4.3  GPIO D
#define SYSCTL_PERIPH_GPIOE     ((3<<8)|4 )  // DC4.4  GPIO E
#define SYSCTL_PERIPH_GPIOF     ((3<<8)|5 )  // DC4.5  GPIO F
#define SYSCTL_PERIPH_GPIOG     ((3<<8)|6 )  // DC4.6  GPIO G
#define SYSCTL_PERIPH_GPIOH     ((3<<8)|7 )  // DC4.7  GPIO H
#define SYSCTL_PERIPH_UDMA      ((3<<8)|13)  // DC4.13 uDMA
#define SYSCTL_PERIPH_USB0      ((3<<8)|16)  // DC4.16 USB0
#define SYSCTL_PERIPH_MAC       ((3<<8)|28)  // DC4.28 ETH MAC
#define SYSCTL_PERIPH_PHY       ((3<<8)|30)  // DC4.30 ETH MAC
#define SYSCTL_PERIPH_IEEE1588  ((3<<8)|24)  // DC4.24 IEEE1588
//*****************************************************************************
//
// The following are values that can be passed to the SysCtlPinPresent() API
// as the ulPin parameter.
//
//*****************************************************************************
#define SYSCTL_PIN_PWM0         0x00000001  // PWM0 pin
#define SYSCTL_PIN_PWM1         0x00000002  // PWM1 pin
#define SYSCTL_PIN_PWM2         0x00000004  // PWM2 pin
#define SYSCTL_PIN_PWM3         0x00000008  // PWM3 pin
#define SYSCTL_PIN_PWM4         0x00000010  // PWM4 pin
#define SYSCTL_PIN_PWM5         0x00000020  // PWM5 pin
#define SYSCTL_PIN_PWM6         0x00000040  // PWM6 pin
#define SYSCTL_PIN_PWM7         0x00000080  // PWM7 pin
#define SYSCTL_PIN_C0MINUS      0x00000040  // C0- pin
#define SYSCTL_PIN_C0PLUS       0x00000080  // C0+ pin
#define SYSCTL_PIN_C0O          0x00000100  // C0o pin
#define SYSCTL_PIN_C1MINUS      0x00000200  // C1- pin
#define SYSCTL_PIN_C1PLUS       0x00000400  // C1+ pin
#define SYSCTL_PIN_C1O          0x00000800  // C1o pin
#define SYSCTL_PIN_C2MINUS      0x00001000  // C2- pin
#define SYSCTL_PIN_C2PLUS       0x00002000  // C2+ pin
#define SYSCTL_PIN_C2O          0x00004000  // C2o pin
#define SYSCTL_PIN_MC_FAULT0    0x00008000  // MC0 Fault pin
#define SYSCTL_PIN_ADC0         0x00010000  // ADC0 pin
#define SYSCTL_PIN_ADC1         0x00020000  // ADC1 pin
#define SYSCTL_PIN_ADC2         0x00040000  // ADC2 pin
#define SYSCTL_PIN_ADC3         0x00080000  // ADC3 pin
#define SYSCTL_PIN_ADC4         0x00100000  // ADC4 pin
#define SYSCTL_PIN_ADC5         0x00200000  // ADC5 pin
#define SYSCTL_PIN_ADC6         0x00400000  // ADC6 pin
#define SYSCTL_PIN_ADC7         0x00800000  // ADC7 pin
#define SYSCTL_PIN_CCP0         0x01000000  // CCP0 pin
#define SYSCTL_PIN_CCP1         0x02000000  // CCP1 pin
#define SYSCTL_PIN_CCP2         0x04000000  // CCP2 pin
#define SYSCTL_PIN_CCP3         0x08000000  // CCP3 pin
#define SYSCTL_PIN_CCP4         0x10000000  // CCP4 pin
#define SYSCTL_PIN_CCP5         0x20000000  // CCP5 pin
#define SYSCTL_PIN_32KHZ        0x80000000  // 32kHz pin

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlLDOSet() API as
// the ulVoltage value, or returned by the SysCtlLDOGet() API.
//
//*****************************************************************************
#define SYSCTL_LDO_2_25V        0x00000005  // LDO output of 2.25V
#define SYSCTL_LDO_2_30V        0x00000004  // LDO output of 2.30V
#define SYSCTL_LDO_2_35V        0x00000003  // LDO output of 2.35V
#define SYSCTL_LDO_2_40V        0x00000002  // LDO output of 2.40V
#define SYSCTL_LDO_2_45V        0x00000001  // LDO output of 2.45V
#define SYSCTL_LDO_2_50V        0x00000000  // LDO output of 2.50V
#define SYSCTL_LDO_2_55V        0x0000001f  // LDO output of 2.55V
#define SYSCTL_LDO_2_60V        0x0000001e  // LDO output of 2.60V
#define SYSCTL_LDO_2_65V        0x0000001d  // LDO output of 2.65V
#define SYSCTL_LDO_2_70V        0x0000001c  // LDO output of 2.70V
#define SYSCTL_LDO_2_75V        0x0000001b  // LDO output of 2.75V

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlLDOConfigSet() API.
//
//*****************************************************************************
#define SYSCTL_LDOCFG_ARST      0x00000001  // Allow LDO failure to reset
#define SYSCTL_LDOCFG_NORST     0x00000000  // Do not reset on LDO failure

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlIntEnable(),
// SysCtlIntDisable(), and SysCtlIntClear() APIs, or returned in the bit mask
// by the SysCtlIntStatus() API.
//
//*****************************************************************************
#define SYSCTL_INT_MOSC_PUP     0x00000100  // MOSC power-up interrupt
#define SYSCTL_INT_USBPLL_LOCK  0x00000080  // USB PLL lock interrupt
#define SYSCTL_INT_PLL_LOCK     0x00000040  // PLL lock interrupt
#define SYSCTL_INT_CUR_LIMIT    0x00000020  // Current limit interrupt
#define SYSCTL_INT_IOSC_FAIL    0x00000010  // Internal oscillator failure int
#define SYSCTL_INT_MOSC_FAIL    0x00000008  // Main oscillator failure int
#define SYSCTL_INT_POR          0x00000004  // Power on reset interrupt
#define SYSCTL_INT_BOR          0x00000002  // Brown out interrupt
#define SYSCTL_INT_PLL_FAIL     0x00000001  // PLL failure interrupt

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlResetCauseClear()
// API or returned by the SysCtlResetCauseGet() API.
//
//*****************************************************************************
#define SYSCTL_CAUSE_LDO        0x00000020  // LDO power not OK reset
#define SYSCTL_CAUSE_SW         0x00000010  // Software reset
#define SYSCTL_CAUSE_WDOG       0x00000008  // Watchdog reset
#define SYSCTL_CAUSE_BOR        0x00000004  // Brown-out reset
#define SYSCTL_CAUSE_POR        0x00000002  // Power on reset
#define SYSCTL_CAUSE_EXT        0x00000001  // External reset

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlBrownOutConfigSet()
// API as the ulConfig parameter.
//
//*****************************************************************************
#define SYSCTL_BOR_RESET        0x00000002  // Reset instead of interrupting
#define SYSCTL_BOR_RESAMPLE     0x00000001  // Resample BOR before asserting

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlPWMClockSet() API
// as the ulConfig parameter, and can be returned by the SysCtlPWMClockGet()
// API.
//
//*****************************************************************************
#define SYSCTL_PWMDIV_1         0x00000000  // PWM clock is processor clock /1
#define SYSCTL_PWMDIV_2         0x00100000  // PWM clock is processor clock /2
#define SYSCTL_PWMDIV_4         0x00120000  // PWM clock is processor clock /4
#define SYSCTL_PWMDIV_8         0x00140000  // PWM clock is processor clock /8
#define SYSCTL_PWMDIV_16        0x00160000  // PWM clock is processor clock /16
#define SYSCTL_PWMDIV_32        0x00180000  // PWM clock is processor clock /32
#define SYSCTL_PWMDIV_64        0x001A0000  // PWM clock is processor clock /64

//SysCtlADCSpeedSet()入口参数
//SyCtlADCSpeedGet()返回参数
//*****************************************************************************
#define SYSCTL_ADCSPEED_1MSPS   0x00000300  // 1,000,000 samples per second
#define SYSCTL_ADCSPEED_500KSPS 0x00000200  // 500,000 samples per second
#define SYSCTL_ADCSPEED_250KSPS 0x00000100  // 250,000 samples per second
#define SYSCTL_ADCSPEED_125KSPS 0x00000000  // 125,000 samples per second

//SysCtlClockSet()函数参数
//*****************************************************************************
#define SYSCTL_SYSDIV_1         ((0<<31)|(2<<22))   //系统频率=pll  /1
#define SYSCTL_SYSDIV_2         ((0<<31)|(3<<22))   //系统频率=pll  /2
#define SYSCTL_SYSDIV_3         ((0<<31)|(5<<22))   //系统频率=pll  /3
#define SYSCTL_SYSDIV_4         ((0<<31)|(7<<22))   //系统频率=pll  /4
#define SYSCTL_SYSDIV_5         ((0<<31)|(9<<22))   //系统频率=pll  /5
#define SYSCTL_SYSDIV_6         ((0<<31)|(11<<22))  //系统频率=pll  /6
#define SYSCTL_SYSDIV_7         ((0<<31)|(13<<22))  //系统频率=pll  /7
#define SYSCTL_SYSDIV_8         ((0<<31)|(15<<22))  //系统频率=pll  /8
#define SYSCTL_SYSDIV_9         ((0<<31)|(17<<22))  //系统频率=pll  /9
#define SYSCTL_SYSDIV_10        ((0<<31)|(19<<22))  //系统频率=pll  /10
#define SYSCTL_SYSDIV_11        ((0<<31)|(21<<22))  //系统频率=pll  /11
#define SYSCTL_SYSDIV_12        ((0<<31)|(23<<22))  //系统频率=pll  /12
#define SYSCTL_SYSDIV_13        ((0<<31)|(25<<22))  //系统频率=pll  /13
#define SYSCTL_SYSDIV_14        ((0<<31)|(27<<22))  //系统频率=pll  /14
#define SYSCTL_SYSDIV_15        ((0<<31)|(29<<22))  //系统频率=pll  /15
#define SYSCTL_SYSDIV_16        ((0<<31)|(31<<22))  //系统频率=pll  /16
#define SYSCTL_SYSDIV_17        ((1<<31)|(33<<22))  //系统频率=pll  /17
#define SYSCTL_SYSDIV_18        ((1<<31)|(35<<22))  //系统频率=pll  /18
#define SYSCTL_SYSDIV_19        ((1<<31)|(37<<22))  //系统频率=pll  /19
#define SYSCTL_SYSDIV_20        ((1<<31)|(38<<22))  //系统频率=pll  /20
#define SYSCTL_SYSDIV_21        ((1<<31)|(39<<22))  //系统频率=pll  /21
#define SYSCTL_SYSDIV_22        ((1<<31)|(41<<22))  //系统频率=pll  /22
#define SYSCTL_SYSDIV_23        ((1<<31)|(43<<22))  //系统频率=pll  /23
#define SYSCTL_SYSDIV_24        ((1<<31)|(45<<22))  //系统频率=pll  /24
#define SYSCTL_SYSDIV_25        ((1<<31)|(47<<22))  //系统频率=pll  /25
#define SYSCTL_SYSDIV_26        ((1<<31)|(49<<22))  //系统频率=pll  /26
#define SYSCTL_SYSDIV_27        ((1<<31)|(51<<22))  //系统频率=pll  /27
#define SYSCTL_SYSDIV_28        ((1<<31)|(53<<22))  //系统频率=pll  /28
#define SYSCTL_SYSDIV_29        ((1<<31)|(55<<22))  //系统频率=pll  /29
#define SYSCTL_SYSDIV_30        ((1<<31)|(57<<22))  //系统频率=pll  /30
#define SYSCTL_SYSDIV_31        ((1<<31)|(58<<22))  //系统频率=pll  /31
#define SYSCTL_SYSDIV_32        ((1<<31)|(59<<22))  //系统频率=pll  /32
#define SYSCTL_SYSDIV_33        ((1<<31)|(61<<22))  //系统频率=pll  /33
#define SYSCTL_SYSDIV_34        ((1<<31)|(63<<22))  //系统频率=pll  /34
#define SYSCTL_SYSDIV_35        ((1<<31)|(65<<22))  //系统频率=pll  /35
#define SYSCTL_SYSDIV_36        ((1<<31)|(67<<22))  //系统频率=pll  /36
#define SYSCTL_SYSDIV_37        ((1<<31)|(68<<22))  //系统频率=pll  /37
#define SYSCTL_SYSDIV_38        ((1<<31)|(69<<22))  //系统频率=pll  /38
#define SYSCTL_SYSDIV_39        ((1<<31)|(71<<22))  //系统频率=pll  /39
#define SYSCTL_SYSDIV_40        ((1<<31)|(73<<22))  //系统频率=pll  /40
#define SYSCTL_SYSDIV_41        ((1<<31)|(75<<22))  //系统频率=pll  /41
#define SYSCTL_SYSDIV_42        ((1<<31)|(77<<22))  //系统频率=pll  /42
#define SYSCTL_SYSDIV_43        ((1<<31)|(78<<22))  //系统频率=pll  /43
#define SYSCTL_SYSDIV_44        ((1<<31)|(79<<22))  //系统频率=pll  /44
#define SYSCTL_SYSDIV_45        ((1<<31)|(81<<22))  //系统频率=pll  /45
#define SYSCTL_SYSDIV_46        ((1<<31)|(83<<22))  //系统频率=pll  /46
#define SYSCTL_SYSDIV_47        ((1<<31)|(85<<22))  //系统频率=pll  /47
#define SYSCTL_SYSDIV_48        ((1<<31)|(87<<22))  //系统频率=pll  /48
#define SYSCTL_SYSDIV_49        ((1<<31)|(88<<22))  //系统频率=pll  /49
#define SYSCTL_SYSDIV_50        ((1<<31)|(89<<22))  //系统频率=pll  /50
#define SYSCTL_SYSDIV_51        ((1<<31)|(91<<22))  //系统频率=pll  /51
#define SYSCTL_SYSDIV_52        ((1<<31)|(93<<22))  //系统频率=pll  /52
#define SYSCTL_SYSDIV_53        ((1<<31)|(95<<22))  //系统频率=pll  /53
#define SYSCTL_SYSDIV_54        ((1<<31)|(97<<22))  //系统频率=pll  /54
#define SYSCTL_SYSDIV_55        ((1<<31)|(98<<22))  //系统频率=pll  /55
#define SYSCTL_SYSDIV_56        ((1<<31)|(99<<22))  //系统频率=pll  /56
#define SYSCTL_SYSDIV_57        ((1<<31)|(101<<22)) //系统频率=pll  /57
#define SYSCTL_SYSDIV_58        ((1<<31)|(103<<22)) //系统频率=pll  /58
#define SYSCTL_SYSDIV_59        ((1<<31)|(105<<22)) //系统频率=pll  /59
#define SYSCTL_SYSDIV_60        ((1<<31)|(107<<22)) //系统频率=pll  /60
#define SYSCTL_SYSDIV_61        ((1<<31)|(108<<22)) //系统频率=pll  /61
#define SYSCTL_SYSDIV_62        ((1<<31)|(109<<22)) //系统频率=pll  /62
#define SYSCTL_SYSDIV_63        ((1<<31)|(111<<22)) //系统频率=pll  /63
#define SYSCTL_SYSDIV_64        ((1<<31)|(113<<22)) //系统频率=pll  /64
#define SYSCTL_USE_PLL          (0<<11)     // PLL输出作为系统时钟
#define SYSCTL_USE_OSC          (7<<11)     // 振荡器作为系统时钟
#define SYSCTL_XTAL_1MHZ        (0<<6)      // 1MHz         
#define SYSCTL_XTAL_1_84MHZ     (1<<6)      // 1.8432MHz    
#define SYSCTL_XTAL_2MHZ        (2<<6)      // 2MHz         
#define SYSCTL_XTAL_2_45MHZ     (3<<6)      // 2.4576MHz    
#define SYSCTL_XTAL_3_57MHZ     (4<<6)      // 3.579545MHz  
#define SYSCTL_XTAL_3_68MHZ     (5<<6)      // 3.6864MHz    
#define SYSCTL_XTAL_4MHZ        (6<<6)      // 4MHz         
#define SYSCTL_XTAL_4_09MHZ     (7<<6)      // 4.096MHz     
#define SYSCTL_XTAL_4_91MHZ     (8<<6)      // 4.9152MHz    
#define SYSCTL_XTAL_5MHZ        (9<<6)      // 5MHz         
#define SYSCTL_XTAL_5_12MHZ     (10<<6)     // 5.12MHz      
#define SYSCTL_XTAL_6MHZ        (11<<6)     // 6MHz 复位值    
#define SYSCTL_XTAL_6_14MHZ     (12<<6)     // 6.144MHz     
#define SYSCTL_XTAL_7_37MHZ     (13<<6)     // 7.3728MHz    
#define SYSCTL_XTAL_8MHZ        (14<<6)     // 8MHz         
#define SYSCTL_XTAL_8_19MHZ     (15<<6)     // 8.192MHz     
#define SYSCTL_XTAL_10MHZ       (16<<6)     // 10 MHz       
#define SYSCTL_XTAL_12MHZ       (17<<6)     // 12 MHz       
#define SYSCTL_XTAL_12_2MHZ     (18<<6)     // 12.288 MHz   
#define SYSCTL_XTAL_13_5MHZ     (19<<6)     // 13.56 MHz    
#define SYSCTL_XTAL_14_3MHZ     (20<<6)     // 14.31818 MHz 
#define SYSCTL_XTAL_16MHZ       (21<<6)     // 16 MHz       
#define SYSCTL_XTAL_16_3MHZ     (22<<6)     // 16.384 MHz   
#define SYSCTL_OSC_MAIN         (0<<4)      // 主振荡器       
#define SYSCTL_OSC_INT          (1<<4)      // 内部振荡器
#define SYSCTL_OSC_INT4         (2<<4)      // 内部振荡器/4
#define SYSCTL_OSC_INT30        (3<<4)      // 内部30KHz 振荡器
#define SYSCTL_OSC_EXT32        ((1<<31)|(8<<4))  // 内部32KHz 振荡器
#define SYSCTL_INT_OSC_DIS      (1<<1)      // 禁能内部振荡器
#define SYSCTL_MAIN_OSC_DIS     (1<<0)      // 禁能主振荡器

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void Init_Pll(void);		               //初始化系统频率子程序
extern u32 SysCtlSRAMSizeGet(void);
extern u32 SysCtlFlashSizeGet(void);
extern u8 SysCtlPinPresent(u32 ulPin);
extern u8 SysCtlPeripheralPresent(u32 ulPeripheral);
extern void SysCtlPeripheralReset(u32 ulPeripheral);
extern void SysCtlPeripheralEnable(u32 ulPeripheral);
extern void SysCtlPeripheralDisable(u32 ulPeripheral);
extern void SysCtlPeripheralSleepEnable(u32 ulPeripheral);
extern void SysCtlPeripheralSleepDisable(u32 ulPeripheral);
extern void SysCtlPeripheralDeepSleepEnable(u32 ulPeripheral);
extern void SysCtlPeripheralDeepSleepDisable(u32 ulPeripheral);
extern void SysCtlPeripheralClockGating(u8 bEnable);
extern void SysCtlIntRegister(void (*pfnHandler)(void));
extern void SysCtlIntUnregister(void);
extern void SysCtlIntEnable(u32 ulInts);
extern void SysCtlIntDisable(u32 ulInts);
extern void SysCtlIntClear(u32 ulInts);
extern u32 SysCtlIntStatus(u8 bMasked);
extern void SysCtlLDOSet(u32 ulVoltage);
extern u32 SysCtlLDOGet(void);
extern void SysCtlLDOConfigSet(u32 ulConfig);
extern void SysCtlReset(void);
extern void SysCtlSleep(void);
extern void SysCtlDeepSleep(void);
extern u32 SysCtlResetCauseGet(void);
extern void SysCtlResetCauseClear(u32 ulCauses);
extern void SysCtlBrownOutConfigSet(u32 ulConfig,u32 ulDelay);
extern void SysCtlDelay(u32 ulCount);
extern void SysCtlClockSet(u32 ulConfig);
extern u32 SysCtlClockGet(void);
extern void SysCtlPWMClockSet(u32 ulConfig);
extern u32 SysCtlPWMClockGet(void);
extern void SysCtlADCSpeedSet(u32 ulSpeed);
extern u32 SysCtlADCSpeedGet(void);
extern void SysCtlIOSCVerificationSet(u8 bEnable);
extern void SysCtlMOSCVerificationSet(u8 bEnable);
extern void SysCtlPLLVerificationSet(u8 bEnable);
extern void SysCtlClkVerificationClear(void);
extern void SysCtlGPIOAHBEnable(u32 ulGPIOPeripheral);
extern void SysCtlGPIOAHBDisable(u32 ulGPIOPeripheral);
extern void SysCtlUSBPLLEnable(void);
extern void SysCtlUSBPLLDisable(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __SYSCTL_H__
