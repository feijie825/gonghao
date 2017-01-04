/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_sysctl.c
;* Author             : 张力阵
;* 系统控制
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"
//*********************取设备所在DCx**************************************
#define SYSCTL_PERIPH_INDEX(a)  (((a) >> 8) & 0xf)

//*********************取设备所在位*******************************************
#define SYSCTL_PERIPH_MASK(a)   ( 1<< ((a) & 0xff))

/*****************************************************************************
* 系统晶振频率列表
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
* 芯片设备有效 判断子程序
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
* 初始化系统频率子程序
*****************************************************************************/
void Init_Pll(void)
{
    if(REVISION_IS_A2)	              //判断器件类型
     SysCtlLDOSet(SYSCTL_LDO_2_75V);  //系统内核工作电压2.75V
    SysCtlClockSet(SYSCTL_SYSDIV_8 |  //系统8分频 系统频率=200/8=25MHz 
                   SYSCTL_USE_PLL |   //使用锁相环PLL
                   SYSCTL_OSC_MAIN |  //使用主振荡器时钟
                   SYSCTL_INT_OSC_DIS|//禁能内部振荡器
                   SYSCTL_XTAL_8MHZ); //系统晶振频率8MHz
    Sysclk=SysCtlClockGet();          //获取系统频率
}
/*****************************************************************************
* 获取芯片SRAM大小子程序 返回SRAM大小
*****************************************************************************/
u32 SysCtlSRAMSizeGet(void)
{
    return(((SYSCTL->DC0 & SYSCTL_DC0_SRAMSZ_M) >> 8) + 0x100);
}

/*****************************************************************************
* 获取芯片FLASH 大小子程序 返回FLASH 大小
*****************************************************************************/
u32	SysCtlFlashSizeGet(void)
{
    return(((SYSCTL->DC0 & SYSCTL_DC0_FLASHSZ_M) << 11) + 0x800);
}

/*****************************************************************************
* 检查 管脚是否存在子程序
* 存在 返回TRUE 不存在 返回 FALSE
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
* 检查芯片设备是否存在子程序 
* 入口:设备号
* 存在 返回TRUE 不存在 返回 FALSE
*****************************************************************************/
u8 SysCtlPeripheralPresent(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
    if(SYSCTL->DC[(SYSCTL_PERIPH_INDEX(ulPeripheral)+1)] & 	//读取芯片设备寄存器
       SYSCTL_PERIPH_MASK(ulPeripheral))
     return(true);
    else
     return(false);
}

/*****************************************************************************
* 芯片设备复位子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralReset(u32 ulPeripheral)
{
    u32 ulDelay;
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
    SYSCTL->SRCR[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);  //复位位 置1
    for(ulDelay = 0; ulDelay < 16; ulDelay++)
     ;
    SYSCTL->SRCR[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral); //清除复位 位
}

/*****************************************************************************
* 设备时钟使能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralEnable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->RCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}
/*****************************************************************************
* 设备时钟禁能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralDisable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->RCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* 设备睡眠使能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralSleepEnable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->SCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* 设备睡眠禁能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralSleepDisable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->SCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* 设备深度睡眠使能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralDeepSleepEnable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->DCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* 设备深度睡眠禁能子程序
* 入口:设备号
*****************************************************************************/
void SysCtlPeripheralDeepSleepDisable(u32 ulPeripheral)
{
    ASSERT(SysCtlPeripheralValid(ulPeripheral));
	SYSCTL->DCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] &=
                ~SYSCTL_PERIPH_MASK(ulPeripheral);
}

/*****************************************************************************
* 设备时钟门控控制 
* 入口： bEnable=TRUE  进入睡眠模式后用睡眠时钟控制寄存器控制设备时钟使能
* 入口： bEnable=FALSE 进入睡眠模式后用运行时钟控制寄存器控制设备时钟使能 
*****************************************************************************/
void SysCtlPeripheralClockGating(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_ACG;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_ACG);
}

/*****************************************************************************
* 设置系统中断服务程序入口并使能系统中断 INT_SYSCTL
* 设置NVIC
*****************************************************************************/
void SysCtlIntRegister(void (*pfnHandler)(void))
{
    IntRegister(INT_SYSCTL, pfnHandler);
    IntEnable(INT_SYSCTL);
}

/*****************************************************************************
* 禁止系统中断 并设置系统中断服务程序入口为IntDefaultHandler
* 设置NVIC
*****************************************************************************/
void SysCtlIntUnregister(void)
{
    IntDisable(INT_SYSCTL);
    IntUnregister(INT_SYSCTL);
}

/*****************************************************************************
* 系统中断使能 设置系统中断屏蔽寄存器
* 入口 ulInts 中断使能 PLLLIM(PLL LOCK 中断使能位) BORIM(掉电复位中断使能位)
*****************************************************************************/
void SysCtlIntEnable(u32 ulInts)
{
    SYSCTL->IMC |= ulInts;
}

/*****************************************************************************
* 系统中断禁能 设置系统中断屏蔽寄存器
* 入口 ulInts 中断禁能 PLLLIM(PLL LOCK 中断使能位) BORIM(掉电复位中断使能位)
*****************************************************************************/
void SysCtlIntDisable(u32 ulInts)
{
    SYSCTL->IMC &= ~(ulInts);
}

/*****************************************************************************
* 清除中断状态寄存器的 PLLLMIS BORMIS
* 写1清除中断
*****************************************************************************/
void SysCtlIntClear(u32 ulInts)
{
    SYSCTL->MISC = ulInts;
}

/*****************************************************************************
* 读系统中断状态寄存器
* 入口 bMasked=1读屏蔽后的中断 bMasked=0 读原始中断
*****************************************************************************/
u32	SysCtlIntStatus(u8 bMasked)
{
    if(bMasked)
     return(SYSCTL->MISC);
    else
     return(SYSCTL->RIS);
}

/*****************************************************************************
* 设置 LDO输出电压 LOW DROP-OUT
* 入口ulVoltage LDO 电压值 可在LM3S21xx_sysctl.h中选取
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
* 读取LDO输出控制寄存器
*****************************************************************************/
u32	SysCtlLDOGet(void)
{
    return(SYSCTL->LDOPCTL);
}
/*****************************************************************************
* 读LDO复位控制寄存器 对于LM3S2XXX 无效
*****************************************************************************/
void SysCtlLDOConfigSet(u32 ulConfig)
{
    ASSERT((ulConfig == SYSCTL_LDOCFG_ARST) ||
           (ulConfig == SYSCTL_LDOCFG_NORST));
    SYSCTL->LDOARST = ulConfig;
}
/*****************************************************************************
* 设置应用中断与复位控制寄存器
*****************************************************************************/
void SysCtlReset(void)
{
    HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    while(1)
     {
     }
}
/*****************************************************************************
* 进入休眠模式
*****************************************************************************/
void SysCtlSleep(void)
{
    CPUwfi();
}

/*****************************************************************************
* 进入深度休眠模式
*****************************************************************************/
void SysCtlDeepSleep(void)
{
    HWREG(NVIC_SYS_CTRL) |= NVIC_SYS_CTRL_SLEEPDEEP;
    CPUwfi();
    HWREG(NVIC_SYS_CTRL) &= ~(NVIC_SYS_CTRL_SLEEPDEEP);
}

/*****************************************************************************
* 读取复位源
*****************************************************************************/
u32	SysCtlResetCauseGet(void)
{
    return(SYSCTL->RESC);
}

/*****************************************************************************
* 清除复位源状态
*****************************************************************************/
void SysCtlResetCauseClear(u32 ulCauses)
{
    SYSCTL->RESC &= ~(ulCauses);
}

/*****************************************************************************
* 设置掉电复位控制
*****************************************************************************/
void SysCtlBrownOutConfigSet(u32 ulConfig, u32 ulDelay)
{
    ASSERT(!(ulConfig & ~(SYSCTL_BOR_RESET | SYSCTL_BOR_RESAMPLE)));
    ASSERT(ulDelay < 8192);
    SYSCTL->PBORCTL = (ulDelay << SYSCTL_PBORCTL_BORTIM_S) | ulConfig;
}

/*****************************************************************************
* 系统控制延时
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
* 设置系统时钟 输出参数参考LM3S2139.h中
*****************************************************************************/
void SysCtlClockSet(u32 ulConfig)
{
    u32 ulDelay, ulRCC, ulRCC2;
    if(CLASS_IS_SANDSTORM && (ulConfig & SYSCTL_RCC2_USERCC2))
     return;                          //SandStorm版器件RCC2域不能替换RCC中相同域  
    SysRCC_CFG=ulConfig;              //备份配置值 定时检测
    ulRCC = SYSCTL->RCC;              //读取RCC  运行模式时钟配置寄存器
    ulRCC2 = SYSCTL->RCC2;            //读取RCC2 运行模式时钟配置寄存器2
    ulRCC |= SYSCTL_RCC_BYPASS;       //先旁路PLL 用OSC产生系统时钟
    ulRCC &= ~(SYSCTL_RCC_USESYSDIV); //先不用系统时钟分频器
    ulRCC2 |= SYSCTL_RCC2_BYPASS2;    //先旁路PLL 用OSC产生系统时钟

    SYSCTL->RCC = ulRCC;              //设置RCC  运行模式时钟配置寄存器
    SysCtlDelay(16);                            //等待稳定
    ulRCC |= SYSCTL_RCC_PWRDN;        //先旁路PLL 再掉电
    SYSCTL->RCC = ulRCC;              //设置RCC  运行模式时钟配置寄存器
    SYSCTL->RCC2 = ulRCC2;            //设置RCC2 运行模式时钟配置寄存器
    if(((ulRCC & SYSCTL_RCC_IOSCDIS) && !(ulConfig & SYSCTL_RCC_IOSCDIS))\
     ||((ulRCC & SYSCTL_RCC_MOSCDIS) && !(ulConfig & SYSCTL_RCC_MOSCDIS))) //内部振荡器禁能是否改变 主振荡器禁能配置是否改变
     {                               //内部振荡器禁能改变 或主振荡器禁能配置改变 执行
      ulRCC &= ((~(SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS))
               |(ulConfig & (SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS))); //设置振荡器使能
      SYSCTL->RCC = ulRCC;    //设置RCC的IOSCDIS 和	MOSCDIS位
      if(((ulRCC2 & SYSCTL_RCC2_USERCC2) &&							//通过RCC2设置
          (((ulRCC2 & SYSCTL_RCC2_OSCSRC2_M) == SYSCTL_RCC2_OSCSRC2_30) ||  //30KHz
           ((ulRCC2 & SYSCTL_RCC2_OSCSRC2_M) == SYSCTL_RCC2_OSCSRC2_32)))	  //32KHz
         ||(!(ulRCC2 & SYSCTL_RCC2_USERCC2) &&								  //不通过RCC2设置30KHz
           ((ulRCC & SYSCTL_RCC_OSCSRC_M) == SYSCTL_RCC_OSCSRC_30)))
       SysCtlDelay(4096);                      //系统延时
      else                                     
       SysCtlDelay(524288);                    //系统延时
     }                                           
    ulRCC &= ~(SYSCTL_RCC_XTAL_M |               //清除晶振频率设置位
               SYSCTL_RCC_OSCSRC_M |             //清除时钟源设置位
               SYSCTL_RCC_PWRDN|                 //清除PLL 断电位
               SYSCTL_RCC_OEN);                  //清除PLL 输出使能位
    ulRCC |= (ulConfig & (SYSCTL_RCC_XTAL_M |    //设置晶振频率设置位
                          SYSCTL_RCC_OSCSRC_M |  //设置时钟源设置位  
                          SYSCTL_RCC_PWRDN |     //设置PLL 断电位    
                          SYSCTL_RCC_OEN));      //设置PLL 输出使能位
    ulRCC2 &= ~(SYSCTL_RCC2_USERCC2|             //清除RCC2使能位
                SYSCTL_RCC2_OSCSRC2_M|           //清除RCC2时钟源位
                SYSCTL_RCC2_PWRDN2);             //清除RCC2PLL 断电位
    ulRCC2 |= (ulConfig & (SYSCTL_RCC2_USERCC2|  //使能RCC2寄存器
                           SYSCTL_RCC_OSCSRC_M|  //设置RCC2时钟源位
                           SYSCTL_RCC2_PWRDN2)); //设置RCC2 PLL断电位
    ulRCC2 |= ((ulConfig & 0x00000008) << 3);    //设置RCC2.3
    SYSCTL->MISC = SYSCTL_INT_PLL_LOCK;          //清除PLL锁定位
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
    SysCtlDelay(16);                            //等待稳定

    ulRCC &= ~(SYSCTL_RCC_SYSDIV_M|             //清除分频位
               SYSCTL_RCC_USESYSDIV|            //清除分频使能位
               SYSCTL_RCC_IOSCDIS|              //清除内部振荡器禁能位
               SYSCTL_RCC_MOSCDIS);             //清除主振荡器禁能位
    ulRCC |= (ulConfig & (SYSCTL_RCC_SYSDIV_M|  //设置分频位           
                          SYSCTL_RCC_USESYSDIV| //设置分频使能位       
                          SYSCTL_RCC_IOSCDIS|   //设置内部振荡器禁能位 
                          SYSCTL_RCC_MOSCDIS)); //设置除主振荡器禁能位   
    ulRCC2 &= ~(SYSCTL_RCC2_SYSDIV2_M);         //清除RCC2分频位
    ulRCC2 |= (ulConfig & SYSCTL_RCC2_SYSDIV2_M);//设置RCC2分频位

    if(!(ulConfig & SYSCTL_RCC_BYPASS))         //判断PLL是否旁路
     {                                          //使用PLL
      for(ulDelay = 32768; ulDelay > 0; ulDelay--)
       {
        if(SYSCTL->RIS & SYSCTL_INT_PLL_LOCK)
         break;                                 //锁相环锁定 跳出循环
       }
      ulRCC &= ~(SYSCTL_RCC_BYPASS);
      ulRCC2 &= ~(SYSCTL_RCC2_BYPASS2);
     }
    SYSCTL->RCC = ulRCC;
    SYSCTL->RCC2 = ulRCC2;
    SysCtlDelay(16);
}
/*****************************************************************************
* 获取系统时钟频率 
* 返回系统时钟频率
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
* 设置PWM系统时钟频率 
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
* 获取PWM系统时钟频率 
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
* 设置ADC采样速率 
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
* 获取ADC采样频率设置 
*****************************************************************************/
u32	SysCtlADCSpeedGet(void)
{
    ASSERT(SYSCTL->DC[1] & SYSCTL_DC1_ADC);
    return(SYSCTL->RCGC[0] & SYSCTL_RCGC0_ADCSPD_M);
}
/*****************************************************************************
* 内部振荡器校准设置
* bEnable=TRUE  校准
* bEnable=FALSE 不校准
*****************************************************************************/
void SysCtlIOSCVerificationSet(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_IOSCVER;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_IOSCVER);
}
/*****************************************************************************
* 主振荡器校准设置
* bEnable=TRUE  校准
* bEnable=FALSE 不校准
*****************************************************************************/
void SysCtlMOSCVerificationSet(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_MOSCVER;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_MOSCVER);
}
/*****************************************************************************
* PLL校准设置
* bEnable=TRUE  校准
* bEnable=FALSE 不校准
*****************************************************************************/
void SysCtlPLLVerificationSet(u8 bEnable)
{
    if(bEnable)
     SYSCTL->RCC |= SYSCTL_RCC_PLLVER;
    else
     SYSCTL->RCC &= ~(SYSCTL_RCC_PLLVER);
}
/*****************************************************************************
* 清除时钟校准
*****************************************************************************/
void SysCtlClkVerificationClear(void)
{
    SYSCTL->CLKVCLR = SYSCTL_CLKVCLR_VERCLR;
    SYSCTL->CLKVCLR = 0;
}
/*****************************************************************************
* GPIO AHB访问使能
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
* GPIO AHB访问禁能
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
