/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_gpio.c
;* Author             : 张力阵
;* I/O口驱动程序库
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "ENG_ERR.h"
#include "define.h"
#include "vari.h"
/*****************************************************************************
* 初始化GPIO口,针对SH5.948.1200 
*****************************************************************************/
void Init_Gpio(void)
{
//设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能端口A时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能端口B时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能端口C时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能端口D时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能端口E时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能端口F时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//使能端口H时钟
//初始化GPIOA口
    GPIOPinTypeUART(GPIOA,                  //端口
                    U0RX|U0TX);             //管脚 初始化UART0接口管脚	 PA.0 PA.1
/* 
    GPIOPinTypeGPIOOutput(GPIOA,            //端口
                          DISP_RST);        //管脚 PA.4 HD7219复位输出
    GPIOPinTypeSSI(GPIOA,                   //端口
                   SSICLK|SSIFSS|SSITX);    //管脚 SSICLK SSIFSS SSITX设置为SSI管脚  PA.2 PA.3 PA.5
    GPIOPinTypeTimer(GPIOA,                 //端口
                     JZ_IN|FH_IN);          //管脚 PA.6 PA.7
*/ 
//初始化GPIOB口

    GPIOPinTypeTimer(GPIOB,                 //端口
                     SZ_MC);                //管脚 PB.1
    GPIOPinTypeI2C(GPIOB,                   //端口
                   I2C_SCL|                 //管脚 PB.2 
																			I2C_SDA);                //管脚 PB.3         
    GPIOPinTypeGPIOOutput(GPIOB,            //端口
                          I2C_WP|
                          CS5460A_MODE|     //管脚 PB.6,
                          CS5460A_CS);      //管脚 PB.5
    GPIOPinTypeGPIOInput(GPIOB,
                         CS5460A_SDO);      //管脚 PB.4
                          
//初始化GPIOC口                             
    GPIOPinTypeTimer(GPIOC,                 //端口
                     PWM_DAC);              //管脚 PC.4
    GPIOPinTypeGPIOInput(GPIOC,             //端口
                         KEY_IN);           //管脚 PC.5 按键输入
                        
//初始化GPIOD口                             
    GPIOPinTypeCAN(GPIOD,                   //端口
                   CANRX|                   //管脚
                   CANTX);                  //初始化CAN接口管脚
/*                   
    GPIOPinTypeUART(GPIOD,                  //端口
                    U1RX|                   //管脚
                    U1TX);                  //初始化UART1接口管脚
*/                    
    GPIOPinTypeGPIOOutput(GPIOD,            //端口
                          CS5460A_SCLK|     //管脚 PD.4
                          CS5460A_RESET|    //管脚 PD.5
																										CS5460A_SDI);     //管脚 PD.7
    GPIOPinTypeGPIOInput(GPIOD,             //端口
                         CS5460A_INT);      //管脚 PD.6    
//初始化GPIOE口  
    GPIOPinTypeGPIOOutput(GPIOE,            //端口
                          CHNL1_Sel|	       //管脚 PE.3
                          P3P4SelOut3|      //     PE.0
                          P3P4SelOut2|      //     PE.1                
                          P3P4SelOut1);     //     PE.2 
                           
//初始化GPIOF口                             
    GPIOPinTypeGPIOOutput(GPIOF,            //端口
                          WDI);             //PF.7 外置看门狗复位           
                                            
    GPIOPinTypeGPIOInput(GPIOF,             //端口
                         GOG_KZ|
                         MC_PN_KZ|
                         MC_WV_KZ|
                         TX_MC|
                         XL_MC|             //     PF.3 需量周期输入
                         TQ_MC|             //备用 PF.4 时段投切脉冲
                         HZ_MC);            //备用 PF.5 合闸脉冲
//初始化GPIOH口                             
    GPIOPinTypeGPIOOutput(GPIOH,            //端口
                          Phase_Sel_K1|     //管脚 PH.0 功耗测试相别选择1
                          Phase_Sel_K2|     //管脚 PH.1 功耗测试相别选择2
                          CHNL2_Sel|        //管脚 PH.2 功耗测试通道2选择
                          CHNL3_Sel);       //管脚 PH.3 功耗测试通道3选择
             
//初始化端口中断 GPIOC
    GPIOPadConfigSet(GPIOC,                 //端口 设置管脚类型    
                     KEY_IN,                //管脚   |XL_MC              
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(GPIOC,                   //端口 设置管脚中断发送
                   KEY_IN,                  //管脚 |XL_MC                
                   GPIO_FALLING_EDGE);      //中断方式             

//初始化端口中断 GPIOD
    GPIOPadConfigSet(GPIOD,                 //端口 设置管脚类型    
                     CS5460A_INT,           //管脚 CS5460A中断。
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(GPIOD,                   //端口 设置管脚中断方式
                   CS5460A_INT,             //管脚 5460A的INT
                   GPIO_FALLING_EDGE);      //中断方
                                    
//端口初步设置
    CS5460A_CS_H;                           //禁止片选Cs5460A
    CS5460A_SCLK_H;                         //
    CS5460A_SDI_H;
    CS5460A_RESET_H;
    CS5460A_MODE_L;
    CNCL_CHNL_Sel1;                          //不选通测试通道
    CNCL_CHNL_Sel2;
    Sel_Phase_NO;                           //不选通测试相别
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;
    WDI_HIGH;
}

/*****************************************************************************
* 检查I/O口基址是否正确 
* 调试时使用
*****************************************************************************/
#ifdef DEBUG
static u8
GPIOBaseValid(GPIO_Typedef *GPIOx)
{
    u32 ulPort= (u32)GPIOx;
    return((ulPort == GPIO_PORTA_BASE) || (ulPort == GPIO_PORTA_AHB_BASE) ||
           (ulPort == GPIO_PORTB_BASE) || (ulPort == GPIO_PORTB_AHB_BASE) ||
           (ulPort == GPIO_PORTC_BASE) || (ulPort == GPIO_PORTC_AHB_BASE) ||
           (ulPort == GPIO_PORTD_BASE) || (ulPort == GPIO_PORTD_AHB_BASE) ||
           (ulPort == GPIO_PORTE_BASE) || (ulPort == GPIO_PORTE_AHB_BASE) ||
           (ulPort == GPIO_PORTF_BASE) || (ulPort == GPIO_PORTF_AHB_BASE) ||
           (ulPort == GPIO_PORTG_BASE) || (ulPort == GPIO_PORTG_AHB_BASE) ||
           (ulPort == GPIO_PORTH_BASE) || (ulPort == GPIO_PORTH_AHB_BASE));
}
#endif

/*****************************************************************************
* 获取I/O设备中断号
*****************************************************************************/
s32 GPIOGetIntNumber(GPIO_Typedef *GPIOx)
{
    u32 ulInt;
    switch((u32)GPIOx)
     {
         case GPIO_PORTA_BASE:
         case GPIO_PORTA_AHB_BASE:
          {   //GPIOA
           ulInt = INT_GPIOA;
           break;
          }
         case GPIO_PORTB_BASE:
         case GPIO_PORTB_AHB_BASE:
          {   //GPIOB
           ulInt = INT_GPIOB;
           break;
          }
         case GPIO_PORTC_BASE:
         case GPIO_PORTC_AHB_BASE:
          {   //GPIOC
           ulInt = INT_GPIOC;
           break;
          }
         case GPIO_PORTD_BASE:
         case GPIO_PORTD_AHB_BASE:
          {   //GPIOD
           ulInt = INT_GPIOD;
           break;
          }
         case GPIO_PORTE_BASE:
         case GPIO_PORTE_AHB_BASE:
          {   //GPIOE
           ulInt = INT_GPIOE;
           break;
          }
         case GPIO_PORTF_BASE:
         case GPIO_PORTF_AHB_BASE:
          {   //GPIOF
           ulInt = INT_GPIOF;
           break;
          }
         case GPIO_PORTG_BASE:
         case GPIO_PORTG_AHB_BASE:
          {   //GPIOG
           ulInt = INT_GPIOG;
           break;
          }
         case GPIO_PORTH_BASE:
         case GPIO_PORTH_AHB_BASE:
          {   //GPIOH
           ulInt = INT_GPIOH;
           break;
          }
         default:
          return(-1);
     }
    return(ulInt);
}
/*****************************************************************************
* 设置I/O口方向
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7 的组合
* ulPinIO 管脚输入输出方向 GPIO_DIR_MODE_IN GPIO_DIR_MODE_OUT GPIO_DIR_MODE_HW
*****************************************************************************/
void GPIODirModeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulPinIO)
{
    ASSERT((ulPinIO == GPIO_DIR_MODE_IN) || (ulPinIO == GPIO_DIR_MODE_OUT) ||
           (ulPinIO == GPIO_DIR_MODE_HW));

    GPIOx->DIR = ((ulPinIO & 1) ? (GPIOx->DIR | ucPins) :          //输出
                                  (GPIOx->DIR & ~(ucPins)));       //输入
    GPIOx->AFSEL = ((ulPinIO & 2) ? (GPIOx->AFSEL | ucPins) :      //第二功能
                                    (GPIOx->AFSEL &( ~(ucPins)))); //I/O口
}

/*****************************************************************************
* 获取I/O口方向设置
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7
* 返回 管脚输入输出方向 GPIO_DIR_MODE_IN GPIO_DIR_MODE_OUT GPIO_DIR_MODE_HW
*****************************************************************************/
u32 GPIODirModeGet(GPIO_Typedef *GPIOx, u8 ucPins)
{
    u32 ulDir, ulAFSEL;

    ASSERT(GPIOBaseValid(GPIOx));

    ulDir = GPIOx->DIR;
    ulAFSEL = GPIOx->AFSEL;
    return(((ulDir & ucPins) ? 1 : 0) | ((ulAFSEL & ucPins) ? 2 : 0));
}

/*****************************************************************************
* 设置I/O口中断方式
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7 的组合
* ulIntType 管脚中断方式 GPIO_FALLING_EDGE GPIO_RISING_EDGE GPIO_BOTH_EDGES GPIO_LOW_LEVEL GPIO_HIGH_LEVEL
*****************************************************************************/
void GPIOIntTypeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulIntType)
{
    ASSERT(GPIOBaseValid(GPIOx));
    ASSERT((ulIntType == GPIO_FALLING_EDGE) ||
           (ulIntType == GPIO_RISING_EDGE) || (ulIntType == GPIO_BOTH_EDGES) ||
           (ulIntType == GPIO_LOW_LEVEL) || (ulIntType == GPIO_HIGH_LEVEL));

    GPIOx->IBE = ((ulIntType & 1) ?
                                  (GPIOx->IBE | ucPins) :
                                  (GPIOx->IBE & ~(ucPins)));
    GPIOx->IS = ((ulIntType & 2) ?
                                 (GPIOx->IS | ucPins) :
                                 (GPIOx->IS & ~(ucPins)));
    GPIOx->IEV = ((ulIntType & 4) ?
                                  (GPIOx->IEV | ucPins) :
                                  (GPIOx->IEV & ~(ucPins)));
}

/*****************************************************************************
* 获取I/O口中断方式设置
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7
* 返回 管脚中断方式 GPIO_FALLING_EDGE GPIO_RISING_EDGE GPIO_BOTH_EDGES GPIO_LOW_LEVEL GPIO_HIGH_LEVEL
*****************************************************************************/
u32 GPIOIntTypeGet(GPIO_Typedef *GPIOx, u8 ucPins)
{
    u32 ulIBE, ulIS, ulIEV;

    ASSERT(GPIOBaseValid(GPIOx));

    ulIBE = GPIOx->IBE;
    ulIS = GPIOx->IS;
    ulIEV = GPIOx->IEV;
    return(((ulIBE & ucPins) ? 1 : 0) | ((ulIS & ucPins) ? 2 : 0) |
           ((ulIEV & ucPins) ? 4 : 0));
}

/*****************************************************************************
* 设置I/O口驱动能力 2mA 4mA 8mA 和管脚类型
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7 的组合
* ulStrength 驱动强度
* ulPinType 管脚输出输入类型 GPIO_PIN_TYPE_STD ...
*****************************************************************************/
void GPIOPadConfigSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulStrength, u32 ulPinType)
{
    ASSERT(GPIOBaseValid(GPIOx));
    ASSERT((ulStrength == GPIO_STRENGTH_2MA) ||
           (ulStrength == GPIO_STRENGTH_4MA) ||
           (ulStrength == GPIO_STRENGTH_8MA) ||
           (ulStrength == GPIO_STRENGTH_8MA_SC));
    ASSERT((ulPinType == GPIO_PIN_TYPE_STD) ||
           (ulPinType == GPIO_PIN_TYPE_STD_WPU) ||
           (ulPinType == GPIO_PIN_TYPE_STD_WPD) ||
           (ulPinType == GPIO_PIN_TYPE_OD) ||
           (ulPinType == GPIO_PIN_TYPE_OD_WPU) ||
           (ulPinType == GPIO_PIN_TYPE_OD_WPD) ||
           (ulPinType == GPIO_PIN_TYPE_ANALOG))

    GPIOx->DR2R = ((ulStrength & 1) ?
                                   (GPIOx->DR2R | ucPins) :
                                   (GPIOx->DR2R & ~(ucPins)));
    GPIOx->DR4R = ((ulStrength & 2) ?
                                   (GPIOx->DR4R | ucPins) :
                                   (GPIOx->DR4R & ~(ucPins)));
    GPIOx->DR8R = ((ulStrength & 4) ?
                                   (GPIOx->DR8R | ucPins) :
                                   (GPIOx->DR8R & ~(ucPins)));
    GPIOx->SLR = ((ulStrength & 8) ?
                                  (GPIOx->SLR | ucPins) :
                                  (GPIOx->SLR & ~(ucPins)));

    GPIOx->ODR = ((ulPinType & 1) ?
                                  (GPIOx->ODR | ucPins) :
                                  (GPIOx->ODR & ~(ucPins)));
    GPIOx->PUR = ((ulPinType & 2) ?
                                  (GPIOx->PUR | ucPins) :
                                  (GPIOx->PUR & ~(ucPins)));
    GPIOx->PDR = ((ulPinType & 4) ?
                                  (GPIOx->PDR | ucPins) :
                                  (GPIOx->PDR & ~(ucPins)));
    GPIOx->DEN = ((ulPinType & 8) ?
                                  (GPIOx->DEN | ucPins) :
                                  (GPIOx->DEN & ~(ucPins)));

    //AMSEL(analog mode select)寄存器 对以后的器件有效 SandStorm-Class Fury-Class 写无害(无效)
    GPIOx->AMSEL =
        ((ulPinType == GPIO_PIN_TYPE_ANALOG) ?
         (GPIOx->AMSEL | ucPins) :
         (GPIOx->AMSEL & ~(ucPins)));
}

/*****************************************************************************
* 获取I/O口驱动能力 2mA 4mA 8mA 和管脚类型
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7
* pulStrength 驱动强度返回指针
* pulPinType 管脚类型返回指针 GPIO_PIN_TYPE_STD ...
*****************************************************************************/
void GPIOPadConfigGet(GPIO_Typedef *GPIOx, u8 ucPins,u32 *pulStrength, u32 *pulPinType)
{
    u32 ulTemp1, ulTemp2, ulTemp3, ulTemp4;

    ASSERT(GPIOBaseValid(GPIOx));

    ulTemp1 = GPIOx->DR2R;
    ulTemp2 = GPIOx->DR4R;
    ulTemp3 = GPIOx->DR8R;
    ulTemp4 = GPIOx->SLR;
    *pulStrength = (((ulTemp1 & ucPins) ? 1 : 0) | ((ulTemp2 & ucPins) ? 2 : 0) |
                    ((ulTemp3 & ucPins) ? 4 : 0) | ((ulTemp4 & ucPins) ? 8 : 0));

    ulTemp1 = GPIOx->ODR;
    ulTemp2 = GPIOx->PUR;
    ulTemp3 = GPIOx->PDR;
    ulTemp4 = GPIOx->DEN;
    *pulPinType = (((ulTemp1 & ucPins) ? 1 : 0) | ((ulTemp2 & ucPins) ? 2 : 0) |
                   ((ulTemp3 & ucPins) ? 4 : 0) | ((ulTemp4 & ucPins) ? 8 : 0));
}

/*****************************************************************************
* I/O口管脚中断使能
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7 的组合
*****************************************************************************/
void GPIOPinIntEnable(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIOx->IM |= ucPins;
}

/*****************************************************************************
* I/O口管脚中断禁能
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7 的组合
*****************************************************************************/
void GPIOPinIntDisable(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIOx->IM &= ~(ucPins);
}

/*****************************************************************************
* 获取I/O口中断状态
* GPIOx I/O口结构体 GPIOA~GPIOH
* bMasked=1 获取屏蔽后的中断状态
* bMasked=0 获取原始的中断状态
*****************************************************************************/
u32 GPIOPinIntStatus(GPIO_Typedef *GPIOx, u8 bMasked)
{
    ASSERT(GPIOBaseValid(GPIOx));
    if(bMasked)
     return(GPIOx->MIS);
    else
     return(GPIOx->RIS);
}

/*****************************************************************************
* 清除I/O口中断状态
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 端口管脚 GPIO_PIN_0~GPIO_PIN_7 的组合
*****************************************************************************/
void GPIOPinIntClear(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIOx->ICR = ucPins;
}
/*****************************************************************************
* 设置I/O口中断服务程序入口
* GPIOx I/O口结构体 GPIOA~GPIOH
* pfnIntHandler 中断入口
*****************************************************************************/
void GPIOPortIntRegister(GPIO_Typedef *GPIOx, void (*pfnIntHandler)(void))
{
    u32 ulPort;
    ASSERT(GPIOBaseValid(GPIOx));
    ulPort = GPIOGetIntNumber(GPIOx);
    IntRegister(ulPort, pfnIntHandler);
    IntEnable(ulPort);
}

/*****************************************************************************
* 撤销I/O口中断服务程序入口
* GPIOx I/O口结构体 GPIOA~GPIOH
*****************************************************************************/
void GPIOPortIntUnregister(GPIO_Typedef *GPIOx)
{
    u32 ulPort;
    ASSERT(GPIOBaseValid(GPIOx));
    ulPort = GPIOGetIntNumber(GPIOx);
    IntDisable(ulPort);
    IntUnregister(ulPort);
}

/*****************************************************************************
* 获取I/O管脚值
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
u32 GPIOPinRead(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    return(GPIOx->DATA[ucPins]);
}

/*****************************************************************************
* 写I/O管脚值
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
* ucVal 管脚值
*****************************************************************************/
void GPIOPinWrite(GPIO_Typedef *GPIOx, u8 ucPins, u8 ucVal)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIOx->DATA[ucPins] = ucVal;
}

/*****************************************************************************
* 设置管脚为ADC输入
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeADC(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_ANALOG);

}

/*****************************************************************************
* 设置管脚为CAN 接口
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeCAN(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* 设置管脚为模拟比较器管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeComparator(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_ANALOG);
}

/*****************************************************************************
* 设置管脚为GPIO I/O输入
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeGPIOInput(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
}

/*****************************************************************************
* 设置管脚为GPIO I/O输出(推挽)
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeGPIOOutput(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_OUT);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* 设置管脚为GPIO I/O输出 开漏
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeGPIOOutputOD(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_OUT);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD);
}

/*****************************************************************************
* 设置管脚为I2C管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeI2C(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD_WPU);
}

/*****************************************************************************
* 设置管脚为PWM管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypePWM(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* 设置管脚为正交编码管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeQEI(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
}
/*****************************************************************************
* 设置管脚为SSI管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeSSI(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* 设置管脚为定时器管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeTimer(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);	 //GPIO_PIN_TYPE_STD
}
/*****************************************************************************
* 设置管脚为UART管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeUART(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);		 //GPIO_PIN_TYPE_STD
}

/*****************************************************************************
* 设置管脚为USB管脚
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeUSBDigital(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

