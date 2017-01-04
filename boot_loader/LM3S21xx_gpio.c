/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_gpio.c
;* Author             : 张力阵
;* I/O口驱动程序库
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "disp.h"
/*****************************************************************************
* 初始化GPIO口 
*****************************************************************************/
void Init_Gpio(void)
{
//设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能端口A时钟
//初始化GPIOA口
    GPIOPinTypeGPIOOutput(GPIOA,            //端口
                          DISP_RST);        //管脚 PA.4 HD7219复位输出
    GPIOPinTypeSSI(GPIOA,                   //端口
                   SSICLK|SSIFSS|SSITX);    //管脚 SSICLK SSIFSS SSITX设置为SSI管脚  PA.2 PA.3 PA.5
//设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能端口F时钟
//初始化GPIOA口
    GPIOPinTypeGPIOOutput(GPIOF,            //端口
                          WDI_MC);          //管脚 PA.4 HD7219复位输出
//初始化GPIOB口                             
//初始化GPIOC口                             
//初始化GPIOD口                             
//初始化GPIOE口  
//初始化GPIOF口                             
//初始化GPIOH口                             
//初始化端口中断 GPIOB
//初始化端口中断 GPIOC
//初始化端口中断 GPIOF
                                    
//端口初步设置
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
* 设置管脚为GPIO I/O输入
* GPIOx I/O口结构体 GPIOA~GPIOH
* ucPins 管脚GPIO_PIN_0~GPIO_PIN_7的组合
*****************************************************************************/
void GPIOPinTypeGPIOInput(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD_WPU);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

