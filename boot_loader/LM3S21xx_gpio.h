/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_gpio.h
;* Author             : 张力阵
;* I/O口驱动程序库声明
*******************************************************************************/

#ifndef __GPIO_H__
#define __GPIO_H__

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
// The following values define the bit field for the ucPins argument to several
// of the APIs.
//
//*****************************************************************************
#define GPIO_PIN_0              0x00000001  // GPIO pin 0
#define GPIO_PIN_1              0x00000002  // GPIO pin 1
#define GPIO_PIN_2              0x00000004  // GPIO pin 2
#define GPIO_PIN_3              0x00000008  // GPIO pin 3
#define GPIO_PIN_4              0x00000010  // GPIO pin 4
#define GPIO_PIN_5              0x00000020  // GPIO pin 5
#define GPIO_PIN_6              0x00000040  // GPIO pin 6
#define GPIO_PIN_7              0x00000080  // GPIO pin 7

//*****************************************************************************
//
// Values that can be passed to GPIODirModeSet as the ulPinIO parameter, and
// returned from GPIODirModeGet.
//
//*****************************************************************************
#define GPIO_DIR_MODE_IN        0x00000000  // Pin is a GPIO input
#define GPIO_DIR_MODE_OUT       0x00000001  // Pin is a GPIO output
#define GPIO_DIR_MODE_HW        0x00000002  // Pin is a peripheral function

//*****************************************************************************
//
// Values that can be passed to GPIOIntTypeSet as the ulIntType parameter, and
// returned from GPIOIntTypeGet.
//
//*****************************************************************************
#define GPIO_FALLING_EDGE       0x00000000  // Interrupt on falling edge
#define GPIO_RISING_EDGE        0x00000004  // Interrupt on rising edge
#define GPIO_BOTH_EDGES         0x00000001  // Interrupt on both edges
#define GPIO_LOW_LEVEL          0x00000002  // Interrupt on low level
#define GPIO_HIGH_LEVEL         0x00000007  // Interrupt on high level

//*****************************************************************************
//
// Values that can be passed to GPIOPadConfigSet as the ulStrength parameter,
// and returned by GPIOPadConfigGet in the *pulStrength parameter.
//
//*****************************************************************************
#define GPIO_STRENGTH_2MA       0x00000001  // 2mA drive strength
#define GPIO_STRENGTH_4MA       0x00000002  // 4mA drive strength
#define GPIO_STRENGTH_8MA       0x00000004  // 8mA drive strength
#define GPIO_STRENGTH_8MA_SC    0x0000000C  // 8mA drive with slew rate control

//*****************************************************************************
//
// Values that can be passed to GPIOPadConfigSet as the ulPadType parameter,
// and returned by GPIOPadConfigGet in the *pulPadType parameter.
//
//*****************************************************************************
#define GPIO_PIN_TYPE_STD       0x00000008  // Push-pull 推挽
#define GPIO_PIN_TYPE_STD_WPU   0x0000000A  // Push-pull with weak pull-up   弱上拉
#define GPIO_PIN_TYPE_STD_WPD   0x0000000C  // Push-pull with weak pull-down 弱下拉
#define GPIO_PIN_TYPE_OD        0x00000009  // Open-drain 开漏
#define GPIO_PIN_TYPE_OD_WPU    0x0000000B  // Open-drain with weak pull-up
#define GPIO_PIN_TYPE_OD_WPD    0x0000000D  // Open-drain with weak pull-down
#define GPIO_PIN_TYPE_ANALOG    0x00000000  // Analog comparator 模拟比较器

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void GPIODirModeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulPinIO);
extern u32 GPIODirModeGet(GPIO_Typedef *GPIOx, u8 ucPin);
extern void GPIOIntTypeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulIntType);
extern u32 GPIOIntTypeGet(GPIO_Typedef *GPIOx, u8 ucPin);
extern void GPIOPadConfigSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulStrength,u32 ulPadType);
extern void GPIOPadConfigGet(GPIO_Typedef *GPIOx, u8 ucPin,u32 *pulStrength,u32 *pulPadType);
extern void GPIOPinIntEnable(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinIntDisable(GPIO_Typedef *GPIOx, u8 ucPins);
extern u32 GPIOPinIntStatus(GPIO_Typedef *GPIOx, u8 bMasked);
extern void GPIOPinIntClear(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPortIntRegister(GPIO_Typedef *GPIOx,void (*pfnIntHandler)(void));
extern void GPIOPortIntUnregister(GPIO_Typedef *GPIOx);
extern u32 GPIOPinRead(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinWrite(GPIO_Typedef *GPIOx, u8 ucPins,u8 ucVal);
extern void GPIOPinTypeADC(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeCAN(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeComparator(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeGPIOInput(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeGPIOOutput(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeGPIOOutputOD(GPIO_Typedef *GPIOx,u8 ucPins);
extern void GPIOPinTypeI2C(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypePWM(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeQEI(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeSSI(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeTimer(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeUART(GPIO_Typedef *GPIOx, u8 ucPins);
extern void GPIOPinTypeUSBDigital(GPIO_Typedef *GPIOx, u8 ucPins);
extern void Init_Gpio(void);          //初始化GPIO口

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __GPIO_H__
