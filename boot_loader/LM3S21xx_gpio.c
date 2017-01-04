/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_gpio.c
;* Author             : ������
;* I/O�����������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "disp.h"
/*****************************************************************************
* ��ʼ��GPIO�� 
*****************************************************************************/
void Init_Gpio(void)
{
//�豸ʱ��ʹ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//ʹ�ܶ˿�Aʱ��
//��ʼ��GPIOA��
    GPIOPinTypeGPIOOutput(GPIOA,            //�˿�
                          DISP_RST);        //�ܽ� PA.4 HD7219��λ���
    GPIOPinTypeSSI(GPIOA,                   //�˿�
                   SSICLK|SSIFSS|SSITX);    //�ܽ� SSICLK SSIFSS SSITX����ΪSSI�ܽ�  PA.2 PA.3 PA.5
//�豸ʱ��ʹ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//ʹ�ܶ˿�Fʱ��
//��ʼ��GPIOA��
    GPIOPinTypeGPIOOutput(GPIOF,            //�˿�
                          WDI_MC);          //�ܽ� PA.4 HD7219��λ���
//��ʼ��GPIOB��                             
//��ʼ��GPIOC��                             
//��ʼ��GPIOD��                             
//��ʼ��GPIOE��  
//��ʼ��GPIOF��                             
//��ʼ��GPIOH��                             
//��ʼ���˿��ж� GPIOB
//��ʼ���˿��ж� GPIOC
//��ʼ���˿��ж� GPIOF
                                    
//�˿ڳ�������
}
/*****************************************************************************
* ����I/O�ڷ���
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7 �����
* ulPinIO �ܽ������������ GPIO_DIR_MODE_IN GPIO_DIR_MODE_OUT GPIO_DIR_MODE_HW
*****************************************************************************/
void GPIODirModeSet(GPIO_Typedef *GPIOx, u8 ucPins,u32 ulPinIO)
{
    ASSERT((ulPinIO == GPIO_DIR_MODE_IN) || (ulPinIO == GPIO_DIR_MODE_OUT) ||
           (ulPinIO == GPIO_DIR_MODE_HW));

    GPIOx->DIR = ((ulPinIO & 1) ? (GPIOx->DIR | ucPins) :          //���
                                  (GPIOx->DIR & ~(ucPins)));       //����
    GPIOx->AFSEL = ((ulPinIO & 2) ? (GPIOx->AFSEL | ucPins) :      //�ڶ�����
                                    (GPIOx->AFSEL &( ~(ucPins)))); //I/O��
}

/*****************************************************************************
* ��ȡI/O�ڷ�������
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7
* ���� �ܽ������������ GPIO_DIR_MODE_IN GPIO_DIR_MODE_OUT GPIO_DIR_MODE_HW
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
* ����I/O���жϷ�ʽ
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7 �����
* ulIntType �ܽ��жϷ�ʽ GPIO_FALLING_EDGE GPIO_RISING_EDGE GPIO_BOTH_EDGES GPIO_LOW_LEVEL GPIO_HIGH_LEVEL
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
* ��ȡI/O���жϷ�ʽ����
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7
* ���� �ܽ��жϷ�ʽ GPIO_FALLING_EDGE GPIO_RISING_EDGE GPIO_BOTH_EDGES GPIO_LOW_LEVEL GPIO_HIGH_LEVEL
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
* ����I/O���������� 2mA 4mA 8mA �͹ܽ�����
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7 �����
* ulStrength ����ǿ��
* ulPinType �ܽ������������ GPIO_PIN_TYPE_STD ...
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

    //AMSEL(analog mode select)�Ĵ��� ���Ժ��������Ч SandStorm-Class Fury-Class д�޺�(��Ч)
    GPIOx->AMSEL =
        ((ulPinType == GPIO_PIN_TYPE_ANALOG) ?
         (GPIOx->AMSEL | ucPins) :
         (GPIOx->AMSEL & ~(ucPins)));
}

/*****************************************************************************
* ��ȡI/O���������� 2mA 4mA 8mA �͹ܽ�����
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7
* pulStrength ����ǿ�ȷ���ָ��
* pulPinType �ܽ����ͷ���ָ�� GPIO_PIN_TYPE_STD ...
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
* I/O�ڹܽ��ж�ʹ��
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7 �����
*****************************************************************************/
void GPIOPinIntEnable(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIOx->IM |= ucPins;
}

/*****************************************************************************
* I/O�ڹܽ��жϽ���
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7 �����
*****************************************************************************/
void GPIOPinIntDisable(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIOx->IM &= ~(ucPins);
}

/*****************************************************************************
* ��ȡI/O���ж�״̬
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* bMasked=1 ��ȡ���κ���ж�״̬
* bMasked=0 ��ȡԭʼ���ж�״̬
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
* ���I/O���ж�״̬
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �˿ڹܽ� GPIO_PIN_0~GPIO_PIN_7 �����
*****************************************************************************/
void GPIOPinIntClear(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIOx->ICR = ucPins;
}

/*****************************************************************************
* ��ȡI/O�ܽ�ֵ
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
u32 GPIOPinRead(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    return(GPIOx->DATA[ucPins]);
}

/*****************************************************************************
* дI/O�ܽ�ֵ
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
* ucVal �ܽ�ֵ
*****************************************************************************/
void GPIOPinWrite(GPIO_Typedef *GPIOx, u8 ucPins, u8 ucVal)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIOx->DATA[ucPins] = ucVal;
}


/*****************************************************************************
* ���ùܽ�ΪGPIO I/O����
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeGPIOInput(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* ���ùܽ�ΪGPIO I/O���(����)
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeGPIOOutput(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_OUT);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* ���ùܽ�ΪGPIO I/O��� ��©
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeGPIOOutputOD(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_OUT);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
}

/*****************************************************************************
* ���ùܽ�ΪI2C�ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeI2C(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD_WPU);
}

/*****************************************************************************
* ���ùܽ�ΪPWM�ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypePWM(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* ���ùܽ�Ϊ��������ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeQEI(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}
/*****************************************************************************
* ���ùܽ�ΪSSI�ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeSSI(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* ���ùܽ�Ϊ��ʱ���ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeTimer(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}
/*****************************************************************************
* ���ùܽ�ΪUART�ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeUART(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));
    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* ���ùܽ�ΪUSB�ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeUSBDigital(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

