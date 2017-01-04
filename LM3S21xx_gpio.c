/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_gpio.c
;* Author             : ������
;* I/O�����������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "ENG_ERR.h"
#include "define.h"
#include "vari.h"
/*****************************************************************************
* ��ʼ��GPIO��,���SH5.948.1200 
*****************************************************************************/
void Init_Gpio(void)
{
//�豸ʱ��ʹ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//ʹ�ܶ˿�Aʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ�ܶ˿�Bʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//ʹ�ܶ˿�Cʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ�ܶ˿�Dʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ�ܶ˿�Eʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//ʹ�ܶ˿�Fʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//ʹ�ܶ˿�Hʱ��
//��ʼ��GPIOA��
    GPIOPinTypeUART(GPIOA,                  //�˿�
                    U0RX|U0TX);             //�ܽ� ��ʼ��UART0�ӿڹܽ�	 PA.0 PA.1
/* 
    GPIOPinTypeGPIOOutput(GPIOA,            //�˿�
                          DISP_RST);        //�ܽ� PA.4 HD7219��λ���
    GPIOPinTypeSSI(GPIOA,                   //�˿�
                   SSICLK|SSIFSS|SSITX);    //�ܽ� SSICLK SSIFSS SSITX����ΪSSI�ܽ�  PA.2 PA.3 PA.5
    GPIOPinTypeTimer(GPIOA,                 //�˿�
                     JZ_IN|FH_IN);          //�ܽ� PA.6 PA.7
*/ 
//��ʼ��GPIOB��

    GPIOPinTypeTimer(GPIOB,                 //�˿�
                     SZ_MC);                //�ܽ� PB.1
    GPIOPinTypeI2C(GPIOB,                   //�˿�
                   I2C_SCL|                 //�ܽ� PB.2 
																			I2C_SDA);                //�ܽ� PB.3         
    GPIOPinTypeGPIOOutput(GPIOB,            //�˿�
                          I2C_WP|
                          CS5460A_MODE|     //�ܽ� PB.6,
                          CS5460A_CS);      //�ܽ� PB.5
    GPIOPinTypeGPIOInput(GPIOB,
                         CS5460A_SDO);      //�ܽ� PB.4
                          
//��ʼ��GPIOC��                             
    GPIOPinTypeTimer(GPIOC,                 //�˿�
                     PWM_DAC);              //�ܽ� PC.4
    GPIOPinTypeGPIOInput(GPIOC,             //�˿�
                         KEY_IN);           //�ܽ� PC.5 ��������
                        
//��ʼ��GPIOD��                             
    GPIOPinTypeCAN(GPIOD,                   //�˿�
                   CANRX|                   //�ܽ�
                   CANTX);                  //��ʼ��CAN�ӿڹܽ�
/*                   
    GPIOPinTypeUART(GPIOD,                  //�˿�
                    U1RX|                   //�ܽ�
                    U1TX);                  //��ʼ��UART1�ӿڹܽ�
*/                    
    GPIOPinTypeGPIOOutput(GPIOD,            //�˿�
                          CS5460A_SCLK|     //�ܽ� PD.4
                          CS5460A_RESET|    //�ܽ� PD.5
																										CS5460A_SDI);     //�ܽ� PD.7
    GPIOPinTypeGPIOInput(GPIOD,             //�˿�
                         CS5460A_INT);      //�ܽ� PD.6    
//��ʼ��GPIOE��  
    GPIOPinTypeGPIOOutput(GPIOE,            //�˿�
                          CHNL1_Sel|	       //�ܽ� PE.3
                          P3P4SelOut3|      //     PE.0
                          P3P4SelOut2|      //     PE.1                
                          P3P4SelOut1);     //     PE.2 
                           
//��ʼ��GPIOF��                             
    GPIOPinTypeGPIOOutput(GPIOF,            //�˿�
                          WDI);             //PF.7 ���ÿ��Ź���λ           
                                            
    GPIOPinTypeGPIOInput(GPIOF,             //�˿�
                         GOG_KZ|
                         MC_PN_KZ|
                         MC_WV_KZ|
                         TX_MC|
                         XL_MC|             //     PF.3 ������������
                         TQ_MC|             //���� PF.4 ʱ��Ͷ������
                         HZ_MC);            //���� PF.5 ��բ����
//��ʼ��GPIOH��                             
    GPIOPinTypeGPIOOutput(GPIOH,            //�˿�
                          Phase_Sel_K1|     //�ܽ� PH.0 ���Ĳ������ѡ��1
                          Phase_Sel_K2|     //�ܽ� PH.1 ���Ĳ������ѡ��2
                          CHNL2_Sel|        //�ܽ� PH.2 ���Ĳ���ͨ��2ѡ��
                          CHNL3_Sel);       //�ܽ� PH.3 ���Ĳ���ͨ��3ѡ��
             
//��ʼ���˿��ж� GPIOC
    GPIOPadConfigSet(GPIOC,                 //�˿� ���ùܽ�����    
                     KEY_IN,                //�ܽ�   |XL_MC              
                     GPIO_STRENGTH_8MA,     //��������             
                     GPIO_PIN_TYPE_STD_WPU);//����/����               
    GPIOIntTypeSet(GPIOC,                   //�˿� ���ùܽ��жϷ���
                   KEY_IN,                  //�ܽ� |XL_MC                
                   GPIO_FALLING_EDGE);      //�жϷ�ʽ             

//��ʼ���˿��ж� GPIOD
    GPIOPadConfigSet(GPIOD,                 //�˿� ���ùܽ�����    
                     CS5460A_INT,           //�ܽ� CS5460A�жϡ�
                     GPIO_STRENGTH_8MA,     //��������             
                     GPIO_PIN_TYPE_STD_WPU);//����/����               
    GPIOIntTypeSet(GPIOD,                   //�˿� ���ùܽ��жϷ�ʽ
                   CS5460A_INT,             //�ܽ� 5460A��INT
                   GPIO_FALLING_EDGE);      //�жϷ�
                                    
//�˿ڳ�������
    CS5460A_CS_H;                           //��ֹƬѡCs5460A
    CS5460A_SCLK_H;                         //
    CS5460A_SDI_H;
    CS5460A_RESET_H;
    CS5460A_MODE_L;
    CNCL_CHNL_Sel1;                          //��ѡͨ����ͨ��
    CNCL_CHNL_Sel2;
    Sel_Phase_NO;                           //��ѡͨ�������
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;
    WDI_HIGH;
}

/*****************************************************************************
* ���I/O�ڻ�ַ�Ƿ���ȷ 
* ����ʱʹ��
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
* ��ȡI/O�豸�жϺ�
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
* ����I/O���жϷ���������
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* pfnIntHandler �ж����
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
* ����I/O���жϷ���������
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
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
* ���ùܽ�ΪADC����
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeADC(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_ANALOG);

}

/*****************************************************************************
* ���ùܽ�ΪCAN �ӿ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeCAN(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_HW);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

/*****************************************************************************
* ���ùܽ�Ϊģ��Ƚ����ܽ�
* GPIOx I/O�ڽṹ�� GPIOA~GPIOH
* ucPins �ܽ�GPIO_PIN_0~GPIO_PIN_7�����
*****************************************************************************/
void GPIOPinTypeComparator(GPIO_Typedef *GPIOx, u8 ucPins)
{
    ASSERT(GPIOBaseValid(GPIOx));

    GPIODirModeSet(GPIOx, ucPins, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_ANALOG);
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

    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD_WPU);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);	 //GPIO_PIN_TYPE_STD
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);		 //GPIO_PIN_TYPE_STD
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
    GPIOPadConfigSet(GPIOx, ucPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}

