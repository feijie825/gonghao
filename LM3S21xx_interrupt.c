/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_interrupt.c
;* Author             : ������
;* �жϹ�����������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "Interrupt.h"

//*****************************************************************************
//
// The processor vector table.
//
// This contains a list of the handlers for the various interrupt sources in
// the system.  The layout of this list is defined by the hardware; assertion
// of an interrupt causes the processor to start executing directly at the
// address given in the corresponding location in this list.
//
//*****************************************************************************
#if defined(ewarm)
static __no_init void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void) @ "VTABLE";
#elif defined(sourcerygxx)
static __attribute__((section(".cs3.region-head.ram")))
void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void);
#else
static __attribute__((section("vtable")))
void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void);	//����ָ��
#endif
/*****************************************************************************
* ��ʼ���ж� 
* �ж����ȼ� 0��� 255���
* ��׼����Ƶ�ʽϸ� �ж���Ч����� ��ֹ������
*****************************************************************************/
void Init_Int(void)
{  
    IntPrioritySet(FAULT_SYSTICK,INT_PRIORITY_5);   //ϵͳ���ķ����� �ж����ȼ�  2 ϵͳ��ʱ���ж�
    IntPrioritySet(INT_UART0,INT_PRIORITY_6);       //UART0          �ж����ȼ�  6 UART0�ж�
    IntPrioritySet(INT_UART1,INT_PRIORITY_6);       //UART1          �ж����ȼ�  6 UART1�ж�
    IntPrioritySet(INT_TIMER0A,INT_PRIORITY_3);     //TIMER0-A       �ж����ȼ�  3 ������������ж�
//    IntPrioritySet(INT_TIMER0B,INT_PRIORITY_0);     //TIMER0-B       �ж����ȼ�  0 ��׼�����Ƶ��������ж�
//    IntPrioritySet(INT_TIMER1A,INT_PRIORITY_1);     //TIMER1-A       �ж����ȼ�  1 ʱ�������������ж�
//    IntPrioritySet(INT_TIMER1B,INT_PRIORITY_3);     //TIMER1-B       �ж����ȼ�  3 ������Ƶ��������ж�
    IntPrioritySet(INT_TIMER2A,INT_PRIORITY_2);     //TIMER2-A       �ж����ȼ�  2 ��׼���Ƶ��������ж� 
//    IntPrioritySet(INT_TIMER2B,INT_PRIORITY_7);   //TIMER2-B       �ж����ȼ�  7 PWM ���
    IntPrioritySet(INT_CAN0,INT_PRIORITY_4);        //CAN            �ж����ȼ�  4 CAN �ж�
//    IntPrioritySet(INT_WATCHDOG,INT_PRIORITY_0);  //���Ź�         �ж����ȼ�  0 ���Ź� �ж�
    IntPrioritySet(INT_GPIOB,INT_PRIORITY_3);       //GPIOB          �ж����ȼ�  3 GPIOB�ж� ���ͷ�������� 
    IntPrioritySet(INT_GPIOC,INT_PRIORITY_5);       //GPIOC          �ж����ȼ�  3 �����ж�
    IntPrioritySet(INT_GPIOD,INT_PRIORITY_3);       //GPIOD          �ж����ȼ�  3 CS5460A_INT
//    IntPrioritySet(INT_GPIOF,INT_PRIORITY_3);       //GPIOF          �ж����ȼ�  3 �������� ʱ��Ͷ�� ��բ�����ж� 
    
    IntEnable(FAULT_SYSTICK);   //ϵͳ���ķ�����NVIC�ж�ʹ��  ϵͳ��ʱ���ж�
    IntEnable(INT_UART0);       //UART0         NVIC�ж�ʹ��  UART0�ж�
    IntEnable(INT_UART1);       //UART1         NVIC�ж�ʹ��  UART1�ж�
    IntEnable(INT_TIMER0A);     //TIMER0-A      NVIC�ж�ʹ��  ������������ж�
//    IntEnable(INT_TIMER0B);     //TIMER0-B      NVIC�ж�ʹ��  ��׼�����Ƶ��������ж�
//    IntEnable(INT_TIMER1A);     //TIMER1-A      NVIC�ж�ʹ��  ʱ�������������ж�
#ifndef __TEST
//    IntEnable(INT_TIMER1B);     //TIMER1-B      NVIC�ж�ʹ��  ������Ƶ��������ж�
#endif
    IntEnable(INT_TIMER2A);     //TIMER2-A      NVIC�ж�ʹ��  ��׼���Ƶ��������ж� 
//    IntEnable(INT_TIMER2B);   //TIMER2-B      NVIC�ж�ʹ��  PWM ���
    IntEnable(INT_CAN0);        //CAN           NVIC�ж�ʹ��  CAN �ж�
//    IntEnable(INT_WATCHDOG);    //���Ź�        NVIC�ж�ʹ��
    IntEnable(INT_GPIOB);       //GPIOB         NVIC�ж�ʹ��  GPIOB�ж� ���ͷ��������
    IntEnable(INT_GPIOC);       //GPIOC         NVIC�ж�ʹ��  GPIOC�ж� ��������
    IntEnable(INT_GPIOD);       //GPIOD         NVIC�ж�ʹ��  GPIOD�ж� CS5460A_INT
//    IntEnable(INT_GPIOF);       //GPIOF         NVIC�ж�ʹ��  GPIOF�ж� �������� ʱ��Ͷ�� ��բ����
	IntEnable(INT_ADC0);							//ʹ��ADC sequence0 NVIC�ж�
}
/*****************************************************************************
* CPU�ж�ʹ��
*****************************************************************************/
u8 IntMasterEnable(void)
{
    return(CPUcpsie());
}

/*****************************************************************************
* CPU�жϽ���
*****************************************************************************/
u8 IntMasterDisable(void)
{
    return(CPUcpsid());
}

/*****************************************************************************
* �����ж�����ӳ���
* ���ulInterrupt �ж�ID��
* ���pfnHandler �жϷ�����
*****************************************************************************/
void IntRegister(u32 ulInterrupt, void (*pfnHandler)(void))
{
    u32 ulIdx;
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    ASSERT(((u32)g_pfnRAMVectors & 0x000003ff) == 0);
    if(NVIC->VTABLE != (u32)g_pfnRAMVectors)
     {
        for(ulIdx = 0; ulIdx < NUM_INTERRUPTS; ulIdx++)
         g_pfnRAMVectors[ulIdx] = (void (*)(void))HWREG(ulIdx * 4);
        NVIC->VTABLE = (u32)g_pfnRAMVectors;
     }
    g_pfnRAMVectors[ulInterrupt] = pfnHandler;
}

/*****************************************************************************
* �����ж��ӳ���
* ���ulInterrupt �ж�ID��
*****************************************************************************/
void IntUnregister(u32 ulInterrupt)
{
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    g_pfnRAMVectors[ulInterrupt] = IntDefaultHandler;
}

/*****************************************************************************
* ���ȼ����������ӳ���
* ���ulBits ���ȼ����� ulBits=0~7 
* ulBits ռ�����ȼ�λ��
*****************************************************************************/
void IntPriorityGroupingSet(u32 ulBits)
{
    ASSERT(ulBits < NUM_PRIORITY);
    ulBits=((~(ulBits<<8)) & NVIC_APINT_PRIGROUP_M);
    NVIC->APINT = (NVIC_APINT_VECTKEY | ulBits);
}

/*****************************************************************************
* ��ȡ���ȼ����������ӳ���
* ���ulBits ���ȼ����� ulBits=0~7 
*****************************************************************************/
u32	IntPriorityGroupingGet(void)
{
    u32 ulValue;

    ulValue = NVIC->APINT ;
    ulValue = ((~(ulValue>>8)) & NVIC_APINT_PRIGROUP_M);
    return(ulValue);
}

/*****************************************************************************
* �����ж����ȼ�
* ��� ulInterrupt �жϺ�
* ��� ucPriority  ���ȼ�
*****************************************************************************/
void IntPrioritySet(u32 ulInterrupt, u8 ucPriority)
{
    u32 ulTemp;
    ASSERT((ulInterrupt >= 4) && (ulInterrupt < NUM_INTERRUPTS));
	if(ulInterrupt<16)
	 {	//����ϵͳ�ж����ȼ�
      ulTemp = NVIC->SYS_PRI[(ulInterrupt>>2)-1];
      ulTemp &= ~(0xFF << (8 * (ulInterrupt & 3)));
      ulTemp |= ucPriority << (8 * (ulInterrupt & 3));
      NVIC->SYS_PRI[(ulInterrupt>>2)-1] = ulTemp;
	 }
	else
	 {  //�����豸�ж����ȼ�
      ulTemp = NVIC->PRI[(ulInterrupt>>2)-4];
      ulTemp &= ~(0xFF << (8 * (ulInterrupt & 3)));
      ulTemp |= ucPriority << (8 * (ulInterrupt & 3));
      NVIC->PRI[(ulInterrupt>>2)-4] = ulTemp;
	 }
}

/*****************************************************************************
* ��ȡ�豸�ж����ȼ�
* ���� ���ȼ�
*****************************************************************************/
u32 IntPriorityGet(u32 ulInterrupt)
{
	u32 Priority;
    ASSERT((ulInterrupt >= 4) && (ulInterrupt < NUM_INTERRUPTS));
	if(ulInterrupt<16)
	 {  //��ȡ�ں��豸���ȼ�
	  Priority=NVIC->SYS_PRI[(ulInterrupt>>2)-1];
	 }
	else
	 {
	  Priority=NVIC->PRI[(ulInterrupt>>2)-4];
	 }
	ulInterrupt &= 3;
	ulInterrupt <<=3;   //*8
    return((Priority >> ulInterrupt) &
           0xFF);
}

/*****************************************************************************
* �ж�ʹ���ӳ���
* ���:ulInterrupt �ж�ID��
*****************************************************************************/
void IntEnable(u32 ulInterrupt)
{
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    if(ulInterrupt == FAULT_MPU)	     //�洢�������ж�
     NVIC->SYS_HND_CTRL |= NVIC_SYS_HND_CTRL_MEM;
    else if(ulInterrupt == FAULT_BUS)    //���ߴ����ж�
     NVIC->SYS_HND_CTRL |= NVIC_SYS_HND_CTRL_BUS;
    else if(ulInterrupt == FAULT_USAGE)  //ʹ�ù���
     NVIC->SYS_HND_CTRL |= NVIC_SYS_HND_CTRL_USAGE;
    else if(ulInterrupt == FAULT_SYSTICK)//ϵͳ���Ķ�ʱ���ж�
     NVIC->ST_CTRL |= NVIC_ST_CTRL_INTEN;
    else
     NVIC->EN[((ulInterrupt-16)/32)] = 1 << ((ulInterrupt - 16)%32);
}

/*****************************************************************************
* �жϽ����ӳ���
* ���:ulInterrupt �ж�ID��
*****************************************************************************/
void IntDisable(u32 ulInterrupt)
{
    ASSERT(ulInterrupt < NUM_INTERRUPTS);
    if(ulInterrupt == FAULT_MPU)
     NVIC->SYS_HND_CTRL &= ~(NVIC_SYS_HND_CTRL_MEM);
    else if(ulInterrupt == FAULT_BUS)
     NVIC->SYS_HND_CTRL &= ~(NVIC_SYS_HND_CTRL_BUS);
    else if(ulInterrupt == FAULT_USAGE)
     NVIC->SYS_HND_CTRL &= ~(NVIC_SYS_HND_CTRL_USAGE);
    else if(ulInterrupt == FAULT_SYSTICK)
     NVIC->ST_CTRL &= ~(NVIC_ST_CTRL_INTEN);
    else
     NVIC->DIS[((ulInterrupt-16)/32)] = 1 << ((ulInterrupt - 16)%32);
}

