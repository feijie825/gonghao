/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Interrupt.c
;* Author             : ������
;* �жϴ���������
;* Ҫ�ı䷢�ͻ����FIFO�������� Ҫ������Ӧ���� 
*******************************************************************************/			   
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "CLK_ERR.h"
#include "ENG_ERR.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#include "CS5460A.h"
/********************************************************
* �����жϷ������
********************************************************/
void IntDefaultHandler(void)
{
    for(;;)
     {}
}

/********************************************************
* ���Ź��жϷ������
********************************************************/
void WatchDogHandler(void)
{
    WatchdogIntClear();	  //������Ź��ж�
}
/********************************************************
* ϵͳ����ʱ�Ӷ�ʱ���жϷ������
********************************************************/
void SysTickIntHandler(void)
{
    Timer_1ms++;                           //1ms��ʱ��
//    STD_CLK_Timer++;                       //����׼ʱ�����嶨ʱ
    CLK_Timer++;                           //ʱ�����嶨ʱ
    if(WORK_Timer)                         //���빤��ģʽ��ʱ
     WORK_Timer--;                         //	
    if(Disp_En_Timer)                      //��ʾʹ�ܶ�ʱ
     Disp_En_Timer--;                      //
    if(!(Timer_1ms%8))
     Timer_8ms++;
}
/********************************************************
* �������з�����0�жϷ������
********************************************************/
void ADC0Handler(void)
{
//#ifdef _ADC
    ADCIntClear(0);                    //����ж�
    ADCSequenceDataGet(0,ADC_SEQ_Data);//��ֵ
    ADC_Data.Data=(u16)ADC_SEQ_Data[0];
    ADC_Data.New_Data=1;               //�����ݱ�־
    ADC_Data.Trig=0;                   //ADC����������־
//#endif
}
/*****************************************************************************
*
* ����0�жϷ������
*
*****************************************************************************/
void UART0IntHandler(void)
{
    u32 ulStatus;
    ulStatus = UARTIntStatus(UART0, true);
    UARTIntClear(UART0, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {       //����FIFO�ж�
      COM0_STimer=(u16)Timer_1ms;      //������ʱ 	    
      if(COM0_InSend!=COM_TX_IS)       //���ڷ���״̬
       return;
      if(COM0_OHead!=COM0_OTail)       //�ж��Ƿ�������Ҫ����
       {	
        if(Send_One_Uart_Frm(COM0_BUF)==BUF_FIFO_NBLK)  //��������һ֡ �ж��Ƿ��к�������
         return;                       //�к������� �˳� �ȴ������´�ѭ�� ����	
       }
      else 
       {	
        COM0_InSend=0;                 //������ڷ��ͱ�־
        COM0_OCoun=0;                  //����������������
       }  	 
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {	                     //����FIFO�жϻ���ճ�ʱ�ж�
      u8 m,c;
      u16 t;
      m=16; 	 
      for(;UARTCharsAvail(UART0);) //ѭ��������FIFO
       {
        c=(u8)UARTCharGet(UART0); //�����ַ�
        if((c=='\r')||            //�յ��س���
           ((c>' ')&&(c<='~')))  
         {                        //�յ����ַ�Ϊ���з�'\r'��ASC��
          t=COM0_IHead+1;
          if(t>=COM0_ILEN)
           t=0;
          if(t==COM0_ITail)
           continue;               //û�пռ� ���������� 	
          COM0_IBuf[COM0_IHead]=c; //��������
          COM0_IHead=t;
          if(c=='\r')
           COM0_ICoun++;         //�յ�һ����������
         }
        m--;
        if(m==0)
         break;
       }  
     }
}
/*****************************************************************************
*
* ����1�жϷ������
*
*****************************************************************************/
void UART1IntHandler(void)
{
    u32 ulStatus;
    ulStatus = UARTIntStatus(UART1, true);
    UARTIntClear(UART1, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {       //����FIFO�ж�
      COM1_STimer=(u16)Timer_1ms;      //������ʱ 	    
      if(COM1_InSend!=COM_TX_IS)       //���ڷ���״̬
       return;
      if(COM1_OHead!=COM1_OTail)       //�ж��Ƿ�������Ҫ����
       {	
        if(Send_One_Uart_Frm(COM1_BUF)==BUF_FIFO_NBLK)  //��������һ֡ �ж��Ƿ��к�������
         return;                       //�к������� �˳� �ȴ������´�ѭ�� ����	
       }
      else 
       {	
        COM1_InSend=0;                 //������ڷ��ͱ�־
        COM1_OCoun=0;                  //����������������
       }  
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {	                     //����FIFO�жϻ���ճ�ʱ�ж�
      u8 m,c;
      u16 t;
      m=16; 	 
      for(;UARTCharsAvail(UART1);) //ѭ��������FIFO
       {
        c=(u8)UARTCharGet(UART1); //�����ַ�
        if((c=='\r')||            //�յ��س���
           ((c>' ')&&(c<='~')))  
         {                        //�յ����ַ�Ϊ���з�'\r'��ASC��
          t=COM1_IHead+1;
          if(t>=COM1_ILEN)
           t=0;
          if(t==COM1_ITail)
           continue;               //û�пռ� ���������� 	
          COM1_IBuf[COM1_IHead]=c; //��������
          COM1_IHead=t;
          if(c=='\r')
           COM1_ICoun++;         //�յ�һ����������
         }
        m--;
        if(m==0)
         break;
       }  
     }
}
/*****************************************************************************
*
* ��ʱ��0-A�жϷ������
* ���ڵ����������
*****************************************************************************/
void Timer0AIntHandler(void)
{
    TIMER0->ICR = TIMER_TIMA_TIMEOUT; //������ж�
}
/*****************************************************************************
*
* ��ʱ��0-B�жϷ������
* ���ڱ�׼�����Ƶ���� ����131.07ms�ж�һ��
* ������Ƶֵ0xFFFF 
* ��ǰ����ֵ=STD_CLK_Cnt*0xFFFF+(0xFFFF-TIMER0B)
*           =((STD_CLK_Cnt+1)*0xFFFF)-TIMER0B
*           =(STD_CLK_Cnt+1)*(0x10000-1)-TIMER0B
*           =(STD_CLK_Cnt+1)*0x10000-(STD_CLK_Cnt+1)-TIMER0B 
*           =(STD_CLK_Cnt+1)*0x10000-STD_CLK_Cnt-TIMER0B-1
*           =(STD_CLK_Cnt+1)*0x10000-STD_CLK_Cnt+(~TIMER0B+1)-1
*           =(STD_CLK_Cnt<<16)-STD_CLK_Cnt+0x10000+(~TIMER0B)
* �������μ�����ֵ ��һ�μ�����λ STD_CLK_Cnt1 TIMER0B1
*                  �ڶ��μ�����λ STD_CLK_Cnt2 TIMER0B2
*                  �����ֵʱ ����0x10000 ����
* ��ֵ=
*****************************************************************************/
void Timer0BIntHandler(void)
{
    TIMER0->ICR = TIMER_TIMB_TIMEOUT;               //������ж�
    TIMER0->TBILR = 0xFFFF;                         //���ü��� 0xFFFF
    TIMER0->CTL &=~TIMER_CTL_TBEN;                  //Timer1-B����
}
/*****************************************************************************
* ��ʱ��1-A�жϷ������
* ����ʱ���������
*****************************************************************************/
void Timer1AIntHandler(void)
{
    TIMER1->ICR = TIMER_TIMA_TIMEOUT;               //������ж�
    TIMER1->TAILR = 60000;                          //���ü���
    TIMER1->CTL |=TIMER_CTL_TAEN;                   //Timer1-A����
    TIMER1->CTL &=~TIMER_CTL_TAEN;                  //Timer1-A ֹͣ����		
    return;
}
/*****************************************************************************
*
* ��ʱ��1-B�жϷ������
* ���ڱ������Ƶ����
*****************************************************************************/
void Timer1BIntHandler(void)
{
#ifdef __TEST
    TIMER1->ICR = TIMER_TIMB_TIMEOUT;           //����ж�
    TIMER1->TBILR = 24;                         //���ü���
    TIMER1->TBMATCHR=12;                        //50%
    TIMER1->CTL |=TIMER_CTL_TBEN;               //Timer2-B����
#else
    TIMER1->ICR = TIMER_TIMB_TIMEOUT;               //������ж�
    TIMER1->TBILR = 0xFFFF;                         //���ü���
    TIMER1->CTL |=TIMER_CTL_TBEN;                   //Timer1-B����
#endif
}
/*****************************************************************************
*
* ��ʱ��2-A�жϷ������
* ���ڱ�׼����Ƶ���� ����ʱ��1.2us
* 30K �ж�ʱ��Ϊ 2.1845s
* 10K �ж�ʱ��Ϊ 6.5535s
*****************************************************************************/
void Timer2AIntHandler(void)
{
    TIMER2->ICR = TIMER_TIMA_TIMEOUT;               //������ж�
    TIMER2->TAILR = 0xFFFF;                         //���ü���
    TIMER2->CTL |=TIMER_CTL_TAEN;                   //Timer2-A����
}
/*****************************************************************************
*
* ��ʱ��2-B�жϷ������
* ����PWM���
*****************************************************************************/
void Timer2BIntHandler(void)
{
    TIMER2->ICR = TIMER_TIMB_TIMEOUT;           //����ж�
    TIMER2->TBILR = 19;                         //���ü���
    TIMER2->TBMATCHR=9;                         //50%
    TIMER2->CTL |=TIMER_CTL_TBEN;               //Timer2-B����
}
/*****************************************************************************
* GPIOB �˿� �жϴ�������
* ���ͷ���������ж�
* ���������ж�(����������������) 
* ʱ�������ж�����(����ʱ����������)
*****************************************************************************/
void GPIOBHandler(void)
{
    u8  Int;                            //�ж�״̬
    Int=GPIOB->MIS;                     //��ȡI/O���ж�״̬
    GPIOB->ICR=Int;                     //���GPIOB I/O���ж�
}
/*****************************************************************************
* GPIOC �˿� �жϴ�������
* �����ж�
*****************************************************************************/
void GPIOCHandler(void)
{
    u8 Int;
    Int=GPIOC->MIS;                   //��ȡ�ж�״̬
    GPIOC->ICR=Int;                   //����˿��ж�
    if(Int&KEY_IN)                     
     {
      GPIOC->IM&=~KEY_IN;             //�����жϿ��жϽ���  
      KEY_Timer=Timer_8ms;            //�����ж�ʹ�ܶ�ʱ
      KEY_INT_REEN=1;                 //�����ʹ�ܱ�־ 
      if(NEW_KEY_FLAG)                //�ϴΰ���û�д��� ��Ϊ���������� �ұ�or����ұ�
       {	
        NEW_KEY_FLAG=0;               //�յ������жϱ�־
        PLUG_CHG_FLAG=1;	
       }
      else
       {	
        NEW_KEY_FLAG=1;               //�յ������жϱ�־
        PLUG_CHG_FLAG=0;	
       }
      Con_KEY_Timer=Timer_8ms;        //����������ʱ
     }
}
/*****************************************************************************
* GPIOD �˿� �жϴ�������
* CS5460A  INT�ж�
*****************************************************************************/
void GPIODHandler(void)
{
  u8 Int;
  Int=GPIOD->MIS;                            //��ȡ�ж�״̬
  GPIOD->ICR=Int;                                //����˿��ж�
  if(Int&CS5460A_INT) 
  {
    u8 Dat;
    Dat*=0;
	   Timer_CS5460A_INT=(u16)Timer_1ms;
    CS5460A_SCLK_L;
    CS5460A_CS_L;
    Dat=Write_Read_OneByte_5460A(REGISTER_READ|STATUS_REG_ADDR);
	   CS5460A_Sts_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Sts_Reg[1]=Write_Read_OneByte_5460A(0xff);  //read status reg
	   CS5460A_Sts_Reg[2]=Write_Read_OneByte_5460A(REGISTER_WRITE|STATUS_REG_ADDR);
	   Dat=Write_Read_OneByte_5460A(CS5460A_Sts_Reg[0]);
	   Dat=Write_Read_OneByte_5460A(CS5460A_Sts_Reg[1]);
	   Dat=Write_Read_OneByte_5460A(CS5460A_Sts_Reg[2]);   //clr status reg
   
	   CS5460A_CS_H;
	   if((CS5460A_Sts_Reg[0]&0x80)==0x00)        //һ����������û�н���������
	     return;
    CS5460A_SCLK_L;
    CS5460A_CS_L;
    Dat=Write_Read_OneByte_5460A(REGISTER_READ|IRMS_REG_ADDR);
	   CS5460A_Irms_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Irms_Reg[1]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Irms_Reg[2]=Write_Read_OneByte_5460A(REGISTER_READ|VRMS_REG_ADDR);
	   CS5460A_Vrms_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Vrms_Reg[1]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_Vrms_Reg[2]=Write_Read_OneByte_5460A(REGISTER_READ|E_REG_ADDR);
	   CS5460A_E_Reg[0]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_E_Reg[1]=Write_Read_OneByte_5460A(0xff);
	   CS5460A_E_Reg[2]=Write_Read_OneByte_5460A(0xfe);
	   CS5460A_CS_H;
    
	   if(ADC_Start!='Y')
	    return;
	   CS5460A_New_Data='Y'; 
  }
}
/*****************************************************************************
* GPIOF �˿� �жϴ�������
* �������������ж�
* ʱ��Ͷ������
* ��բ����
*****************************************************************************/
void GPIOFHandler(void)
{
    u8 Int;
    Int=GPIOF->MIS;                         //��ȡ�ж�״̬
    GPIOF->ICR=Int;                         //����˿��ж�
}
/*****************************************************************************
* CAN �жϴ�������
*****************************************************************************/
void CANHandler(void)
{
    u32 ulStatus;
    ulStatus = CANIntStatus(CAN0, CAN_INT_STS_CAUSE);  //�������жϵı��Ķ���ı��
    if(ulStatus==0)                                    //���ж�
     return;                                           //����
    if(ulStatus==0x8000)                               //�ж��Ƿ�Ϊ״̬�ж�
     {                                                 //״̬�ж�
      CAN_STS.BYTE=CANStatusGet(CAN0, CAN_STS_CONTROL);//�����ߴ���״̬
      CANErrCntrGet(CAN0,&CANERR_CNT);
      return;
     }                                                 
    if(ulStatus>MSG_OBJ_NUMB)                          
     {                                                 //���ж�
      CANStatusGet(CAN0, CAN_STS_CONTROL);             //
      return;                                          //�������Ķ�������� ��ΪΪ������ջ���
     }
    CAN_ERR=0;                                         //�������� 
    CAN_STS.BYTE =0;                                   //��ȷ���ջ��� ���״̬�ж�
    CANERR_CNT=0;                                      //����������� 
    if(CAN_MSG_SET_TAB[(ulStatus-1)].CTL.BIT.RX_INT_EN)//�ж��Ƿ�Ϊ���ձ����ж�
     {                                                 //�����ж�
      memset(&CAN_MSG_Rx,0,8);                         //������ձ��Ŀ�������ٲ���
      CAN_MSG_Rx.CTL.BIT.IDx=ulStatus;                 //���Ķ�����
      CAN_Rx_Msg(CAN0, &CAN_MSG_Rx, 1);                //����֡ ������ж�
      switch(ulStatus)                                 //0x8000״̬�ж�  0x01---0x20 �����жϵı��Ķ���
       {	                                                                  
        case MST_SCDATA_TX:                             //�������ڷ��Ͷ̹㲥���ݴ�ID8
        case MST_SSDATA_TX:                             //�������ڷ��Ͷ̵������ݴ�ID9
         {                                              //�����ݽ��մ��� �����ж�
          if((CAN_SDATA_MSG_IHead+1)!=CAN_SDATA_MSG_ITail)//�ж��Ƿ��п��пռ�
           {
            memcpy(&CAN_SDATA_MSG_IBUF[CAN_SDATA_MSG_IHead],
                   &CAN_MSG_Rx,16);                    //��������
            CAN_SDATA_MSG_IHead++;                     //ָ���1 ָ����һ֡
            if(CAN_SDATA_MSG_IHead>=CAN_SDILEN)        //�ж��Ƿ񳬳�ѭ��
             CAN_SDATA_MSG_IHead=0;                    //��ͷ��ʼѭ�� 
           }
           break;
         }
        default:
         {
          if((CAN_LDATA_MSG_IHead+1)!=CAN_LDATA_MSG_ITail)//�ж��Ƿ��п��пռ�
           {
            memcpy(&CAN_LDATA_MSG_IBUF[CAN_LDATA_MSG_IHead],
                   &CAN_MSG_Rx,16);                    //��������
            CAN_LDATA_MSG_IHead++;                     //ָ���1 ָ����һ֡
            if(CAN_LDATA_MSG_IHead>=CAN_LDILEN)        //�ж��Ƿ񳬳�ѭ��
             CAN_LDATA_MSG_IHead=0;                    //��ͷ��ʼѭ�� 
           }
          break;
         }
       }
     }
    else
     {                                                 //�����ж�
      switch(ulStatus)                                 //0x8000״̬�ж�  0x01---0x20 �����жϵı��Ķ���
       {	                                             
        case SLV_SDATA_TX:                          //�ӻ����Ͷ����ݴ���֡��� ID13
         {                                          //�����ж�
          CAN_SMSG_TX_STS =COM_TX_IDLE;             //��֡�������
          CAN_STX_OVTimer=(u16)Timer_1ms;           //���÷��Ͷ�ʱ
          break;
         }
        default:
         break;
       }
      CANIntClear(CAN0,ulStatus);                      //��������ж�
     }  
}