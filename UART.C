/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : UART.c
;* Author             : ������
;* �������ݴ�������
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#include "DISP.h"
#include "eng_err.h"
#include "stdlib.h"
#include "CS5460A.h"
#include "stdio.h"
//CAN �ӿڵ�ַ����
const u32 UART_PORT[UART_NUMB]=
{
	UART0_BASE,
	UART1_BASE,
};
/****************************************************************************
* COM0�����������
****************************************************************************/
const u8 COM0_ICMD_TAB[][8]=
{
    {"C00\r"},        //
    {"SEQUP:"},       //���õ�����
    {"Gd:"},
    {"Mtr:"},
    {"Ge:"},
    {"Gf:"},
    {"Gg:"},
		
		{"Gh:"},          //������ѹ
    {"Gi:"},          //��������
		{"Gou:"},         //������ѹ���
		{"Goi:"},         //�����������
    {"Gj:"},
    {"VER\r"},
    {"SOLID\r"},
    {"BOOT:"},        //������������
    {"$I1:"},         //A�������Чֵ
    {"$I2:"},         //B�������Чֵ
    {"$I3:"},         //C�������Чֵ
    {"I:"},           //���������Чֵ
};
/****************************************************************************
* COM1�����������
****************************************************************************/
const u8 COM1_ICMD_TAB[][8]=
{
    {"C00\r"},        //
    {"SEQUP:"},       //���õ�����
    {"Gd:"},
    {"Mtr:"},
    {"Ge:"},
    {"Gf:"},
    {"Gg:"},
		{"Gh:"},          //������ѹ
    {"Gi:"},          //��������
		{"Gou:"},         //������ѹ���
		{"Goi:"},         //�����������
    {"Gj:"},
    {"VER\r"},
    {"SOLID\r"},
    {"BOOT:"},        //������������
    {"$I1:"},         //A�������Чֵ
    {"$I2:"},         //B�������Чֵ
    {"$I3:"},         //C�������Чֵ
    {"I:"},           //���������Чֵ
};
/****************************************************************************
* ���������趨�Ľ���ƥ�������������
****************************************************************************/
const u16 CMD_NUM_TAB[]=
{
    sizeof(COM0_ICMD_TAB)/sizeof(COM0_ICMD_TAB[0]),       //COM0���ջ�����        
    sizeof(COM1_ICMD_TAB)/sizeof(COM1_ICMD_TAB[0]),       //COM1���ջ�����        
};

/****************************************************************************
* ��������������ݱ�־����
****************************************************************************/
const u8 Data_Flag_Tab[]=
{
    ':',                           //COM0���ջ�����  ��׼��   
    ':',                           //COM1���ջ�����              
};
/****************************************************************************
* ����ͷ��ַ����
****************************************************************************/
const CChar_Ptr CMD_ADDR_TAB[]=
{
    COM0_ICMD_TAB[0],              //COM0���ջ�����        
    COM1_ICMD_TAB[0],              //COM1���ջ�����        
};
/****************************************************************************
* ��������ÿ������ͷ���ȱ���
****************************************************************************/
const u8 CMD_LEN_TAB[]=
{
    sizeof(COM0_ICMD_TAB[0]),         //COM0���ջ�����  ��׼��    
    sizeof(COM1_ICMD_TAB[0]),         //COM1���ջ�����        
};
/****************************************************************************
* ���ջ��������� ����
****************************************************************************/
const IBUF_Pr IBUF_Ptr_TAB[]=
{
    {//COM0���ջ�����
     COM0_IBuf,    
     &COM0_IHead,
     &COM0_ITail,
     COM0_ILEN,
    },
    {//COM1���ջ�����
     COM1_IBuf,    
     &COM1_IHead,
     &COM1_ITail,
     COM1_ILEN,
    },
};
/****************************************************************************
* ���ͻ��������� ����
****************************************************************************/
const OBUF_Pr OBUF_Ptr_TAB[]=
{
    {//COM0���ͻ�����
     COM0_OBuf,    
     &COM0_OHead,
     &COM0_OTail,
     &COM0_OHead,         //��Ч Ϊ��ƥ��ṹ��
     &COM0_InSend,        //��Ч Ϊ��ƥ��ṹ��
     COM0_OLEN,
    },
    {//COM1���ͻ�����
     COM1_OBuf,    
     &COM1_OHead,
     &COM1_OTail,
     &COM1_OHead,         //��Ч Ϊ��ƥ��ṹ��
     &COM1_InSend,        //��Ч Ϊ��ƥ��ṹ��
     COM1_OLEN,
    },
};
/*****************************************************************************
* ����BOOT �ر��ж�
*****************************************************************************/
void BOOT_INT_DIS(void)
{
    u32 *Ptr;
    Ptr=(u32*)0x20000000;
    *Ptr     = 'E';                 //λ������ַ����0x20000000 
    *(Ptr+1) = 'E';                 //λ������ַ����0x20000004
    *(Ptr+2) = Rx_Para[0][0];       //�������ݵ�ID  
    IntMasterDisable();          //����BOOTǰ �����ж�
    IntDisable(FAULT_SYSTICK);   //ϵͳ���ķ�����NVIC�ж�ʹ��  ϵͳ��ʱ���ж�
    IntDisable(INT_UART0);       //UART0         NVIC�ж�ʹ��  UART0�ж�
    IntDisable(INT_UART1);       //UART1         NVIC�ж�ʹ��  UART1�ж�
    IntDisable(INT_TIMER0A);     //TIMER0-A      NVIC�ж�ʹ��  ������������ж�
    IntDisable(INT_TIMER0B);     //TIMER0-B      NVIC�ж�ʹ��  ��׼�����Ƶ��������ж�
    IntDisable(INT_TIMER1A);     //TIMER1-A      NVIC�ж�ʹ��  ʱ�������������ж�
#ifndef __TEST
    IntDisable(INT_TIMER1B);     //TIMER1-B      NVIC�ж�ʹ��  �������Ƶ��������ж�
#endif
    IntDisable(INT_TIMER2A);     //TIMER2-A      NVIC�ж�ʹ��  ��׼����Ƶ��������ж� 
//    IntDisable(INT_TIMER2B);   //TIMER2-B      NVIC�ж�ʹ��  PWM ���
    IntDisable(INT_CAN0);        //CAN           NVIC�ж�ʹ��  CAN �ж�
//    IntDisable(INT_WATCHDOG);    //���Ź�        NVIC�ж�ʹ��
    IntDisable(INT_GPIOB);       //GPIOB         NVIC�ж�ʹ��  GPIOB�ж� ���ͷ��������
    IntDisable(INT_GPIOC);       //GPIOC         NVIC�ж�ʹ��  GPIOC�ж� ��������
    IntDisable(INT_GPIOF);       //GPIOF         NVIC�ж�ʹ��  GPIOF�ж� �������� ʱ��Ͷ�� ��բ����
//    IntMasterEnable();           //CPU�ж�����
   	for(;;)	                        //��ѭ���ȴ����Ź���λ
    {}
}
/****************************************************************************
* �ӽ��ջ�������ȡһ���ַ�
* �õ� IBUF_Ptr ���ջ����������ṹ��
****************************************************************************/
u8 Get_Char(void)
{
    u8 c;
    if(*IBUF_Ptr.ITail==*IBUF_Ptr.IHead) //ָ��ͷ�Ƿ�=ָ��β
     return('\0');	
    c=IBUF_Ptr.IBuf[*IBUF_Ptr.ITail];    // ȡһ���ַ�
    IBUF_Ptr.IBuf[*IBUF_Ptr.ITail]=0x00; // ����������
    (*IBUF_Ptr.ITail)++;                 // ����ָ���1 ��������� ����ָ��++ ������ָ���ָ��+1
    if(*IBUF_Ptr.ITail>=IBUF_Ptr.IBLEN)  // ����ָ��ѭ��
     *IBUF_Ptr.ITail=0;                       
    return c;                            // �����ַ�
}
/****************************************************************************
* ����һ�ַ�
****************************************************************************/
void PutChar(u8 c)
{
    OBUF_Ptr.OBuf[*OBUF_Ptr.OHead]=c;
    (*OBUF_Ptr.OHead)++;
    if(*OBUF_Ptr.OHead>=OBUF_Ptr.OBLEN)
     *OBUF_Ptr.OHead=0;	
}
/****************************************************************************
* �ַ�������
* ���: Str Ҫ���͵�����
* ���: Len Ҫ�������ݵĳ���
        ��Len=0ʱ  ���ַ������� ֱ������������'\0' '\r' 0xff
        ��Len!=0ʱ ֱ�ӷ���Len���ȸ��ַ�
* ��׼������ �ݲ��øó�����
* ���ӳ������ݷ��͵����ݷ��ͻ�����
****************************************************************************/
void SendStr(u8 Buf_Id,u8 *Str,u8 Len)
{
    u8 c;
    if(Buf_Id>COM1_BUF)                    //�жϷ��ͻ������Ƿ�Ϸ�
     return;            	  
    memcpy(&OBUF_Ptr,                      //װ�ط��ͻ����������ṹ��
           &OBUF_Ptr_TAB[Buf_Id],          //���ͻ����������ṹ�����
           sizeof(OBUF_Pr));               //����
    if(Len)                                //����Len���ַ�
     {
      for(;;)
       {
        PutChar(*(Str++));
        Len--;
        if(Len==0)
         return;	
       }
     }
		else
		{
			for(;;)                                
			 {
				c=*Str;                              //ȡ���ַ���
				Str++;
				if((c=='\0')||                       //�ַ��������� 
					 (c==0xFF))
				 c='\r';                                      
				PutChar(c);
				if(c=='\r')                          //�ж��ַ����Ƿ����
				 return; 
			 }
	 }
}
/****************************************************************************
* ���ڴ�������:�ҳ�һ��ָ��ͷ�ַ�����һ��ָ��������ַ����е����
* ��������������ж�Ӧ�����
* ��������ͷ����Get_Com_Cmd()�������޳�
****************************************************************************/
u16 Case_num(u8 Buf_Id)
{
    u8 m,n,k; 
    const u8 *Ptr1,*Ptr2;         //����ͷ����ָ��
    u8 *Str;                      //����ͷָ��
      Ptr1=CMD_ADDR_TAB[Buf_Id].Ptr;//�Ƚ�����ͷ��ʼλ��
      Ptr2=Ptr1;                    //�����ַ
      k=CMD_NUM_TAB[Buf_Id];        //�������
      n=CMD_LEN_TAB[Buf_Id];        //����ͷ����
    for(m=0;m<k;m++)
     {                 
      Str=Rx_Com;                   //����ͷָ��
      for(;;)                       
       {                            
        if(*Str!=*Ptr1)             
         {                          
          Ptr1=Ptr2+n;              //ָ����һ������
          Ptr2=Ptr1;                //���浱ǰָ��
          break;
         }
        if((*Ptr1=='\r')||
           (*Ptr1==Data_Flag_Tab[Buf_Id]))
         return(m);
        Str++;
        Ptr1++;
       }				
     }
    return(0xFFFF);
}
/****************************************************************************
* �ӽ��ջ�������ȡ������
* m ���� �ڼ�������
* 
* ����:����ȡ�귵��0 ���в�������1 �������󷵻�0xFF
****************************************************************************/
u8 Get_Para(u8 Buf_Id,u8 m)
{
    u8 s;
    u8 n;
    if(m>=RX_PARA_NUM)
     return 0;
    if(Buf_Id>COM1_BUF)	           //�ж϶˿��Ƿ����
     return 0;	 
    memcpy(&IBUF_Ptr,              //���ջ����������ṹ��
           &IBUF_Ptr_TAB[Buf_Id],  
           sizeof(IBUF_Pr));
    for(n=0;n<(RX_PARA_LEN-1);n++)
     {
      s=Get_Char();               //����ȡ���ַ�
      if(s=='\r')                 //������ȡ����
       {
        Rx_Para[m][n]='\0';
        return 0;                 //���в���ȡ�귴��
       }
      if((s>0x1f)&&(s<0x7F))      //�ж��Ƿ�Ϊ�����Ϸ��ַ�
       {
        Rx_Para[m][n]=s;
        if(s==',')
         {
          Rx_Para[m][n]='\0';
          return 1;               //������ȡ�� ���к������� ����1
         }
       } 
     }
    Rx_Para[m][n]='\0';
    for(;;)			                  //�ӵ����������
     {
      s=Get_Char();
      if((s=='\0')||
         (s=='\r'))
       return 0xFF;               //��������
     }		
}
/****************************************************************************
* �ӽ��ջ�����ȡһ��ָ��
* ָ��ͷ����Rx_Com ��
* ��������Rx_Para��
* ָ���������󳤶���RX_CMD_LEN����
****************************************************************************/
void Get_Com_Cmd(u8 Buf_Id)   
{
    u8 n,s;
    if(Buf_Id>COM1_BUF)
     return;	 
    memcpy(&IBUF_Ptr,           //���ջ����������ṹ��
           &IBUF_Ptr_TAB[Buf_Id],  
           sizeof(IBUF_Pr));
    	          
      for(n=0;n<(RX_CMD_LEN-1);n++)
       {
        s=Get_Char();             //ȡһ�ַ�
        if(s>'~')
         continue;
        Rx_Com[n]=s;
        if(s=='\r')               //�س���
         {
          Rx_Com[n]='\r';
          Rx_Para[0][0]='\0';
          return;               //�޲�������
         }
        else if(s==Data_Flag_Tab[Buf_Id])
          goto G_Para;
       }
      Rx_Com[n]='\0';             //����ͷ����
      for(;;)                     //�ӵ����������
       {
        s=Get_Char();             //ȡһ�ַ�
        if(s=='\r')
         return;           //����ͷ����
        if(s==':')
         break;           //����ͷ����
       }
G_Para:
      if(!Get_Para(COM0_BUF,0))    return;
      if(Get_Para(COM0_BUF,1)) 
	     {
		      for(;;)			                  //�ӵ����������
		      {
			       s=Get_Char();
			       if(s=='\r')  return;
		      }
	     }	      
}
/****************************************************************************
* û��ƥ�������
* �˿�:Buf_Id ��������� 
* ������������ �����ݶ˿ڻ�����
* �����ݶ˿ڻ�������� ���� Clear_Ldata()����
****************************************************************************/
void No_Match_Cmd_Pr(u8 Buf_Id)
{
    u8 c; 
    if(Buf_Id>COM1_BUF)            //�ж϶˿��Ƿ����
     return ;	 
    memcpy(&IBUF_Ptr,              //���ջ����������ṹ��
           &IBUF_Ptr_TAB[Buf_Id],  
           sizeof(IBUF_Pr));
    for(;;)                       //�ӵ����������
     {
      c=Get_Char();               //ȡ�ַ�������õ�Ԫ
      if((c=='\0')||
         (c=='\r'))
       return;  
     }
}
/****************************************************************************
* ����һ��������֡������FIFO
* ���: Buf ������
*                  COM0_BUF        COM0���ջ�����ID��       COM0
*                  COM1_BUF        COM1���ջ�����ID��       COM1
* ����: 0 ������ UART FIFO ����                 BUF_FIFO_BLK
*       1 ������ UART FIFO ����һ������         BUF_FIFO_NBLK
*       2 ������ UART FIFO һ�������������� ONE_CMD_BLK
****************************************************************************/
u8 Send_One_Uart_Frm(u8 Buf)
{
    u8  c,n=0;
    UART_Typedef* UARTx;
    if(Buf>COM1_BUF)                                             //�жϻ������Ƿ����
     return(BUF_FIFO_BLK);                                       //�����˳�
    UARTx=(UART_Typedef*)UART_PORT[Buf];                         //UART��Ӳ����ַ
    for(;n<(UART_TFIFO_LEN-2);)                                  //ѭ������(FIFO����-2)���ֽ�
     {
      if(*OBUF_Ptr_TAB[Buf].OTail==*OBUF_Ptr_TAB[Buf].OHead)     //�ж����������Ƿ������
       {
        *OBUF_Ptr_TAB[Buf].OCoun=0;                              //������������������
        n=0;
        break;	
       }  
      c=OBUF_Ptr_TAB[Buf].OBuf[*OBUF_Ptr_TAB[Buf].OTail];        //����һ�ַ�
      OBUF_Ptr_TAB[Buf].OBuf[*OBUF_Ptr_TAB[Buf].OTail]=0;        //����ַ�
      (*OBUF_Ptr_TAB[Buf].OTail)++;                              //����ָ��+1 ���Ų���ȥ�� ����������ָ��
      if(*OBUF_Ptr_TAB[Buf].OTail>=OBUF_Ptr_TAB[Buf].OBLEN)      //�ж��Ƿ񳬳�ѭ��
       *OBUF_Ptr_TAB[Buf].OTail=0;
      if(c=='\r')                                                //�ж��Ƿ�Ϊ�س���
       {                                                         
        (*OBUF_Ptr_TAB[Buf].OCoun)--;                            //��������1
        UARTCharPut(UARTx,c);                                    //����һ�ַ�
        n=0;
        break;
       }                                                         
      else if(c>=' ')                                            //ASC��
       {	 
        n++;
        UARTCharPut(UARTx,c);                                    //����һ�ַ�
       }	  
     } 
    COM1_STimer=(u16)Timer_1ms;                                  //������ʱ 	    
    if(n==0)                                                     //ʵ��д�봮�ڷ���FIFO���ֽ���
     return(BUF_FIFO_BLK);                                       //������Ҫ���� 
    else
     return(BUF_FIFO_NBLK);                                      //���������� 	 	
}


/******************************************************
* ��ѯ���Ĳ�������
******************************************************/
void Com_CHK_Pr(void)
{
  u8 n;
  if((Rx_Com[0]!='C')||(ADC_Start!='Y'))                                       //���Ĳ����Ƿ�ʼ
		{
//			 memset(Rx_Com,0x00,RX_CMD_LEN);
//			 CHK_OneTime_Flag=0x00;
		 	return;
		}

  if((0x2f<Rx_Com[1]<0x3a)&&(0x2f<Rx_Com[2]<0x3a))       //��ѯ�ı�λ���Ƿ�Ϊ�Ϸ�����
	 {
	   n=(Rx_Com[1]&0x0f)*10+(Rx_Com[2]&0x0f);
		  if(n!=Mtr_Numb_ID)                                   //�Ƿ��ѯ�ù���ģ��
				{
//					 memset(Rx_Com,0x00,RX_CMD_LEN);
//					 CHK_OneTime_Flag=0x00; 
					 return;
				}	
    else
    {
      memset(Rx_Com,0x00,RX_CMD_LEN);
      CHK_OneTime_Flag='Y';  
    }
  }  
//		else
//		{
//				memset(Rx_Com,0x00,RX_CMD_LEN);
//				CHK_OneTime_Flag=0x00; 
//		}
}  
/******************************************************
* ��ѯ��ѹ����ֵ
******************************************************/
void Com_CHK_Ui(void)
{
  u8 n;
  if((Rx_Com[0]!='Q')||(ADC_Start!='Y'))                                       //���Ĳ����Ƿ�ʼ
		{
//			 memset(Rx_Com,0x00,RX_CMD_LEN);
//			 CHK_OneTime_Flag=0x00;
		 	return;
		}

  if((0x2f<Rx_Com[1]<0x3a)&&(0x2f<Rx_Com[2]<0x3a))       //��ѯ�ı�λ���Ƿ�Ϊ�Ϸ�����
	 {
	   n=(Rx_Com[1]&0x0f)*10+(Rx_Com[2]&0x0f);
		  if(n!=Mtr_Numb_ID)                                   //�Ƿ��ѯ�ù���ģ��
				{
//					 memset(Rx_Com,0x00,RX_CMD_LEN);
//					 CHK_OneTime_Flag=0x00; 
					 return;
				}	
    else
    {
      memset(Rx_Com,0x00,RX_CMD_LEN);
      CHK_ONETIME_FLAG1='Y';  
    }
  }  
//		else
//		{
//				memset(Rx_Com,0x00,RX_CMD_LEN);
//				CHK_OneTime_Flag=0x00; 
//		}
}  

/******************************************************
* ������̨��������
******************************************************/
void Com_Set_ST(void)
{
    if(Rx_Para[0][0]=='3')                          //�Ƿ�����Ϊ����̨
     {	         
      if(SINGLE_OR_THREE!=THREE)                 //���������÷����ı�
       {
        SINGLE_OR_THREE=THREE;  
       }                               
     } 
    else if(Rx_Para[0][0]=='1')
     {	
      if(SINGLE_OR_THREE!=SINGLE)                //̨��״̬�ı�
       {
        SINGLE_OR_THREE=SINGLE;  
       }
     }
    else
     return;                       			       
}
/********************************************************
* ȡ������λ�źͲ���ͨ���Ķ�Ӧ��ϵ 
* ����0��˵��û�ж�Ӧ�ı�λ
* ����1���ж�Ӧ�ı�λ
********************************************************/
u8 Com_Get_Mea_Met(u8 Met)
{
  u8 n;
  for(n=0;n<3;n++)
	 {
	   if(Met==0)
	   {
		    Current_Mea_MetNUM=0;           //��ǰ�����ı�λ��       
		    Current_Mea_Channel=0;          //��ǰ������ͨ����
		    break;
		  }
	   if(Met==XIUZ.CH0_MTR[n])
	   {
		    Current_Mea_MetNUM=Met;         //��ǰ�����ı�λ��        
		    Current_Mea_Channel=(n+1);      //��ǰ������ͨ����
		    break;
		  } 
  }
	 if(n==3)
	  return 0;                          //û�ж�Ӧ�ı�λ�ţ����ı�ԭ������Ԫ 
  else
   return 1;
}
/********************************************************
* ���Ĳ��� ��λ�����ѡ�� ���� 
* Gd:XXA(0d)  XXҪ�����ı�λ
* A Ҫ�������� 0:ֹͣ���� 1:A�� 2:B�� 3:C��
********************************************************/
void Com_Power_Test_Pr(void)
{
    u8 ID=0,c;
    ID=(Rx_Para[0][0]&0x0f)*10+(Rx_Para[0][1]&0x0f);             //ȡ��������λ�� 
    c=Rx_Para[0][2]&0x0f;                                       //ȡ����������
    if((c>3)||(Com_Get_Mea_Met(ID)==0))                         //�жϿ������Ƿ����,Ҫ�����ı�λ�Ų��ڹ���ģ��Ĳ�����Χ��
     return;
    Current_Mea_Phase=c;                       //�洢�������
    CNCL_CHNL_Sel1;                             //�ͷ����в���ͨ��
    CNCL_CHNL_Sel2;
    Sel_Phase_NO;                              //�������κ����
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;
    if((ID==0)||                               //�����ı�λ����0���˳����Ĳ���
    	  (c==0))                                 //����������Ϊ0���˳����Ĳ���
    {
      ADC_Start=0x00;                          //�˳����Ĳ���    	
      Stop_CS5460AConv();
      return;
    } 	 
    else
    {
      if(Current_Mea_Phase==1)                 //����A���ѹ��������
     	  Sel_Phase_A; 
      else if(Current_Mea_Phase==2)            //����B���ѹ��������
    		  Sel_Phase_B;                           
      else if(Current_Mea_Phase==3)            //����C���ѹ��������
    		  Sel_Phase_C;                          
       
      if(Current_Mea_Channel==1)               //ͨ��1����
      {
        Sel_Channel1;	
        if((Current_Wire_Type==WIRE_P1)||                //�����й�
          (Current_Wire_Type==WIRE_Q1)||                //�����޹�
          (Current_Wire_Type==WIRE_P4)||                //���������й�
          (Current_Wire_Type==WIRE_Q4_3)||              //��������3Ԫ���޹�
          (Current_Wire_Type==WIRE_Q4_R))               //�����������޹�
          P4_Sel1;
        else
          P3_Sel1;
      }
      else if(Current_Mea_Channel==2)          //ͨ��2����
      {
        Sel_Channel2;	
        if((Current_Wire_Type==WIRE_P1)||                //�����й�
          (Current_Wire_Type==WIRE_Q1)||                //�����޹�
          (Current_Wire_Type==WIRE_P4)||                //���������й�
          (Current_Wire_Type==WIRE_Q4_3)||              //��������3Ԫ���޹�
          (Current_Wire_Type==WIRE_Q4_R))               //�����������޹�
          P4_Sel2;
        else
          P3_Sel2;
      }
      else if(Current_Mea_Channel==3)          //ͨ��3����
      {
        Sel_Channel3;	
        if((Current_Wire_Type==WIRE_P1)||                //�����й�
          (Current_Wire_Type==WIRE_Q1)||                //�����޹�
          (Current_Wire_Type==WIRE_P4)||                //���������й�
          (Current_Wire_Type==WIRE_Q4_3)||              //��������3Ԫ���޹�
          (Current_Wire_Type==WIRE_Q4_R))               //�����������޹�
          P4_Sel3;
        else
          P3_Sel3;
      }
      
      ADC_Start='Y';                           //����ADCת��
      Start_CS5460AConv();                     //����CS5460Aת��
   }
} 
/********************************************************
* ���ù��Ĳ���ͨ����Ӧ��λ��
********************************************************/
void Com_Set_Mea_Met_Tab(void)
{
  u8 chnl,Met;
  chnl=(Rx_Para[0][0]&0x0f)*10+(Rx_Para[0][1]&0x0f);             //ȡ��Ҫ���õ�ͨ���� 
	 Met=(Rx_Para[0][2]&0x0f)*10+(Rx_Para[0][3]&0x0f);              //ȡ����������
  if((chnl==0)||(chnl>3))                      //ͨ���ŵķ�Χ1-3
	  return; 	
	 if((Met==0)||Met>255)                            //��λ�ŵķ�Χ1-255
	  return;
  chnl--; 
	 if(Met==XIUZ.CH0_MTR[chnl])                        //���õ�ֵ��ԭ��Ӧֵһ�£�
	  return;
	 XIUZ.CH0_MTR[chnl]=Met;
}

/********************************************************
* ��������ֵ
* 0:���õ�ѹ�й���������ֵ
* 1:���õ�ѹ���ڹ�������ֵ
* 2:���õ����й���������ֵ
* 3:���õ�ѹ����ֵ
* 4:���õ�������ֵ
* 5:���õ�ѹ�������ֵ
* 6:���õ����������ֵ
********************************************************/
void Com_Set_Vref(u8 type)
{
	 float f,f2;
	 if(type>6)
	 	return;                                       //����ֵ����ֻ����0-6֮�������
	 f=atof(Rx_Para[0]);                            //ȡ����������ϵ��
  if(type==UP_TYP)
  {
		f2=atof(U_Pr_Str_Com);
		f/=f2;
    f*=XIUZ.U_PXZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>3))   
     return;
    XIUZ.U_PXZ=f;
  }
  else if(type==US_TYP)
  {
		f2=atof(U_Pm_Str_Com);
		f/=f2;
    f*=XIUZ.U_SXZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>3))   
     return;
    XIUZ.U_SXZ=f;
  }
  else if(type==IS_TYP)
  {
		f2=atof(I_Pm_Str_Com);
		f/=f2;
    f*=XIUZ.I_SXZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>3))   
     return;
    XIUZ.I_SXZ=f;
  } 
  else if(type==U_TYP)
  {
		XIUZ.XZDU=f;                            //�����������ѹֵ
		f/=U_current;                    //�Զ���������ϵ��
    f*=XIUZ.U_XZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>3))   
     return;
    XIUZ.U_XZ=f;
		
  } 
  else if(type==I_TYP)
  {
		XIUZ.XZDI=f;                            //�������������ֵ
		f/=I_current;                    //�Զ���������ϵ��
    f*=XIUZ.I_XZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>4))   
     return;
    XIUZ.I_XZ=f;
  }  	
	else if(type==OU_TYP)               //��ǰ�������ֵ�ټ�Ҫ������ֵ
	{
		XIUZ.U_OFFSET+=f;
		
	}
	else if(type==OI_TYP)								//��ǰ�������ֵ�ټ�Ҫ������ֵ
	{
		XIUZ.I_OFFSET+=f;
	}
}
/******************************************************
* ���߷�ʽ����
******************************************************/
void Com_Set_Wire_Type(void)
{
  u8 type;
  if(Rx_Para[0][1]=='\0')
    type=Rx_Para[0][0];
  else if(Rx_Para[0][3]=='\0')
    type=Rx_Para[0][2];
  if((type<WIRE_P1)||(type>WIRE_Q1)||(Current_Wire_Type==type))
   return;
  if(SINGLE_OR_THREE==SINGLE)                //����ֻ̨��һ�ֽ��߷�ʽ
  {
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;  
    if((type!=WIRE_P1)&&(type!=WIRE_Q1))   
     Current_Wire_Type=WIRE_P1;
    else
     Current_Wire_Type=type;
  }
  else                                       //����̨
     Current_Wire_Type=type;     
}
/******************************************************
* ���汾��,ֻ�е�һ��λ���ذ汾��
******************************************************/
void Com_Set_Read_Ver(void)             
{
    if(Mtr_Numb_ID==1)                       //�Ƿ�Ϊ��1��λ
    {    
      u8 n,d;
      for(n=0;n<10;)
       {
        d=VER_TAB[n];
        if(d=='\0')
         break;
        UART_TEMP_STR[n]=d; 
        n++;
       }
      if(n==0||n==10)                          //���ȴ���
       return;	    
     
      SendStr(COM0_BUF,                  //����
              UART_TEMP_STR,
              7);
      COM0_OCoun=0x01;
    }
}
/****************************************************************************
* ���յ�����Чֵ����
****************************************************************************/
void Com_RCV_CRT(u8 Phase)
{
  float f=0.0;
	 if(ADC_Start!='Y'||Phase!=Current_Mea_Phase)
	  return;
	 f=atof(Rx_Para[0]);
	 if(f<0||f>200)
	  return;
	 Current_I_Value=f; 	
}
/****************************************************************************
* ����COM0���ջ�����
****************************************************************************/
void Proc_COM0_IBuf(void)  
{
    u16 m;
    if(!COM0_ICoun)                                                    //�Ƿ��յ����κϳ�������
     return;                                                          
    COM0_ICoun--;                                                     
    Get_Com_Cmd(COM0_BUF);                                             //ȡ����ͷ
  
    if(Rx_Para[0][0]=='\0')                                            //�޲�������
    {
      if(Rx_Com[0]=='C') 
      {
        Com_CHK_Pr();
        return;       
      }
			else if(Rx_Com[0]=='Q')
			{
				Com_CHK_Ui();
				return;
			}
    }  
    
    m=Case_num(COM0_BUF);                                              //����ͷ�Ƚ� 
    if(m==0xffff)   return;
    memset(Rx_Com,0x00,RX_CMD_LEN);
    switch(m)                                                          //
     {
      case UART_ICMD_CHK:                               break;         //���ղ�ѯ��Ϣ����ID
      case UART_ICMD_SETST:      Com_Set_ST();          break;         //���õ���������
      case UART_ICMD_MEA:        Com_Power_Test_Pr();   break;         //��ʼ������������
      case UART_ICMD_PMTR:       Com_Set_Mea_Met_Tab(); break;         //���ñ�λ��Ӧ��ϵ
      case UART_ICMD_XUP:        Com_Set_Vref(UP_TYP);  break;         //������ѹ�й�����
      case UART_ICMD_XUS:        Com_Set_Vref(US_TYP);  break;         //������ѹ���ڹ���
      case UART_ICMD_XIS:        Com_Set_Vref(IS_TYP);  break;         //�����������ڹ���
			case UART_ICMD_XU:         Com_Set_Vref(U_TYP);   break;         //������ѹ
			case UART_ICMD_XI:	       Com_Set_Vref(I_TYP);   break;         //��������
			case UART_ICMD_XOU:        Com_Set_Vref(OU_TYP);   break;         //������ѹ���
			case UART_ICMD_XOI:        Com_Set_Vref(OI_TYP);   break;         //�����������
      case UART_ICMD_TYPE:       Com_Set_Wire_Type();   break;
      case UART_ICMD_VER:        Com_Set_Read_Ver();    break;         //��汾��
      case UART_ICMD_SOLID:      Set_SOLID();           break;         //�̻�����
      case UART_ICMD_BOOT:       BOOT_INT_DIS();        break;         //������������
      case UART_ICMD_CRT_A:      Com_RCV_CRT(Phase_A);  break;         //����A�������Чֵ
      case UART_ICMD_CRT_B:      Com_RCV_CRT(Phase_B);  break;         //����B�������Чֵ
      case UART_ICMD_CRT_C:	     Com_RCV_CRT(Phase_C);  break;         //����C�������Чֵ
						case UART_ICMD_CRT_S:      Com_RCV_CRT(Phase_A);  break;         //���յ��������Чֵ
      default:                                          break;                                               
     }
     	    	
}         
/****************************************************************************
* ����COM0���ͻ�����
* ���ʱ����
* 2������ͼ��ʱ�� ��û�з��� ������ڷ��ͱ�־ ��ֹ�жϲ�������ڷ��ͱ�־
****************************************************************************/
void Proc_COM0_OBuf(void)
{
    u16 t;
    if(!COM0_OCoun)                    //�ж��Ƿ�����Ҫ���͵�����
     return;
    t=((u16)Timer_1ms-COM0_STimer);    // 	
    if(t<COM0_TIME)
     return;
    if(!COM0_InSend)                   //�ж��Ƿ��ڷ���״̬
     {
      if(COM0_OHead!=COM0_OTail)       //�ж��Ƿ�������Ҫ����
       {	
        if(UARTBusy(UART0))            //���UART�Ƿ����ڷ�������
         return;                       //������Ҫ���� ���ͻ���������
        UARTIntDisable(UART0,UART_INT_TX); //�����η����ж� ���ⷢ��ʱ�����ж�
        t=Send_One_Uart_Frm(COM0_BUF);
        UARTIntEnable(UART0,UART_INT_TX);  //����д����ϴ򿪷����ж� �ȴ��ж�
        if(t==BUF_FIFO_NBLK)             //��������һ֡ �ж��Ƿ��к�������
         return;                         //�к������� �˳� �ȴ������´�ѭ�� ����	
       }
      else 
       {	
        COM0_InSend=0;                 //������ڷ��ͱ�־
        COM0_OCoun=0;                  //����������������
       }  
      COM0_STimer=(u16)Timer_1ms;      //������ʱ 	    
     }   		
    else if(t>COM0_OVTM)
     {	
      COM0_InSend=0;                   //������ڷ��ͱ�־
      COM0_STimer=(u16)Timer_1ms;      //������ʱ 	    
     }	 
}           
           