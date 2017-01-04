/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CAN.C
;* Author             : ������
;* CAN�������ݴ������� 
;* ������CAN�ӿڵ�LM3S ϵ�д����� ����CANʱ�Ӻ�CPUʱ��֮��Ƶ�ʵĲ�һ��,��ɷ���CAN�Ĵ���ʱ
;* Ҫ���Ӷ�����ʱ CPU ʱ��Ƶ�ʸ���CANʱ��Ƶ��
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
#include "string.h"
#include "define.h"
#include "vari.h"
#include "CS5460A.h"
#include "����.h"
/****************************************************************************
* CAN������֡����
****************************************************************************/
const CAN_MSG CAN_TX_SMSG=
{   
    0x00030050,    //������ EXD_ID=1 TX_INT_EN=1 
    0x18000002,    //�ٲ��� EXD=1 DIR=1 DEV=2
    0x0000,        //DA1
    0x0000,        //DA2
    0x0000,        //DA3
    0x0000,        //DA4
}; 
/*****************************************************************************
* �ӻ�CAN���ݷ��ͳ�ʱ����
*****************************************************************************/
void Proc_CAN_OvTm(void)
{
    if(CAN_SMSG_TX_STS)                              //������֡�Ƿ��ڷ���״̬
     {
      if((u16)(Timer_1ms-CAN_STX_OVTimer)>=CAN_TX_OVTM)//�����ݷ��ͳ�ʱ����
       {
        CAN_SMSG_TX_STS=COM_TX_IDLE;                   //��������ݷ��ͱ�־      ��Ϊ���Ϳ���״̬
       }		
     }		
}
/*****************************************************************************
* ����������ѯ��Ϣ
*****************************************************************************/
void Proc_Mst_Check(void)
{
/*
	if(Echo_Sts==MST_CHK_RCVD)
	 {                                            //�յ�������ѯ֡
      CAN_MSG CAN_ECHO_MSG;
      memcpy(&CAN_ECHO_MSG,
             &CAN_MSG_SET_TAB[SLV_CHK_ECHO-1],
             8);                                //�����������ٲ���
      CAN_ECHO_MSG.ID.BIT.NUM = Mtr_Numb_ID;    //��λ��
      CAN_Tx_Msg(CAN0,&CAN_ECHO_MSG,MSG_OBJ_TYPE_TX);
      Echo_Sts= SLV_ECHO_SEND;                  //�Ѿ����뷢�� ����MSG RAM
      CAN_RX_OVTimer=(u16)Timer_1ms;            //��ʼ��CAN��ʱ��ʱ��
	 }
*/
}
/********************************************************
* IO�ڻ�ַ����
********************************************************/
const u32 PORT_BASE_ADDR_TAB[]=
{
    GPIO_PORTA_BASE,            //GPIOA ��ַ
    GPIO_PORTB_BASE,            //GPIOB ��ַ
    GPIO_PORTC_BASE,            //GPIOC ��ַ
    GPIO_PORTD_BASE,            //GPIOD ��ַ
    GPIO_PORTE_BASE,            //GPIOE ��ַ
    GPIO_PORTF_BASE,            //GPIOF ��ַ
    GPIO_PORTG_BASE,            //GPIOG ��ַ
    GPIO_PORTH_BASE,            //GPIOH ��ַ
};
/********************************************************
* �������ݵ�CAN������
* �˿�: CMD ������
* �˿�: Len ���ݳ���
* �˿�: Ptr ����������ָ��
********************************************************/
void Send_Data(u16 CMD,u8 Len,u8 *Ptr)
{
    u8 m;
    if(CAN_ERR)                           //���ߴ���
     return;                              //�˳�
    if(Len>8)                             //�жϳ����Ƿ񳬹�������ݳ���
     Len=8;                               
    memcpy(CAN_MSG_OPtr,                  //CAN����ָ��
           &CAN_TX_SMSG,                  //Ĭ��֡
           16);                           
    CAN_MSG_OPtr->ID.BIT.NUM=Mtr_Numb_ID; //��λ��   �ٲ���
    CAN_MSG_OPtr->ID.BIT.CMD=CMD;         //����     �ٲ���
    if(Len!=0)
     {
      CAN_MSG_OPtr->CTL.BIT.LEN=Len;      //���ݳ��� ������
      for(m=0;m<Len;m++)                  //������Ч����
       {
       	CAN_MSG_OPtr->Data.BYTE[m]=*Ptr;
       	Ptr++;
       }	 
     }         
    SDATA_MSG_OHead_ADD_ONE();            //����ָ���1����           
}                                    
/********************************************************
* �ܿ�������������
********************************************************/
void Set_Master_Start(void)
{
    MASTER_START=1;                             	//�ܿ������Ѿ�������־
}
/******************************************************
* ������̨��������
******************************************************/
void Set_TS(void)
{
    if(CAN_MSG_IPtr->Data.BYTE[0]=='3')          //�Ƿ�����Ϊ����̨
     {	         
      if(SINGLE_OR_THREE!=THREE)                 //���������÷����ı�
       {
        SINGLE_OR_THREE=THREE;  
       }                               
     } 
    else if(CAN_MSG_IPtr->Data.BYTE[0]=='1')
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
u8 Get_Mea_Met(u8 Met)
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
* XXXY��XXY  XXX��XX Ҫ�����ı�λ
* Y Ҫ�������� 0:ֹͣ���� 1:A�� 2:B�� 3:C��
********************************************************/
void Power_Test_Pr(void)
{
    u8 ID=0,c;
    ID=CAN_MSG_IPtr->Data.BYTE[0];             //ȡ��������λ�� 
    c=CAN_MSG_IPtr->Data.BYTE[1];              //ȡ����������
    if(c>3)                                    //�жϿ������Ƿ����
     return;
    Current_Mea_Phase=c;                       //�洢�������
    CNCL_CHNL_Sel1;                             //�ͷ����в���ͨ��
    CNCL_CHNL_Sel2;
    Sel_Phase_NO;                              //�������κ����
    P4_Sel1;
    P4_Sel2;
    P4_Sel3;
    if((Get_Mea_Met(ID)==0)||                  //Ҫ�����ı�λ�Ų��ڹ���ģ��Ĳ�����Χ��
    	  (ID==0)||                               //�����ı�λ����0���˳����Ĳ���
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
void Set_Mea_Met_Tab(void)
{
  u8 chnl,Met;
  chnl=CAN_MSG_IPtr->Data.BYTE[0];             //ȡ��Ҫ���õ�ͨ���� 
  Met=CAN_MSG_IPtr->Data.BYTE[1];              //ȡ����������
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
********************************************************/
void Set_Vref(u8 type)
{
	 float f;
	 if(type>2)
	 	return;                                       //����ֵ����ֻ����0-2֮�������
	 memcpy(&f,
	        &CAN_MSG_IPtr->Data.BYTE,
	        4);                                     //ȡ����������ϵ��
  if(type==UP_TYP)
  {
    f*=XIUZ.U_PXZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>2))   
     return;
    XIUZ.U_PXZ=f;
  }
  else if(type==US_TYP)
  {
    f*=XIUZ.U_SXZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>2))   
     return;
    XIUZ.U_SXZ=f;
  }
  else if(type==IS_TYP)
  {
    f*=XIUZ.I_SXZ;                      //��ǰϵ���ٳ�Ҫ������ϵ��
    if((f<0.2)||(f>2))   
     return;
    XIUZ.I_SXZ=f;
  }  
}
/******************************************************
* ���汾��,ֻ�е�һ��λ���ذ汾��
******************************************************/
void Set_Read_Ver(void)             
{
    if(Mtr_Numb_ID==1)                       //�Ƿ�Ϊ��1��λ
     {
      u8 n,d;
      for(n=0;n<10;)
       {
        d=VER_TAB[n];
        if(d=='\0')
         break;
        TEMP_STR[n]=d; 
        n++;
       }
      if(n==0||n==10)                          //���ȴ���
       return;	    
      Send_Data(CAN_OCMD_VER,                  //����
                n,
                TEMP_STR);
     }           
}
/******************************************************
* �̻�����ֵ���ݼ�ͨ����Ӧ��ϵ���� 
******************************************************/
void Set_SOLID(void)
{
    Solid_Save_Tab();             //�̻���׼����� 
}
/******************************************************
* ���ý��߷�ʽ
******************************************************/
void Set_Wire_Type(void)
{
  u8 type;
  type=CAN_MSG_IPtr->Data.BYTE[0];           //ȡ�����߷�ʽ������
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
/*****************************************************************************
* ����CAN�����ݽ���ָ�����
*****************************************************************************/
void Proc_SDATA_IBUF(void)
{
    if(CAN_SDATA_MSG_IHead==CAN_SDATA_MSG_ITail)
     return;                                                       //û���յ��������˳�
    CAN_RX_OVTimer=(u16)Timer_1ms;                                 //��ʼ��CAN��ʱ��ʱ��
    MASTER_START=1;                                                //����������
    switch(CAN_MSG_IPtr->ID.BIT.CMD)                               //����������ת����Ӧλ�ô���
     {
      case CAN_ICMD_CHK:    Set_Master_Start();            break;  //�ܿ�������������
      case CAN_ICMD_SETST:  Set_TS();                      break;  //���õ���������
      case CAN_ICMD_PMTR:   Set_Mea_Met_Tab();             break;  //���ñ�λ��Ӧ��ϵ
      case CAN_ICMD_MEA:    Power_Test_Pr();               break;  //��ʼ������������
      case CAN_ICMD_XUP:    Set_Vref(UP_TYP);              break;  //������ѹ�й�����
      case CAN_ICMD_XUS:    Set_Vref(US_TYP);              break;  //������ѹ���ڹ���
      case CAN_ICMD_XIS:    Set_Vref(IS_TYP);              break;  //�����������ڹ���
      case CAN_ICMD_VER:    Set_Read_Ver();                break;  //��汾��
      case CAN_ICMD_SOLID:  Set_SOLID();                   break;  //�̻�����
      case CAN_ICMD_BOOT:                                  break;  //������������
      case CAN_ICMD_TYPE:   Set_Wire_Type();               break;  //����ѡ��
      default:break;     
     }
    CAN_SDATA_MSG_ITail++;
    if(CAN_SDATA_MSG_ITail>=CAN_SDILEN)
     {
      CAN_SDATA_MSG_ITail=0;
      CAN_MSG_IPtr=&CAN_SDATA_MSG_IBUF[0];//��ʼ�����ձ��Ĵ���ָ��
     }                                    
    else                                  
     CAN_MSG_IPtr+=1;                     //ָ�� 

}
/*****************************************************************************
* CAN�����ݷ���ָ�����ͷָ���1����
*****************************************************************************/
void SDATA_MSG_OHead_ADD_ONE(void)
{
    CAN_SDATA_MSG_OHead++;
    if(CAN_SDATA_MSG_OHead>=CAN_SDOLEN)
     {
      CAN_SDATA_MSG_OHead=0;              //����ָ��
      CAN_MSG_OPtr=&CAN_SDATA_MSG_OBUF[0];//��ʼ�����ͱ��Ĵ���ָ��
     }
    else
     CAN_MSG_OPtr+=1;
}           
/*****************************************************************************
* ����CAN�����ݷ���ָ�����
*****************************************************************************/
void Proc_SDATA_OBUF(void)
{
    if(CAN_SDATA_MSG_OHead==CAN_SDATA_MSG_OTail)
     return;                                  //û�ж�����Ҫ�����˳�
    if(CAN_SMSG_TX_STS != COM_TX_IDLE )       //CAN�������Ƿ��ڷ���״̬
     return;
    if(!MASTER_START)                         //�ܿ������Ƿ�����
     return;	
    if(CAN_SEND_DELAY)                        //Ϊ��ֹ���ɾ��� ���ӱ�λ��ʱ
     {
      CAN_SEND_DELAY--;
      return;
     }	
    CAN_SEND_DELAY=CAN_SEND_DTIME;            // 	
    CAN_SMSG_TX_STS = SLV_SMSG_TX_IS;
    CAN_STX_OVTimer=(u16)Timer_1ms;           //���÷��Ͷ�ʱ
    CAN_Tx_Msg(CAN0,&CAN_SDATA_MSG_OBUF[CAN_SDATA_MSG_OTail],MSG_OBJ_TYPE_TX);  //���Ͷ�֡
    CAN_SDATA_MSG_OTail++;                    //ָ���1 ָ����һ֡ 
    if(CAN_SDATA_MSG_OTail>=CAN_SDOLEN)       //�ж��Ƿ񳬳�ѭ��   
     CAN_SDATA_MSG_OTail=0;                   //��ͷ��ʼѭ��       
}
/*****************************************************************************
* ����CAN����״̬
* ������ʱ
*****************************************************************************/
void Proc_CAN_STS(void)        
{
    if((CAN_STS.BYTE&(CAN_STS_BOFF|CAN_STS_EWARN|CAN_STS_EPASS))||
      ((u16)(Timer_1ms-CAN_RX_OVTimer)>CAN_RX_OVTM)) 
     {
      CAN_RX_OVTimer=(u16)Timer_1ms;      //��ʱ��ʱ	
      CAN_STS.BYTE=0;                     //�������
      CAN_ERR=1;                          //CAN����״̬��־
      CAN_SMSG_TX_STS=COM_TX_IDLE;        //��������ݷ��ͱ�־      ��Ϊ���Ϳ���״̬
      CAN_LMSG_TX_STS=COM_TX_IDLE;        //���CAN������֡����״̬ ��Ϊ���Ϳ���״̬
      CAN_LDATA_TX_STS = SLV_LDATA_TX_NO; //��������ݷ���״̬      ��Ϊ���Ϳ���״̬         
      Init_CAN();                         //���³�ʼ������
     } 	
}