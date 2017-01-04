/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
* File Name          : Init.c
* Author             : ������
* ��ʼ��
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "stdio.h"
#include "ENG_ERR.h"   //���ܴ��������ļ�
#include "define.h"
#include "vari.h"
/*****************************************************************************
* �汾��
*****************************************************************************/
const u8 VER_TAB[]={"V1.0.4\r"};

/*****************************************************************************
* ����׼����ű������״̬
* ����Current_Save_Tab��ǰʹ�õı�λ�ű���ID
* Save_Tab_Sts ��׼�����״̬����״̬�б�
*****************************************************************************/
void Check_SMtr_Tab_Sts(void)
{
    u16 m,n;
    u16 Len;
    const u8 *Str;
    u8    c;
    Len=sizeof(SAVE_S);
    Len+=0x03;
    Len&=0xFFFC;			                              //4�ֽڶ���
    Current_Save_Tab=0;				                      //Ĭ���ޱ���ֵ
    for(n=0;n<SAVE_TAB_NUMB;n++)
    {
      Str=(u8 *)&Save_Tab;
   
      Str+=n*Len;
      c=*Str;
      if(c==YES)                                 //������ֵ
      {
        memcpy(&XIUZ,Str,sizeof(SAVE_S));        //��������ֵ
        Current_Save_Tab=(n+1);		                //��λ�ű�־ΪDATA_YES ������������Ч
        Save_Tab_Sts[n]=VALIDE;                  
      }                                         
      else if(c!=0xff)                           
       Save_Tab_Sts[n]=NOT_BLACK;	               //��λ�ű�־��Ϊ0xff ֱ���ò��ձ�־
      else                                       
      {                                         
        Save_Tab_Sts[n]=BLANK;                   //�����ñ�λ�ű���Ϊ�� ����д���λ������
        for(m=0;m<(Len/4);m++)
        {                                       //�������ֽ�
          if(*((u32 *)Str)!=0xFFFFFFFF)
          {
            Save_Tab_Sts[n]=NOT_BLACK;
            break;
          }
          Str+=4; 
        }
      }
    }
    if(Current_Save_Tab)                         //��ǰ��������������
    {                                           //�ж�����ֵ�����Ƿ����
      if(XIUZ.U_PXZ<0.2)                         //����ֵ�Ƿ�̫С
       XIUZ.U_PXZ=1.000;                         //Ĭ��Ϊ1.0
      if(XIUZ.U_SXZ<0.2)                         //����ֵ�Ƿ�̫С
       XIUZ.U_SXZ=1.000;                         //Ĭ��Ϊ1.0
      if(XIUZ.I_SXZ<0.2)                         //����ֵ�Ƿ�̫С
       XIUZ.I_SXZ=1.000;                         //Ĭ��Ϊ1.0
			if(XIUZ.U_XZ<0.2)                         //����ֵ�Ƿ�̫С
       XIUZ.U_XZ=1.000;                         //Ĭ��Ϊ1.0
      if(XIUZ.I_XZ<0.2)                         //����ֵ�Ƿ�̫С
       XIUZ.I_XZ=1.000;                         //Ĭ��Ϊ1.0
      return;                                    //�б���ֵ�˳�
    } 
    XIUZ.Flag=YES;                               //��־
    XIUZ.CH0_MTR[0]=1;                           //ͨ��1Ĭ�϶�Ӧ1��λ
    XIUZ.CH0_MTR[1]=2;                           //ͨ��2Ĭ�϶�Ӧ2��λ
    XIUZ.CH0_MTR[2]=3;                           //ͨ��3Ĭ�϶�Ӧ3��λ
    XIUZ.U_PXZ=1.000;                            //Ĭ������ֵ��ѹ�й���������ֵ
    XIUZ.U_SXZ=1.000;                            //Ĭ������ֵ��ѹ���ڹ�������ֵ	
    XIUZ.I_SXZ=1.000;                            //Ĭ������ֵ�������ڹ�������ֵ	
		XIUZ.U_XZ=1.000;                            //Ĭ������ֵ��ѹ����ֵ	
    XIUZ.I_XZ=1.000;                            //Ĭ������ֵ��������ֵ
    for(m=0;m<SAVE_TAB_NUMB;m++)                 //�鿴�Ƿ��п�λ��
    {
      if(Save_Tab_Sts[m]==BLANK)                 //�Ƿ��(��д)
       return;                                   //�пյı�λ�ű����˳�
    }                                           
    FlashErase(SAVE_BASE);                       //û�б�λֵ ��û�пձ�λλ�� ������λBANK
}
//�̻�����ֵ ��RAM->FLASH
void Solid_Save_Tab(void)
{
    u32 *Ptr,Addr,Data=0;
    u16 Len;
    u8  n;
    Len=sizeof(SAVE_S);                          //����ֵ����
    Len+=0x03;
    Len&=0xFFFC;                                 //4�ֽڶ���
    for(n=0;n<SAVE_TAB_NUMB;n++)
     {
      if(Save_Tab_Sts[n]==BLANK)
       break;                                    //�пյ�������������ѭ��
     }
    if(n==SAVE_TAB_NUMB)                         //�ж��Ƿ�û�пձ���
     {
      FlashErase(SAVE_BASE);                     //û�б�λֵ ��û�пձ�λλ�� ������λBANK
      for(n=0;n<SAVE_TAB_NUMB;n++)
       Save_Tab_Sts[n]=BLANK;
      Current_Save_Tab=0;
      n=0;
     } 
    if(Current_Save_Tab!=0)                        
     {                                             //��ǰ������ֵ 
      Addr=(u32)(&Save_Tab);    
      Addr+=(Current_Save_Tab-1)*Len;              //��ǰ����ֵ��ַ(��Ҫ����) 
      FlashProgram(&Data,Addr,4);                  //д��0 ��ʾ��λ����Ч
      Save_Tab_Sts[Current_Save_Tab-1]=NOT_BLACK;  //not blank
     }
    Ptr=(u32 *)&XIUZ;
    Addr=(u32)&Save_Tab;
    Addr+=n*Len;
    Save_Tab_Sts[n]=VALIDE;                        //������Ч
    Current_Save_Tab=n+1;                          //���µ�ǰ�������ID��
    FlashProgram(Ptr,Addr,Len);                    //��� ��д��ǰֵ
}
//����λ��
void Read_Mtr_Numb(void)
{
    u8 m,n;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);  //ʹ�ܶ˿�Gʱ��
    SysCtlDelay(500);                             //��ʱ0.08ms
//��ʼ��GPIOG��                             
    GPIODirModeSet(GPIOG,
                   BW,
                   GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIOG,                       //�˿� ���ùܽ�����    
                     BW,                          //�ܽ�                 
                     GPIO_STRENGTH_8MA,           //��������             
                     GPIO_PIN_TYPE_STD_WPD);      //����            
    SysCtlDelay(500);                             //��ʱ0.08ms
    for(;;)                                 
     {                                      
      m=GPIOPinRead(GPIOG,BW);                    //����λ��
      SysCtlDelay(5000);                          //��ʱ0.5ms
      n=GPIOPinRead(GPIOG,BW);                    //����λ��  
      if(m==n)                                    //�ȶ�
       break;	   
     }
    if(m!=0xFF)
     m++;	                                 //
    Mtr_Numb_ID=m;                         //
    Mtr_Numb_Str[2]=m%10;                  //��λ
    m/=10;                               
    Mtr_Numb_Str[1]=m%10;                  //��λ
    m/=10;                               
    Mtr_Numb_Str[0]=m%10;                  //��λ
    if(m==0)                             
    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOG);//���ܶ˿�Gʱ��
}
/****************************************************************************
* �ַ������� ��Str1 ������Str2��
* Str1 ���������ַ���
* Str2 ��д����ַ���
* Len Str2 ��󳤶�
* �������ַ������Ȳ��� ǰ�����ո� ' ' Ϊ����ʾ�����
****************************************************************************/
void Fill_Space_StrCpy(u8 *Str1,u8 *Str2,u8 Len)
{
    u8 m,n=0;
    if(Len<2)                        //����̫��
     {	
      *Str2='\0';	
      return;
     } 
    Len--;                           //�ַ������������� '\0' 
    for(n=0;n<Len;n++)               //ѭ���޳�Str1�п�ͷ�Ŀ��ַ�' '
     {
      if(*Str1!=' ')
       break;	
      Str1++;
     }
    if(n==Len)                       //�ж��Ƿ�û����Ч����
     {
      for(m=0;m<Len;m++)
       {
        *Str2=' ';                   //���ո�
        Str2++;
       }
      *Str2='\0';  	 	
      return; 
     }
    else 	
     m=n=strlen((const char*)Str1);  //����ַ���Str1 ʵ�ʳ���
    if(m<Len)
     {
      for(;m<Len;m++)
       {
        *Str2=' ';                   //���ո�
        Str2++;
       }
      Len=n;                         //����ʵ�ʳ���  	
     }
    if(Str1[Len-1]=='.')             //�ж�Ҫ���������һ���ַ��Ƿ�Ϊ'.' 
     {
      *Str2=' ';                     //�����һ���ո�
      Str2++;
      Len--;	
     }
    for(m=0;m<Len;m++)
     {
      *Str2=*Str1;
      Str1++;
      Str2++;
     } 	
    *Str2='\0';  	 	
}

/*****************************************************************************
* RAM�ͱ�����ʼ��
*****************************************************************************/
void Init_Ram(void)
{   
    Read_Mtr_Numb();                        //������
	   Check_SMtr_Tab_Sts();                   //����׼������
    SINGLE_OR_THREE=SINGLE;                 //Ĭ�ϵ���̨
    CAN_STS.BYTE =0;                        //�������״̬��־
    CAN_MSG_IPtr=&CAN_SDATA_MSG_IBUF[0];    //��ʼ�����ձ��Ĵ���ָ��
    CAN_MSG_OPtr=&CAN_SDATA_MSG_OBUF[0];    //��ʼ�����ͱ��Ĵ���ָ��
    CAN_RX_OVTimer=(u16)Timer_1ms;          //��ʼ��CAN��ʱ��ʱ��
    CAN_ERR=0;                              //
    CAN_SEND_DTIME=(Mtr_Numb_ID%10);        //10��Ϊһ��
    CAN_SEND_DELAY=CAN_SEND_DTIME;          //��ʱ �ܿ�
	   ADC_Data.Trig=1;                        //�Ѿ�������־
    CS5460A_Vref=2.5;                       //CS5460A��׼��ѹ
    ADC0_Vref=3.0;                          //LM3S2139Ƭ��ADC��׼��ѹ

    Current_I_Value=5.0;                    //��ǰ����ֵ
    memset(Rx_Com,0x00,RX_CMD_LEN);
    CHK_OneTime_Flag=0x00;  
    Current_Wire_Type=WIRE_P1;
}
