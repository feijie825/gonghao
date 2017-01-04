/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.c
;* Author             : ������
;* ��ʾ����
;* û�б�׼��������   ��ʾ no bEP
;* û�б�׼ʱ������   ��ʾ no bCP
;* û�б����ʱ������ ��ʾ no cLP
;* ʱ������û���ȶ�   ��ʾ no Stb
;* ��ѹ�������       ��ʾ dL   End
;* ʱ��Ƶ��           ��ʾ Fxxxxxxx
;* �ռ�ʱ���         ��ʾ dxxxxxxx
;* ��������           ��ʾ Lxxxxxxx
;* �յ���բ����       ��ʾ H  R  H2
;* �յ�Ͷ������       ��ʾ 5  R  Sd
;* ������բ           OPEN      01
;* �̵�������         ERR       01
** 
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "string.h"
#include "stdlib.h"
#include "Disp.h"
#include "define.h"
#include "vari.h"
/*****************************************************************************
* 7279��λ ��ʼ��
*****************************************************************************/
void Reset_HD7279(void)
{
    SSIDataLen(SSI0,SSI_CR0_DSS_8);	      //�������ݳ���Ϊ8bit
    DISP_RST_EN;
    SysCtlDelay(625);                     //ÿ������ʱ160ns  ��ʱ100us
    DISP_RST_DN;
    SysCtlDelay(6250);                    //ÿ������ʱ160ns  ��ʱ1mS
    SSIDataPut(SSI0,LED_RESET);           //������  
    SysCtlDelay(62500);                   //ÿ������ʱ160ns  ��ʱ10mS
//    Disp_Data(Disp_Code_Mode);            //��λ����ʾ ����ʽ0����(��ԭ�������뷽ʽ��ͬ     
}
/*****************************************************************************
* ��ʽ3����ʱ,����ת����
*****************************************************************************/
const u8 Disp_Code_Tab[]=
{
  // 0     1    2    3   4     5   6    7     8   9
    0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,
  // A    B     C    D   E     F   g    H    L     R 
    0x77,0x7F,0x4E,0x7E,0x4F,0x47,0x7B,0x37,0x0E,0x77, 
  // -   �ո�   b    d   U    t    n    o    P    N
    0x01,0x00,0x1F,0x3D,0x3e,0x0F,0x15,0x1d,0x67,0x76,
  // c   8.
    0x0d,0xFF        
};
/*****************************************************************************
* ˢ����ʾ������
* ˢ��������
* Disp_Mode ����ˢ�·�ʽ
* Disp_Buf[0] ��Ӧ �����������(���λ) Disp_Buf[7] ��Ӧ �����������(���λ)
*****************************************************************************/
void Disp_Data(u8 Disp_Mode)
{
    u8 m;
    u16 t;
    if(Disp_En_Timer)                   //��ʱδ��
     return;	 
    SSIDataLen(SSI0,SSI_CR0_DSS_16);	  //�������ݳ���Ϊ16bit
    for(m=0;m<8;m++)
     {
      t=((Disp_Mode|m)<<8);
      if(Disp_Mode==LED_SEND_DATA_CODE2)
       {
        if((Disp_Buf[m]&0x7f)<sizeof(Disp_Code_Tab))
         t|=Disp_Code_Tab[Disp_Buf[m]&0x7f];
        else
         t|=Disp_Code_Tab[DISP_BLANK];
        t|=(Disp_Buf[m]&0x80);         //�ж��Ƿ���С����
       }
      else    
       t|=Disp_Buf[m];
      SSIDataPut(SSI0,t);
      SysCtlDelay(250);                //��ʱ40uS
     }
    Disp_Timer=Timer_8ms;              //ˢ����ʾ��ʱ��
    Disp_En_Timer=DISP_EN_TIME;        //��ʾʹ�ܶ�ʱ
}
/*****************************************************************************
* ���µ�ǰУ��Ȧ����ʾ������
* ֻ��ʾ����λ ��ʾ����ʾ�������
*****************************************************************************/
void Update_N_Buf(void)
{
    u8 m=CURRENT_N;                        //��ǰȦ��
    CURRENT_N_ASC[1]=(m%10)|'0';           //Ȧ����λ
    m/=10;                                 //Ȧ����λ
    if(m>9)                                //�жϵ�ǰȦ���Ƿ񳬹�100
     m%=10;                                //
    CURRENT_N_ASC[0]=(m|'0');              //Ȧ����λ
    Disp_Buf[0]=(CURRENT_N_ASC[0]&0x0f);   //Ȧ����λ
    Disp_Buf[1]=(CURRENT_N_ASC[1]&0x0f);   //Ȧ�����λ
}
/*****************************************************************************
* ���±�λ����ʾ������
*****************************************************************************/
void Update_Mtr_Num(void)
{					   
      Disp_Buf[5]=Mtr_Numb_Str[0];         //��λ
      Disp_Buf[6]=Mtr_Numb_Str[1];         //��λ
      Disp_Buf[7]=Mtr_Numb_Str[2];         //��λ
}
/*****************************************************************************
* �����ַ������ݵ���ʾ������
* ���: Len    Ҫ�������ݵĳ���
* �˿�: Offset ��ʾ��������ʼ��ַ ��ʾ��λ(��������λ)
* �˿ڣ�*Ptr   Ҫ�������ݵ�ָ�� Ҫ��ʾ������  
*****************************************************************************/
void Copy_Str_To_DSBUF(u8 Len,u8 Offset,u8 *Ptr)
{
    u8 m,n=Offset,d;
    if(GZ_FLAG||NZTZ_FLAG)              //�Ƿ��ڹ��Ϻ���բ״̬
     return;                            //�˳� ������ʾ��������	 
    for(m=0;(m<Len&&n<8);m++,n++)       //ѭ��������ʾ
     {
      if(Ptr[m]!='.')                   //�ж��Ƿ�ΪС����
       {                                
        if((Ptr[m]==' ')||              //�Ƿ�Ϊ�ո�
           (Ptr[m]=='\0')||             //�ַ�������  
           (Ptr[m]=='+'))               //�Ƿ�Ϊ'+'
         d=DISP_BLANK;                   //����ʾ
        else if(Ptr[m]=='-')            //����
         d=DISP_MINUS;                   //'-'
        else if(Ptr[m]=='H')            //
         d=DISP_H;                      //��ʾ'H'
        else if(Ptr[m]=='L')
         d=DISP_L;                      //��ʾ'L'
        else                         
         d=Ptr[m]-'0';                  //ASC��Ϊ
        Disp_Buf[n]=d;
       }
      else                              //С���㴦��
       {                                
        if(n==Offset)                   
         {                              
          Disp_Buf[2]=0x80;             //��ʾС����
          n=3;                          
         }                              
        else                            
         {                              
          if(n>Offset)                  
           n--;                         //�ص��ϸ��ַ�
          else                          
           n=Offset;                    // 
          Disp_Buf[n]|=0x80;            //��ʾС����
         } 
       }
     }
} 
/*****************************************************************************
* ��ʾ����������
* ���: Len  �����ʾ����
* �˿�: Addr ��ʾ��ʼ��ַ ��ʾ��λ(��������λ) 1--8
* �˿ڣ�Data Ҫ��ʾ������  
* �˿�: Sts  0:��λ���� 1:��λ����ʾ
*****************************************************************************/
void Disp_Long_Data(u8 Len,u8 Addr,u32 Data,u8 Sts)
{
    u8 m;
    if(Addr==0)
     return;
    if(GZ_FLAG||NZTZ_FLAG)             //�Ƿ��ڹ��Ϻ���բ״̬
     return;                           //�˳� ������ʾ��������	 
    Addr--; 
    for(m=0;m<Len;m++)
     {
      Disp_Buf[Addr]=Data%10;          //ת����ʮ����BCD��
      Data/=10;                        //ȥ����λ
      if((Addr==0)||(Data==0))         
       break;
      Addr--;
     }
   for(;m<Len;m++)
    {
     if(Addr==0)
      break;
     Addr--; 
     if(Sts)
      Disp_Buf[Addr]=DISP_BLANK;        //����ʾ
     else
      Disp_Buf[Addr]=0;                 //��ʾ0
    } 
   Disp_Timer=(Timer_8ms-DISP_TIME);    //������ʾ
//   Disp_Data(Disp_Code_Mode);          //����ʽ0����(��ԭ�������뷽ʽ��ͬ)      
}
/*****************************************************************************
* ��ʾ��ʱ����
*****************************************************************************/
void Disp_Time_Pr(void)
{
    if((u8)(Timer_8ms-Disp_Timer)<DISP_TIME)//�ж���ʾ��ʱ�Ƿ�abs(Timer_8ms-Disp_Timer)
     return;                            //���� �˳�
    Disp_Data(Disp_Code_Mode);          //��λ����ʾ ����ʽ0����(��ԭ�������뷽ʽ��ͬ     
}            
