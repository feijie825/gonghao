/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : ENG_ERR.c
;* Author             : ������
;* �������崦��������
;* ���幤��ģʽ �� vari.h ��MODE ����
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "stdio.h"	      //������ͷ�ļ�
#include "string.h"
#include "define.h"
#include "vari.h"
#include "ENG_ERR.h"
/********************************************************
* ���PLLʱ�� 200ms ���һ�� �Ĵ���
* ʵʱ��� ��׼����ʱ(131msһ��)
* Ĭ�ϲ�ʹ�� RCC2
* �б�׼����ʱ STD_CLK_Cnt �൱�� 131ms��ʱ��
* û�б�׼���� ��Ӱ����(�޷�ͨ����׼������PLL����)
* ���Ǳ�׼����Ƶ�ʳ���1000kʱ����������
* ��׼����Ƶ�ʸı�ʱ Ҫ�����Ƚ�ֵ ��if(t<76)�е�76
* ������ ��������ֵΪ131 PLL�쳣ʧ�� ����ֵԼΪ42
* �� CPU�ٶȱ���(8mHz) Timer_1ms �����ٶ�ΪPLL����ʱ�� 8/25=0.32(32%)
* ��׼�����ж�ʱ�䲻�� ��Ϊ131ms Timer_1ms ����ֵ���� ʱ��ʱ��Ϊ3.125ms
* ���ʧ����־
********************************************************/
void Check_PLL(void)
{
    if(((u8)(Timer_8ms-PLL_CHK_Timer))>=PLL_CHK_TIME)//�ж���ʾ��ʱ�Ƿ�
     {
      u32  ulRCC,ulRCC2;
      PLL_CHK_Timer=Timer_8ms;                    //��ʱ��������ʱ
      if(!(SYSCTL->RIS & SYSCTL_INT_PLL_LOCK))    //PLLʧ����־
       {
        PLL_ERR_Cnt=0;                            //	
        Init_Pll();                               //��ʼ�����໷	 
        return;	
       }
      ulRCC = SYSCTL->RCC;                        //��ȡRCC  ����ģʽʱ�����üĴ���
      ulRCC2 = SYSCTL->RCC2;                      //��ȡRCC2 ����ģʽʱ�����üĴ���2
      ulRCC&=((SYSCTL_RCC_ACG|                    //ʱ���ſ��� 
               SYSCTL_RCC_SYSDIV_M|               //ϵͳ��Ƶ��
               SYSCTL_RCC_USESYSDIV|              //ʹ�÷�Ƶ��
               SYSCTL_RCC_PWRDN|                  //������
               SYSCTL_RCC_BYPASS|                 //PLL��·��
               SYSCTL_RCC_XTAL_M|                 //������
               SYSCTL_RCC_OSCSRC_M|               //����ѡ����
               SYSCTL_RCC_IOSCDIS|                //�ڲ�����������
               SYSCTL_RCC_MOSCDIS));              //������������
      if((ulRCC2&SYSCTL_RCC2_USERCC2)||           //��ʹ��RCC2
         (ulRCC!=SysRCC_CFG))                     
       {   
        PLL_ERR_Cnt=0;                            //����������
        Init_Pll();                               //��ʼ�����໷	 
       }  
     }  
}
/*****************************************************************************
* ���Ĳ��Խ������,ͨ�� CAN���ߡ�
*****************************************************************************/
void Power_Mea_Resualt_Send(void)
{
   if(ADC_Start!='Y')
   {
     Rslt_Send_Timer=(u16)Timer_1ms;
     return;
   }  
   else
   {
     if((u16)(Timer_1ms-Rslt_Send_Timer)<RSLT_SEND_TIME)   //û�е�����ʱ�䣬����
       return;                                             //2���ӷ���һ��
     else
       Rslt_Send_Timer=(u16)Timer_1ms;
       TEMP_STR[0]=Current_Mea_MetNUM;            //������λ
       TEMP_STR[1]=U_Pr_Str[1];                   //
       TEMP_STR[2]=U_Pr_Str[0];                   //��ѹ�й�����
       TEMP_STR[3]=U_Pm_Str[1];                   //
       TEMP_STR[4]=U_Pm_Str[0];                   //��ѹ���ڹ���
       TEMP_STR[5]=I_Pm_Str[1];                   //
       TEMP_STR[6]=I_Pm_Str[0];                   //
       Send_Data(CAN_OCMD_MEA,                    //����
                7,
                TEMP_STR);
   }
}

//�ַ������� ��ճ�ַ�������С��20 ճ���ַ�������С��10 
void Rmstrcat (unsigned char *str1,unsigned char *str2)
{
     unsigned char Len=0;
     for(;;)
      {
       if(*str1=='\0')
        break;
       str1++;
       Len++;
       if(Len>40)
        return; 
      }
     for(Len=0;Len<15;Len++)
      {
       *str1=*str2;
       if(*str2=='\0')
        return;
       str1++;
       str2++;  
      } 
     if(Len==15)
      *str1='\0'; 
}


/*****************************************************************************
* ���Ĳ��Խ����ѯʱ����,ͨ��RS485���ߡ�
*****************************************************************************/
void Power_Mea_Resualt_Com_Send(void)
{
   if(ADC_Start=='Y')                                         //
   {
     if(CHK_OneTime_Flag=='Y')
     {
       CHK_OneTime_Flag=0x00;
       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
       UART_TEMP_STR[0]='C';
       UART_TEMP_STR[1]='W';
       UART_TEMP_STR[2]=':';
       UART_TEMP_STR[3]=(Current_Mea_MetNUM/10)|'0';
       UART_TEMP_STR[4]=(Current_Mea_MetNUM%10)|'0';
       UART_TEMP_STR[5]=',';
       Rmstrcat(UART_TEMP_STR,U_Pr_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,U_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Pm_Str_Com);
			  Rmstrcat(UART_TEMP_STR,"\r");
//       SendStr(COM0_BUF,                  //����
//              UART_TEMP_STR,
///              0);

//       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
			 Rmstrcat(UART_TEMP_STR,U_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Str_Com);
       Rmstrcat(UART_TEMP_STR,"\r");
     
       SendStr(COM0_BUF,                  //����
              UART_TEMP_STR,
              0);
       COM0_OCoun=0x01;
     }  
   } 
}
/*****************************************************************************
* ���Ĳ��Խ����ѯʱ����,ͨ��RS485���ߡ�
*****************************************************************************/
void Ui_Mea_Resualt_Com_Send(void)
{
   if(ADC_Start=='Y')                                         //
   {
     if(CHK_ONETIME_FLAG1=='Y')
     {
       CHK_ONETIME_FLAG1=0x00;
       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
       UART_TEMP_STR[0]='Q';
       UART_TEMP_STR[1]='W';
       UART_TEMP_STR[2]=':';
       UART_TEMP_STR[3]=(Current_Mea_MetNUM/10)|'0';
       UART_TEMP_STR[4]=(Current_Mea_MetNUM%10)|'0';
       UART_TEMP_STR[5]=',';
       Rmstrcat(UART_TEMP_STR,U_Pr_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,U_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Pm_Str_Com);
			 Rmstrcat(UART_TEMP_STR,",");
//       SendStr(COM0_BUF,                  //����
//              UART_TEMP_STR,
///              0);

//       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
			 Rmstrcat(UART_TEMP_STR,U_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Str_Com);
       Rmstrcat(UART_TEMP_STR,"\r");
     
       SendStr(COM0_BUF,                  //����
              UART_TEMP_STR,
              UART_TEMP_STR_LEN);
       COM0_OCoun=0x01;
     }  
   } 
}


/*
void Power_Mea_Resualt_Com_Send(void)
{
   if(ADC_Start!='Y')                                         //
   {
     Rslt_Com_Send_Timer=(u16)Timer_1ms;
     return;
   }
   else
   {
     if((u16)(Timer_1ms-Rslt_Com_Send_Timer)<RSLT_SEND_TIME)   //û�е�����ʱ�䣬����
       return;                                             //2���ӷ���һ��
     else
       Rslt_Com_Send_Timer=(u16)Timer_1ms;
       memset(UART_TEMP_STR,0x00,UART_TEMP_STR_LEN);
       UART_TEMP_STR[0]='C';
       UART_TEMP_STR[1]='W';
       UART_TEMP_STR[2]=':';
       UART_TEMP_STR[3]=(Current_Mea_MetNUM/10)|'0';
       UART_TEMP_STR[4]=(Current_Mea_MetNUM%10)|'0';
       UART_TEMP_STR[5]=',';
       Rmstrcat(UART_TEMP_STR,U_Pr_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,U_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,",");
       Rmstrcat(UART_TEMP_STR,I_Pm_Str_Com);
       Rmstrcat(UART_TEMP_STR,"\r");
     
       SendStr(COM0_BUF,                  //����
              UART_TEMP_STR,
              0);
       COM0_OCoun=0x01;
      
   } 
}
*/
/*****************************************************************************
* AD������ʱ��ʱ����
*****************************************************************************/
void Proc_ADC_Timer(void)
{
    if(ADC_Data.Trig)                             //�Ƿ��Ѿ�����AD����
     {
       if((u16)(Timer_1ms-ADC_Timer)<AD_OVER_TIME)//�ж�AD�����Ƿ�ʱ
        return;
       Init_Adc();                                //���³�ʼ��ADC 
     }		
    else
     {	
       if((u16)(Timer_1ms-ADC_Timer)<AD_TRIG_TIME)//�ж�AD������ʱ�Ƿ�
        return;
       ADCProcessorTrigger(0);                    //��������
     	 ADC_Timer=(u16)Timer_1ms;                  //���ò�����ʱ 
       ADC_Data.Trig=1;
     }
}   
/*****************************************************************************
* ����AD����ֵ 
* �Ƿ��ۼ�
*****************************************************************************/
void Proc_ADC_Data(void)
{
    if(ADC_Start!='Y')
    	return;
    if(ADC_Data.New_Data)            //�ж��Ƿ���������
    {
       u16 Data;
       float f,f_Temp;
       u8 lon,temp; 
       u8 str[20];
     
       ADC_Data.New_Data=0;          //��������ݱ�־ 
       Data=ADC_Data.Data;           //��������ֵ
       f=Data*ADC0_Vref*XIUZ.I_SXZ/1024;
	      I_Pmax[I_Mea_Cnt]=f*Current_I_Value/20;  //�˵���ֵ(Ĭ��5A).
	      I_Mea_Cnt++;                  //������������
	      f=0;
	      if(I_Mea_Cnt>=I_MEA_CNT)      //���������ﵽ5��
	      {
	        I_Mea_Cnt=0;
	        lon=I_MEA_CNT;
         temp=lon;
         for(;lon>0;lon--)
          f+=I_Pmax[lon-1];
         f/=temp;                    //ȡ5�β�����ƽ��ֵ 
         f_Temp*=0;
         f_Temp=f;
         f+=0.0005;                  // 
         f*=1000;                    //�������Ӹ�����ת������
         Data=(u16)f;
         I_Pm_Str[0]=(u8)((Data&0xff00)>>8);
         I_Pm_Str[1]=(u8)(Data&0x00ff);
        
         sprintf(str,"%6.4f",f_Temp);
         memcpy(I_Pm_Str_Com,str,7);
         I_Data_Ready='Y';
         ADC_Data.Data=0;
	      }   
    }
}

/*****************************************************************************
* ���ÿ��Ź�ι��
* 
*****************************************************************************/
void Ext_WDT_Feed(void)
{
  if((u8)(Timer_1ms-Ext_WDT_Timer)<EXT_WDT_TIME)            //�ж�AD�����Ƿ�ʱ
    return;
  Ext_WDT_Timer=(u8)Timer_1ms;
  WDI_CHANGE;
}