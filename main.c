/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
* File Name          : main.c
* Author             : ������
* ������
*******************************************************************************/
#include "LM3S2139.h"   // include <*.h> ��ʾϵͳ�ļ��� include "*.h"  ��ʾ�����ļ���ͷ�ļ�
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
#include "string.h"
#include "Disp.h"
#include "define.h"
#include "vari.h"
#include "CS5460A.h"
#include "math.h"
#include "stdmtr.h"
#include "CS5460A.h"
int main(void)
{  
    SysCtlDelay(3125000);                  //��ʱ500mS
    IntMasterDisable();                    //���������ж�
    Init_Pll();                            //��ʼ�����໷
    Init_Gpio();                           //��ʼ��I/O��
    SysCtlDelay(625000);                   //��ʱ100ms
    Init_SysTick();                        //��ʼ��ϵͳ���Ķ�ʱ�� ��ʹ���ж�
    Init_CS5460A();                        //��ʼ��CS5460A
    Init_Uart();                           //��ʼ������
    Init_Adc();                            //��ʼ��ADC
    Init_Ram();                            //��ʼ��RAM���ͱ���
    Init_CAN();                            //����CAN�ӿ�
    //Init_Wdt();                            //��ʼ�����Ź�������
    Init_Int();                            //��ʼ��NVIC�ж�ʹ�ܺ����ȼ�����
    IntMasterEnable();                     //CPU�ж�����
	   ADC_Start='Y';                         //����ADCת��
    Start_CS5460AConv();                   //����CS5460Aת��
    for(;;)                                
     {	                                    
       //WDTFeed();                          
       Ext_WDT_Feed();                     
                                           
       Proc_SDATA_IBUF();                  //����CAN�����ݽ���ָ�����
       Proc_SDATA_OBUF();                  //����CAN�����ݷ���ָ�����
       Proc_CAN_OvTm();                    //CAN ���߳�ʱ����
       Proc_CAN_STS();                     //����CAN����״̬
                                           
       Proc_COM0_IBuf();                   //������1���ջ�����
       Proc_COM0_OBuf();                   //������1���ͻ�����
       Proc_ADC_Timer();                   //AD������ʱ/��ʱ����
                                           
       Check_PLL();                        //���PLLʱ��
                                           
       New_Data_Prc();                     //CS5460Aת������
       Proc_ADC_Data();                    //ADC���ݴ���
                                           
       Power_Mea_Resualt_Send();           //���Ĳ�������(CAN)��ʱ����
       Power_Mea_Resualt_Com_Send();       //���Ĳ�������(RS485)��ʱ����
			 Ui_Mea_Resualt_Com_Send();
      
     }
}
