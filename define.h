/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : define.h
;* Author             : ������
;* �û�����������Ԥ����
;* ���ȼ����� 1.����֡���ڴӻ�֡(����D=0 �ӻ�D=1 ID������λ��ͬ����� )
;*            2.��׼֡������չ֡
;*            3.������֡���ڶ�����֡
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_gpio.h"
//CAN �������������
#define  MSG_OBJ_NUMB      13              //���Ķ���ʹ�ø���  Ҫ��LM3S21xx_CAN.C CAN_MSG_SET_TAB ��񳤶�һ֡
#define  CAN_LDT_TYPE_NUMB 10              //����������������
#define  CAN_LDT_ACT_NUMB  2               //����������ʵ�ʸ��� ������CAN_LDT_TYPE_NUMB
//CAN ID ��������붨��
#define  IDX_MASK_CODE    (0x07<<0  )      //֡��Ų�����˴��� 3BIT
#define  END_MASK_CODE    (0x01<<3  )      //����λ������˴��� 1BIT
#define  TYPE_MSAK_CODE   (0x0F<<4  )      //�����������˴���	4BIT
#define  MNUM_MASK_CODE   (0xff<<8  )      //��λ�������˴��� 8BIT
#define  DIR_MASK_CODE    (0x01<<16 )      //����λ������˴��� 1BIT
#define  CMD_MASK_CODE    (0x7ff<<17)      //�����������˴��� 11BIT
#define  EXD_MASK_CODE	  (0x01<<28 )      //ID����չ֡��ʶ�� 	1 BIT Ϊ���ñ�׼֡���ȼ�������չ֡ ����
#define  H6BCMD_MASK_CODE (0x7E0<<17)      //ǰ6λ�����������˴��� 
//CAN ������ѯ�ͻ�Ӧ״̬����
#define  MST_CHK_RCVD     'C'	           //���յ�������ѯ����
#define  SLV_ECHO_SEND    'S'	           //���ڷ��ʹӻ���Ӧ����
#define  SLV_ECHO_ACK     'A'	           //�ӻ���Ӧ����ɹ�����
//�ӻ� CAN�����ݷ���״̬����
#define  SLV_LDATA_TX_NO   0               //�ӻ������ݿ���״̬
#define  SLV_LDATA_TX_REQ 'R'              //�ӻ������ݷ��������ѷ��� REQUEST
#define  SLV_LDATA_TX_ACK 'A'              //�ӻ������ݷ���������׼ ACK
#define  SLV_LDATA_RETX   'R'              //��������ӻ��ط���־ Retransmit
#define  SLV_LDATA_TX_IS  'S'              //�ӻ����ڷ��ͳ�����
#define  SLV_LDATA_TX_FI  'F'              //��֡�������
#define  SLV_LDATA_TX_LAST 'L'             //�ӻ����ڷ������һ֡����
#define  SLV_LDATA_TX_END 'E'              //�ӻ������ݷ��ͽ���
//�ӻ� CAN�����ݽ���״̬����
#define  SLV_LDATA_RX_NO   0               //����״̬
#define  SLV_LDATA_RX_IS  'S'              //�ӻ����������ڽ���״̬
#define  SLV_LDATA_RX_END 'E'              //�ӻ������ݽ��ս���
#define  SLV_LDATA_RX_OUT 'O'              //���յ����������ڴ���
//CAN ��֡���ݷ���״̬
#define  SLV_SMSG_TX_IS   'S'              //���ڷ��Ͷ�֡
/*
//���ڽ���״̬����
#define  COM_RX_NO         0               //����״̬
#define  COM_RX_IS        'R'              //�������ڽ�������
#define  COM_RX_END       'E'              //�������ݽ��ս���
//���ڷ���״̬����
#define  COM_TX_NO         0               //���ͻ���������״̬
#define  COM_TX_IS        'O'              //�������ڷ�������
#define  COM_TX_EN        'E'              //����������Ч
//������Ч����
#define  DATA_YES         'Y'              //�洢�������ݱ�־ ��λ��
#define  DATA_VALIDE      'E'              //������Ч��־ 
#define  DATA_BLANK       'B'              //���ݿձ�־
#define  DATA_NOT_BLACK   'N'              //���ݲ��ձ�־
*/
//����MSG RAM ID��Ӧ����
// 29bit ID ��IDmask ��ʽ 1DCCCCCCCCCCXXXXXXXXDDDDDDDD	CCCCCCCCCCC =0~1983 D DDDDDDDD �豸���ͺ�
//                         D ���� CCCCCCCCCC ����  XXXXXXXX ��λ�� 
//MST ��ʾ�������� �ӻ����� SLV ��ʾ�ӻ����� ��������
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  R=0
* �豸���ͺ�: DDDDDDDD �����Ԫ 1 ���ĵ�Ԫ 2
* ��λ��    : XXXXXXXX=0        
* �����  : �������ڷ��Ͷ̹㲥���ݴ�
* �����˲�λ:          MM             MMMMMMMM  DDDDDDDD               
* �˲�����  : ��       1DCCCCCCCCCCC  XXXXXXXX  11111111              
* �����ʽ  :          10CCCCCCCCCCC  00000000  00000002              
* �������ȼ�: ��չ֡������ 
* �����    : 0~1983 
* �㲥/���� : �㲥
**************************************************************/
#define MST_SCDATA_TX    1    //�㲥���ݴ�
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  R=0
* �豸���ͺ�: DDDDDDDD �����Ԫ 1 ���ĵ�Ԫ 2
* ��λ��    : XXXXXXXX=�������         
* �����  : �������ڷ��Ͷ̵������ݴ�
* �����˲�λ:          MM             MMMMMMMM DDDDDDDD
* �˲�����  : ��       1R CCCCCCCCCCC XXXXXXXX 11111111
* �����ʽ  :          10 CCCCCCCCCCC XXXXXXXX 00000002
* �������ȼ�: ��չ֡������ 
* �����    : 0~1983 
* �㲥/���� : ����
**************************************************************/
#define MST_SSDATA_TX    2   //�������ݴ�

/**************************************************************
* ֡����    : ��չ֡
* �����  : �ӻ�->����  D=1
* ��λ��    : XXXXXXXX=�������         
* �����  : �ӻ����ڷ��Ͷ����ݴ�
* �����˲�λ: ��
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :        1CCCCCCCCCCC 1 XXXXXXXX 0000~1111 X XXX
* �������ȼ�: ��չ֡������ 
* �����    : 4~2015 ���� ��Ϊ7��256��ָ�����һ��220��ָ���� ÿ��ָ�����Ӧһ�����ܵ�Ԫ 
* ����������: TTTT 0~15	 ������Ԥ��
* �����ݽ���: E   0      ������������
* ���ݱ��  : III 0      ������Ԥ��
* �㲥/���� : ����(ֻ�е���)
**************************************************************/
#define SLV_SDATA_TX      3   //SHORT  DATA �����ݴ�


//����AD������ʱʱ��
#define AD_OVER_TIME      100          //��λ:1ms  100ms ����125k 64��ƽ��512us
#define AD_TRIG_TIME      1000         //1�����1��
#define RSLT_SEND_TIME    2000         //2���ӷ���һ�ι��Ĳ��Խ��
#define EXT_WDT_TIME      20           //���ÿ��Ź�ι�����
//AD����ֵ�������ȶ�����
#define AD_MAX_InStable   20           //�������ȶ�����


#define CAN_PORT          0            //CAN�ӿ�
#define COM_PORT          1            //COM�ӿ�
//COM ����״̬
#define COM_RX_IDLE       0            //���տ���
#define COM_RX_IS         'R'
#define COM_RX_EN         'E'          //�������
#define COM_RX_DN         'D'          //��ֹ����                                         
//COM ����״̬
#define COM_TX_IDLE       0            //���Ϳ���
#define COM_TX_IS         'T'          //���ڷ���
//������Ч����
#define YES               'Y'          //�洢�������ݱ�־ ��λ��
#define VALIDE            'E'          //������Ч��־ 
#define BLANK             'B'          //���ݿձ�־
#define NOT_BLACK         'N'          //���ݲ��ձ�־

#define SAVE_TAB_NUMB      (1024/((sizeof(SAVE_S)+3)&0xFFFC))-1  //��ౣ������ 4�ֽڶ��� ����1

#define SMTR_COM_FIFO_LEN  14
#define GPS_COM_FIFO_LEN   14
#define WAVE_COM_FIFO_LEN  14
#define RS485_COM_FIFO_LEN 14

#define COM0_ILEN          512         //COM0���ջ���������
#define COM0_OLEN          512         //COM0���ͻ��������� 

#define COM1_ILEN          512         //COM1���ջ���������
#define COM1_OLEN          512         //COM1���ͻ��������� 

#define COM0_BUF           0           //COM0������ID��          COM0
#define COM1_BUF           1           //COM1������ID��          COM1

#define UART_TFIFO_LEN     14

#define BUF_FIFO_BLK       0             //������ UART FIFO ���� 
#define BUF_FIFO_NBLK      1             //������ UART FIFO ����һ������
#define ONE_CMD_BLK        2             //һ����������

#define UART_NUMB           2            //���ڸ���
//�����㵥Ԫ�๦�ܶ˿�                 
#define MTRCOM              0            //ģ���(�๦�ܱ�)�˿�
#define LCTCOM              1            //�����ն˶˿�
//�����ʶ��� ����λ                        
#define UART_5_BIT          0            //����λ���� 5bit
#define UART_6_BIT          1            //����λ���� 6bit
#define UART_7_BIT          2            //����λ���� 7bit
#define UART_8_BIT          3            //����λ���� 8bit
//�����ʶ��� ֹͣλ                      
#define UART_1_STOP         0            //1   stop bit      
#define UART_2_STOP         1            //2   stop bit      
//�����ʶ��� У��λ                      
#define UART_N_PARITY       0            //��У��λ
#define UART_O_PARITY       1            //ODD ��У��
#define UART_E_PARITY       2            //EVEN żУ��
#define UART_M_PARITY       3            //Mark У��(1)
#define UART_S_PARITY       4            //SpaceУ��(0)

#define UART0_BAUD          9600         //UART0Ĭ�β�����
#define UART1_BAUD          9600         //UART1Ĭ�β�����

#define MIN_BAUD            110          //��С������
#define MAX_BAUD            115200       //�������

#define Wh_Wire          '0'       //�й�����
#define Var_Wire         '1'       //�޹�����

#define VLOSS_1          '1'      //��U=100% 1S3��    ��ԭװ��Э����ͬ
#define VLOSS_2          '2'      //��U=100% 1������ 
#define VLOSS_3          '3'      //��U=50%  1����   

#define PRE_COIL         '1'      //��׼��������
#define SUB_COIL         '2'      //��׼�����μ�

#define GB               '1'      //�ұ�             ��ԭװ��Э�鲻ͬ 
#define BGB              '2'      //���ұ�

#define GD_E             '1'      //�����伯         
#define GD_C             '2'      //�����缫

#define HC_2             '1'      //�ϳ���· 
#define HC_3             '2'      //�ϳ���·
#define HC_4             '3'      //�ϳ���·

#define GDT_PLS          '1'      //���ͷ����
#define DZ_PLS           '2'      //��������
#define SZ_PLS           '3'      //ʱ������
#define XUL_PLS          '4'      //��������
#define TQ_PLS           '5'      //Ͷ������
#define HZ_PLS           '6'      //��բ����

#define EPLS_T           '1'      //����������������
#define SZ_T             '2'      //����ʱ����������
#define XUL_T            '3'      //����������������        
#define TQ_T             '4'      //Ͷ������
#define HZ_T             '5'      //��բ����

#define UNION_PLS        '1'      //���϶๦������ ʱ�� ���� Ͷ�� �ȹ���
#define ALONE_PLS        '2'      //�����๦������ ʱ�� ���� Ͷ�� �ֿ�����         
//ֻ����������������Ч
#define NO_PLS           '0'      //Ĭ������������
#define SZCLK_PLS        '1'      //��ǰ����Ϊʱ������
#define XULCLK_PLS       '2'      //��ǰ����Ϊ��������
#define TQCLK_PLS        '3'      //��ǰ����ΪͶ������

#define PA_PLS           '0'      //�����й�
#define QA_PLS           '1'      //�����޹�
#define PR_PLS           '2'      //�����й�
#define QR_PLS           '3'      //�����޹�

#define NCATCH_HB        '0'      //�ڰ�δ��׽
#define CATCH_HB         '1'      //�ڰ��Ѳ�׽

#define ZZ_STRT          '0'      //���ֿ�ʼ
#define ZZ_END           '1'      //���ֽ���

#define NY_GOOD          '0'      //��ѹ�ϸ�
#define NY_BAD           '1'      //��ѹ����
#define NY_UNKW          '2'      //δ֪

#define MEA_STOP         '0'      //ֹͣ����
#define MEA_ORDER        '1'      //��������
 
#define UA_PHASE        (1<<0)    //A���ѹ
#define UB_PHASE        (1<<1)    //B���ѹ
#define UC_PHASE        (1<<2)    //C���ѹ
#define ALL_PHASE       (UA_PHASE|UB_PHASE|UC_PHASE)//�����ѹ

#define OFF             0    //�Ͽ�
#define ON              1    //����



#define SINGLE           0   //����̨
#define THREE            1   //����̨
//���߷�ʽ����
#define WIRE_P1          '0'      //���߷�ʽ �����й�
#define WIRE_P4          '1'      //���߷�ʽ ���������й�
#define WIRE_P3_2        '2'      //���߷�ʽ ���������й�
#define WIRE_Q4_3        '3'      //���߷�ʽ ���������޹�90����Ԫ���޹�
#define WIRE_Q3_60       '4'      //���߷�ʽ ������������60���޹� 60����Ԫ���޹�
#define WIRE_Q3_90       '5'      //���߷�ʽ �������߿����޹�90����Ԫ���޹�
#define WIRE_Q4_R        '6'      //���߷�ʽ �����������޹�
#define WIRE_Q3_R        '7'      //���߷�ʽ �����������޹�
#define WIRE_P3_3        '8'      //���߷�ʽ ���������й� UA UB UC �������������UB�Եز�Ϊ0
#define WIRE_Q3_2        '9'      //���߷�ʽ ���������޹� ���������������� UB��U0
#define WIRE_Q3_CT       ':'      //���߷�ʽ ���������˹����ĵ��޹�
#define WIRE_Q1          ';'      //���෽ʽ �޹�

#define PHA              0	      //A��
#define PHB              1	      //B��
#define PHC              2	      //C��
#define SUM              3        //����
                                  
#define PUA              0	      //A���ѹ
#define PUB              1	      //B���ѹ
#define PUC              2	      //C���ѹ
#define PIA              3	      //A�����
#define PIB              4	      //B�����
#define PIC              5	      //C�����
                                  
#define UA               (1<<PUA) //A���ѹ λ����
#define UB               (1<<PUB) //B���ѹ λ���� 
#define UC               (1<<PUC) //C���ѹ λ����
#define IA               (1<<PIA) //A����� λ����
#define IB               (1<<PIB) //B����� λ����
#define IC               (1<<PIC) //C����� λ����

//��������
#define RX_PARA_NUM       5      //ָ��Я���������� �����ͨ������ ���5������
#define RX_CMD_LEN        10     //ָ��ͷ��������
#define RX_PARA_LEN       20     //ָ�������������
#define SMTR_CMD_LEN      8      //��׼�������
//CS5460A�����
//start conversions
#define START_SINGLE_CONVERSION  0xe0    //�𶯵��μ���
#define START_MULTI_CONVERSION   0xe8    //������������   
#define INIT_SYNC0_END           0xfe    //���³�ʼ��SPI β����
#define INIT_SYNC1_START         0xff    //three or more sync1+sync0 initialize SPI
#define POWER_UP                 0xa0    //power-up from power-down or halt computation from power-on
#define HALT_AND_STANDBY         0x88    //ֹͣ�������ȴ�״̬
#define HALT_AND_SLEEP           0x90    //ֹͣ��������״̬
#define CALIBRATION_COMMAND      0xc0    //��������+����ѡ��
#define CALIBRATION_CURRENT      0x08    //+�������� ��������
#define CALIBRATION_VOLTAGE      0x10    //+�������� ������ѹ
#define CALIBRATION_I_AND_V      0x18    //+�������� ������ѹ�͵���
#define CALIBRATION_GAIN         0x02    //+�������� ��������
#define CALIBRATION_OFFSET       0x01    //+�������� ƫ��������
#define REGISTER_READ            0x00    //+�Ĵ�����ַ ���Ĵ���
#define REGISTER_WRITE           0x40    //+�Ĵ�����ַ д�Ĵ���
#define CONFIG_REG_ADDR          0x00    //��ʼ���Ĵ�����ַ
#define IOFF_REG_ADDR            0x02    //����ƫ�����Ĵ�����ַ
#define IGN_REG_ADDR             0x04    //��������Ĵ�����ַ
#define VOFF_REG_ADDR	           0x06    //��ѹƫ�����Ĵ�����ַ
#define VGN_REG_ADDR             0x08    //��ѹ����Ĵ�����ַ
#define CYCLE_COUNT_REG_ADDR     0x0a    //ת�����ڼ����Ĵ�����ַ
#define PULSE_RATE_REG_ADDR      0x0c    //����Ƶ�ʼĴ�����ַ
#define I_REG_ADDR               0x0e    //��һ�ε���ֵ�Ĵ�����ַ
#define V_REG_ADDR               0x10    //��һ�ε�ѹֵ�Ĵ�����ַ
#define P_REG_ADDR               0x12    //����ֵ�Ĵ�����ַ
#define E_REG_ADDR               0x14    //����ֵ�Ĵ�����ַ
#define IRMS_REG_ADDR            0x16    //�ϸ����ڵ�����Чֵ�Ĵ�����ַ
#define VRMS_REG_ADDR            0x18    //�ϸ����ڵ�����Чֵ�Ĵ�����ַ
#define TBC_REG_ADDR             0x1a    //timebase calibration register address
#define POFF_REG_ADDR            0x1c    //power offset register
#define STATUS_REG_ADDR          0x1e    //״̬�Ĵ�����ַ
#define IACOFF_REG_ADDR          0x20    //AC current offset register 
#define VACOFF_REG_ADDR          0x22    //AC voltage offset register
#define MASK_REG_ADDR            0x34    //�ж����μĴ�����ַ
#define CTRL_REG_ADDR            0x38    //control register
#define OFFSET_DIVED_VALUE       0xffffff //signed 
#define GAIN_DIVED_VALUE         0x3fffff //unsigned
#define PULSE_DIVED_VALUE        0x000020 //unsigned
#define IVPE_DIVED_VALUE         0x7fffff //signed 
#define IVRMS_DIVED_VALUE        0xffffff //unsigned 
#define TIMEBASE_DIVED_VALUE     0x7fffff //unsigned
//����CAN�����
#define MULTI_DATA_CMD    0       //0-3     ������֡����   4��
#define MTR_DATA_CMD      4       //4-259   ԭ��Ԫ���� 256��
#define CLOCK_DATA_CMD    260     //260-515	ʱ��У�������� 256��
#define FKZD_DATA_CMD     516     //516-771 �����ն�����   256��
							  //772-2015 1244��ָ��Ԥ��
#define MTR_TAB_NUMB      128    
                          
#define WATCH_TIME 	      1000    //���Ź����ڵ�λ:mS ����
#define SYS_TIME          1000    //ϵͳ���Ķ�ʱ���� ��λ:uS ΢��
                                  
#define CAN_TX_OVTM       5000    //CAN���ͳ�ʱ5S
#define CAN_RX_OVTM       12000   //CAN���ճ�ʱ18S
#define NY_SEND_TIME      4000    //����Ͷ�ʱ
#define ZK2009_OVTM       10000   //ZK2009������ʱ
#define CAN_LDATA_SERR_MAX 3      //CAN����������ʹ������ �����ü���ֵ ��������ݱ�־ ���Խ�����һ֡

#define TIMER_8MS         8
#define NY_CHK_TIME       (200/TIMER_8MS)  //��ѹ��ⶨʱ
#define GDT_RST_TIME      (200/TIMER_8MS)  //���ͷ��λ��ʱ ��λ:ms
#define GDT_REN_TIME      (40/TIMER_8MS)   //��������ж�����ʹ�ܶ�ʱ 40ms
#define DZ_REEN_TIME      (40/TIMER_8MS)   //���������ж�����ʹ�ܶ�ʱ 40ms
#define SZ_REEN_TIME      (400/TIMER_8MS)  //ʱ�������ж�����ʹ�ܶ�ʱ 200ms          
#define XUL_REEN_TIME     (400/TIMER_8MS)  //���������ж�����ʹ�ܶ�ʱ 200ms  
#define TQ_REEN_TIME      (400/TIMER_8MS)  //Ͷ�������ж�����ʹ�ܶ�ʱ 200ms  
#define HZ_REEN_TIME      (400/TIMER_8MS)  //��բ�����ж�����ʹ�ܶ�ʱ 200ms  

#define KEY_REEN_TIME     (200/TIMER_8MS)  //��������ʹ���ж϶�ʱ
#define KEY_PLUG_TIME     (600/TIMER_8MS)  //�����ұ�ʱ
#define PLL_CHK_TIME      (200/TIMER_8MS)  //���໷PLL��鶨ʱ

#define MBJ_SEND_TIME     21000            //�������Ͷ�ʱ
#define WZTZ_SEND_TIME    19000            //����բ���Ͷ�ʱ
#define NZTZ_SEND_TIME    17000            //������բ���Ͷ�ʱ
#define GZ_SEND_TIME      15000            //��ʱ���͹���
#define GZ_STB_TIME       1000             //�����ȶ���ʱ
#define HC165_TIME        20               //HC165������ʱ

#define COM1_TIME         10               //���λ��������Ͷ�ʱ50ms ����һ��ָ��
#define COM1_OVTM         100              //���λ��������ͳ�ʱ
#define COM0_TIME         10               //���ͻ��������Ͷ�ʱ50ms ����һ��ָ��
#define COM0_OVTM         100              //���ͻ��������ͳ�ʱ
 
#define DISP_EN_TIME      15               //��ʾʹ�ܶ�ʱ

#define POWER_UP_TIME     4000             //�ϵ綨ʱ �ϵ��ȶ�
                          
#define STD_CLK_FREQ      500000           //��׼ʱ��Ƶ��
//��׼ʱ����������������ж�ʱ��=0xFFFF*1000/STD_CLK_FREQ=131ms
#define STD_CLK_OVTM     (1000/8)          //��λ:8ms Լ1s 
#define STD_ECLK_OVTM     20000            //��λ:1ms Լ20s 

//�ڱ�Ŷ���

#define PORTA             0   //PORTA �ڱ�Ŷ���
#define PORTB             1   //PORTB �ڱ�Ŷ���
#define PORTC             2   //PORTC �ڱ�Ŷ���
#define PORTD             3   //PORTD �ڱ�Ŷ���
#define PORTE             4   //PORTE �ڱ�Ŷ���
#define PORTF             5   //PORTF �ڱ�Ŷ���
#define PORTG             6   //PORTG �ڱ�Ŷ���
#define PORTH             7   //PORTH �ڱ�Ŷ���
              
//�ܽŶ������SH5.948.1200���԰�
//PORTA
#define U0RX              GPIO_PIN_0	  //UART0����
#define U0TX              GPIO_PIN_1	  //UART0����
#define SSICLK            GPIO_PIN_2   //SSICLK HD7279 ʱ��
#define SSIFSS            GPIO_PIN_3   //SSIFSS HD7279 Ƭѡ
#define DISP_RST          GPIO_PIN_4   //SSIRX  HD7279 ��λ
#define SSITX             GPIO_PIN_5   //SSITX  HD7279 ����
#define JZ_IN             GPIO_PIN_6   //CCP1���Ⱦ�������ӿ�
#define FH_IN             GPIO_PIN_7	  //CCP4��׼���Ƶ����
//PORTB
#define I2C_WP            GPIO_PIN_0   //I2Cд����
#define SZ_MC             GPIO_PIN_1   //CCP2ʱ������
#define I2C_SCL           GPIO_PIN_2   //I2Cģ��SCL
#define I2C_SDA           GPIO_PIN_3   //I2Cģ��SDA
#define CS5460A_SDO       GPIO_PIN_4   //5460A��SDO����,PB4
#define CS5460A_CS        GPIO_PIN_5   //5460A��CS����,PB5
#define CS5460A_MODE      GPIO_PIN_6   //5460A��MODE����,PB6
#define TRST              GPIO_PIN_7   //����TRST

//PORTC
#define TCK               GPIO_PIN_0   //����TCK SWCLK SW����
#define TMS               GPIO_PIN_1   //����TMS SWDIO SW����
#define TDI               GPIO_PIN_2   //����TDI
#define TDO               GPIO_PIN_3   //����TDO SWO   SW����
#define PWM_DAC	          GPIO_PIN_4   //CCP5 PWMģ�����
#define KEY_IN            GPIO_PIN_5   //��������
#define NY_IN             GPIO_PIN_6   //* �ı� NY ��ѹ�������

//PORTD                       
#define CANRX             GPIO_PIN_0   //CAN����
#define CANTX             GPIO_PIN_1   //CAN����
#define U1RX              GPIO_PIN_2   //UART1����
#define U1TX              GPIO_PIN_3   //UART1����
#define CS5460A_SCLK      GPIO_PIN_4   //5460A��SCLK����,PD4
#define CS5460A_RESET     GPIO_PIN_5   //5460A��RESET����,PD5
#define CS5460A_INT       GPIO_PIN_6   //5460A��INT����,PD6
#define CS5460A_SDI       GPIO_PIN_7   //5460A��SDO����,PD7

//PORTE                       
#define P3P4SelOut3       GPIO_PIN_0   //��λ3������ѡ��
#define P3P4SelOut2       GPIO_PIN_1   //��λ2������ѡ��
#define P3P4SelOut1       GPIO_PIN_2   //��λ1������ѡ��
#define CHNL1_Sel         GPIO_PIN_3   //��λ1���Ľ���

//PORTF                       
#define GOG_KZ            GPIO_PIN_0   //��������干�߹���ѡ�����
#define MC_PN_KZ          GPIO_PIN_1   //�ı䱻����������������� ���������� Positive(0) or Negative(1)
#define MC_WV_KZ          GPIO_PIN_2   //�ı䱻����������������� �������޹� Watt(0) or Var(1)
#define XL_MC             GPIO_PIN_3   //     �ı�* ������������
#define TQ_MC             GPIO_PIN_4   //���� �ı�* ʱ��Ͷ������
#define HZ_MC             GPIO_PIN_5   //���� �ı�* ��բ����
#define TX_MC             GPIO_PIN_6   //���� �ı�* ͨ��ָʾ
#define WDI               GPIO_PIN_7   //��� ���ÿ��Ź��ź�

//PORTG                       
#define BW       (GPIO_PIN_0|GPIO_PIN_1| \
                  GPIO_PIN_2|GPIO_PIN_3| \
                  GPIO_PIN_4|GPIO_PIN_5| \
                  GPIO_PIN_6|GPIO_PIN_7)

//PORTH   
#define Phase_Sel_K1      GPIO_PIN_0    //���Ĳ������ѡ��1
#define Phase_Sel_K2      GPIO_PIN_1    //���Ĳ������ѡ��2
#define CHNL2_Sel         GPIO_PIN_2    //��λ2���Ľ���
#define CHNL3_Sel         GPIO_PIN_3    //��λ3���Ľ���
//I/O�ں궨��
//PORTA
#define DISP_RST_EN      GPIOPinWrite(GPIOA,DISP_RST,0);       //��λ�ܽ���0
#define DISP_RST_DN      GPIOPinWrite(GPIOA,DISP_RST,DISP_RST);//��λ�ܽ���1

//PORTB
#define I2C_WP_H         GPIOPinWrite(GPIOB,I2C_WP,I2C_WP)     //I2Cд��������
#define I2C_WP_L         GPIOPinWrite(GPIOB,I2C_WP,0)          //I2Cд�����ر�
#define CS5460A_CS_H     GPIOPinWrite(GPIOB,CS5460A_CS,CS5460A_CS)         //�ͷ�CS5460A��
#define CS5460A_CS_L     GPIOPinWrite(GPIOB,CS5460A_CS,0)          //ƬѡCS5460A
#define CS5460A_MODE_H   GPIOPinWrite(GPIOB,CS5460A_MODE,CS5460A_MODE) //��CS5460A��MODE���ŷ��͸ߵ�ƽ
#define CS5460A_MODE_L   GPIOPinWrite(GPIOB,CS5460A_MODE,0)       //��CS5460A��MODE���ŷ��͵͵�ƽ
#define CS5460A_SDO_B    GPIOPinRead(GPIOB,CS5460A_SDO)        //��ȡCS5460A��SDO���ŵĵ�ƽ

//PORTC
//PORTD 
#define CS5460A_RESET_H  GPIOPinWrite(GPIOD,CS5460A_RESET,CS5460A_RESET)//��CS5460A��RESET���ŷ��͸ߵ�ƽ
#define CS5460A_RESET_L  GPIOPinWrite(GPIOD,CS5460A_RESET,0)//��CS5460A��RESET���ŷ��͵͵�ƽ
#define CS5460A_SDI_H    GPIOPinWrite(GPIOD,CS5460A_SDI,CS5460A_SDI)//��CS5460A��SDI���ŷ��͸ߵ�ƽ
#define CS5460A_SDI_L    GPIOPinWrite(GPIOD,CS5460A_SDI,0)//��CS5460A��SDI���ŷ��͵͵�ƽ
#define CS5460A_SCLK_H   GPIOPinWrite(GPIOD,CS5460A_SCLK,CS5460A_SCLK)//��CS5460A��SCLK���ŷ��͸ߵ�ƽ
#define CS5460A_SCLK_L   GPIOPinWrite(GPIOD,CS5460A_SCLK,0)        //��CS5460A��SCLK���ŷ��͵͵�ƽ

//PORTE��PORTH���Ĳ���ѡ��
#define Sel_Channel3     GPIOPinWrite(GPIOH,CHNL3_Sel,0)            //ѡ�񹦺Ĳ���3��ͨ��
#define Sel_Channel2     GPIOPinWrite(GPIOH,CHNL2_Sel,0)            //ѡ�񹦺Ĳ���2��ͨ��
#define Sel_Channel1     GPIOPinWrite(GPIOE,CHNL1_Sel,0)            //ѡ�񹦺Ĳ���1��ͨ��
#define P3_Sel1          GPIOPinWrite(GPIOE,P3P4SelOut1,0)          //ѡ�񹦺Ĳ���1��ͨ��P3���߷�ʽ
#define P4_Sel1          GPIOPinWrite(GPIOE,P3P4SelOut1,P3P4SelOut1)//ѡ�񹦺Ĳ���1��ͨ��P4���߷�ʽ��Ĭ�ϣ�
#define P3_Sel2          GPIOPinWrite(GPIOE,P3P4SelOut2,0)          //ѡ�񹦺Ĳ���2��ͨ��P3���߷�ʽ
#define P4_Sel2          GPIOPinWrite(GPIOE,P3P4SelOut2,P3P4SelOut2)//ѡ�񹦺Ĳ���2��ͨ��P4���߷�ʽ��Ĭ�ϣ�
#define P3_Sel3          GPIOPinWrite(GPIOE,P3P4SelOut3,0)          //ѡ�񹦺Ĳ���3��ͨ��P3���߷�ʽ
#define P4_Sel3          GPIOPinWrite(GPIOE,P3P4SelOut3,P3P4SelOut3)//ѡ�񹦺Ĳ���3��ͨ��P4���߷�ʽ��Ĭ�ϣ�
#define CNCL_CHNL_Sel1   GPIOPinWrite(GPIOE,CHNL1_Sel,CHNL1_Sel)    //�ͷŹ��Ĳ���ͨ��
#define CNCL_CHNL_Sel2   GPIOPinWrite(GPIOH,CHNL2_Sel|CHNL3_Sel,CHNL2_Sel|CHNL3_Sel)
#define Sel_Phase_NO     GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,0)                         //�������κ����
#define Sel_Phase_A      GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,Phase_Sel_K1)              //����A�๦��
#define Sel_Phase_B      GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,Phase_Sel_K2)              //����B�๦��
#define Sel_Phase_C      GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,Phase_Sel_K1|Phase_Sel_K2) //����C�๦��

//PORTF
#define POS_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,0)                //ѡ�������й�����
#define NEG_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_PN_KZ)         //ѡ�����й�����
#define POS_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ)         //ѡ�������޹�����
#define NEG_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ|MC_PN_KZ)//ѡ�����޹�����
#define DOWN_JOIN      GPIOPinWrite(GPIOF,GOG_KZ,GOG_KZ)       //���Ͷ� ���伫����һ�� E
#define UP_JOIN        GPIOPinWrite(GPIOF,GOG_KZ,0)            //���߶� ���缫����һ�� C

#define TX_MC_ON       GPIOPinWrite(GPIOF,TX_MC,TX_MC)         //ͨ��ָʾ����
#define TX_MC_OFF      GPIOPinWrite(GPIOF,TX_MC,0)             //ͨ��ָʾ����
/*
#define RED_485_EN     GPIOPinWrite(GPIOF,TXXZ_MC,TXXZ_MC)     //��� ͨ��ѡ�� ����ͨ�� AB�ڶ�ͨ���л�
#define RED_485_DN     GPIOPinWrite(GPIOF,TXXZ_MC,0)           //��� ͨ��ѡ�� AB�ڶ�ͨ���л� Ĭ�ϵڶ�ͨ��
*/
#define WDI_HIGH       GPIOPinWrite(GPIOF,WDI,WDI)     //��� �ⲿ���Ź���λ         
#define WDI_LOW        GPIOPinWrite(GPIOF,WDI,0)           //���   
#define WDI_CHANGE     GPIOPinWrite(GPIOF,WDI,~GPIOPinRead(GPIOF,WDI))
#define CS5460A_RW_Delay  80
//CAN���������ȶ���
#define CAN_SDILEN		 50      //CAN�����ݽ��ջ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����
#define CAN_SDOLEN		 50      //CAN�����ݷ��ͻ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����
                       
#define CAN_LDILEN		 20      //CAN�����ݽ��ջ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����
#define CAN_LDOLEN		 10      //CAN�����ݷ��ͻ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����

//��׼��                    
//��׼��������֡��        
#define SMTR_MAX_RFRAM      16     //��׼��������֡��
//��׼�� ���ջ������ܳ���      
#define SMTR_ILEN           4096   //COM2���ջ���������	�ܳ��� Ϊ�˴��䲨������(��ɼ�)
//��׼�������֡��        
#define SMTR_MAX_TFRAM      50     //��׼�������֡��
//��׼�� ���ͻ������ܳ���   
#define SMTR_OLEN           512    //COM2���ͻ��������� 	�ܳ���

//CAN ���������� ��CANBitClkSettings[]�ж�Ӧ
#define CANBAUD_100K   0
#define CANBAUD_125K   1
#define CANBAUD_250K   2
#define CANBAUD_500K   3
#define CANBAUD_1M     4
#define CANBAUD	       CANBAUD_500K
//��Ԫ��CANָ֡��ֿ� ÿ256������Ϊһ��
//ԭ��������								          //0 1 2 3  Ԥ���������ݴ���
#define  CAN_ICMD_CHK     0                     //���ղ�ѯ��Ϣ����ID
#define  CAN_ICMD_SETST   CAN_ICMD_CHK+1        //���õ���������
#define  CAN_ICMD_PMTR    CAN_ICMD_SETST+1      //���ñ�λ��Ӧ��ϵ
#define  CAN_ICMD_MEA     CAN_ICMD_PMTR+1       //��ʼ������������
#define  CAN_ICMD_XUP     CAN_ICMD_MEA+1        //������ѹ�й�����
#define  CAN_ICMD_XUS     CAN_ICMD_XUP+1        //������ѹ���ڹ���
#define  CAN_ICMD_XIS     CAN_ICMD_XUS+1        //�����������ڹ���
#define  CAN_ICMD_VER     CAN_ICMD_XIS+1        //��汾��
#define  CAN_ICMD_SOLID   CAN_ICMD_VER+1        //�̻�����
#define  CAN_ICMD_BOOT    CAN_ICMD_SOLID+1      //������������
#define  CAN_ICMD_TYPE    CAN_ICMD_BOOT+1       //����ѡ��
//���ĵ�Ԫ������������                           
#define  CAN_OCMD_CHK     0     //�����ѯ��Ϣ
#define  CAN_OCMD_MEA     1     //�����������
#define  CAN_OCMD_MEAB    2     //�����������(����)
#define  CAN_OCMD_VER     3     //����汾��
//CS5460A
#define  U_MEA_CNT    5         //��ѹ��������
#define  I_MEA_CNT    5         //������������
#define  UP_TYP     0x00        //
#define  US_TYP     0x01        //
#define  IS_TYP     0x02        //
#define  U_TYP      0x03        //
#define  I_TYP      0x04        //
#define  OU_TYP     0x05        //
#define  OI_TYP     0x06        //

//����ͨѶ����
#define  UART_ICMD_CHK     0                     //���ղ�ѯ��Ϣ����ID
#define  UART_ICMD_SETST   UART_ICMD_CHK+1       //���õ���������
#define  UART_ICMD_MEA     UART_ICMD_SETST+1     //���ñ�λ��Ӧ��ϵ
#define  UART_ICMD_PMTR    UART_ICMD_MEA+1       //��ʼ������������
#define  UART_ICMD_XUP     UART_ICMD_PMTR+1      //������ѹ�й�����
#define  UART_ICMD_XUS     UART_ICMD_XUP+1       //������ѹ���ڹ���
#define  UART_ICMD_XIS     UART_ICMD_XUS+1       //�����������ڹ���
#define  UART_ICMD_XU      UART_ICMD_XIS+1       //������ѹ
#define  UART_ICMD_XI      UART_ICMD_XU+1       //��������
#define  UART_ICMD_XOU     UART_ICMD_XI+1       //������ѹ���
#define  UART_ICMD_XOI     UART_ICMD_XOU+1       //�����������
#define  UART_ICMD_TYPE    UART_ICMD_XOI+1       //���߷�ʽ
#define  UART_ICMD_VER     UART_ICMD_TYPE+1      //��汾��
#define  UART_ICMD_SOLID   UART_ICMD_VER+1       //�̻�����
#define  UART_ICMD_BOOT    UART_ICMD_SOLID+1     //������������ 
#define  UART_ICMD_CRT_A   UART_ICMD_BOOT+1      //����A�������Чֵ 
#define  UART_ICMD_CRT_B   UART_ICMD_CRT_A+1     //����B�������Чֵ
#define  UART_ICMD_CRT_C   UART_ICMD_CRT_B+1     //����C�������Чֵ
#define  UART_ICMD_CRT_S   UART_ICMD_CRT_C+1     //���յ��������Чֵ

//���߷�ʽ����
#define WIRE_P1          '0'      //���߷�ʽ �����й�
#define WIRE_P4          '1'      //���߷�ʽ ���������й�
#define WIRE_P3_2        '2'      //���߷�ʽ ���������й�
#define WIRE_Q4_3        '3'      //���߷�ʽ ���������޹�90����Ԫ���޹�
#define WIRE_Q3_60       '4'      //���߷�ʽ ������������60���޹� 60����Ԫ���޹�
#define WIRE_Q3_90       '5'      //���߷�ʽ �������߿����޹�90����Ԫ���޹�
#define WIRE_Q4_R        '6'      //���߷�ʽ �����������޹�
#define WIRE_Q3_R        '7'      //���߷�ʽ �����������޹�
#define WIRE_P3_3        '8'      //���߷�ʽ ���������й� UA UB UC �������������UB�Եز�Ϊ0
#define WIRE_Q3_2        '9'      //���߷�ʽ ���������޹� ���������������� UB��U0
#define WIRE_Q3_CT       ':'      //���߷�ʽ ���������˹����ĵ��޹�
#define WIRE_Q1          ';'      //���෽ʽ �޹�

#define  UART_TEMP_STR_LEN      64

#define Phase_A    0x01
#define Phase_B    0x02
#define Phase_C    0x03
void Init_Ram(void);            // RAM�ͱ�����ʼ��
void Solid_Mtr_Tab(void);	      //�̻���λ�� ��RAM->FLASH
/*****************************************************************************
* ����CAN����״̬
* ����ʱ
*****************************************************************************/
void Proc_CAN_STS(void);        
//�̻�����ֵ ��RAM->FLASH
void Solid_Save_Tab(void);
void Check_SMtr_Tab_Sts(void);
/****************************************************************************
* ����COM0���ջ�����                                                                                   
****************************************************************************/  
void Proc_COM0_IBuf(void);                                                   
/****************************************************************************
* ����COM0���ͻ�����                                               
****************************************************************************/
void Proc_COM0_OBuf(void);
