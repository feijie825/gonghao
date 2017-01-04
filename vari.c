/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : vari.c
;* Author             : 张力阵
;* 变量定义
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"
//位定义 定义字变量 包含32个位变量	4字节对齐 在LM3S2139.h 中定义
//变量定义

u32         ADC_SEQ_Data[8];                //AD 采样序列
Adc_Dat     ADC_Data;                       //AD 采样数据
u16         ADC_Timer;                      //AD采样定时
u16         ADC_SUM_Data;                   //累加数据
u8          ADC_SUM_Cnt;                    //AD累加计数
                                            
u8          Current_Save_Tab;               //当前保存值所在表格
u8          Save_Tab_Sts[SAVE_TAB_NUMB];    //表位号状态表格

SAVE_S      XIUZ;                           //

u32         SysRCC_CFG;                     //系统RCC配置
u8          PLL_ERR_Cnt;                    //锁相环错误计数
u8          CMD_DATA;                       //单字节命令数据
u8          WIRE_TYPE;                      //接线方式
  
u8          Ext_WDT_Timer;                  //外置看门狗定时器

u8          Mtr_Numb_ID;                    //表位号
u8          Mtr_Numb_Str[3];                //表位号ASC码

u32         Sysclk;                         //系统频率
//中断中用到的定时器 用volatile 定义 仿真程序循环等待 不取当前定时值问题
vu32        Timer_1ms;                      //1ms定时器
vu32        CLK_Timer;                      //时钟测量定时
vu8         Timer_8ms;                      //8ms定时器 扩展定时时间 单字节扩展到2.048s
vu8         GDT_Timer;                      //光电头口中断重启定时器
vu8         DZ_Timer;                       //电子脉冲口中断重启定时器
vu8         SZ_Timer;                       //时钟脉冲管脚重开中断定时
vu8         XUL_Timer;                      //需量口中断重启定时器
vu8         TQ_Timer;                       //投切口中断重启定时器
vu8         HZ_Timer;                       //合闸口中断重启定时器
vu8         KEY_Timer;                      //按键口中断重启定时器
//vu8         HC165_Timer;                    //HC165采样定时
//vu8         CD4094_Timer;                   //CD4094输出定时
vu8         PLL_CHK_Timer;                  //PLL检查定时

//以下定时用Timer_1ms 计时 精度较高 或时间长于2.048s
u32         CYCLE_OLD_Timer;                //周期测量定时器(老)
u32         CYCLE_NEW_Timer;                //周期测量定时器(新)
//u32         CLK_Timer_Max;                  //时钟测量脉冲最大值
//u32         CLK_Timer_Min;                  //时钟测量脉冲最小值
//u16         SY_Timer;                       //失压计时
//u16         NY_SEND_Timer;                  //命令回送定时
u16         CAN_STX_OVTimer;                //CAN 短数据发送超时定时器
u16         CAN_LTX_OVTimer;                //CAN 长数据发送超时定时器
u16         CAN_RX_OVTimer;                 //CAN 总线接收超时定时器 用于处理总线错误
u16         WORK_Timer;                     //进入工作模式定时器 单位:ms 最多65.535S
u16         STD_ECLK_Timer;                 //标准电能脉冲检测定时
//u16         MBJ_Send_Timer;                 //表报警数据回送定时
//u16         WZTZ_Send_Timer;                //外置跳闸数据回送定时
//u16         NZTZ_Send_Timer;                //内置跳闸数据回送定时
//u16         XLJDQ_ERR_Timer;                //续流继电器故障报警定时
u16         NEW_PLL_Ref_Timer;              //锁相环参考定时器
u16         OLD_PLL_Ref_Timer;              //锁相环参考定时器
//以下定时器 用Timer_8ms 计时
u8          Disp_Timer;                     //显示定时处理
u8          GDT_RST_Timer;                  //光电头复位延时
//u8          Com_Rx_Time[UART_NUMB];         //COM 接收定时
//u8          Com_Rx_OvTm[UART_NUMB];         //UART接收超时处理
//u8          UJDQ_Timer;                     //电压继电器定时
//u8          IJDQ_Timer;                     //电流继电器定时
//u8          ESwitch_Timer;                  //电子开关定时
//u8          NY_CHK_Timer;                   //耐压查询定时
u8          Con_KEY_Timer;                  //连续按键定时
u8          Disp_En_Timer;                  //显示使能定时 防止连续写入SSI
u8          STD_CLK_Timer;                  //标准时钟脉冲检测定时
u8          SMtr_Set_Timer;                 //标准表设置定时

u8          Disp_Buf[8];                    //显示缓冲区
UART_SET    Uart_Para[UART_NUMB];           //UART 参数
//CAN通信定义                               
u16         CANERR_CNT;                     //CAN错误计数  
u8          CAN_LMSG_TX_TYPE;               //当前发送CAN长数据类型
u8          CAN_LMSG_RX_TYPE;               //读取接收CAN长数据类型
CAN_LMSG_PR CAN_LMSG_RX_Ptr;                //CAN长数据接收结构体指针
CAN_LMSG_PR CAN_LMSG_TX_Ptr;                //CAN长数据发送结构体指针
CAN_STS_U   CAN_STS;                        //总线状态
u8          CAN_LDATA_TX_STS;               //CAN长数据发送状态 见define.h CAN长数据发送状态定义
u8          CAN_NEXT_MSG_IDx;               //CAN下一帧报文的索引号 根据该号判断是否丢失报文
u8          CAN_LMSG_TX_STS;                //CAN长帧发送状态
u8          CAN_SMSG_TX_STS;                //CAN短帧发送状态
CAN_MSG     CAN_MSG_Rx;                     //用于临时接收CAN报文
CAN_MSG     CAN_SMSG_Tx;                    //用于临时发送CAN短数据报文
CAN_MSG     CAN_LMSG_Tx;                    //用于临时发送CAN长数据报文
u8          Echo_Sts;                       //响应状态  0 无定义 'C':收到查询 'S':已经发送回应命令 'A':已经响应
u8          CAN_SEND_DTIME;                 //CAN帧发送延时时间
u8          CAN_SEND_DELAY;                 //CAN帧发送延时定时器 防止同时竞争总线
CAN_MSG     CAN_SDATA_MSG_IBUF[CAN_SDILEN]; //CAN短数据接收指令缓冲区
u8          CAN_SDATA_MSG_IHead;            //CAN短数据接收指令缓冲区头指针 接收指针
u8          CAN_SDATA_MSG_ITail;            //CAN短数据接收指令缓冲区尾指针 处理指针
CAN_MSG     *CAN_MSG_IPtr;                  //CAN短数据帧接收处理指针
CAN_MSG     CAN_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN短数据发送指令缓冲区
u8          CAN_SDATA_MSG_OHead;            //CAN短数据发送指令缓冲区头指针 接收指针
u8          CAN_SDATA_MSG_OTail;            //CAN短数据发送指令缓冲区尾指针 处理指针
CAN_MSG     *CAN_MSG_OPtr;                  //CAN短数据帧发送处理指针

CAN_MSG     CAN_LDATA_MSG_IBUF[CAN_LDILEN]; //CAN长数据接收指令缓冲区               
u8          CAN_LDATA_MSG_IHead;            //CAN长数据接收指令缓冲区头指针 接收指针
u8          CAN_LDATA_MSG_ITail;            //CAN长数据接收指令缓冲区尾指针 处理指针
u8          CAN_LDATA_SERR_Cnt;             //CAN长数据发送错误计数


//为编写缓冲区 共用的发送和接收程序定义的结构体 包含指针的指针
IBUF_Pr     IBUF_Ptr;                        //接收缓冲区处理结构体
OBUF_Pr     OBUF_Ptr;                        //发送缓冲区处理结构体 
OBUF_Pr     UART_Ptr[4];                     //串口发送缓冲区处理结构体 可能在中断中调用                                                                   

u8          COM0_InSend;                     //COM0数据发送状态 'Y'正在发送数据 其它 不在发送数据
u8          COM0_IBuf[COM0_ILEN];            //COM0接收缓冲区
u8          COM0_OBuf[COM0_OLEN];            //COM0发送缓冲区
u16         COM0_IHead;                      //COM0接收缓冲区头指针 接收指针
u16         COM0_ITail;                      //COM0接收缓冲区尾指针 处理指针
u16         COM0_ICoun;                      //COM0接收命令计数
u16         COM0_OHead;                      //COM0发送缓冲区头指针接收指针
u16         COM0_OTail;                      //COM0发送缓冲区尾指针处理指针
u16         COM0_OCoun;                      //COM0发送命令计数
u16         COM0_STimer;                     //COM0发送定时

u8          COM1_InSend;                     //COM1数据发送状态 'Y'正在发送数据 其它 不在发送数据
u8          COM1_IBuf[COM1_ILEN];            //COM1接收缓冲区
u8          COM1_OBuf[COM1_OLEN];            //COM1发送缓冲区
u16         COM1_IHead;                      //COM1接收缓冲区头指针 接收指针
u16         COM1_ITail;                      //COM1接收缓冲区尾指针 处理指针
u16         COM1_ICoun;                      //COM1接收命令计数
u16         COM1_OHead;                      //COM1发送缓冲区头指针接收指针
u16         COM1_OTail;                      //COM1发送缓冲区尾指针处理指针
u16         COM1_OCoun;                      //COM1发送命令计数
u16         COM1_STimer;                     //COM1发送定时


u8          Rx_Com[RX_CMD_LEN];                //取出的指令头//com0  com1 com2 公用
u8          Rx_Para[RX_PARA_NUM][RX_PARA_LEN]; //取出的指令头 取出的数据 公用 最多带两个数据
u8          Para_Numb;                         //参数个数

u8          CANT_STR[8];                     //CAN帧字符串临时缓冲区
u8          TEMP_STR[30];                    //CAN临时用字符串
u8          UART_TEMP_STR[UART_TEMP_STR_LEN];               //UART临时用字符串
u8          CHK_OneTime_Flag;                    //接收到一次查询命令标志。
u8						CHK_ONETIME_FLAG1;
//ADC处理定义
u8          ADC_Start;                       //ADC转换开始标志
//5460A处理定义
u8          CS5460A_New_Data;                //CS5460A有新数据等待处理
u8          U_Mea_Cnt;                       //电压测量次数
u8          I_Mea_Cnt;                       //电流测量次数
u8          CS5460A_E_Reg[3];                //CS5460A功率寄存器
u8          CS5460A_Vrms_Reg[3];             //电压有效值寄存器
u8          CS5460A_Irms_Reg[3];             //电流有效值寄存器
u8          U_Data_Ready;                    //电压线圈功耗数据准备好标志
u8          I_Data_Ready;                    //电流线圈功耗数据准备好标志
u8          U_Pr_Str[2];                     //电压有功功率存储空间
u8          U_Pm_Str[2];                     //电压视在功率存储空间
u8          I_Pm_Str[2];                     //电流视在功率存储空间
u8          U_Pr_Str_Com[10];                 //电压有功功率存储空间
u8          U_Pm_Str_Com[10];                 //电压视在功率存储空间
u8          I_Pm_Str_Com[10];                 //电流视在功率存储空间
u8          U_Str_Com[10];                 //电压存储空间
u8          I_Str_Com[10];                 //电流存储空间
u8          U_Mea_Cal;                       //电压测量可以开始
u8          I_Mea_Cal;                       //电流测量可以开始
u8          CS5460A_Sts_Reg[3];              //CS5460A状态寄存器
float       U_Prms[U_MEA_CNT];               //电压有功功率有效值
float       U_Pmax[U_MEA_CNT];               //电压视在功率有效值
float       I_Pmax[I_MEA_CNT];               //电流视在功率有效值
float       U_P[U_MEA_CNT];               //电压有效值
float       I_P[I_MEA_CNT];               //电流有效值
float U_current;
float I_current;
//float       Current_Adj_Val[3];              //当前功耗修正值系数
float       ADC0_Vref;                       //电流采样回路参考电压3.0V
float       CS5460A_Vref;                    //CS5460A参考电压2.5V
float       Current_I_Value;                 //电流有效值(默认5A)
Int_Char    Temp_Cal1;
Float_Char  Temp_Cal2;
u16         Timer_CS5460A_INT;               //CS5460A中断定时
//u8          Mea_Met_Tab[3];                  //测量通道和测量表位的对应关系
u8          Current_Mea_MetNUM;              //当前正在测量的表位号
u8          Current_Mea_Channel;             //当前正在测量的通道号
u8          Current_Mea_Phase;               //测量的相别
u16         Rslt_Send_Timer;                 //测量结果发送定时器，2s发送一次
u16         Rslt_Com_Send_Timer;             //测量数据结果发送定时起，2s发送一次
u8          Current_Wire_Type;               //当前接线方式

