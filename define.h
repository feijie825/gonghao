/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : define.h
;* Author             : 张力阵
;* 用户程序声明和预定义
;* 优先级规则 1.主机帧高于从机帧(主机D=0 从机D=1 ID中其他位相同情况下 )
;*            2.标准帧高于扩展帧
;*            3.长数据帧高于短数据帧
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_gpio.h"
//CAN 长数据种类个数
#define  MSG_OBJ_NUMB      13              //报文对象使用个数  要和LM3S21xx_CAN.C CAN_MSG_SET_TAB 表格长度一帧
#define  CAN_LDT_TYPE_NUMB 10              //长数据总类最大个数
#define  CAN_LDT_ACT_NUMB  2               //长数据总类实际个数 不大于CAN_LDT_TYPE_NUMB
//CAN ID 参与过滤码定义
#define  IDX_MASK_CODE    (0x07<<0  )      //帧编号参与过滤代码 3BIT
#define  END_MASK_CODE    (0x01<<3  )      //结束位参与过滤代码 1BIT
#define  TYPE_MSAK_CODE   (0x0F<<4  )      //类型域参与过滤代码	4BIT
#define  MNUM_MASK_CODE   (0xff<<8  )      //表位域参与过滤代码 8BIT
#define  DIR_MASK_CODE    (0x01<<16 )      //方向位参与过滤代码 1BIT
#define  CMD_MASK_CODE    (0x7ff<<17)      //命令域参与过滤代码 11BIT
#define  EXD_MASK_CODE	  (0x01<<28 )      //ID中扩展帧标识符 	1 BIT 为了让标准帧优先级高于扩展帧 增加
#define  H6BCMD_MASK_CODE (0x7E0<<17)      //前6位命令域参与过滤代码 
//CAN 主机查询和回应状态定义
#define  MST_CHK_RCVD     'C'	           //已收到主机查询命令
#define  SLV_ECHO_SEND    'S'	           //正在发送从机响应命令
#define  SLV_ECHO_ACK     'A'	           //从机响应命令成功发送
//从机 CAN长数据发送状态定义
#define  SLV_LDATA_TX_NO   0               //从机长数据空闲状态
#define  SLV_LDATA_TX_REQ 'R'              //从机长数据发送请求已发出 REQUEST
#define  SLV_LDATA_TX_ACK 'A'              //从机长数据发送请求被批准 ACK
#define  SLV_LDATA_RETX   'R'              //主机请求从机重发标志 Retransmit
#define  SLV_LDATA_TX_IS  'S'              //从机正在发送长数据
#define  SLV_LDATA_TX_FI  'F'              //单帧发送完成
#define  SLV_LDATA_TX_LAST 'L'             //从机正在发送最后一帧数据
#define  SLV_LDATA_TX_END 'E'              //从机长数据发送结束
//从机 CAN长数据接收状态定义
#define  SLV_LDATA_RX_NO   0               //空闲状态
#define  SLV_LDATA_RX_IS  'S'              //从机长数据正在接收状态
#define  SLV_LDATA_RX_END 'E'              //从机长数据接收结束
#define  SLV_LDATA_RX_OUT 'O'              //接收到的数据正在处理
//CAN 短帧数据发送状态
#define  SLV_SMSG_TX_IS   'S'              //正在发送短帧
/*
//串口接收状态定义
#define  COM_RX_NO         0               //空闲状态
#define  COM_RX_IS        'R'              //串口正在接收数据
#define  COM_RX_END       'E'              //串口数据接收结束
//串口发送状态定义
#define  COM_TX_NO         0               //发送缓冲区空闲状态
#define  COM_TX_IS        'O'              //串口正在发送数据
#define  COM_TX_EN        'E'              //串口数据有效
//数据有效定义
#define  DATA_YES         'Y'              //存储区有数据标志 表位号
#define  DATA_VALIDE      'E'              //数据有效标志 
#define  DATA_BLANK       'B'              //数据空标志
#define  DATA_NOT_BLACK   'N'              //数据不空标志
*/
//定义MSG RAM ID对应命令
// 29bit ID 和IDmask 格式 1DCCCCCCCCCCXXXXXXXXDDDDDDDD	CCCCCCCCCCC =0~1983 D DDDDDDDD 设备类型号
//                         D 方向 CCCCCCCCCC 命令  XXXXXXXX 表位号 
//MST 表示主机发送 从机接收 SLV 表示从机发送 主机接收
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  R=0
* 设备类型号: DDDDDDDD 电机单元 1 功耗单元 2
* 表位号    : XXXXXXXX=0        
* 命令功能  : 主机正在发送短广播数据串
* 参与滤波位:          MM             MMMMMMMM  DDDDDDDD               
* 滤波屏蔽  : 开       1DCCCCCCCCCCC  XXXXXXXX  11111111              
* 命令格式  :          10CCCCCCCCCCC  00000000  00000002              
* 命令优先级: 扩展帧中依次 
* 命令号    : 0~1983 
* 广播/单播 : 广播
**************************************************************/
#define MST_SCDATA_TX    1    //广播数据串
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  R=0
* 设备类型号: DDDDDDDD 电机单元 1 功耗单元 2
* 表位号    : XXXXXXXX=本机表号         
* 命令功能  : 主机正在发送短单播数据串
* 参与滤波位:          MM             MMMMMMMM DDDDDDDD
* 滤波屏蔽  : 开       1R CCCCCCCCCCC XXXXXXXX 11111111
* 命令格式  :          10 CCCCCCCCCCC XXXXXXXX 00000002
* 命令优先级: 扩展帧中依次 
* 命令号    : 0~1983 
* 广播/单播 : 单播
**************************************************************/
#define MST_SSDATA_TX    2   //单播数据串

/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 从机->主机  D=1
* 表位号    : XXXXXXXX=本机表号         
* 命令功能  : 从机正在发送短数据串
* 参与滤波位: 无
* 滤波屏蔽  : 关 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :        1CCCCCCCCCCC 1 XXXXXXXX 0000~1111 X XXX
* 命令优先级: 扩展帧中依次 
* 命令号    : 4~2015 分组 分为7个256条指令组和一个220条指令组 每个指令组对应一个功能单元 
* 长数据类型: TTTT 0~15	 该命令预留
* 长数据结束: E   0      该命令无意义
* 数据编号  : III 0      该命令预留
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define SLV_SDATA_TX      3   //SHORT  DATA 短数据串


//定义AD采样超时时间
#define AD_OVER_TIME      100          //单位:1ms  100ms 正常125k 64次平均512us
#define AD_TRIG_TIME      1000         //1秒采样1次
#define RSLT_SEND_TIME    2000         //2秒钟发送一次功耗测试结果
#define EXT_WDT_TIME      20           //外置看门狗喂狗间隔
//AD采样值连续不稳定次数
#define AD_MAX_InStable   20           //连续不稳定次数


#define CAN_PORT          0            //CAN接口
#define COM_PORT          1            //COM接口
//COM 接收状态
#define COM_RX_IDLE       0            //接收空闲
#define COM_RX_IS         'R'
#define COM_RX_EN         'E'          //允许接收
#define COM_RX_DN         'D'          //禁止接收                                         
//COM 接收状态
#define COM_TX_IDLE       0            //发送空闲
#define COM_TX_IS         'T'          //正在发送
//数据有效定义
#define YES               'Y'          //存储区有数据标志 表位号
#define VALIDE            'E'          //数据有效标志 
#define BLANK             'B'          //数据空标志
#define NOT_BLACK         'N'          //数据不空标志

#define SAVE_TAB_NUMB      (1024/((sizeof(SAVE_S)+3)&0xFFFC))-1  //最多保存条数 4字节对齐 保留1

#define SMTR_COM_FIFO_LEN  14
#define GPS_COM_FIFO_LEN   14
#define WAVE_COM_FIFO_LEN  14
#define RS485_COM_FIFO_LEN 14

#define COM0_ILEN          512         //COM0接收缓冲区长度
#define COM0_OLEN          512         //COM0发送缓冲区长度 

#define COM1_ILEN          512         //COM1接收缓冲区长度
#define COM1_OLEN          512         //COM1发送缓冲区长度 

#define COM0_BUF           0           //COM0缓冲区ID号          COM0
#define COM1_BUF           1           //COM1缓冲区ID号          COM1

#define UART_TFIFO_LEN     14

#define BUF_FIFO_BLK       0             //缓冲区 UART FIFO 都空 
#define BUF_FIFO_NBLK      1             //缓冲区 UART FIFO 至少一个不空
#define ONE_CMD_BLK        2             //一条命令发送完毕

#define UART_NUMB           2            //串口个数
//误差计算单元多功能端口                 
#define MTRCOM              0            //模拟表(多功能表)端口
#define LCTCOM              1            //负控终端端口
//波特率定义 数据位                        
#define UART_5_BIT          0            //数据位长度 5bit
#define UART_6_BIT          1            //数据位长度 6bit
#define UART_7_BIT          2            //数据位长度 7bit
#define UART_8_BIT          3            //数据位长度 8bit
//波特率定义 停止位                      
#define UART_1_STOP         0            //1   stop bit      
#define UART_2_STOP         1            //2   stop bit      
//波特率定义 校验位                      
#define UART_N_PARITY       0            //无校验位
#define UART_O_PARITY       1            //ODD 奇校验
#define UART_E_PARITY       2            //EVEN 偶校验
#define UART_M_PARITY       3            //Mark 校验(1)
#define UART_S_PARITY       4            //Space校验(0)

#define UART0_BAUD          9600         //UART0默任波特率
#define UART1_BAUD          9600         //UART1默任波特率

#define MIN_BAUD            110          //最小波特率
#define MAX_BAUD            115200       //最大波特率

#define Wh_Wire          '0'       //有功接线
#define Var_Wire         '1'       //无功接线

#define VLOSS_1          '1'      //ΔU=100% 1S3次    与原装置协议相同
#define VLOSS_2          '2'      //ΔU=100% 1个周期 
#define VLOSS_3          '3'      //ΔU=50%  1分钟   

#define PRE_COIL         '1'      //标准表接入初级
#define SUB_COIL         '2'      //标准表接入次级

#define GB               '1'      //挂表             与原装置协议不同 
#define BGB              '2'      //不挂表

#define GD_E             '1'      //共发射集         
#define GD_C             '2'      //共集电极

#define HC_2             '1'      //合成两路 
#define HC_3             '2'      //合成三路
#define HC_4             '3'      //合成四路

#define GDT_PLS          '1'      //光电头脉冲
#define DZ_PLS           '2'      //电子脉冲
#define SZ_PLS           '3'      //时钟脉冲
#define XUL_PLS          '4'      //需量脉冲
#define TQ_PLS           '5'      //投切脉冲
#define HZ_PLS           '6'      //合闸脉冲

#define EPLS_T           '1'      //测量电能脉冲周期
#define SZ_T             '2'      //测量时钟脉冲周期
#define XUL_T            '3'      //测量需量脉冲周期        
#define TQ_T             '4'      //投切脉冲
#define HZ_T             '5'      //合闸脉冲

#define UNION_PLS        '1'      //联合多功能脉冲 时钟 需量 投切 等共用
#define ALONE_PLS        '2'      //独立多功能脉冲 时钟 需量 投切 分开输入         
//只有在联合脉冲下有效
#define NO_PLS           '0'      //默认无脉冲输入
#define SZCLK_PLS        '1'      //当前输入为时钟脉冲
#define XULCLK_PLS       '2'      //当前输入为需量脉冲
#define TQCLK_PLS        '3'      //当前输入为投切脉冲

#define PA_PLS           '0'      //正向有功
#define QA_PLS           '1'      //正向无功
#define PR_PLS           '2'      //反向有功
#define QR_PLS           '3'      //反向无功

#define NCATCH_HB        '0'      //黑斑未捕捉
#define CATCH_HB         '1'      //黑斑已捕捉

#define ZZ_STRT          '0'      //走字开始
#define ZZ_END           '1'      //走字结束

#define NY_GOOD          '0'      //耐压合格
#define NY_BAD           '1'      //耐压击穿
#define NY_UNKW          '2'      //未知

#define MEA_STOP         '0'      //停止测量
#define MEA_ORDER        '1'      //正常工作
 
#define UA_PHASE        (1<<0)    //A相电压
#define UB_PHASE        (1<<1)    //B相电压
#define UC_PHASE        (1<<2)    //C相电压
#define ALL_PHASE       (UA_PHASE|UB_PHASE|UC_PHASE)//三相电压

#define OFF             0    //断开
#define ON              1    //接入



#define SINGLE           0   //单相台
#define THREE            1   //三相台
//接线方式定义
#define WIRE_P1          '0'      //接线方式 单相有功
#define WIRE_P4          '1'      //接线方式 三相四线有功
#define WIRE_P3_2        '2'      //接线方式 三相三线有功
#define WIRE_Q4_3        '3'      //接线方式 三相四线无功90度三元件无功
#define WIRE_Q3_60       '4'      //接线方式 三相三线移相60度无功 60度两元件无功
#define WIRE_Q3_90       '5'      //接线方式 三相三线跨线无功90度两元件无功
#define WIRE_Q4_R        '6'      //接线方式 三相四线真无功
#define WIRE_Q3_R        '7'      //接线方式 三相三线真无功
#define WIRE_P3_3        '8'      //接线方式 三相三线有功 UA UB UC 按三相四线输出UB对地不为0
#define WIRE_Q3_2        '9'      //接线方式 三相三线无功 输出按三相三线输出 UB接U0
#define WIRE_Q3_CT       ':'      //接线方式 三相三线人工中心点无功
#define WIRE_Q1          ';'      //单相方式 无功

#define PHA              0	      //A相
#define PHB              1	      //B相
#define PHC              2	      //C相
#define SUM              3        //合相
                                  
#define PUA              0	      //A相电压
#define PUB              1	      //B相电压
#define PUC              2	      //C相电压
#define PIA              3	      //A相电流
#define PIB              4	      //B相电流
#define PIC              5	      //C相电流
                                  
#define UA               (1<<PUA) //A相电压 位定义
#define UB               (1<<PUB) //B相电压 位定义 
#define UC               (1<<PUC) //C相电压 位定义
#define IA               (1<<PIA) //A相电流 位定义
#define IB               (1<<PIB) //B相电流 位定义
#define IC               (1<<PIC) //C相电流 位定义

//参数定义
#define RX_PARA_NUM       5      //指令携带参数个数 被检表通信设置 最多5个参数
#define RX_CMD_LEN        10     //指令头长度限制
#define RX_PARA_LEN       20     //指令参数长度限制
#define SMTR_CMD_LEN      8      //标准表命令长度
//CS5460A命令表
//start conversions
#define START_SINGLE_CONVERSION  0xe0    //起动单次计算
#define START_MULTI_CONVERSION   0xe8    //启动连续计算   
#define INIT_SYNC0_END           0xfe    //重新初始化SPI 尾命令
#define INIT_SYNC1_START         0xff    //three or more sync1+sync0 initialize SPI
#define POWER_UP                 0xa0    //power-up from power-down or halt computation from power-on
#define HALT_AND_STANDBY         0x88    //停止计算进入等待状态
#define HALT_AND_SLEEP           0x90    //停止进入休眠状态
#define CALIBRATION_COMMAND      0xc0    //修正命令+修正选择
#define CALIBRATION_CURRENT      0x08    //+修正命令 修正电流
#define CALIBRATION_VOLTAGE      0x10    //+修正命令 修正电压
#define CALIBRATION_I_AND_V      0x18    //+修正命令 修正电压和电流
#define CALIBRATION_GAIN         0x02    //+修正命令 增益修正
#define CALIBRATION_OFFSET       0x01    //+修正命令 偏移量修正
#define REGISTER_READ            0x00    //+寄存器地址 读寄存器
#define REGISTER_WRITE           0x40    //+寄存器地址 写寄存器
#define CONFIG_REG_ADDR          0x00    //初始化寄存器地址
#define IOFF_REG_ADDR            0x02    //电流偏移量寄存器地址
#define IGN_REG_ADDR             0x04    //电流增益寄存器地址
#define VOFF_REG_ADDR	           0x06    //电压偏移量寄存器地址
#define VGN_REG_ADDR             0x08    //电压增益寄存器地址
#define CYCLE_COUNT_REG_ADDR     0x0a    //转换周期计数寄存器地址
#define PULSE_RATE_REG_ADDR      0x0c    //脉冲频率寄存器地址
#define I_REG_ADDR               0x0e    //上一次电流值寄存器地址
#define V_REG_ADDR               0x10    //上一次电压值寄存器地址
#define P_REG_ADDR               0x12    //功率值寄存器地址
#define E_REG_ADDR               0x14    //电能值寄存器地址
#define IRMS_REG_ADDR            0x16    //上个周期电流有效值寄存器地址
#define VRMS_REG_ADDR            0x18    //上个周期电流有效值寄存器地址
#define TBC_REG_ADDR             0x1a    //timebase calibration register address
#define POFF_REG_ADDR            0x1c    //power offset register
#define STATUS_REG_ADDR          0x1e    //状态寄存器地址
#define IACOFF_REG_ADDR          0x20    //AC current offset register 
#define VACOFF_REG_ADDR          0x22    //AC voltage offset register
#define MASK_REG_ADDR            0x34    //中断屏蔽寄存器地址
#define CTRL_REG_ADDR            0x38    //control register
#define OFFSET_DIVED_VALUE       0xffffff //signed 
#define GAIN_DIVED_VALUE         0x3fffff //unsigned
#define PULSE_DIVED_VALUE        0x000020 //unsigned
#define IVPE_DIVED_VALUE         0x7fffff //signed 
#define IVRMS_DIVED_VALUE        0xffffff //unsigned 
#define TIMEBASE_DIVED_VALUE     0x7fffff //unsigned
//定义CAN命令表
#define MULTI_DATA_CMD    0       //0-3     长数据帧命令   4条
#define MTR_DATA_CMD      4       //4-259   原误差单元命令 256条
#define CLOCK_DATA_CMD    260     //260-515	时钟校验仪命令 256条
#define FKZD_DATA_CMD     516     //516-771 负控终端命令   256条
							  //772-2015 1244条指令预留
#define MTR_TAB_NUMB      128    
                          
#define WATCH_TIME 	      1000    //看门狗周期单位:mS 毫秒
#define SYS_TIME          1000    //系统节拍定时周期 单位:uS 微秒
                                  
#define CAN_TX_OVTM       5000    //CAN发送超时5S
#define CAN_RX_OVTM       12000   //CAN接收超时18S
#define NY_SEND_TIME      4000    //命令发送定时
#define ZK2009_OVTM       10000   //ZK2009启动超时
#define CAN_LDATA_SERR_MAX 3      //CAN长数据最大发送错误计数 超过该计数值 清除有数据标志 可以接收下一帧

#define TIMER_8MS         8
#define NY_CHK_TIME       (200/TIMER_8MS)  //耐压检测定时
#define GDT_RST_TIME      (200/TIMER_8MS)  //光电头复位延时 单位:ms
#define GDT_REN_TIME      (40/TIMER_8MS)   //光电脉冲中断重新使能定时 40ms
#define DZ_REEN_TIME      (40/TIMER_8MS)   //电子脉冲中断重新使能定时 40ms
#define SZ_REEN_TIME      (400/TIMER_8MS)  //时钟脉冲中断重新使能定时 200ms          
#define XUL_REEN_TIME     (400/TIMER_8MS)  //需量脉冲中断重新使能定时 200ms  
#define TQ_REEN_TIME      (400/TIMER_8MS)  //投切脉冲中断重新使能定时 200ms  
#define HZ_REEN_TIME      (400/TIMER_8MS)  //合闸脉冲中断重新使能定时 200ms  

#define KEY_REEN_TIME     (200/TIMER_8MS)  //按键重启使能中断定时
#define KEY_PLUG_TIME     (600/TIMER_8MS)  //按键挂表定时
#define PLL_CHK_TIME      (200/TIMER_8MS)  //锁相环PLL检查定时

#define MBJ_SEND_TIME     21000            //表报警回送定时
#define WZTZ_SEND_TIME    19000            //表跳闸回送定时
#define NZTZ_SEND_TIME    17000            //外置跳闸回送定时
#define GZ_SEND_TIME      15000            //定时回送故障
#define GZ_STB_TIME       1000             //故障稳定定时
#define HC165_TIME        20               //HC165采样定时

#define COM1_TIME         10               //波形缓冲区发送定时50ms 发送一条指令
#define COM1_OVTM         100              //波形缓冲区发送超时
#define COM0_TIME         10               //发送缓冲区发送定时50ms 发送一条指令
#define COM0_OVTM         100              //发送缓冲区发送超时
 
#define DISP_EN_TIME      15               //显示使能定时

#define POWER_UP_TIME     4000             //上电定时 上电稳定
                          
#define STD_CLK_FREQ      500000           //标准时钟频率
//标准时钟脉冲计数器理论中断时间=0xFFFF*1000/STD_CLK_FREQ=131ms
#define STD_CLK_OVTM     (1000/8)          //单位:8ms 约1s 
#define STD_ECLK_OVTM     20000            //单位:1ms 约20s 

//口编号定义

#define PORTA             0   //PORTA 口编号定义
#define PORTB             1   //PORTB 口编号定义
#define PORTC             2   //PORTC 口编号定义
#define PORTD             3   //PORTD 口编号定义
#define PORTE             4   //PORTE 口编号定义
#define PORTF             5   //PORTF 口编号定义
#define PORTG             6   //PORTG 口编号定义
#define PORTH             7   //PORTH 口编号定义
              
//管脚定义针对SH5.948.1200测试板
//PORTA
#define U0RX              GPIO_PIN_0	  //UART0接收
#define U0TX              GPIO_PIN_1	  //UART0发送
#define SSICLK            GPIO_PIN_2   //SSICLK HD7279 时钟
#define SSIFSS            GPIO_PIN_3   //SSIFSS HD7279 片选
#define DISP_RST          GPIO_PIN_4   //SSIRX  HD7279 复位
#define SSITX             GPIO_PIN_5   //SSITX  HD7279 接收
#define JZ_IN             GPIO_PIN_6   //CCP1高稳晶振输入接口
#define FH_IN             GPIO_PIN_7	  //CCP4标准表高频输入
//PORTB
#define I2C_WP            GPIO_PIN_0   //I2C写保护
#define SZ_MC             GPIO_PIN_1   //CCP2时钟脉冲
#define I2C_SCL           GPIO_PIN_2   //I2C模块SCL
#define I2C_SDA           GPIO_PIN_3   //I2C模块SDA
#define CS5460A_SDO       GPIO_PIN_4   //5460A的SDO引脚,PB4
#define CS5460A_CS        GPIO_PIN_5   //5460A的CS引脚,PB5
#define CS5460A_MODE      GPIO_PIN_6   //5460A的MODE引脚,PB6
#define TRST              GPIO_PIN_7   //调试TRST

//PORTC
#define TCK               GPIO_PIN_0   //调试TCK SWCLK SW调试
#define TMS               GPIO_PIN_1   //调试TMS SWDIO SW调试
#define TDI               GPIO_PIN_2   //调试TDI
#define TDO               GPIO_PIN_3   //调试TDO SWO   SW调试
#define PWM_DAC	          GPIO_PIN_4   //CCP5 PWM模拟输出
#define KEY_IN            GPIO_PIN_5   //按键输入
#define NY_IN             GPIO_PIN_6   //* 改变 NY 耐压结果输入

//PORTD                       
#define CANRX             GPIO_PIN_0   //CAN接收
#define CANTX             GPIO_PIN_1   //CAN发送
#define U1RX              GPIO_PIN_2   //UART1接收
#define U1TX              GPIO_PIN_3   //UART1发送
#define CS5460A_SCLK      GPIO_PIN_4   //5460A的SCLK引脚,PD4
#define CS5460A_RESET     GPIO_PIN_5   //5460A的RESET引脚,PD5
#define CS5460A_INT       GPIO_PIN_6   //5460A的INT引脚,PD6
#define CS5460A_SDI       GPIO_PIN_7   //5460A的SDO引脚,PD7

//PORTE                       
#define P3P4SelOut3       GPIO_PIN_0   //表位3，接线选择
#define P3P4SelOut2       GPIO_PIN_1   //表位2，接线选择
#define P3P4SelOut1       GPIO_PIN_2   //表位1，接线选择
#define CHNL1_Sel         GPIO_PIN_3   //表位1功耗接入

//PORTF                       
#define GOG_KZ            GPIO_PIN_0   //被检表脉冲共高共低选择控制
#define MC_PN_KZ          GPIO_PIN_1   //改变被检表电子脉冲输入控制 控制正反向 Positive(0) or Negative(1)
#define MC_WV_KZ          GPIO_PIN_2   //改变被检表电子脉冲输入控制 控制有无功 Watt(0) or Var(1)
#define XL_MC             GPIO_PIN_3   //     改变* 需量周期输入
#define TQ_MC             GPIO_PIN_4   //备用 改变* 时段投切脉冲
#define HZ_MC             GPIO_PIN_5   //备用 改变* 合闸脉冲
#define TX_MC             GPIO_PIN_6   //备用 改变* 通信指示
#define WDI               GPIO_PIN_7   //输出 外置看门狗信号

//PORTG                       
#define BW       (GPIO_PIN_0|GPIO_PIN_1| \
                  GPIO_PIN_2|GPIO_PIN_3| \
                  GPIO_PIN_4|GPIO_PIN_5| \
                  GPIO_PIN_6|GPIO_PIN_7)

//PORTH   
#define Phase_Sel_K1      GPIO_PIN_0    //功耗测试相别选择1
#define Phase_Sel_K2      GPIO_PIN_1    //功耗测试相别选择2
#define CHNL2_Sel         GPIO_PIN_2    //表位2功耗接入
#define CHNL3_Sel         GPIO_PIN_3    //表位3功耗接入
//I/O口宏定义
//PORTA
#define DISP_RST_EN      GPIOPinWrite(GPIOA,DISP_RST,0);       //复位管脚置0
#define DISP_RST_DN      GPIOPinWrite(GPIOA,DISP_RST,DISP_RST);//复位管脚置1

//PORTB
#define I2C_WP_H         GPIOPinWrite(GPIOB,I2C_WP,I2C_WP)     //I2C写保护开启
#define I2C_WP_L         GPIOPinWrite(GPIOB,I2C_WP,0)          //I2C写保护关闭
#define CS5460A_CS_H     GPIOPinWrite(GPIOB,CS5460A_CS,CS5460A_CS)         //释放CS5460A。
#define CS5460A_CS_L     GPIOPinWrite(GPIOB,CS5460A_CS,0)          //片选CS5460A
#define CS5460A_MODE_H   GPIOPinWrite(GPIOB,CS5460A_MODE,CS5460A_MODE) //向CS5460A的MODE引脚发送高电平
#define CS5460A_MODE_L   GPIOPinWrite(GPIOB,CS5460A_MODE,0)       //向CS5460A的MODE引脚发送低电平
#define CS5460A_SDO_B    GPIOPinRead(GPIOB,CS5460A_SDO)        //读取CS5460A的SDO引脚的电平

//PORTC
//PORTD 
#define CS5460A_RESET_H  GPIOPinWrite(GPIOD,CS5460A_RESET,CS5460A_RESET)//向CS5460A的RESET引脚发送高电平
#define CS5460A_RESET_L  GPIOPinWrite(GPIOD,CS5460A_RESET,0)//向CS5460A的RESET引脚发送低电平
#define CS5460A_SDI_H    GPIOPinWrite(GPIOD,CS5460A_SDI,CS5460A_SDI)//向CS5460A的SDI引脚发送高电平
#define CS5460A_SDI_L    GPIOPinWrite(GPIOD,CS5460A_SDI,0)//向CS5460A的SDI引脚发送低电平
#define CS5460A_SCLK_H   GPIOPinWrite(GPIOD,CS5460A_SCLK,CS5460A_SCLK)//向CS5460A的SCLK引脚发送高电平
#define CS5460A_SCLK_L   GPIOPinWrite(GPIOD,CS5460A_SCLK,0)        //向CS5460A的SCLK引脚发送低电平

//PORTE，PORTH功耗测试选择
#define Sel_Channel3     GPIOPinWrite(GPIOH,CHNL3_Sel,0)            //选择功耗测试3号通道
#define Sel_Channel2     GPIOPinWrite(GPIOH,CHNL2_Sel,0)            //选择功耗测试2号通道
#define Sel_Channel1     GPIOPinWrite(GPIOE,CHNL1_Sel,0)            //选择功耗测试1号通道
#define P3_Sel1          GPIOPinWrite(GPIOE,P3P4SelOut1,0)          //选择功耗测试1号通道P3接线方式
#define P4_Sel1          GPIOPinWrite(GPIOE,P3P4SelOut1,P3P4SelOut1)//选择功耗测试1号通道P4接线方式（默认）
#define P3_Sel2          GPIOPinWrite(GPIOE,P3P4SelOut2,0)          //选择功耗测试2号通道P3接线方式
#define P4_Sel2          GPIOPinWrite(GPIOE,P3P4SelOut2,P3P4SelOut2)//选择功耗测试2号通道P4接线方式（默认）
#define P3_Sel3          GPIOPinWrite(GPIOE,P3P4SelOut3,0)          //选择功耗测试3号通道P3接线方式
#define P4_Sel3          GPIOPinWrite(GPIOE,P3P4SelOut3,P3P4SelOut3)//选择功耗测试3号通道P4接线方式（默认）
#define CNCL_CHNL_Sel1   GPIOPinWrite(GPIOE,CHNL1_Sel,CHNL1_Sel)    //释放功耗测试通道
#define CNCL_CHNL_Sel2   GPIOPinWrite(GPIOH,CHNL2_Sel|CHNL3_Sel,CHNL2_Sel|CHNL3_Sel)
#define Sel_Phase_NO     GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,0)                         //不测试任何相别
#define Sel_Phase_A      GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,Phase_Sel_K1)              //测试A相功耗
#define Sel_Phase_B      GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,Phase_Sel_K2)              //测试B相功耗
#define Sel_Phase_C      GPIOPinWrite(GPIOH,Phase_Sel_K1|Phase_Sel_K2,Phase_Sel_K1|Phase_Sel_K2) //测试C相功耗

//PORTF
#define POS_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,0)                //选择正向有功脉冲
#define NEG_Watt_SEL   GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_PN_KZ)         //选择反向有功脉冲
#define POS_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ)         //选择正向无功脉冲
#define NEG_Var_SEL    GPIOPinWrite(GPIOF,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ|MC_PN_KZ)//选择反向无功脉冲
#define DOWN_JOIN      GPIOPinWrite(GPIOF,GOG_KZ,GOG_KZ)       //共低端 发射极连在一起 E
#define UP_JOIN        GPIOPinWrite(GPIOF,GOG_KZ,0)            //共高端 集电极连在一起 C

#define TX_MC_ON       GPIOPinWrite(GPIOF,TX_MC,TX_MC)         //通信指示灯亮
#define TX_MC_OFF      GPIOPinWrite(GPIOF,TX_MC,0)             //通信指示灯亮
/*
#define RED_485_EN     GPIOPinWrite(GPIOF,TXXZ_MC,TXXZ_MC)     //输出 通信选择 红外通信 AB第二通道切换
#define RED_485_DN     GPIOPinWrite(GPIOF,TXXZ_MC,0)           //输出 通信选择 AB第二通道切换 默认第二通道
*/
#define WDI_HIGH       GPIOPinWrite(GPIOF,WDI,WDI)     //输出 外部看门狗复位         
#define WDI_LOW        GPIOPinWrite(GPIOF,WDI,0)           //输出   
#define WDI_CHANGE     GPIOPinWrite(GPIOF,WDI,~GPIOPinRead(GPIOF,WDI))
#define CS5460A_RW_Delay  80
//CAN缓冲区长度定义
#define CAN_SDILEN		 50      //CAN短数据接收缓冲区长度	每个结构体长16字节 8字节有效数据
#define CAN_SDOLEN		 50      //CAN短数据发送缓冲区长度	每个结构体长16字节 8字节有效数据
                       
#define CAN_LDILEN		 20      //CAN长数据接收缓冲区长度	每个结构体长16字节 8字节有效数据
#define CAN_LDOLEN		 10      //CAN长数据发送缓冲区长度	每个结构体长16字节 8字节有效数据

//标准表                    
//标准表最大接收帧数        
#define SMTR_MAX_RFRAM      16     //标准表最大接收帧数
//标准表 接收缓冲区总长度      
#define SMTR_ILEN           4096   //COM2接收缓冲区长度	总长度 为了传输波形数据(表采集)
//标准表最大发送帧数        
#define SMTR_MAX_TFRAM      50     //标准表最大发送帧数
//标准表 发送缓冲区总长度   
#define SMTR_OLEN           512    //COM2发送缓冲区长度 	总长度

//CAN 波特率设置 与CANBitClkSettings[]中对应
#define CANBAUD_100K   0
#define CANBAUD_125K   1
#define CANBAUD_250K   2
#define CANBAUD_500K   3
#define CANBAUD_1M     4
#define CANBAUD	       CANBAUD_500K
//误差单元短CAN帧指令分块 每256个命令为一组
//原误差板命令								          //0 1 2 3  预留给长数据处理
#define  CAN_ICMD_CHK     0                     //接收查询信息命令ID
#define  CAN_ICMD_SETST   CAN_ICMD_CHK+1        //设置单三相命令
#define  CAN_ICMD_PMTR    CAN_ICMD_SETST+1      //设置表位对应关系
#define  CAN_ICMD_MEA     CAN_ICMD_PMTR+1       //开始测量功耗命令
#define  CAN_ICMD_XUP     CAN_ICMD_MEA+1        //修正电压有功功耗
#define  CAN_ICMD_XUS     CAN_ICMD_XUP+1        //修正电压视在功耗
#define  CAN_ICMD_XIS     CAN_ICMD_XUS+1        //修正电流视在功耗
#define  CAN_ICMD_VER     CAN_ICMD_XIS+1        //查版本号
#define  CAN_ICMD_SOLID   CAN_ICMD_VER+1        //固化数据
#define  CAN_ICMD_BOOT    CAN_ICMD_SOLID+1      //进入引导程序
#define  CAN_ICMD_TYPE    CAN_ICMD_BOOT+1       //接线选择
//功耗单元回送数据命令                           
#define  CAN_OCMD_CHK     0     //输出查询信息
#define  CAN_OCMD_MEA     1     //输出测量数据
#define  CAN_OCMD_MEAB    2     //输出测量数据(保留)
#define  CAN_OCMD_VER     3     //输出版本号
//CS5460A
#define  U_MEA_CNT    5         //电压测量次数
#define  I_MEA_CNT    5         //电流测量次数
#define  UP_TYP     0x00        //
#define  US_TYP     0x01        //
#define  IS_TYP     0x02        //
#define  U_TYP      0x03        //
#define  I_TYP      0x04        //
#define  OU_TYP     0x05        //
#define  OI_TYP     0x06        //

//串口通讯命令
#define  UART_ICMD_CHK     0                     //接收查询信息命令ID
#define  UART_ICMD_SETST   UART_ICMD_CHK+1       //设置单三相命令
#define  UART_ICMD_MEA     UART_ICMD_SETST+1     //设置表位对应关系
#define  UART_ICMD_PMTR    UART_ICMD_MEA+1       //开始测量功耗命令
#define  UART_ICMD_XUP     UART_ICMD_PMTR+1      //修正电压有功功耗
#define  UART_ICMD_XUS     UART_ICMD_XUP+1       //修正电压视在功耗
#define  UART_ICMD_XIS     UART_ICMD_XUS+1       //修正电流视在功耗
#define  UART_ICMD_XU      UART_ICMD_XIS+1       //修正电压
#define  UART_ICMD_XI      UART_ICMD_XU+1       //修正电流
#define  UART_ICMD_XOU     UART_ICMD_XI+1       //修正电压零点
#define  UART_ICMD_XOI     UART_ICMD_XOU+1       //修正电流零点
#define  UART_ICMD_TYPE    UART_ICMD_XOI+1       //接线方式
#define  UART_ICMD_VER     UART_ICMD_TYPE+1      //查版本号
#define  UART_ICMD_SOLID   UART_ICMD_VER+1       //固化数据
#define  UART_ICMD_BOOT    UART_ICMD_SOLID+1     //进入引导程序 
#define  UART_ICMD_CRT_A   UART_ICMD_BOOT+1      //接收A相电流有效值 
#define  UART_ICMD_CRT_B   UART_ICMD_CRT_A+1     //接收B相电流有效值
#define  UART_ICMD_CRT_C   UART_ICMD_CRT_B+1     //接收C相电流有效值
#define  UART_ICMD_CRT_S   UART_ICMD_CRT_C+1     //接收单相电流有效值

//接线方式定义
#define WIRE_P1          '0'      //接线方式 单相有功
#define WIRE_P4          '1'      //接线方式 三相四线有功
#define WIRE_P3_2        '2'      //接线方式 三相三线有功
#define WIRE_Q4_3        '3'      //接线方式 三相四线无功90度三元件无功
#define WIRE_Q3_60       '4'      //接线方式 三相三线移相60度无功 60度两元件无功
#define WIRE_Q3_90       '5'      //接线方式 三相三线跨线无功90度两元件无功
#define WIRE_Q4_R        '6'      //接线方式 三相四线真无功
#define WIRE_Q3_R        '7'      //接线方式 三相三线真无功
#define WIRE_P3_3        '8'      //接线方式 三相三线有功 UA UB UC 按三相四线输出UB对地不为0
#define WIRE_Q3_2        '9'      //接线方式 三相三线无功 输出按三相三线输出 UB接U0
#define WIRE_Q3_CT       ':'      //接线方式 三相三线人工中心点无功
#define WIRE_Q1          ';'      //单相方式 无功

#define  UART_TEMP_STR_LEN      64

#define Phase_A    0x01
#define Phase_B    0x02
#define Phase_C    0x03
void Init_Ram(void);            // RAM和变量初始化
void Solid_Mtr_Tab(void);	      //固化表位号 即RAM->FLASH
/*****************************************************************************
* 处理CAN总线状态
* 处理超时
*****************************************************************************/
void Proc_CAN_STS(void);        
//固化参数值 即RAM->FLASH
void Solid_Save_Tab(void);
void Check_SMtr_Tab_Sts(void);
/****************************************************************************
* 处理COM0接收缓冲区                                                                                   
****************************************************************************/  
void Proc_COM0_IBuf(void);                                                   
/****************************************************************************
* 处理COM0发送缓冲区                                               
****************************************************************************/
void Proc_COM0_OBuf(void);
