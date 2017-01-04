/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : vari.h
;* Author             : 张力阵
;* 变量声明
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "StdMtr.h"
typedef struct
{
    u16   Flag;          //保存标志
    u16   CH0_MTR[3];    //通道 表位 对应关系 默认三通道
    float U_PXZ;         //电压有功功率修正值
    float U_SXZ;         //电压视在功率修正值	
    float I_SXZ;         //电流视在功率修正值	
	  float U_XZ;          //电压修正值	
	  float I_XZ;          //电流修正值
		float U_OFFSET;      //电压零点修正
	  float I_OFFSET;      //电流零点修正
    float rset;          //设定电阻值	
	
		float XZDU;					//电压修正点值
		float XZDI;         //电流修正点值
}SAVE_S;
//AD数据结构体
typedef struct
{
    u16 Data          :10;	 //数据
    u16 New_Data      :1;    //有新数据
    u16 Trig          :1;    //触发标志
}Adc_Dat;

//修正值结构体 必须为4byte倍数 否则烧写FLASH有问题 FLASH 烧写地址必须是4的倍数
//IEEE标准浮点数格式高位--->低位 符号(1bit)+指数(8bit)+尾数(23位)
//TI 浮点数格式 低字节(高位--->低位) 指数(8bit)+7bit(尾数高位)+1bit(符号)+8bit(尾数低位)+8bit(尾数最低位)
//接收 TI 浮点数 小端模式
//IEEE 浮点数 大端模式 结构体 
typedef struct
{
    u32 Mant: 23;  //尾数
    u32 Expo: 8 ;  //指数
    u32 Sign: 1 ;  //符号
}IEEE_float_S;
//IEEE 浮点数 大端模式 联合体 
typedef union
{
    float WORD;
    u8    BYTE[4];
    IEEE_float_S BIT;	
}IEEE_float_U ;
typedef struct
{
    u32 Expo: 8 ;  //指数
    u32 Mant1:7 ;  //尾数高7位
    u32 Sign :1 ;  //符号
    u32 Mant2:8 ;  //尾数低8位
    u32 Mant3:8;   //尾数最低8位
}TI_float;
//半字共用体
typedef union
{
    u16 WORD;
    u8  BYTE[2];     
}DB_U;
//字共用体
typedef union
{
    u32 WORD;
    u8  BYTE[4];	
}WORD_U;
//接收缓冲区处理结构体 用于取出一个字符
typedef struct
{
    u8  *IBuf;            //指向接收IBUF的指针
    u16 *IHead;	          //接收指针
    u16 *ITail;           //接收处理指针
    u16  IBLEN;           //接收缓冲区长度
}IBUF_Pr;
//发送缓冲区处理结构体 用于发送一个字符
typedef struct
{
    u8  *OBuf;            //指向接收IBUF的指针
    u16 *OHead;	          //接收指针
    u16 *OTail;           //接收处理指针
    u16 *OCoun;           //命令计数
    u8  *InSend;          //正在发送标志
    u16 OBLEN;            //发送缓冲区长度
}OBUF_Pr;

//定义返回u8 的程序指针
typedef struct
{
    u8 (*Ptr)(void);
}ret_u8_Ptr;
//不返回 的程序指针
typedef struct
{
    void(*Ptr)(u16 a,u16 b);	
}ret_void_Ptr;
//不返回 的程序指针
typedef struct
{
    void(*Ptr)(void);	
}void_void_Ptr;
//不返回 带一个参数输入的程序指针
typedef struct
{
    void (*Ptr)(u8 a);	
}void_u8_Ptr;
//字符指针结构体
typedef struct
{
    const u8 *Ptr;
}CChar_Ptr;
typedef struct
{
    u8 *Ptr;
}Char_Ptr;
typedef struct
{
    vu32 Flag;                //标志
    vu32 Numb;                //表位号
}Mtr_Numb;
//CS5460A计算
typedef union
{
  u8   Chrd[4];
  u32  Intd;
}Int_Char;
typedef union
{
  u8 Chr[4];
  float  Flt;
}Float_Char;
typedef struct 
{
    u32 Break     :1;        //发送终止位
    u32 Parity_En :1;        //校验允许 
    u32 Even      :1;        //偶校验选择
    u32 Stop2     :1;        //2 stop bit
    u32 Fifo_En   :1;        //Fifo 使能
    u32 Data_Len  :2;        //数据位长度 0~5 3~8
    u32 Parity_01 :1;        //奇偶校验固定为0 或1 parity_En=1 (Even=1 校验位为1,Even=0 校验位为0)
    u32 Baud      :18;       //波特率
}UART_PARA;
typedef struct
{
    u32 BAUD     :18;        //波特率
    u32 LEN      :4;         //数据长度 5 6 7 8
    u32 STOP     :2;         //停止位 1 2 3=1.5   
    u32 PARITY   :3;         //校验位 0:'N' 1:'O' 2:'E' 3:'M' 4:'S'
}UART_SET;
//工作模式定义
typedef enum
{
    CAL_ENG_ERR_M   = 0,       //计算被检表电能脉冲误差
    MTR_PLUG_M      = 1,       //选择挂表状态
    CATCH_HB_M      = 2,       //对黑斑
    START_STOP_M    = 3,       //监视电能脉冲计数 启动潜动模式下脉冲计数
    VERIFY_READY_M  = 4,       //准备开始走字 继电器吸合
    VERIFY_START_M  = 5,       //进入走字试验
    VOLTAGE_DOWN_M  = 6,       //进入电压跌落试验
    MEASURE_ENG_M   = 7,       //进入计电能试验 计量被检表电能
    PANZHUAN_ERR_M  = 8,       //进入盘转误差试验
    MEA_CST_M       = 9,       //常数测试试验
    MEA_POWER_D_M   =10,       //功耗测量试验
    MEA_ENG_DUTY_M  =11,       //测量设定脉冲周期和占空比
    PULSE_ZZ_M      =12,       //定脉冲走字试验
    NYSY_M          =13,       //耐压试验 并记录耐压状态下脉冲状态
}MODE;
//CAN长数据处理结构体
typedef struct
{
    vu8 *Ptr;                  //指向缓冲区状态 指针	如:指向Com0_Tx_Sts
}vu8_Ptr;

//CAN长数据处理结构体
typedef struct
{
    u32 *Ptr;                  //指向缓冲区状态 指针	如:指向Com0_Tx_Sts
}u32_Ptr;       

extern u32         ADC_SEQ_Data[8];                //AD 采样序列
extern Adc_Dat     ADC_Data;                       //AD 采样数据
extern u16         ADC_Timer;                      //AD采样定时
extern u16         ADC_SUM_Data;                   //累加数据
extern u8          ADC_SUM_Cnt;                    //AD累加计数
                                                   
extern u8          Current_Save_Tab;               //当前保存值所在表格
extern u8          Save_Tab_Sts[SAVE_TAB_NUMB];    //表位号状态表格

extern SAVE_S      XIUZ;                           //
extern u8          Ext_WDT_Timer;                  //外置看门狗定时器
//ADC数据定义
extern u8          ADC_Start;                       //ADC转换开始标志
//CS5460A数据定义
extern u8          CS5460A_New_Data;                //CS5460A有新数据等待处理
extern u8          U_Mea_Cnt;                       //电压测量次数
extern u8          I_Mea_Cnt;                       //电流测量次数
extern u8          CS5460A_E_Reg[3];                //CS5460A功率寄存器
extern u8          CS5460A_Vrms_Reg[3];             //电压有效值寄存器
extern u8          CS5460A_Irms_Reg[3];             //电流有效值寄存器
extern u8          U_Data_Ready;                    //电压线圈功耗数据准备好标志
extern u8          I_Data_Ready;                    //电流线圈功耗数据准备好标志
extern u8          U_Pr_Str[2];                     //电压有功功率ASC码
extern u8          U_Pm_Str[2];                     //电压视在功率ASC码
extern u8          I_Pm_Str[2];                     //电流视在功率ASC码
extern u8          U_Pr_Str_Com[10];                 //电压有功功率存储空间
extern u8          U_Pm_Str_Com[10];                 //电压视在功率存储空间
extern u8          I_Pm_Str_Com[10];                 //电流视在功率存储空间
extern u8          U_Str_Com[10];                 //电压存储空间
extern u8          I_Str_Com[10];                 //电流存储空间
extern u8          U_Mea_Cal;                       //电压测量可以开始
extern u8          I_Mea_Cal;                       //电流测量可以开始
extern u8          CS5460A_Sts_Reg[3];              //CS5460A状态寄存器
extern float       U_Prms[U_MEA_CNT];               //电压有功功率有效值
extern float       U_Pmax[U_MEA_CNT];               //电压视在功率有效值
extern float       I_Pmax[I_MEA_CNT];               //电流视在功率有效值
extern float       U_P[U_MEA_CNT];               //电压有效值
extern float       I_P[I_MEA_CNT];               //电流有效值
extern float U_current;
extern float I_current;
extern float       Current_Adj_Val[3];              //修正值
extern float       ADC0_Vref;                       //电流采样回路参考电压，3.0V
extern float       CS5460A_Vref;                    //CS5460A的参考电压2.5V
extern float       Current_I_Value;                 //电流有效值5A
extern Int_Char    Temp_Cal1;
extern Float_Char  Temp_Cal2;
extern u16         Timer_CS5460A_INT;               //CS5460A中断定时
//extern u8          Mea_Met_Tab[3];                  //测量通道和测量表位的对应关系
extern u8          Current_Mea_MetNUM;              //当前正在测量的表位号
extern u8          Current_Mea_Channel;             //当前正在测量的通道号
extern u8          Current_Mea_Phase;               //测量的相别
extern u16         Rslt_Send_Timer;                 //测量结果发送定时器，2s发送一次
extern u16         Rslt_Com_Send_Timer;           
extern u8          Current_Wire_Type;               //当前接线方式
//位定义 定义字变量 包含32个位变量	4字节对齐 在LM3S2139.h 中定义
extern u32         SysRCC_CFG;                     //系统RCC配置
extern u8          PLL_ERR_Cnt;                    //锁相环错误计数
extern u8          Set_Uclop;                      //设置电压接入
extern u8          Pwr_Phase;                      //功耗相
extern u8          DXTZ;                           //单相费控表跳闸继电器状态
extern u8          MBJ_DATA;                       //表报警数据
extern u8          WZHZ_DATA[2];                   //外置合闸信号 [0]常闭 [1]常开 
extern u8          GZ_DATA;                        //故障数据
extern u8          NZHZ_DATA;                      //内置合闸信号(电流旁路继电器信号)
extern u8          TIME_ASC[8];                    //时间ASC HH-MM-SS  时分秒
extern u8          PLSGD;                          //脉冲共高共低
extern u8          MFClk_Mode;                     //多功能脉冲输入方式
extern u8          MFClk_Type;                     //多功能脉冲输入类型
extern u8          PLSHC_MODE;                     //脉冲合成方式
extern u8          PLS_QUAD;                       //脉冲象限
extern u8          Disp_Choose;                    //显示选择
extern u8          Disp_Code_Mode;                 //显示译码模式
extern u16         ELEC_PLS_CNT;                   //当前电子脉冲计数
extern u8          ENG_ERR_ASC[9];                 //电能误差ASC码 最多带四位小数点 符号位 +5位数据
extern u8          CLK_FREQ_ASC[9];                //时钟频率
extern u8          DAY_ERR_ASC[9];                 //日计时误差 最多3个小数点
extern u8          XUL_TIME_ASC[9];                //需量周期ASC码 
extern u8          VERIFY_ENG_ASC[9];              //校核超时走字电能ASC表示
extern u8          CURRENT_N_ASC[2];               //当前圈数ASC码
extern u8          CURRENT_ENG_ASC[9];             //当前电能ASC码
extern u8          CURRENT_PLS_ASC[9];             //当前脉冲数ASC码
extern u8          HIGH_LVL_TIME[9];               //高电平时间 第一个字节 'H'
extern u8          LOW_LVL_TIME[9];                //低电平时间 第一个字节 'L'
extern u32         PLS_Lvl_Time[2];                //脉冲电平时间
extern u32         High_Lvl_Time_Tp;               //临时存放脉冲高电平时间
extern u32         Low_Lvl_Time_Tp;                //临时存放脉冲低电平时间
extern u8          High_Lvl_CNT;                   //高电平中断测量计数
extern u8          Low_Lvl_CNT;                    //低电平中断测量计数
extern u8          CMD_DATA;                       //单字节命令数据
extern u8          WIRE_TYPE;                      //接线方式
extern u16         DIVIDE_Coef;                    //分频系数
extern u8          CYCLE_MEA_SEL;                  //周期测量选择
extern u8          CYCLE_MEA_ID;                   //周期测量选择ID编号
extern u8          SY_CNT;                         //失压计数
extern u8          SY_PROCESS;                     //失压进程
extern u8          SY_MODE;                        //失压模式 
extern u8          SY_PHASE;                       //失压相 0:三相同时 1:UA 2:UB 3:UC
extern u8          PZZS;                           //计算盘转误差 多少圈算次误差 
extern u8          PZBZ;                           //盘转和脉冲比值
extern u8          PZ_STD_CNT;                     //盘转试验标准计数

extern u16         CUR_PZ_Cnt_Val;                 //盘转采样点   电子脉冲计数值
extern u16         PRE_PZ_Cnt_Val;                 //上一个采样点 电子脉冲计数值

extern u8          PZ_ERR_ASC[3];                  //盘转误差圈数
extern float       VERIFY_ENG_Kwh;                 //校核常数走字度数
extern u32         VERIFY_PLS_Set;                 //校核常数走字脉冲数
extern u32         CURRENT_VERIFY_PLS;             //当前校核常数走字脉冲数
extern u32         ZZ_PLS_Set;                     //走字脉冲数设置
extern u32         CURRENT_ZZ_PLS;                 //当前走字脉冲数    


//电能脉冲计数累加和总电能                                
extern u32         CURRENT_PLS_CNT;                //脉冲计数 开机后一直累加
extern float       CURRENT_ENG_KWh;                //当前电能
//检高频变量定义                                   
extern float       GP_ENG_CST;                     //被检表高频常数
extern u32         GP_CLK_SET;                     //检高频脉冲数设置
extern u32         GP_CLK_ACT;                     //实际高频计数值
extern u16         GP_RELOAD_TIME;                 //时钟频率重装次数
extern u16         GP_RELOAD_VAL;                  //时钟频率重装值
extern u16         GP_RELOAD_Cnt;                  //高频脉冲重装计数
//标准时钟脉冲计数高位
extern u32         STD_CLK_Cnt;                    //标准时钟计数高位 
//时钟脉冲频率和日计时误差计算变量定义             
extern u8          CLK_MEA_CTL;                    //时钟频率测量控制
extern u16         CLK_STB_RNG;                    //时钟脉冲稳定误差限
extern u16         CLK_RELOAD_TIME_N;              //时钟频率重装次数新值
extern u16         CLK_RELOAD_TIME_O;              //时钟频率重装次数旧值
extern u16         CLK_RELOAD_VAL_N;               //时钟频率重装值新值
extern u16         CLK_RELOAD_VAL_O;               //时钟频率重装值旧值
extern u16         CLK_RELOAD_Cnt;                 //时钟脉冲重装计数
extern u8          CLK_MEA_TIME;                   //时钟频率测量定时 单位:s 秒  10---100s之间 默认20s
extern float       CLK_FREQ_SET;                   //时钟频率设定值 默认1HZ
extern float       CLK_FREQ_INT;                   //时钟频率规格化值 默认1HZ
extern double      CLK_DAY_ERR;                    //日计时误差 
extern double      CLK_ACT_FREQ;                   //实际频率 化整后 2009.3.13 为提高精度 改为双精度浮点数
extern double      STD_CLK_VAL_ONE;                //标准时钟计算值 用于计算频率  单次中断             计算时钟频率 标准值
extern double      STD_CLK_VAL_SUM;                //标准时钟计算值 用于计算频率 CLK_RELOAD_TIME次中断 计算时钟频率 标准值
extern u32         STD_CLK_CNT_ONE;                //标准时钟计数值 每次中断                 应计的标准脉冲数
extern u32         STD_CLK_CNT_SUM;                //标准时钟计数值 总 CLK_RELOAD_TIME次中断 应计的标准脉冲数
extern u32         RAW_CLK_VAL;                    //原始时钟脉冲计数值
extern u32         CUR_CLK_VAL;                    //当前时钟脉冲计数值
extern u32         PRE_CLK_VAL;                    //上次时钟脉冲计数值

extern u32         SCLK_Cnt_OSub;                  //本次计数值 
extern u32         SCLK_Cnt_NSub;                  //上次计数值
extern u32         SCLK_Cnt_RSet_Max;              //合计数改变触发重设时钟频率最大值
extern u32         SCLK_STB_RNG;                   //时钟脉冲稳定误差限(多次)

extern u32         OCLK_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OCLK_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OCLK_Cnt_RSet_Max;              //单次计数改变触发重设时钟频率最大值
extern u32         OCLK_STB_RNG;                   //时钟脉冲稳定误差限(单次)
//需量周期变量定义
extern u8          XUL_MEA_CTL;                    //需量周期测量控制
extern float       XUL_TIME;                       //需量周期
extern u8          XUL_RELOAD_TIME;                //需量脉冲重装次数
extern u8          XUL_RELOAD_Cnt;                 //需量脉冲计数

extern u32         RAW_XUL_VAL;                    //原始需量中断标准时钟脉冲计数值
extern u32         CUR_XUL_VAL;                    //当前需量中断标准时钟脉冲计数值
extern u32         PRE_XUL_VAL;                    //上次需量中断标准时钟脉冲计数值

extern u32         SXUL_Cnt_OSub;                  //本次计数值 
extern u32         SXUL_Cnt_NSub;                  //上次计数值

extern u32         OXUL_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OXUL_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用

//被检表误差计算变量定义                           
extern u8          ENG_CLK_CH;                     //脉冲选择 0:光点头脉冲 1:电子脉冲 2:标准表脉冲
extern MODE        WORK_MODE;                      //工作模式   
extern u16         ENG_STB_RNG;                    //电能稳定误差限   
extern float       STD_ENG_CNT_VAL;                //标准电能脉冲应计个数 = STD_ENG_CST*ACT_N/MTR_ENG_CST
extern float       STD_ENG_CST;                    //标准表高频常数 用于计算误差
extern float       MTR_ENG_CST;                    //本表位脉冲低频常数
extern float       MTR_MIN_CST;                    //所有表位最小脉冲低频常数
extern u8          CURRENT_N;                      //当前圈数
extern u8          ACT_N;                          //实际校验圈数 ACT_N=MTR_ENG_CST*SET_N/MTR_MIN_CST
extern u8          SET_N;                          //设定校验圈数
extern u32         STD_ENG_Cnt;                    //标准时钟计数高位

extern u32         CUR_ENG_VAL;                    //当前电能脉冲计数值
extern u32         PRE_ENG_VAL;                    //上次电能脉冲计数值
extern u32         RAW_ENG_VAL;                    //原始电能脉冲计数值

extern u16         SENG_STB_RNG;                   //时钟脉冲稳定误差限(多次)

extern u32         OEND_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OEND_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用
extern u16         OEND_STB_RNG;                   //时钟脉冲稳定误差限(单次)

extern float       ENG_ERR;                        //电能误差
                                                   
extern u8          Mtr_Numb_ID;                    //表位号
extern u8          Mtr_Numb_Str[3];                //表位号ASC码
//extern u8          Current_Mtr_Tab;                //当前表位号所在表格
//extern u8          Mtr_Tab_Sts[MTR_TAB_NUMB];      //表位号状态表格

extern u32         Sysclk;                         //系统频率

//中断中用到的定时器 用volatile 定义
extern vu32        Timer_1ms;                      //1ms定时器
extern vu32        CLK_Timer;                      //时钟测量定时
extern vu8         Timer_8ms;                      //8ms定时器 扩展定时时间 单字节扩展到2.048s
extern vu8         GDT_Timer;                      //光电头口中断重启定时器
extern vu8         DZ_Timer;                       //电子脉冲口中断重启定时器
extern vu8         SZ_Timer;                       //时钟脉冲管脚重开中断定时
extern vu8         XUL_Timer;                      //需量口中断重启定时器
extern vu8         TQ_Timer;                       //投切口中断重启定时器
extern vu8         HZ_Timer;                       //HZ脉冲重开中断定时
extern vu8         KEY_Timer;                      //按键重启中断定时器
extern vu8         HC165_Timer;                    //HC165采样定时
extern vu8         CD4094_Timer;                   //CD4094输出定时
extern vu8         PLL_CHK_Timer;                  //PLL检查定时
//以下定时用Timer_1ms 计时 精度较高 或时间长于2.048s
extern u32         CYCLE_OLD_Timer;                //周期测量定时器(老)
extern u32         CYCLE_NEW_Timer;                //周期测量定时器(新)
extern u32         CLK_Timer_Max;                  //时钟测量脉冲最大值
extern u32         CLK_Timer_Min;                  //时钟测量脉冲最小值
extern u16         SY_Timer;                       //失压计时
extern u16         NY_SEND_Timer;                  //命令回送定时
extern u16         CAN_STX_OVTimer;                //CAN 短数据发送超时定时器
extern u16         CAN_LTX_OVTimer;                //CAN 长数据发送超时定时器
extern u16         CAN_RX_OVTimer;                 //CAN 总线接收超时定时器
extern u16         WORK_Timer;                     //进入工作模式定时器 单位:ms 最多65.535S
extern u16         STD_ECLK_Timer;                 //标准电能脉冲检测定时
extern u16         MBJ_Send_Timer;                 //表报警数据回送定时
extern u16         WZTZ_Send_Timer;                //外置跳闸数据回送定时
extern u16         NZTZ_Send_Timer;                //内置跳闸数据回送定时
extern u16         XLJDQ_ERR_Timer;                //续流继电器故障报警定时
extern u16         NEW_PLL_Ref_Timer;              //锁相环参考定时器
extern u16         OLD_PLL_Ref_Timer;              //锁相环参考定时器
extern u16         SMtr_RCV_Timer;                 //标准表接收定时
extern u16         SMtr_TRS_Timer;                 //标准表发送定时
extern u16         SMtr_RDATA_Timer;               //标准表读数据定时
extern u16         SMtr_RSTS_Timer;                //标准表读状态定时
extern u16         SMtr_ChangeD_Timer;             //标准表改变定时
extern u16         SMtr_Check_Timer;               //标准表检测定时
extern u16         SMtr_TRS_OVTimer;               //标准表发送超时
//以下定时器 用Timer_8ms 计时
extern u8          Disp_Timer;                     //显示定时处理
extern u8          GDT_RST_Timer;                  //光电头复位延时
extern u8          Com_Rx_Time[UART_NUMB];         //COM 接收定时
extern u8          Com_Rx_OvTm[UART_NUMB];         //UART接收超时处理
extern u8          UJDQ_Timer;                     //电压继电器定时
extern u8          IJDQ_Timer;                     //电流继电器定时
extern u8          ESwitch_Timer;                  //电子开关定时
extern u8          NY_CHK_Timer;                   //耐压查询定时
extern u8          Con_KEY_Timer;                  //连续按键定时
extern u8          Disp_En_Timer;                  //显示使能定时 防止连续写入SSI
extern u8          STD_CLK_Timer;                  //标准时钟脉冲定时
extern u8          SMtr_Set_Timer;                 //标准表设置定时

extern u8          Disp_Buf[8];                    //显示缓冲区
extern UART_SET    Uart_Para[UART_NUMB] ;          //UART 参数
//CAN通信定义                                      
extern u16         CANERR_CNT;                     //CAN错误计数  
extern u8          CAN_LMSG_TX_TYPE;               //当前发送CAN长数据类型
extern u8          CAN_LMSG_RX_TYPE;               //读取接收CAN长数据类型
extern CAN_LMSG_PR CAN_LMSG_RX_Ptr;                //CAN长数据接收结构体指针
extern CAN_LMSG_PR CAN_LMSG_TX_Ptr;                //CAN长数据发送结构体指针
extern CAN_STS_U   CAN_STS;                        //总线状态
extern u8          CAN_LDATA_TX_STS;               //CAN长数据发送状态 0 无定义 'R':已经发出请求 'S' 正在发送数据 'E' 发送结束
extern u8          CAN_NEXT_MSG_IDx;               //CAN下一帧报文的索引号 根据该号判断是否丢失报文
extern u8          CAN_LMSG_TX_STS;                //CAN长帧发送状态
extern u8          CAN_SMSG_TX_STS;                //CAN短帧发送状态
extern CAN_MSG     CAN_MSG_Rx;                     //用于临时接收CAN报文
extern CAN_MSG     CAN_SMSG_Tx;                    //用于临时发送CAN短数据报文
extern CAN_MSG     CAN_LMSG_Tx;                    //用于临时发送CAN长数据报文
extern u8          Echo_Sts;                       //响应状态  0 无定义 'C':收到查询 'S':已经发送回应命令 'A':已经响应
extern u8          CAN_SEND_DTIME;                 //CAN帧发送延时时间
extern u8          CAN_SEND_DELAY;                 //CAN帧发送延时 防止同时竞争总线

extern CAN_MSG     CAN_SDATA_MSG_IBUF[CAN_SDILEN]; //CAN短数据接收指令缓冲区
extern u8          CAN_SDATA_MSG_IHead;            //CAN短数据接收指令缓冲区头指针 接收指针
extern u8          CAN_SDATA_MSG_ITail;            //CAN短数据接收指令缓冲区尾指针 处理指针
extern CAN_MSG     *CAN_MSG_IPtr;                  //CAN短数据帧接收处理指针
extern CAN_MSG     CAN_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN短数据发送指令缓冲区
extern u8          CAN_SDATA_MSG_OHead;            //CAN短数据发送指令缓冲区头指针 接收指针
extern u8          CAN_SDATA_MSG_OTail;            //CAN短数据发送指令缓冲区尾指针 处理指针
extern CAN_MSG     *CAN_MSG_OPtr;                  //CAN短数据帧发送处理指针

extern CAN_MSG     CAN_LDATA_MSG_IBUF[CAN_LDILEN]; //CAN长数据接收指令缓冲区               
extern u8          CAN_LDATA_MSG_IHead;            //CAN长数据接收指令缓冲区头指针 接收指针
extern u8          CAN_LDATA_MSG_ITail;            //CAN长数据接收指令缓冲区尾指针 处理指针
extern u8          CAN_LDATA_SERR_Cnt;             //CAN长数据发送错误计数

//COM2 标准表 帧                                             
extern u8          SMtr_IBuf[SMTR_ILEN];            //COM2接收缓冲区
extern u16         SMtr_IHead;                      //COM2接收缓冲区接收指针
extern u16         SMtr_ITail;                      //COM2接收缓冲区接收处理指针
extern u16         SMtr_RFrm_Ptr[SMTR_MAX_RFRAM];   //接收帧指针 记录接收帧终止位置
extern u8          SMtr_RFrm_Cnt;                   //接收帧数
extern u8          SMtr_RFrm_IHead;                 //帧接收指针
extern u8          SMtr_RFrm_ITail;                 //帧接收处理指针
extern u16         SMtr_RFrm_CLen;                  //当前帧长度(剩余长度)
extern u8          SMtr_Rx_Sts;                     //COM2接收状态     0:空闲状态 其它:接收状态
extern u8          SMtr_Rx_OvTm;                    //COM2接收超时设置
extern u8          SMtr_RFrm_IS_Pr;                 //标准表接收帧正在处理标志
extern u8          SMtr_OBuf[SMTR_OLEN];  					//COM2发送缓冲区
extern u16         SMtr_OHead;                      //COM2发送缓冲区发送指针
extern u16         SMtr_OTail;                      //COM2发送缓冲区发送处理指针
extern u16         SMtr_TFrm_Ptr[SMTR_MAX_TFRAM];   //发送帧指针 记录发送帧终止位置 
extern u8          SMtr_TFrm_Attr[SMTR_MAX_TFRAM];  //帧属性 数据类型
extern u8          SMtr_InSend;                     //COM2数据发送状态 0:空闲状态 其它:正在发送
extern u8          SMtr_TFrm_Cnt;                   //发送帧数
extern u8          SMtr_TFrm_OHead;                 //帧发送指针
extern u8          SMtr_TFrm_OTail;                 //帧发送处理指针
extern u8          SMtr_TFrm_Cmd_Cnt[SMTR_CMD_NUMB];//在标准表发送缓冲区各种命令帧计数
                                                    //目的 只发送最新的设置命令(旧的设置命令不在发送)
                                                    //每条命令都有单独的计数器
                                                    //命令送入缓冲区后计时器加1
                                                    //命令发出或清除后计数器减1
                                                    //启动发送时如果该命令计数器不为1 说明有新的命令 该命令直接清除
extern u8          SMtr_Set_Sts[SMTR_CMD_NUMB];     //标准表设置命令状态 
extern u8          SMtr_TFrm_Cnt;                   //发送帧数
extern u16         SMtr_RFrm_CLen;                  //当前帧长度(剩余长度)
extern u16         SMtr_IHead;                      //COM2接收缓冲区接收指针
extern u16         SMtr_ITail;                      //COM2接收缓冲区接收处理指针
extern u16         SMtr_RFrm_Ptr[SMTR_MAX_RFRAM];   //接收帧指针 记录接收帧终止位置
extern u16         SMtr_OHead;                      //COM2发送缓冲区发送指针
extern u16         SMtr_OTail;                      //COM2发送缓冲区发送处理指针
extern u16         SMtr_TFrm_Ptr[SMTR_MAX_TFRAM];   //发送帧指针 记录发送帧终止位置 
                                                    //目的 只发送最新的设置命令(旧的设置命令不在发送)
                                                    //每条命令都有单独的计数器
                                                    //命令送入缓冲区后计时器加1
                                                    //命令发出或清除后计数器减1
                                                    //启动发送时如果该命令计数器不为1 说明有新的命令 该命令直接清除
extern u16         SMtr_TRS_IntVal;                 //标准表发送定时设置
extern u16         SMtr_RDATA_IntVal;               //标准表读数据定时周期 单位:ms
extern u16         SMtr_RSTS_IntVal;                //标准表读数据定时周期 单位:ms
extern u8          READ_WAVE_BIT;                   //要读取的波形 BIT0:ua BIT1:ub BIT2:uc BIT3:ia BIT4:ib BIT5:ic
extern u8          READ_WAVE_ID;                    //读取波形数据的相别             


extern IBUF_Pr     IBUF_Ptr;                        //接收缓冲区处理结构体
extern OBUF_Pr     OBUF_Ptr;                        //发送缓冲区处理结构体 
extern OBUF_Pr     UART_Ptr[4];                     //串口发送缓冲区处理结构体 可能在中断中调用                                                                   

extern u8          COM0_InSend;                     //COM0数据发送状态 'Y'正在发送数据 其它 不在发送数据
extern u8          COM0_IBuf[COM0_ILEN];            //COM0接收缓冲区
extern u8          COM0_OBuf[COM0_OLEN];            //COM0发送缓冲区
extern u16         COM0_IHead;                      //COM0接收缓冲区头指针 接收指针
extern u16         COM0_ITail;                      //COM0接收缓冲区尾指针 处理指针
extern u16         COM0_ICoun;                      //COM0接收命令计数
extern u16         COM0_OHead;                      //COM0发送缓冲区头指针接收指针
extern u16         COM0_OTail;                      //COM0发送缓冲区尾指针处理指针
extern u16         COM0_OCoun;                      //COM0发送命令计数
extern u16         COM0_STimer;                     //COM0发送定时

extern u8          COM1_InSend;                     //COM1数据发送状态 'Y'正在发送数据 其它 不在发送数据
extern u8          COM1_IBuf[COM1_ILEN];            //COM1接收缓冲区
extern u8          COM1_OBuf[COM1_OLEN];            //COM1发送缓冲区
extern u16         COM1_IHead;                      //COM1接收缓冲区头指针 接收指针
extern u16         COM1_ITail;                      //COM1接收缓冲区尾指针 处理指针
extern u16         COM1_ICoun;                      //COM1接收命令计数
extern u16         COM1_OHead;                      //COM1发送缓冲区头指针接收指针
extern u16         COM1_OTail;                      //COM1发送缓冲区尾指针处理指针
extern u16         COM1_OCoun;                      //COM1发送命令计数
extern u16         COM1_STimer;                     //COM1发送定时

extern u8          Rx_Com[RX_CMD_LEN];                //取出的指令头//com0  com1 com2 公用
extern u8          Rx_Para[RX_PARA_NUM][RX_PARA_LEN]; //取出的指令头 取出的数据 公用 最多带两个数据
extern u8          Para_Numb;                         //参数个数

extern u8          CANT_STR[8];                    //CAN帧字符串临时缓冲区
extern u8          TEMP_STR[30];                   //CAN临时用字符串
extern u8          UART_TEMP_STR[UART_TEMP_STR_LEN];              //UART临时用字符串
extern u8          CHK_OneTime_Flag;                    //接收到一次查询命令标志。
extern u8						CHK_ONETIME_FLAG1;
//位变量定义
extern u32         SINGLE_OR_THREE ;               //单相台 三相台标志 0: 单相台 1:三相台
extern u32         NEW_ENG_PLS     ;               //收到新电能脉冲标志
extern u32         NEW_CLK_PLS     ;               //收到新时钟脉冲标志
extern u32         NEW_XUL_PLS     ;               //收到新需量脉冲标志 
extern u32         FIRST_ENG_PLS   ;               //首次测量电能脉冲标志
extern u32         FIRST_CLK_PLS   ;               //首次测量时钟脉冲标志
extern u32         FIRST_XUL_PLS   ;               //首次测量需量脉冲标志
extern u32         OVER_ERR_FLAG   ;               //超差标志
extern u32         NEW_ENG_DATA    ;               //电能误差新数据
extern u32         BEEP_EN         ;               //蜂鸣器使能标志
extern u32         MTR_PLUG        ;               //挂表标志 0:不挂表 1:挂表
extern u32         GDT_RST_FLAG    ;               //光电头复位标志
extern u32         NEW_CMD         ;               //新命令标志
extern u32         HB_BACK_EDGE    ;               //后延对斑标志
extern u32         SY_START        ;               //失压启动标志
extern u32         ZZ_PLS_LOADED   ;               //脉冲是否已经预置
extern u32         HB_CATCHED      ;               //黑斑对准标志
extern u32         VERIFY_END      ;               //校核常数走字试验结束标志 
extern u32         SY_ACT          ;               //失压已动作标志 
extern u32         RISE_FALL_LVL   ;               //上升沿/下降沿中断
extern u32         ENG_STB_CHK     ;               //电能脉冲稳定检查标志
extern u32         SCLK_STB_CHK    ;               //综合时钟脉冲稳定检查标志
extern u32         OCLK_STB_CHK    ;               //单次时钟脉冲稳定检查标志
extern u32         XUL_STB_CHK     ;               //需量脉冲稳定检测标志
extern u32         NO_STD_CLK      ;               //标准时钟脉冲存在标志 0:存在 1:不存在
extern u32         NO_CLK_PLS      ;               //没有检测到时钟脉冲标志
//extern u32         NO_XUL_PLS      ;               //没有检测到需量脉冲标志
extern u32         NO_STD_ENG      ;               //没有检测到标准电能脉冲标志
extern u32         DISP_HL_LVL     ;               //显示脉冲周期标志 0: 显示低电平时间 1:显示高电平时间
extern u32         PULSE_ZZ_END    ;               //脉冲走字试验结束标志
extern u32         NEW_WZHZ_PLS    ;               //外置合闸脉冲标志
extern u32         NEW_NZHZ_PLS    ;               //内置合闸脉冲标志
extern u32         NEW_TQ_PLS      ;               //新时段投切脉冲标志
extern u32         NEW_MBJ_PLS     ;               //表报警脉冲标志
extern u32         NEW_JBJ_PLS     ;               //续流继电器报警标志
extern u32         REF_JZ_INT      ;               //标准晶振中断标志

extern u32         TX_ZS_BIT       ;               //通信指示等标志
extern u32         SZCLK_SET_TooM_T;               //时钟频率设置太小标志(临时 第一次检测到设置太小)
extern u32         SZCLK_SET_TooM  ;               //时钟频率设置太小标志
extern u32         NEW_KEY_FLAG    ;               //新按键标志
extern u32         PLUG_CHG_FLAG   ;               //挂表改变标志
extern u32         CD4094_FLAG     ;               //4094改变输出标志                                                   
extern u32         KEY_PC_PLUG     ;               //按键选择挂表 还是 上位机选择挂表
extern u32         TZEN            ;               //合闸检测使能命令   
extern u32         MBJEN           ;               //表报警信号检测使能
extern u32         GZ_FLAG         ;               //电流旁路继电器故障标志
extern u32         GZS_FLAG        ;               //临时用故障标志
extern u32         NZTZ_FLAG       ;               //内置跳闸标志(表位续流继电器) 
extern u32         WZTZ_FLAG       ;               //外置跳闸标志(表跳闸触点) 
extern u32         MASTER_START    ;               //总控中心启动标志                                                   
extern u32         GDT_INT_REEN    ;               //光电头口中断重启定时器
extern u32         DZ_INT_REEN     ;               //电子脉冲口中断重启定时器
extern u32         SZ_INT_REEN     ;               //时钟脉冲管脚重开中断定时
extern u32         XUL_INT_REEN    ;               //需量口中断重启定时器
extern u32         TQ_INT_REEN     ;               //投切口中断重启定时器
extern u32         HZ_INT_REEN     ;               //合闸口中断重启定时器
extern u32         KEY_INT_REEN    ;               //按键口中断重启定时器
extern u32         CAN_ERR         ;               //CAN总线错误                                                   
                                                   
extern u32         I_JDQ           ;               //电流继电器状态 0: 断开(电流接入) 1: 吸合(电流旁路)
extern u32         I_JDQ_CHG       ;               //电流继电器状态改变
extern u32         I_JDQ_EN        ;               //电流继电器使能信号状态
extern u32         UJDQ_FLAG       ;               //电压继电器状态发生改变标志
extern u32         ESwitch_FLAG    ;               //电子开关状态发生改变标志
extern u32         U_JDQ[3]        ;               //电压继电器状态
extern u32         U_ESwitch[3]    ;               //电压电子开关状态


//内存0x20000004 单元 4字节 位定义
/****************************************************************************
* CAN短数据帧发送
****************************************************************************/
extern const CAN_MSG CAN_TX_SMSG;

/********************************************************
* IO口基址定义
********************************************************/
extern const u32 PORT_BASE_ADDR_TAB[];
/********************************************************
* 周期测量脉冲输入管脚表格 
* GDT_PLS   '1'      //光电头脉冲 GDT_MC
* DZ_PLS    '2'      //电子脉冲   DZ_MC
* SZ_PLS    '3'      //时钟脉冲   SZ_MC
* XUL_PLS   '4'      //需量脉冲   XL_MC
* TQ_PLS    '5'      //投切脉冲   TQ_MC
* HZ_PLS    '6'      //合闸脉冲   HZ_MC
********************************************************/
extern const u8 CYCLE_PIN_TAB[];
/********************************************************
* 管脚重开定时器 
* 光电头脉冲重开定时器 GDT_MC
* 电子脉冲重开定时器   DZ_MC
* 时钟脉冲重开定时器   SZ_MC
* 需量脉冲重开定时器   XL_MC
* 投切脉冲重开定时器   TQ_MC
* 合闸脉冲重开定时器   HZ_MC
********************************************************/
extern const vu8_Ptr IO_Timer_Tab[];
/********************************************************
* 管脚重开中断标志 
* 光电头脉冲重开定时器 GDT_MC
* 电子脉冲重开定时器   DZ_MC
* 时钟脉冲重开定时器   SZ_MC
* 需量脉冲重开定时器   XL_MC
* 投切脉冲重开定时器   TQ_MC
* 合闸脉冲重开定时器   HZ_MC
********************************************************/
extern const u32_Ptr IO_REEN_TAB[];
/*****************************************************************************
* 串口默认参数列表 MTRCOM LCTCOM
*****************************************************************************/
extern const UART_SET UART_PARA_TAB[UART_NUMB];
/****************************************************************************
* 标准表接收命令头地址表格
****************************************************************************/
extern const CChar_Ptr SMTR_CMD_ADDR_TAB[];
/****************************************************************************
* 标准表接收匹配命令个数表格
****************************************************************************/
extern const u16 SMTR_CMD_NUM_TAB[];
/****************************************************************************
* 标准表每条命令头长度表格
****************************************************************************/
extern const u8 SMTR_CMD_LEN_TAB[];
/****************************************************************************
* COM0接收命令表格
****************************************************************************/
extern const u8 COM0_ICMD_TAB[][8];
/****************************************************************************
* COM1接收命令表格
****************************************************************************/
extern const u8 COM1_ICMD_TAB[][8];

/****************************************************************************
* 各缓冲区设定的接收匹配命令个数表格
****************************************************************************/
extern const u16 CMD_NUM_TAB[];

extern const SAVE_S Save_Tab;

extern const u8 VER_TAB[];
