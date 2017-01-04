/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : BitVari.c
;* Author             : 张力阵
;* 位变量定义
;* 位地址空间 0x22000000---0x22000000+RAM大小*32
;* 每个字节(8bit) 对应8个位地址空间内存 
;* 该区中变量只能赋值 0 或 1
;* 变量定义超过256时(最大地址 0x220003fc),要在vari.c中分配更多空间  
;* 绝对地址分配 //__attribute__((section(".ARM.__at_0x22000000")));
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_types.h"
//内存0x20000000 单元 4字节 位定义
u32         SINGLE_OR_THREE ;               //单相台 三相台标志 0: 单相台 1:三相台
u32         NEW_ENG_PLS     ;               //收到新电能脉冲标志
u32         NEW_CLK_PLS     ;               //收到新时钟脉冲标志
u32         NEW_XUL_PLS     ;               //收到新需量脉冲标志 
u32         FIRST_ENG_PLS   ;               //首次测量电能脉冲标志
u32         FIRST_CLK_PLS   ;               //首次测量时钟脉冲标志
u32         FIRST_XUL_PLS   ;               //首次测量需量脉冲标志
u32         OVER_ERR_FLAG   ;               //超差标志
u32         NEW_ENG_DATA    ;               //电能误差新数据
u32         BEEP_EN         ;               //蜂鸣器使能标志
u32         MTR_PLUG        ;               //挂表标志 0:不挂表 1:挂表
u32         GDT_RST_FLAG    ;               //光电头复位标志
u32         NEW_CMD         ;               //新命令标志
u32         HB_BACK_EDGE    ;               //后延对斑标志
u32         SY_START        ;               //失压启动标志
u32         ZZ_PLS_LOADED   ;               //脉冲是否已经预置
u32         HB_CATCHED      ;               //黑斑对准标志
u32         VERIFY_END      ;               //校核常数走字试验结束标志 
u32         SY_ACT          ;               //失压已动作标志 
u32         RISE_FALL_LVL   ;               //上升沿/下降沿中断
u32         ENG_STB_CHK     ;               //电能脉冲稳定检查标志
u32         SCLK_STB_CHK    ;               //综合时钟脉冲稳定检查标志
u32         OCLK_STB_CHK    ;               //单次时钟脉冲稳定检查标志
u32         XUL_STB_CHK     ;               //需量脉冲稳定检测标志
u32         NO_STD_CLK      ;               //标准时钟脉冲存在标志 0:存在 1:不存在
u32         NO_CLK_PLS      ;               //没有检测到时钟脉冲标志
//u32         NO_XUL_PLS      ;               //没有检测到需量脉冲标志
u32         NO_STD_ENG      ;               //没有检测到标准电能脉冲标志
u32         DISP_HL_LVL     ;               //显示脉冲周期标志 0: 显示低电平时间 1:显示高电平时间
u32         PULSE_ZZ_END    ;               //脉冲走字试验结束标志
u32         NEW_WZHZ_PLS    ;               //外置合闸脉冲标志
u32         NEW_NZHZ_PLS    ;               //内置合闸脉冲标志
u32         NEW_TQ_PLS      ;               //新时段投切脉冲标志
u32         NEW_MBJ_PLS     ;               //表报警脉冲标志
u32         NEW_JBJ_PLS     ;               //续流继电器报警标志
u32         REF_JZ_INT      ;               //标准晶振中断标志

u32         TX_ZS_BIT       ;               //通信指示灯标志
u32         SZCLK_SET_TooM_T;               //时钟频率设置太小标志(临时 第一次检测到设置太小)
u32         SZCLK_SET_TooM  ;               //时钟频率设置太小标志
u32         NEW_KEY_FLAG    ;               //新按键标志
u32         PLUG_CHG_FLAG   ;               //挂表改变标志
u32         CD4094_FLAG     ;               //4094改变输出标志                                                   
u32         KEY_PC_PLUG     ;               //按键选择挂表 还是 上位机选择挂表
u32         TZEN            ;               //合闸检测使能命令   
u32         MBJEN           ;               //表报警信号检测使能
u32         GZ_FLAG         ;               //电流旁路继电器故障标志
u32         GZS_FLAG        ;               //临时用故障标志
u32         NZTZ_FLAG       ;               //内置跳闸标志(表位续流继电器)
u32         WZTZ_FLAG       ;               //外置跳闸标志(表跳闸触点)
u32         MASTER_START    ;               //总控中心启动标志                                                   
u32         GDT_INT_REEN    ;               //光电头口中断重启定时器
u32         DZ_INT_REEN     ;               //电子脉冲口中断重启定时器
u32         SZ_INT_REEN     ;               //时钟脉冲管脚重开中断定时
u32         XUL_INT_REEN    ;               //需量口中断重启定时器
u32         TQ_INT_REEN     ;               //投切口中断重启定时器
u32         HZ_INT_REEN     ;               //合闸口中断重启定时器
u32         KEY_INT_REEN    ;               //按键口中断重启定时器
u32         CAN_ERR         ;               //CAN总线错误                                                   
u32         I_JDQ           ;               //电流继电器状态 0: 断开(电流接入) 1: 吸合(电流旁路)
u32         I_JDQ_CHG       ;               //电流继电器状态改变
u32         I_JDQ_EN        ;               //电流继电器使能信号状态
u32         UJDQ_FLAG       ;               //电压继电器状态发生改变标志
u32         ESwitch_FLAG    ;               //电子开关状态发生改变标志
u32         U_JDQ[3]        ;               //电压继电器状态
u32         U_ESwitch[3]    ;               //电压电子开关状态
//内存0x20000004 单元 4字节 位定义

