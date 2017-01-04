/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_flash.c
;* Author             : 张力阵
;* flash 驱动库
*******************************************************************************/

#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"

/*****************************************************************************
* 读US重装值 微秒 该寄存器产生US
* US重装值=(sysclk/1000000)-1
*****************************************************************************/
u32 FlashUsecGet(void)
{
    return(FLASH->USECRL + 1);
}
/*****************************************************************************
* 设置us 产生寄存器 微秒 该寄存器产生US
* US重装值=(sysclk/1000000)-1
*****************************************************************************/
void FlashUsecSet(u32 ulClocks)
{
    FLASH->USECRL = ulClocks - 1;
}

/*****************************************************************************
* FLASH 擦除
* 入口:ulAddress 要擦除的地址 1k对齐
* 出口:0 擦除完成 -1 擦除失败
*****************************************************************************/
s32 FlashErase(u32 ulAddress)
{
    ASSERT(!(ulAddress & (FLASH_ERASE_SIZE - 1)));

    FLASH->FCMISC = FLASH_FCMISC_AMISC;             //清除访问错误中断

    FLASH->FMA = ulAddress;                         //设置地址
    FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_ERASE; //页擦除

    while(FLASH->FMC & FLASH_FMC_ERASE)
     {                                               //查看页擦除命令是否写入
     }
    if(FLASH->FCRIS & FLASH_FCRIS_ARIS)
     return(-1);                                    //擦除失败
    return(0);                                      //擦除成功
}
/*****************************************************************************
* FLASH 编程
* 入口:ulAddress 要编程的起始地址 4字节对齐
* 出口:0 编程完成 -1 编程失败
*****************************************************************************/
s32 FlashProgram(u32 *pulData, u32 ulAddress,u32 ulCount)
{
    ASSERT(!(ulAddress & 3));
    ASSERT(!(ulCount & 3));
    FLASH->FCMISC = FLASH_FCMISC_AMISC;             //清除访问错误中断
    while(ulCount)
     {
      FLASH->FMA = ulAddress;                     //设置编程地址  
      FLASH->FMD = *pulData;                      //要编程的数据
      FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;//启动编程
      while(FLASH->FMC & FLASH_FMC_WRITE)         //等待编程有效
       {
       }
      pulData++;
      ulAddress += 4;
      ulCount -=4;
     }
    if(FLASH->FCRIS & FLASH_FCRIS_ARIS)             //flash 访问错误
     return(-1);                                    //编程失败
    return(0);                                      //编程成功
}
/*****************************************************************************
* 获取FLASH 保护状态
* 入口:ulAddress 地址页 2k对齐 
* 返回 保护状态 FlashReadWrite FlashReadOnly FlashExecuteOnly
*****************************************************************************/
tFlashProtection FlashProtectGet(u32 ulAddress)
{
    u32 ulFMPRE, ulFMPPE;
    u32 ulBank;
    ASSERT(!(ulAddress & (FLASH_PROTECT_SIZE - 1)));

    ulBank = (((ulAddress / FLASH_PROTECT_SIZE) / 32) % 4); //可以分为4个64k bank
    ulAddress &= ((FLASH_PROTECT_SIZE * 32) - 1);

    ulFMPRE = FLASH->FMPRE[ulBank];
    ulFMPPE = FLASH->FMPPE[ulBank];

    if(CLASS_IS_SANDSTORM && (REVISION_IS_C1 || REVISION_IS_C2))
     ulFMPRE |= (FLASH_FMP_BLOCK_31 | FLASH_FMP_BLOCK_30);
    
    switch((((ulFMPRE >> (ulAddress / FLASH_PROTECT_SIZE)) & FLASH_FMP_BLOCK_0) << 1) |
           ((ulFMPPE >> (ulAddress / FLASH_PROTECT_SIZE)) & FLASH_FMP_BLOCK_0))
     {
      case 0:
      case 1:
       return(FlashExecuteOnly);	 //只能执行
      case 2:
       return(FlashReadOnly);		 //只读
      case 3:
      default:
       return(FlashReadWrite);	 //读写允许
    }
}

/*****************************************************************************
* 设置FLASH 保护
* 入口:ulAddress 地址页 2k对齐 
* eProtect 保护状态 FlashReadWrite FlashReadOnly FlashExecuteOnly
* 返回设置结果 0 保护设置成功 -1:保护设置失败
*****************************************************************************/
s32 FlashProtectSet(u32 ulAddress, tFlashProtection eProtect)
{
    u32 ulProtectRE, ulProtectPE;
    u32 ulBank;

    ASSERT(!(ulAddress & (FLASH_PROTECT_SIZE - 1)));
    ASSERT((eProtect == FlashReadWrite) || (eProtect == FlashReadOnly) ||
           (eProtect == FlashExecuteOnly));
    ulAddress /= FLASH_PROTECT_SIZE;
    ulBank = ((ulAddress / 32) % 4);
    ulAddress %= 32;
    ulProtectRE = FLASH->FMPRE[ulBank];
    ulProtectPE = FLASH->FMPPE[ulBank];
    if(CLASS_IS_SANDSTORM && (REVISION_IS_C1 || REVISION_IS_C2))
     {
      if((ulAddress >= 30) && (eProtect == FlashExecuteOnly))
       return(-1);
         
     }
    switch(eProtect)
     {
      case FlashExecuteOnly:
       {
        ulProtectRE &= ~(FLASH_FMP_BLOCK_0 << ulAddress);
        ulProtectPE &= ~(FLASH_FMP_BLOCK_0 << ulAddress);
        break;
       }
      
      case FlashReadOnly:
       {
        if(((ulProtectRE >> ulAddress) & FLASH_FMP_BLOCK_0) !=
           FLASH_FMP_BLOCK_0)
          return(-1);
        ulProtectPE &= ~(FLASH_FMP_BLOCK_0 << ulAddress);
        break;
       }
      case FlashReadWrite:
      default:
       {
        if((((ulProtectRE >> ulAddress) & FLASH_FMP_BLOCK_0) != FLASH_FMP_BLOCK_0) ||
           (((ulProtectPE >> ulAddress) & FLASH_FMP_BLOCK_0) != FLASH_FMP_BLOCK_0))
         return(-1);
        return(0);
       }
     }
    if(CLASS_IS_SANDSTORM && (REVISION_IS_C1 || REVISION_IS_C2))
     {
      ulProtectRE &= ~(FLASH_FMP_BLOCK_31 | FLASH_FMP_BLOCK_30);
      ulProtectRE |= (FLASH->FMPRE[ulBank] &
                     (FLASH_FMP_BLOCK_31 | FLASH_FMP_BLOCK_30));
     }
    FLASH->FMPRE[ulBank] = ulProtectRE;
    FLASH->FMPPE[ulBank] = ulProtectPE;
    return(0);
}

/*****************************************************************************
* 保存FLASH 保护设置
* 返回设置结果 0 保护设置成功 -1:保护设置失败
*****************************************************************************/
s32 FlashProtectSave(void)
{
    int ulTemp, ulLimit;
    ulLimit = CLASS_IS_SANDSTORM ? 2 : 8;
    for(ulTemp = 0; ulTemp < ulLimit; ulTemp++)
     {
      FLASH->FMA = ulTemp;
      FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_COMT;
      while(FLASH->FMC & FLASH_FMC_COMT)
       {
       }
     }
    return(0);
}
/*****************************************************************************
* 读用户一次性设置字
* 返回设置结果 0 读成功 -1:读失败
*****************************************************************************/
s32 FlashUserGet(u32 *pulUser0, u32 *pulUser1)
{
    ASSERT(pulUser0 != 0);
    ASSERT(pulUser1 != 0);
    if(CLASS_IS_SANDSTORM)
     return(-1);
    *pulUser0 = FLASH->USERREG[0];
    *pulUser1 = FLASH->USERREG[1];
    return(0);
}

/*****************************************************************************
* 写用户一次性设置字 慎用 一次性
* 入口: ulUser0	ulUser1
* 返回设置结果 0 写成功 -1:写失败
*****************************************************************************/
s32 FlashUserSet(u32 ulUser0, u32 ulUser1)
{
    if(CLASS_IS_SANDSTORM)
     return(-1);
    FLASH->USERREG[0] = ulUser0;
    FLASH->USERREG[1] = ulUser1;
    return(0);
}

/*****************************************************************************
* 保存用户一次性设置字 慎用 一次性
* 返回保存结果 0 保存成功 -1:保存失败
*****************************************************************************/
s32 FlashUserSave(void)
{
    if(CLASS_IS_SANDSTORM)
     return(-1);
    FLASH->FMA = 0x80000000;
    FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_COMT;
    while(FLASH->FMC & FLASH_FMC_COMT)
     {
     }
    FLASH->FMA = 0x80000001;
    FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_COMT;
    while(FLASH->FMC & FLASH_FMC_COMT)
     {
     }
    return(0);
}

/*****************************************************************************
* 设置flash 中断服务函数入口
* pfnHandler flash 中断处理函数
* 同时NVIC中断使能
*****************************************************************************/
void FlashIntRegister(void (*pfnHandler)(void))
{
    IntRegister(INT_FLASH, pfnHandler);
    IntEnable(INT_FLASH);
}

/*****************************************************************************
* 撤销flash 中断服务函数入口
* 同时中断禁能
*****************************************************************************/
void FlashIntUnregister(void)
{
    IntDisable(INT_FLASH);
    IntUnregister(INT_FLASH);
}

/*****************************************************************************
* 设置 FLASH控制器中断使能状态
*****************************************************************************/
void FlashIntEnable(u32 ulIntFlags)
{
    FLASH->FCIM |= ulIntFlags;
}

/*****************************************************************************
* 设置 FLASH控制器中断禁能状态
*****************************************************************************/
void FlashIntDisable(u32 ulIntFlags)
{
    FLASH->FCIM &= ~(ulIntFlags);
}

/*****************************************************************************
* 获取中断状态
* bMasked=1 返回屏蔽后的中断状态
* bMasked=0 返回原始的中断状态
*****************************************************************************/
u32	FlashIntGetStatus(u8 bMasked)
{
    if(bMasked)
     return(FLASH->FCMISC);
    else
     return(FLASH->FCRIS);
}

/*****************************************************************************
* 清除中断状态
* 入口:ulIntFlags 清除编程中断状态位还是访问中断状态位
*****************************************************************************/
void FlashIntClear(u32 ulIntFlags)
{
    FLASH->FCMISC = ulIntFlags;
}

