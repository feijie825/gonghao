/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_flash.c
;* Author             : ������
;* flash ������
*******************************************************************************/

#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "define.h"
#include "vari.h"

/*****************************************************************************
* ��US��װֵ ΢�� �üĴ�������US
* US��װֵ=(sysclk/1000000)-1
*****************************************************************************/
u32 FlashUsecGet(void)
{
    return(FLASH->USECRL + 1);
}
/*****************************************************************************
* ����us �����Ĵ��� ΢�� �üĴ�������US
* US��װֵ=(sysclk/1000000)-1
*****************************************************************************/
void FlashUsecSet(u32 ulClocks)
{
    FLASH->USECRL = ulClocks - 1;
}

/*****************************************************************************
* FLASH ����
* ���:ulAddress Ҫ�����ĵ�ַ 1k����
* ����:0 ������� -1 ����ʧ��
*****************************************************************************/
s32 FlashErase(u32 ulAddress)
{
    ASSERT(!(ulAddress & (FLASH_ERASE_SIZE - 1)));

    FLASH->FCMISC = FLASH_FCMISC_AMISC;             //������ʴ����ж�

    FLASH->FMA = ulAddress;                         //���õ�ַ
    FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_ERASE; //ҳ����

    while(FLASH->FMC & FLASH_FMC_ERASE)
     {                                               //�鿴ҳ���������Ƿ�д��
     }
    if(FLASH->FCRIS & FLASH_FCRIS_ARIS)
     return(-1);                                    //����ʧ��
    return(0);                                      //�����ɹ�
}
/*****************************************************************************
* FLASH ���
* ���:ulAddress Ҫ��̵���ʼ��ַ 4�ֽڶ���
* ����:0 ������ -1 ���ʧ��
*****************************************************************************/
s32 FlashProgram(u32 *pulData, u32 ulAddress,u32 ulCount)
{
    ASSERT(!(ulAddress & 3));
    ASSERT(!(ulCount & 3));
    FLASH->FCMISC = FLASH_FCMISC_AMISC;             //������ʴ����ж�
    while(ulCount)
     {
      FLASH->FMA = ulAddress;                     //���ñ�̵�ַ  
      FLASH->FMD = *pulData;                      //Ҫ��̵�����
      FLASH->FMC = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;//�������
      while(FLASH->FMC & FLASH_FMC_WRITE)         //�ȴ������Ч
       {
       }
      pulData++;
      ulAddress += 4;
      ulCount -=4;
     }
    if(FLASH->FCRIS & FLASH_FCRIS_ARIS)             //flash ���ʴ���
     return(-1);                                    //���ʧ��
    return(0);                                      //��̳ɹ�
}
/*****************************************************************************
* ��ȡFLASH ����״̬
* ���:ulAddress ��ַҳ 2k���� 
* ���� ����״̬ FlashReadWrite FlashReadOnly FlashExecuteOnly
*****************************************************************************/
tFlashProtection FlashProtectGet(u32 ulAddress)
{
    u32 ulFMPRE, ulFMPPE;
    u32 ulBank;
    ASSERT(!(ulAddress & (FLASH_PROTECT_SIZE - 1)));

    ulBank = (((ulAddress / FLASH_PROTECT_SIZE) / 32) % 4); //���Է�Ϊ4��64k bank
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
       return(FlashExecuteOnly);	 //ֻ��ִ��
      case 2:
       return(FlashReadOnly);		 //ֻ��
      case 3:
      default:
       return(FlashReadWrite);	 //��д����
    }
}

/*****************************************************************************
* ����FLASH ����
* ���:ulAddress ��ַҳ 2k���� 
* eProtect ����״̬ FlashReadWrite FlashReadOnly FlashExecuteOnly
* �������ý�� 0 �������óɹ� -1:��������ʧ��
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
* ����FLASH ��������
* �������ý�� 0 �������óɹ� -1:��������ʧ��
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
* ���û�һ����������
* �������ý�� 0 ���ɹ� -1:��ʧ��
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
* д�û�һ���������� ���� һ����
* ���: ulUser0	ulUser1
* �������ý�� 0 д�ɹ� -1:дʧ��
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
* �����û�һ���������� ���� һ����
* ���ر����� 0 ����ɹ� -1:����ʧ��
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
* ����flash �жϷ��������
* pfnHandler flash �жϴ�����
* ͬʱNVIC�ж�ʹ��
*****************************************************************************/
void FlashIntRegister(void (*pfnHandler)(void))
{
    IntRegister(INT_FLASH, pfnHandler);
    IntEnable(INT_FLASH);
}

/*****************************************************************************
* ����flash �жϷ��������
* ͬʱ�жϽ���
*****************************************************************************/
void FlashIntUnregister(void)
{
    IntDisable(INT_FLASH);
    IntUnregister(INT_FLASH);
}

/*****************************************************************************
* ���� FLASH�������ж�ʹ��״̬
*****************************************************************************/
void FlashIntEnable(u32 ulIntFlags)
{
    FLASH->FCIM |= ulIntFlags;
}

/*****************************************************************************
* ���� FLASH�������жϽ���״̬
*****************************************************************************/
void FlashIntDisable(u32 ulIntFlags)
{
    FLASH->FCIM &= ~(ulIntFlags);
}

/*****************************************************************************
* ��ȡ�ж�״̬
* bMasked=1 �������κ���ж�״̬
* bMasked=0 ����ԭʼ���ж�״̬
*****************************************************************************/
u32	FlashIntGetStatus(u8 bMasked)
{
    if(bMasked)
     return(FLASH->FCMISC);
    else
     return(FLASH->FCRIS);
}

/*****************************************************************************
* ����ж�״̬
* ���:ulIntFlags �������ж�״̬λ���Ƿ����ж�״̬λ
*****************************************************************************/
void FlashIntClear(u32 ulIntFlags)
{
    FLASH->FCMISC = ulIntFlags;
}

