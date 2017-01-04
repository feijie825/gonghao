/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_mpu.c
;* Author             : 张力阵
;* 存储器保护驱动
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
/*****************************************************************************
* 初始化MPU
*****************************************************************************/
void Init_Mpu(void)
{
    unsigned int bFail = 0;
    unsigned long g_ulMPUFaultCount;
    unsigned long g_ulMMAR;
    unsigned long g_ulFaultStatus;

    //
    // Configure an executable, read-only MPU region for flash.  It is an 8 KB
    // region with the last 1 KB disabled to result in a 7 KB executable
    // region.  This region is needed so that the program can execute from
    // flash.
    //
    MPURegionSet(0, FLASH_BASE,				   //区域 基址
                 MPU_RGN_SIZE_8K |			   //区域大小
                 MPU_RGN_PERM_EXEC |		   //可取指
                 MPU_RGN_PERM_PRV_RO_USR_RO |  //特权:只读 用户:只读
                 MPU_SUB_RGN_DISABLE_7 |	   //子区域7禁能
                 MPU_RGN_ENABLE);			   //区域使能

    //
    // Configure a read-write MPU region for RAM.  It is a 64 KB region.  There
    // is a 8 KB sub-region in the middle that is disabled in order to open up
    // a hole in which different permissions can be applied.
    //
    MPURegionSet(1, SRAM_BASE,
                 MPU_RGN_SIZE_64K |
                 MPU_RGN_PERM_NOEXEC |
                 MPU_RGN_PERM_PRV_RW_USR_RW |
                 MPU_SUB_RGN_DISABLE_4 |
                 MPU_RGN_ENABLE);

    //
    // Configure a read-only MPU region for the 8 KB of RAM that is disabled in
    // the previous region.  This region is used for demonstrating read-only
    // permissions.
    //
    MPURegionSet(2, SRAM_BASE + 0x8000,
                 MPU_RGN_SIZE_2K |
                 MPU_RGN_PERM_NOEXEC |
                 MPU_RGN_PERM_PRV_RO_USR_RO |
                 MPU_RGN_ENABLE);

    //
    // Configure a read-write MPU region for peripherals.  The region is 512 KB
    // total size, with several sub-regions disabled to prevent access to areas
    // where there are no peripherals.  This region is needed because the
    // program needs access to some peripherals.
    //
    MPURegionSet(3, 0x40000000,
                 MPU_RGN_SIZE_512K |
                 MPU_RGN_PERM_NOEXEC |
                 MPU_RGN_PERM_PRV_RW_USR_RW |
                 MPU_SUB_RGN_DISABLE_1 |
                 MPU_SUB_RGN_DISABLE_6 |
                 MPU_SUB_RGN_DISABLE_7 |
                 MPU_RGN_ENABLE);

    //
    // Configure a read-write MPU region for access to the NVIC.  The region is
    // 4 KB in size.  This region is needed because NVIC registers are needed
    // in order to control the MPU.
    //
    MPURegionSet(4, NVIC_BASE,
                 MPU_RGN_SIZE_4K |
                 MPU_RGN_PERM_NOEXEC |
                 MPU_RGN_PERM_PRV_RW_USR_RW |
                 MPU_RGN_ENABLE);

    //
    // Need to clear the NVIC fault status register to make sure there is no
    // status hanging around from a previous program.
    //
    g_ulFaultStatus = HWREG(NVIC_FAULT_STAT);
    HWREG(NVIC_FAULT_STAT) = g_ulFaultStatus;

    //
    // Enable the MPU fault.
    //
    IntEnable(FAULT_MPU);

    //
    // Enable the MPU.  This will begin to enforce the memory protection
    // regions.  The MPU is configured so that when in the hard fault or NMI
    // exceptions, a default map will be used.  Neither of these should occur
    // in this example program.
    //
    MPUEnable(MPU_CONFIG_HARDFLT_NMI);

    //
    // Attempt to write to the flash.  This should cause a protection fault due
    // to the fact that this region is read-only.
    //
 //  OSRAM128x64x4StringDraw("Check flash write", 0, 16, 8);
    g_ulMPUFaultCount = 0;
    HWREG(0x100) = 0x12345678;

    //
    // Verify that the fault occurred, at the expected address.
    //
    if((g_ulMPUFaultCount == 1) &&
       (g_ulFaultStatus == 0x82) &&
       (g_ulMMAR == 0x100))
    {
	    bFail = 0;
    //    OSRAM128x64x4StringDraw(" OK", 108, 16, 15);
    }
    else
    {
        bFail = 1;
    //    OSRAM128x64x4StringDraw("NOK", 108, 16, 15);
    }

    //
    // The MPU was disabled when the previous fault occurred, so it needs to be
    // re-enabled.
    //
    MPUEnable(MPU_CONFIG_HARDFLT_NMI);

    //
    // Attempt to read from the disabled section of flash, the upper 1 KB of
    // the 8 KB region.
    //
    //OSRAM128x64x4StringDraw("Check flash read", 0, 24, 8);
    g_ulMPUFaultCount = 0;
    HWREG(0x1C10);

    //
    // Verify that the fault occurred, at the expected address.
    //
    if((g_ulMPUFaultCount == 1) &&
       (g_ulFaultStatus == 0x82) &&
       (g_ulMMAR == 0x1C10))
    {
	    bFail = 0;
       // OSRAM128x64x4StringDraw(" OK", 108, 24, 15);
    }
    else
    {
        bFail = 1;
       //  OSRAM128x64x4StringDraw("NOK", 108, 24, 15);
    }

    //
    // The MPU was disabled when the previous fault occurred, so it needs to be
    // re-enabled.
    //
    MPUEnable(MPU_CONFIG_HARDFLT_NMI);

    //
    // Attempt to read from the read-only area of RAM, the middle 8 KB of the
    // 64 KB region.
    //
    //OSRAM128x64x4StringDraw("Check RAM read", 0, 32, 8);
    g_ulMPUFaultCount = 0;
    HWREG(0x20008440);

    //
    // Verify that the RAM read did not cause a fault.
    //
    if(g_ulMPUFaultCount == 0)
    {
	 bFail = 0;
       // OSRAM128x64x4StringDraw(" OK", 108, 32, 15);
    }
    else
    {
        bFail = 1;
       // OSRAM128x64x4StringDraw("NOK", 108, 32, 15);
    }

    //
    // The MPU should not have been disabled since the last access was not
    // supposed to cause a fault.  But if it did cause a fault, then the MPU
    // will be disabled, so re-enable it here anyway, just in case.
    //
    MPUEnable(MPU_CONFIG_HARDFLT_NMI);

    //
    // Attempt to write to the read-only area of RAM, the middle 8 KB of the
    // 64 KB region.
    //
    //OSRAM128x64x4StringDraw("Check RAM write", 0, 40, 8);
    g_ulMPUFaultCount = 0;
    HWREG(0x20008460) = 0xabcdef00;

    //
    // Verify that the RAM write caused a fault.
    //
    if((g_ulMPUFaultCount == 1) &&
       (g_ulFaultStatus == 0x82) &&
       (g_ulMMAR == 0x20008460))
    {
	 bFail = 0;
       // OSRAM128x64x4StringDraw(" OK", 108, 40, 15);
    }
    else
    {
        bFail = 1;
       // OSRAM128x64x4StringDraw("NOK", 108, 40, 14);
    }

    //
    // Display the results of the example program.
    //
    if(bFail)
    {
	//    OSRAM128x64x4StringDraw("Failure!", 36, 56, 15);
    }
    else
    {
     //   OSRAM128x64x4StringDraw("Success!", 36, 56, 15);
    }

    //
    // Disable the MPU, so there are no lingering side effects if another
    // program is run.
    //
    MPUDisable();

}
/*****************************************************************************
* MPU使能设置子程序
* 入口ulMPUConfig 设置MPU 控制寄存器
*****************************************************************************/
void MPUEnable(u32 ulMPUConfig)
{
    ASSERT(!(ulMPUConfig & ~(MPU_CONFIG_PRIV_DEFAULT |
                             MPU_CONFIG_HARDFLT_NMI)));
    NVIC->MPU_CTRL = ulMPUConfig | NVIC_MPU_CTRL_ENABLE;
}

/*****************************************************************************
* MPU禁能设置子程序
*****************************************************************************/
void MPUDisable(void)
{
    NVIC->MPU_CTRL &= ~NVIC_MPU_CTRL_ENABLE;
}

/*****************************************************************************
* 获取MPU的DREGION域
*****************************************************************************/
u32	MPURegionCountGet(void)
{
    return((NVIC->MPU_TYPE & NVIC_MPU_TYPE_DREGION_M)
            >> NVIC_MPU_TYPE_DREGION_S);
}

/*****************************************************************************
* MPU保护使能
* 入口ulRegion 设置MPU 保护区域使能
*****************************************************************************/
void MPURegionEnable(u32 ulRegion)
{
    ASSERT(ulRegion < 8);

    NVIC->MPU_NUMBER = ulRegion;

    NVIC->MPU_ATTR |= NVIC_MPU_ATTR_ENABLE;
}

/*****************************************************************************
* MPU保护禁能
* 入口ulRegion 设置MPU 保护区域禁
*****************************************************************************/
void MPURegionDisable(u32 ulRegion)
{
    ASSERT(ulRegion < 8);

    NVIC->MPU_NUMBER = ulRegion;
    NVIC->MPU_ATTR &= ~NVIC_MPU_ATTR_ENABLE;
}

/*****************************************************************************
* MPU保护使能
* ulRegion 区域
* ulAddr Region Addr
* ulFlags Region Size
*****************************************************************************/
void MPURegionSet(u32 ulRegion, u32 ulAddr,u32 ulFlags)
{
    ASSERT(ulRegion < 8);  //判断区域是否有效
    ASSERT((ulAddr & (~0) << (((ulFlags & NVIC_MPU_ATTR_SIZE_M) >> 1) + 1))
            == ulAddr);	   //判断地址是否有效
    NVIC->MPU_BASE = ulAddr | ulRegion | NVIC_MPU_BASE_VALID;
    NVIC->MPU_ATTR = ((ulFlags & ~(NVIC_MPU_ATTR_TEX_M |
                                   NVIC_MPU_ATTR_CACHEABLE)) |
                       NVIC_MPU_ATTR_SHAREABLE |
                       NVIC_MPU_ATTR_BUFFRABLE);
}

/*****************************************************************************
* 获取MPU 基址和属性
*****************************************************************************/
void MPURegionGet(u32 ulRegion, u32 *pulAddr,u32 *pulFlags)
{
    ASSERT(ulRegion < 8);
    ASSERT(pulAddr);
    ASSERT(pulFlags);

    NVIC->MPU_NUMBER = ulRegion;

    *pulAddr = NVIC->MPU_BASE;
    *pulFlags = NVIC->MPU_ATTR;
}

/*****************************************************************************
* 设置MPU中断复位程序入口
* 入口 pfnHandler 中断复位程序入口
* 使能MPU中断
*****************************************************************************/
void MPUIntRegister(void (*pfnHandler)(void))
{
    ASSERT(pfnHandler);

    IntRegister(FAULT_MPU, pfnHandler);

    IntEnable(FAULT_MPU);
}

/*****************************************************************************
* 撤销MPU中断
* 中断服务程序入口设置为系统默任服务程序
*****************************************************************************/
void MPUIntUnregister(void)
{
    IntDisable(FAULT_MPU);
    IntUnregister(FAULT_MPU);
}
