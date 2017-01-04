/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_mpu.h
;* Author             : 张力阵
;* 存储器保护驱动 声明和预定义
*******************************************************************************/

#ifndef __MPU_H__
#define __MPU_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Flags that can be passed to MPUEnable..
//
//*****************************************************************************
#define MPU_CONFIG_PRIV_DEFAULT     4
#define MPU_CONFIG_HARDFLT_NMI      2
#define MPU_CONFIG_NONE             0

//*****************************************************************************
//
// Flags for the region size to be passed to MPURegionSet.
//
//*****************************************************************************
#define MPU_RGN_SIZE_32B            (4 << 1)
#define MPU_RGN_SIZE_64B            (5 << 1)
#define MPU_RGN_SIZE_128B           (6 << 1)
#define MPU_RGN_SIZE_256B           (7 << 1)
#define MPU_RGN_SIZE_512B           (8 << 1)

#define MPU_RGN_SIZE_1K             (9 << 1)
#define MPU_RGN_SIZE_2K             (10 << 1)
#define MPU_RGN_SIZE_4K             (11 << 1)
#define MPU_RGN_SIZE_8K             (12 << 1)
#define MPU_RGN_SIZE_16K            (13 << 1)
#define MPU_RGN_SIZE_32K            (14 << 1)
#define MPU_RGN_SIZE_64K            (15 << 1)
#define MPU_RGN_SIZE_128K           (16 << 1)
#define MPU_RGN_SIZE_256K           (17 << 1)
#define MPU_RGN_SIZE_512K           (18 << 1)

#define MPU_RGN_SIZE_1M             (19 << 1)
#define MPU_RGN_SIZE_2M             (20 << 1)
#define MPU_RGN_SIZE_4M             (21 << 1)
#define MPU_RGN_SIZE_8M             (22 << 1)
#define MPU_RGN_SIZE_16M            (23 << 1)
#define MPU_RGN_SIZE_32M            (24 << 1)
#define MPU_RGN_SIZE_64M            (25 << 1)
#define MPU_RGN_SIZE_128M           (26 << 1)
#define MPU_RGN_SIZE_256M           (27 << 1)
#define MPU_RGN_SIZE_512M           (28 << 1)

#define MPU_RGN_SIZE_1G             (29 << 1)
#define MPU_RGN_SIZE_2G             (30 << 1)
#define MPU_RGN_SIZE_4G             (31 << 1)

//*****************************************************************************
//
// Flags for the permissions to be passed to MPURegionSet.
//
//*****************************************************************************
#define MPU_RGN_PERM_EXEC           0x00000000	   //代码可执行
#define MPU_RGN_PERM_NOEXEC         0x10000000	   //代码不可执行
#define MPU_RGN_PERM_PRV_NO_USR_NO  0x00000000	   //特权:禁能 用户:禁能
#define MPU_RGN_PERM_PRV_RW_USR_NO  0x01000000	   //特权:读写 用户:禁能
#define MPU_RGN_PERM_PRV_RW_USR_RO  0x02000000	   //特权:读写 用户:只读
#define MPU_RGN_PERM_PRV_RW_USR_RW  0x03000000	   //特权:读写 用户:读写
#define MPU_RGN_PERM_PRV_RO_USR_NO  0x05000000	   //特权:只读 用户:禁能
#define MPU_RGN_PERM_PRV_RO_USR_RO  0x06000000	   //特权:只读 用户:只读

//*****************************************************************************
//
// Flags for the sub-region to be passed to MPURegionSet.
//
//*****************************************************************************
#define MPU_SUB_RGN_DISABLE_0       0x00000100	  //子区域0禁能
#define MPU_SUB_RGN_DISABLE_1       0x00000200	  //子区域1禁能
#define MPU_SUB_RGN_DISABLE_2       0x00000400	  //子区域2禁能
#define MPU_SUB_RGN_DISABLE_3       0x00000800	  //子区域3禁能
#define MPU_SUB_RGN_DISABLE_4       0x00001000	  //子区域4禁能
#define MPU_SUB_RGN_DISABLE_5       0x00002000	  //子区域5禁能
#define MPU_SUB_RGN_DISABLE_6       0x00004000	  //子区域6禁能
#define MPU_SUB_RGN_DISABLE_7       0x00008000	  //子区域7禁能

//*****************************************************************************
//
// Flags to enable or disable a region, to be passed to MPURegionSet.
//
//*****************************************************************************
#define MPU_RGN_ENABLE              1  //区域使能
#define MPU_RGN_DISABLE             0  //区域禁能

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern void MPUEnable(u32 ulMPUConfig);
extern void MPUDisable(void);
extern u32  MPURegionCountGet(void);
extern void MPURegionEnable(u32 ulRegion);
extern void MPURegionDisable(u32 ulRegion);
extern void MPURegionSet(u32 ulRegion, u32 ulAddr,u32 ulFlags);
extern void MPURegionGet(u32 ulRegion, u32 *pulAddr,u32 *pulFlags);
extern void MPUIntRegister(void (*pfnHandler)(void));
extern void MPUIntUnregister(void);
extern void Init_Mpu(void);   // 初始化MPU

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __MPU_H__
