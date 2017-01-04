/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_types.h
;* Author             : 张力阵
;* 数据类型预定义
*******************************************************************************/

#ifndef __TYPES_H__
#define __TYPES_H__

//*****************************************************************************
//
// Define a boolean type, and values for true and false.
//
//*****************************************************************************
typedef unsigned char tBoolean;

typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;
typedef unsigned int   BOOL;

typedef unsigned long   u32;
typedef unsigned short  u16;
typedef unsigned char   u8;

typedef unsigned long  const uc32;  /* Read Only */
typedef unsigned short const uc16;  /* Read Only */
typedef unsigned char  const uc8;   /* Read Only */

typedef signed long   s32;
typedef signed short  s16;
typedef signed char   s8;

typedef signed long  const sc32;  /* Read Only */
typedef signed short const sc16;  /* Read Only */
typedef signed char  const sc8;   /* Read Only */


typedef volatile unsigned long   vu32;
typedef volatile unsigned short  vu16;
typedef volatile unsigned char   vu8;

typedef volatile unsigned long  const vuc32;  /* Read Only */
typedef volatile unsigned short const vuc16;  /* Read Only */
typedef volatile unsigned char  const vuc8;   /* Read Only */

typedef volatile signed long   vs32;
typedef volatile signed short  vs16;
typedef volatile signed char   vs8;

typedef volatile signed long  const vsc32;  /* Read Only */
typedef volatile signed short const vsc16;  /* Read Only */
typedef volatile signed char  const vsc8;   /* Read Only */

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

//*****************************************************************************
//
// Macros for hardware access, both direct and via the bit-band region.
//
//*****************************************************************************
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x)))
#define HWREGH(x)                                                             \
        (*((volatile unsigned short *)(x)))
#define HWREGB(x)                                                             \
        (*((volatile unsigned char *)(x)))
#define HWREGBITW(x, b)                                                       \
        HWREG(((unsigned long)(x) & 0xF0000000) | 0x02000000 |                \
              (((unsigned long)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITH(x, b)                                                       \
        HWREGH(((unsigned long)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned long)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITB(x, b)                                                       \
        HWREGB(((unsigned long)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned long)(x) & 0x000FFFFF) << 5) | ((b) << 2))

//*****************************************************************************
//
// Helper Macros for determining silicon revisions, etc.
//
// These macros will be used by Driverlib at "run-time" to create necessary
// conditional code blocks that will allow a single version of the Driverlib
// "binary" code to support multiple(all) Stellaris silicon revisions.
//
// It is expected that these macros will be used inside of a standard 'C'
// conditional block of code, e.g.
//
//     if(CLASS_IS_SANDSTORM)
//     {
//         do some Sandstorm-class specific code here.
//     }
//
// By default, these macros will be defined as run-time checks of the
// appropriate register(s) to allow creation of run-time conditional code
// blocks for a common DriverLib across the entire Stellaris family.
//
// However, if code-space optimization is required, these macros can be "hard-
// coded" for a specific version of Stellaris silicon.  Many compilers will
// then detect the "hard-coded" conditionals, and appropriately optimize the
// code blocks, eliminating any "unreachable" code.  This would result in
// a smaller Driverlib, thus producing a smaller final application size, but
// at the cost of limiting the Driverlib binary to a specific Stellaris
// silicon revision.
//
//*****************************************************************************
#ifndef CLASS_IS_SANDSTORM
#define CLASS_IS_SANDSTORM                                                    \
        (((SYSCTL->DID[0] & SYSCTL_DID0_VER_M) == SYSCTL_DID0_VER_0) ||   \
         ((SYSCTL->DID[0] & (SYSCTL_DID0_VER_M | SYSCTL_DID0_CLASS_M)) == \
          (SYSCTL_DID0_VER_1 | SYSCTL_DID0_CLASS_SANDSTORM)))
#endif

#ifndef CLASS_IS_FURY
#define CLASS_IS_FURY                                                        \
        ((SYSCTL->DID[0] & (SYSCTL_DID0_VER_M | SYSCTL_DID0_CLASS_M)) == \
         (SYSCTL_DID0_VER_1 | SYSCTL_DID0_CLASS_FURY))
#endif

#ifndef CLASS_IS_DUSTDEVIL
#define CLASS_IS_DUSTDEVIL                                                   \
        ((SYSCTL->DID[0] & (SYSCTL_DID0_VER_M | SYSCTL_DID0_CLASS_M)) == \
         (SYSCTL_DID0_VER_1 | SYSCTL_DID0_CLASS_DUSTDEVIL))
#endif

#ifndef REVISION_IS_A0
#define REVISION_IS_A0                                                     \
        ((SYSCTL->DID[0] & (SYSCTL_DID0_MAJ_M | SYSCTL_DID0_MIN_M)) == \
         (SYSCTL_DID0_MAJ_REVA | SYSCTL_DID0_MIN_0))
#endif

#ifndef REVISION_IS_A2
#define REVISION_IS_A2                                                     \
        ((SYSCTL->DID[0] & (SYSCTL_DID0_MAJ_M | SYSCTL_DID0_MIN_M)) == \
         (SYSCTL_DID0_MAJ_REVA | SYSCTL_DID0_MIN_2))
#endif

#ifndef REVISION_IS_C1
#define REVISION_IS_C1                                                     \
        ((SYSCTL->DID[0] & (SYSCTL_DID0_MAJ_M | SYSCTL_DID0_MIN_M)) == \
         (SYSCTL_DID0_MAJ_REVC | SYSCTL_DID0_MIN_1))
#endif

#ifndef REVISION_IS_C2
#define REVISION_IS_C2                                                     \
        ((SYSCTL->DID[0] & (SYSCTL_DID0_MAJ_M | SYSCTL_DID0_MIN_M)) == \
         (SYSCTL_DID0_MAJ_REVC | SYSCTL_DID0_MIN_2))
#endif

//*****************************************************************************
//
// Deprecated silicon class and revision detection macros.
//
//*****************************************************************************
#ifndef DEPRECATED
#define DEVICE_IS_SANDSTORM     CLASS_IS_SANDSTORM
#define DEVICE_IS_FURY          CLASS_IS_FURY
#define DEVICE_IS_REVA2         REVISION_IS_A2
#define DEVICE_IS_REVC1         REVISION_IS_C1
#define DEVICE_IS_REVC2         REVISION_IS_C2
#endif

#endif // __HW_TYPES_H__
