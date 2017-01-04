;******************************************************************************
;
; bl_startup_rvmdk.S - Startup code for RV-MDK.
;
; Copyright (c) 2007-2008 Luminary Micro, Inc.  All rights reserved.
; 
; Software License Agreement
; 
; Luminary Micro, Inc. (LMI) is supplying this software for use solely and
; exclusively on LMI's microcontroller products.
; 
; The software is owned by LMI and/or its suppliers, and is protected under
; applicable copyright laws.  All rights are reserved.  You may not combine
; this software with "viral" open-source software in order to form a larger
; program.  Any use in violation of the foregoing restrictions may subject
; the user to criminal sanctions under applicable laws, as well as to civil
; liability for the breach of the terms and conditions of this license.
; 
; THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; 
; This is part of revision 3416 of the Stellaris Peripheral Driver Library.
;
;******************************************************************************

    include bl_config.inc

;******************************************************************************
;
; Put the assembler into the correct configuration.
;
;******************************************************************************
    thumb
    require8
    preserve8

SYSCTL_LDOPCTL         equ  0x400FE034
SYSCTL_LDOPCTL_2_75V   equ  0x0000001B
SYSCTL_RCC             equ  0x400FE060
SYSCTL_RCC_OSCSRC_MAIN equ  0x00000000
SYSCTL_RCC_XTAL_8MHZ   EQU  0x00000380  ; Using a 8MHz crystal
SYSCTL_RCC_IOSCDIS     EQU  0x00000002  ; Internal oscillator disable

;******************************************************************************
;
; The stack gets placed into the zero-init section.
;
;******************************************************************************
    area    ||.bss||, noinit, align=2

;******************************************************************************
;
; Allocate storage for the stack.
;
;******************************************************************************
g_pulStack
    space   _STACK_SIZE * 4
;******************************************************************************
;
; This portion of the file goes into the text section.
;
;******************************************************************************
    area    RESET, code, readonly, align=3

;******************************************************************************
;
; The minimal vector table for a Cortex-M3 processor.
;
;******************************************************************************
    export  __Vectors
__Vectors
    dcd     g_pulStack + (_STACK_SIZE * 4)  ; Offset 00: Initial stack pointer
    dcd     Reset_Handler                   ; Offset 04: Reset handler
    dcd     NmiSR + 0x20000000              ; Offset 08: NMI handler
    dcd     FaultISR + 0x20000000           ; Offset 0C: Hard fault handler
    dcd     IntDefaultHandler + 0x20000000  ; Offset 10: MPU fault handler
    dcd     IntDefaultHandler + 0x20000000  ; Offset 14: Bus fault handler
    dcd     IntDefaultHandler + 0x20000000  ; Offset 18: Usage fault handler
    dcd     0                               ; Offset 1C: Reserved
    dcd     0                               ; Offset 20: Reserved
    dcd     0                               ; Offset 24: Reserved
    dcd     0                               ; Offset 28: Reserved
    dcd     UpdateHandler                   ; Offset 2C: SVCall handler
    dcd     IntDefaultHandler + 0x20000000  ; Offset 30: Debug monitor handler
    dcd     0                               ; Offset 34: Reserved
    dcd     IntDefaultHandler + 0x20000000  ; Offset 38: PendSV handler
    import  SysTickIntHandler
    dcd     SysTickIntHandler               ; Offset 3C: SysTick handler
    if      :def:_BOOT_UART0 :land: :def:_UART_ENABLE_UPDATE :land: :def:_UART_AUTOBAUD  	 
    import  GPIOIntHandler
    dcd     GPIOIntHandler                  ; GPIO port A handler
    else
    dcd     IntDefaultHandler + 0x20000000  ; PORTA
    endif
    dcd     IntDefaultHandler + 0x20000000  ; PORTB
    dcd     IntDefaultHandler + 0x20000000  ; PORTC
    if      :def:_BOOT_UART1 :land: :def:_UART_ENABLE_UPDATE :land: :def:_UART_AUTOBAUD    
    import  GPIOIntHandler
    dcd     GPIOIntHandler                  ; GPIO port D handler
    else
    dcd     IntDefaultHandler + 0x20000000  ; PORTD
    endif

;******************************************************************************
;
; The reset handler, which gets called when the processor starts.
;
;******************************************************************************
    export  Reset_Handler
Reset_Handler
     mov     r1,#8
Del0 mov     r0,#40000          ;开机延时20ms
Del1 subs    r0, #1
     bne     Del1
	 subs    r1,#1
	 bne     Del0
     bl      Reset_Target       ;复位
     bl      ProcessorInit		   ;初始化
; See if a an update should be performed.
; 检测强制更新管脚  
    import  CheckForceUpdate
    bl      CheckForceUpdate
    cbz     r0, CallApplication
; Configure the microcontroller.
; 是否允许通过以太网升级程序
    if      :def:_ENET_ENABLE_UPDATE
    import  ConfigureEnet
    bl      ConfigureEnet
    else
; 芯片配置程序    
    import  ConfigureDevice
    bl      ConfigureDevice
    endif
; Branch to the update handler.
;
    if      :def:_ENET_ENABLE_UPDATE
    import  UpdateBOOTP
    b       UpdateBOOTP
    else
    import  Updater
    b       Updater
    endif
; Call the application via the reset handler in its vector table.  Load the
; address of the application vector table.
; 调用用户应用程序
CallApplication
    ldr     r0, =_APP_START_ADDRESS
; Set the vector table address to the beginning of the application.
; 设置中断向量偏移地址-> 指向用户程序起始地址
    ldr     r1, =0xe000ed08
    str     r0, [r1]
; Load the stack pointer from the application's vector table.
; 从用户应用程序中取出堆栈表设置堆栈指针
    ldr     r1, [r0]
    mov     sp, r1
; Load the initial PC from the application's vector table and branch to
; the application's entry point.
; 跳转到用户程序
    ldr     r0, [r0, #4]
    bx      r0

;内核复位 zlz2008.11.12添加
Reset_Target
    if :def: _BOOST_LDO_VOLTAGE
    ldr   r0,=SYSCTL_LDOPCTL;
    ldr   r1,[r0]
    orr   r1,#SYSCTL_LDOPCTL_2_75V ;内核电压设置r2
    str   r1,[r0]
    endif
    if :def: _CRYSTAL_FREQ
    ldr   r0,=SYSCTL_RCC
    ldr   r1,[r0]
    movw  r2,#0x003f3
    orr   r2,#0x40000
    mvn   r2,r2
    and   r1,r2
    orr   r1,#SYSCTL_RCC_XTAL_8MHZ
    orr   r1,#SYSCTL_RCC_IOSCDIS
    str   r1,[r0]
    endif
    bx    lr
;******************************************************************************
;
; Initialize the processor by copying the boot loader from flash to SRAM, zero
; filling the .bss section, and moving the vector table to the beginning of
; SRAM.  The return address is modified to point to the SRAM copy of the boot
; loader instead of the flash copy, resulting in a branch to the copy now in
; SRAM.
;
;******************************************************************************
ProcessorInit
; 将程序代码从00~程序结束拷贝到RAM区 起始地址0x20000000
; 清除引导使能标志
    import  BOOT_EN
    mrs     r0, PRIMASK
    cpsid   i
    movs    r2, #0
    movs    r3, #0
    ldr     r0, =0x20000000
    ldr     r1, [r0], #4
    cmp     r1,#'E'
    bne     BOOT1
    ldr     r1, [r0], #4
    cmp     r1,#'E'
    bne     BOOT1
    movs    r2, #0x45
    ldr     r3, [r0]
BOOT1
    ldr     r0, =BOOT_EN
    str     r2,[r0],#4           ;BOOT_EN[0]
    str     r2,[r0],#4	         ;BOOT_EN[1]				    
    str     r3,[r0]  	         ;BOOT_EN[2]				    
    movs    r0, #0x00000000
    ldr     r1, =0x20000000
    import  ||Image$$ZI$$Base||
    ldr     r2, =||Image$$ZI$$Base||
copy_loop
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    cmp     r1, r2
    blt     copy_loop
; Zero fill the .bss section.
; 清除剩余RAM区
    movs    r0, #0x00000000
    import  ||Image$$ZI$$Limit||
    ldr     r2, =||Image$$ZI$$Limit||
zero_loop
    str     r0, [r1], #4
    cmp     r1, r2
    blt     zero_loop
; Set the vector table pointer to the beginning of SRAM.
; 设置中断向量起始地址为0x20000000即RAM区
    ldr     r0, =0xe000ed08		;nvic->vtable
    ldr     r1, =0x20000000
    str     r1, [r0]
; Set the return address to the code just copied into SRAM.
; 程序进入RAM运行
    orr     lr, lr, #0x20000000
    mrs     r0, PRIMASK
    cpsie   i
    bx      lr
    

;******************************************************************************
;
; Wrapper function for the CPSID instruction.  Returns the state of PRIMASK
; on entry.
; 关闭中断
;******************************************************************************
    export CPUcpsid
CPUcpsid
    mrs     r0, PRIMASK
    cpsid   i
    bx      lr

;******************************************************************************
;
; Wrapper function for the CPSIE instruction.  Returns the state of PRIMASK
; on entry.
; 使能中断
;******************************************************************************
    export CPUcpsie
CPUcpsie
    mrs     r0, PRIMASK
    cpsie   i
    bx      lr


;******************************************************************************
;
; The update handler, which gets called when the application would like to
; start an update.
;

;******************************************************************************
UpdateHandler
; Initialize the processor.
;
    bl      ProcessorInit
; Load the stack pointer from the vector table.
;
    movs    r0, #0x00000000
    ldr     r0, [r0]
    mov     sp, r0
; Branch to the update handler.
;
    if      :def:_ENET_ENABLE_UPDATE
    b       UpdateBOOTP
    else
    b       Updater
    endif

;******************************************************************************
;
; The NMI handler.
;
;******************************************************************************
NmiSR
; Loop forever since there is nothing that we can do about a NMI.
;
    b       .

;******************************************************************************
;
; The hard fault handler.
;
;******************************************************************************
FaultISR
; Loop forever since there is nothing that we can do about a hard fault.
;
    b       .

;******************************************************************************
;
; The default interrupt handler.
;
;******************************************************************************
IntDefaultHandler
; Loop forever since there is nothing that we can do about an unexpected
; interrupt.
;
    b       .

;******************************************************************************
;
; Provides a small delay.  The loop below takes 3 cycles/loop.
;
;******************************************************************************
    export  Delay
Delay
    subs    r0, #1
    bne     Delay
    bx      lr

;******************************************************************************
;
; This is the end of the file.
;
;******************************************************************************
    align   4
    end
