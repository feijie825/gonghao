;******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : LM3S21xx_Vect.s
;* Author             : 张力阵
;* 中断向量表
;*******************************************************************************/

;******************************************************************************
;
; <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;
;******************************************************************************
Stack    EQU     0x00000200
RAM_SIZE EQU     0x4000
;******************************************************************************
;
; <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;
;******************************************************************************
Heap    EQU     0x00000000

;******************************************************************************
;
; Allocate space for the stack.
;
;******************************************************************************
        AREA    STACK, NOINIT, READWRITE, ALIGN=3
StackMem
        SPACE   Stack
__initial_sp

;******************************************************************************
;
; Allocate space for the heap.
;
;******************************************************************************
        AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
HeapMem
        SPACE   Heap
__heap_limit

;******************************************************************************
;
; Indicate that the code in this file preserves 8-byte alignment of the stack.
;
;******************************************************************************
        PRESERVE8
SYSCTL_LDOPCTL         equ  0x400FE034
SYSCTL_LDOPCTL_2_75V   equ  0x0000001B
SYSCTL_RCC             equ  0x400FE060
SYSCTL_RCC_OSCSRC_MAIN equ  0x00000000
SYSCTL_RCC_XTAL_8MHZ   EQU  0x00000380  ; Using a 8MHz crystal
SYSCTL_RCC_IOSCDIS     EQU  0x00000002  ; Internal oscillator disable

;******************************************************************************
;
; Place code into the reset code section.
;
;******************************************************************************
        AREA    RESET, CODE, READONLY
        THUMB
;******************************************************************************
;
; External declaration for the interrupt handler used by the application.
;
;******************************************************************************
		EXTERN  WatchDogHandler
		EXTERN  SysTickIntHandler
		EXTERN  ADC0Handler
		EXTERN  UART0IntHandler
		EXTERN  UART1IntHandler
		EXTERN  Timer0AIntHandler
		EXTERN  Timer0BIntHandler
		EXTERN  Timer1AIntHandler
		EXTERN  Timer1BIntHandler
		EXTERN  Timer2AIntHandler
		EXTERN  Timer2BIntHandler
		EXTERN  CANHandler
		EXTERN  GPIOBHandler
		EXTERN  GPIOCHandler
  EXTERN  GPIODHandler
		EXTERN  GPIOFHandler
;******************************************************************************
;		
; The vector table.
;
;******************************************************************************
        EXPORT  __Vectors
__Vectors
        DCD     StackMem + Stack            ; Top of Stack
        DCD     Reset_Handler               ; Reset Handler
        DCD     NmiSR                       ; NMI Handler
        DCD     FaultISR                    ; Hard Fault Handler
        DCD     IntDefaultHandler           ; MPU Fault Handler
        DCD     IntDefaultHandler           ; Bus Fault Handler
        DCD     IntDefaultHandler           ; Usage Fault Handler
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     IntDefaultHandler           ; SVCall Handler
        DCD     IntDefaultHandler           ; Debug Monitor Handler
        DCD     0                           ; Reserved
        DCD     IntDefaultHandler           ; PendSV Handler
        DCD     SysTickIntHandler           ; SysTick Handler
        DCD     IntDefaultHandler           ; GPIO Port A
        DCD     GPIOBHandler                ; IntDefaultHandler; GPIO Port B 光点头脉冲中断处理
        DCD     GPIOCHandler                ; IntDefaultHandler; GPIO Port C 按键中断
        DCD     GPIODHandler                ; GPIO Port D
        DCD     IntDefaultHandler           ; GPIO Port E
        DCD     UART0IntHandler             ; UART0
        DCD     UART1IntHandler             ; UART1
        DCD     IntDefaultHandler           ; SSI
        DCD     IntDefaultHandler           ; I2C
        DCD     IntDefaultHandler           ; PWM Fault
        DCD     IntDefaultHandler           ; PWM Generator 0
        DCD     IntDefaultHandler           ; PWM Generator 1
        DCD     IntDefaultHandler           ; PWM Generator 2
        DCD     IntDefaultHandler           ; Quadrature Encoder
        DCD     ADC0Handler                 ; ADC Sequence 0
        DCD     IntDefaultHandler           ; ADC Sequence 1
        DCD     IntDefaultHandler           ; ADC Sequence 2
        DCD     IntDefaultHandler           ; ADC Sequence 3
        DCD     WatchDogHandler             ; Watchdog
        DCD     Timer0AIntHandler           ; Timer 0A
        DCD     Timer0BIntHandler           ; Timer 0B
        DCD     Timer1AIntHandler           ; Timer 1A
        DCD     Timer1BIntHandler           ; Timer 1B
        DCD     Timer2AIntHandler           ; Timer 2A
        DCD     Timer2BIntHandler           ; Timer 2B
        DCD     IntDefaultHandler           ; Comp 0
        DCD     IntDefaultHandler           ; Comp 1
        DCD     IntDefaultHandler           ; Comp 2
        DCD     IntDefaultHandler           ; System Control
        DCD     IntDefaultHandler           ; Flash Control
        DCD     GPIOFHandler                ; GPIO Port F  需量周期脉冲中断 时段投切 合闸脉冲
        DCD     IntDefaultHandler           ; GPIO Port G
        DCD     IntDefaultHandler           ; GPIO Port H
        DCD     IntDefaultHandler           ; UART2 Rx and Tx
        DCD     IntDefaultHandler           ; SSI1 Rx and Tx
        DCD     IntDefaultHandler           ; Timer 3 subtimer A
        DCD     IntDefaultHandler           ; Timer 3 subtimer B
        DCD     IntDefaultHandler           ; I2C1 Master and Slave
        DCD     IntDefaultHandler           ; Quadrature Encoder 1
        DCD     CANHandler                  ; CAN0
        DCD     IntDefaultHandler           ; CAN1
        DCD     IntDefaultHandler           ; CAN2
        DCD     IntDefaultHandler           ; Ethernet
        DCD     IntDefaultHandler           ; Hibernate
        DCD     IntDefaultHandler           ; USB0
        DCD     IntDefaultHandler           ; PWM Generator 3
        DCD     IntDefaultHandler           ; uDMA Software Transfer
        DCD     IntDefaultHandler           ; uDMA Error

;******************************************************************************
;
; This is the code that gets called when the processor first starts execution
; following a reset event.
;
;******************************************************************************
        EXPORT  Reset_Handler
Reset_Handler
        ;
        ; Call the C library enty point that handles startup.  This will copy
        ; the .data section initializers from flash to SRAM and zero fill the
        ; .bss section.
        ;
        IMPORT  __main
;设置内核电压为2.75V        
        ldr   r0,=SYSCTL_LDOPCTL;
        ldr   r1,[r0]
        and   r1,#0xFFFFFFC0
       orr   r1,#SYSCTL_LDOPCTL_2_75V ;内核电压设置r2
        str   r1,[r0]
;设置工作时钟 工作于晶振频率       
        ldr   r0,=SYSCTL_RCC
        ldr   r1,[r0]
        movw  r2,#0x003f3
        orr   r2,#0x40000
        mvn   r2,r2
        and   r1,r2
        orr   r1,#SYSCTL_RCC_XTAL_8MHZ
        orr   r1,#SYSCTL_RCC_IOSCDIS
        str   r1,[r0]

        movs    r0, #0x00000000
        ldr     r1, =0x20000000
        add     r2,r1, #RAM_SIZE
zero_loop
        str     r0, [r1], #4
        cmp     r1, r2
        blt     zero_loop

        B       __main

;******************************************************************************
;
; This is the code that gets called when the processor receives a NMI.  This
; simply enters an infinite loop, preserving the system state for examination
; by a debugger.
;
;******************************************************************************
NmiSR
        B       NmiSR

;******************************************************************************
;
; This is the code that gets called when the processor receives a fault
; interrupt.  This simply enters an infinite loop, preserving the system state
; for examination by a debugger.
;
;******************************************************************************
FaultISR
        B       FaultISR

;******************************************************************************
;
; This is the code that gets called when the processor receives an unexpected
; interrupt.  This simply enters an infinite loop, preserving the system state
; for examination by a debugger.
;
;******************************************************************************
IntDefaultHandler
        B       IntDefaultHandler

;******************************************************************************
;
; Make sure the end of this section is aligned.
;
;******************************************************************************
        ALIGN

;******************************************************************************
;
; Some code in the normal code section for initializing the heap and stack.
;
;******************************************************************************
        AREA    |.text|, CODE, READONLY

;******************************************************************************
;
; The function expected of the C library startup code for defining the stack
; and heap memory locations.  For the C library version of the startup code,
; provide this function so that the C library initialization code can find out
; the location of the stack and heap.
;
;******************************************************************************
    IF :DEF: __MICROLIB
        EXPORT  __initial_sp
        EXPORT  __heap_base
        EXPORT __heap_limit
    ELSE
        IMPORT  __use_two_region_memory
        EXPORT  __user_initial_stackheap
__user_initial_stackheap
        LDR     R0, =HeapMem
        LDR     R1, =(StackMem + Stack)
        LDR     R2, =(HeapMem + Heap)
        LDR     R3, =StackMem
        BX      LR
    ENDIF

;******************************************************************************
;
; Make sure the end of this section is aligned.
;
;******************************************************************************
        ALIGN

;******************************************************************************
;
; Tell the assembler that we're done.
;
;******************************************************************************
        END
