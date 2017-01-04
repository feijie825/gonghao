;******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : cpu_Keil.s
;* Author             : ’≈¡¶’Û
;* 
;*******************************************************************************/

;******************************************************************************
;
; The assembly code preamble required to put the assembler into the correct
; configuration.
;
;******************************************************************************
    thumb
    require8
    preserve8

;******************************************************************************
;
; The entire contents of this file go into the code section.
;
;******************************************************************************
    area    ||.text||, code, readonly, align=2

;******************************************************************************
;
; Wrapper function for the CPSID instruction.  Returns the state of PRIMASK
; on entry.
;
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
;
;******************************************************************************
    export CPUcpsie
CPUcpsie
    mrs     r0, PRIMASK
    cpsie   i
    bx      lr

;******************************************************************************
;
; Wrapper function for the WFI instruction.
;
;******************************************************************************
    export CPUwfi
CPUwfi
    wfi
    bx      lr

    end
