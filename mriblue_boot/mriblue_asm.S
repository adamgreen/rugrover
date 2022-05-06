/*  Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Implementation of Cortex-M architecture assembly language routines to be used by mriblue debug monitor. */
    .text
    .code 16
    .syntax unified


    .global HardFault_Handler
    .type HardFault_Handler, function
    .thumb_func
    /* extern "C" void HardFault_Handler(void); */
HardFault_Handler:
    mrs     r0, psp
    mrs     r1, msp
    mov     r2, lr
    push    {r12, lr}
    bl      mriFaultHandler
    movs    r0, r0
    bmi     1$
    pop     {r12, pc}
1$: pop     {r12, lr}
    b       mriExceptionHandler
    .pool
    .size   HardFault_Handler, .-HardFault_Handler


    .global MemoryManagement_Handler
    .type MemoryManagement_Handler, function
    .thumb_func
    /* extern "C" void MemoryManagement_Handler(void); */
MemoryManagement_Handler:
    b       HardFault_Handler


    .global BusFault_Handler
    .type BusFault_Handler, function
    .thumb_func
    /* extern "C" void BusFault_Handler(void); */
BusFault_Handler:
    b       HardFault_Handler


    .global UsageFault_Handler
    .type UsageFault_Handler, function
    .thumb_func
    /* extern "C" void UsageFault_Handler(void); */
UsageFault_Handler:
    b       HardFault_Handler


    .end
