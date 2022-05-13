/* Copyright 2022 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
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
