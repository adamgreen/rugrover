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
/* Circular queue used to communicate between BLE stack and MRI. */
#ifndef CICRCULAR_QUEUE_H_
#define CICRCULAR_QUEUE_H_

#include <stdint.h>


#define CIRCULAR_QUEUE_SIZE 256

typedef struct CircularQueue {
    uint32_t write;
    uint32_t read;
    uint32_t peek;
    uint32_t peekSize;
    volatile uint32_t count;
    volatile uint8_t  queue[CIRCULAR_QUEUE_SIZE];
} CircularQueue;

void CircularQueue_Init(CircularQueue* pThis);
uint32_t CircularQueue_Write(CircularQueue* pThis, const uint8_t* pData, uint32_t dataSize);
uint32_t CircularQueue_BytesToRead(CircularQueue* pThis);
uint32_t CircularQueue_IsFull(CircularQueue* pThis);
uint32_t CircularQueue_IsEmpty(CircularQueue* pThis);
uint32_t CircularQueue_Read(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize);
uint32_t CircularQueue_Peek(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize);
void CircularQueue_CommitPeek(CircularQueue* pThis);
void CircularQueue_RollbackPeek(CircularQueue* pThis);

#endif // CICRCULAR_QUEUE_H_
