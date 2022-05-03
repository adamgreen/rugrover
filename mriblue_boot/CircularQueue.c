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
#include <nrf_assert.h>
#include <nrf_atomic.h>
#include "CircularQueue.h"

static uint32_t incrementIndex(uint32_t index);

void CircularQueue_Init(CircularQueue* pThis)
{
    ASSERT ( (sizeof(pThis->queue) & (sizeof(pThis->queue)-1)) == 0 );

    pThis->write = 0;
    pThis->read = 0;
    pThis->peek = 0;
    pThis->peekSize = 0;
    pThis->count = 0;
}

uint32_t CircularQueue_Write(CircularQueue* pThis, const uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );

    uint32_t bytesLeft = sizeof(pThis->queue) - pThis->count;
    uint32_t bytesToWrite = (bytesLeft < dataSize) ? bytesLeft : dataSize;
    for (uint32_t i = 0 ; i < bytesToWrite ; i++)
    {
        pThis->queue[pThis->write] = *pData++;
        pThis->write = incrementIndex(pThis->write);
    }
    nrf_atomic_u32_add(&pThis->count, bytesToWrite);
    return bytesToWrite;
}

static uint32_t incrementIndex(uint32_t index)
{
    return (index + 1) & (CIRCULAR_QUEUE_SIZE-1);
}

uint32_t CircularQueue_BytesToRead(CircularQueue* pThis)
{
    return pThis->count;
}

uint32_t CircularQueue_IsFull(CircularQueue* pThis)
{
    return CircularQueue_BytesToRead(pThis) == CIRCULAR_QUEUE_SIZE;
}

uint32_t CircularQueue_IsEmpty(CircularQueue* pThis)
{
    return CircularQueue_BytesToRead(pThis) == 0;
}

uint32_t CircularQueue_Read(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );

    uint32_t bytesToRead = (dataSize > pThis->count) ? pThis->count : dataSize;
    for (uint32_t i = 0 ; i < bytesToRead ; i++)
    {
        *pData++ = pThis->queue[pThis->read];
        pThis->read = incrementIndex(pThis->read);
    }
    nrf_atomic_u32_sub(&pThis->count, bytesToRead);
    return bytesToRead;
}

uint32_t CircularQueue_Peek(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );
    ASSERT ( pThis->peekSize == 0 );

    pThis->peek = pThis->read;
    uint32_t bytesToRead = (dataSize > pThis->count) ? pThis->count : dataSize;
    for (uint32_t i = 0 ; i < bytesToRead ; i++)
    {
        *pData++ = pThis->queue[pThis->peek];
        pThis->peek = incrementIndex(pThis->peek);
    }
    pThis->peekSize = bytesToRead;
    return bytesToRead;
}

void CircularQueue_CommitPeek(CircularQueue* pThis)
{
    ASSERT ( pThis->peekSize > 0 );

    pThis->read = pThis->peek;
    nrf_atomic_u32_sub(&pThis->count, pThis->peekSize);
    pThis->peekSize = 0;
}

void CircularQueue_RollbackPeek(CircularQueue* pThis)
{
    ASSERT ( pThis->peekSize > 0 );

    pThis->peekSize = 0;
}
