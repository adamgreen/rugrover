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
// Generic Lock-Free Circular buffer that supports multiple producers and consumers.
#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

template <class T, size_t COUNT>
class CircularBuffer
{
    public:
        CircularBuffer()
        {
            // COUNT must be a multiple of 2.
            ASSERT ( (COUNT & (COUNT-1)) == 0 );
            m_writeIndex = 0;
            m_readIndex = 0;
            m_allocIndex = 0;
            m_freeIndex = 0;
        }

        bool write(const T& entryToAdd)
        {
            // Supports multiple writers running from different interrupt priority levels so use exclusive load/stores.
            size_t alloc;
            if (!allocate(&alloc))
            {
                return false;
            }

            m_entries[alloc].m_data = entryToAdd;
            __DMB();
            m_entries[alloc].m_isValid = true;
            commitAllocsToWrites();
            return true;
        }
        bool read(T& entryToFill)
        {
            // Supports multiple readers running from different interrupt priority levels so use exclusive load/stores.
            size_t read;
            if (!free(&read))
            {
                return false;
            }

            entryToFill = m_entries[read].m_data;
            __DMB();
            m_entries[read].m_isValid = false;
            commitFreesToReads();
            return true;
        }

        bool isEmpty()
        {
            return m_writeIndex == m_freeIndex;
        }
        bool isFull()
        {
            return nextIndex(m_allocIndex) == m_readIndex;
        }

    protected:
        enum { MASK = COUNT - 1 };

        class Entry
        {
            public:
                Entry()
                {
                    m_isValid = false;
                }

            T             m_data;
            volatile bool m_isValid;
        };

        size_t nextIndex(size_t index)
        {
            return (index + 1) & MASK;
        }

        bool allocate(size_t* pAlloc)
        {
            size_t alloc = m_allocIndex;
            bool result;
            do
            {
                size_t next = nextIndex(alloc);
                if (next == m_readIndex)
                {
                    return false;
                }
                result = __atomic_compare_exchange_n(&m_allocIndex, &alloc, next, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
                // Exit the loop if m_allocIndex wasn't modified behind our backs by a higher priority ISR.
                // Otherwise loop around and try again.
            } while (!result);

            *pAlloc = alloc;
            return true;
        }
        void commitAllocsToWrites()
        {
            // This is kind of like a last one to leave should turn off the lights. The last write to finish will
            // increment m_writeIndex to match m_allocIndex.
            size_t write = m_writeIndex;
            bool   result;
            do
            {
                if (write == m_allocIndex)
                {
                    return;
                }
                if (!m_entries[write].m_isValid)
                {
                    return;
                }
                size_t next = nextIndex(write);
                result = __atomic_compare_exchange_n(&m_writeIndex, &write, next, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
                write = next;
                // Exit if a higher priority ISR is also in the process of the commit since it is already ahead of
                // us. Otherwise loop around and see if we can increment m_writeIndex some more.
            } while(result);
        }

        bool free(size_t* pFree)
        {
            size_t free = m_freeIndex;
            bool result;
            do
            {
                if (free == m_writeIndex)
                {
                    return false;
                }
                size_t next = nextIndex(free);
                result = __atomic_compare_exchange_n(&m_freeIndex, &free, next, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
                // Exit the loop if m_freeIndex wasn't modified behind our backs by a higher priority ISR.
                // Otherwise loop around and try again.
            } while (!result);

            *pFree = free;
            return true;
        }
        void commitFreesToReads()
        {
            // The last read to finish will increment m_readIndex to match m_freeIndex.
            size_t read = m_readIndex;
            bool   result;
            do
            {
                if (read == m_freeIndex)
                {
                    return;
                }
                if (m_entries[read].m_isValid)
                {
                    return;
                }
                size_t next = nextIndex(read);
                result = __atomic_compare_exchange_n(&m_readIndex, &read, next, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
                read = next;
                // Exit if a higher priority ISR is also in the process of the commit since it is already ahead of
                // us. Otherwise loop around and see if we can increment m_readIndex some more.
            } while(result);
        }

        Entry               m_entries[COUNT];
        volatile size_t     m_writeIndex;
        volatile size_t     m_readIndex;
        volatile size_t     m_allocIndex;
        volatile size_t     m_freeIndex;
};

#endif // CIRCULAR_BUFFER_H_
