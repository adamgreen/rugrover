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
/* Constants used from C and Assembly Language code. Most relate to location of application code. */
#ifndef APP_H_
#define APP_H_


// Where the crash dump stores the CPU context in FLASH.
#define CRASH_DUMP_CONTEXT  0x2b000
// Where the crash dump stores the contents of SRAM at the time of the crash.
#define CRASH_DUMP_RAM      0x2c000 // (CRASH_DUMP_CONTEXT+0x1000)

// Where the application starts in FLASH.
#define APP_START           0x3C000 // (CRASH_DUMP_RAM+0x10000)

// Size of nRF52 FLASH pages in bytes.
#define FLASH_PAGE_SIZE     4096

// The start of RAM.
#define START_OF_RAM        0x20000000

// One byte past the end of RAM.
#define END_OF_RAM          0x20010000

// Top of the stack when bootloader starts and application will get whats left after bootloader uses what it needs.
#define TOP_OF_STACK        END_OF_RAM

#endif // APP_H_
