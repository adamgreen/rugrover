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

// Where the application starts in FLASH.
#define APP_START 0x2b000

// Top of the stack when bootloader starts and application will get whats left after bootloader uses what it needs.
#define TOP_OF_STACK 0x20010000

#endif // APP_H_
