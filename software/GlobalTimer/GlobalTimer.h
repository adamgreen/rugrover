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
// Driver to create a global 1MHz timer that can be queried to determine number of microseconds since reset.
// Mimics Arduino's micros() functionality.
#ifndef GLOBAL_TIMER_H_
#define GLOBAL_TIMER_H_

#include <stdint.h>

uint32_t micros();

#endif // GLOBAL_TIMER_H_
