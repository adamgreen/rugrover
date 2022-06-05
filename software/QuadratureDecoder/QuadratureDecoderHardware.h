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
// Driver to interface with quadrature encoders using the QDEC peripheral on the nRF52 microcontroller.
#ifndef QUADRATURE_DECODER_HARDWARE_
#define QUADRATURE_DECODER_HARDWARE_

#include <stdint.h>
#include <nrf_drv_qdec.h>


class QuadratureDecoderHardware
{
    public:
        QuadratureDecoderHardware(uint8_t aPin, uint8_t bPin);

        bool init();

        int32_t getCount();

    protected:
        int32_t m_count;
        uint8_t m_aPin;
        uint8_t m_bPin;
};

#endif // QUADRATURE_DECODER_HARDWARE_
