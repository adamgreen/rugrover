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
// 4-wire SPI Driver for the SH1106 OLED driver.
#ifndef OLED_SH1106_H_
#define OLED_SH1106_H_

#include <app_util_platform.h>
#include <nrf_drv_spi.h>
#include <Adafruit_GFX.h>

// Maximum screen dimensions supported by the SH1106 driver IC.
#define OLED_SH1106_MAX_WIDTH   132
#define OLED_SH1106_MAX_HEIGHT  64

// Size in bytes of the 1-bit monochrome frame buffer in the SH1106 driver IC.
#define OLED_SH1106_FRAME_SIZE  ((OLED_SH1106_MAX_WIDTH * OLED_SH1106_MAX_HEIGHT) / 8)

class OLED_SH1106: public Adafruit_GFX
{
public:
    // Constructor
    //  width, height - Defines the actual dimensions of the OLED connected to the driver.
    //  columnOffset, rowOffset - The OLED may not have column and row 0 connected to the driver's common and segment 0.
    //                            These offsets account for that.
    //  pSpiInstance - Points to the SPI peripheral instance to be used for communicating with the SH1106 driver.
    //  dcPin - The number of the pin connected to the A0 pin of the SH1106 driver.
    //  csPin - The number of the pin connected to the CS pin of the SH1106 driver. Can be set to
    //          NRF_DRV_SPI_PIN_NOT_USED if always pulled low.
    //  resetPin - The number of the pin connected to the RES pin of the SH1106 driver to reset it. It is optional.
    //  irqPriority - What priority level should the SPI interrupt handler which kicks off each page of the frame
    //                transfer be set at. It defaults to the lowest application priority level.
    OLED_SH1106(uint8_t width, uint8_t height, uint8_t columnOffset, uint8_t rowOffset,
                const nrf_drv_spi_t* pSpiInstance, uint8_t sckPin, uint8_t mosiPin, uint8_t dcPin,
                uint8_t csPin = NRF_DRV_SPI_PIN_NOT_USED, uint8_t resetPin = NRF_DRV_SPI_PIN_NOT_USED,
                app_irq_priority_t irqPriority = APP_IRQ_PRIORITY_LOWEST);

    // Actually initialize the required GPIO pins and SH1106 driver. Constructor just stores away pin numbers, etc.
    bool init();

    // Must be defined by Adafruit_GFX subclass.
    // Sets the desired pixel in the in-memory frame buffer, m_frame.
    // Must call refresh() or refreshAsync() to later rendered on actual OLED screen.
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color);

    // Optionally defined by Adafruit_GFX subclass.
    virtual void invertDisplay(bool invert);

    // Start of device specific methods.
    void setBrightness(uint8_t brightness);
    void enableDisplay(bool enable);
    void clearScreen();
    int printf(const char* pFormat, ...);

    // Called to copy in-memory frame buffer to actual OLED screen.
    //  Async version, which will return false if there is already a refresh in progress.
    bool refresh();
    //  Blocking version.
    void refreshAndBlock()
    {
        refresh();
        waitUntilDone();
    }

    // Returns true while async SPI operations are in flight.
    bool isBusy()
    {
        return m_state != DoneState;
    }

    void waitUntilDone()
    {
        while (isBusy())
        {
        }
    }


protected:
    void dataMode()
    {
        nrf_gpio_pin_set(m_dcPin);
    }
    void commandMode()
    {
        nrf_gpio_pin_clear(m_dcPin);
    }
    void selectChip()
    {
        if (m_csPin != NRF_DRV_SPI_PIN_NOT_USED)
        {
            nrf_gpio_pin_clear(m_csPin);
        }
    }
    void unselectChip()
    {
        if (m_csPin != NRF_DRV_SPI_PIN_NOT_USED)
        {
            nrf_gpio_pin_set(m_csPin);
        }
    }
    void sendCommands(const uint8_t* pCommands, size_t commandSize, bool block = true);

    void handleEvent(nrf_drv_spi_evt_t const * pEvent);
    static void handleSpiEvent(nrf_drv_spi_evt_t const * pEvent);


    // The priority level at which the SPI ISR runs.
    app_irq_priority_t m_irqPriority;

    // Adafruit_GFX drawing APIs will end up drawing to this in-memory frame buffer.
    uint8_t m_frame[OLED_SH1106_FRAME_SIZE];

    // The OLED may not have column and row 0 connected to common and segment 0. These offsets account for that.
    uint8_t m_colOffset;
    uint8_t m_rowOffset;

    // Pointer to SPI peripheral to be used for communicating with OLED driver.
    const nrf_drv_spi_t*    m_pSpi;

    // Pins to be used for communicating with the OLED driver.
    uint8_t m_sckPin;
    uint8_t m_mosiPin;
    uint8_t m_dcPin;
    uint8_t m_csPin;
    uint8_t m_resetPin;

    // The current page being transferred by the SPI ISR.
    volatile uint8_t m_pageNumber;
    // The page transfer is broken down into command and data portions.
    volatile enum { DoneState, CommandState, PageCommandState, PageDataState } m_state;

    // Array of bytes sent as page/column select command at the beginning of each page transfer.
    uint8_t m_pageSetupCommands[3];
};

#endif // OLED_SH1106_H_
