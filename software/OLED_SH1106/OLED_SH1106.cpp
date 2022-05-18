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
#include <memory.h>
#include <stdarg.h>
#include <nrf_assert.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include "OLED_SH1106.h"

// SH1106 SPI commands
#define SH1106_SET_COLUMN_ADDR_LOW  0x00
#define SH1106_SET_COLUMN_ADDR_HIGH 0x10
#define SH1106_SET_CONTRAST         0x81
#define SH1106_SET_SEGMENT_REMAP    0xA0
#define SH1106_SET_INVERT_DISPLAY   0xA6
#define SH1106_SET_DISPLAY_ON       0xAE
#define SH1106_SET_PAGE_ADDR        0xB0
#define SH1106_SET_COM_SCAN_DIR     0xC0

// Page layout of the SH1106 frame buffer.
#define PAGE_COUNT  8
#define PAGE_SIZE   (OLED_SH1106_FRAME_SIZE / PAGE_COUNT)


// Single object instance.
static OLED_SH1106* g_pOLED;



OLED_SH1106::OLED_SH1106(uint8_t width, uint8_t height, uint8_t columnOffset, uint8_t rowOffset,
                         const nrf_drv_spi_t* pSpiInstance, uint8_t sckPin, uint8_t mosiPin, uint8_t dcPin,
                         uint8_t csPin /* = NRF_DRV_SPI_PIN_NOT_USED */,
                         uint8_t resetPin /* = NRF_DRV_SPI_PIN_NOT_USED */,
                         app_irq_priority_t irqPriority /* = APP_IRQ_PRIORITY_LOWEST */)
: Adafruit_GFX(width, height)
{
    m_irqPriority = irqPriority;
    m_colOffset = columnOffset;
    m_rowOffset = rowOffset;
    m_pSpi = pSpiInstance;
    m_sckPin = sckPin;
    m_mosiPin = mosiPin;
    m_dcPin = dcPin;
    m_csPin = csPin;
    m_resetPin = resetPin;
    m_pageNumber = 0;
    m_state = DoneState;

    // The 0th element of this command array will be setup with the correct page number before each page is sent.
    m_pageSetupCommands[1] = (uint8_t)(SH1106_SET_COLUMN_ADDR_LOW | (m_colOffset & 0x0F));
    m_pageSetupCommands[2] = (uint8_t)(SH1106_SET_COLUMN_ADDR_HIGH | ((m_colOffset >> 4) & 0x0F));
}

bool OLED_SH1106::init()
{
    ASSERT ( g_pOLED == NULL );
    g_pOLED = this;

    nrf_gpio_pin_set(m_dcPin);
    nrf_gpio_cfg_output(m_dcPin);

    if (m_csPin != NRF_DRV_SPI_PIN_NOT_USED)
    {
        nrf_gpio_pin_set(m_csPin);
        nrf_gpio_cfg_output(m_csPin);
    }

    nrf_drv_spi_config_t spiConfig =
    {
        .sck_pin      = m_sckPin,
        .mosi_pin     = m_mosiPin,
        .miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
        .irq_priority = m_irqPriority,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_2M,
        .mode         = NRF_DRV_SPI_MODE_2,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
    int errorCode = nrf_drv_spi_init(m_pSpi, &spiConfig, handleSpiEvent);
    APP_ERROR_CHECK(errorCode);

    if(m_resetPin != NRF_DRV_SPI_PIN_NOT_USED) {
        nrf_gpio_pin_clear(m_resetPin);
        nrf_gpio_cfg_output(m_resetPin);
        nrf_delay_us(10);
        nrf_gpio_pin_set(m_resetPin);
    }

    // Clear out the frame buffer on the SH1106 OLED driver before turning the display on.
    clearScreen();
    refreshAndBlock();

    // Send commands needed to initialize the SH1106 OLED driver.
    uint8_t initCommands[] = {
        // Can flip horizontally using this command.
        SH1106_SET_SEGMENT_REMAP | 0,
        // Can flip vertically using this command.
        SH1106_SET_COM_SCAN_DIR | 0,
        // Set maximum brightness.
        SH1106_SET_CONTRAST,
        0xFF,
        // Turn the display on.
        SH1106_SET_DISPLAY_ON | 1
    };
    sendCommands(initCommands, sizeof(initCommands));

    // Datasheet says to wait 100ms after turning screen on before doing anything else.
    nrf_delay_ms(100);

    return true;
}

void OLED_SH1106::sendCommands(const uint8_t* pCommands, size_t commandSize, bool block /* = true */)
{
    if (block)
    {
        selectChip();
        m_state = CommandState;
    }
    else
    {
        m_state = PageCommandState;
    }

    commandMode();
    uint32_t errorCode = nrf_drv_spi_transfer(m_pSpi, pCommands, commandSize, NULL, 0);
    APP_ERROR_CHECK(errorCode);
    if (!block)
    {
        return;
    }

    while (m_state != DoneState)
    {
    }
    unselectChip();
 }

void OLED_SH1106::handleSpiEvent(nrf_drv_spi_evt_t const * pEvent)
{
    ASSERT ( g_pOLED != NULL );

    g_pOLED->handleEvent(pEvent);
}

void OLED_SH1106::handleEvent(nrf_drv_spi_evt_t const * pEvent)
{
    uint32_t errorCode;

    switch (m_state)
    {
        case CommandState:
            m_state = DoneState;
            break;
        case PageCommandState:
            m_state = PageDataState;
            dataMode();
            errorCode = nrf_drv_spi_transfer(m_pSpi, m_frame + m_pageNumber * PAGE_SIZE, WIDTH, NULL, 0);
            APP_ERROR_CHECK(errorCode);
            break;
        case PageDataState:
            m_pageNumber++;
            if (m_pageNumber < PAGE_COUNT)
            {
                m_state = PageCommandState;
                m_pageSetupCommands[0] = SH1106_SET_PAGE_ADDR | m_pageNumber;
                sendCommands(m_pageSetupCommands, sizeof(m_pageSetupCommands), false);
            }
            else
            {
                m_state = DoneState;
                unselectChip();
            }
            break;
        case DoneState:
            ASSERT ( m_state != DoneState );
            break;
    }
}

void OLED_SH1106::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
    {
        return;
    }

    int16_t t;
    switch (rotation) {
    case 1:
        t = x;
        x = WIDTH - 1 - y;
        y = t;
        break;
    case 2:
        x = WIDTH - 1 - x;
        y = HEIGHT - 1 - y;
        break;
    case 3:
        t = x;
        x = y;
        y = HEIGHT - 1 - t;
        break;
    }

    // The frame buffer uses the same page / column major ordering within a page configuration as the actual SH1106.
    int16_t page = y / 8;
    int16_t col = x;
    int16_t colOffset = (y % 8);
    uint8_t colMask = 1 << (uint8_t)colOffset;
    uint8_t* p = &m_frame[page*PAGE_SIZE + col];
    if (color)
    {
        *p |= colMask;
    }
    else
    {
        *p &= ~colMask;
    }
}

void OLED_SH1106::invertDisplay(bool invert)
{
    uint8_t setNormalReverseCommand = SH1106_SET_INVERT_DISPLAY | (invert ? 1 : 0);
    sendCommands(&setNormalReverseCommand, sizeof(setNormalReverseCommand));
}

void OLED_SH1106::setBrightness(uint8_t brightness)
{
    uint8_t setConstrastCommands[] =
    {
        SH1106_SET_CONTRAST,
        brightness
    };
    sendCommands(setConstrastCommands, sizeof(setConstrastCommands));
}

void OLED_SH1106::enableDisplay(bool enable)
{
    uint8_t enableDisplayCommand = SH1106_SET_DISPLAY_ON | (enable ? 1 : 0);
    sendCommands(&enableDisplayCommand, sizeof(enableDisplayCommand));
}

void OLED_SH1106::clearScreen()
{
    memset(m_frame, 0, sizeof(m_frame));
}

int OLED_SH1106::printf(const char* pFormat, ...)
{
    char buffer[32*8+1];
    va_list vaList;

    va_start(vaList, pFormat);
    int result = vsnprintf(buffer, sizeof(buffer), pFormat, vaList);
    va_end(vaList);
    print(buffer);
    return result;
}

bool OLED_SH1106::refresh()
{
    if (isBusy())
    {
        return false;
    }

    // Queue up the page select for page 0 to kick off the async frame transfer.
    m_state = PageCommandState;
    m_pageNumber = 0;
    m_pageSetupCommands[0] = SH1106_SET_PAGE_ADDR | m_pageNumber;
    selectChip();
    sendCommands(m_pageSetupCommands, sizeof(m_pageSetupCommands), false);
    return true;
}
