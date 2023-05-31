/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifdef RP2040

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "st7789_driver.h"

#define PIN_DIN 0
#define PIN_CLK 1
#define PIN_CS 2
#define PIN_DC 3
#define PIN_RESET 4
#define PIN_BL 20

#define SERIAL_CLK_DIV 1.f

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
static const uint8_t st7789_init_seq[] = {
        1, 20, 0x01,                        // Software reset
        1, 10, 0x11,                        // Exit sleep mode
        2, 2, 0x3a, 0x55,                   // Set colour mode to 16 bit
        2, 0, 0x36, 0x00,                   // Set MADCTL: row then column, refresh is bottom to top ????
        5, 0, 0x2a, 0x00, 0x00, LCD_WIDTH >> 8, LCD_WIDTH & 0xff,   // CASET: column addresses
        5, 0, 0x2b, 0x00, 0x00, LCD_HEIGHT >> 8, LCD_HEIGHT & 0xff, // RASET: row addresses
        1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                         // Normal display on, then 10 ms delay
        1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
        0                                   // Terminate list
};

static inline void lcd_set_dc_cs(bool dc, bool cs) {
    sleep_us(1);
    gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count) {
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    st7789_lcd_put(pio, sm, *cmd++);
    if (count >= 2) {
        st7789_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            st7789_lcd_put(pio, sm, *cmd++);
    }
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void st7789_start_pixels(PIO pio, uint sm) {
    uint8_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
}

static inline void st7798_set_address_window(PIO pio, uint sm, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    // CASET 0x2A
    // RASET 0x2B
    // RAMWR 0x2C
    uint32_t xa = ((uint32_t)x << 16) | (x + w - 1);
    uint32_t ya = ((uint32_t)y << 16) | (y + h - 1);

    // write MSB or LSB first?

    uint8_t cmd = 0x2a; // CASET
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
    st7789_lcd_put(pio, sm, xa >> 24);
    st7789_lcd_put(pio, sm, xa >> 16);
    st7789_lcd_put(pio, sm, xa >> 8);
    st7789_lcd_put(pio, sm, xa);

    cmd = 0x2B; // RASET
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
    st7789_lcd_put(pio, sm, ya >> 24);
    st7789_lcd_put(pio, sm, ya >> 16);
    st7789_lcd_put(pio, sm, ya >> 8);
    st7789_lcd_put(pio, sm, ya);

    // write to ram
    cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
}

ST7789::ST7789()
{
    initialised = false;
    currentWidth = LCD_WIDTH;
    currentHeight = LCD_HEIGHT;
}

void ST7789::InitLCD()
{
    pio = pio0;
    sm = 0;
    uint offset = pio_add_program(pio, &st7789_lcd_program);
    // how do I change this to use 32 bits? Or should I just use someone else's library?
    st7789_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

    gpio_init(PIN_CS);
    gpio_init(PIN_DC);
    gpio_init(PIN_RESET);
    gpio_init(PIN_BL);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_set_dir(PIN_BL, GPIO_OUT);

    gpio_put(PIN_CS, 1);
    gpio_put(PIN_RESET, 1);
    lcd_init(pio, sm, st7789_init_seq);
    gpio_put(PIN_BL, 1);

    initialised = true;
}

void ST7789::SetDisplayArea(int x, int y)
{
    if (!initialised)
    {
        return;
    }

    currentWidth = x;
    currentHeight = y;

    int startX = (LCD_WIDTH - currentWidth)/2;
    int startY = (LCD_HEIGHT - currentHeight)/2;
    st7798_set_address_window(pio, sm, startX, startY, currentWidth, currentHeight);
}

void ST7789::ClearScreen()
{
    if (!initialised)
    {
        return;
    }
    
    st7798_set_address_window(pio, sm, 0, 0, LCD_WIDTH, LCD_HEIGHT);
    st7789_start_pixels(pio, sm);
    for (int i = 0; i < LCD_HEIGHT; ++i)
    {
        for (int j = 0; j < LCD_WIDTH; ++j)
        {
            st7789_lcd_put(pio, sm, 0);
            st7789_lcd_put(pio, sm, 0);
        }
    }

    int startX = (LCD_WIDTH - currentWidth)/2;
    int startY = (LCD_HEIGHT - currentHeight)/2;
    st7798_set_address_window(pio, sm, startX, startY, currentWidth, currentHeight);
}

void ST7789::WriteBuffer(uint16_t* buffer)
{
    if (!initialised)
    {
        return;
    }

    st7789_start_pixels(pio, sm);
    for (int i = 0; i < currentHeight; ++i)
    {
        for (int j = 0; j < currentWidth; ++j)
        {
            int pixel = i*currentWidth + j;
            uint16_t colour = buffer[pixel];
            st7789_lcd_put(pio, sm, colour >> 8);
            st7789_lcd_put(pio, sm, colour & 0xFF);
        }
    }
}

#endif