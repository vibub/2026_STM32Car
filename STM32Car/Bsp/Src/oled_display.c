#include "oled_display.h"
#include "main.h"

#define OLED_I2C_ADDRESS       (0x3CU << 1U)
#define OLED_I2C_TIMEOUT_MS    20U
#define OLED_WIDTH             128U
#define OLED_PAGE_COUNT        8U
#define OLED_CHAR_WIDTH        6U

static uint8_t s_oled_ready;

static uint8_t Oled_WriteCommand(uint8_t command)
{
    uint8_t buffer[2];

    buffer[0] = 0x00U;
    buffer[1] = command;
    return (HAL_I2C_Master_Transmit(&hi2c4, OLED_I2C_ADDRESS, buffer,
                                    sizeof(buffer), OLED_I2C_TIMEOUT_MS) == HAL_OK);
}

static uint8_t Oled_WriteData(const uint8_t *data, uint16_t size)
{
    uint8_t buffer[17];
    uint16_t offset;
    uint16_t chunk;
    uint16_t index;

    offset = 0U;
    while (offset < size) {
        chunk = (uint16_t) (size - offset);
        if (chunk > (uint16_t) (sizeof(buffer) - 1U)) {
            chunk = (uint16_t) (sizeof(buffer) - 1U);
        }

        buffer[0] = 0x40U;
        for (index = 0U; index < chunk; index++) {
            buffer[index + 1U] = data[offset + index];
        }

        if (HAL_I2C_Master_Transmit(&hi2c4, OLED_I2C_ADDRESS, buffer,
                                    (uint16_t) (chunk + 1U),
                                    OLED_I2C_TIMEOUT_MS) != HAL_OK) {
            return 0U;
        }
        offset = (uint16_t) (offset + chunk);
    }

    return 1U;
}

static uint8_t Oled_SetCursor(uint8_t page, uint8_t column)
{
    if ((Oled_WriteCommand((uint8_t) (0xB0U | (page & 0x07U))) == 0U) ||
        (Oled_WriteCommand((uint8_t) (0x00U | (column & 0x0FU))) == 0U) ||
        (Oled_WriteCommand((uint8_t) (0x10U | ((column >> 4U) & 0x0FU))) == 0U)) {
        return 0U;
    }

    return 1U;
}

static void Oled_GetGlyph(char ch, uint8_t glyph[5])
{
    uint8_t index;
    static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
    static const uint8_t digit0[5] = {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU};
    static const uint8_t digit1[5] = {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U};
    static const uint8_t digit2[5] = {0x42U, 0x61U, 0x51U, 0x49U, 0x46U};
    static const uint8_t digit3[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U};
    static const uint8_t digit4[5] = {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U};
    static const uint8_t digit5[5] = {0x27U, 0x45U, 0x45U, 0x45U, 0x39U};
    static const uint8_t digit6[5] = {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U};
    static const uint8_t digit7[5] = {0x01U, 0x71U, 0x09U, 0x05U, 0x03U};
    static const uint8_t digit8[5] = {0x36U, 0x49U, 0x49U, 0x49U, 0x36U};
    static const uint8_t digit9[5] = {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU};
    static const uint8_t h[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
    static const uint8_t e[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t l[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
    static const uint8_t o[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
    const uint8_t *source;

    source = blank;
    if (ch == ':') {
        source = colon;
    } else if (ch == '0') {
        source = digit0;
    } else if (ch == '1') {
        source = digit1;
    } else if (ch == '2') {
        source = digit2;
    } else if (ch == '3') {
        source = digit3;
    } else if (ch == '4') {
        source = digit4;
    } else if (ch == '5') {
        source = digit5;
    } else if (ch == '6') {
        source = digit6;
    } else if (ch == '7') {
        source = digit7;
    } else if (ch == '8') {
        source = digit8;
    } else if (ch == '9') {
        source = digit9;
    } else if (ch == 'H') {
        source = h;
    } else if (ch == 'E') {
        source = e;
    } else if (ch == 'L') {
        source = l;
    } else if (ch == 'O') {
        source = o;
    }

    for (index = 0U; index < 5U; index++) {
        glyph[index] = source[index];
    }
}

static uint8_t Oled_WriteChar(char ch)
{
    uint8_t glyph[OLED_CHAR_WIDTH];

    Oled_GetGlyph(ch, glyph);
    glyph[5] = 0x00U;
    return Oled_WriteData(glyph, sizeof(glyph));
}

static uint8_t Oled_WriteString(const char *text)
{
    while (*text != '\0') {
        if (Oled_WriteChar(*text) == 0U) {
            return 0U;
        }
        text++;
    }

    return 1U;
}

static void Oled_ScaleGlyphColumn2x(uint8_t source, uint8_t *low,
                                      uint8_t *high)
{
    uint8_t bit;
    uint16_t scaled = 0U;

    for (bit = 0U; bit < 7U; bit++) {
        if ((source & (uint8_t) (1U << bit)) != 0U) {
            scaled |= (uint16_t) (1U << (bit * 2U));
            scaled |= (uint16_t) (1U << ((bit * 2U) + 1U));
        }
    }

    *low = (uint8_t) (scaled & 0xFFU);
    *high = (uint8_t) ((scaled >> 8U) & 0xFFU);
}

static uint8_t Oled_WriteLargeString(uint8_t page, uint8_t column,
                                      const char *text)
{
    uint8_t upper[96];
    uint8_t lower[96];
    uint8_t glyph[5];
    uint8_t text_index;
    uint8_t glyph_index;
    uint8_t out_index = 0U;
    uint8_t upper_byte;
    uint8_t lower_byte;

    for (text_index = 0U; text[text_index] != '\0'; text_index++) {
        Oled_GetGlyph(text[text_index], glyph);
        for (glyph_index = 0U; glyph_index < 5U; glyph_index++) {
            Oled_ScaleGlyphColumn2x(glyph[glyph_index], &upper_byte,
                                    &lower_byte);
            upper[out_index] = upper_byte;
            lower[out_index] = lower_byte;
            out_index++;
            upper[out_index] = upper_byte;
            lower[out_index] = lower_byte;
            out_index++;
        }
        upper[out_index] = 0x00U;
        lower[out_index] = 0x00U;
        out_index++;
        upper[out_index] = 0x00U;
        lower[out_index] = 0x00U;
        out_index++;
    }

    if (Oled_SetCursor(page, column) == 0U) {
        return 0U;
    }
    if (Oled_WriteData(upper, out_index) == 0U) {
        return 0U;
    }
    if (Oled_SetCursor((uint8_t) (page + 1U), column) == 0U) {
        return 0U;
    }
    return Oled_WriteData(lower, out_index);
}

static uint8_t Oled_Clear(void)
{
    uint8_t page;
    uint8_t column;
    uint8_t zeros[16];

    for (column = 0U; column < sizeof(zeros); column++) {
        zeros[column] = 0x00U;
    }

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        if (Oled_SetCursor(page, 0U) == 0U) {
            return 0U;
        }
        for (column = 0U; column < OLED_WIDTH; column += sizeof(zeros)) {
            if (Oled_WriteData(zeros, sizeof(zeros)) == 0U) {
                return 0U;
            }
        }
    }

    return Oled_SetCursor(0U, 0U);
}

void OledDisplay_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0x20U, 0x02U, 0xB0U, 0xC8U, 0x00U, 0x10U, 0x40U,
        0x81U, 0x7FU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
        0x00U, 0xD5U, 0x80U, 0xD9U, 0xF1U, 0xDAU, 0x12U, 0xDBU,
        0x40U, 0x8DU, 0x14U, 0xAFU
    };
    uint8_t index;

    s_oled_ready = 0U;
    HAL_Delay(100U);

    if (HAL_I2C_IsDeviceReady(&hi2c4, OLED_I2C_ADDRESS, 2U,
                              OLED_I2C_TIMEOUT_MS) != HAL_OK) {
        return;
    }

    for (index = 0U; index < sizeof(init_commands); index++) {
        if (Oled_WriteCommand(init_commands[index]) == 0U) {
            return;
        }
    }

    if (Oled_Clear() == 0U) {
        return;
    }

    s_oled_ready = 1U;
}

void OledDisplay_ShowHello(void)
{
    if (s_oled_ready == 0U) {
        return;
    }

    if (Oled_SetCursor(0U, 0U) == 0U) {
        s_oled_ready = 0U;
        return;
    }

    if (Oled_WriteString("HELLO") == 0U) {
        s_oled_ready = 0U;
    }
}

void OledDisplay_ShowTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    char text[9];

    if (s_oled_ready == 0U) {
        return;
    }

    hours %= 100U;
    minutes %= 60U;
    seconds %= 60U;
    text[0] = (char) ('0' + (hours / 10U));
    text[1] = (char) ('0' + (hours % 10U));
    text[2] = ':';
    text[3] = (char) ('0' + (minutes / 10U));
    text[4] = (char) ('0' + (minutes % 10U));
    text[5] = ':';
    text[6] = (char) ('0' + (seconds / 10U));
    text[7] = (char) ('0' + (seconds % 10U));
    text[8] = '\0';

    if (Oled_WriteLargeString(2U, 16U, text) == 0U) {
        s_oled_ready = 0U;
    }
}

uint8_t OledDisplay_IsReady(void)
{
    return s_oled_ready;
}