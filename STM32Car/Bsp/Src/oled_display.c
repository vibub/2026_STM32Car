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
    static const uint8_t h[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
    static const uint8_t e[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t l[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
    static const uint8_t o[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
    const uint8_t *source;

    source = blank;
    if (ch == 'H') {
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

uint8_t OledDisplay_IsReady(void)
{
    return s_oled_ready;
}