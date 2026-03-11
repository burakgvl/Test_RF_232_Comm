/*
 * rf_serial.c
 *
 *  Created on: 4 Sub 2026
 *      Author: burak.guvelioglu
 */
#include "rf_serial.h"
#include <string.h>
#include <ctype.h>

#define RF_LINE_MAX                 (64U)
#define RF_UART_TX_TIMEOUT_MS       (200U)
#define RF_DISCARD_TMP_MAX          (8U)
#define RF_LINE_POLL_SLICE_MS        (5U)

static UART_HandleTypeDef *rfUartHandle = NULL;

static volatile bool rfRxStarted = false;

static volatile uint16_t rfLineLen = 0U;
static volatile bool rfLineReady = false;
static char rfLine[RF_LINE_MAX];

static bool parseFirstU16(const char *text, uint16_t *outVal);
static uint16_t cmdCoreLength(const char *cmd);
static bool isEchoLineOfCmd(const char *line, const char *cmd);
static bool readNonEchoLine(const char *cmd, char *out, uint16_t outMax, uint32_t timeoutMs);
static int8_t hexNibble(char ch);
static bool parseRfOnOffFromStatusLine(const char *line, uint16_t *outOnOff);

/**
 * @brief Initializes RF serial driver.
 */
void rfInit(UART_HandleTypeDef *uartHandle)
{
    rfUartHandle = uartHandle;

    rfLineLen = 0U;
    rfLineReady = false;

    rfRxStarted = true;
}

/**
 * @brief Processes one received byte from RF UART.
 */
void rfOnRxByte(uint8_t rxByte)
{
    uint8_t maskedByte;

    if (rfRxStarted == false)
    {
        return;
    }

    maskedByte = (uint8_t)(rxByte & 0x7FU);

    if (maskedByte == (uint8_t)'\r')
    {
        if (rfLineLen < RF_LINE_MAX)
        {
            rfLine[rfLineLen] = '\0';
        }

        rfLineReady = true;
        rfLineLen = 0U;
    }
    else
    {
        if (rfLineLen < (RF_LINE_MAX - 1U))
        {
            rfLine[rfLineLen] = (char)maskedByte;
            rfLineLen++;
        }
    }
}

void rfClearBufferedLine(void)
{
    __disable_irq();
    rfLineReady = false;
    rfLineLen = 0U;
    __enable_irq();
}

void rfDrainRx(uint32_t timeoutMs)
{
    char tmp[RF_DISCARD_TMP_MAX];
    uint32_t startTick;

    startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeoutMs)
    {
        if (rfReadLine(tmp, (uint16_t)sizeof(tmp), RF_LINE_POLL_SLICE_MS) == false)
        {
            continue;
        }
    }
}

/**
 * @brief Sends raw command string to RF UART.
 */
bool rfSendCmd(const char *cmd)
{
    HAL_StatusTypeDef status;

    if (rfUartHandle == NULL)
    {
        return false;
    }

    if (cmd == NULL)
    {
        return false;
    }

    status = HAL_UART_Transmit(
        rfUartHandle,
        (uint8_t *)cmd,
        (uint16_t)strlen(cmd),
        RF_UART_TX_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        return false;
    }

    return true;
}

/**
 * @brief Reads one response line terminated by CR.
 */
bool rfReadLine(char *out, uint16_t outMax, uint32_t timeoutMs)
{
    uint32_t startTick;

    if (out == NULL)
    {
        return false;
    }

    if (outMax == 0U)
    {
        return false;
    }

    startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeoutMs)
    {
        if (rfLineReady == true)
        {
            __disable_irq();
            rfLineReady = false;
            __enable_irq();

            strncpy(out, rfLine, outMax);
            out[outMax - 1U] = '\0';

            return true;
        }
    }

    return false;
}

/**
 * @brief Sends command and parses first uint16 value from response.
 */
bool rfQueryU16(const char *cmd, uint16_t *outVal, uint32_t timeoutMs)
{
    char line[RF_LINE_MAX];

    if (cmd == NULL)
    {
        return false;
    }

    if (outVal == NULL)
    {
        return false;
    }

    rfClearBufferedLine();

    if (rfSendCmd(cmd) == false)
    {
        return false;
    }

    while (readNonEchoLine(cmd, line, (uint16_t)sizeof(line), timeoutMs) == true)
    {
        if (parseFirstU16(line, outVal) == true)
        {
            return true;
        }
    }

    return false;
}

bool rfQueryOnOffStatus(uint16_t *outOnOff, uint32_t timeoutMs)
{
    char line[RF_LINE_MAX];

    if (outOnOff == NULL)
    {
        return false;
    }

    rfClearBufferedLine();

    if (rfSendCmd("R\r") == false)
    {
        return false;
    }

    while (readNonEchoLine("R\r", line, (uint16_t)sizeof(line), timeoutMs) == true)
    {
        if (parseRfOnOffFromStatusLine(line, outOnOff) == true)
        {
            return true;
        }
    }

    return false;
}


static uint16_t cmdCoreLength(const char *cmd)
{
    uint16_t len;

    len = (uint16_t)strlen(cmd);

    while ((len > 0U) && ((cmd[len - 1U] == '\r') || (cmd[len - 1U] == '\n')))
    {
        len--;
    }

    return len;
}

static bool isEchoLineOfCmd(const char *line, const char *cmd)
{
    uint16_t coreLen;

    if (line == NULL)
    {
        return false;
    }

    if (cmd == NULL)
    {
        return false;
    }

    coreLen = cmdCoreLength(cmd);

    if (coreLen == 0U)
    {
        return false;
    }

    if (strncmp(line, cmd, coreLen) == 0)
    {
        if ((line[coreLen] == '\0') || (line[coreLen] == ' ') || (line[coreLen] == ':') || (line[coreLen] == '>'))
        {
            return true;
        }
    }

    return false;
}

static bool readNonEchoLine(const char *cmd, char *out, uint16_t outMax, uint32_t timeoutMs)
{
    uint32_t startTick;

    if (out == NULL)
    {
        return false;
    }

    if (outMax == 0U)
    {
        return false;
    }

    startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeoutMs)
    {
        uint32_t remainMs;

        remainMs = timeoutMs - (HAL_GetTick() - startTick);
        if (rfReadLine(out, outMax, remainMs) == false)
        {
            return false;
        }

        if (isEchoLineOfCmd(out, cmd) == true)
        {
            continue;
        }

        return true;
    }

    return false;
}

static int8_t hexNibble(char ch)
{
    if ((ch >= '0') && (ch <= '9'))
    {
        return (int8_t)(ch - '0');
    }

    if ((ch >= 'A') && (ch <= 'F'))
    {
        return (int8_t)(10 + (ch - 'A'));
    }

    if ((ch >= 'a') && (ch <= 'f'))
    {
        return (int8_t)(10 + (ch - 'a'));
    }

    return -1;
}

static bool parseRfOnOffFromStatusLine(const char *line, uint16_t *outOnOff)
{
    uint16_t i;

    if (line == NULL)
    {
        return false;
    }

    if (outOnOff == NULL)
    {
        return false;
    }

    for (i = 0U; line[i] != '\0'; i++)
    {
        uint16_t j;
        int8_t nibble;

        for (j = 0U; j < 7U; j++)
        {
            if (line[i + j] == '\0')
            {
                break;
            }

            if (isxdigit((unsigned char)line[i + j]) == 0)
            {
                break;
            }
        }

        if (j < 7U)
        {
            continue;
        }

        nibble = hexNibble(line[i + 3U]);
        if (nibble < 0)
        {
            continue;
        }

        if ((nibble & 0x08) != 0)
        {
            *outOnOff = 1U;
        }
        else
        {
            *outOnOff = 0U;
        }

        return true;
    }

    return false;
}

static bool parseFirstU16(const char *text, uint16_t *outVal)
{
    const char *p;
    uint32_t value;

    if (text == NULL)
    {
        return false;
    }

    if (outVal == NULL)
    {
        return false;
    }

    p = text;

    while ((*p != '\0') && (isdigit((unsigned char)*p) == 0))
    {
        p++;
    }

    if (*p == '\0')
    {
        return false;
    }

    value = 0U;

    while ((*p != '\0') && (isdigit((unsigned char)*p) != 0))
    {
        value = (value * 10U) + (uint32_t)(*p - '0');

        if (value > 65535U)
        {
            return false;
        }

        p++;
    }

    *outVal = (uint16_t)value;

    return true;
}
