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

static UART_HandleTypeDef *rfUartHandle = NULL;

static volatile bool rfRxStarted = false;

static volatile uint16_t rfLineLen = 0U;
static volatile bool rfLineReady = false;
static char rfLine[RF_LINE_MAX];

static bool parseFirstU16(const char *text, uint16_t *outVal);

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

bool rfDiscardLine(uint32_t timeoutMs)
{
    char tmp[RF_DISCARD_TMP_MAX];

    return rfReadLine(tmp, (uint16_t)sizeof(tmp), timeoutMs);
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
    bool ok;

    if (outVal == NULL)
    {
        return false;
    }

    rfClearBufferedLine();

    ok = rfSendCmd(cmd);
    if (ok == false)
    {
        return false;
    }

    ok = rfReadLine(line, (uint16_t)sizeof(line), timeoutMs);
    if (ok == false)
    {
        return false;
    }

    ok = parseFirstU16(line, outVal);
    if (ok == false)
    {
        return false;
    }

    return true;
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
