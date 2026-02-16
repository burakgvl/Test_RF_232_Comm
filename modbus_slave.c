/*
 * modbus_slave.c
 *
 *  Created on: 4 Sub 2026
 *      Author: burak.guvelioglu
 */
#include "modbus_slave.h"
#include "gateway_regs.h"
#include <string.h>

#define MODBUS_RX_MAX                (256U)
#define MODBUS_FRAME_GAP_MS          (3U)

#define MODBUS_FUNC_READ_HOLDING     (0x03U)
#define MODBUS_FUNC_WRITE_SINGLE     (0x06U)
#define MODBUS_FUNC_WRITE_MULTIPLE   (0x10U)

#define MODBUS_EX_ILLEGAL_FUNCTION   (0x01U)
#define MODBUS_EX_ILLEGAL_ADDRESS    (0x02U)
#define MODBUS_EX_ILLEGAL_VALUE      (0x03U)
#define MODBUS_EX_SLAVE_FAILURE      (0x04U)

#define MODBUS_READ_MAX_QTY          (64U)

static uint8_t rxBuffer[MODBUS_RX_MAX];
static volatile uint16_t rxLen = 0U;
static volatile uint32_t lastByteTick = 0U;

static void rs485SetTx(bool enabled);
static void sendFrame(ModbusSlave_t *modbusSlave, const uint8_t *data, uint16_t dataLen);
static void sendException(ModbusSlave_t *modbusSlave, uint8_t funcCode, uint8_t exceptionCode);
static void putU16Be(uint8_t *dst, uint16_t value);
static bool getRegU16(uint16_t addr, uint16_t *outVal);
static bool setRegU16(uint16_t addr, uint16_t value);
static void handleFrame(ModbusSlave_t *modbusSlave, const uint8_t *frame, uint16_t frameLen);

static void rs485SetTx(bool enabled)
{
    (void)enabled;
}

static void sendFrame(ModbusSlave_t *modbusSlave, const uint8_t *data, uint16_t dataLen)
{
    rs485SetTx(true);
    HAL_UART_Transmit(modbusSlave->uartHandle, (uint8_t *)data, dataLen, 200U);
    rs485SetTx(false);
}

static void sendException(ModbusSlave_t *modbusSlave, uint8_t funcCode, uint8_t exceptionCode)
{
    uint8_t resp[5U];
    uint16_t crc;

    resp[0U] = modbusSlave->slaveId;
    resp[1U] = (uint8_t)(funcCode | 0x80U);
    resp[2U] = exceptionCode;

    crc = modbusCrc16(resp, 3U);
    resp[3U] = (uint8_t)(crc & 0x00FFU);
    resp[4U] = (uint8_t)((crc >> 8U) & 0x00FFU);

    sendFrame(modbusSlave, resp, 5U);
}

static void putU16Be(uint8_t *dst, uint16_t value)
{
    dst[0U] = (uint8_t)((value >> 8U) & 0x00FFU);
    dst[1U] = (uint8_t)(value & 0x00FFU);
}

static bool getRegU16(uint16_t addr, uint16_t *outVal)
{
    if (outVal == NULL)
    {
        return false;
    }

    if (addr == REG_RAMP_UP)
    {
        *outVal = gatewayRegs.rampUpTimeSec;
        return true;
    }

    if (addr == REG_RAMP_DOWN)
    {
        *outVal = gatewayRegs.rampDownTimeSec;
        return true;
    }

    if (addr == REG_RF_ONOFF)
    {
        *outVal = (uint16_t)(gatewayRegs.rfOnOff & 0x0001U);
        return true;
    }

    if (addr == REG_SETPOINT)
    {
        *outVal = gatewayRegs.setpoint;
        return true;
    }

    if (addr == REG_FWD_PWR)
    {
        *outVal = gatewayRegs.forwardPower;
        return true;
    }

    if (addr == REG_REFL_PWR)
    {
        *outVal = gatewayRegs.reflectedPower;
        return true;
    }

    return false;
}

static bool setRegU16(uint16_t addr, uint16_t value)
{
    bool handled;

    handled = true;

    __disable_irq();

    if (addr == REG_RAMP_UP)
    {
        gatewayRegs.rampUpTimeSec = value;
        gatewayRegs.pendingWriteMask |= GATEWAY_WRITE_RAMP_UP;
    }
    else if (addr == REG_RAMP_DOWN)
    {
        gatewayRegs.rampDownTimeSec = value;
        gatewayRegs.pendingWriteMask |= GATEWAY_WRITE_RAMP_DOWN;
    }
    else if (addr == REG_RF_ONOFF)
    {
        gatewayRegs.rfOnOff = (uint16_t)(value & 0x0001U);
        gatewayRegs.pendingWriteMask |= GATEWAY_WRITE_RF_ONOFF;
    }
    else if (addr == REG_SETPOINT)
    {
        gatewayRegs.setpoint = value;
        gatewayRegs.pendingWriteMask |= GATEWAY_WRITE_SETPOINT;
    }
    else
    {
        handled = false;
    }

    __enable_irq();

    return handled;
}

/**
 * @brief Calculates Modbus RTU CRC16.
 */
uint16_t modbusCrc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc;
    uint16_t pos;
    uint8_t bit;

    if (buf == NULL)
    {
        return 0U;
    }

    crc = 0xFFFFU;

    for (pos = 0U; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc = (uint16_t)(crc >> 1U);
            }
        }
    }

    return crc;
}

/**
 * @brief Initializes Modbus RTU slave instance.
 */
void modbusSlaveInit(ModbusSlave_t *modbusSlave, UART_HandleTypeDef *uartHandle, uint8_t slaveId)
{
    if (modbusSlave == NULL)
    {
        return;
    }

    modbusSlave->uartHandle = uartHandle;
    modbusSlave->slaveId = slaveId;

    rxLen = 0U;
    lastByteTick = HAL_GetTick();
}

/**
 * @brief Feeds one received byte to Modbus RTU frame collector.
 */
void modbusSlaveOnRxByte(ModbusSlave_t *modbusSlave, uint8_t rxByte)
{
    (void)modbusSlave;

    if (rxLen < MODBUS_RX_MAX)
    {
        rxBuffer[rxLen] = rxByte;
        rxLen++;

        lastByteTick = HAL_GetTick();
    }
    else
    {
        rxLen = 0U;
    }
}

static void handleFrame(ModbusSlave_t *modbusSlave, const uint8_t *frame, uint16_t frameLen)
{
    uint16_t crcRx;
    uint16_t crcCalc;
    uint8_t func;

    if (modbusSlave == NULL)
    {
        return;
    }

    if (frame == NULL)
    {
        return;
    }

    if (frameLen < 4U)
    {
        return;
    }

    if (frame[0U] != modbusSlave->slaveId)
    {
        return;
    }

    crcRx = (uint16_t)frame[frameLen - 2U] | ((uint16_t)frame[frameLen - 1U] << 8U);
    crcCalc = modbusCrc16(frame, (uint16_t)(frameLen - 2U));

    if (crcCalc != crcRx)
    {
        return;
    }

    func = frame[1U];

    if (func == MODBUS_FUNC_READ_HOLDING)
    {
        uint16_t startAddr;
        uint16_t qty;
        uint16_t i;
        uint16_t outLen;
        uint16_t outCrc;
        uint8_t resp[3U + (2U * MODBUS_READ_MAX_QTY) + 2U];

        if (frameLen != 8U)
        {
            sendException(modbusSlave, func, MODBUS_EX_ILLEGAL_VALUE);
            return;
        }

        startAddr = (uint16_t)((uint16_t)frame[2U] << 8U) | frame[3U];
        qty = (uint16_t)((uint16_t)frame[4U] << 8U) | frame[5U];

        if ((qty == 0U) || (qty > MODBUS_READ_MAX_QTY))
        {
            sendException(modbusSlave, func, MODBUS_EX_ILLEGAL_VALUE);
            return;
        }

        resp[0U] = modbusSlave->slaveId;
        resp[1U] = func;
        resp[2U] = (uint8_t)(qty * 2U);

        for (i = 0U; i < qty; i++)
        {
            uint16_t value;
            bool ok;

            ok = getRegU16((uint16_t)(startAddr + i), &value);
            if (ok == false)
            {
                sendException(modbusSlave, func, MODBUS_EX_ILLEGAL_ADDRESS);
                return;
            }

            putU16Be(&resp[3U + (2U * i)], value);
        }

        outLen = (uint16_t)(3U + (2U * qty));
        outCrc = modbusCrc16(resp, outLen);

        resp[outLen + 0U] = (uint8_t)(outCrc & 0x00FFU);
        resp[outLen + 1U] = (uint8_t)((outCrc >> 8U) & 0x00FFU);

        sendFrame(modbusSlave, resp, (uint16_t)(outLen + 2U));
        return;
    }

    if (func == MODBUS_FUNC_WRITE_SINGLE)
    {
        uint16_t addr;
        uint16_t value;
        uint16_t outCrc;
        uint8_t resp[8U];
        bool ok;

        if (frameLen != 8U)
        {
            sendException(modbusSlave, func, MODBUS_EX_ILLEGAL_VALUE);
            return;
        }

        addr = (uint16_t)((uint16_t)frame[2U] << 8U) | frame[3U];
        value = (uint16_t)((uint16_t)frame[4U] << 8U) | frame[5U];

        ok = setRegU16(addr, value);
        if (ok == false)
        {
            sendException(modbusSlave, func, MODBUS_EX_SLAVE_FAILURE);
            return;
        }

        memcpy(resp, frame, 6U);
        outCrc = modbusCrc16(resp, 6U);
        resp[6U] = (uint8_t)(outCrc & 0x00FFU);
        resp[7U] = (uint8_t)((outCrc >> 8U) & 0x00FFU);

        sendFrame(modbusSlave, resp, 8U);
        return;
    }

    if (func == MODBUS_FUNC_WRITE_MULTIPLE)
    {
        uint16_t startAddr;
        uint16_t qty;
        uint8_t byteCount;
        uint16_t i;
        uint16_t outCrc;
        uint8_t resp[8U];

        if (frameLen < 9U)
        {
            sendException(modbusSlave, func, MODBUS_EX_ILLEGAL_VALUE);
            return;
        }

        startAddr = (uint16_t)((uint16_t)frame[2U] << 8U) | frame[3U];
        qty = (uint16_t)((uint16_t)frame[4U] << 8U) | frame[5U];
        byteCount = frame[6U];

        if ((qty == 0U) || (qty > MODBUS_READ_MAX_QTY) || (byteCount != (uint8_t)(qty * 2U)))
        {
            sendException(modbusSlave, func, MODBUS_EX_ILLEGAL_VALUE);
            return;
        }

        for (i = 0U; i < qty; i++)
        {
            uint16_t value;
            bool ok;

            value = (uint16_t)((uint16_t)frame[7U + (2U * i)] << 8U) | frame[8U + (2U * i)];

            ok = setRegU16((uint16_t)(startAddr + i), value);
            if (ok == false)
            {
                sendException(modbusSlave, func, MODBUS_EX_SLAVE_FAILURE);
                return;
            }
        }

        resp[0U] = modbusSlave->slaveId;
        resp[1U] = func;
        resp[2U] = (uint8_t)((startAddr >> 8U) & 0x00FFU);
        resp[3U] = (uint8_t)(startAddr & 0x00FFU);
        resp[4U] = (uint8_t)((qty >> 8U) & 0x00FFU);
        resp[5U] = (uint8_t)(qty & 0x00FFU);

        outCrc = modbusCrc16(resp, 6U);
        resp[6U] = (uint8_t)(outCrc & 0x00FFU);
        resp[7U] = (uint8_t)((outCrc >> 8U) & 0x00FFU);

        sendFrame(modbusSlave, resp, 8U);
        return;
    }

    sendException(modbusSlave, func, MODBUS_EX_ILLEGAL_FUNCTION);
}

/**
 * @brief Processes Modbus RTU frames and sends responses.
 */
void modbusSlaveProcess(ModbusSlave_t *modbusSlave)
{
    uint16_t len;
    uint8_t frame[MODBUS_RX_MAX];
    uint32_t nowTick;

    if (rxLen == 0U)
    {
        return;
    }

    nowTick = HAL_GetTick();

    if ((nowTick - lastByteTick) < MODBUS_FRAME_GAP_MS)
    {
        return;
    }

    __disable_irq();
    len = rxLen;
    memcpy(frame, rxBuffer, len);
    rxLen = 0U;
    __enable_irq();

    handleFrame(modbusSlave, frame, len);
}
