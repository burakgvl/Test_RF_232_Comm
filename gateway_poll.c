/*
 * gateway_poll.c
 *
 *  Created on: 4 Sub 2026
 *      Author: burak.guvelioglu
 */
#include "gateway_poll.h"
#include "rf_serial.h"
#include "gateway_regs.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>

#define GATEWAY_TASK_PERIOD_MS          (100U)
#define GATEWAY_RF_QUERY_TIMEOUT_MS     (100U)
#define GATEWAY_RF_WRITE_COOLDOWN_MS    (100U)
#define GATEWAY_RF_WRITE_DISCARD_MS     (30U)

static void applyPendingWrites(uint32_t nowTick);
static uint8_t getPendingMaskSnapshot(uint16_t *rampUp,
                                      uint16_t *rampDown,
                                      uint16_t *rfOnOff,
                                      uint16_t *setpoint);
static void clearPendingBitIfMatched(uint8_t bit, uint16_t desiredValue);

volatile GatewayRegs_t gatewayRegs =
{
    .rampUpTimeSec = 0U,
    .rampDownTimeSec = 0U,
    .rfOnOff = 0U,
    .setpoint = 0U,

    .forwardPower = 0U,
    .reflectedPower = 0U,

    .lastWriteTick = 0U,
    .pendingWriteMask = 0U,
    .reserved1 = 0U,
    .reserved2 = 0U
};
/**
 * @brief Periodic gateway task.
 *
 * Scheduler runs every 100ms:
 * - Read forward/reflected power and update cache
 * - Apply one pending write to RF (priority based)
 */
void gatewayTaskProcess(void)
{
    static uint32_t lastTaskTick = 0U;
    uint32_t nowTick;
    uint16_t value;
    bool ok;

    nowTick = HAL_GetTick();

    if ((nowTick - lastTaskTick) < GATEWAY_TASK_PERIOD_MS)
    {
        return;
    }

    lastTaskTick = nowTick;

    ok = rfQueryU16("W?\r", &value, GATEWAY_RF_QUERY_TIMEOUT_MS);
    if (ok == true)
    {
        gatewayRegs.forwardPower = value;
    }

    ok = rfQueryU16("R?\r", &value, GATEWAY_RF_QUERY_TIMEOUT_MS);
    if (ok == true)
    {
        gatewayRegs.reflectedPower = value;
    }

    ok = rfQueryU16("QRU\r", &value, GATEWAY_RF_QUERY_TIMEOUT_MS);
    if (ok == true)
    {
        if ((gatewayRegs.pendingWriteMask & GATEWAY_WRITE_RAMP_UP) == 0U)
        {
            gatewayRegs.rampUpTimeSec = value;
        }
    }

    ok = rfQueryU16("QRD\r", &value, GATEWAY_RF_QUERY_TIMEOUT_MS);
    if (ok == true)
    {
        if ((gatewayRegs.pendingWriteMask & GATEWAY_WRITE_RAMP_DOWN) == 0U)
        {
            gatewayRegs.rampDownTimeSec = value;
        }
    }

    ok = rfQueryU16("QSET\r", &value, GATEWAY_RF_QUERY_TIMEOUT_MS);
    if (ok == true)
    {
        if ((gatewayRegs.pendingWriteMask & GATEWAY_WRITE_SETPOINT) == 0U)
        {
            gatewayRegs.setpoint = value;
        }
    }

    ok = rfQueryU16("R\r", &value, GATEWAY_RF_QUERY_TIMEOUT_MS);
    if (ok == true)
    {
        if ((gatewayRegs.pendingWriteMask & GATEWAY_WRITE_RF_ONOFF) == 0U)
        {
            gatewayRegs.rfOnOff = value;
        }
    }

    applyPendingWrites(nowTick);
}

/**
 * @brief Takes an atomic snapshot of pending write mask and desired register values.
 *
 * This function enters a short critical section to ensure that the values read are
 * consistent with each other (mask + desired values).
 *
 * @param rampUp   Output pointer for desired ramp-up time (s)
 * @param rampDown Output pointer for desired ramp-down time (s)
 * @param rfOnOff  Output pointer for desired RF on/off value (bit0)
 * @param setpoint Output pointer for desired power setpoint (W)
 *
 * @return Pending write mask snapshot.
 */
static uint8_t getPendingMaskSnapshot(uint16_t *rampUp,
                                      uint16_t *rampDown,
                                      uint16_t *rfOnOff,
                                      uint16_t *setpoint)
{
    uint8_t mask;

    __disable_irq();
    mask = gatewayRegs.pendingWriteMask;
    *rampUp = gatewayRegs.rampUpTimeSec;
    *rampDown = gatewayRegs.rampDownTimeSec;
    *rfOnOff = gatewayRegs.rfOnOff;
    *setpoint = gatewayRegs.setpoint;
    __enable_irq();

    return mask;
}

/**
 * @brief Clears a pending write bit if the desired value is still current.
 *
 * This avoids clearing a pending bit if the PLC updated the desired register value
 * while the RF write was in progress.
 *
 * Note: Same-value write suppression is intentionally NOT used. Only time-based
 * cooldown controls the write frequency.
 *
 * @param bit          Pending bit to clear (one of GATEWAY_WRITE_*)
 * @param desiredValue Desired value that was written to RF
 */
static void clearPendingBitIfMatched(uint8_t bit, uint16_t desiredValue)
{
    __disable_irq();

    if (bit == GATEWAY_WRITE_RAMP_UP)
    {
        if (gatewayRegs.rampUpTimeSec == desiredValue)
        {
            gatewayRegs.pendingWriteMask &= (uint8_t)(~GATEWAY_WRITE_RAMP_UP);
        }
    }
    else if (bit == GATEWAY_WRITE_RAMP_DOWN)
    {
        if (gatewayRegs.rampDownTimeSec == desiredValue)
        {
            gatewayRegs.pendingWriteMask &= (uint8_t)(~GATEWAY_WRITE_RAMP_DOWN);
        }
    }
    else if (bit == GATEWAY_WRITE_RF_ONOFF)
    {
        if (gatewayRegs.rfOnOff == desiredValue)
        {
            gatewayRegs.pendingWriteMask &= (uint8_t)(~GATEWAY_WRITE_RF_ONOFF);
        }
    }
    else if (bit == GATEWAY_WRITE_SETPOINT)
    {
        if (gatewayRegs.setpoint == desiredValue)
        {
            gatewayRegs.pendingWriteMask &= (uint8_t)(~GATEWAY_WRITE_SETPOINT);
        }
    }
    else
    {
    }

    gatewayRegs.lastWriteTick = HAL_GetTick();

    __enable_irq();
}

/**
 * @brief Applies at most one pending write to the RF power supply.
 *
 * This function enforces a time-based write cooldown. If the cooldown has not elapsed,
 * no write is performed. If multiple writes are pending, they are applied in priority:
 * 1) RF On/Off
 * 2) Setpoint
 * 3) Ramp Up
 * 4) Ramp Down
 *
 * @param nowTick Current system tick (ms)
 */
static void applyPendingWrites(uint32_t nowTick)
{
    uint16_t rampUp;
    uint16_t rampDown;
    uint16_t rfOnOff;
    uint16_t setpoint;
    uint8_t mask;
    char cmd[24U];
    bool ok;
    uint32_t elapsed;

    elapsed = nowTick - gatewayRegs.lastWriteTick;

    if (elapsed < GATEWAY_RF_WRITE_COOLDOWN_MS)
    {
        return;
    }

    mask = getPendingMaskSnapshot(&rampUp, &rampDown, &rfOnOff, &setpoint);

    if ((mask & GATEWAY_WRITE_RF_ONOFF) != 0U)
    {
        if ((rfOnOff & 0x0001U) != 0U)
        {
            ok = rfSendCmd("G\r");
        }
        else
        {
            ok = rfSendCmd("S\r");
        }

        if (ok == true)
        {
            clearPendingBitIfMatched(GATEWAY_WRITE_RF_ONOFF, rfOnOff);
            rfDrainRx(GATEWAY_RF_WRITE_DISCARD_MS);
        }

        return;
    }

    if ((mask & GATEWAY_WRITE_SETPOINT) != 0U)
    {
        (void)snprintf(cmd, sizeof(cmd), "%u_W\r", (unsigned)setpoint);
        ok = rfSendCmd(cmd);

        if (ok == true)
        {
            clearPendingBitIfMatched(GATEWAY_WRITE_SETPOINT, setpoint);
            rfDrainRx(GATEWAY_RF_WRITE_DISCARD_MS);
        }

        return;
    }

    if ((mask & GATEWAY_WRITE_RAMP_UP) != 0U)
    {
        (void)snprintf(cmd, sizeof(cmd), "%u_UP\r", (unsigned)rampUp);
        ok = rfSendCmd(cmd);

        if (ok == true)
        {
            clearPendingBitIfMatched(GATEWAY_WRITE_RAMP_UP, rampUp);
            rfDrainRx(GATEWAY_RF_WRITE_DISCARD_MS);
        }

        return;
    }

    if ((mask & GATEWAY_WRITE_RAMP_DOWN) != 0U)
    {
        (void)snprintf(cmd, sizeof(cmd), "%u_DN\r", (unsigned)rampDown);
        ok = rfSendCmd(cmd);

        if (ok == true)
        {
            clearPendingBitIfMatched(GATEWAY_WRITE_RAMP_DOWN, rampDown);
            rfDrainRx(GATEWAY_RF_WRITE_DISCARD_MS);
        }

        return;
    }
}
