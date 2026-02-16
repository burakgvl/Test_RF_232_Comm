/*
 * gateway_regs.h
 *
 *  Created on: 4 Sub 2026
 *      Author: burak.guvelioglu
 */
#ifndef INC_GATEWAY_REGS_H_
#define INC_GATEWAY_REGS_H_

#include <stdint.h>

/* Modbus holding register addresses */
#define REG_RAMP_UP                (105U)
#define REG_RAMP_DOWN              (106U)
#define REG_RF_ONOFF               (121U)
#define REG_SETPOINT               (122U)
#define REG_FWD_PWR                (123U)
#define REG_REFL_PWR               (124U)

/* Internal pending write mask bits */
#define GATEWAY_WRITE_RAMP_UP      (0x01U)
#define GATEWAY_WRITE_RAMP_DOWN    (0x02U)
#define GATEWAY_WRITE_RF_ONOFF     (0x04U)
#define GATEWAY_WRITE_SETPOINT     (0x08U)

/**
 * @brief Gateway register cache.
 *
 * This structure contains:
 * - Desired values written by PLC (rampUpTimeSec, setpoint, etc.)
 * - Latest measured values read from RF power supply (forwardPower, reflectedPower)
 * - Internal fields used by gateway scheduler (pendingWriteMask, lastSent values, timing)
 */
typedef struct
{
    /* Desired / cached settings (written by PLC, applied to RF by gateway task) */
    uint16_t rampUpTimeSec;
    uint16_t rampDownTimeSec;
    uint16_t rfOnOff;
    uint16_t setpoint;

    /* Measured values (read from RF power supply periodically) */
    uint16_t forwardPower;
    uint16_t reflectedPower;

    /* Internal: scheduling */
    uint32_t lastWriteTick;
    uint8_t pendingWriteMask;
    uint8_t reserved1;
    uint16_t reserved2;
} GatewayRegs_t;

extern volatile GatewayRegs_t gatewayRegs;

#endif /* INC_GATEWAY_REGS_H_ */
