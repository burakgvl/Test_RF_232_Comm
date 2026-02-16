/*
 * modbus_slave.h
 *
 *  Created on: 4 Sub 2026
 *      Author: burak.guvelioglu
 */
#ifndef INC_MODBUS_SLAVE_H_
#define INC_MODBUS_SLAVE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Modbus RTU slave instance.
 */
typedef struct
{
    UART_HandleTypeDef *uartHandle;
    uint8_t slaveId;
} ModbusSlave_t;

/**
 * @brief Initializes Modbus RTU slave.
 * @param modbusSlave Modbus slave instance.
 * @param uartHandle UART handle for PLC Modbus (USART2).
 * @param slaveId Modbus slave address.
 */
void modbusSlaveInit(ModbusSlave_t *modbusSlave, UART_HandleTypeDef *uartHandle, uint8_t slaveId);

/**
 * @brief Feeds one received byte to Modbus RTU frame collector.
 * @param modbusSlave Modbus slave instance.
 * @param rxByte Received byte.
 */
void modbusSlaveOnRxByte(ModbusSlave_t *modbusSlave, uint8_t rxByte);

/**
 * @brief Processes Modbus RTU frames and sends responses.
 * @param modbusSlave Modbus slave instance.
 */
void modbusSlaveProcess(ModbusSlave_t *modbusSlave);

/**
 * @brief Calculates Modbus RTU CRC16.
 * @param buf Input buffer.
 * @param len Length.
 * @return CRC16 value.
 */
uint16_t modbusCrc16(const uint8_t *buf, uint16_t len);

#endif /* INC_MODBUS_SLAVE_H_ */
