/*
 * rf_serial.h
 *
 *  Created on: 4 Sub 2026
 *      Author: burak.guvelioglu
 */
#ifndef INC_RF_SERIAL_H_
#define INC_RF_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Initializes RF serial driver.
 * @param uartHandle UART handle for LPUART1 (RF serial).
 */
void rfInit(UART_HandleTypeDef *uartHandle);

/**
 * @brief Feeds one received byte to RF serial line parser.
 * @param rxByte Received byte.
 */
void rfOnRxByte(uint8_t rxByte);

/**
 * @brief Sends a raw RF serial command.
 * @param cmd Null-terminated command string including '\r'.
 * @return true if transmit succeeded, false otherwise.
 */
bool rfSendCmd(const char *cmd);

/**
 * @brief Reads one response line terminated by '\r'.
 * @param out Output buffer.
 * @param outMax Output buffer size.
 * @param timeoutMs Timeout in milliseconds.
 * @return true if a line received, false on timeout.
 */
bool rfReadLine(char *out, uint16_t outMax, uint32_t timeoutMs);

/**
 * @brief Clears any currently buffered RF line.
 */
void rfClearBufferedLine(void);

/**
 * @brief Drains/consumes all RF lines that arrive within timeout window.
 *
 * Useful when command echo is enabled and/or write commands produce acknowledgements
 * that should not be interpreted as query responses.
 *
 * @param timeoutMs Timeout window in milliseconds.
 */
void rfDrainRx(uint32_t timeoutMs);

/**
 * @brief Sends a command and parses first uint16 value from the response.
 * @param cmd Command string including '\r'.
 * @param outVal Parsed value.
 * @param timeoutMs Timeout in milliseconds.
 * @return true on success, false otherwise.
 */
bool rfQueryU16(const char *cmd, uint16_t *outVal, uint32_t timeoutMs);

#endif /* INC_RF_SERIAL_H_ */
