/*
 * callbacks.c
 *
 *  Created on: 5 Sub 2026
 *      Author: burak.guvelioglu
 */
#include "main.h"
#include "modbus_slave.h"
#include "rf_serial.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern ModbusSlave_t modbusSlave;
extern uint8_t plcRxByte;
extern uint8_t rfRxByte;

/**
 * @brief UART RX complete callback.
 *
 * Routes received bytes to Modbus and RF parsers.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle == NULL)
    {
        return;
    }

    if (uartHandle->Instance == USART2)
    {
        modbusSlaveOnRxByte(&modbusSlave, plcRxByte);
        HAL_UART_Receive_IT(&huart2, &plcRxByte, 1U);
    }
    else if (uartHandle->Instance == USART1)
    {
        rfOnRxByte(rfRxByte);
        HAL_UART_Receive_IT(&huart1, &rfRxByte, 1U);
    }
    else
    {
    }
}
