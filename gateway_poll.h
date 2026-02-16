/*
 * gateway_poll.h
 *
 *  Created on: 4 Sub 2026
 *      Author: burak.guvelioglu
 */
#ifndef INC_GATEWAY_POLL_H_
#define INC_GATEWAY_POLL_H_

/**
 * @brief Periodic gateway task.
 *
 * This function should be called frequently from main loop.
 * It executes a 100ms scheduler:
 * - Reads RF measurements and updates cached registers
 * - Applies pending register writes to RF power supply
 */
void gatewayTaskProcess(void);

#endif /* INC_GATEWAY_POLL_H_ */
