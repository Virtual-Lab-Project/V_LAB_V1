/*
 * comm.h
 *
 *  Created on: Jan 19, 2023
 *      Author: anand
 */

#ifndef SRC_COMM_H_
#define SRC_COMM_H_

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* SRC_COMM_H_ */
