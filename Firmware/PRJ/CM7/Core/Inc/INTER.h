/*
 * INTER.h
 *
 * Created on: Jun 2, 2026
 * Author: Geral
 */

#ifndef INC_INTER_H_
#define INC_INTER_H_

#include "stm32h7xx_hal.h"

/* =========================================================
   Interface Pública de Comunicação (Node-RED)
   ========================================================= */

/* Processamento Principal */
void Update_Interface(UART_HandleTypeDef *huart); // Compila as strings JSON e envia por DMA

#endif /* INC_INTER_H_ */
