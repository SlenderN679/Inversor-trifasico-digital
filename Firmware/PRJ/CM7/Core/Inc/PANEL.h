/*
 * Panel.h
 *
 * Created on: Apr 25, 2026
 * Author: Geral
 */

#ifndef INC_PANEL_H_
#define INC_PANEL_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* =========================================================
   Interface Pública da IHM (TFT Display e Encoder)
   ========================================================= */

/* Hardware Handles */
extern TIM_HandleTypeDef htim4; // Handle externo do Encoder Rotativo

/* Controlo da Interface Gráfica */
void Panel_Init_Sequence(void);    // Inicia o display SPI e o Encoder
void Panel_Update_UI(int safety);  // Processa inputs e desenha menus (Chamada a 10Hz)

#endif /* INC_PANEL_H_ */
