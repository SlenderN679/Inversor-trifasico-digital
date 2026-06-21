/*
 * SIMUL.h
 *
 * Created on: May 25, 2026
 * Author: Geral
 * Hardware-in-the-Loop Simulator.
 */

#ifndef INC_SIMUL_H_
#define INC_SIMUL_H_

#include "stm32h7xx_hal.h"

/* =========================================================
   Interface Pública do Simulador HIL
   ========================================================= */

/* Inicialização e Lógica Core */
void DacSim_Init(DAC_HandleTypeDef* hdac);        	 // Inicializa o hardware do DAC1
void DacSim_Update_Grid_DC(DAC_HandleTypeDef* hdac); // Atualiza os modelos físicos da Rede e Condensador
void Simul_Update_Motor_Currents(void);           	 // Atualiza a malha magnética do motor virtual

/* Getters: Sinais Simulados (Para injeção no Sense.c) */
float Simul_Get_I_A(void);     // Getter: Corrente simulada da Fase A
float Simul_Get_I_B(void);     // Getter: Corrente simulada da Fase B
float Simul_Get_I_C(void);     // Getter: Corrente simulada da Fase C
float Simul_Get_I_Grid(void);  // Getter: Corrente injetada na rede virtual

#endif /* INC_SIMUL_H_ */
