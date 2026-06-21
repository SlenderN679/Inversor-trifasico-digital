/*
 * PWM.h
 *
 *  Created on: Apr 25, 2026
 *      Author: Geral
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "stm32h7xx_hal.h"

/* =========================================================
   Interface Pública do Módulo PWM
   ========================================================= */

/* 1. Definições de Constantes e Handles de Hardware */
#define PI 3.14159265358979323846f

/* Handles dos Temporizadores (Disponíveis externamente) */
extern TIM_HandleTypeDef htim1; // Inversor (TIM1)
extern TIM_HandleTypeDef htim8; // AFE (TIM8)

/* Variáveis de Estado do SPWM (Acesso externo para telemetria/controlo) */
extern float modulation_index; 	// Índice de modulação (0.0 a 1.0)
extern float target_frequency; 	// Frequência alvo do motor (Hz)
extern float phase_accumulator; // Acumulador de fase para o SPWM

/* 2. Funções de Inicialização e Controlo Geral */
void PWM_Start_All(void); 		// Inicializa todos os periféricos de PWM
void PWM_Emergency_Stop(void); 	// Paragem de emergência (corta saídas)
void Check_Safety(void); 		// Função de vigilância (Overvoltage/Overcurrent)
void PWM_Handle_SoftStart_Sequence(int vdc);
void PWM_Set_Main_Contactor(uint8_t state);
void PWM_Set_PreCharge_Relay(uint8_t state);

/* 3. Interface de Controlo do AFE (Ponte H de Potência) */
void PWM_AFE_Control_Update(uint8_t user_enable, uint8_t enable_control);
void PWM_AFE_Process_Background_Math(float v_pg, float v_dc_link);  // PLL e Filtros
void PWM_AFE_Run_PI_and_Hysteresis(float i_pg); 					// Algoritmo principal de controlo
void PWM_AFE_PWM_Kill(void); 										// Corte de segurança para o AFE

/* 4. Interface de Controlo do Inversor (Motor Trifásico) */
void PWM_Inverter_Update(uint8_t enable_inverter);  // Aplica a lógica V/f
void PWM_Inverter_Set_Freq(float new_freq); 		// Define a frequência do motor
void PWM_Inverter_Set_Ramp(float new_ramp); 		// Define a velocidade da rampa (Hz/s)

/* Getters de Estado do Hardware (Duty Cycles) */
uint32_t PWM_Get_Duty_A(void);
uint32_t PWM_Get_Duty_B(void);
uint32_t PWM_Get_Duty_C(void);
uint32_t PWM_AFE_Get_Duty_CH2(void);
uint32_t PWM_AFE_Get_Duty_CH3(void);
float PWM_AFE_Get_Preg(void); 			// Getter: Potência de referência
float PWM_AFE_Get_I_Ref(void); 			// Getter: Corrente de referência

/* 5. Interface de Gestão Térmica */
void PWM_Update_Cooling_Fan(float current_temp); // Controlo PWM da ventoinha

#endif /* INC_PWM_H_ */
