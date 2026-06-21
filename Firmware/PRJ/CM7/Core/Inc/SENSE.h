/*
 * Sense.h
 *
 * Created on: Apr 25, 2026
 * Author: Geral
 */

#ifndef INC_SENSE_H_
#define INC_SENSE_H_

#include "stm32h7xx_hal.h"

/* =========================================================
   Constantes de Calibração Física e Ganhos
   ========================================================= */
#define V_MAX_SENSE            400.0f
#define ADC_RESOLUTION         65535.0f  // 16-bit
#define VREF                   3.3f

#define VOLTAGE_DIVIDER_RATIO  100.0f
#define AMC1301_GAIN           8.2f
#define CURRENT_SENSITIVITY    0.066f

/* =========================================================
   Interface Pública do Módulo de Sensores
   ========================================================= */

/* Funções Core */
void Sense_Init_DMA(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3); // Inicializa a aquisição
void Sense_Process_Data(void); // Converte dados em bruto para grandezas físicas (Chamada a 25kHz)

/* Getters: Grandezas DC e RMS (Filtradas) */
float Sense_Get_V_DC_Link(void);     // Getter: Tensão do Barramento DC (Volts)
float Sense_Get_V_Grid(void);        // Getter: Tensão RMS da Rede (Volts)
float Sense_Get_I_Grid(void);        // Getter: Corrente RMS da Rede (Amperes)
float Sense_Get_Heatsink_Temp(void); // Getter: Temperatura do Dissipador (Celsius)

/* Getters: Correntes das Fases do Motor (Instantâneas) */
float Sense_Get_I_Phase_A(void);     // Getter: Corrente Fase A (Amperes)
float Sense_Get_I_Phase_B(void);     // Getter: Corrente Fase B (Amperes)
float Sense_Get_I_Phase_C(void);     // Getter: Corrente Fase C (Amperes)

/* Getters: Sinais Instantâneos para Controlo e PLL */
float Sense_Get_V_Grid_Inst(void);   // Getter: Tensão instantânea da rede (Senoide pura)
float Sense_Get_I_Grid_Inst(void);   // Getter: Corrente instantânea da rede

#endif /* INC_SENSE_H_ */
