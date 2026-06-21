/*
 * DD_ADC.h
 *
 * Created on: May 24, 2026
 * Author: Geral
 */

#ifndef INC_DD_ADC_H_
#define INC_DD_ADC_H_

#include "stm32h7xx_hal.h"

/* =========================================================
   Estruturas de Dados (Vetores Matemáticos)
   ========================================================= */

// Vetor "IN" (Lado da Rede)
typedef struct {
    float v_grid;
    float i_grid;
} Input_Vector_t;

// Vetor "OUT" (Lado do Motor)
typedef struct {
    float i_a;
    float i_b;
    float i_c;
} Output_Vector_t;

// Vetor "SYSTEM" (Gestão Interna)
typedef struct {
    float v_dc_link;
    float temp_heatsink;
} System_Vector_t;

/* =========================================================
   Interface Pública de Hardware
   ========================================================= */

/* Inicialização e Processamento */
void ADC_Init_DMA(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3); // Configura e arranca o DMA
void ADC_Process_Data(void); // Lê os buffers em bruto e preenche os vetores

/* Getters dos Vetores (Acesso de Leitura Seguro) */
Input_Vector_t ADC_Get_Input_Vector(void);   // Getter: Variáveis da Rede
Output_Vector_t ADC_Get_Output_Vector(void); // Getter: Correntes do Motor
System_Vector_t ADC_Get_System_Vector(void); // Getter: Tensão DC e Temperatura

#endif /* INC_DD_ADC_H_ */
