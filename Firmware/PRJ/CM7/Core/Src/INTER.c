/*
 * INTER.c
 *
 * Módulo de Interface e Comunicação (SCADA).
 * Compila todas as informações do sistema em strings JSON compactas
 * e envia-as via porta Série/UART para consumo do Node-RED.
 */
#include "PANEL.h"
#include "DD_ADC.h"
#include "PWM.h"
#include <stdio.h>
#include <SENSE.h>
#include "st7789.h"
#include "gpio.h"
#include "usart.h"
#include "tim.h"

char tx_buffer[512] __attribute__((section(".sram1"), aligned(32))); // Obrigatório SRAM1 para o DMA!

extern volatile uint8_t afe_user_enable;
extern volatile uint8_t afe_control_enable;
extern volatile uint8_t afe_control_stage;

/* --- Construtores de Estrutura JSON --- */
int Telemetry_Build_JSON(char *buffer, int max_len) {
    return snprintf(buffer, max_len,
           "\"vdc\":%.1f, \"vgrid\":%.1f, \"igrid\":%.2f, \"temp\":%.1f, \"ia\":%.2f, \"ib\":%.2f, \"ic\":%.2f, ",
           Sense_Get_V_DC_Link(), Sense_Get_V_Grid(), Sense_Get_I_Grid(), Sense_Get_Heatsink_Temp(),
           Sense_Get_I_Phase_A(), Sense_Get_I_Phase_B(), Sense_Get_I_Phase_C());
}
//---
int Raw_Telemetry_Build_JSON(char *buffer, int max_len) {
    Input_Vector_t  raw_in  = ADC_Get_Input_Vector();
    Output_Vector_t raw_out = ADC_Get_Output_Vector();
    System_Vector_t raw_sys = ADC_Get_System_Vector();

    return snprintf(buffer, max_len,
           "\"r_ia\":%d, \"r_ib\":%d, \"r_ic\":%d, \"r_vgrid\":%d, \"r_igrid\":%d, \"r_vdc\":%d, \"r_temp\":%d, ",
           (int)raw_out.i_a, (int)raw_out.i_b, (int)raw_out.i_c,
           (int)raw_in.v_grid, (int)raw_in.i_grid,
           (int)raw_sys.v_dc_link, (int)raw_sys.temp_heatsink);
}
//---
int AFE_Telemetry_Build_JSON(char *buffer, int max_len) {
    uint32_t arr = htim8.Instance->ARR;
    if (arr == 0) arr = 1;

    int afe_d2_pct = (PWM_AFE_Get_Duty_CH2() * 100) / arr;
    int afe_d3_pct = (PWM_AFE_Get_Duty_CH3() * 100) / arr;

    return snprintf(buffer, max_len,
           "\"c_iref\":%.2f, \"c_preg\":%.1f, \"c_state\":%d, \"c_en\":%d, \"afe_d2\":%d, \"afe_d3\":%d, ",
           PWM_AFE_Get_I_Ref(), PWM_AFE_Get_Preg(), afe_control_stage, afe_control_enable, afe_d2_pct, afe_d3_pct);
}
//---
int Inverter_Telemetry_Build_JSON(char *buffer, int max_len) {
    uint32_t arr = htim1.Instance->ARR;
    if (arr == 0) arr = 1;

    return snprintf(buffer, max_len,
           "\"inv_da\":%d, \"inv_db\":%d, \"inv_dc\":%d",
           (PWM_Get_Duty_A() * 100) / arr, (PWM_Get_Duty_B() * 100) / arr, (PWM_Get_Duty_C() * 100) / arr);
}

/* =========================================================
   Montagem Final e Transmissão
   ========================================================= */
void Update_Interface(UART_HandleTypeDef *huart)
{
    int offset = 0;
    int max_len = sizeof(tx_buffer);

    /* 1. Abre Objeto JSON */
    offset += snprintf(tx_buffer + offset, max_len - offset, "{");

    /* 2. Anexa todos os blocos modulares */
    offset += Telemetry_Build_JSON(tx_buffer + offset, max_len - offset);
    offset += Raw_Telemetry_Build_JSON(tx_buffer + offset, max_len - offset);
    offset += AFE_Telemetry_Build_JSON(tx_buffer + offset, max_len - offset);
    offset += Inverter_Telemetry_Build_JSON(tx_buffer + offset, max_len - offset);

    /* 3. Fecha Objeto e adiciona escape final */
    offset += snprintf(tx_buffer + offset, max_len - offset, "}\r\n");

    /* 4. Envio Assíncrono via DMA */
    if (offset > 0) {
        SCB_CleanDCache_by_Addr((uint32_t*)tx_buffer, sizeof(tx_buffer)); // Imperativo no STM32H7!
        HAL_UART_Transmit_DMA(huart, (uint8_t*)tx_buffer, offset);
    }
}
