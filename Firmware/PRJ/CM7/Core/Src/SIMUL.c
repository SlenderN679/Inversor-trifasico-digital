/*
 * SIMUL.c
 *
 * Módulo de Hardware-In-The-Loop (Digital Twin).
 * Gera grandezas físicas falsas via DAC ou Matemática Interna
 * para permitir o teste do controlador sem estar ligado a 400V.
 */
#include "tim.h"
#include "dac.h"
#include "SIMUL.h"
#include <math.h>

#define SAMPLE_RATE_HZ      25000.0f
#define GRID_FREQ_HZ        50.0f
#define PI_VAL              3.14159265f
#define THETA_INCREMENT     ((2.0f * PI_VAL * GRID_FREQ_HZ) / SAMPLE_RATE_HZ)

static float fake_theta = 0.0f;
static float sim_vdc_dac_val = 0.0f; // Valor Interno Simulação (0 a 4095)

extern volatile uint8_t afe_control_stage;
extern volatile uint8_t afe_control_enable;
extern float PWM_AFE_Get_Preg(void);
extern volatile uint8_t inverter_enable;
extern float modulation_index;
extern float phase_accumulator;

void DacSim_Init(DAC_HandleTypeDef* hdac)
{
    HAL_DAC_Start(hdac, DAC_CHANNEL_1); // Tensão da Rede
    HAL_DAC_Start(hdac, DAC_CHANNEL_2); // Tensão DC Link

    // Configuração de descanso inicial (1.65V e 0V)
    HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    HAL_DAC_SetValue(hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
}

void DacSim_Update_Grid_DC(DAC_HandleTypeDef* hdac)
{
    /* --- 1. Dinâmica da Rede Virtual (Canal 1 a 30VAC) --- */
    fake_theta += THETA_INCREMENT;
    if (fake_theta > 6.283185f) fake_theta -= 6.283185f;

    // A rede recua sob esforço. A 30VAC (42.4V Pico)
    float corrente_puxada = Simul_Get_I_Grid();
    float queda_de_tensao = fabsf(corrente_puxada) * 1.5f;

    // O pico de 30VAC é ~42.4V. Convertido para a resolução do DAC (0-3.3V -> 0-4095)
    // Assumindo que a vossa placa lê 41V como ~0.4V no ADC (via divisor de tensão)
    // O valor base de amplitude DAC tem de ser baixo (ex: 200 pontos ADC)
    float grid_amplitude = 200.0f - queda_de_tensao;
    if (grid_amplitude < 50.0f) grid_amplitude = 50.0f; // Proteção contra colapso virtual

    uint32_t grid_dac_out = (uint32_t)(2048.0f + (grid_amplitude * sinf(fake_theta)));
    HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, grid_dac_out);


    /* --- 2. Dinâmica do Condensador Virtual (Canal 2 a 41V DC) --- */
    // Com 30VAC, o barramento estabiliza nos ~41V DC.
    // O limite do DAC correspondente a 41V no vosso sistema de medida (ex: 41V lidos no divisor)
    // vamos mapear virtualmente 41V para um valor DAC de ~400.
    const float MAX_VDC_DAC = 400.0f;

    if (afe_control_stage == 0) {
        // Descarga Passiva (Mais rápida a baixa tensão)
        if (sim_vdc_dac_val > 0.0f) sim_vdc_dac_val -= 0.5f;
    }
    else if (afe_control_stage == 1) {
        // Pré-Carga Lenta até aos 41V (Max VDC DAC)
        if (sim_vdc_dac_val < MAX_VDC_DAC) sim_vdc_dac_val += 0.06f;
    }
    else if (afe_control_stage == 2) {
        if (afe_control_enable) {
            /* Equação Física: dV/dt = C * (P_injetada - P_consumida) */
            float p_load = 0.0f;
            // Potência consumida a 41V
            if (inverter_enable) p_load = 20.0f * modulation_index;

            float p_afe = PWM_AFE_Get_Preg();
            float constante_capacidade = 0.005f; // Responde mais rápido a baixas potências

            sim_vdc_dac_val += (p_afe - p_load) * constante_capacidade;

            // Limites do Conversor
            if (sim_vdc_dac_val > 4095.0f) sim_vdc_dac_val = 4095.0f;
            if (sim_vdc_dac_val < 0.0f) sim_vdc_dac_val = 0.0f;
        } else {
            // Fugas naturais
            if (sim_vdc_dac_val > MAX_VDC_DAC) sim_vdc_dac_val -= 0.1f;
        }
    }
    HAL_DAC_SetValue(hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)sim_vdc_dac_val);
}

/* --- Simulação do Motor (Fator de Potência Virtual) --- */
static float sim_i_a = 0.0f, sim_i_b = 0.0f, sim_i_c = 0.0f;

void Simul_Update_Motor_Currents(void) {
    if (inverter_enable) {
        float lag = 0.52f; // Carga indutiva causa atraso da corrente (~30º)
        float i_amplitude = 8.0f * modulation_index;

        sim_i_a = i_amplitude * sinf(phase_accumulator - lag);
        sim_i_b = i_amplitude * sinf(phase_accumulator - 2.094395f - lag);
        sim_i_c = i_amplitude * sinf(phase_accumulator + 2.094395f - lag);
    } else {
        sim_i_a = 0.0f; sim_i_b = 0.0f; sim_i_c = 0.0f;
    }
}

float Simul_Get_I_A(void) { return sim_i_a; }
float Simul_Get_I_B(void) { return sim_i_b; }
float Simul_Get_I_C(void) { return sim_i_c; }

/* --- Injeção de Reativos na Rede --- */
extern float PWM_AFE_Get_I_Ref(void);
float sim_qreg = 0.0f;

float Simul_Get_I_Grid(void){
    float corrente_ativa = PWM_AFE_Get_I_Ref();
    float corrente_reativa = sim_qreg * cosf(fake_theta); // Atrasada em 90º da tensão
    return corrente_ativa + corrente_reativa;
}
