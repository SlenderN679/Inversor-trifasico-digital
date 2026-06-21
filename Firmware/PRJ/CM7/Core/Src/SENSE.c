/*
 * Sense.c
 *
 * Módulo responsável por traduzir dados brutos (RAW) dos conversores
 * em grandezas físicas reais (Volts, Amperes, Celsius), integrando
 * também os ganchos (hooks) para o simulador HIL.
 */
#include "DD_ADC.h"
#include "SIMUL.h"
#include <math.h>
#include <SENSE.h>

/* =========================================================
   Constantes de Calibração de Hardware
   ========================================================= */
#define VREF                   3.3f
#define ADC_RESOLUTION         65535.0f

/* Limites físicos do hardware quando o pino atinge 3.3V */
#define PEAK_V_DC_LINK         193.0f    // Volts
#define PEAK_V_GRID_AC         91.0f     // Volts (Pico AC)
#define PEAK_I_GRID_AC         5.0f     // Amperes (Pico AC)
#define PEAK_I_MOTOR_AC        5.0f     // Amperes (Pico AC)
//#define PEAK_TEMP_C            100.0f    // Celsius

#define V_OFFSET_AC            (VREF / 2.0f) // Ponto central para sinais bipolares (1.65V)

/* =========================================================
   Constantes do Termistor (NTCALUG02A103G)
   ========================================================= */
#define NTC_R_NOMINAL          10000.0f  // Resistência nominal a 25°C (10k ohms)
#define NTC_T_NOMINAL_K        298.15f   // Temperatura nominal em Kelvin (25°C + 273.15)
#define NTC_BETA               3984.0f   // Parâmetro Beta do NTC
#define NTC_R_PULLUP           10000.0f  // Valor do resistor superior (Rtemp) no divisor de tensão

/* =========================================================
   Variáveis Privadas
   ========================================================= */
static volatile float v_dc_link = 0.0f;
static volatile float v_grid = 0.0f;
static volatile float i_grid = 0.0f;
static volatile float i_phase_a = 0.0f;
static volatile float i_phase_b = 0.0f;
static volatile float i_phase_c = 0.0f;
static volatile float heatsink_temp = 0.0f;

/* Variáveis para Cálculo True RMS (Janela de 20ms = 500 amostras @ 25kHz) */
static volatile float v_grid_rms = 0.0f;
static volatile float i_grid_rms = 0.0f;
static float sum_vgrid_sq = 0.0f;
static float sum_igrid_sq = 0.0f;
static int rms_counter = 0;

/* =========================================================
   Funções Auxiliares de Conversão Físicas (Inlined)
   ========================================================= */
static inline float Convert_To_Volts_Single(uint16_t adc_raw) {
    float v_pin = ((float)adc_raw / ADC_RESOLUTION) * VREF;
    return v_pin * (PEAK_V_DC_LINK / VREF);
}

static inline float Convert_To_Amps(uint16_t adc_raw) {
    float v_pin = ((float)adc_raw / ADC_RESOLUTION) * VREF;
    return (v_pin - V_OFFSET_AC) * (PEAK_I_MOTOR_AC / V_OFFSET_AC);
}

static inline float Convert_To_Volts_AC(uint16_t adc_raw) {
    float v_pin = ((float)adc_raw / ADC_RESOLUTION) * VREF;
    return (v_pin - V_OFFSET_AC) * (PEAK_V_GRID_AC / V_OFFSET_AC);
}

static inline float Convert_To_Amps_AC(uint16_t adc_raw) {
    float v_pin = ((float)adc_raw / ADC_RESOLUTION) * VREF;
    return (v_pin - V_OFFSET_AC) * (PEAK_I_GRID_AC / V_OFFSET_AC);
}

static inline float Convert_To_Temp(uint16_t adc_raw) {
    // Proteções contra divisão por zero nos extremos do ADC
    if (adc_raw == 0) return -55.0f; // Temperatura mínima teórica do sensor
    if (adc_raw >= (uint16_t)ADC_RESOLUTION) return 125.0f; // Temperatura máxima teórica

    // 1. Calcula a resistência atual do NTC baseado no divisor de tensão
    // Formula: R_NTC = R_PULLUP * (ADC_RAW / (ADC_MAX - ADC_RAW))
    float r_ntc = NTC_R_PULLUP * ((float)adc_raw / (ADC_RESOLUTION - (float)adc_raw));

    // 2. Aplica a Equação Beta (Steinhart-Hart simplificada) para obter a temperatura em Kelvin
    // Formula: 1/T = 1/T0 + (1/Beta) * ln(R/R0)
    float temp_k = r_ntc / NTC_R_NOMINAL;        // (R/R0)
    temp_k = logf(temp_k);                       // ln(R/R0)
    temp_k /= NTC_BETA;                          // (1/Beta) * ln(R/R0)
    temp_k += (1.0f / NTC_T_NOMINAL_K);          // + (1/T0)
    temp_k = 1.0f / temp_k;                      // Inverte para obter T (Kelvin)

    // 3. Converte Kelvin para Celsius
    return temp_k - 273.15f;
}

/* =========================================================
   Core de Processamento (Chamado a 25 kHz)
   ========================================================= */
void Sense_Init_DMA(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3)
{
    ADC_Init_DMA(hadc1, hadc3);
    // Reinicia todas as variáveis globais
    v_dc_link = 0.0f; v_grid = 0.0f; i_grid = 0.0f;
    i_phase_a = 0.0f; i_phase_b = 0.0f; i_phase_c = 0.0f;
    heatsink_temp = 0.0f;
}

void Sense_Process_Data(void)
{
    /* 1. Atualizar o Device Driver */
    ADC_Process_Data();

    /* 2. Recuperar vetores brutos */
    Input_Vector_t  raw_in  = ADC_Get_Input_Vector();
    Output_Vector_t raw_out = ADC_Get_Output_Vector();
    System_Vector_t raw_sys = ADC_Get_System_Vector();

    /* 3. Injeção de Dados Simulados (Digital Twin / HIL Override) */
    i_phase_a = Simul_Get_I_A();
    i_phase_b = Simul_Get_I_B();
    i_phase_c = Simul_Get_I_C();
    i_grid    = Simul_Get_I_Grid();

    /* 4. Conversão Física de Sensores Reais */
    v_grid        = Convert_To_Volts_AC((uint16_t)raw_in.v_grid);
    v_dc_link     = Convert_To_Volts_Single((uint16_t)raw_sys.v_dc_link);
    heatsink_temp = Convert_To_Temp((uint16_t)raw_sys.temp_heatsink);

    /* 5. Cálculo True RMS Contínuo */
    sum_vgrid_sq += (v_grid * v_grid);
    sum_igrid_sq += (i_grid * i_grid);
    rms_counter++;

    if (rms_counter >= 500) { // Dispara a cada 20ms exatos (1 período a 50Hz)
        v_grid_rms = sqrtf(sum_vgrid_sq / 500.0f);
        i_grid_rms = sqrtf(sum_igrid_sq / 500.0f);
        sum_vgrid_sq = 0.0f;
        sum_igrid_sq = 0.0f;
        rms_counter  = 0;
    }
}

/* =========================================================
   Funções Getter Públicas
   ========================================================= */
float Sense_Get_V_DC_Link(void)     { return v_dc_link; }
float Sense_Get_V_Grid(void)        { return v_grid_rms; }
float Sense_Get_I_Grid(void)        { return i_grid_rms; }
float Sense_Get_Heatsink_Temp(void) { return heatsink_temp; }
float Sense_Get_I_Phase_A(void)     { return i_phase_a; }
float Sense_Get_I_Phase_B(void)     { return i_phase_b; }
float Sense_Get_I_Phase_C(void)     { return i_phase_c; }

/* Sinais Instantâneos para as Malhas de Controlo (PLL) */
float Sense_Get_V_Grid_Inst(void)   { return v_grid; }
float Sense_Get_I_Grid_Inst(void)   { return i_grid; }
