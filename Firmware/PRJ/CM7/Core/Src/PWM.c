/*
 * PWM.c
 *
 * Módulo de Eletrónica de Potência.
 * Contém os algoritmos de Controlo AFE (PI + Histerese) e do
 * Inversor Trifásico (Curva V/f constante e Modulação SPWM).
 */
#include <SENSE.h>
#include "tim.h"
#include "PWM.h"
#include "gpio.h"

/* =========================================================
   Variáveis Globais (Inversor SPWM)
   ========================================================= */
float pwm_freq = 25000.0f;
float modulation_index = 0.8f;
float target_frequency = 50.0f;
float phase_accumulator = 0.0f;
float sampling_time = 0.0f;
float sine_lut[1024];

void Generate_Sine_LUT(void) {
    for (int i = 0; i < 1024; i++) {
        ((float*)sine_lut)[i] = sinf((2.0f * PI * i) / 1024.0f);
    }
}

/* =========================================================
   Variáveis de Estado do AFE
   ========================================================= */
#define AFE_TS           (1.0f / 25000.0f)
#define PI_VAL           3.14159265f
#define PI2_VAL          (2.0f * PI_VAL)
#define VDC_REF          400.0f

/* Malha PLL (Phase-Locked Loop) */
static float pll_uni = 0.0f, pll_pll = 0.0f;
static float _sin = 0.0f, _cos = 0.0f;
static float sum_err_a = 0.0f, sum_err_p = 0.0f;
static float intg_a = 0.0f, intg_p = 0.0f;
static float w0 = 314.159265f; // Frequência Base (50Hz = 314 rad/s)
static float ki_a = 100.0f * AFE_TS;
static float ki_p = 10.0f * AFE_TS;
static float kp_p = 1.0f;

/* Filtro Média Móvel DC (Janela 10ms = 250 amostras) */
#define VDC_ARRAY_SIZE 250
static float v_cc_Array[VDC_ARRAY_SIZE] = {0};
static float sum_vcc = 0.0f;
static float Vcc_medio = 0.0f;
static int v_cc_array_index = 0;
static int flag_vcc_array_full = 0;

/* Malha de Controlo PI (Tensão -> Corrente Ref) */
static float Vcc_sum_e = 0.0f;
static float preg = 0.0f;
static float i_ref = 0.0f;
static float kp_Vcc = 10.0f;
static float ki_Vcc = 0.01f;

/* Função Utilitária para Limites Matemáticos */
static inline float limiter(float variavel, float max_limit, float min_limit) {
    if (variavel > max_limit) return max_limit;
    if (variavel < min_limit) return min_limit;
    return variavel;
}

/* =========================================================
   Controlo Eletromecânico (Relés de Corte e Pre-Charge)
   ========================================================= */

/* * Controla o contactor principal (Relé de Corte).
 * state = 1 (Fecha o circuito, liga à rede)
 * state = 0 (Abre o circuito, isola a rede)
 */
void PWM_Set_Main_Contactor(uint8_t state)
{
    if (state) {
        HAL_GPIO_WritePin(CHOPPER_GPIO_Port, CHOPPER_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(CHOPPER_GPIO_Port, CHOPPER_Pin, GPIO_PIN_RESET);
    }
}

/* * Controla o relé de bypass da resistência de pré-carga (Soft-Start).
 * state = 1 (Faz o bypass à resistência - Modo normal de operação)
 * state = 0 (Corrente passa pela resistência - Modo de pré-carga)
 */
void PWM_Set_PreCharge_Relay(uint8_t state)
{
    if (state) {
        HAL_GPIO_WritePin(SoftStart_GPIO_Port, SoftStart_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(SoftStart_GPIO_Port, SoftStart_Pin, GPIO_PIN_RESET);
    }
}

/* * Lógica Automática de Pré-carga.
 * Monitoriza o V_DC_Link e fecha o relé de soft-start quando seguro.
 */
void PWM_Handle_SoftStart_Sequence(int vdc)
{
	// Com AC a 30V RMS, o pico DC retificado é ~42.4V.
	// Limite de segurança para fechar o relé ajustado para 35V.
	if (vdc > 35.0f) {
	    PWM_Set_PreCharge_Relay(1); // Fecha relé, bypass da resistência (Soft-Start concluído)
	}
	// Histerese de segurança: Se a tensão cair para valores anormais, abre o bypass
	else if (vdc < 15.0f) {
	    PWM_Set_PreCharge_Relay(0); // Reativa a proteção da resistência
	}
}

/* =========================================================
   Algoritmos do Retificador Ativo (AFE)
   ========================================================= */
static void PWM_AFE_Update_PLL(float v_grid) {
    float err = v_grid - pll_pll;
    float err_a = err * _sin;
    float err_p = err * _cos;

    sum_err_a += err_a;
    sum_err_p += err_p;

    float pi_p = (kp_p * err_p) + (ki_p * sum_err_p) + w0;
    intg_p += pi_p * AFE_TS;

    // Normalizar fase entre -PI e +PI
    if (intg_p > PI_VAL)  intg_p -= PI2_VAL;
    if (intg_p < -PI_VAL) intg_p += PI2_VAL;

    _sin = sinf(intg_p);
    _cos = cosf(intg_p);

    intg_a = sum_err_a * ki_a;
    if (intg_a == 0.0f) intg_a = 0.000001f; // Evitar zero absoluto

    pll_pll = _sin * intg_a;
    pll_uni = _sin; // Onda unitária para sincronismo
}

static void PWM_AFE_Update_Vdc_Avg(float v_dc_link) {
    if (v_cc_array_index >= VDC_ARRAY_SIZE) {
        v_cc_array_index = 0;
        flag_vcc_array_full = 1;
    }
    if (flag_vcc_array_full) {
        sum_vcc = sum_vcc + v_dc_link - v_cc_Array[v_cc_array_index];
    } else {
        sum_vcc = sum_vcc + v_dc_link;
    }
    v_cc_Array[v_cc_array_index] = v_dc_link;
    v_cc_array_index++;
    Vcc_medio = sum_vcc / (float)VDC_ARRAY_SIZE;
}

// Tarefa de Fundo Constante (25kHz)
void PWM_AFE_Process_Background_Math(float v_pg, float v_dc_link) {
    PWM_AFE_Update_PLL(v_pg);
    PWM_AFE_Update_Vdc_Avg(v_dc_link);
}

// Tarefa de Controlo Ativo (Apenas no estado RUNNING)
void PWM_AFE_Run_PI_and_Hysteresis(float i_pg) {
    /* 1. Cálculo PI da Potência Exigida */
    float Vcc_erro = VDC_REF - Vcc_medio;
    Vcc_sum_e += Vcc_erro;
    Vcc_sum_e = limiter(Vcc_sum_e, 200.0f, -200.0f);

    float Vcc_out_control = (Vcc_erro * kp_Vcc) + (Vcc_sum_e * ki_Vcc);
    preg = limiter(Vcc_out_control, 2000.0f, -2000.0f);

    /* 2. Referência de Corrente (Sincronizada c/ a Rede) */
    i_ref = (2.0f * preg * pll_uni) / intg_a;
    uint32_t arr = htim8.Instance->ARR;

    /* 3. Controlo Bang-Bang (Histerese com 95% limite de segurança) */
    if (i_ref < i_pg) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)(arr * 0.95f));
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)(arr * 0.05f));
    } else {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)(arr * 0.05f));
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)(arr * 0.95f));
    }
}

void PWM_AFE_PWM_Kill(void) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    preg = 0.0f;
    i_ref = 0.0f;
}

float PWM_AFE_Get_Preg(void)  { return preg; }
float PWM_AFE_Get_I_Ref(void) { return i_ref; }

/* =========================================================
   Inversor do Motor (Curva V/f e SPWM)
   ========================================================= */
static float current_frequency = 0.0f;
static float v_f_ratio = 1.0f / 50.0f; // Frequência nominal = 50Hz = Modulação 100%
static float freq_ramp_hz_s = 25.0f;

static uint32_t duty_a_out = 0;
static uint32_t duty_b_out = 0;
static uint32_t duty_c_out = 0;

void PWM_Inverter_Set_Ramp(float new_ramp) {
    if (new_ramp >= 1.0f && new_ramp <= 100.0f) freq_ramp_hz_s = new_ramp;
}

void PWM_Inverter_Set_Freq(float new_freq) {
    if (new_freq >= 0.0f && new_freq <= 60.0f) target_frequency = new_freq;
}

void PWM_Inverter_Update(uint8_t enable_inverter)
{
    if (!enable_inverter) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        current_frequency = 0.0f;
        return;
    }

    /* 1. Rampa de Aceleração (Soft-Start) */
    float step = freq_ramp_hz_s * sampling_time;
    if (current_frequency < target_frequency) {
        current_frequency += step;
        if (current_frequency > target_frequency)
        	current_frequency = target_frequency;
    } else if (current_frequency > target_frequency) {
        current_frequency -= step;
        if (current_frequency < target_frequency)
        	current_frequency = target_frequency;
    }

    /* 2. Curva Escalar V/f (Controlo de Tensão) */
    modulation_index = current_frequency * v_f_ratio;
    if (modulation_index > 0.95f) modulation_index = 0.95f; // Limite segurança Dead-Time

    /* 3. Acumulador de Fase (Integração Continua) */
    phase_accumulator += PI2_VAL * current_frequency * sampling_time;
    if (phase_accumulator > PI2_VAL) phase_accumulator -= PI2_VAL;

    /* 4. Sistema Trifásico (120º Desfasamento = 2.0944 rad) */
    float theta_a = phase_accumulator;
    float theta_b = phase_accumulator - 2.094395f;
    float theta_c = phase_accumulator + 2.094395f;

    /* 5. Geração e Offset do SPWM */
    uint32_t arr = htim1.Instance->ARR;
    float half_arr = (float)arr / 2.0f;

    duty_a_out = (uint32_t)(half_arr + (half_arr * modulation_index * sinf(theta_a)));
    duty_b_out = (uint32_t)(half_arr + (half_arr * modulation_index * sinf(theta_b)));
    duty_c_out = (uint32_t)(half_arr + (half_arr * modulation_index * sinf(theta_c)));

    /* 6. Atuação em Hardware */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_a_out);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_b_out);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_c_out);
}

uint32_t PWM_Get_Duty_A(void) { return duty_a_out; }
uint32_t PWM_Get_Duty_B(void) { return duty_b_out; }
uint32_t PWM_Get_Duty_C(void) { return duty_c_out; }
uint32_t PWM_AFE_Get_Duty_CH2(void) { return __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_2); }
uint32_t PWM_AFE_Get_Duty_CH3(void) { return __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_3); }

/* =========================================================
   Controlo de Gestão Térmica (Ventoinha)
   ========================================================= */
#define TEMP_FAN_OFF  35.0f
#define TEMP_FAN_MAX  60.0f
static uint32_t fan_duty_pct = 0;

void PWM_Update_Cooling_Fan(float current_temp)
{
    if (current_temp <= TEMP_FAN_OFF) fan_duty_pct = 0;
    else if (current_temp >= TEMP_FAN_MAX) fan_duty_pct = 100;
    else {
        // Interpolação Linear Proporcional
        float temperature_range = TEMP_FAN_MAX - TEMP_FAN_OFF;
        float current_excess = current_temp - TEMP_FAN_OFF;
        fan_duty_pct = (uint32_t)((current_excess / temperature_range) * 100.0f);
    }

    uint32_t arr = htim3.Instance->ARR;
    if (arr == 0) arr = 1;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (fan_duty_pct * arr) / 100);
}

/* =========================================================
   Arranque e Proteção Geral do Sistema
   ========================================================= */
void PWM_Start_All(void)
{
    sampling_time = 1.0f / (4 * pwm_freq); // Timer em modo Center-Aligned (x4)
    htim1.State = HAL_TIM_STATE_READY;

    /* 1. Iniciar TIM8 (Retificador AFE) com canais Invertidos */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);

    /* 2. Iniciar TIM1 (Inversor do Motor) com canais Invertidos */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim1); // Ativa Interrupção mestre de 25kHz

    /* Iniciar aos 50% (Tensão Zero) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim1)/2.0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim1)/2.0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, __HAL_TIM_GET_AUTORELOAD(&htim1)/2.0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim8)/2.0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, __HAL_TIM_GET_AUTORELOAD(&htim8)/2.0);

    /* Ativar Saídas Físicas Principais e Interrupções de Segurança */
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);

    /* 3. Start TIM3 (Ventoinha de Arrefecimento) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

    Generate_Sine_LUT();
}

void PWM_Emergency_Stop(void)
{
    /* 1. Corte de Transístores Físico (Nanosegundos) */
    __HAL_TIM_MOE_DISABLE(&htim1);
    __HAL_TIM_MOE_DISABLE(&htim8);

    /* 2. Reforço Lógico (Software) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

    /* 3. Corte Eletromecânico de Segurança */
    PWM_Set_Main_Contactor(0);  // Abre o contactor principal imediatamente
    PWM_Set_PreCharge_Relay(0); // Abre o bypass do soft-start
}

/* Dependências para o Desarme (Interlock) */
extern volatile uint8_t afe_control_stage;
extern volatile uint8_t afe_control_enable;
extern volatile uint8_t afe_user_enable;
extern volatile uint8_t inverter_enable;
extern int system_fault;
extern int fault_source;

void Check_Safety(void)
{
    float vdc_atual  = Sense_Get_V_DC_Link();
    float ia_atual   = Sense_Get_I_Phase_A();
    float ib_atual   = Sense_Get_I_Phase_B();
    float ic_atual   = Sense_Get_I_Phase_C();
    float temp_atual = Sense_Get_Heatsink_Temp();

    /* 1. Falha por Sobretensão DC */
    if (vdc_atual > 450.0f) {
        PWM_Emergency_Stop();
        PWM_AFE_PWM_Kill();
        system_fault = 1; fault_source = 1;
        afe_user_enable = 0; afe_control_enable = 0;
        afe_control_stage = 0; inverter_enable = 0;
    }

    /* 2. Falha por Sobrecorrente AC (Picos no Motor) */
    if (ia_atual > 15.0f || ia_atual < -15.0f ||
        ib_atual > 15.0f || ib_atual < -15.0f ||
        ic_atual > 15.0f || ic_atual < -15.0f) {
        PWM_Emergency_Stop();
        PWM_AFE_PWM_Kill();
        system_fault = 1; fault_source = 2;
        afe_user_enable = 0; afe_control_enable = 0;
        afe_control_stage = 0; inverter_enable = 0;
    }

    /* 3. Falha Térmica (Excesso Calor no Dissipador) */
    if (temp_atual > 75.0f) {
        PWM_Emergency_Stop();
        PWM_AFE_PWM_Kill();
        system_fault = 1; fault_source = 4;
        afe_user_enable = 0; afe_control_enable = 0;
        afe_control_stage = 0; inverter_enable = 0;
    }
}
