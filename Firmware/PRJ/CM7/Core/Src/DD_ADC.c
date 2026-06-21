/*
 * DD_ADC.c
 *
 * Abstração de Hardware (Device Driver) para os Conversores Analógico-Digitais.
 * Lida com o DMA e o mapeamento de memória específico do STM32H7.
 */
#include "DD_ADC.h"

/* =========================================================
   Constantes Internas de Calibração
   ========================================================= */
#define VREF               3.3f
#define ADC_RESOLUTION     65535.0f

#define ACS_SENSITIVITY    0.066f
#define ACS_OFFSET         1.65f

#define AMC_GAIN           8.2f
#define AMC_SE_OFFSET      1.44f
#define VOLTAGE_DIVIDER    100.0f

/* =========================================================
   Variáveis Privadas (Protegidas por 'static')
   ========================================================= */
static volatile Input_Vector_t  in_vector;
static volatile Output_Vector_t out_vector;
static volatile System_Vector_t sys_vector;

/* O ADC1 (DMA Normal) tem de ser mapeado para o domínio D2 RAM (sram1 ou sram2) */
static volatile uint16_t adc1_buffer[3] __attribute__((section(".sram1")));

/* O ADC3 (BDMA) tem de ser mapeado para o domínio D3 RAM (sram4) */
static volatile uint16_t adc3_buffer[4] __attribute__((section(".sram4")));

/* =========================================================
   Inicialização e Controlo
   ========================================================= */
void ADC_Init_DMA(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3)
{
    /* 1. Limpar flags residuais de erros antes do arranque seguro */
    __HAL_ADC_CLEAR_FLAG(hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS);
    __HAL_ADC_CLEAR_FLAG(hadc3, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS);

    /* 2. Calibração de Hardware (Obrigatório com o ADC ainda desligado) */
    HAL_ADCEx_Calibration_Start(hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(hadc3, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    /* 3. Ligar os motores de DMA contínuo apenas após a calibração */
    HAL_ADC_Start_DMA(hadc1, (uint32_t*)adc1_buffer, 3);
    HAL_ADC_Start_DMA(hadc3, (uint32_t*)adc3_buffer, 4);

    /* Desligar interrupções do DMA para poupar ciclos de CPU (vamos ler em polling no ciclo de 25kHz) */
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_DisableIRQ(BDMA_Channel0_IRQn);
}

void ADC_Process_Data(void)
{
    /* Mapeamento de Leitura: Correntes do Motor (ADC1) */
    out_vector.i_a = (float)adc1_buffer[0];
    out_vector.i_b = (float)adc1_buffer[1];
    out_vector.i_c = (float)adc1_buffer[2];

    /* Mapeamento de Leitura: Rede e Sistema (ADC3) */
    in_vector.v_grid      = (float)adc3_buffer[0];
    in_vector.i_grid      = (float)adc3_buffer[1];
    sys_vector.v_dc_link  = (float)adc3_buffer[2];
    sys_vector.temp_heatsink = (float)adc3_buffer[3];
}

/* =========================================================
   Funções Getter Públicas (Acesso de Leitura Seguro)
   ========================================================= */
Input_Vector_t  ADC_Get_Input_Vector(void)  { return in_vector; }
Output_Vector_t ADC_Get_Output_Vector(void) { return out_vector; }
System_Vector_t ADC_Get_System_Vector(void) { return sys_vector; }
