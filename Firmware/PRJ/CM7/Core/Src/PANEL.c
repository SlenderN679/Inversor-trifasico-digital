/*
 * Panel.c
 *
 * Módulo de Gestão da Interação Humana Local (HMI).
 * Lida com as renderizações no Ecrã TFT ST7789 via SPI e gere
 * as entradas físicas do utilizador através do Encoder Rotativo.
 */
#include "PANEL.h"
#include "DD_ADC.h"
#include "PWM.h"
#include <stdio.h>
#include <SENSE.h>
#include "st7789.h"
#include "gpio.h"
#include "usart.h"

/* =========================================================
   Variáveis da Interface de Controlo
   ========================================================= */
volatile int encoder_button_pressed = 0;

extern volatile uint8_t afe_user_enable;
extern volatile uint8_t afe_control_enable;
extern volatile uint8_t afe_control_stage;
extern volatile uint8_t inverter_enable;

#define NUM_TABS 5
static int current_tab = 0;
static int in_edit_mode = 0;
static int force_redraw = 1;

static int16_t last_encoder_pos = 0;
static int motor_freq_setpoint = 50;
static int motor_ramp_setpoint = 10;

extern int fault_source; // 0=None, 1=VDC, 2=IAC, 3=HW, 4=Temp

/* =========================================================
   Inicialização e Desenhos Visuais
   ========================================================= */
void Panel_Init_Sequence(void) {
    ST7789_Init();
    ST7789_Fill_Color(BLACK);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    last_encoder_pos = ((int16_t)__HAL_TIM_GET_COUNTER(&htim4)) / 2;
}

void Panel_Draw_Fault_Screen(void) {
    ST7789_WriteString(10, 10, "!!! ALARM !!!  ", Font_11x18, WHITE, RED);
    ST7789_WriteString(10, 50, "SYSTEM TRIPPED ", Font_11x18, RED, BLACK);

    if (fault_source == 0)      ST7789_WriteString(10, 80, "STARTUP LOCK   ", Font_11x18, YELLOW, BLACK);
    else if (fault_source == 1) ST7789_WriteString(10, 80, "DC OVERVOLTAGE ", Font_11x18, YELLOW, BLACK);
    else if (fault_source == 2) ST7789_WriteString(10, 80, "AC OVERCURRENT ", Font_11x18, YELLOW, BLACK);
    else if (fault_source == 3) ST7789_WriteString(10, 80, "HARDWARE BREAK ", Font_11x18, YELLOW, BLACK);
    else if (fault_source == 4) ST7789_WriteString(10, 80, "OVERTEMPERATURE", Font_11x18, YELLOW, BLACK);

    ST7789_WriteString(10, 110,"Press to RESET ", Font_11x18, WHITE, BLACK);
}

/* =========================================================
   A Função Core da UI (Chamada a 10 Hz)
   ========================================================= */
void Panel_Update_UI(int safety)
{
    /* 1. Leitura Dinâmica do Encoder */
    int16_t encoder_pos = ((int16_t)__HAL_TIM_GET_COUNTER(&htim4)) / 2;
    int16_t delta = encoder_pos - last_encoder_pos;
    last_encoder_pos = encoder_pos;

    /* 2. Tratamento do Clique de Botão */
    if (encoder_button_pressed) {
        encoder_button_pressed = 0;

        if (safety) {
           extern int system_fault;
           system_fault = 0; fault_source = 0; force_redraw = 1;
           return; // Bloqueio: o reset não deve ativar funções de navegação
        }

        // Operações mediante o separador atual
        if (current_tab == 1) { // AFE Tab
            if (afe_control_stage == 0) afe_user_enable = 1;
            else if (afe_control_stage == 2 && afe_control_enable == 0) afe_control_enable = 1;
            else { afe_user_enable = 0; afe_control_enable = 0; }
        }
        else if (current_tab == 2 || current_tab == 4) { // Edição Numérica
            in_edit_mode = !in_edit_mode;
            force_redraw = 1;
        }
        else if (current_tab == 3) { // Ligação Inversor
            if (afe_control_stage == 2 && afe_control_enable == 1) inverter_enable = !inverter_enable;
            force_redraw = 1;
        }
    }

    /* 3. Tratamento da Rotação */
    if (delta != 0) {
        if (in_edit_mode && current_tab == 2) {
            motor_freq_setpoint += delta;
            if (motor_freq_setpoint > 100) motor_freq_setpoint = 100;
            if (motor_freq_setpoint < 0)  motor_freq_setpoint = 0;
            PWM_Inverter_Set_Freq((float)motor_freq_setpoint);
        }
        else if (in_edit_mode && current_tab == 4) {
            motor_ramp_setpoint += delta;
            if (motor_ramp_setpoint > 50) motor_ramp_setpoint = 50;
            if (motor_ramp_setpoint < 1)  motor_ramp_setpoint = 1;
            extern void PWM_Inverter_Set_Ramp(float);
            PWM_Inverter_Set_Ramp((float)motor_ramp_setpoint);
        }
        else if (!in_edit_mode) {
            current_tab += delta;
            if (current_tab >= NUM_TABS) current_tab = 0;
            if (current_tab < 0) current_tab = NUM_TABS - 1;
            force_redraw = 1;
        }
    }

    /* 4. Renderização do Ecrã */
    if (force_redraw) { ST7789_Fill_Color(BLACK); force_redraw = 0; }

    char buf[32];
    if(safety) {
        Panel_Draw_Fault_Screen();
    } else {
        // Renderizar Cabeçalhos Variáveis
        if (current_tab == 0) ST7789_WriteString(10, 10, "<- TELEMETRY ->", Font_11x18, CYAN, BLACK);
        if (current_tab == 1) ST7789_WriteString(10, 10, "<- AFE CTRL  ->", Font_11x18, YELLOW, BLACK);
        if (current_tab == 2) ST7789_WriteString(10, 10, "<- INV FREQ  ->", Font_11x18, MAGENTA, BLACK);
        if (current_tab == 3) ST7789_WriteString(10, 10, "<- INV PWR   ->", Font_11x18, RED, BLACK);
        if (current_tab == 4) ST7789_WriteString(10, 10, "<- ACCEL RAMP->", Font_11x18, ORANGE, BLACK);
        ST7789_WriteString(0, 25, "-----------------", Font_11x18, WHITE, BLACK);

        // Renderizar Corpo das Abas
        if (current_tab == 0) {
            sprintf(buf, "V_DC: %3.0f V   ", Sense_Get_V_DC_Link());
            ST7789_WriteString(10, 50, buf, Font_11x18, WHITE, BLACK);
            sprintf(buf, "TEMP: %2.1f C   ", Sense_Get_Heatsink_Temp());
            ST7789_WriteString(10, 80, buf, Font_11x18, GREEN, BLACK);
            sprintf(buf, "I_GRID: %2.1f A ", Sense_Get_I_Grid());
            ST7789_WriteString(10, 110, buf, Font_11x18, WHITE, BLACK);
        }
        else if (current_tab == 1) {
            ST7789_WriteString(10, 50, "PRESS TO CHANGE:", Font_11x18, WHITE, BLACK);
            if (afe_control_stage == 0) ST7789_WriteString(10, 80, "[ STANDBY ]   ", Font_11x18, RED, BLACK);
            else if (afe_control_stage == 1) ST7789_WriteString(10, 80, "[ PRE-CHARGE ]", Font_11x18, YELLOW, BLACK);
            else if (afe_control_stage == 2) {
                if (afe_control_enable) ST7789_WriteString(10, 80, "[ RUNNING ]   ", Font_11x18, GREEN, BLACK);
                else ST7789_WriteString(10, 80, "[ DC READY ]  ", Font_11x18, CYAN, BLACK);
            }
        }
        else if (current_tab == 2) {
            ST7789_WriteString(10, 50, "MOTOR FREQ:", Font_11x18, WHITE, BLACK);
            sprintf(buf, "%d Hz    ", motor_freq_setpoint);
            if (in_edit_mode) {
                ST7789_WriteString(10, 80, buf, Font_11x18, BLACK, GREEN);
                ST7789_WriteString(10, 110, "<- EDITING ->", Font_11x18, GREEN, BLACK);
            } else {
                ST7789_WriteString(10, 80, buf, Font_11x18, GREEN, BLACK);
                ST7789_WriteString(10, 110, "[ LOCKED ]   ", Font_11x18, RED, BLACK);
            }
        }
        else if (current_tab == 3) {
            ST7789_WriteString(10, 50, "MOTOR POWER:", Font_11x18, WHITE, BLACK);
            if (afe_control_stage != 2 || !afe_control_enable) {
                ST7789_WriteString(10, 80, "[ WAIT AFE ]  ", Font_11x18, YELLOW, BLACK);
                ST7789_WriteString(10, 110, "Needs 400V DC  ", Font_11x18, WHITE, BLACK);
            } else {
                if (inverter_enable) {
                    ST7789_WriteString(10, 80, "[ RUNNING ]   ", Font_11x18, GREEN, BLACK);
                    ST7789_WriteString(10, 110, "Press to STOP  ", Font_11x18, WHITE, BLACK);
                } else {
                    ST7789_WriteString(10, 80, "[ STOPPED ]   ", Font_11x18, RED, BLACK);
                    ST7789_WriteString(10, 110, "Press to START ", Font_11x18, WHITE, BLACK);
                }
            }
        }
        else if (current_tab == 4) {
            ST7789_WriteString(10, 50, "RAMP (Hz/s):", Font_11x18, WHITE, BLACK);
            sprintf(buf, "%d Hz/s    ", motor_ramp_setpoint);
            if (in_edit_mode) {
                ST7789_WriteString(10, 80, buf, Font_11x18, BLACK, GREEN);
                ST7789_WriteString(10, 110, "<- EDITING ->", Font_11x18, GREEN, BLACK);
            } else {
                ST7789_WriteString(10, 80, buf, Font_11x18, ORANGE, BLACK);
                ST7789_WriteString(10, 110, "[ LOCKED ]   ", Font_11x18, RED, BLACK);
            }
        }
    }
}

/* =========================================================
   Rotina de Interrupção com Debounce
   ========================================================= */
uint32_t last_button_time = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == SW_Pin) {
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_button_time) > 250) { // 250ms delay proteção física
            if (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET) {
                encoder_button_pressed = 1;
                last_button_time = current_time;
            }
        }
    }
}
