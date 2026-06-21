/* USER CODE BEGIN Header */
/******************************************************************************
 * @file           : Arquitetura_Projeto.txt / main.c
 * @brief          : Controlo Digital de Variador de Velocidade e Retificador AFE
 * @author         : Nuno Costa (a107069) e Afonso Carvalho (a107058)
 * @institution    : Universidade do Minho
 * @date           : Junho de 2026
 * * @description    :
 * Este projeto implementa o controlo digital em tempo real (25 kHz) de um
 * motor trifásico usando a técnica SPWM e curva V/f constante, alimentado
 * por um Retificador Ativo (AFE - Active Front End) com controlo de histerese.
 * O sistema opera num STM32H7 e conta ainda com telemetria SCADA, Interface
 * local e um Simulador incorporado.
 *
 * * ============================================================================
 * ESTRUTURA DE FICHEIROS E MÓDULOS
 * ============================================================================
 * * [ CORE ]
 * > main.c / main.h
 * Gere a máquina de estados principal, inicializa os
 * periféricos e implementa os três domínios de tempo de execução:
 * - Prioridade Máxima: Interrupções de segurança e paragem de emergência.
 * - 25 kHz (Tempo Real): Controlo físico de potência, processamento ADC e algoritmos PI.
 * - 10 Hz (Assíncrono): Atualização de ecrã e transmissão de telemetria.
 *
 * * [ POTÊNCIA E CONTROLO ]
 * > PWM.c / PWM.h
 * Módulo de Eletrónica de Potência. Executa a malha PLL (Phase-Locked Loop),
 * o controlo PI de tensão do barramento DC e o controlo de corrente
 * (histerese) do AFE. Gera a modulação SPWM trifásica defasada a 120º e a rampa
 * de aceleração para o inversor do motor.
 *
 * * [ SENSORES E SINAIS ]
 * > DD_ADC.c / DD_ADC.h
 * Device Driver de Abstração de Hardware. Gere o mapeamento de memória SRAM
 * específico do hardware (D2/D3) e a transferência direta de memória (DMA e BDMA)
 * dos conversores ADC1 e ADC3.
 * * > SENSE.c / SENSE.h
 * Módulo de Processamento de Sinal. Converte os valores RAW dos conversores
 * em grandezas físicas reais (Volts, Amperes, Celsius) e processa os
 * cálculos True RMS (janela de 20ms). Age como intermediário para a injeção
 * de dados do simulador.
 *
 * * [ INTERFACE E TELEMETRIA ]
 * > PANEL.c / PANEL.h
 * Human-Machine Interface (HMI). Controla o ecrã ST7789 via comunicação
 * SPI e gere a navegação das tabs de interface e edição de setpoints através
 * de um Encoder Rotativo com debounce de hardware.
 * * > INTER.c / INTER.h
 * Módulo de Comunicação. Compila a telemetria completa do sistema num
 * formato JSON otimizado e gere o envio não-bloqueante dos pacotes via
 * UART/DMA para o *dashboard* Node-RED.
 *
 * * [ VALIDAÇÃO E TESTE ]
 * > SIMUL.c / SIMUL.h
 * Hardware-in-the-Loop (Digital Twin). Emula matematicamente a impedância da
 * rede elétrica, a resposta física do condensador do barramento DC e o consumo
 * indutivo do motor. Injeta os sinais virtuais via DAC e no código para validação
 * do controlo sem potência real aplicada.
 * ******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "bdma.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//--------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include "st7789.h"
#include "PWM.h"
#include "SENSE.h"
#include "SIMUL.h"
#include "INTER.h"
#include "PANEL.h"
//--------------------------------------------------------------------------
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--------------------------------------------------------------------------
/* Variáveis de Máquina de Estados Global */
volatile uint8_t afe_user_enable = 0;
volatile uint8_t afe_control_enable = 0;
volatile uint8_t afe_control_stage = 0;

volatile int tim = 0;
volatile int Emerg = 0;

int system_fault = 0;
int fault_source = 0;
volatile uint8_t inverter_enable = 0;
//--------------------------------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //--------------------------------------------------------------------------
  /* Arranque Seguro Sequencial dos Sistemas Próprios */
  Panel_Init_Sequence();          // Lança a Interface Humana
  Sense_Init_DMA(&hadc1, &hadc3); // Lança o cérebro de Sensoreamento DMA
  DacSim_Init(&hdac1);            // Instancia os modelos de simulação na placa
  PWM_Start_All();                // Energiza todos os módulos de sinal PWM
  //--------------------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
	  /* =========================================================
		 DOM�?NIO 1: ALARME E INTERRUPÇÕES F�?SICAS (Prioridade 1)
		 ========================================================= */
	  if (Emerg) {
		  PWM_Emergency_Stop();
		  PWM_AFE_PWM_Kill();
		  PWM_Set_Main_Contactor(0); // Abre a rede
		  system_fault = 1;
		  fault_source = 3; // O hardware ativou o BKIN

		  /* Cortar e desligar lógica humana */
		  afe_user_enable = 0;
		  afe_control_enable = 0;
		  afe_control_stage = 0;

		  Emerg = 0; // Desarma flag da interrupção
	  }

	  /* =========================================================
		 DOM�?NIO 2: TEMPO REAL 25 kHz (Prioridade 2)
		 ========================================================= */
	  if (tim) {
		  /* 1. Ciclo Físico -> Virtual -> Sensorial -> Defesa */
		  DacSim_Update_Grid_DC(&hdac1);
		  Simul_Update_Motor_Currents();
		  Sense_Process_Data();
		  Check_Safety();

		  /* 2. Passagem do mundo real para o espaço matemático */
		  float v_pg = Sense_Get_V_Grid_Inst();
		  float i_pg = Sense_Get_I_Grid_Inst();
		  float v_cc = Sense_Get_V_DC_Link();

		  PWM_AFE_Process_Background_Math(v_pg, v_cc); // Processamento passivo contínuo

		  /* 3. Máquina de Estados do Retificador (AFE Control) */
		  switch(afe_control_stage){
			  case 0: // STANDBY (DESCARGA DE SEGURANÇA)
				  HAL_GPIO_WritePin(SoftStart_GPIO_Port, SoftStart_Pin, GPIO_PIN_RESET);
				  PWM_AFE_PWM_Kill();
				  inverter_enable = 0;
				  PWM_Inverter_Update(0); // Força desligamento do Inversor
				  PWM_Set_Main_Contactor(0); // Abre a rede

				  if (afe_user_enable) afe_control_stage = 1; // Ativação pelo UI
				  break;

			  case 1: // PRÉ-CARGA (CONTROLO TÉRMICO)
			  	  system_fault = 0;
			  	  // 1. Fecha o contactor principal da rede (se ainda não estiver fechado)
			  	  PWM_Set_Main_Contactor(1);
  				  // 2. Executa a lógica que verifica a tensão e fecha o relé de Soft-Start
  				  PWM_Handle_SoftStart_Sequence(v_cc);
  				  // 3. Verifica se a tensão já atingiu o patamar seguro para avançar a máquina de estados
  				  if (v_cc > 35.0f) {
  					  afe_control_stage = 2; // Passa ao controlo de potência final
  				  }
  				  if (!afe_user_enable) {
                        PWM_Set_Main_Contactor(0); // Abre a rede se o utilizador desistir
                        afe_control_stage = 0;
                    }
  				  break;

			  case 2: // RETIFICAÇÃO ATIVA E DESPACHO
				  if (afe_control_enable) {
					  PWM_AFE_Run_PI_and_Hysteresis(i_pg);
					  PWM_Inverter_Update(inverter_enable); // Depende da UI do Inversor
				  } else {
					  PWM_AFE_PWM_Kill();
					  inverter_enable = 0;
					  PWM_Inverter_Update(0);
					  PWM_Set_Main_Contactor(0); // Abre a rede
				  }

				  if (!afe_user_enable && !afe_control_enable) afe_control_stage = 0;
				  break;
		  }
		  tim = 0; // Sinaliza fim de processamento do ciclo temporal
	  }

	  /* =========================================================
		 DOM�?NIO 3: INTERFACE E TELEMETRIA (10 Hz)
		 ========================================================= */
	  static uint32_t last_update = 0;

	  if (HAL_GetTick() - last_update > 100) // Delay não-bloqueante
	  {
		 last_update = HAL_GetTick();
		 Panel_Update_UI(system_fault);
		 Update_Interface(&huart3);

		 float temp_atual = Sense_Get_Heatsink_Temp();
		 PWM_Update_Cooling_Fan(temp_atual);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 10;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//--------------------------------------------------------------------------
/* Interrupção Base do Sistema de Tempo (25 kHz via TIM1) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1)
    	tim = 1;
}

/* Interrupção Assíncrona de Falha Eletrónica (Falha Ativa em Nível Lógico) */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1 || htim->Instance == TIM8)
    	Emerg = 1;
}
//--------------------------------------------------------------------------
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
