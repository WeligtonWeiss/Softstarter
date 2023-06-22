/* USER CODE BEGIN Header */
/*
 * Código do Soft-Starter totalmente controlado de microcontrolador
 * Alunos: Augusto Sturm, Stephanie Staub e Weligton Weiss
 * Turma 4411
 * Existem algumas redundâncias no código para garantir o funcionamento e impedir que algo que não deveria acontecer ->
 * aconteça durante o funcionamento do motor
 */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Constantes do programa
#define TMAX 10
#define TIMEOUT 1000
#define NOMINAL 450
#define ESCALA 0.80566
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum{SOBE=0,DESCE} bordas_t;
typedef enum{PARADO=0,SUBIDA, DESCIDA, BYPASS} rampa_t;	// Estados do motor
uint16_t tpulso[2] = {17400,17940}; // Variável usada para redefinir o CCR do OC
uint16_t ANGULOSLista[] = {	17400,	// Lista de angulos calculada a partir de um script feito em python para ir encrementando a tensão no motor linearmente
		17000,
		16500,
		16172,
		15403,
		14805,
		14294,
		13838,
		13419,
		13028,
		12659,
		12307,
		11968,
		11640,
		11321,
		11010,
		10705,
		10405,
		10108,
		9815,
		9523,
		9233,
		8944,
		8655,
		8364,
		8072,
		7778,
		7480,
		7178,
		6871,
		6557,
		6235,
		5903,
		5559,
		5200,
		4823,
		4423,
		3991,
		3516,
		2977,
		2329,
		1300};

rampa_t rampa=PARADO;
bordas_t borda;
uint8_t i = 0; // variavel de contagem
char msg[TMAX] = "\0";
uint8_t proto[4];
float ang = 0, sobrelimite = 1.5, correntedesliga = 2;
float partida = 5, desliga = 5, ADvalue, corrente;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */
	htim3.Instance->CCMR1 |= TIM_CCMR1_OC1CE;
	HAL_UART_Receive_IT(&huart2, proto, 4);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim10);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, tpulso[borda]);
		if(borda==DESCE){
			borda=SOBE;
		}
		else{
			borda=DESCE;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		if(rampa==SUBIDA){
			if(i<=41){
				tpulso[0]=ANGULOSLista[i];
				tpulso[1]=tpulso[0]+540;
				if(i==41){ // Se i está no valor máximo de contagem
					rampa=BYPASS; // Redefine o estado do motor
					htim3.Instance->CCMR1 &=~ TIM_CCMR1_OC1CE; // Reseta o registrador do clear enable
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); // Aciona o relé de bypass
					return;
				}
				if(corrente<(NOMINAL*sobrelimite)){ // Só permite decrementar o angulo caso a corrente esteja abaixo do valor de sobrecorrente
					i++;
				}else{
					strcpy(msg, "SCR\n"); // Envia para o monitor que a sobrecorrente foi atingida para ele exibir uma notificação
					HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
				}
			}
			strcpy(msg, "RDS\n"); // Envia para o monitor o estado do motor - Rampa de subida
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
		}
		else if(rampa==DESCIDA){
			if(i>=0){
				tpulso[0]=ANGULOSLista[i];
				tpulso[1]=tpulso[0]+540; // Acrescenta 540 do ARR no CCR para definir a borda de descida do pulso
				if(i==0){ // Se i está no valor minimo de contagem
					rampa=PARADO; // Redefine o estado do motor
					HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1); // Desliga o OC
					htim3.Instance->CCMR1 |= TIM_CCMR1_OC1CE; // Seta o registrador do clear enable
					return;
				}
				i--;
			}
			strcpy(msg, "RDD\n"); // Envia para o monitor o estado do motor - Rampa de descida
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
		}
		else if(rampa==PARADO){
			i=0;
			tpulso[0]=ANGULOSLista[i];
			tpulso[1]=tpulso[0]+540; // Acrescenta 540 do ARR no CCR para definir a borda de descida do pulso
			HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1); // Desliga o OC
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); // Desaciona o relé
			htim3.Instance->CCMR1 |= TIM_CCMR1_OC1CE; // Seta o registrador do clear enable
			strcpy(msg, "OFF\n"); // Envia para o monitor o estado do motor - parado
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
		}
		else if(rampa==BYPASS){
			htim3.Instance->CCMR1 &=~ TIM_CCMR1_OC1CE; // Reseta o registrador do clear enable
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); // Aciona o relé
			strcpy(msg, "ON\n"); // Envia para o monitor o estado do motor - na rede
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
		}
	}
	else if(htim->Instance == TIM10){
		HAL_ADC_Start_IT(&hadc1);
		if(rampa==SUBIDA || rampa==DESCIDA){
			ang=ANGULOSLista[i]/100;
			sprintf(msg, "%.0f\n", ang);
		}
		else{
			if(rampa == PARADO){
				ang = 180;
				sprintf(msg, "%.0f\n", ang);
			}
			else{
				ang = 0;
				sprintf(msg, "%.0f\n", ang);
			}
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// Analizando agora, poderia ter sido feito um switch case no lugar de vários if's
	if(proto[0] == 'd'){ // Rampa de descida
		desliga = (proto[1]-'0')*10+(proto[2]-'0'); // Pega os valores definidos como tempo de desligamento
	}
	else if(proto[0] == 'l'){ // Rampa de subida
		partida = (proto[1]-'0')*10+(proto[2]-'0'); // Pega os valores definidos como tempo de partida
	}
	else if(proto[0] == 's'){ // Sobrecorrente limite
		sobrelimite = (proto[1]-'0')*100+(proto[2]-'0')*10+(proto[3]-'0'); // pega os valores de sobrecorrente
		sobrelimite = sobrelimite/100;									   // valor de sobrecorrente em %
	}
	else if(proto[0] == 'c'){ // Corrente de desligamento
		correntedesliga = ((proto[1]-'0')*100+(proto[2]-'0')*10+(proto[3]-'0')); // pega os valores de corrente de desligamento
		correntedesliga = correntedesliga/100;									 // valor de corrente de desligamento em %
	}
	else if(proto[0] == 'L'){ // Botao de liga
		rampa=SUBIDA;
		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1); // liga os pulsos
		__HAL_TIM_SET_PRESCALER(&htim2, partida/0.042); // Seta o tempo que o angulo de disparo vai ser alterado no upddate do tim2 acordo com o tempo definido no slider
	}
	else if(proto[0] == 'D'){ // Botao de desliga
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); // Desaciona o relé
		rampa=DESCIDA; //
		__HAL_TIM_SET_PRESCALER(&htim2, desliga/0.042); // Seta o tempo que o angulo de disparo vai ser alterado no upddate do tim2 acordo com o tempo definido no slider
	}
	else if(proto[0]== 'E'){ // Botao de emergencia
		HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1); // Desliga o OC
		i=0; // Zera a contagem
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); // Desaciona o relé
		htim3.Instance->CCMR1 |= TIM_CCMR1_OC1CE; // Seta o registrador do clear enable
		rampa=PARADO; // Redefine o estado do motor
	}

	HAL_UART_Receive_IT(&huart2, proto, 4);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	ADvalue = HAL_ADC_GetValue(&hadc1);
	corrente = (ADvalue*ESCALA)*0.7 + corrente*0.3; // Faz uma média, atualizando com 70% do valor atual do adc e mantendo 30% do valor anterior
	sprintf(msg,"%-.2f mA\n", corrente); // Envia a corrente para o monitor para ser exibido na ihm
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
	if(corrente>(NOMINAL*correntedesliga) && (rampa == SUBIDA || rampa == BYPASS)){ // Se atingiu a corrente de desligamento enquanto estava acelerando ou na rede
		HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1); // Desliga o OC
		htim3.Instance->CCMR1 |= TIM_CCMR1_OC1CE; // Seta o registrador do clear enable
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); // Desaciona o relé
		rampa=PARADO; // Redefine o estado do motor
		i=0; // Zera a contagem
		strcpy(msg, "CDG\n"); // Envia para o monitor que a corrente de desligamento foi atingida para ele exibir uma notificação
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), TIMEOUT);
	}
}

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
