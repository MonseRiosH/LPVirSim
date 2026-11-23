/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define length 2 //ADC inputs
#define PI 3.14159 //PI
#define NBSAMPLES 6 //constante define el orden del filtro
#define N_ADC 2  //numero de canales de ADC para el filtro
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t TEXT[100]; //Size of the buffer for the text to print
uint16_t ADC1_buffer[length]; //Number of ADC channels
uint16_t adc1_ch5, adc1_ch5_conv; //Variables to save the ADC data
uint16_t duty; // PWM
uint8_t reset; //Reset the program
uint8_t enc1_sent_rot; //Sentido del giro del encoder
int32_t enc1_cont;
uint8_t enc_direction; //Direction of the motor
uint8_t flag, conta; //Help to reset
float pos_rad,step; // Position in radians
uint8_t cont;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//ENCODER
//Use this function when TIM2 and/or TIM5 (have 32 bits) are activated as Encoder
int32_t encoder_read_32_bits(TIM_HandleTypeDef *htim, uint8_t reset, uint8_t *dir) {
	//when TIM2 or TIM5 are used they have 32 bits
#if (htim == htim2 || htim == htim5) //"#if preprocessor directive"
	static int32_t cnt_enc=0;
	cnt_enc = __HAL_TIM_GET_COUNTER(htim);
    *dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);

    if (reset) {
		__HAL_TIM_SET_COUNTER(htim,0);
	}

    return cnt_enc;

#endif
}
//FILTRO
uint16_t rr_fir_filter(uint8_t ch, uint16_t newsample) {
	if (ch>N_ADC) return 0;
	static uint16_t samples[N_ADC][NBSAMPLES]={0};
	static uint16_t numsample=0;
	static uint16_t slidingsum[N_ADC]={0};
	slidingsum[ch] -= samples[ch][numsample];
	samples[ch][numsample]= newsample;
	slidingsum[ch]+= samples[ch][numsample];
	numsample++;
	numsample%=NBSAMPLES;
	return slidingsum[ch]/NBSAMPLES;
}
//TIMER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static int16_t cnt1 = 0;
	static int16_t cnt2 = 0;
	static int16_t cnt3 = 0;
	static int32_t cntTime = 0;

	if(htim->Instance == TIM6){//si se ejecuta TIM6 1kHz
		cnt1++; //inician los contadores
		cnt2++;
		//cnt3++; //Apaga el driver
		duty = 0;
		/*Frecuencia de muestreo a 1 kHz en esta seccion*/

		adc1_ch5 = ADC1_buffer[0];
	    adc1_ch5_conv = rr_fir_filter(0, ADC1_buffer[0]);

		reset = HAL_GPIO_ReadPin(BTN_AZUL_GPIO_Port, BTN_AZUL_Pin);
		enc1_cont = encoder_read_32_bits(&htim2, reset, &enc1_sent_rot);
		pos_rad = (enc1_cont/(4.0*520))*2*PI;

		if(reset == 1  && flag ==0){
			cont++;
			flag = 1;
		}
		if(cont == 1){
			duty = 890;
			HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(HBridge_Sleep_GPIO_Port, HBridge_Sleep_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
			HAL_GPIO_WritePin(Dir_M1_GPIO_Port, Dir_M1_Pin, GPIO_PIN_SET);
			cnt3++;
		}

		if(cont ==2){
			cont = 0;
			HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(HBridge_Sleep_GPIO_Port, HBridge_Sleep_Pin, GPIO_PIN_RESET);
		    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(Dir_M1_GPIO_Port, Dir_M1_Pin, GPIO_PIN_RESET);
			cnt3 = 0;
		}
		if(reset == 0){
			flag = 0;
		}
		/*----------------------------*/
		//Instancia a otra velocidad de muestreo
		if(cnt1 == 200){ //200ms = 5 Hz
			HAL_GPIO_TogglePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin);
			cnt1 = 0;
		}

		if(cnt3 >4999){
			HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(HBridge_Sleep_GPIO_Port, HBridge_Sleep_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				HAL_GPIO_WritePin(Dir_M1_GPIO_Port, Dir_M1_Pin, GPIO_PIN_RESET);
				cnt3 = 0;
				cont = 0;
				duty = 0;

		}
		if(cnt2 == 10){//10ms = 100 Hz
			step = 01.123;
			//sprintf(TEXT+5, "%2.3f\r\n",step);
			sprintf(TEXT, "%16ld, %4d, %4ld, %4ld, %8ld, %9.3f, %4ld, %4u\r\n",cntTime, adc1_ch5, adc1_ch5_conv, cont,enc1_cont, pos_rad,cnt3, duty);
			//Contador ms
			//Contador del encoder
			//Posicion en radianes
			//Contador ms
			// duty
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)TEXT, strlen(TEXT));
			cnt2 = 0;
		}


	}
}

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim6);//inicia al TIM6 como interval INT
//sprintf(TEXT, "UAEM ");
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_buffer, length);

//inicializa el TIM2 en modo encoder con ambos canales A, B
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //TIM2 es de 32 bits

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
