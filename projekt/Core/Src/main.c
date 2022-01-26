/* USER CODE BEGIN Header */
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD_i2c_config.h"
#include "LCD_i2c.h"
#include "stdio.h"
#include "arm_math.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//parametry regulatora
#define PID_TS         0.1
#define PID_KP1        0.15
#define PID_KI1        0.35
#define PID_KD1        0.00

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//inicjalizacje zmiennych

//sygnaly i regulator
uint16_t sygnal_pomiarowy;

int16_t PWM = 0;
int16_t uchyb = 0;
float PWM_float = 0;
char sygnal_sterujacy_send[4];

arm_pid_instance_f32 PID_regulator;

int16_t sygnal_sterujacy;

//komunikacja szeregowa i ekran LCD
char send_line[26];
char send_line_usart[100];

//przetwornik ADC
const uint32_t ADC_REG_MAX = 0xfff;
const float ADC_VOLTAGE_MAX = 3.3;
const uint32_t ADC_TIMEOUT = 100;

uint32_t ADC_measure = 0;
float ADC_measure_V;
uint32_t ADC_measure_mV = 0;
_Bool LCD_update = 0;

//enkoder
uint16_t enco_abs = 0;
int16_t enco = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//reset countera od enkodera
void TIM_ResetCounter(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Reset the Counter Register value */
  TIMx->CNT = 0;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM6)
	{
		if(!LCD_update)
		{
			//pomiar napiecia na dzielniku
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT);
			ADC_measure = HAL_ADC_GetValue(&hadc1);
			ADC_measure_V = ((float)ADC_measure/(float)ADC_REG_MAX) * ADC_VOLTAGE_MAX;
			ADC_measure_mV = (uint32_t)(1000.0 * ADC_measure_V);
			sygnal_pomiarowy = (uint16_t)ADC_measure_mV;
		}

		//uchyb i moc PWM
		uchyb = (int16_t)(sygnal_sterujacy - sygnal_pomiarowy);

		PWM_float = arm_pid_f32(&PID_regulator, uchyb);
		PWM = (uint16_t)(PWM_float);

		//anti-windup
		if(PWM_float > 2000)
		{
			PWM = 2000;
			if(PID_regulator.state[2] > 2000)
					{
						PID_regulator.state[2] = 2000;
					}
		}
		else if (PWM_float < 0)
		{
			PWM = 0;
			PID_regulator.state[2] = 0;
		}

		//przypisanie % PWM do odpowiednich kanalow
		if(PWM <= 1000)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (PWM - 1000));
		}
	}



	if (htim->Instance == TIM7)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		LCD_update = 1;
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//potwierdzenie wyslania danych
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	if(huart->Instance == USART3)
	{
		sygnal_sterujacy = 1000*((int8_t)sygnal_sterujacy_send[0]-'0')+100*((int8_t)sygnal_sterujacy_send[1]-'0')+10*((int8_t)sygnal_sterujacy_send[2]-'0')+1*((int8_t)sygnal_sterujacy_send[3]-'0');

		//ograniczenie sygnalu sterujacego w bezpiecznym zakresie
			if(sygnal_sterujacy > 2700)
			{
				sygnal_sterujacy = 2700;
			}
			else if(sygnal_sterujacy < 1600)
			{
				sygnal_sterujacy = 1600;
			}

		//ustawienie wartosci sygnalu sterujacego w porcie szeregowym
		HAL_UART_Receive_IT(&huart3, (uint8_t*)sygnal_sterujacy_send, 4);

	}

}
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //inicjalizacje kanalow PWM, enkodera i USART
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)sygnal_sterujacy_send, 4);

  lcd_init(&hLCD_1);
  HAL_Delay(10);
  lcd_send_cmd(&hLCD_1, LCD_DISPLAY_ON_OFF_CONTROL | LCD_OPT_D);

  //parametry regulatora oraz inicjalizacja
  PID_regulator.Kp = PID_KP1;
  PID_regulator.Ki = PID_KI1 * PID_TS;
  PID_regulator.Kd = PID_KD1 / PID_TS;

  arm_pid_init_f32(&PID_regulator, 1);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  	  //odczyt stanu enkodera
	  	  	  enco_abs = __HAL_TIM_GET_COUNTER(&htim4) / 4;

	  	  	  //ustawienie zakresu pracy enkodera
	  	  	  if(enco_abs > 0 && enco_abs < 1000)
	  	  	  {
	  	  		  enco = enco_abs;
	  	  	  }
	  	  	  else if(enco_abs <= 16384 && enco_abs > 15000)
	  	  	  {
	  	  		  enco = -(16384 - enco_abs);
	  	  	  }

	  	  	  //zmiana wartosci sygnalu sterujacego za pomoca enkodera
	  	  	  sygnal_sterujacy = sygnal_sterujacy + enco;

	  	  	  //ograniczenie sygnalu sterujacego, gdy jest zmieniany z uzyciem enkodera
	  	  	  if(sygnal_sterujacy > 2700)
	  	  				{
	  	  					sygnal_sterujacy = 2700;
	  	  				}
	  	  				else if(sygnal_sterujacy < 1600)
	  	  				{
	  	  					sygnal_sterujacy = 1600;
	  	  				}

	  	  	  //reset stanu enkodera
	  	  	  TIM_ResetCounter(TIM4);
	  	  	  enco_abs = 0;
	  	  	  enco = 0;

	  		if(LCD_update)
	  			  {
	  				  //odczyt wartosci sygnalow na ekranie LCD
	  				  sprintf(send_line, "Set: mV:  PWM:");
	  				  lcd_clear(&hLCD_1);
	  				  lcd_send_string (&hLCD_1, send_line, 0, 0);
	  				  HAL_Delay(2);
	  				  sprintf(send_line, "%d %d %d", (uint16_t)sygnal_sterujacy, (uint16_t)sygnal_pomiarowy, (uint16_t)PWM);
	  				  lcd_send_string (&hLCD_1, send_line, 1, 0);

	  				  //odczyt wartosci sygnalow w porcie szeregowym
	  		  		  uint8_t n = sprintf(send_line_usart, "Sygnal_sterujacy: %d; Sygnal_pomiarowy: %d; PWM: %d; \n\r", (uint16_t)sygnal_sterujacy, (uint16_t)sygnal_pomiarowy, (uint16_t)PWM);
	  		  		  HAL_UART_Transmit(&huart3, (uint8_t*)send_line_usart, n, 100);
	  				  LCD_update = 0;
	  			  }

	  	  //opoznienie odczytu danych
	 	  HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

