/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

#define DATA_SIZE  38554                     /*  sample_t secund = 1/96000000/4/(122 + 12.5)  DATA_SIZE = 0.2 secund /sample_t   */

#define DEFAULT_CHAUNKS 4                    /*  total time 0.2 * 4 = 0.8   */
#define DEFAULT_DISCARGED_CHAUNKS 1           /*                 */
#define DEFAULT_REQUESTS_TIME_MS 1000         
#define DEFAULT_COUNT_DISCHARGED_PULSES 30


volatile uint16_t adc_value[DATA_SIZE];


uint16_t count = 0;
volatile uint16_t count_pulses = 0;

volatile uint8_t MAX_CHANKS = 4;
volatile uint8_t current_chank = 0;
volatile uint8_t isChaunksFull = 0;


uint8_t flag = 0;
unsigned long T;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum 
{
  FALSE = 0U, 
  TRUE = !FALSE
} BoolState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


struct response {
  uint16_t count_pulses;
  uint8_t flag_requests;
}; 
struct response current_response, ready_response = { 0, FALSE };


struct configurations {

  uint16_t response_period_ms;
  uint8_t current_chanks;
  uint8_t discharged_mode;
  uint8_t count_discharged_chanks;
  uint16_t count_discarge_pulses;

}; 

struct configurations __conf_deault, conf = { DEFAULT_REQUESTS_TIME_MS, DEFAULT_CHAUNKS, 
                                              TRUE, DEFAULT_DISCARGED_CHAUNKS, DEFAULT_COUNT_DISCHARGED_PULSES };


void response_copy(){
  ready_response.count_pulses = current_response.count_pulses;
  ready_response.flag_requests = FALSE;
  current_response.count_pulses = 0;
}


void send_message_virtual_com(int current){
  sprintf((char*)UserTxBufferFS, "count pulses detected: %u\r\n", current);
  CDC_Transmit_FS(UserTxBufferFS, sizeof(UserTxBufferFS)/sizeof UserTxBufferFS[0]);
  ready_response.flag_requests = TRUE;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  if (hadc->Instance == ADC1){
    HAL_ADC_Stop_DMA(&hadc1);
    current_chank++;
    
    int count = 0;
    for (int i = 0; i < DATA_SIZE; i++){
      if (adc_value[i] > 2000){
        count++;
      }else{
        if (count > 0){
          // sprintf((char*)UserTxBufferFS, "count pulses: %u\r\n", count_pulses);
          // CDC_Transmit_FS(UserTxBufferFS, sizeof(UserTxBufferFS)/sizeof UserTxBufferFS[0]);
          count_pulses++;
        }
        count = 0;
      }
    }
    current_response.count_pulses += count_pulses;

    if (current_chank == conf.current_chanks){
      isChaunksFull = TRUE;
      current_chank = 0;
      count_pulses = 0;
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_value, DATA_SIZE);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_value, DATA_SIZE);
  T = HAL_GetTick();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    TIM2 -> CCR1 = 10;
    TIM2 -> CCR2 = 810;

    HAL_GPIO_WritePin(GPIOC, GEN_Pin, GPIO_PIN_SET);
	  HAL_Delay(1);

	  HAL_GPIO_WritePin(GPIOC, GEN_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);
    
    if (isChaunksFull){
      isChaunksFull = FALSE;
      response_copy();
    }


    if (HAL_GetTick() - T >= conf.response_period_ms){
      T = HAL_GetTick();

      int current = 0;
      if (ready_response.flag_requests){
        current = 0;
      }else{
        current = ready_response.count_pulses;
      }
      send_message_virtual_com(current);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/