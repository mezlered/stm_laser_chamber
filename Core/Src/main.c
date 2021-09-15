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
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include <stdbool.h>

#define adress_i2c 0x2A
uint8_t rx_buf[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

// rx_buf = {1, 2, 3, 4, 5, 6, 7, 8, 9}



extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_RX_DATA_SIZE];

#define DATA_SIZE  38554                                               /*  sample_t secund = 1/96000000/4/(122 + 12.5)  DATA_SIZE = 0.2 secund /sample_t        */

#define DEFAULT_CHAUNKS 5                                              /*  total time 0.2 * 4 = 0.8                                                             */
#define DEFAULT_DISCARGED_CHAUNKS 1                                    /*                                                                                       */
#define DEFAULT_REQUESTS_TIME_MS 1000         
#define DEFAULT_COUNT_DISCHARGED_PULSES 30
#define DEFAULT_TRACING_CHANGS 10
#define DEFAULT_POLLING_PERIPHERAL_TIME_MS 5000
#define TIMEOUT 20                                                       /* Таймут в мс функции HAL_ADC_PollForConversion --------------------------> TODO     */
#define COUNT_ADC_PERIPHERAL 3                                           /* Число каналов опроса переферии. Без учата канала главного фотодиода.               */

volatile uint16_t adc_value[DATA_SIZE];
volatile uint16_t adc_periphernal_value[COUNT_ADC_PERIPHERAL + 1];

volatile uint16_t count_traking_chanks = 0;                            /* Глобальный счетчик периодов слежения. Относится к conf.count_tracking_chanks.         */  



volatile uint8_t current_chank = 0;                                     /* Глобальный счетчик чанков.                                                           */    
volatile uint8_t isChaunksFull = 0;                                     /* Флаг окончания опроса current_chank == conf.current_chanks.                          */

/* Пременные настройки работы АЦП -------------------------------------------*/
uint8_t is_adc_flag_polling_peripheral = 0;
uint8_t number_peripheral_devices = COUNT_ADC_PERIPHERAL;                /* Число каналов опроса переферии. Без учата канала главного фотодиода.                */
uint8_t is_start_polling_periphery = 0;                                  /* Флаг запроса на опрос переифефрии.                                                  */
u_int8_t is_ready_flag_data_peripheral = 0;                              /* Флаг готовности опроса всекх переферийных устройств.                                */

/* Пременные настройки периода событий----------------------------------------*/
unsigned long t_requests;
unsigned long t_polling_peripheral;



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

  uint16_t response_period_ms;                                          /* 1 - Прериод отправки сообщей.                                                                 */
  uint8_t current_chanks;                                               /* 2 - Число чанков, один чанк 0,2 с, то есть время опросо кратно 0,2 с.                         */
  
  uint8_t discharged_mode;                                              /* 3 - Флаг мода режима работы лазера.
                                                                               TRUE - лазер будет включен  время `count_discharged_chanks/current_chanks * 0,2 с`
                                                                               FALSE - лазер включен всегда                                                               */
  uint8_t count_discharged_chanks;                                       /* 4 - Число чанков когда включен лазер, всегда   count_discharged_chanks <= current_chanks      */
  uint16_t count_discarge_pulses;                                        /* 5 - Отладочный коэффициент числа импульсов в 1 чанке,
                                                                                например, время сбора current_chanks = 4 чанка, count_discharged_chanks = 2, тогда общее
                                                                                чило частиц для принятия решения будет равно count_discharged_chanks*count_discarge_pulses.
                                                                              ------------------------------------------------------------------------------------->> TODO */

  uint8_t tracking_mod;                                                  /* 6 - Флаг мода режима слежения.                                                                 */
  uint16_t count_tracking_chanks;                                        /* 7 - Число периодов слежения. Время слежения равно count_tracking_chanks*current_chanks* 0,2 с. */  
  uint16_t polling_period_of_peripheral;                                 /* 8 - Период опроса перефирии.                                                                   */
}; 
struct configurations __conf_deault, conf = {                             
                            DEFAULT_REQUESTS_TIME_MS,                    /* 1 */
                            DEFAULT_CHAUNKS,                             /* 2 */         
                            TRUE,                                        /* 3 */     
                            DEFAULT_DISCARGED_CHAUNKS,                   /* 4 */   
                            DEFAULT_COUNT_DISCHARGED_PULSES,             /* 5 */   
                            TRUE,                                        /* 6 */   
                            DEFAULT_TRACING_CHANGS,                      /* 7 */
                            DEFAULT_POLLING_PERIPHERAL_TIME_MS,          /* 8 */
};                          


void response_copy(){
  ready_response.count_pulses = current_response.count_pulses;
  ready_response.flag_requests = FALSE;
  current_response.count_pulses = 0;
}


void clear_buff(){
  for (int i = 0; i < 500; i++){
    UserTxBufferFS[i] = 0;
  }
}


void send_message_virtual_com(int current){
  clear_buff();
  sprintf((char*)UserTxBufferFS, "info: count pulses detected: %u \r\n", current);
  CDC_Transmit_FS(UserTxBufferFS, sizeof(UserTxBufferFS)/sizeof UserTxBufferFS[0]);
  ready_response.flag_requests = TRUE;
}


void laser_off(){
  HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_SET);
}


void laser_on(){
  HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_RESET);
}

int get_signal_analysis(void){
  int count = 0;
  int count_pulses = 0;
  for (int i = 0; i < DATA_SIZE; i++){
    if (adc_value[i] > 4000){
      count++;
    }else{
      if (count > 0){
        count_pulses++;
      }
      count = 0;
    }
  }
  return count_pulses;
}


void discharge_mode(){
  /**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */

  if (current_chank == conf.count_discharged_chanks){
    count_traking_chanks++;
    int count = conf.count_discarge_pulses * conf.count_discharged_chanks;
    if (current_response.count_pulses >= count){

      if (conf.tracking_mod){
        count_traking_chanks = 0;
      }
      laser_on();
    }
    else{
      if (conf.tracking_mod){
        if (conf.count_tracking_chanks < count_traking_chanks){
          laser_off();  
        }
      }else{
        laser_off();
      }
    }
  }
}

void get_peripheral_data(){

  is_adc_flag_polling_peripheral = TRUE;                                               
  MX_ADC1_Init();

  HAL_ADC_Start(&hadc1);
  
  for (uint8_t i = 0; i <= number_peripheral_devices;  i++){
    if (HAL_ADC_PollForConversion(&hadc1, TIMEOUT) == HAL_OK)
    {
      adc_periphernal_value[i] = HAL_ADC_GetValue(&hadc1);
    }
  }

  is_ready_flag_data_peripheral = TRUE;

  HAL_ADC_Stop(&hadc1);
  is_adc_flag_polling_peripheral = FALSE;
  MX_ADC1_Init();
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  if (hadc->Instance == ADC1){
    HAL_ADC_Stop_DMA(&hadc1);
    current_chank++;
    current_response.count_pulses += get_signal_analysis();

    if (conf.discharged_mode){
      discharge_mode();
    }

    if (current_chank == conf.current_chanks){
      isChaunksFull = TRUE;
      current_chank = 0;

      if (is_start_polling_periphery){
        get_peripheral_data();
        is_start_polling_periphery = FALSE;
      }

      laser_on();

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100); 
  
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_value, DATA_SIZE);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  t_requests = HAL_GetTick();
  t_polling_peripheral = HAL_GetTick();

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


    if (HAL_GetTick() - t_requests >= conf.response_period_ms){
      t_requests = HAL_GetTick();

      int current = 0;
      if (ready_response.flag_requests){
        current = 0;
      }else{
        current = ready_response.count_pulses;
      }
      send_message_virtual_com(current);


      const char wmsg[] = "We love STM32!dskl";

      HAL_I2C_Master_Transmit(&hi2c1, adress_i2c, (uint8_t*)wmsg, sizeof(wmsg), 10000);

    }

    if (is_ready_flag_data_peripheral){
      float temp;
      int temper;

      temp = ((float)adc_periphernal_value[3]) / 4095 * 3300;
      temp = ((temp - 760.0) / 2.5) + 25;
      temper = temp*100;

      clear_buff();
      sprintf((char*)UserTxBufferFS, "debug: DATA EXTERNAL: %u %u %u \r\n", adc_periphernal_value[1], adc_periphernal_value[2],  temper);
      CDC_Transmit_FS(UserTxBufferFS, sizeof(UserTxBufferFS)/sizeof UserTxBufferFS[0]);
      is_ready_flag_data_peripheral = FALSE;
      
      HAL_Delay(100);
      clear_buff();
      sprintf((char*)UserTxBufferFS, "error: DATA ERROR TYPE: %u %u %u \r\n", adc_periphernal_value[1], adc_periphernal_value[2],  temper);
      CDC_Transmit_FS(UserTxBufferFS, sizeof(UserTxBufferFS)/sizeof UserTxBufferFS[0]);
      
      
      HAL_Delay(100);
      clear_buff();
      sprintf((char*)UserTxBufferFS, "exception: EXETTIONS DATA: %u %u\r\n", adc_periphernal_value[1], adc_periphernal_value[2]);
      CDC_Transmit_FS(UserTxBufferFS, sizeof(UserTxBufferFS)/sizeof UserTxBufferFS[0]);

    }

    if (HAL_GetTick() - t_polling_peripheral >= conf.polling_period_of_peripheral){
      t_polling_peripheral = HAL_GetTick();
      is_start_polling_periphery = TRUE;


      // HAL_I2C_Master_Transmit(&hi2c1, adress_i2c, &rx_buf, 1, 100);


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
