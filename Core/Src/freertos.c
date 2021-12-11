/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "nrf24l01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t res;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for nRF24L01 */
osThreadId_t nRF24L01Handle;
const osThreadAttr_t nRF24L01_attributes = {
  .name = "nRF24L01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start_nRF24L01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of nRF24L01 */
  nRF24L01Handle = osThreadNew(Start_nRF24L01, NULL, &nRF24L01_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET); // switch on
    osDelay(1000);
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET); // switch off
	osDelay(1000);
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET); // switch on
  /* Infinite loop */
  for(;;)
  {
	osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_nRF24L01 */
/**
* @brief Function implementing the nRF24L01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_nRF24L01 */
void Start_nRF24L01(void *argument)
{
  /* USER CODE BEGIN Start_nRF24L01 */
  const uint64_t pipe1 = 0xE8E8F0F0E2LL; // адрес первой трубы
  res = isChipConnected(); // проверяет подключён ли модуль к SPI

  char str[64] = {0,};
  snprintf(str, 64, "Connected: %s\n", 1 ? "OK" : "NOT OK");
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  res = NRF_Init(); // инициализация

  snprintf(str, 64, "Init: %s\n", res > 0 && res < 255 ? "OK" : "NOT OK");
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  ////////////// SET ////////////////
  enableAckPayload();
  //setAutoAck(false);
  //setPayloadSize(3);
  setChannel(19);
  
  
  openReadingPipe(1, pipe1);
  startListening();
 
  
  ///////////////////////////////////

  ////////////////////////// Вывод всяких статусов, для работы не нужно /////////////////////////////
  uint8_t status = get_status();
  snprintf(str, 64, "get_status: 0x%02x\n", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  status = getPALevel();
  snprintf(str, 64, "getPALevel: 0x%02x  ", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x00)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_MIN\n", strlen("RF24_PA_MIN\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_LOW\n", strlen("RF24_PA_LOW\n"), 1000);
  }
  else if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_HIGH\n", strlen("RF24_PA_HIGH\n"), 1000);
  }
  else if(status == 0x03)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_PA_MAX\n", strlen("RF24_PA_MAX\n"), 1000);
  }

  status = getChannel();
  snprintf(str, 64, "getChannel: 0x%02x № %d\n", status, status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  status = getDataRate();
  snprintf(str, 64, "getDataRate: 0x%02x  ", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_250KBPS\n", strlen("RF24_250KBPS\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_2MBPS\n", strlen("RF24_2MBPS\n"), 1000);
  }
  else
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_1MBPS\n", strlen("RF24_1MBPS\n"), 1000);
  }

  status = getPayloadSize();
  snprintf(str, 64, "getPayloadSize: %d\n", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  status = getCRCLength();
  snprintf(str, 64, "getCRCLength: 0x%02x  ", status);
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x00)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_CRC_DISABLED\n", strlen("RF24_CRC_DISABLED\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_CRC_8\n", strlen("RF24_CRC_8\n"), 1000);
  }
  else if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RF24_CRC_16\n", strlen("RF24_CRC_16\n"), 1000);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////


  maskIRQ(true, true, true); // маскируем прерывания
  /* Infinite loop */
  for(;;)
  {
	///////////////////////////////////// ПРИЁМ /////////////////////////////////////////////
	uint8_t nrf_data[32] = {0,}; // буфер указываем максимального размера
	//static uint8_t remsg = 0;
	char remsg[] = "Hello Zhan! How are you?";
	uint8_t pipe_num = 0;
	
	if(available(&pipe_num)) // проверяем пришло ли что-то
	{
		//remsg++;

		writeAckPayload(pipe_num, &remsg, sizeof(remsg)); // отправляем полезную нагрузку вместе с подтверждением

		if(pipe_num == 0) // проверяем в какую трубу пришли данные
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"pipe 0\n", strlen("pipe 0\n"), 1000);
		}

		else if(pipe_num == 1)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"pipe 1\n", strlen("pipe 1\n"), 1000);

			uint8_t count = getDynamicPayloadSize(); // смотрим сколько байт прилетело

			read(&nrf_data, count); // Читаем данные в массив nrf_data и указываем сколько байт читать
                        //writeAckPayload(pipe_num, &nrf_data[0], sizeof(&nrf_data[0])); // отправляем полезную нагрузку вместе с подтверждением
			if(nrf_data[0] == 77 && nrf_data[1] == 86 && nrf_data[2] == 97) // проверяем правильность данных
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				snprintf(str, 64, "data[0]=%d data[1]=%d data[2]=%d\n", nrf_data[0], nrf_data[1], nrf_data[2]);
				HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);
			}
		}

		else if(pipe_num == 2)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"pipe 2\n", strlen("pipe 2\n"), 1000);
		}

		else
		{
			while(availableMy()) // если данные придут от неуказанной трубы, то попадут сюда
			{
				read(&nrf_data, sizeof(nrf_data));
				HAL_UART_Transmit(&huart2, (uint8_t*)"Unknown pipe\n", strlen("Unknown pipe\n"), 1000);
			}
		}
	}
    osDelay(1);
  }
  /* USER CODE END Start_nRF24L01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
