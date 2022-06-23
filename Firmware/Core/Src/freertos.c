/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "MQTTSim800.h"
#include "event_groups.h"
#include "queue.h"
#include "l70.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// #define BIT_SEND_SMS        (1 << 0)
#define BIT_RECEIVE_GPS     (1 << 1)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern SIM800_t SIM800;
char sim_number[15] = {0};
EventGroupHandle_t event_group;
EventBits_t event_bits;
QueueHandle_t latitude_queue, longitude_queue;
TaskHandle_t sms_handler, gps_handler, tracking_gps_handler, init_sim_handler;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  latitude_queue = xQueueCreate(20, sizeof(uint8_t));
  longitude_queue = xQueueCreate(20, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(vTaskSendSMS, "vTaskSendSMS", 128, NULL, 5, sms_handler);
  xTaskCreate(vTaskReceiveGPS, "vTaskReceiveGPS", 256, NULL, 5, gps_handler);
  xTaskCreate(vTaskTrackingGPS, "vTaskTrackingGPS", 128, NULL, 5, tracking_gps_handler);
  // xTaskCreate(vTaskInitSim, "vTaskInitSim", 128, NULL, 10, init_sim_handler);
  event_group = xEventGroupCreate(); 
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
// void StartDefaultTask(void *argument)
// {
//   /* USER CODE BEGIN StartDefaultTask */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END StartDefaultTask */
// }

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vTaskSendSMS(void *argument)
{
  char smsATCommand[50] = {0};
  char endSMS[2] = {0};
  endSMS[0] = 0x1A;
  endSMS[1] = '\0';
  char latData[15] = {0};
  char longData[15] = {0};
  char simMessage[100] = {0};
  while(HAL_GPIO_ReadPin(GSM_STATUS_GPIO_Port, GSM_STATUS_Pin) == GPIO_PIN_RESET)
  {
    HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, SET);
    osDelay(1000);
  }
  // MQTT_Init();
  sim_init();
  sprintf(smsATCommand, "AT+CMGS=\"%s\"\r\n", sim_number);
  for(;;)
  {
    if(SIM800.sms == true)
    {
      xEventGroupSetBits(event_group, BIT_RECEIVE_GPS);
      vTaskSuspend(gps_handler);
      SIM800_SendCommand("AT+CMGF=1\r\n", "OK\r\n", CMD_DELAY);
      SIM800_SendCommand(smsATCommand, ">", CMD_DELAY);
      xQueueReceive(latitude_queue, (void*)latData, (TickType_t)1);
      xQueueReceive(longitude_queue, (void*)longData, (TickType_t)1);
      sprintf(simMessage, "https://www.google.com/maps/search/?api=1&query=%s,%s\n", latData, longData);
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)simMessage, strlen(simMessage));
      vTaskDelay(1);
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)endSMS, 1);
      SIM800.sms == false;
    }
  }
}

void vTaskReceiveGPS(void *argument)
{
  char latData[15] = {0};
  char longData[15] = {0};
  char *gps_response;
  // while(HAL_GPIO_ReadPin(GSM_STATUS_GPIO_Port, GSM_STATUS_Pin) == GPIO_PIN_RESET)
  // {
  //   HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, SET);
  //   vTaskDelay(1000);
  // }
  sim_init();
  vTaskDelay(100);
  l70_init();
  for(;;)
  {
    event_bits = xEventGroupWaitBits(event_group, BIT_RECEIVE_GPS, pdTRUE, pdFALSE, (TickType_t)1);
    if(event_bits & BIT_RECEIVE_GPS)
    {
      gps_response = l70_receiveGPS();
      l70_handleGPS(latData, longData, gps_response);
      xQueueSend(latitude_queue, (void*)&latData, (TickType_t)1);
      xQueueSend(longitude_queue, (void*)&longData, (TickType_t)1);
      vTaskResume(gps_handler);
    }
  }
}

void vTaskTrackingGPS(void *argument)
{
  char latData[15] = {0};
  char longData[15] = {0};
  char *gps_response;
  for(;;)
  {
    if(SIM800.tracking == true)
    {
      
    }
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
