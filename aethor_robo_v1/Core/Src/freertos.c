/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "aethor_app.h"
#include "stm32_platform.h"
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
/* USER CODE END Variables */
osThreadId ArmControlTaskHandle;
uint32_t armControlTaskBuffer[ 512 ];
osStaticThreadDef_t armControlTaskControlBlock;
osThreadId CanRxTaskHandle;
uint32_t canRxTaskBuffer[ 384 ];
osStaticThreadDef_t canRxTaskControlBlock;
osThreadId ProtocolTaskHandle;
uint32_t protocolTaskBuffer[ 512 ];
osStaticThreadDef_t protocolTaskControlBlock;
osThreadId UsbTxTaskHandle;
uint32_t usbTxTaskBuffer[ 384 ];
osStaticThreadDef_t usbTxTaskControlBlock;
osThreadId TelemetryTaskHandle;
uint32_t telemetryTaskBuffer[ 384 ];
osStaticThreadDef_t telemetryTaskControlBlock;
osThreadId DiagnosticsTaskHandle;
uint32_t diagnosticsTaskBuffer[ 384 ];
osStaticThreadDef_t diagnosticsTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void NotifyCanRxTaskFromIsr(void);
static void NotifyProtocolTaskFromIsr(void);
static void NotifyUsbTxTaskFromIsr(void);

/* USER CODE END FunctionPrototypes */

void StartArmControlTask(void const * argument);
void StartCanRxTask(void const * argument);
void StartProtocolTask(void const * argument);
void StartUsbTxTask(void const * argument);
void StartTelemetryTask(void const * argument);
void StartDiagnosticsTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of ArmControlTask */
  osThreadStaticDef(ArmControlTask, StartArmControlTask, osPriorityRealtime, 0, 512, armControlTaskBuffer, &armControlTaskControlBlock);
  ArmControlTaskHandle = osThreadCreate(osThread(ArmControlTask), NULL);

  /* definition and creation of CanRxTask */
  osThreadStaticDef(CanRxTask, StartCanRxTask, osPriorityHigh, 0, 384, canRxTaskBuffer, &canRxTaskControlBlock);
  CanRxTaskHandle = osThreadCreate(osThread(CanRxTask), NULL);

  /* definition and creation of ProtocolTask */
  osThreadStaticDef(ProtocolTask, StartProtocolTask, osPriorityAboveNormal, 0, 512, protocolTaskBuffer, &protocolTaskControlBlock);
  ProtocolTaskHandle = osThreadCreate(osThread(ProtocolTask), NULL);

  /* definition and creation of UsbTxTask */
  osThreadStaticDef(UsbTxTask, StartUsbTxTask, osPriorityNormal, 0, 384, usbTxTaskBuffer, &usbTxTaskControlBlock);
  UsbTxTaskHandle = osThreadCreate(osThread(UsbTxTask), NULL);

  /* definition and creation of TelemetryTask */
  osThreadStaticDef(TelemetryTask, StartTelemetryTask, osPriorityBelowNormal, 0, 384, telemetryTaskBuffer, &telemetryTaskControlBlock);
  TelemetryTaskHandle = osThreadCreate(osThread(TelemetryTask), NULL);

  /* definition and creation of DiagnosticsTask */
  osThreadStaticDef(DiagnosticsTask, StartDiagnosticsTask, osPriorityLow, 0, 384, diagnosticsTaskBuffer, &diagnosticsTaskControlBlock);
  DiagnosticsTaskHandle = osThreadCreate(osThread(DiagnosticsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  configASSERT(ArmControlTaskHandle != NULL);
  configASSERT(CanRxTaskHandle != NULL);
  configASSERT(ProtocolTaskHandle != NULL);
  configASSERT(UsbTxTaskHandle != NULL);
  configASSERT(TelemetryTaskHandle != NULL);
  configASSERT(DiagnosticsTaskHandle != NULL);
  stm32_platform_set_isr_notifiers(NotifyCanRxTaskFromIsr,
                                   NotifyProtocolTaskFromIsr,
                                   NotifyUsbTxTaskFromIsr);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartArmControlTask */
/**
  * @brief  Runs the 4 ms absolute-period arm control service.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartArmControlTask */
void StartArmControlTask(void const * argument)
{
  /* USER CODE BEGIN StartArmControlTask */
  TickType_t lastWakeTime = xTaskGetTickCount();

  (void)argument;
  for(;;)
  {
    uint64_t timestampUs = (uint64_t)HAL_GetTick() * 1000ULL;
    CanFrame pendingFrame;
    CanTxPriority pendingPriority;

    aethor_app_service(timestampUs);
    if (aethor_app_next_can_frame(timestampUs,
                                  &pendingFrame,
                                  &pendingPriority) ==
        MOTOR_RUNTIME_STATUS_FRAME_READY)
    {
      (void)stm32_platform_can_submit(pendingPriority, &pendingFrame);
    }
    (void)stm32_platform_can_service_tx(ARM_JOINT_COUNT);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(4U));
  }
  /* USER CODE END StartArmControlTask */
}

/* USER CODE BEGIN Header_StartCanRxTask */
/**
  * @brief  Waits for FDCAN RX notification before bounded task-context decode.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCanRxTask */
void StartCanRxTask(void const * argument)
{
  (void)argument;
  for(;;)
  {
    uint8_t processedFrameCount = 0U;
    CanFrame receivedFrame;

    while ((processedFrameCount < CAN_RX_INBOX_CAPACITY) &&
           (stm32_platform_can_pop_received(&receivedFrame) ==
            CAN_RX_INBOX_STATUS_OK))
    {
      (void)aethor_app_receive_can_frame(
          &receivedFrame,
          (uint64_t)HAL_GetTick() * 1000ULL);
      ++processedFrameCount;
    }
    if (processedFrameCount < CAN_RX_INBOX_CAPACITY)
    {
      (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    else
    {
      taskYIELD();
    }
  }
}

/* USER CODE BEGIN Header_StartProtocolTask */
/**
  * @brief  Owns USB startup and waits for complete protocol input work.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProtocolTask */
void StartProtocolTask(void const * argument)
{
  MX_USB_DEVICE_Init();
  (void)argument;
  for(;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    /* Protocol dispatch is connected in the lifecycle slice. */
  }
}

/* USER CODE BEGIN Header_StartUsbTxTask */
/**
  * @brief  Services priority-ordered USB CDC output without blocking control.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUsbTxTask */
void StartUsbTxTask(void const * argument)
{
  (void)argument;
  for(;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
    stm32_platform_usb_service_tx();
  }
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
  * @brief  Reserves the default 50 Hz telemetry publication cadence.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void const * argument)
{
  TickType_t lastWakeTime = xTaskGetTickCount();

  (void)argument;
  for(;;)
  {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20U));
  }
}

/* USER CODE BEGIN Header_StartDiagnosticsTask */
/**
  * @brief  Reserves the 100 ms diagnostics aggregation cadence.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDiagnosticsTask */
void StartDiagnosticsTask(void const * argument)
{
  TickType_t lastWakeTime = xTaskGetTickCount();

  (void)argument;
  for(;;)
  {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100U));
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/** @brief Wakes CanRxTask from the FDCAN receive callback. */
static void NotifyCanRxTaskFromIsr(void)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (CanRxTaskHandle != NULL)
  {
    vTaskNotifyGiveFromISR((TaskHandle_t)CanRxTaskHandle, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

/** @brief Wakes ProtocolTask from the USB CDC receive callback. */
static void NotifyProtocolTaskFromIsr(void)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (ProtocolTaskHandle != NULL)
  {
    vTaskNotifyGiveFromISR((TaskHandle_t)ProtocolTaskHandle, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

/** @brief Wakes UsbTxTask when the CDC endpoint releases its active buffer. */
static void NotifyUsbTxTaskFromIsr(void)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (UsbTxTaskHandle != NULL)
  {
    vTaskNotifyGiveFromISR((TaskHandle_t)UsbTxTaskHandle, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

/* USER CODE END Application */
