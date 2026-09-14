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
#include "monotonic_time.h"
#include "stm32_platform.h"
#include "debug_ui_config.h"
#if AETHOR_DEBUG_UI_ENABLE
#include "debug_ui_task.h"
#endif
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if AETHOR_DEBUG_UI_ENABLE
#define AETHOR_STATIC_TASK_COUNT (7U)
#else
#define AETHOR_STATIC_TASK_COUNT (6U)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static AethorMonotonicTimeState aethorMonotonicTimeState;
#if AETHOR_DEBUG_UI_ENABLE
static osThreadId DebugUiTaskHandle;
static uint32_t debugUiTaskBuffer[2048];
static osStaticThreadDef_t debugUiTaskControlBlock;
static uint32_t controlExecutionMaxUs;
static uint32_t controlPeriodMaxUs;
static uint32_t controlPreviousCycle;
static uint8_t controlCycleValid;
#endif
/* USER CODE END Variables */
osThreadId ArmControlTaskHandle;
uint32_t armControlTaskBuffer[ 1280 ];
osStaticThreadDef_t armControlTaskControlBlock;
osThreadId CanRxTaskHandle;
uint32_t canRxTaskBuffer[ 384 ];
osStaticThreadDef_t canRxTaskControlBlock;
osThreadId ProtocolTaskHandle;
uint32_t protocolTaskBuffer[ 1280 ];
osStaticThreadDef_t protocolTaskControlBlock;
osThreadId UsbTxTaskHandle;
uint32_t usbTxTaskBuffer[ 384 ];
osStaticThreadDef_t usbTxTaskControlBlock;
osThreadId TelemetryTaskHandle;
uint32_t telemetryTaskBuffer[ 1024 ];
osStaticThreadDef_t telemetryTaskControlBlock;
osThreadId DiagnosticsTaskHandle;
uint32_t diagnosticsTaskBuffer[ 384 ];
osStaticThreadDef_t diagnosticsTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void NotifyCanRxTaskFromIsr(void);
static void NotifyProtocolTaskFromIsr(void);
static void NotifyUsbTxTaskFromIsr(void);
static void QueueProtocolOutputBatch(const ProtocolOutputBatch *outputBatch);
static uint64_t AethorMonotonicTimestampUs(void);
static void EnterAethorAppTaskCritical(void);
static void ExitAethorAppTaskCritical(void);

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
  aethor_app_set_task_critical_hooks(EnterAethorAppTaskCritical,
                                     ExitAethorAppTaskCritical);
#if AETHOR_DEBUG_UI_ENABLE
  /* Internal DWT counting does not enable the PB3 SWO output pin. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif

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
  /* Result metadata extends cleanup depth: Keil reports >=3152 B before IRQ context. */
  osThreadStaticDef(ArmControlTask, StartArmControlTask, osPriorityRealtime, 0, 1280, armControlTaskBuffer, &armControlTaskControlBlock);
  ArmControlTaskHandle = osThreadCreate(osThread(ArmControlTask), NULL);

  /* definition and creation of CanRxTask */
  osThreadStaticDef(CanRxTask, StartCanRxTask, osPriorityHigh, 0, 384, canRxTaskBuffer, &canRxTaskControlBlock);
  CanRxTaskHandle = osThreadCreate(osThread(CanRxTask), NULL);

  /* definition and creation of ProtocolTask */
  osThreadStaticDef(ProtocolTask, StartProtocolTask, osPriorityAboveNormal, 0, 1280, protocolTaskBuffer, &protocolTaskControlBlock);
  ProtocolTaskHandle = osThreadCreate(osThread(ProtocolTask), NULL);

  /* definition and creation of UsbTxTask */
  osThreadStaticDef(UsbTxTask, StartUsbTxTask, osPriorityNormal, 0, 384, usbTxTaskBuffer, &usbTxTaskControlBlock);
  UsbTxTaskHandle = osThreadCreate(osThread(UsbTxTask), NULL);

  /* definition and creation of TelemetryTask */
  osThreadStaticDef(TelemetryTask, StartTelemetryTask, osPriorityBelowNormal, 0, 1024, telemetryTaskBuffer, &telemetryTaskControlBlock);
  TelemetryTaskHandle = osThreadCreate(osThread(TelemetryTask), NULL);

  /* definition and creation of DiagnosticsTask */
  osThreadStaticDef(DiagnosticsTask, StartDiagnosticsTask, osPriorityLow, 0, 384, diagnosticsTaskBuffer, &diagnosticsTaskControlBlock);
  DiagnosticsTaskHandle = osThreadCreate(osThread(DiagnosticsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
#if AETHOR_DEBUG_UI_ENABLE
  osThreadStaticDef(DebugUiTask, StartDebugUiTask, osPriorityLow, 0, 2048, debugUiTaskBuffer, &debugUiTaskControlBlock);
  DebugUiTaskHandle = osThreadCreate(osThread(DebugUiTask), NULL);
  configASSERT(DebugUiTaskHandle != NULL);
#endif
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
#if AETHOR_DEBUG_UI_ENABLE
    uint32_t controlStartCycle = DWT->CYCCNT;
    uint32_t cyclesPerMicrosecond = SystemCoreClock / 1000000U;
#endif
    uint64_t timestampUs = AethorMonotonicTimestampUs();
    CanFrame pendingFrame;
    CanFrame controlGroup[ARM_JOINT_COUNT];
    CanTxPriority pendingPriority;

    if ((aethor_app_service(timestampUs) != 0U) &&
        (ProtocolTaskHandle != NULL))
    {
      (void)xTaskNotifyGive((TaskHandle_t)ProtocolTaskHandle);
    }
    while (aethor_app_pop_emergency_can_frame(&pendingFrame) != 0U)
    {
      (void)stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY,
                                      &pendingFrame);
    }
    if (aethor_app_pop_control_group(controlGroup) != 0U)
    {
      CanTxSchedulerStatus controlGroupStatus =
          stm32_platform_can_submit_control_group(controlGroup,
                                                  ARM_JOINT_COUNT);

      if ((controlGroupStatus != CAN_TX_SCHEDULER_STATUS_OK) &&
          (aethor_app_report_control_group_failure(controlGroupStatus,
                                                   timestampUs) != 0U) &&
          (ProtocolTaskHandle != NULL))
      {
        (void)xTaskNotifyGive((TaskHandle_t)ProtocolTaskHandle);
      }
    }
    if (aethor_app_next_can_frame(timestampUs,
                                  &pendingFrame,
                                  &pendingPriority) ==
        MOTOR_RUNTIME_STATUS_FRAME_READY)
    {
      (void)stm32_platform_can_submit(pendingPriority, &pendingFrame);
    }
    (void)stm32_platform_can_service_tx(ARM_JOINT_COUNT);
#if AETHOR_DEBUG_UI_ENABLE
    if (cyclesPerMicrosecond != 0U)
    {
      uint32_t elapsedUs = (DWT->CYCCNT - controlStartCycle) /
          cyclesPerMicrosecond;
      if (elapsedUs > controlExecutionMaxUs)
      {
        controlExecutionMaxUs = elapsedUs;
      }
      if (controlCycleValid != 0U)
      {
        uint32_t periodUs = (controlStartCycle - controlPreviousCycle) /
            cyclesPerMicrosecond;
        if (periodUs > controlPeriodMaxUs)
        {
          controlPeriodMaxUs = periodUs;
        }
      }
      controlPreviousCycle = controlStartCycle;
      controlCycleValid = 1U;
    }
#endif
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
      uint64_t timestampUs = AethorMonotonicTimestampUs();

      taskENTER_CRITICAL();
      (void)aethor_app_receive_can_frame(&receivedFrame, timestampUs);
      taskEXIT_CRITICAL();
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
  char protocolLine[USB_CDC_STREAM_LINE_CAPACITY];
  ProtocolOutputBatch outputBatch;

  MX_USB_DEVICE_Init();
  (void)argument;
  for(;;)
  {
    UsbCdcStreamStatus lineStatus;
#if AETHOR_DEBUG_UI_ENABLE
    uint8_t protocolLinesProcessed = 0U;
#endif

    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (aethor_app_pop_protocol_result_output(&outputBatch) != 0U)
    {
      QueueProtocolOutputBatch(&outputBatch);
    }
#if AETHOR_DEBUG_UI_ENABLE
    aethor_app_debug_ui_process(AethorMonotonicTimestampUs());
#endif
    do
    {
      uint16_t lineLength = 0U;

      lineStatus = stm32_platform_usb_next_line(protocolLine,
                                                sizeof(protocolLine),
                                                &lineLength);
      if (lineStatus == USB_CDC_STREAM_STATUS_OK)
      {
        (void)aethor_app_process_protocol_line(
            protocolLine,
            lineLength,
            AethorMonotonicTimestampUs(),
            &outputBatch);
        QueueProtocolOutputBatch(&outputBatch);
      }
      else if (lineStatus == USB_CDC_STREAM_STATUS_LINE_TOO_LONG)
      {
        if (aethor_app_format_line_too_long(&outputBatch) ==
            PROTOCOL_ENGINE_STATUS_OK)
        {
          QueueProtocolOutputBatch(&outputBatch);
        }
      }
#if AETHOR_DEBUG_UI_ENABLE
      ++protocolLinesProcessed;
#endif
    } while (((lineStatus == USB_CDC_STREAM_STATUS_OK) ||
              (lineStatus == USB_CDC_STREAM_STATUS_LINE_TOO_LONG))
#if AETHOR_DEBUG_UI_ENABLE
             && (protocolLinesProcessed < 4U)
#endif
             );
    {
      uint64_t timestampUs = AethorMonotonicTimestampUs();

      if (aethor_app_generate_stream_output(timestampUs, &outputBatch) != 0U)
      {
        QueueProtocolOutputBatch(&outputBatch);
      }
    }
#if AETHOR_DEBUG_UI_ENABLE
    /* Keep the sole producer responsive even under continuous USB input. */
    aethor_app_debug_ui_process(AethorMonotonicTimestampUs());
    if (protocolLinesProcessed >= 4U)
    {
      (void)xTaskNotifyGive((TaskHandle_t)ProtocolTaskHandle);
      vTaskDelay(pdMS_TO_TICKS(1U));
    }
#endif
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
    if (ProtocolTaskHandle != NULL)
    {
      (void)xTaskNotifyGive((TaskHandle_t)ProtocolTaskHandle);
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10U));
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
  uint32_t previousCanRxOverflowCount = 0U;
  uint32_t previousCanBusOffCount = 0U;
  uint32_t previousUsbHighQueueFullCount = 0U;

  (void)argument;
  for(;;)
  {
    const Stm32PlatformDiagnostics *platformDiagnostics =
        stm32_platform_get_diagnostics();
    RuntimeDiagnosticSample runtimeSample;
    TaskHandle_t applicationTaskHandles[AETHOR_STATIC_TASK_COUNT];
    uint32_t minimumStackWords = UINT32_MAX;
    const uint32_t applicationStackWords[AETHOR_STATIC_TASK_COUNT] = {
      sizeof(armControlTaskBuffer) / sizeof(armControlTaskBuffer[0]),
      sizeof(canRxTaskBuffer) / sizeof(canRxTaskBuffer[0]),
      sizeof(protocolTaskBuffer) / sizeof(protocolTaskBuffer[0]),
      sizeof(usbTxTaskBuffer) / sizeof(usbTxTaskBuffer[0]),
      sizeof(telemetryTaskBuffer) / sizeof(telemetryTaskBuffer[0]),
      sizeof(diagnosticsTaskBuffer) / sizeof(diagnosticsTaskBuffer[0])
#if AETHOR_DEBUG_UI_ENABLE
      , sizeof(debugUiTaskBuffer) / sizeof(debugUiTaskBuffer[0])
#endif
    };
    uint8_t taskIndex;

    memset(&runtimeSample, 0, sizeof(runtimeSample));
    applicationTaskHandles[0] = (TaskHandle_t)ArmControlTaskHandle;
    applicationTaskHandles[1] = (TaskHandle_t)CanRxTaskHandle;
    applicationTaskHandles[2] = (TaskHandle_t)ProtocolTaskHandle;
    applicationTaskHandles[3] = (TaskHandle_t)UsbTxTaskHandle;
    applicationTaskHandles[4] = (TaskHandle_t)TelemetryTaskHandle;
    applicationTaskHandles[5] = (TaskHandle_t)DiagnosticsTaskHandle;
#if AETHOR_DEBUG_UI_ENABLE
    applicationTaskHandles[6] = (TaskHandle_t)DebugUiTaskHandle;
#endif
    for (taskIndex = 0U; taskIndex < AETHOR_STATIC_TASK_COUNT; ++taskIndex)
    {
      if (applicationTaskHandles[taskIndex] != NULL)
      {
        UBaseType_t stackWords = uxTaskGetStackHighWaterMark(
            applicationTaskHandles[taskIndex]);

        runtimeSample.task_stacks[taskIndex].free_words = (uint32_t)stackWords;
        runtimeSample.task_stacks[taskIndex].allocated_words = applicationStackWords[taskIndex];

        if ((uint32_t)stackWords < minimumStackWords)
        {
          minimumStackWords = (uint32_t)stackWords;
        }
      }
    }
    runtimeSample.can_rx_frames = platformDiagnostics->can_rx_frame_count;
    runtimeSample.can_tx_frames = platformDiagnostics->can_tx_success_count;
    runtimeSample.can_rx_overflow_count =
        platformDiagnostics->can_rx_overflow_count;
    runtimeSample.can_tx_error_count = platformDiagnostics->can_tx_error_count;
    runtimeSample.can_bus_off_count = platformDiagnostics->can_bus_off_count;
    runtimeSample.can_tx_queue_high_watermark =
        platformDiagnostics->can_tx_queue_high_watermark;
    runtimeSample.control_group_reject_count =
        platformDiagnostics->control_group_reject_count;
    runtimeSample.usb_rx_bytes = platformDiagnostics->usb_rx_byte_count;
    runtimeSample.usb_rx_overflow_count =
        platformDiagnostics->usb_rx_overflow_count;
    runtimeSample.usb_overlong_line_count =
        platformDiagnostics->usb_overlong_line_count;
    runtimeSample.usb_high_queue_high_watermark =
        platformDiagnostics->usb_high_queue_high_watermark;
    runtimeSample.usb_query_queue_high_watermark =
        platformDiagnostics->usb_query_queue_high_watermark;
    runtimeSample.usb_telemetry_queue_high_watermark =
        platformDiagnostics->usb_telemetry_queue_high_watermark;
    runtimeSample.usb_telemetry_drop_count =
        platformDiagnostics->usb_telemetry_drop_count;
    runtimeSample.usb_high_queue_full_count =
        platformDiagnostics->usb_high_queue_full_count;
    runtimeSample.usb_transmit_busy_count =
        platformDiagnostics->usb_transmit_busy_count;
    runtimeSample.usb_transmit_error_count =
        platformDiagnostics->usb_transmit_error_count;
    runtimeSample.control_group_skew_max_us =
        DIAGNOSTIC_WATERMARK_NOT_SAMPLED;
    runtimeSample.minimum_stack_words = minimumStackWords;
    runtimeSample.minimum_heap_bytes = (uint32_t)xPortGetFreeHeapSize();
    aethor_app_update_runtime_diagnostics(&runtimeSample);
    {
      uint32_t transportFaultDetail = 0U;

      if (platformDiagnostics->can_rx_overflow_count >
          previousCanRxOverflowCount)
      {
        transportFaultDetail |= 1U;
      }
      if (platformDiagnostics->can_bus_off_count > previousCanBusOffCount)
      {
        transportFaultDetail |= 2U;
      }
      if (platformDiagnostics->usb_high_queue_full_count >
          previousUsbHighQueueFullCount)
      {
        transportFaultDetail |= 4U;
      }
      previousCanRxOverflowCount =
          platformDiagnostics->can_rx_overflow_count;
      previousCanBusOffCount = platformDiagnostics->can_bus_off_count;
      previousUsbHighQueueFullCount =
          platformDiagnostics->usb_high_queue_full_count;
      if ((transportFaultDetail != 0U) &&
           (aethor_app_report_transport_fault(
                transportFaultDetail,
                AethorMonotonicTimestampUs()) != 0U) &&
          (ArmControlTaskHandle != NULL))
      {
        (void)xTaskNotifyGive((TaskHandle_t)ArmControlTaskHandle);
      }
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100U));
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#if AETHOR_DEBUG_UI_ENABLE
/** @brief Wakes the sole command producer from the low-priority UI task. */
void AethorNotifyProtocolTask(void)
{
  if (ProtocolTaskHandle != NULL)
  {
    (void)xTaskNotifyGive((TaskHandle_t)ProtocolTaskHandle);
  }
}

/** @brief Returns the same monotonic clock used for command admission/control. */
uint64_t AethorUiTimestampUs(void)
{
  return AethorMonotonicTimestampUs();
}

/** @brief Copies DWT timing watermarks; no board measurements are fabricated. */
void AethorGetControlTiming(uint32_t *execution_max_us,
                            uint32_t *period_max_us)
{
  taskENTER_CRITICAL();
  if (execution_max_us != NULL)
  {
    *execution_max_us = controlExecutionMaxUs;
  }
  if (period_max_us != NULL)
  {
    *period_max_us = controlPeriodMaxUs;
  }
  taskEXIT_CRITICAL();
}
#endif

/** @brief Enters the scheduler boundary shared by app query readers. */
static void EnterAethorAppTaskCritical(void)
{
  taskENTER_CRITICAL();
}

/** @brief Exits the scheduler boundary shared by app query readers. */
static void ExitAethorAppTaskCritical(void)
{
  taskEXIT_CRITICAL();
}

/**
 * @brief Returns one shared 64-bit timestamp extended from the wrapping HAL tick.
 * @return Monotonic timestamp in microseconds.
 */
static uint64_t AethorMonotonicTimestampUs(void)
{
  uint64_t timestampUs;

  taskENTER_CRITICAL();
  timestampUs = aethor_monotonic_time_update(&aethorMonotonicTimeState,
                                              HAL_GetTick());
  taskEXIT_CRITICAL();
  return timestampUs;
}

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

/** @brief Routes encoded protocol outputs into their bounded USB priority queues. */
static void QueueProtocolOutputBatch(const ProtocolOutputBatch *outputBatch)
{
  uint8_t messageIndex;

  configASSERT(outputBatch != NULL);
  for (messageIndex = 0U; messageIndex < outputBatch->count; ++messageIndex)
  {
    const ProtocolOutputMessage *message = &outputBatch->messages[messageIndex];

    taskENTER_CRITICAL();
    if (message->priority == PROTOCOL_OUTPUT_HIGH_PRIORITY)
    {
      (void)stm32_platform_usb_queue_high_priority(
          (const uint8_t *)message->data,
          message->length);
    }
    else if (message->priority == PROTOCOL_OUTPUT_QUERY)
    {
      (void)stm32_platform_usb_queue_query((const uint8_t *)message->data,
                                           message->length);
    }
    else
    {
      (void)stm32_platform_usb_queue_telemetry(
          (const uint8_t *)message->data,
          message->length);
    }
    taskEXIT_CRITICAL();
  }
  if (UsbTxTaskHandle != NULL)
  {
    (void)xTaskNotifyGive((TaskHandle_t)UsbTxTaskHandle);
  }
}

/* USER CODE END Application */
