/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   Deterministic three-task washer inspection control loop
  ******************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"
#include "inspection_protocol.h"
#include "usart.h"
#include "watchdog.h"

#define SENSOR_ACTIVE_LEVEL          GPIO_PIN_RESET
#define BELT_RUN_LEVEL               GPIO_PIN_SET
#define BELT_STOP_LEVEL              GPIO_PIN_RESET
#define SENSOR_DEBOUNCE_MS           5U
#define SENSOR_POLL_MS               2U
#define CYLINDER_PULSE_MS            120U
#define UART_BYTE_TIMEOUT_MS         4U
#define UART_RETRY_AT_MS             195U
/* Retry only after avg E2E latency (195 ms) has elapsed; remaining 105 ms
   within the 300 ms budget is a best-effort window for the retry. */
#define UART_TRANSACTION_BUDGET_MS   400U
#define EXECUTION_WAIT_MS            (UART_TRANSACTION_BUDGET_MS + 50U)
/* Supervision and liveness parameters. */
#define COMM_IDLE_POLL_MS            10U
#define MCU_HEARTBEAT_PERIOD_MS      1000U
#define VISION_ALIVE_TIMEOUT_MS      3000U
#define TASK_MONITOR_PERIOD_MS       100U
#define TASK_MONITOR_STRIKE_LIMIT    5U
#define EXECUTE_POLL_MS              50U

typedef struct
{
    uint8_t code;
    uint8_t confidence;
    uint8_t valid;
} InspectionDecision;

typedef struct
{
    uint32_t sensor_triggers;
    uint32_t accepted_parts;
    uint32_t rejected_parts;
    uint32_t fail_safe_rejects;
    uint32_t uart_retries;
    uint32_t uart_timeouts;
    uint32_t uart_errors;
    uint32_t stale_frames;
    uint32_t protocol_errors;
    uint32_t max_transaction_ms;
    uint32_t heartbeats_sent;
    uint32_t heartbeats_received;
    uint32_t vision_lost_rejects;
} InspectionMetrics;

static osSemaphoreId_t inspectionStartSem;
static osSemaphoreId_t communicationStartSem;
static osSemaphoreId_t communicationDoneSem;

static StaticSemaphore_t inspectionStartSemCb;
static StaticSemaphore_t communicationStartSemCb;
static StaticSemaphore_t communicationDoneSemCb;

static StaticTask_t detectTaskCb;
static StaticTask_t executeTaskCb;
static StaticTask_t communicationTaskCb;
static StaticTask_t monitorTaskCb;
static StackType_t detectTaskStack[128];
static StackType_t executeTaskStack[192];
static StackType_t communicationTaskStack[224];
static StackType_t monitorTaskStack[96];

static osThreadId_t detectTaskHandle;
static osThreadId_t executeTaskHandle;
static osThreadId_t communicationTaskHandle;

static volatile uint8_t activeSequence;
static InspectionDecision sharedDecision;
static volatile InspectionMetrics metrics;

/* Liveness counters incremented once per iteration by each supervised
   task; the Monitor task compares them and services the IWDG only while
   all three keep advancing. */
static volatile uint32_t detectAliveCounter;
static volatile uint32_t executeAliveCounter;
static volatile uint32_t communicationAliveCounter;
static volatile uint32_t visionLastSeenAt;

static void DetectTask(void *argument);
static void ExecuteTask(void *argument);
static void CommunicationTask(void *argument);
static void MonitorTask(void *argument);
static bool CommunicationExchange(uint8_t sequence, InspectionDecision *decision);
static void CommunicationIdleService(void);
static bool CommunicationIsVisionAlive(void);
static void BeltSetRunning(bool running);
static void FailSafeHalt(void);

void MX_FREERTOS_Init(void)
{
    static const osSemaphoreAttr_t inspectionStartSemAttr = {
        .name = "inspectionStart",
        .cb_mem = &inspectionStartSemCb,
        .cb_size = sizeof(inspectionStartSemCb)
    };
    static const osSemaphoreAttr_t communicationStartSemAttr = {
        .name = "communicationStart",
        .cb_mem = &communicationStartSemCb,
        .cb_size = sizeof(communicationStartSemCb)
    };
    static const osSemaphoreAttr_t communicationDoneSemAttr = {
        .name = "communicationDone",
        .cb_mem = &communicationDoneSemCb,
        .cb_size = sizeof(communicationDoneSemCb)
    };
    static const osThreadAttr_t detectTaskAttr = {
        .name = "Detect",
        .cb_mem = &detectTaskCb,
        .cb_size = sizeof(detectTaskCb),
        .stack_mem = detectTaskStack,
        .stack_size = sizeof(detectTaskStack),
        .priority = osPriorityNormal
    };
    static const osThreadAttr_t executeTaskAttr = {
        .name = "Execute",
        .cb_mem = &executeTaskCb,
        .cb_size = sizeof(executeTaskCb),
        .stack_mem = executeTaskStack,
        .stack_size = sizeof(executeTaskStack),
        .priority = osPriorityHigh
    };
    static const osThreadAttr_t communicationTaskAttr = {
        .name = "Communicate",
        .cb_mem = &communicationTaskCb,
        .cb_size = sizeof(communicationTaskCb),
        .stack_mem = communicationTaskStack,
        .stack_size = sizeof(communicationTaskStack),
        .priority = osPriorityAboveNormal
    };
    static const osThreadAttr_t monitorTaskAttr = {
        .name = "Monitor",
        .cb_mem = &monitorTaskCb,
        .cb_size = sizeof(monitorTaskCb),
        .stack_mem = monitorTaskStack,
        .stack_size = sizeof(monitorTaskStack),
        .priority = osPriorityRealtime
    };

    inspectionStartSem = osSemaphoreNew(1U, 0U, &inspectionStartSemAttr);
    communicationStartSem = osSemaphoreNew(1U, 0U, &communicationStartSemAttr);
    communicationDoneSem = osSemaphoreNew(1U, 0U, &communicationDoneSemAttr);

    detectTaskHandle = osThreadNew(DetectTask, NULL, &detectTaskAttr);
    executeTaskHandle = osThreadNew(ExecuteTask, NULL, &executeTaskAttr);
    communicationTaskHandle = osThreadNew(CommunicationTask, NULL,
                                          &communicationTaskAttr);

    if ((inspectionStartSem == NULL) ||
        (communicationStartSem == NULL) ||
        (communicationDoneSem == NULL) ||
        (detectTaskHandle == NULL) ||
        (executeTaskHandle == NULL) ||
        (communicationTaskHandle == NULL) ||
        (osThreadNew(MonitorTask, NULL, &monitorTaskAttr) == NULL))
    {
        Error_Handler();
    }
}

static void BeltSetRunning(bool running)
{
    HAL_GPIO_WritePin(ENA_GPIO_Port,
                      ENA_Pin,
                      running ? BELT_RUN_LEVEL : BELT_STOP_LEVEL);
}

static void DetectTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        ++detectAliveCounter;
        if (HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin) == SENSOR_ACTIVE_LEVEL)
        {
            osDelay(SENSOR_DEBOUNCE_MS);
            if (HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin) == SENSOR_ACTIVE_LEVEL)
            {
                ++metrics.sensor_triggers;
                (void)osSemaphoreRelease(inspectionStartSem);

                while (HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin) ==
                       SENSOR_ACTIVE_LEVEL)
                {
                    osDelay(SENSOR_POLL_MS);
                }
            }
        }
        osDelay(SENSOR_POLL_MS);
    }
}

static void ExecuteTask(void *argument)
{
    uint8_t nextSequence = 0U;
    osStatus_t waitStatus;

    (void)argument;
    BeltSetRunning(true);

    for (;;)
    {
        ++executeAliveCounter;
        waitStatus = osSemaphoreAcquire(inspectionStartSem, EXECUTE_POLL_MS);
        if (waitStatus != osOK)
        {
            continue;
        }
        BeltSetRunning(false);

        ++nextSequence;
        activeSequence = nextSequence;
        sharedDecision.code = INSPECTION_RESULT_VISION_ERROR;
        sharedDecision.confidence = 0U;
        sharedDecision.valid = 0U;

        /*
         * The heartbeat watchdog already knows the vision link is dead:
         * reject immediately instead of spending the full transaction
         * budget.  An uninspected part must never pass the line.
         */
        if (!CommunicationIsVisionAlive())
        {
            ++metrics.fail_safe_rejects;
            ++metrics.rejected_parts;
            ++metrics.vision_lost_rejects;
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_SET);
            osDelay(CYLINDER_PULSE_MS);
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_RESET);
            BeltSetRunning(true);
            continue;
        }

        (void)osSemaphoreRelease(communicationStartSem);
        waitStatus = osSemaphoreAcquire(communicationDoneSem, EXECUTION_WAIT_MS);

        /*
         * Fail-safe policy: a timeout, corrupt response or vision failure is
         * rejected.  An uninspected part must never silently pass the line.
         */
        if ((waitStatus != osOK) ||
            (sharedDecision.valid == 0U) ||
            (sharedDecision.code != INSPECTION_RESULT_NORMAL))
        {
            if ((waitStatus != osOK) ||
                (sharedDecision.valid == 0U) ||
                (sharedDecision.code >= INSPECTION_RESULT_VISION_ERROR))
            {
                ++metrics.fail_safe_rejects;
            }
            ++metrics.rejected_parts;
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_SET);
            osDelay(CYLINDER_PULSE_MS);
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_RESET);
        }
        else
        {
            ++metrics.accepted_parts;
        }

        BeltSetRunning(true);
    }
}

static void CommunicationTask(void *argument)
{
    InspectionDecision decision;

    (void)argument;
    /* Scheduler is running here, so the ISR may safely use FreeRTOS
       FromISR APIs.  Enable interrupt-driven RX before the first exchange. */
    UART1_IsrStart();
    for (;;)
    {
        ++communicationAliveCounter;
        if (osSemaphoreAcquire(communicationStartSem, COMM_IDLE_POLL_MS) != osOK)
        {
            /* Between parts: service heartbeats and link supervision. */
            CommunicationIdleService();
            continue;
        }
        decision.code = INSPECTION_RESULT_VISION_ERROR;
        decision.confidence = 0U;
        decision.valid = CommunicationExchange(activeSequence, &decision) ? 1U : 0U;
        sharedDecision = decision;
        metrics.uart_errors += UART1_ConsumeErrorCount();
        (void)osSemaphoreRelease(communicationDoneSem);
    }
}

static void CommunicationIdleService(void)
{
    static InspectionParser idleParser;
    static uint8_t heartbeatSequence;
    static uint32_t lastHeartbeatAt;
    static bool idleParserReady;
    InspectionFrame frame;
    uint8_t receivedByte;
    uint8_t heartbeatFrame[INSPECTION_FRAME_SIZE];
    uint8_t acknowledgeFrame[INSPECTION_FRAME_SIZE];

    if (!idleParserReady)
    {
        InspectionProtocol_ParserInit(&idleParser);
        idleParserReady = true;
    }

    while (UART1_IsrReadByte(&receivedByte, 0U))
    {
        if (InspectionProtocol_PushByte(&idleParser, receivedByte, &frame))
        {
            /* Any valid frame from the vision service proves the link. */
            visionLastSeenAt = HAL_GetTick();
            if (frame.type == INSPECTION_MSG_HEARTBEAT)
            {
                ++metrics.heartbeats_received;
                InspectionProtocol_Build(INSPECTION_MSG_ACK,
                                         frame.sequence,
                                         INSPECTION_RESULT_NORMAL,
                                         0U,
                                         acknowledgeFrame);
                if (!UART1_IsrSend(acknowledgeFrame, INSPECTION_FRAME_SIZE))
                {
                    ++metrics.uart_errors;
                }
            }
            else if (frame.type != INSPECTION_MSG_ACK)
            {
                ++metrics.stale_frames;
            }
        }
    }

    /* Announce the controller while the line is idle so the vision
       service can tell the MCU is alive too. */
    if ((HAL_GetTick() - lastHeartbeatAt) >= MCU_HEARTBEAT_PERIOD_MS)
    {
        lastHeartbeatAt = HAL_GetTick();
        ++heartbeatSequence;
        InspectionProtocol_Build(INSPECTION_MSG_HEARTBEAT,
                                 heartbeatSequence,
                                 INSPECTION_RESULT_NORMAL,
                                 0U,
                                 heartbeatFrame);
        if (UART1_IsrSend(heartbeatFrame, INSPECTION_FRAME_SIZE))
        {
            ++metrics.heartbeats_sent;
        }
        else
        {
            ++metrics.uart_errors;
        }
    }
}

static bool CommunicationIsVisionAlive(void)
{
    return ((HAL_GetTick() - visionLastSeenAt) < VISION_ALIVE_TIMEOUT_MS);
}

static void MonitorTask(void *argument)
{
    uint32_t lastDetect;
    uint32_t lastExecute;
    uint32_t lastCommunication;
    uint32_t strikes;

    (void)argument;
    lastDetect = detectAliveCounter;
    lastExecute = executeAliveCounter;
    lastCommunication = communicationAliveCounter;
    strikes = 0U;

    for (;;)
    {
        osDelay(TASK_MONITOR_PERIOD_MS);
        if ((detectAliveCounter == lastDetect) ||
            (executeAliveCounter == lastExecute) ||
            (communicationAliveCounter == lastCommunication))
        {
            ++strikes;
        }
        else
        {
            strikes = 0U;
            Watchdog_Refresh();
        }
        if (strikes >= TASK_MONITOR_STRIKE_LIMIT)
        {
            /* A supervised task stopped running: stop servicing the
               independent watchdog and wait for the hardware reset. */
            for (;;)
            {
            }
        }
        lastDetect = detectAliveCounter;
        lastExecute = executeAliveCounter;
        lastCommunication = communicationAliveCounter;
    }
}

static bool CommunicationExchange(uint8_t sequence, InspectionDecision *decision)
{
    uint8_t triggerFrame[INSPECTION_FRAME_SIZE];
    uint8_t acknowledgeFrame[INSPECTION_FRAME_SIZE];
    uint8_t receivedByte;
    uint32_t startedAt;
    uint32_t elapsed;
    bool retried = false;
    InspectionParser parser;
    InspectionFrame frame;

    InspectionProtocol_ParserInit(&parser);
    InspectionProtocol_Build(INSPECTION_MSG_TRIGGER,
                             sequence,
                             0U,
                             0U,
                             triggerFrame);

    startedAt = HAL_GetTick();
    if (!UART1_IsrSend(triggerFrame, INSPECTION_FRAME_SIZE))
    {
        ++metrics.uart_errors;
        return false;
    }

    for (;;)
    {
        ++communicationAliveCounter;
        elapsed = HAL_GetTick() - startedAt;
        if (elapsed >= UART_TRANSACTION_BUDGET_MS)
        {
            ++metrics.uart_timeouts;
            return false;
        }

        /* Block on the RX stream buffer for one byte; the ISR wakes this
           task as soon as a byte arrives. */
        if (UART1_IsrReadByte(&receivedByte, UART_BYTE_TIMEOUT_MS))
        {
            if (InspectionProtocol_PushByte(&parser, receivedByte, &frame))
            {
                visionLastSeenAt = HAL_GetTick();
                if ((frame.type == INSPECTION_MSG_RESULT) &&
                    (frame.sequence == sequence))
                {
                    if ((frame.code != INSPECTION_RESULT_NORMAL) &&
                        (frame.code != INSPECTION_RESULT_NOTCH) &&
                        (frame.code != INSPECTION_RESULT_DEFORMATION) &&
                        (frame.code != INSPECTION_RESULT_VISION_ERROR))
                    {
                        ++metrics.protocol_errors;
                        return false;
                    }

                    decision->code = frame.code;
                    decision->confidence = frame.auxiliary;
                    InspectionProtocol_Build(INSPECTION_MSG_ACK,
                                             sequence,
                                             frame.code,
                                             0U,
                                             acknowledgeFrame);
                    if (!UART1_IsrSend(acknowledgeFrame, INSPECTION_FRAME_SIZE))
                    {
                        ++metrics.uart_errors;
                        return false;
                    }
                    elapsed = HAL_GetTick() - startedAt;
                    if (elapsed > metrics.max_transaction_ms)
                    {
                        metrics.max_transaction_ms = elapsed;
                    }
                    return true;
                }
                ++metrics.stale_frames;
            }
        }

        elapsed = HAL_GetTick() - startedAt;
        if ((!retried) && (elapsed >= UART_RETRY_AT_MS))
        {
            retried = true;
            ++metrics.uart_retries;
            if (!UART1_IsrSend(triggerFrame, INSPECTION_FRAME_SIZE))
            {
                ++metrics.uart_errors;
                return false;
            }
        }
    }
}

static void FailSafeHalt(void)
{
    taskDISABLE_INTERRUPTS();
    BeltSetRunning(false);
    HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, GPIO_PIN_RESET);
    for (;;)
    {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *taskName)
{
    (void)task;
    (void)taskName;
    FailSafeHalt();
}
