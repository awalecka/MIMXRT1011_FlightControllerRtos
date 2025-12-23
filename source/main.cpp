/**
 * @file    main.cpp
 * @brief   Application entry point.
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "flight_controller.h"
#include "fsl_debug_console.h"
#include "peripherals.h"
#include "command_handler.h"
#include "fsl_lpuart.h"

extern "C" {
#include "i2c_sync.h"
}

// EDMA Handle
static edma_handle_t s_edmaHandle;

// --- Global Variable Definitions ---
// These are the actual memory definitions for the externs in flight_controller.h
volatile FlightState_t g_flight_state = STATE_BOOT;
lsm6dsox_handle_t g_sensor_handle = {0};
lis3mdl_handle_t g_mag_handle = {0};

// Buffer must be aligned for DMA access
uint8_t g_dmaRxBuffer[IBUS_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

// Main State Manager Handle
TaskHandle_t g_state_manager_task_handle = NULL;

// Queue Handles (Definitions restored here)
QueueHandle_t g_controls_data_queue = NULL;
QueueHandle_t g_command_data_queue = NULL;
QueueHandle_t g_state_change_request_queue = NULL;

volatile TickType_t g_heartbeat_frequency = pdMS_TO_TICKS(500); // 1Hz

// --- Queue Static Allocation ---

// Controls Data Queue
#define CONTROLS_QUEUE_LENGTH 1
#define CONTROLS_QUEUE_ITEM_SIZE sizeof(ActuatorOutput)
static StaticQueue_t xControlsQueueControlBlock;
static uint8_t ucControlsQueueStorageArea[CONTROLS_QUEUE_LENGTH * CONTROLS_QUEUE_ITEM_SIZE];

// Command Data Queue
#define COMMAND_QUEUE_LENGTH 1
#define COMMAND_QUEUE_ITEM_SIZE sizeof(RC_Channels_t)
static StaticQueue_t xCommandQueueControlBlock;
static uint8_t ucCommandQueueStorageArea[COMMAND_QUEUE_LENGTH * COMMAND_QUEUE_ITEM_SIZE];

// State Change Request Queue
#define STATE_CHANGE_QUEUE_LENGTH 2
#define STATE_CHANGE_QUEUE_ITEM_SIZE sizeof(FlightState_t)
static StaticQueue_t xStateChangeQueueControlBlock;
static uint8_t ucStateChangeQueueStorageArea[STATE_CHANGE_QUEUE_LENGTH * STATE_CHANGE_QUEUE_ITEM_SIZE];

// State Manager Task Allocation
#define STATE_MANAGER_TASK_PRIORITY     (tskIDLE_PRIORITY + 4)
#define STATE_MGR_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t xStateMgrStack[STATE_MGR_STACK_SIZE];
static StaticTask_t xStateMgrTaskControlBlock;

// Local functions
static void initCustomConfig();

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    initCustomConfig();

    PRINTF("Flight Controller Initializing Complete...\r\n");

    // Create Queues
    g_controls_data_queue = xQueueCreateStatic(CONTROLS_QUEUE_LENGTH, CONTROLS_QUEUE_ITEM_SIZE, ucControlsQueueStorageArea, &xControlsQueueControlBlock);
    g_command_data_queue = xQueueCreateStatic(COMMAND_QUEUE_LENGTH, COMMAND_QUEUE_ITEM_SIZE, ucCommandQueueStorageArea, &xCommandQueueControlBlock);
    g_state_change_request_queue = xQueueCreateStatic(STATE_CHANGE_QUEUE_LENGTH, STATE_CHANGE_QUEUE_ITEM_SIZE, ucStateChangeQueueStorageArea, &xStateChangeQueueControlBlock);

    if (g_controls_data_queue == NULL || g_command_data_queue == NULL || g_state_change_request_queue == NULL) {
        PRINTF("FATAL: Failed to create one or more queues.\r\n");
        while(1);
    }

    // Create the State Manager Task
    g_state_manager_task_handle = xTaskCreateStatic(stateManagerTask, "StateMgrTask", STATE_MGR_STACK_SIZE, NULL, STATE_MANAGER_TASK_PRIORITY, xStateMgrStack, &xStateMgrTaskControlBlock);
    if (g_state_manager_task_handle == NULL) {
        PRINTF("FATAL: Failed to create state manager task.\r\n");
        while(1);
    }

    // Start Scheduler
    vTaskStartScheduler();

    // Should not reach here
    while(1);

    return 0;
}

static void initCustomConfig()
{
    // 1. GLOBAL INIT REMOVED:
    //    DMAMUX_Init, EDMA_Init, and EDMA_CreateHandle are removed.
    //    The Controller is already initialized by BOARD_InitBootPeripherals (via MEX).

    // 2. LPUART Initialization (Kept because we disabled it in MEX):
    lpuart_config_t lpuartConfig;
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = 115200;
    lpuartConfig.enableTx = false;
    lpuartConfig.enableRx = true;
    lpuartConfig.rxFifoWatermark = 0; // DMA request on every byte

    // We must manually init the clock if we disabled the component in MEX,
    // or ensure BOARD_BootClockRUN configures it.
    // (The MEX Clocks tool [cite: 164] shows UART_CLK_ROOT is active at 80MHz, so this is safe).
    LPUART_Init(IBUS_LPUART_INSTANCE, &lpuartConfig, BOARD_BOOTCLOCKRUN_UART_CLK_ROOT);

    // 3. Re-create the EDMA Handle for the IBUS Channel (0)
    //    We need s_edmaHandle valid for our local usage, even if EDMA is globally init.
    EDMA_CreateHandle(&s_edmaHandle, IBUS_DMA_BASE, IBUS_DMA_CHANNEL);

    // 4. Configure DMAMUX for Channel 0 (IBUS)
    //    (DMAMUX global init is done by BOARD_Init, but we must set the source for Ch0)
    DMAMUX_SetSource(IBUS_DMAMUX_BASE, IBUS_DMA_CHANNEL, IBUS_DMA_SOURCE);
    DMAMUX_EnableChannel(IBUS_DMAMUX_BASE, IBUS_DMA_CHANNEL);

    // 5. Configure TCD for Circular Buffer (Existing Logic)
    edma_transfer_config_t transferConfig;
    EDMA_PrepareTransfer(&transferConfig,
                         (void *)(uint32_t)LPUART_GetDataRegisterAddress(IBUS_LPUART_INSTANCE),
                         1,
                         g_dmaRxBuffer,
                         1,
                         1,
                         IBUS_DMA_BUFFER_SIZE,
                         kEDMA_PeripheralToMemory);

    EDMA_SubmitTransfer(&s_edmaHandle, &transferConfig);

    // Hardware Loop / Ring Buffer Logic
    IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].DLAST_SGA = -((int32_t)IBUS_DMA_BUFFER_SIZE);
    IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].CSR &= ~(DMA_CSR_DREQ_MASK);

    EDMA_StartTransfer(&s_edmaHandle);

    // 6. Enable LPUART specific features
    LPUART_EnableInterrupts(IBUS_LPUART_INSTANCE, kLPUART_IdleLineInterruptEnable);
    NVIC_SetPriority(IBUS_LPUART_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(IBUS_LPUART_IRQn);
    LPUART_EnableRxDMA(IBUS_LPUART_INSTANCE, true);
}
