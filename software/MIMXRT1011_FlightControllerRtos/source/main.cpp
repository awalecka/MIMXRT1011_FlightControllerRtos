#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "controllers/flight_controller.h"
#include "radio/command_handler.h"
#include "system/logging_task.h"
#include "system/state_manager.h"
#include "system/heartbeat_task.h"
#include "io/gps_handler.h"
#include "telemetry/usb_telemetry.h"

// --- Static Queue Allocations ---
static constexpr size_t CONTROLS_QUEUE_LENGTH = 30;
static constexpr size_t COMMAND_QUEUE_LENGTH = 1;
static constexpr size_t STATE_CHANGE_QUEUE_LENGTH = 2;
static constexpr size_t IMU_QUEUE_LENGTH = 1;
static constexpr size_t MAG_QUEUE_LENGTH = 1;
static constexpr size_t GPS_DATA_QUEUE_LENGTH = 1;

static StaticQueue_t s_controlsQueueControlBlock;
static uint8_t s_controlsQueueStorage[CONTROLS_QUEUE_LENGTH * sizeof(LogMessage_t)];
static QueueHandle_t s_controlsQueue;

static StaticQueue_t s_commandQueueControlBlock;
static uint8_t s_commandQueueStorage[COMMAND_QUEUE_LENGTH * sizeof(RC_Channels_t)];
static QueueHandle_t s_commandQueue;

static StaticQueue_t s_stateChangeQueueControlBlock;
static uint8_t s_stateChangeQueueStorage[STATE_CHANGE_QUEUE_LENGTH * sizeof(FlightState_t)];
static QueueHandle_t s_stateChangeQueue;

static StaticQueue_t s_imuQueueControlBlock;
static uint8_t s_imuQueueStorage[IMU_QUEUE_LENGTH * sizeof(FlightController::ImuData)];
static QueueHandle_t s_imuQueue;

static StaticQueue_t s_magQueueControlBlock;
static uint8_t s_magQueueStorage[MAG_QUEUE_LENGTH * sizeof(FlightController::MagData)];
static QueueHandle_t s_magQueue;

static StaticQueue_t s_gpsDataQueueControlBlock;
static uint8_t s_gpsDataQueueStorage[GPS_DATA_QUEUE_LENGTH * sizeof(firmware::sensors::GpsData)];
static QueueHandle_t s_gpsDataQueue;

// --- Shared System State ---
static volatile FlightState_t s_flightState = STATE_BOOT;
static volatile TickType_t s_heartbeatFrequency = pdMS_TO_TICKS(500);

int main(void) {
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    boardInitUartDma(IBUS_LPUART_INSTANCE, 115200, g_ibusDmaRxBuffer, IBUS_DMA_BUFFER_SIZE,
                     ibus_idle_interrupt_callback, IBUS_DMA_BASE, IBUS_DMA_CHANNEL,
                     IBUS_DMAMUX_BASE, IBUS_DMA_SOURCE, IBUS_LPUART_IRQn);

    boardInitUartDma(GPS_LPUART_INSTANCE, 9600, g_gpsDmaRxBuffer, GPS_DMA_BUFFER_SIZE,
                     gps_idle_interrupt_callback, GPS_DMA_BASE, GPS_DMA_CHANNEL,
                     GPS_DMAMUX_BASE, GPS_DMA_SOURCE, GPS_LPUART_IRQn);

    // Create the queues
    s_controlsQueue = xQueueCreateStatic(CONTROLS_QUEUE_LENGTH, sizeof(LogMessage_t), s_controlsQueueStorage, &s_controlsQueueControlBlock);
    s_commandQueue = xQueueCreateStatic(COMMAND_QUEUE_LENGTH, sizeof(RC_Channels_t), s_commandQueueStorage, &s_commandQueueControlBlock);
    s_stateChangeQueue = xQueueCreateStatic(STATE_CHANGE_QUEUE_LENGTH, sizeof(FlightState_t), s_stateChangeQueueStorage, &s_stateChangeQueueControlBlock);
    s_imuQueue = xQueueCreateStatic(IMU_QUEUE_LENGTH, sizeof(FlightController::ImuData), s_imuQueueStorage, &s_imuQueueControlBlock);
    s_magQueue = xQueueCreateStatic(MAG_QUEUE_LENGTH, sizeof(FlightController::MagData), s_magQueueStorage, &s_magQueueControlBlock);
    s_gpsDataQueue = xQueueCreateStatic(GPS_DATA_QUEUE_LENGTH, sizeof(firmware::sensors::GpsData), s_gpsDataQueueStorage, &s_gpsDataQueueControlBlock);

    // Instantiate Active Objects HERE as block-scope statics so they receive valid queue handles
    static FlightController s_flightController(FlightController::LOOP_DT_S);
    static StateManager s_stateManager(s_flightController, s_stateChangeQueue, s_imuQueue, s_magQueue, s_flightState, s_heartbeatFrequency);
    static CommandHandler s_commandHandler(s_commandQueue);
    static GpsHandler s_gpsHandler(s_gpsDataQueue);
    static HeartbeatTask s_heartbeatTask(s_heartbeatFrequency);
    static LoggingTask s_loggingTask(s_controlsQueue);

    // Inject dependencies into the flight controller
    s_flightController.injectDependencies(s_gpsDataQueue, s_imuQueue, s_magQueue, s_commandQueue, s_controlsQueue, &s_flightState);

    // Start all Active Objects
    s_stateManager.start();
    s_commandHandler.start();
    s_gpsHandler.start();
    s_heartbeatTask.start();
    s_loggingTask.start();

    // Register ISR bounds
    commandHandlerRegisterIsrTask(s_commandHandler.getTaskHandle());
    gpsHandlerRegisterIsrTask(s_gpsHandler.getTaskHandle());

    vTaskStartScheduler();

    while(1) {}
    return 0;
}
