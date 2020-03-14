#include <stdbool.h>
#include "time_manager.h"
#include "calendar.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "ble_manager.h"

#include "app_error.h"

#define NRF_LOG_MODULE_NAME time_manager
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

// Time thread
static StaticTask_t xTaskBuffer;
static StackType_t xStack[TIME_MANAGER_TASK_STACK_SIZE];
TaskHandle_t xHandle = NULL;

static void monitor_thread(void * pvParameters)
{
    UNUSED_PARAMETER(pvParameters);
    TickType_t last_wake_time;

    while(true)
    {
        last_wake_time = xTaskGetTickCount();
        
        // Add seconds to the calendar to keep time in sync
        calendar_add_seconds(TIME_MANAGER_INTERVAL_MS / 1000);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TIME_MANAGER_INTERVAL_MS));
    }
}

bool time_manager_init(void)
{
    NRF_LOG_INFO("Initializing");
    calendar_init();
    xHandle = xTaskCreateStatic(
                      monitor_thread,
                      "Time",
                      TIME_MANAGER_TASK_STACK_SIZE,
                      NULL,
                      TIME_MANAGER_TASK_PRIORITY,
                      xStack,
                      &xTaskBuffer);
    if(xHandle == NULL) {
        return false;
    }
    
    NRF_LOG_INFO("Initialized");
    return true;
}