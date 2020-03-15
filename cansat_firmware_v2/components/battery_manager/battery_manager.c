#include "battery_manager.h"
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "battery_manager_hal.h"

#define NRF_LOG_MODULE_NAME battery_manager
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

// Mutex
static StaticSemaphore_t mutex_buffer;
static SemaphoreHandle_t mutex;

// Task
static StackType_t stack[BATTERY_MANAGER_STACK_SIZE];
static StaticTask_t task;
static TaskHandle_t task_handle;

// Battery data
battery_data_t battery_data = { 0 };

static void battery_sample_task(void* args)
{
    battery_data_t temp_data;

    while(1)
    {
        // Check battery and charger status every 1 second
        vTaskDelay(pdMS_TO_TICKS(BATTERY_MANAGER_CHECK_INTERVAL_MS));
        NRF_LOG_DEBUG("Checking battery");

        // Read all the data from the fuel-gauge
        temp_data.soc = battery_manager_hal_get_soc();
        temp_data.volts = battery_manager_hal_get_voltage();
        temp_data.avg_current = battery_manager_hal_get_avg_current();
        temp_data.total_capacity = battery_manager_hal_get_total_capacity();
        temp_data.remaining_capacity = battery_manager_hal_get_remaining_capacity();
        temp_data.health = battery_manager_hal_get_health();
        temp_data.is_charging = battery_manager_hal_is_battery_charging();

        // Inform on charger connect/disconnect
        if(battery_data.is_charging != temp_data.is_charging)
        {
            NRF_LOG_INFO("Battery %s", temp_data.is_charging ? "started charging" : "stopped charging");
            NRF_LOG_INFO("Battery SOC: %d %%", temp_data.soc);
            NRF_LOG_INFO("Battery avg current: %d mA", temp_data.avg_current);
            NRF_LOG_INFO("Battery voltage: %d mV", temp_data.volts);
            NRF_LOG_INFO("Battery temperature: %.2f", battery_manager_hal_get_temperature());

            // Led indication when charging
            if(temp_data.is_charging)
            {
                led_manager_slow_blink(true);
            }
            else
            {
                led_manager_on();
            }
        }

        NRF_LOG_DEBUG("Battery SOC: %d %%", temp_data.soc);
        NRF_LOG_DEBUG("Battery voltage: %d mV", temp_data.volts);
        NRF_LOG_DEBUG("Battery avg current: %d mA", temp_data.avg_current);
        NRF_LOG_DEBUG("Battery total capacity: %d mAh", temp_data.total_capacity);
        NRF_LOG_DEBUG("Battery remaining capacity: %d mAh", temp_data.remaining_capacity);
        NRF_LOG_DEBUG("Battery health %d %%", temp_data.remaining_capacity);
        NRF_LOG_DEBUG("Battery is %s", temp_data.is_charging ? "charging" : "not charging");

        if(temp_data.soc != battery_data.soc)
        {
            NRF_LOG_INFO("Battery SOC: %d %%", temp_data.soc);
            NRF_LOG_INFO("Battery avg current: %d mA", temp_data.avg_current);
            NRF_LOG_INFO("Battery voltage: %d mV", temp_data.volts);
            NRF_LOG_INFO("Battery temperature: %.2f", battery_manager_hal_get_temperature());
        }

        // Take the mutex so other tasks cannot read the data while it's being
        //  modified
        if(xSemaphoreTake(mutex, 1000 / portTICK_PERIOD_MS))
        {
            // Copy all the data
            battery_data = temp_data;

            // Set max charging current
            battery_manager_hal_set_charging_current(BATTERY_MANAGER_HAL_MAX_CHARGING_CURRENT);
            battery_data.charging_at_max = true;
            NRF_LOG_DEBUG("Charging current at " STRINGIFY(BATTERY_MANAGER_HAL_MAX_CHARGING_CURRENT) " mA");
            
            xSemaphoreGive(mutex);
        }
        else 
        {
            NRF_LOG_ERROR("Error getting mutex on battery_sample_task()");
        }
    }
}

bool battery_manager_init(void)
{
    NRF_LOG_INFO("Initializing");

    // Create mutex for this resource
    mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
    
    // Initialize gauge driver
    if(battery_manager_hal_init())
    {
        NRF_LOG_ERROR("Failed initializing fuel gauge");
    }

    // Task to sample sensors
    NRF_LOG_DEBUG("Starting task");
    task_handle = xTaskCreateStatic(battery_sample_task, "battery", BATTERY_MANAGER_STACK_SIZE, 
        NULL, BATTERY_MANAGER_TASK_PRIORITY, stack, &task);

    NRF_LOG_INFO("Initialized");
    return false;
}

battery_data_t battery_manager_get(void)
{
    battery_data_t data = { 0 };

    // Take the mutex so the struct cannot be changed while returning it
    if(xSemaphoreTake(mutex, pdMS_TO_TICKS(BATTERY_MANAGER_MUTEX_TIMEOUT_MS)))
    {
        // Make a copy so we are safe to pass it atomically
        data = battery_data;
        xSemaphoreGive(mutex);
        
        return data;
    }
    
    return data;
}