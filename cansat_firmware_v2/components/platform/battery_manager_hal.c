#include <stdarg.h>

#include "battery_manager_hal.h"
#include "bq27441.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME battery_manager_hal
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

bool battery_manager_hal_init(void)
{
    // Initialization sometimes fails so we retry it a few times
    for(unsigned int n = 0; n < 10; ++n)
    {
        NRF_LOG_DEBUG("Initializing %d/10", n+1);
        if(bq27441_init())
        {
            // Enter configuration mode
            NRF_LOG_DEBUG("Entering configuration mode");
            if(bq27441_enter_config(true))
            {
                // Capacity
                NRF_LOG_DEBUG("Setting capacity");
                if(!bq27441_set_capacity(BATTERY_MANAGER_HAL_BATTERY_CAPACITY_MAH))
                {
                    NRF_LOG_ERROR("Error setting battery capacity");
                    continue;
                }

                NRF_LOG_DEBUG("Exitting config mode");
                if(!bq27441_exit_config(true))
                {
                    NRF_LOG_ERROR("Error exitting config");
                    continue;
                }

                // Set GPOUT, CHARGING_DETECTION_PIN and CHARGER_DETECTION_PIN pins as input.
                // Currently we are not using GPOUT pin at all.
                NRF_LOG_DEBUG("Configuring pins");
                nrf_gpio_cfg_input(BATTERY_MANAGER_HAL_CHARGER_DETECTION_PIN, NRF_GPIO_PIN_NOPULL);
                nrf_gpio_cfg_input(BATTERY_MANAGER_HAL_CHARGING_DETECTION_PIN, NRF_GPIO_PIN_NOPULL);
                nrf_gpio_cfg_input(BATTERY_MANAGER_HAL_GPOUT_PIN, NRF_GPIO_PIN_NOPULL);

                // Set ISET pin as input at start so the charging current is 100 mA
                nrf_gpio_cfg_input(BATTERY_MANAGER_HAL_CHARGER_ISET_PIN, NRF_GPIO_PIN_NOPULL);
                battery_manager_hal_set_charging_current(BATTERY_MANAGER_HAL_MIN_CHARGING_CURRENT);

                NRF_LOG_DEBUG("Initialized");
                return true;
            }
            else 
            {
                NRF_LOG_WARNING("Error entering configuration mode");
            }

            // Wait a bit between retries
            nrf_delay_ms(10);
        }
    }

    return false;
}

// Read SOC from gauge
uint8_t battery_manager_hal_get_soc(void)
{
    return bq27441_get_soc(FILTERED);
}

// Read battery voltage in millivolts from gauge driver
uint16_t battery_manager_hal_get_voltage(void)
{
    return bq27441_get_voltage();
}

// Read average current in milliamps from gauge driver
int16_t battery_manager_hal_get_avg_current(void)
{
    return bq27441_get_current(AVG);
}

// Read total battery capacity from gauge
uint16_t battery_manager_hal_get_total_capacity(void)
{
    return bq27441_get_capacity(FULL);
}

void battery_manager_hal_set_charging_current(uint16_t current)
{
    // Set max charging current
    if(current >= BATTERY_MANAGER_HAL_MAX_CHARGING_CURRENT) {
        nrf_gpio_cfg_output(BATTERY_MANAGER_HAL_CHARGER_ISET_PIN);
        nrf_gpio_pin_set(BATTERY_MANAGER_HAL_CHARGER_ISET_PIN);
    }
    else 
    {
        // Set minimum charging current
        nrf_gpio_cfg_input(BATTERY_MANAGER_HAL_CHARGER_ISET_PIN, NRF_GPIO_PIN_NOPULL);
    }
}

// Read remaining battery capacity from gauge
uint16_t battery_manager_hal_get_remaining_capacity(void)
{
    return bq27441_get_capacity(REMAIN);
}

// Read battery health from gauge
uint8_t battery_manager_hal_get_health(void)
{
    return bq27441_get_soh(PERCENT);
}

float battery_manager_hal_get_temperature(void)
{
    return bq27441_get_temperature(INTERNAL_TEMP);
}

bool battery_manager_hal_is_battery_charging(void)
{
    return !nrf_gpio_pin_input_get(BATTERY_MANAGER_HAL_CHARGING_DETECTION_PIN);
}

bool battery_manager_hal_is_charger_connected(void)
{
    return false;
}