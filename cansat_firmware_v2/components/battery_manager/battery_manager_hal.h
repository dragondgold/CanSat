#ifndef __BATTERY_MANAGER_HAL_H__
#define __BATTERY_MANAGER_HAL_H__

#include <stdbool.h>
#include <stdint.h>

#define BATTERY_MANAGER_HAL_BATTERY_CAPACITY_MAH        600
#define BATTERY_MANAGER_HAL_MAX_CHARGING_CURRENT        300
#define BATTERY_MANAGER_HAL_MIN_CHARGING_CURRENT        100
#define BATTERY_MANAGER_CHECK_INTERVAL_MS               1000

#define BATTERY_MANAGER_HAL_CHARGER_DETECTION_PIN       NRF_GPIO_PIN_MAP(0, 21)
#define BATTERY_MANAGER_HAL_CHARGING_DETECTION_PIN      NRF_GPIO_PIN_MAP(0, 22)
#define BATTERY_MANAGER_HAL_GPOUT_PIN                   NRF_GPIO_PIN_MAP(0, 23)
#define BATTERY_MANAGER_HAL_CHARGER_ISET_PIN            NRF_GPIO_PIN_MAP(0, 24)

// Initialize the fuel gauge driver
bool battery_manager_hal_init(void);

// Read SOC from gauge
uint8_t battery_manager_hal_get_soc(void);

// Read battery voltage in millivolts from gauge driver
uint16_t battery_manager_hal_get_voltage(void);

// Read average current in milliamps from gauge driver
int16_t battery_manager_hal_get_avg_current(void);

// Read total battery capacity from gauge
uint16_t battery_manager_hal_get_total_capacity(void);

// Read remaining battery capacity from gauge
uint16_t battery_manager_hal_get_remaining_capacity(void);

// Read battery health from gauge
uint8_t battery_manager_hal_get_health(void);

// Read battery current temperature
float battery_manager_hal_get_temperature(void);

// Set battery charging current in mA
void battery_manager_hal_set_charging_current(uint16_t current);

bool battery_manager_hal_is_battery_charging(void);

bool battery_manager_hal_is_charger_connected(void);

#endif