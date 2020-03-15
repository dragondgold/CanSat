#ifndef __BATTERY_MANAGER_H__
#define __BATTERY_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>

#define BATTERY_MANAGER_STACK_SIZE              1500
#define BATTERY_MANAGER_TASK_PRIORITY           2
#define BATTERY_MANAGER_MUTEX_TIMEOUT_MS        500

typedef struct 
{
    uint8_t soc;                    // State of charge in %
    uint16_t volts;                 // Battery voltage in mV
    int16_t avg_current;            // Average current in mA
    uint8_t total_capacity;         // Total battery capacity in mAh
    uint8_t remaining_capacity;     // Remaining battery capacity in mAh
    uint8_t health;                 // Battery health in %
    bool is_charging;               // Is the battery being charged?
    bool charging_at_max;           // Is the battery charging with full current?
} battery_data_t;

bool battery_manager_init(void);

battery_data_t battery_manager_get(void);

#endif // __BATTERY_MANAGER_H__
