#ifndef __I2C_MANAGER_H__
#define __I2C_MANAGER_H__

#include <stdbool.h>
#include "FreeRTOS.h"
#include "nrf_gpio.h"

#define I2C_MANAGER_TWI_INSTANCE_ID             1
#define I2C_MANAGER_MAX_PENDING_TRANSACTIONS    5
#define I2C_MANAGER_ENABLE_PIN                  NRF_GPIO_PIN_MAP(0, 15)
#define I2C_MANAGER_MUTEX_TIMEOUT               (200 / portTICK_PERIOD_MS)
#define I2C_MANAGER_SDA_PIN                     NRF_GPIO_PIN_MAP(0, 18)
#define I2C_MANAGER_SCL_PIN                     NRF_GPIO_PIN_MAP(0, 19)

bool i2c_manager_init(void);

void i2c_manager_enable_buffer(void);
void i2c_manager_disable_buffer(void);
bool i2c_manager_is_buffer_disabled(void);

bool i2c_manager_acquire(unsigned int port, TickType_t timeout);
bool i2c_manager_release(unsigned int port);

ret_code_t i2c_manager_read_register(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, uint8_t* value);
ret_code_t i2c_manager_read_register_multiple(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, unsigned int length, uint8_t* values);
ret_code_t i2c_manager_write_register(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, uint8_t value);
ret_code_t i2c_manager_write_register_multiple(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t* values);

#endif