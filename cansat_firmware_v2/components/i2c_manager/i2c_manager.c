#include "i2c_manager.h"

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "sdk_config.h"
#include "nrf_twi_mngr.h"
#include "nrf_gpio.h"

#define NRF_LOG_MODULE_NAME i2c_manager
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

static StaticSemaphore_t mutex_buffer;
static SemaphoreHandle_t mutex;
static bool enabled = false;

static StaticSemaphore_t i2c_mutex_buffer;
static SemaphoreHandle_t i2c_mutex;

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, I2C_MANAGER_MAX_PENDING_TRANSACTIONS, I2C_MANAGER_TWI_INSTANCE_ID);

__STATIC_INLINE const nrf_twi_mngr_t * get_twi_manager(unsigned int port)
{
    UNUSED_PARAMETER(port);
    return &m_nrf_twi_mngr;
}

bool i2c_manager_init(void)
{
    NRF_LOG_INFO("Initializing");

    // Create the mutexes for the I2C buffer and I2C peripherals
    mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
    i2c_mutex = xSemaphoreCreateMutexStatic(&i2c_mutex_buffer);

    // Configure I2C buffer enable pin
    nrf_gpio_cfg_output(I2C_MANAGER_ENABLE_PIN);

    // Disable buffer
    i2c_manager_disable_buffer();

    /******************************************************************/
    /*       SET GENERAL I2C BUS FOR SENSORS AND EXTERNAL MODULES     */
    /******************************************************************/
    // Set I2C bus for sensors and external modules
    ret_code_t err_code;

    const nrf_drv_twi_config_t i2c_config = {
       .scl                = I2C_MANAGER_SCL_PIN,
       .sda                = I2C_MANAGER_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &i2c_config);
    if(err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Error initializing TWI manager: %d -> %s", err_code, nrf_strerror_get(err_code))
        return false;
    }

    NRF_LOG_INFO("Initialized");
    return true;
}

void i2c_manager_enable_buffer(void)
{
    if(xSemaphoreTake(mutex, I2C_MANAGER_MUTEX_TIMEOUT))
    {
        enabled = true;
        nrf_gpio_pin_set(I2C_MANAGER_ENABLE_PIN);
        xSemaphoreGive(mutex);
    }
    else
    {
        // Timeout!
        NRF_LOG_WARNING("Timeout enabling I2C buffer");
    }
}

void i2c_manager_disable_buffer(void) 
{
    if(xSemaphoreTake(mutex, I2C_MANAGER_MUTEX_TIMEOUT))
    {
        enabled = false;
        nrf_gpio_pin_clear(I2C_MANAGER_ENABLE_PIN);
        xSemaphoreGive(mutex);
    }
    else
    {
        // Timeout!
        NRF_LOG_WARNING("Timeout enabling I2C buffer");
    }
}

bool i2c_manager_is_buffer_disabled(void)
{
    return enabled;
}

bool i2c_manager_acquire(unsigned int port, TickType_t timeout)
{
    if(port > 1)
    {
        return false;
    }

    if(xSemaphoreTake(i2c_mutex, timeout)) return true;
    else return false;
}

bool i2c_manager_release(unsigned int port)
{
    if(port > 1)
    {
        return false;
    }

    if(xSemaphoreGive(i2c_mutex)) return true;
    else return false;
}

ret_code_t i2c_manager_read_register(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, uint8_t* value)
{
    ret_code_t err;

    // Acquire the I2C module
    if(!i2c_manager_acquire(port, timeout))
    {
        return NRF_ERROR_TIMEOUT;
    }

    // Create the I2C commands
    nrf_twi_mngr_transfer_t const transfers[2] =
    {
        NRF_TWI_MNGR_WRITE(slave_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
        NRF_TWI_MNGR_READ(slave_addr | 0x01, value, 1, 0)
    };

    // This will block until everything is sent
    if((err = nrf_twi_mngr_perform(get_twi_manager(port), NULL, transfers, 2, NULL)) != NRF_SUCCESS) 
    {
        NRF_LOG_ERROR("Error performing I2C register read: %d -> %s", err, nrf_strerror_get(err));
    }

    // Release the bus
    i2c_manager_release(port);
    return err;
}

ret_code_t i2c_manager_read_register_multiple(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, unsigned int length, uint8_t* values)
{
    ret_code_t err;

    // Acquire the I2C module
    if(!i2c_manager_acquire(port, timeout))
    {
        return NRF_ERROR_TIMEOUT;
    }

    // Create the I2C commands
    nrf_twi_mngr_transfer_t const transfers[2] =
    {
        NRF_TWI_MNGR_WRITE(slave_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
        NRF_TWI_MNGR_READ(slave_addr | 0x01, values, length, 0)
    };

    // This will block until everything is sent
    if((err = nrf_twi_mngr_perform(get_twi_manager(port), NULL, transfers, 2, NULL)) != NRF_SUCCESS) 
    {
        NRF_LOG_ERROR("Error performing I2C register reads: %d -> %s", err, nrf_strerror_get(err));
    }

    // Release the bus
    i2c_manager_release(port);
    return err;
}

ret_code_t i2c_manager_write_register_multiple(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t* values)
{
    static uint8_t temp_data_array[256];
    ret_code_t err;

    // Acquire the I2C module
    if(!i2c_manager_acquire(port, timeout))
    {
        return NRF_ERROR_TIMEOUT;
    }

    // Unfortunatelly the TWI manager doesn't let us to write a single byte and then a byte array without
    //  inserting an I2C start between them, it has to be done in a single transfer, so we just copy the entire
    //  data array to a temp array and insert the address byte before the data bytes.
    temp_data_array[0] = reg_addr;
    memcpy(temp_data_array + 1, values, length);

    // Create the I2C commands
    nrf_twi_mngr_transfer_t const transfers[1] =
    {
        NRF_TWI_MNGR_WRITE(slave_addr, &temp_data_array, length + 1, 0)
    };

    // This will block until everything is sent
    if((err = nrf_twi_mngr_perform(get_twi_manager(port), NULL, transfers, 1, NULL)) != NRF_SUCCESS) 
    {
        NRF_LOG_ERROR("Error performing I2C register writes: %d -> %s", err, nrf_strerror_get(err));
    }

    // Release the bus
    i2c_manager_release(port);
    return err;
}

ret_code_t i2c_manager_write_register(unsigned int port, TickType_t timeout, uint8_t slave_addr, uint8_t reg_addr, uint8_t value)
{
    ret_code_t err;

    // Acquire the I2C module
    if(!i2c_manager_acquire(port, timeout))
    {
        return NRF_ERROR_TIMEOUT;
    }

    // Create the I2C commands
    uint8_t temp_data[2] = { reg_addr, value };
    nrf_twi_mngr_transfer_t const transfers[1] =
    {
        NRF_TWI_MNGR_WRITE(slave_addr, &temp_data, sizeof(temp_data), 0)
    };

    // This will block until everything is sent
    if((err = nrf_twi_mngr_perform(get_twi_manager(port), NULL, transfers, 1, NULL)) != NRF_SUCCESS) 
    {
        NRF_LOG_ERROR("Error performing I2C register write: %d -> %s", err, nrf_strerror_get(err));
    }

    // Release the bus
    i2c_manager_release(port);
    return err;
}
