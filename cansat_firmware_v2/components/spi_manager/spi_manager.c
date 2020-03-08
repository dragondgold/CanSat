#include "spi_manager.h"

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "nrf_gpio.h"
#include "nrf_spi_mngr.h"

#define NRF_LOG_MODULE_NAME spi_manager
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

#define IS_MUTEX_TAKEN(mutex) (xQueuePeek((xQueueHandle) (mutex), (void *) NULL, (portTickType) NULL) != pdTRUE)

// Mutex for SPI buffer access
static StaticSemaphore_t buffer_mutex_buffer;
static SemaphoreHandle_t buffer_mutex;

// Mutex to guard spi_manager_acquire/spi_manager_release access
static StaticSemaphore_t acquire_release_mutex_buffer;
static SemaphoreHandle_t acquire_release_mutex;

static StaticSemaphore_t spi_mutex_buffers[2];
static SemaphoreHandle_t spi_mutexes[2];

static bool enabled = false;
static spi_manager_cfg_t * devices_cfg;

#ifdef SPI_MANAGER_INSTANCE_ID_1
    NRF_SPI_MNGR_DEF(m_nrf_spi_mngr_1, SPI_MANAGER_QUEUE_SIZE, SPI_MANAGER_INSTANCE_ID_1);
#endif

#ifdef SPI_MANAGER_INSTANCE_ID_2
    NRF_SPI_MNGR_DEF(m_nrf_spi_mngr_2, SPI_MANAGER_QUEUE_SIZE, SPI_MANAGER_INSTANCE_ID_2);
#endif

static spi_manager_peripheral_t get_peripheral_for_device(unsigned int device)
{
    spi_manager_peripheral_t peripheral = devices_cfg->devices[device].spi_peripheral;

    // In this case we check what peripheral is free and use it
    if(peripheral == SPI_MANAGER_PERIPHERAL_ANY)
    {
        #ifdef SPI_MANAGER_INSTANCE_ID_1
        if(!IS_MUTEX_TAKEN(spi_mutexes[SPI_MANAGER_PERIPHERAL_1]))
        {
            return SPI_MANAGER_PERIPHERAL_1;
        }
        #endif
        #ifdef SPI_MANAGER_INSTANCE_ID_2
        if(!IS_MUTEX_TAKEN(spi_mutexes[SPI_MANAGER_PERIPHERAL_2]))
        {
            return SPI_MANAGER_PERIPHERAL_2;
        }
        #endif
    }

    if(peripheral > SPI_MANAGER_PERIPHERAL_2)
    {
        peripheral = SPI_MANAGER_PERIPHERAL_2;
    }

#if !defined(SPI_MANAGER_INSTANCE_ID_2) && defined(SPI_MANAGER_INSTANCE_ID_1)
    if(peripheral == SPI_MANAGER_PERIPHERAL_2) {
        peripheral = SPI_MANAGER_PERIPHERAL_1;
    }
#endif
#if !defined(SPI_MANAGER_INSTANCE_ID_1) && defined(SPI_MANAGER_INSTANCE_ID_2)
    if(peripheral == SPI_MANAGER_PERIPHERAL_1) {
        peripheral = SPI_MANAGER_PERIPHERAL_2;
    }
#endif

    return peripheral;
}

static bool spi_manager_acquire(unsigned int device, TickType_t timeout, spi_manager_peripheral_t * peripheral)
{
    xSemaphoreTake(acquire_release_mutex, timeout);
    spi_manager_peripheral_t p = get_peripheral_for_device(device);

    if(xSemaphoreTake(spi_mutexes[p], timeout)) 
    {
        *peripheral = p;
        xSemaphoreGive(acquire_release_mutex);
        return true;
    }

    xSemaphoreGive(acquire_release_mutex);
    return false;
}

static bool spi_manager_release(spi_manager_peripheral_t peripheral)
{
    if(xSemaphoreGive(spi_mutexes[peripheral]))
    {
        return true;
    }
    return false;
}

static nrf_drv_spi_config_t * get_device_drv_config(unsigned int device)
{
    static nrf_drv_spi_config_t config;

    if(device >= devices_cfg->length)
    {
        NRF_LOG_ERROR("Can't find SPI device %d. Devices length: %d.", device, devices_cfg->length);
        return false;
    }

    // Make it static so we can return a pointer to the struct. Make sure this is called
    //  from one thread at a time.
    config.sck_pin        = devices_cfg->devices[device].clk_pin;
    config.mosi_pin       = devices_cfg->devices[device].mosi_pin;
    config.miso_pin       = devices_cfg->devices[device].miso_pin;
    config.ss_pin         = devices_cfg->devices[device].cs_pin;
    config.irq_priority   = APP_IRQ_PRIORITY_LOWEST;
    config.orc            = 0xFF;
    config.frequency      = NRF_DRV_SPI_FREQ_8M;
    config.mode           = NRF_DRV_SPI_MODE_0;
    config.bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    return &config;
}

static const nrf_spi_mngr_t * get_peripheral_spi_manager(spi_manager_peripheral_t peripheral)
{
#ifdef SPI_MANAGER_INSTANCE_ID_1
    if(peripheral == SPI_MANAGER_PERIPHERAL_1)
    {
        return &m_nrf_spi_mngr_1;
    }
#endif
#ifdef SPI_MANAGER_INSTANCE_ID_2
    if(peripheral == SPI_MANAGER_PERIPHERAL_2)
    {
        return &m_nrf_spi_mngr_2;
    }
#endif

    NRF_LOG_ERROR("get_peripheral_spi_manager should NEVER EVER reach here!");
    return NULL;
}

bool spi_manager_init(spi_manager_cfg_t * cfg)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Initializing");
    devices_cfg = cfg;

    // Create the buffer_mutex for this resource
    buffer_mutex = xSemaphoreCreateMutexStatic(&buffer_mutex_buffer);
    spi_mutexes[0] = xSemaphoreCreateMutexStatic(&spi_mutex_buffers[0]);
    spi_mutexes[1] = xSemaphoreCreateMutexStatic(&spi_mutex_buffers[1]);
    acquire_release_mutex = xSemaphoreCreateMutexStatic(&acquire_release_mutex_buffer);

    // Configure the pin for the SPI buffer
    nrf_gpio_cfg_output(SPI_MANAGER_BUFFER_PIN);
    spi_manager_disable_buffer();

    // Configure CS pin for each SPI device and set it to high (inactive)
    for(unsigned int n = 0; n < cfg->length; ++n)
    {
        nrf_gpio_cfg_output(cfg->devices[n].cs_pin);
        nrf_gpio_pin_set(cfg->devices[n].cs_pin);
    }

    // Initialize the SPI peripheral with the first device if available
    if(cfg->length > 0)
    {
        spi_manager_peripheral_t peripheral = get_peripheral_for_device(0);
        if((err_code = nrf_spi_mngr_init(get_peripheral_spi_manager(peripheral), get_device_drv_config(0))) != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Error initializing SPI device 0: %d -> %s", err_code, nrf_strerror_get(err_code));
            return false;
        }
    }

    NRF_LOG_INFO("Initialized");

    return true;
}

void spi_manager_enable_buffer(void)
{
    if(xSemaphoreTake(buffer_mutex, SPI_MANAGER_MUTEX_TIMEOUT))
    {
        enabled = true;
        nrf_gpio_pin_set(SPI_MANAGER_BUFFER_PIN);
        xSemaphoreGive(buffer_mutex);
    }
    else
    {
        // Timeout!
        NRF_LOG_WARNING("Timeout enabling SPI buffer");
    }
}

void spi_manager_disable_buffer(void) 
{
    if(xSemaphoreTake(buffer_mutex, SPI_MANAGER_MUTEX_TIMEOUT))
    {
        enabled = true;
        nrf_gpio_pin_clear(SPI_MANAGER_BUFFER_PIN);
        xSemaphoreGive(buffer_mutex);
    }
    else
    {
        // Timeout!
        NRF_LOG_WARNING("Timeout disabling SPI buffer");
    }
}

bool spi_manager_is_buffer_disabled(void)
{
    return enabled;
}

bool spi_manager_device_transmit(spi_manager_transaction_t * transaction, unsigned int device, TickType_t timeout)
{
    bool error = false;
    ret_code_t err_code;
    spi_manager_peripheral_t peripheral;

    // Acquire the SPI bus
    if(!spi_manager_acquire(device, timeout, &peripheral))
    {
        NRF_LOG_ERROR("Timeout claiming SPI bus");
        return false;
    }

    // Create the command for the SPI transfer
    nrf_spi_mngr_transfer_t NRF_SPI_MNGR_BUFFER_LOC_IND transfer_cmd[1] =
    {
        NRF_SPI_MNGR_TRANSFER(transaction->tx_data, transaction->tx_length, transaction->rx_data, transaction->rx_length)
    };

    if((err_code = nrf_spi_mngr_perform(get_peripheral_spi_manager(peripheral), get_device_drv_config(device), &transfer_cmd[0], 1, NULL)) != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Error performing SPI transfer: %d -> %s", err_code, nrf_strerror_get(err_code));
        error = true;
    }

    // Release the bus and return
    spi_manager_release(peripheral);
    return error;
}