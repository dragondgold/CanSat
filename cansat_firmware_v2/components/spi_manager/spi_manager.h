#ifndef __SPI_MANAGER_H__
#define __SPI_MANAGER_H__

#include <stdbool.h>
#include "FreeRTOS.h"
#include "nrf_gpio.h"

#define SPI_MANAGER_BUFFER_PIN          NRF_GPIO_PIN_MAP(0, 20)
#define SPI_MANAGER_QUEUE_SIZE          5
#define SPI_MANAGER_MUTEX_TIMEOUT       (200 / portTICK_PERIOD_MS)

// We support up to 2 SPI hardware instances at the same time. Comment the instance to not use it.
#define SPI_MANAGER_INSTANCE_ID_1       0
#define SPI_MANAGER_INSTANCE_ID_2       2

#if !defined(SPI_MANAGER_INSTANCE_ID_1) && !defined (SPI_MANAGER_INSTANCE_ID_2)
    #error "You must define an instance for spi_manager"
#endif

typedef enum
{
    SPI_MANAGER_PERIPHERAL_ANY = -1,
    SPI_MANAGER_PERIPHERAL_1 = 0,
    SPI_MANAGER_PERIPHERAL_2 = 1
} spi_manager_peripheral_t;

typedef struct
{
    uint8_t * tx_data;
    uint8_t tx_length;
    uint8_t * rx_data;
    uint8_t rx_length;
} spi_manager_transaction_t; 

typedef struct
{
     uint8_t mosi_pin;
     uint8_t miso_pin;
     uint8_t clk_pin;
     uint8_t cs_pin;
     int8_t spi_peripheral;
} spi_manager_devices_t; 

typedef struct 
{
    spi_manager_devices_t* devices;
    unsigned int length;
} spi_manager_cfg_t; 

bool spi_manager_init(spi_manager_cfg_t * cfg);

/**
 * @brief Transmit the requested spi_transaction_t to the SPI bus. This waits for
 *  the SPI data to be send synchronously, this method is not async.
 * 
 * @param trans transaction to send and used to receive the data back
 * @param timeout max time to wait for the SPI bus to be available
 * @return true if transaction was successfull
 */
bool spi_manager_device_transmit(spi_manager_transaction_t * transaction, unsigned int device, TickType_t timeout);

void spi_manager_enable_buffer(void);
void spi_manager_disable_buffer(void);
bool spi_manager_is_buffer_disabled(void);

#endif