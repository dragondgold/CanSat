/*
 * cansat.h
 *
 *  Created on: Nov 12, 2018
 *      Author: tmax4
 */

#ifndef CANSAT_CONFIG_CANSAT_H_
#define CANSAT_CONFIG_CANSAT_H_

#include "periph/gpio.h"

// GPIO
#define ENABLE_5V_PORT PORT_A
#define ENABLE_5V_PIN 7
#define ENABLE_5V_MODE GPIO_OUT

#define ENABLE_3V3_PORT PORT_B
#define ENABLE_3V3_PIN 0
#define ENABLE_3V3_MODE GPIO_OUT

#define CHG_ALLOWED_PORT PORT_B
#define CHG_ALLOWED_PIN 1
#define CHG_ALLOWED_MODE GPIO_IN_PU

#define CHG_DETECT_PORT PORT_A
#define CHG_DETECT_PIN 2
#define CHG_DETECT_MODE GPIO_IN_PD

#define SW_OPEN_PORT PORT_B
#define SW_OPEN_PIN 2
#define SW_OPEN_MODE GPIO_IN_PU

#define GPOUT_PORT PORT_B
#define GPOUT_PIN 5
#define GPOUT_MODE GPIO_IN

#define P1_PORT PORT_B
#define P1_PIN 7
#define P1_MODE GPIO_OUT

#define CHARGING_PORT PORT_B
#define CHARGING_PIN 8
#define CHARGING_MODE GPIO_IN_PU

#define ISET_PORT PORT_B
#define ISET_PIN 9
#define ISET_MODE GPIO_OUT

#define IO0_PORT PORT_C
#define IO0_PIN 2
#define IO0_MODE GPIO_IN

#define IO1_PORT PORT_C
#define IO1_PIN 3
#define IO1_MODE GPIO_IN

#define IO2_PORT PORT_B
#define IO2_PIN 10
#define IO2_MODE GPIO_IN

#define IO3_PORT PORT_B
#define IO3_PIN 11
#define IO3_MODE GPIO_IN

#define CHG_ENABLE_PORT PORT_D
#define CHG_ENABLE_PIN 2
#define CHG_ENABLE_MODE GPIO_OUT

#define S_GLOBO_PORT PORT_C
#define S_GLOBO_PIN 0
#define S_GLOBO_MODE GPIO_OUT

#define S_PARACAIDAS_PORT PORT_C
#define S_PARACAIDAS_PIN 1
#define S_PARACAIDAS_MODE GPIO_OUT

#define I2C_ENABLE_PORT PORT_C
#define I2C_ENABLE_PIN 4
#define I2C_ENABLE_MODE GPIO_OUT

#define SPI_BUFFER_ENABLE_PORT PORT_C
#define SPI_BUFFER_ENABLE_PIN 5
#define SPI_BUFFER_ENABLE_MODE GPIO_OUT

#define IO_ENABLE_PORT PORT_C
#define IO_ENABLE_PIN 8
#define IO_ENABLE_MODE GPIO_OUT

// Analog Inputs
#define I_SENSE_5V_PORT PORT_A
#define I_SENSE_5V_PIN 0
#define I_SENSE_5V_CHANNEL 1

#define I_SENSE_3V3_PORT PORT_A
#define I_SENSE_3V3_PIN 1
#define I_SENSE_3V3_CHANNEL 2

#define V_SENSE_5V_PORT PORT_A
#define V_SENSE_5V_PIN 5
#define V_SENSE_5V_CHANNEL 2

#define V_SENSE_3V3_PORT PORT_A
#define V_SENSE_3V3_PIN 4
#define V_SENSE_3V3_CHANNEL 1

#define I_SENSE_BATT_PORT PORT_A
#define I_SENSE_BATT_PIN 3
#define I_SENSE_BATT_CHANNEL 4

// UART
#define GPS_UART 2
#define COMMAND_UART 3

void cansat_init(void);

#endif /* CANSAT_CONFIG_CANSAT_H_ */