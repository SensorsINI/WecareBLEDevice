#ifndef _DAC70508M_H_
#define _DAC70508M_H_

#include <stdint.h>

// SPI master API
#include "spi_531.h"
#include "spi.h"

//Uart
#include "uart_utils.h"

/**
 ****************************************************************************************
 * @brief Macro definitions
 ****************************************************************************************
*/
// Register map
#define NOP 0
#define DEVICE_ID 1
#define SYNC 2
#define CONFIG 3
#define GAIN 4
#define TRIGGER 5
#define BRDCAST 6
#define STATUS 7
#define DAC0 8
#define DAC1 9
#define DAC2 10
#define DAC3 11
#define DAC4 12
#define DAC5 13
#define DAC6 14
#define DAC7 15

/**
 ****************************************************************************************
 * @brief Function declartion
 ****************************************************************************************
*/
int spi2_dac_read_register(uint16_t regAddr, uint16_t *readVal);
int spi2_dac_write_register(uint16_t regAddr, uint16_t setVal);
void spi2_dac_set_data(uint16_t *DACValBuf);

#endif // _DAC70508M_H_
