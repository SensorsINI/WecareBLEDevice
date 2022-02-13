#ifndef _MCR35614R_H_
#define _MCR35614R_H_

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

// MCR35614R's SPI address is '01' which is hardcoded in the chip
#define SPI_ADDR 1  
#define SPI_ADDR_BIT_OFFSET 6

// command type
#define STATIC_READ 1
#define INC_WRITE 2
#define INC_READ 3

#define FAST_CMD_START_CONVERSION 0x28
#define FAST_CMD_ADC_STANDBY 0x2C
#define FAST_CMD_ADC_SHUTDOWN 0x30
#define FAST_CMD_FULL_STUTDOWN 0x34
#define FAST_CMD_FULL_RESET 0x38

// Register map
#define REG_ADDR_BIT_OFFSET 2

#define ADCDATA 0
#define CONFIG0 1
#define CONFIG1 2
#define CONFIG2 3
#define CONFIG3 4
#define IRQ 5
#define MUX 6
#define SCAN 7
#define TIMER 8
#define OFFSETCAL 9
#define GAINCAL 10
#define RESERVED0 11
#define RESERVED1 12
#define LOCK 13
#define RESERVED2 14
#define CRCCFG 15

#define ADCDATA_BYTES 4   // Use 32-bit data format
#define CONFIG0_BYTES 1
#define CONFIG1_BYTES 1
#define CONFIG2_BYTES 1
#define CONFIG3_BYTES 1
#define IRQ_BYTES 1
#define MUX_BYTES 1
#define SCAN_BYTES 3
#define TIMER_BYTES 3
#define OFFSETCAL_BYTES 3
#define GAINCAL_BYTES 3
#define RESERVED0_BYTES 3
#define RESERVED1_BYTES 1
#define LOCK_BYTES 1
#define RESERVED2_BYTES 2
#define CRCCFG_BYTES 2

#define TOTAL_BYTES 31


/**
 ****************************************************************************************
 * @brief Function declartion
 ****************************************************************************************
*/
void spi2_adc_static_read_register(uint8_t regAddr, uint8_t *readBuf, uint8_t readBytes);
void spi2_adc_increment_read_register(uint8_t regAddr, uint8_t *readBuf, uint8_t readBytes);
void spi2_adc_fast_command(uint8_t cmdType);
void spi2_adc_write_register(uint8_t regAddr, uint8_t *sendBuf, uint8_t writeBytes);
uint32_t swapBufToRealVal(uint8_t *buf, uint8_t regLenByte);


#endif // _MCR35614R_H_
