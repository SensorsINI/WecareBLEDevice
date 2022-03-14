#include "MCR35614R.h"

/**
 ****************************************************************************************
 * @brief SPI2 ADC (MCR35614R) static read register function.
 * @return void
 ****************************************************************************************
*/
void spi2_adc_static_read_register(uint8_t regAddr, uint8_t *readBuf, uint8_t readBytes)
{
	  uint8_t adc1_command = (SPI_ADDR << SPI_ADDR_BIT_OFFSET) + (regAddr << REG_ADDR_BIT_OFFSET) + STATIC_READ;
	  spi_cs_low();
		spi_send(&adc1_command, 1, SPI_OP_BLOCKING);
    spi_receive(readBuf, readBytes, SPI_OP_BLOCKING);			
    spi_cs_high();
}

/**
 ****************************************************************************************
 * @brief SPI2 ADC (MCR35614R) increment read register function.
 * @return void
 ****************************************************************************************
*/
void spi2_adc_increment_read_register(uint8_t regAddr, uint8_t *readBuf, uint8_t readBytes)
{
	  uint8_t adc1_command = (SPI_ADDR << SPI_ADDR_BIT_OFFSET) + (regAddr << REG_ADDR_BIT_OFFSET) + INC_READ;
	  spi_cs_low();
		spi_send(&adc1_command, 1, SPI_OP_BLOCKING);
    spi_receive(readBuf, readBytes, SPI_OP_BLOCKING);			
    spi_cs_high();
}

/**
 ****************************************************************************************
 * @brief SPI2 ADC (MCR35614R) fast command control function.
 * @return void
 ****************************************************************************************
*/
void spi2_adc_fast_command(uint8_t cmdType)
{
	  uint8_t adc1_command = (SPI_ADDR << SPI_ADDR_BIT_OFFSET) + cmdType;
	  spi_cs_low();
		spi_send(&adc1_command, 1, SPI_OP_BLOCKING);
    spi_cs_high();	
}

/**
 ****************************************************************************************
 * @brief SPI2 ADC (MCR35614R) write register function.
 * @return void
 ****************************************************************************************
*/
void spi2_adc_write_register(uint8_t regAddr, uint8_t *sendBuf, uint8_t writeBytes)
{
	  uint8_t adc1_command = (SPI_ADDR << SPI_ADDR_BIT_OFFSET) + (regAddr << REG_ADDR_BIT_OFFSET) + INC_WRITE;
	  spi_cs_low();
		spi_send(&adc1_command, 1, SPI_OP_BLOCKING);
		spi_send(sendBuf, writeBytes, SPI_OP_BLOCKING);
    spi_cs_high();		
}

/**
 ****************************************************************************************
 * @brief Swap the little endian byte buffer to form the real register value.
 * @return void
 ****************************************************************************************
*/
uint32_t swapBufToRealVal(uint8_t *buf, uint8_t regLenByte)
{
		uint32_t retVal = 0;
		for(int i = 0; i < regLenByte; i++)
		{
				uint16_t bitOffset = (regLenByte - 1 - i) * 8;
			  retVal += buf[i] << bitOffset;
		}
		return retVal;
}
