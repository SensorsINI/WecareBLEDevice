#include "DAC70508M.h"

/**
 ****************************************************************************************
 * @brief SPI2 DAC (DAC70508M) read register function.
 * @return int. 0: Success -1: Fail
 ****************************************************************************************
*/
int spi2_dac_read_register(uint16_t regAddr, uint16_t *readVal)
{
	  if(regAddr >= 16)
		{
			printf_string(UART1, "Register address is out of range.");
			return -1;   // DAC70508M only have 16 registers.
		}
	  uint32_t dac_command = ((uint32_t)regAddr << 16);
	  dac_command ^= (1 << 23);  // Set bit 23 to 1 to indicate it is a read operation.
	  spi_cs_low();
    spi_send(&dac_command, 1, SPI_OP_BLOCKING);
    spi_cs_high();
	  // Readback data
	  uint32_t receiveBuf; 
    spi_cs_low();
    spi_receive(&receiveBuf, 1, SPI_OP_BLOCKING);
    spi_cs_high();
	  *readVal = receiveBuf >> 8;     // Return bits 23 to 8 
	  return 0;
}	  

/**
 ****************************************************************************************
 * @brief SPI2 DAC (DAC70508M) write register function.
 * @return int. 0: Success -1: Fail
 ****************************************************************************************
*/
int spi2_dac_write_register(uint16_t regAddr, uint16_t setVal)
{
	  if(regAddr >= 16)
		{
			printf_string(UART1, "Register address is out of range.");
			return -1;   // DAC70508M only have 16 registers.
		}
	  uint32_t dac_command = ((uint32_t)regAddr << 16) + setVal;
	  spi_cs_low();
    spi_send(&dac_command, 1, SPI_OP_BLOCKING);
    spi_cs_high();
	  
	  //Verify if it is written successfully
	  uint16_t tmpRead;
	  int retStatus = spi2_dac_read_register(regAddr, &tmpRead) != 0;
	
		if(retStatus != 0 | tmpRead != setVal)
		{
//			printf_string(UART1, "Write failed.\r\n");
//		  printf_string(UART1, "The register address is: ");
//			print_hword(UART1, regAddr);
//			printf_string(UART1, ". Read value is: 0x");
//			print_hword(UART1, tmpRead);
//			printf_string(UART1, ", but the data we want to write is: ");
//			print_hword(UART1, setVal);
//			printf_string(UART1, ".\r\n");
			return -1;
		}
	  else return 0;	  
}
