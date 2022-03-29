#include "user_periph_setup.h"
#include "da14531_printf.h"

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


 /****************************************************************************************
 * @brief SPI2 DAC (DAC70508M) set input data function.
* @param DACValBuf: input. DAC input data.
 * @return void
 ****************************************************************************************
*/
void spi2_dac_set_data(uint16_t *DACValBuf)
{
		GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_SPI_EN, true); //Enable DAC
		GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_GPIO, true);  // Disable IO
		GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_GPIO, true); //Disable ADC2			
    spi_initialize(&spi2_dac_cfg);

    uint16_t regData; 

    if(!spi2_dac_read_register(DEVICE_ID, &regData)) 
		{
			  // da14531_printf("DEVICE ID is: 0x%x.\r\n", regData);			
		}

    if(!spi2_dac_read_register(GAIN, &regData)) 
		{
			  // da14531_printf("GAIN channel 4 register is: 0x%x.\r\n", regData);			
		}

    if(!spi2_dac_read_register(STATUS, &regData))  
		{
			  // da14531_printf("STATUS channel 4 register is: 0x%x.\r\n", regData);			
		}
				
//		// Reset the device
//		spi2_dac_write_register(TRIGGER, 0xa);
    
		spi2_dac_write_register(GAIN, 0x1ff);
    if(!spi2_dac_read_register(STATUS, &regData))  
		{
			  // da14531_printf("STATUS channel 4 register is: 0x%x.\r\n", regData);			
		}
    
		// spi2_dac_write_register(BRDCAST, 0xafff);
		
		const uint8_t DAC_CHS[8] = {DAC0, DAC1, DAC2, DAC3, DAC4, DAC5, DAC6, DAC7};
		for (int i = 0; i < 8; i++)
		{
				// da14531_printf("The value to be set to DAC channel %d is: 0x%x.\r\n", i, DACValBuf[i]);
				spi2_dac_write_register(DAC_CHS[i], DACValBuf[i]);
		}
}
