#include "user_periph_setup.h"
#include "da14531_printf.h"

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

/**
 ****************************************************************************************
 * @brief SPI2 ADC1 module (MCP3564R) init function.
 * @return void
 ****************************************************************************************
*/
void spi2_adc1_init(void)
{	
	  uint32_t regVal;
	  uint8_t sendBuf[TOTAL_BYTES] = {0};
	  uint8_t receiveBuf[TOTAL_BYTES] = {0}; 
 
		// Full reset
		spi2_adc_fast_command(FAST_CMD_FULL_RESET);

		// Read all the register values.
		spi2_adc_increment_read_register(ADCDATA, receiveBuf, TOTAL_BYTES);
		
		// Set PRE[1:0] to 1 (default) and OSR to some big enough value because ADC conversion rate is faster than SPI speed.
		// Here we set OSR to 20480. That is the minimum setting we tested by experiments.
		sendBuf[0] = 0x28;
		spi2_adc_write_register(CONFIG1, sendBuf, CONFIG1_BYTES);	
		
//		// Configure CONFIG2 register: GAIN('000'): set gain to 1/3.
//		sendBuf[0] = 0x83; 
//		spi2_adc_write_register(CONFIG2, sendBuf, CONFIG2_BYTES);	
		
		// Configure CONFIG3 register: CONV_MODE('11'): Continous conversion in scan mode. DATA_FORMAT('11'): 32bit with channel ID.
		sendBuf[0] = 0xF0; 
		spi2_adc_write_register(CONFIG3, sendBuf, CONFIG3_BYTES);	
		
		// Configure IRQ regiseter: IRQ_Mode('01'): IRQ output and inactive state is logic high
		// This configuration is important, because if we use the default configuration, then
		// IRQ is in high-Z state and because we don't have external pull-up resistor on board.
		// Therefore, IRQ cannot generate falling edge and SDO cannot be updated resulting in 
		// SPI reading on ADCDATA always 0.
		sendBuf[0] = 0x07;
		spi2_adc_write_register(IRQ, sendBuf, IRQ_BYTES);
		
		// Configure MUX register: AVDD reading
		sendBuf[0] = 0x98;
		spi2_adc_write_register(MUX, sendBuf, MUX_BYTES);
		
		// Configure SCAN register: 4 Differential Channels plus 8 Single-Ended Channels.
		sendBuf[0] = 0x00;
		sendBuf[1] = 0x0F;
		sendBuf[2] = 0xFF;
		spi2_adc_write_register(SCAN, sendBuf, SCAN_BYTES);

		// Configure CONFIG0 register: Internal V_ref and internal master clock, no bias and ADC conversion mode.
		// This register configuration should be put in the last of init function as this register is used to start
		// the conversion.
		sendBuf[0] = 0xF3;  // The first byte is the MSbs for registers have more than 8bits
		// sendBuf[1] = 0x02;
		// sendBuf[2] = 0x03;
		spi2_adc_write_register(CONFIG0, sendBuf, CONFIG0_BYTES);
		
		// Read CONFIG0 register.
		spi2_adc_static_read_register(CONFIG0, receiveBuf, CONFIG0_BYTES);	
		regVal = swapBufToRealVal(receiveBuf, CONFIG0_BYTES);
		da14531_printf("CONFIG0 register data is: 0x%x.\r\n", regVal);
}


/**
 ****************************************************************************************
 * @brief SPI2 ADC1 module (MCP3564R) test timer callback function.
 * @return bool. true: valid conversion; false: invalid conversion
 ****************************************************************************************
*/
void spi2_adc1_readout(uint32_t *ADCReadValBuf)
{ 
		static bool initFlag = true;   // Make sure initilization only execute once
		static uint32_t errorCnt = 0;
		static uint32_t adcConversionCount = 0;
    spi_initialize(&spi2_adc1_cfg);
	
    uint32_t regVal = 0;	
		uint8_t sendBuf[TOTAL_BYTES] = {0};	
    uint8_t receiveBuf[TOTAL_BYTES] = {0}; 
 
		if(initFlag)
		{			
				spi2_adc1_init();		
				initFlag = false;
		}

//		da14531_printf("Reset the watchdog.\r\n");
//		wdg_reload(WATCHDOG_DEFAULT_PERIOD);		

//		//Disable the interrupts
//		GLOBAL_INT_STOP();	
		
		adcConversionCount++;
		da14531_printf("Start the %dth ADC conversion. \r\n", adcConversionCount);
		
		// Start ADC conversion
		spi2_adc_fast_command(FAST_CMD_START_CONVERSION);
	
		spi2_adc_static_read_register(CONFIG2, receiveBuf, CONFIG2_BYTES);	
		regVal = swapBufToRealVal(receiveBuf, CONFIG2_BYTES);
    // Obtain the GAIN bits value
		uint8_t gainReg = (regVal & 0x38) >> 3;
	  // Convert it to the real gain
		float gainFactor = 0.333;  // if gainReg == 0, then gainFactor is 1/3
		if(gainReg != 0)
		{
				gainFactor = 1 << (gainReg - 1);
		}		
		
		float voltage[12];
		bool validFlg = true; // Check if this conversion valid
		for(int i = 0; i < 12; i++)
		{		
				sendBuf[0] = ADC_CHANNEL_ID[i];
				spi2_adc_write_register(MUX, sendBuf, MUX_BYTES);		
			
				// Read IRQ register
				spi2_adc_static_read_register(IRQ, receiveBuf, IRQ_BYTES);
				uint8_t irqVal = swapBufToRealVal(receiveBuf, IRQ_BYTES);
			  // Wait the conversion finish
				while((irqVal & 0x40) != 0)
				{
						spi2_adc_static_read_register(IRQ, receiveBuf, IRQ_BYTES);
						irqVal = swapBufToRealVal(receiveBuf, IRQ_BYTES);
				}			
				
				// STATIC Read ADCDATA
				spi2_adc_static_read_register(ADCDATA, receiveBuf, ADCDATA_BYTES);	
				regVal = swapBufToRealVal(receiveBuf, ADCDATA_BYTES);
				volatile uint32_t channelID = (regVal >> 28) & 0xF;
				regVal = (regVal & 0xFFFFFFF) + (((regVal >> 24) & 0xF) << 28);
				int32_t voltageVal = (int32_t)(regVal);
				voltage[channelID] = voltageVal/(0x800000 * gainFactor) * 2.4;    // The internal reference voltage is 2.4V
				da14531_printf("The voltage of channel ID %d is: %.4fV.\r\n",  channelID, voltage[channelID]);
				
				if ((channelID + i) != 11)
				{
						da14531_printf("ADC conversion error.\r\n");
						validFlg = false;
				}
		}
		
		if(validFlg == false)
		{
				errorCnt++;
		}
		
		// Copy voltage hex buffer to value shared with BLE for sending to the host
		memcpy(ADCReadValBuf, voltage, sizeof(float) * 12);			
    ADCReadValBuf[12] = adcConversionCount; // Store the ADC conversion count
		ADCReadValBuf[13] = errorCnt;
		ADCReadValBuf[14] = validFlg;           // Indicate the current conversion valid or not
		
		// Shutdown ADC to save power after conversion
		spi2_adc_fast_command(FAST_CMD_ADC_SHUTDOWN);
		
		da14531_printf("ADC conversion error count is %d.\r\n", errorCnt);
		
//		// restore interrupts
//		GLOBAL_INT_START();		
}
