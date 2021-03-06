/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

/*
 * Pinout:
 * ARJ2	AD0	F14	MISO
 * 		AD1	F13	SCK
 * 		AD2	F12	SS
 * 		AD4	E12	MOSI
 */

/***************************** Include Files *********************************/

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"

#include "xparameters.h"	/* SDK generated parameters */
#include "xspips.h"		/* SPI device driver */

/************************** Constant Definitions *****************************/

/*
 * The following constant map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define SPI_DEVICE_ID		XPAR_XSPIPS_0_DEVICE_ID

/*
 * The following constant specify the max amount of data the slave is
 * expecting to receive from the master.
 */
#define MAX_DATA		64	// Should be < 128

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

#define SpiPs_RecvByte(BaseAddress) \
		(u8)XSpiPs_In32((BaseAddress) + XSPIPS_RXD_OFFSET)

#define SpiPs_SendByte(BaseAddress, Data) \
		XSpiPs_Out32((BaseAddress) + XSPIPS_TXD_OFFSET, (Data))

/************************** Function Prototypes ******************************/

void SpiSlaveRead(int ByteCount);

void SpiSlaveWrite(u8 *Sendbuffer, int ByteCount);

int SpiPsSlavePolledExample(u16 SpiDeviceId);

/************************** Variable Definitions *****************************/

/*
 * The instances to support the device drivers are global such that they
 * are initialized to zero each time the program runs. They could be local
 * but should at least be static so they are zeroed.
 */
static XSpiPs SpiInstance;

/*
 * The ReadBuffer is used to read to the data which it received from the SPI
 * Bus which master has sent.
 */
u8 ReadBuffer[MAX_DATA];

/*****************************************************************************/
/**
*
* Main function to call the SPI Slave Example.
*
* @param	None
*
* @return
*		- XST_SUCCESS if successful
*		- XST_FAILURE if not successful
*
* @note		None
*
******************************************************************************/
int main()
{
	int Status;

    init_platform();

	xil_printf("Running SpiPS Slave Polled Example \r\n");

	/*
	 * Run the SpiPs Slave Polled example.
	 */
	Status = SpiPsSlavePolledExample(SPI_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("SpiPs Slave Polled Example Failed \r\n");
		return XST_FAILURE;
	}

	xil_printf("Successfully ran SpiPs Slave Polled Example \r\n");

    cleanup_platform();
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* The purpose of this function is to illustrate how to use the XSpiPs
* device driver in Slave mode. This function reads data from a SPI Master
* and will echo it back to the Master.
*
* @param	SpiDeviceId is the Instance Id of SPI in the system.
*
* @return
*		- XST_SUCCESS if successful
*		- XST_FAILURE if not successful
*
* @note		None
*
*
*****************************************************************************/
int SpiPsSlavePolledExample(u16 SpiDeviceId)
{
	int Status;
	u8 *BufferPtr;
	XSpiPs_Config *SpiConfig;

	/*
	 * Initialize the SPI driver so that it's ready to use
	 */
	xil_printf("SPI slave: Initializing...\r\n");
	SpiConfig = XSpiPs_LookupConfig(SpiDeviceId);
	if (NULL == SpiConfig) {
		return XST_FAILURE;
	}

	xil_printf("SPI slave: Setting configurations...\r\n");
	Status = XSpiPs_CfgInitialize((&SpiInstance), SpiConfig,
					SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * The SPI device is a slave by default and the clock phase
	 * have to be set according to its master. In this example, CPOL is set
	 * to quiescent high and CPHA is set to 1.
	 */
	xil_printf("SPI slave: Setting SPI mode...\r\n");
	Status = XSpiPs_SetOptions((&SpiInstance), (XSPIPS_CR_CPHA_MASK) | \
			(XSPIPS_CR_CPOL_MASK));
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	xil_printf("SPI slave: Setting ReadBuffer...\r\n");
	memset(ReadBuffer, 0x00, sizeof(ReadBuffer));

	/*
	 * Set the Rx FIFO Threshold to the Max Data
	 */
	xil_printf("SPI slave: Setting Rx FIFO Threshold...\r\n");
	XSpiPs_SetRXWatermark((&SpiInstance),MAX_DATA);

	/*
	 * Enable the device.
	 */
	xil_printf("SPI slave: Enabling...\r\n");
	XSpiPs_Enable((&SpiInstance));

	xil_printf("SPI slave initialized.\r\n");

	/*
	 * Read the contents of the Receive buffer
	 * Master is expected to send MAX_DATA number of bytes
	 */
	xil_printf("Reading from SPI master...\r\n");
	SpiSlaveRead(MAX_DATA);

	for(int Count = 0; Count < MAX_DATA; Count++){
		xil_printf("%03d ", ReadBuffer[Count]);
		if ((Count & 0x001F) == 0x001F) xil_printf("\r\n");
	}
	xil_printf("\r\n");

	/*
	 * Setup a pointer to the start of the data that was read into the read
	 * buffer and the same back
	 */
//	BufferPtr = ReadBuffer;

	/*
	 * Send the data received back to Master
	 * Master is expected to send MAX_DATA number of dummy bytes for
	 * the slave to be able to echo previously received data.
	 */
//	SpiSlaveWrite(BufferPtr, MAX_DATA);

	/*
	 * Disable the device.
	 */
	 XSpiPs_Disable((&SpiInstance));

//	XSpiPs_Abort((&SpiInstance));
//	XSpiPs_Reset((&SpiInstance));

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function reads from the Rx buffer
*
* @param	ByteCount is the number of bytes to be read from Rx buffer.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void SpiSlaveRead(int ByteCount)
{
	int Count;
	u32 StatusReg;

	StatusReg = XSpiPs_ReadReg(SpiInstance.Config.BaseAddress,
					XSPIPS_SR_OFFSET);

	/*
	 * Polling the Rx Buffer for Data
	 */
	do{
		StatusReg = XSpiPs_ReadReg(SpiInstance.Config.BaseAddress,
					XSPIPS_SR_OFFSET);
	}while(!(StatusReg & XSPIPS_IXR_RXNEMPTY_MASK));

	/*
	 * Reading the Rx Buffer
	 */
	for(Count = 0; Count < ByteCount; Count++){
		ReadBuffer[Count] = SpiPs_RecvByte(
				SpiInstance.Config.BaseAddress);
//		xil_printf("Byte #%03d: %03d\r\n", Count, ReadBuffer[Count]);
	}

}

/*****************************************************************************/
/**
*
* This function writes Data into the Tx buffer
*
* @param	Sendbuffer is the buffer whose data is to be sent onto the
* 		Tx FIFO.
* @param	ByteCount is the number of bytes to be read from Rx buffer.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void SpiSlaveWrite(u8 *Sendbuffer, int ByteCount)
{
	u32 StatusReg;
	int TransCount = 0;

	StatusReg = XSpiPs_ReadReg(SpiInstance.Config.BaseAddress,
				XSPIPS_SR_OFFSET);

	/*
	 * Fill the TXFIFO with as many bytes as it will take (or as
	 * many as we have to send).
	 */
	while ((ByteCount > 0) &&
		(TransCount < XSPIPS_FIFO_DEPTH)) {
		SpiPs_SendByte(SpiInstance.Config.BaseAddress,
				*Sendbuffer);
		Sendbuffer++;
		++TransCount;
		ByteCount--;
	}

	/*
	 * Wait for the transfer to finish by polling Tx fifo status.
	 */
	do {
		StatusReg = XSpiPs_ReadReg(
				SpiInstance.Config.BaseAddress,
					XSPIPS_SR_OFFSET);
	} while ((StatusReg & XSPIPS_IXR_TXOW_MASK) == 0);

}
