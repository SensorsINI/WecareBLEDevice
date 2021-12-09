/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup SPI
 * @{
 *
 * @file spi.h
 *
 * @brief SPI driver header file.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _SPI_H_
#define _SPI_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if defined (__DA14531__)
    #include "spi_531.h"
#else
    #include "spi_58x.h"
#endif


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable clock for SPI peripheral.
 ****************************************************************************************
 */
void spi_enable(void);

/**
 ****************************************************************************************
 * @brief Disable SPI block and clock for SPI peripheral. Force CS to inactive state.
 ****************************************************************************************
 */
void spi_disable(void);

/**
 ****************************************************************************************
 * @brief Set SPI word size (on-the-fly).
 * @param[in] spi_wsz           SPI_WSZ_MODE_CFG (8, 16, 32 or 9 bits width)
 ****************************************************************************************
 */
void spi_set_bitmode(SPI_WSZ_MODE_CFG spi_wsz);

/**
 ****************************************************************************************
 * @brief Register SPI callback (on-the-fly). Callback to return in interrupt/DMA-driven
 * sending operations.
 * @param[in] cb                pointer to callback of spi_cb_t type
 ****************************************************************************************
 */
void spi_register_send_cb(spi_cb_t cb);

/**
 ****************************************************************************************
 * @brief Register SPI callback (on-the-fly). Callback to return in interrupt/DMA-driven
 * receiving operations.
 * @param[in] cb                pointer to callback of spi_cb_t type
 ****************************************************************************************
 */
void spi_register_receive_cb(spi_cb_t cb);

/**
 ****************************************************************************************
 * @brief Register SPI callback (on-the-fly). Callback to return in interrupt/DMA-driven
 * transferring operations.
 * @param[in] cb                pointer to callback of spi_cb_t type
 ****************************************************************************************
 */
void spi_register_transfer_cb(spi_cb_t cb);

/**
 ****************************************************************************************
 * @brief Initialize SPI peripheral with selected configuration.
 *        If DMA is supported, it will also initialize the DMA engine for the SPI peripheral
 * @note In the case of DA14585/DA14586, the PD_PER domain shall be functional
 * prior to SPI initialization:
 * SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
 * while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));
 * @param[in] spi_cfg       SPI configuration struct of spi_cfg_t type
 ****************************************************************************************
 */
int8_t spi_initialize(const spi_cfg_t *spi_cfg);

/**
 ****************************************************************************************
 * @brief Activate the configured Chip Select line
 ****************************************************************************************
 */
void spi_cs_low(void);

/**
 ****************************************************************************************
 * @brief Deactivate the configured Chip Select line
 ****************************************************************************************
 */
void spi_cs_high(void);

/**
 ****************************************************************************************
 * @brief Start sending data to SPI
 * @param[in] data      Pointer to buffer with data to send to SPI. It can be a pointer to
 *                      uint8_t, uint16_t or uint32_t data type depending on SPI
 *                      configuration.
 * @param[in] num       Number of data items to send. (Must be non-zero)
 * @param[in] op        Blocking, interrupt-driven or DMA-driven operation.
 * @note In 16 and 32 bitmodes data are transferred in big endian format.
 * @note DMA operation is supported only for 8 and 16 bitmode transfers.
 * @return status code
 ****************************************************************************************
 */
int8_t spi_send(const void *data, uint16_t num, SPI_OP_CFG op);

/**
 ****************************************************************************************
 * @brief Start receiving data from SPI
 * @param[in] data      Pointer to buffer with data to receive from SPI. It can be a
 *                      pointer to uint8_t, uint16_t or uint32_t data type depending on SPI
 *                      configuration.
 * @param[in] num       Number of data items to receive. (Must be non-zero)
 * @param[in] op        Blocking, interrupt-driven or DMA-driven operation.
 * @note In 16 and 32 bitmodes data are transferred in big endian format.
 * @note DMA operation is supported only for 8 and 16 bitmode transfers.
 * @return status code
 ****************************************************************************************
 */
int8_t spi_receive(void *data, uint16_t num, SPI_OP_CFG op);

/**
 ****************************************************************************************
 * @brief Start sending/receiving data to/from SPI - Full Duplex.
 * @param[in] data_out  Pointer to buffer with data to send to SPI. It can be a pointer to
 *                      uint8_t, uint16_t or uint32_t data type depending on SPI
 *                      configuration.
 * @param[in] data_in   Pointer to buffer with data to receive from SPI. It can be a
 *                      pointer to uint8_t, uint16_t or uint32_t data type depending on SPI
 *                      configuration.
 * @param[in] num       Number of data items to transfer. (Must be non-zero)
 * @param[in] op        Blocking, interrupt-driven or DMA-driven operation.
 * @note In 16 and 32 bitmodes data are transferred in big endian format.
 * @note DMA operation is supported only for 8 and 16 bitmode transfers.
 * @return status code
 ****************************************************************************************
 */
int8_t spi_transfer(const void *data_out, void *data_in, uint16_t num, SPI_OP_CFG op);

// Legacy API support

/**
 ****************************************************************************************
 * @brief SPI master access to slave without acting on CS (works in all bit modes).
 * @param[in] dataToSend Data to send
 * @return data read
 ****************************************************************************************
 */
uint32_t spi_access(uint32_t dataToSend);

/**
 ****************************************************************************************
 * @brief SPI master transaction to slave with CS handling (works in all bit modes).
 * @param[in] dataToSend Data to send
 * @return data read
 ****************************************************************************************
 */
uint32_t spi_transaction(uint32_t dataToSend);

/**
 ****************************************************************************************
 * @brief Releases SPI.
 ****************************************************************************************
 */
#define spi_release() spi_disable()

#if defined (CFG_SPI_DMA_SUPPORT)

/**
 ****************************************************************************************
 * @brief Waits for DMA write to finish the current transfer.
 ****************************************************************************************
 */
void spi_wait_dma_write_to_finish(void);

/**
 ****************************************************************************************
 * @brief Waits for DMA read to finish the current transfer.
 ****************************************************************************************
 */
void spi_wait_dma_read_to_finish(void);

#endif // (CFG_SPI_DMA_SUPPORT)

/**
 ****************************************************************************************
 * @brief Set SPI mode.
 * @param[in] spi_cp the SPI mode (CPOL, CPHA).
 ****************************************************************************************
 */
void spi_set_cp_mode(SPI_CP_MODE_CFG spi_cp);

/**
 ****************************************************************************************
 * @brief Set SPI speed.
 * @param[in] spi_speed the SPI speed mode.
 ****************************************************************************************
 */
void spi_set_speed(SPI_SPEED_MODE_CFG spi_speed);

/**
 ****************************************************************************************
 * @brief SPI Master transfer data.
 * @param[in] tx_buf    Pointer to buffer with data to send to SPI. It can be a pointer to
 *                      uint8_t, uint16_t or uint32_t data type depending on SPI
 *                      configuration.
 * @param[in] rx_buf    Pointer to buffer with data to receive from SPI. It can be a
 *                      pointer to uint8_t, uint16_t or uint32_t data type depending on SPI
 *                      configuration.
 * @param[in] length    Number of data items to transfer
 * @param[in] op        Blocking or DMA-driven operation.
 * @return status
 * @note In 16 and 32 bitmodes data are transferred in big endian format.
 * @note DMA operation is supported only for 8 and 16 bitmode transfers.
 ****************************************************************************************
 */
int8_t spi_master_transfer(const void *tx_buf, void *rx_buf, uint16_t length, SPI_OP_CFG op);

/**
 ****************************************************************************************
 * @brief Set SPI chip select mode
 * @param[in] spi_cs the SPI chip select mode
 ****************************************************************************************
 */
void spi_set_cs_mode(SPI_CS_MODE_CFG spi_cs);

#endif // _SPI_H_

///@}
///@}
