/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup SPI_Flash SPI Flash
 * @brief SPI Flash driver API
 * @{
 *
 * @file spi_flash.h
 *
 * @brief SPI Flash driver header file.
 *
 * @details Tested SPI Flash devices:
 *          - Winbond: W25x10, W25x20
 *          - Adesto/Dialog: AT25DN011, AT25DF011, AT25DF021A, AT25EU0021A, AT25XE041D
 *          - Macronix: MX25R2035F (embedded in DA14586 as well)
 *          - Puya: P25Q10U
 *          - GigaDevice: GD25WD20
 *
 * Copyright (C) 2012-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "spi.h"
#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */

#define SPI_FLASH_DRIVER_VERSION (3)
#define SPI_FLASH_DRIVER_SUBVERSION (1)

#if (USE_SPI_FLASH_EXTENSIONS)
#if defined (CFG_WLCSP_PACKAGE)
#error "Control of external flash with extra GPIO pin is not supported in WLCSP package"
#endif
#endif

/// @name Global definitions for standard SPI Flash devices
///@{
/** Definition for standard SPI Flash devices */
#define SPI_FLASH_SECTOR_SIZE                   4096
#define SPI_FLASH_PAGE_SIZE                     256
#define SPI_FLASH_MEM_PROT_NONE                 0
#define SPI_FLASH_MEM_PROT_MASK                 0x7C
///@}

/// @name W25x10CL-specific definitions
///@{
/** W25x10CL-specific */
#define W25X10CL_MAN_DEV_ID                     0xEF10
#define W25X10CL_JEDEC_ID                       0xEF3011
#define W25X10CL_CHIP_SIZE                      0x20000
#define W25X10CL_MEM_PROT_UPPER_HALF            4
#define W25X10CL_MEM_PROT_LOWER_HALF            36
#define W25X10CL_MEM_PROT_ALL                   8
///@}

/// @name W25x20CL-specific definitions
///@{
/** W25x20CL-specific */
#define W25X20CL_MAN_DEV_ID                     0xEF11
#define W25X20CL_JEDEC_ID                       0xEF3012
#define W25X20CL_CHIP_SIZE                      0x40000
#define W25X20CL_MEM_PROT_UPPER_QUARTER         4
#define W25X20CL_MEM_PROT_UPPER_HALF            8
#define W25X20CL_MEM_PROT_LOWER_QUARTER         36
#define W25X20CL_MEM_PROT_LOWER_HALF            40
#define W25X20CL_MEM_PROT_ALL                   12
///@}

/// @name AT25DN011-/AT25DF011-specific definitions
///@{
/** AT25DN011-/AT25DF011-specific */
#define AT25DX011_JEDEC_ID                      0x1F4200
#define AT25DX011_CHIP_SIZE                     0x20000
#define AT25DX011_MEM_PROT_ALL                  0x04
///@}

/// @name AT25DF021A-specific definitions
///@{
/** AT25DF021A-specific */
#define AT25DF021A_JEDEC_ID                     0x1F4301
#define AT25DF021A_CHIP_SIZE                    0x40000
#define AT25DF021A_MEM_PROT_ALL                 0x7F
///@}

/// @name AT25EU0021A-specific definitions
///@{
/** AT25EU0021A-specific */
#define AT25EU0021A_JEDEC_ID                    0x1F1101
#define AT25EU0021A_CHIP_SIZE                   0x40000
#define AT25EU0021A_MEM_PROT_ALL                0x7C
///@}

/// @name AT25XE041D-specific definitions
///@{
/** AT25XE041D-specific */
#define AT25XE041D_JEDEC_ID                     0x1F440C
#define AT25XE041D_CHIP_SIZE                    0x80000
#define AT25XE041D_MEM_PROT_ALL                 0x1C
///@}

/// @name MX25V1006E-specific definitions
///@{
/** MX25V1006E-specific */
#define MX25V1006E_MAN_DEV_ID                   0xC210
#define MX25V1006E_JEDEC_ID                     0xC22011
#define MX25V1006E_CHIP_SIZE                    0x20000
#define MX25V1006E_MEM_PROT_ALL                 0x0C
///@}

/// @name MX25R1035F-specific definitions
///@{
/** MX25R1035F-specific */
#define MX25R1035F_MAN_DEV_ID                   0xC211
#define MX25R1035F_JEDEC_ID                     0xC22811
#define MX25R1035F_CHIP_SIZE                    0x20000
#define MX25R1035F_MEM_PROT_1_BLOCK             0x04
#define MX25R1035F_MEM_PROT_ALL                 0x0C
///@}

/// @name MX25R2035F-specific definitions
///@{
/** MX25R2035F-specific */
#define MX25R2035F_MAN_DEV_ID                   0xC212
#define MX25R2035F_JEDEC_ID                     0xC22812
#define MX25R2035F_CHIP_SIZE                    0x40000
#define MX25R2035F_MEM_PROT_1_BLOCK             0x04
#define MX25R2035F_MEM_PROT_2_BLOCK             0x08
#define MX25R2035F_MEM_PROT_ALL                 0x0C
///@}

/// @name MX25R4035F-specific definitions
///@{
/** MX25R4035F-specific */
#define MX25R4035F_MAN_DEV_ID                   0xC213
#define MX25R4035F_JEDEC_ID                     0xC22813
#define MX25R4035F_CHIP_SIZE                    0x80000
#define MX25R4035F_MEM_PROT_1_BLOCK             0x04
#define MX25R4035F_MEM_PROT_2_BLOCK             0x08
#define MX25R4035F_MEM_PROT_4_BLOCK             0x0C
#define MX25R4035F_MEM_PROT_ALL                 0x10
///@}

/// @name MX25R8035F-specific definitions
///@{
/** MX25R8035F-specific */
#define MX25R8035F_MAN_DEV_ID                   0xC214
#define MX25R8035F_JEDEC_ID                     0xC22814
#define MX25R8035F_CHIP_SIZE                    0x100000
#define MX25R8035F_MEM_PROT_1_BLOCK             0x04
#define MX25R8035F_MEM_PROT_2_BLOCK             0x08
#define MX25R8035F_MEM_PROT_4_BLOCK             0x0C
#define MX25R8035F_MEM_PROT_8_BLOCK             0x10
#define MX25R8035F_MEM_PROT_ALL                 0x14
///@}

/// @name MX25R1635F-specific definitions
///@{
/** MX25R1635F-specific */
#define MX25R1635F_MAN_DEV_ID                   0xC215
#define MX25R1635F_JEDEC_ID                     0xC22815
#define MX25R1635F_CHIP_SIZE                    0x200000
#define MX25R1635F_MEM_PROT_1_BLOCK             0x04
#define MX25R1635F_MEM_PROT_2_BLOCK             0x08
#define MX25R1635F_MEM_PROT_4_BLOCK             0x0C
#define MX25R1635F_MEM_PROT_8_BLOCK             0x10
#define MX25R1635F_MEM_PROT_16_BLOCK            0x14
#define MX25R1635F_MEM_PROT_ALL                 0x18
///@}

/// @name MX25Rxxxxx-specific definitions
///@{
/** MX25Rxxxxx-specific */
#define MX25R_MAN_TYP_ID 0xC228
///@}

/// @name MX25V1035F-specific definitions
///@{
/** MX25V1035F-specific */
#define MX25V1035F_MAN_DEV_ID                   0xC211
#define MX25V1035F_JEDEC_ID                     0xC22311
#define MX25V1035F_CHIP_SIZE                    0x20000
#define MX25V1035F_MEM_PROT_1_BLOCK             0x04
#define MX25V1035F_MEM_PROT_ALL                 0x0C
///@}

/// @name MX25V2035F-specific definitions
///@{
/** MX25V2035F-specific */
#define MX25V2035F_MAN_DEV_ID                   0xC212
#define MX25V2035F_JEDEC_ID                     0xC22312
#define MX25V2035F_CHIP_SIZE                    0x40000
#define MX25V2035F_MEM_PROT_1_BLOCK             0x04
#define MX25V2035F_MEM_PROT_2_BLOCK             0x08
#define MX25R2035F_MEM_PROT_ALL                 0x0C
///@}

/// @name MX25V4035F-specific definitions
///@{
/** MX25V4035F-specific */
#define MX25V4035F_MAN_DEV_ID                   0xC213
#define MX25V4035F_JEDEC_ID                     0xC22313
#define MX25V4035F_CHIP_SIZE                    0x80000
#define MX25V4035F_MEM_PROT_1_BLOCK             0x04
#define MX25V4035F_MEM_PROT_2_BLOCK             0x08
#define MX25V4035F_MEM_PROT_4_BLOCK             0x0C
#define MX25V4035F_MEM_PROT_ALL                 0x10
///@}

/// @name MX25V8035F-specific definitions
///@{
/** MX25V8035F-specific */
#define MX25V8035F_MAN_DEV_ID                   0xC214
#define MX25V8035F_JEDEC_ID                     0xC22314
#define MX25V8035F_CHIP_SIZE                    0x100000
#define MX25V8035F_MEM_PROT_1_BLOCK             0x04
#define MX25V8035F_MEM_PROT_2_BLOCK             0x08
#define MX25V8035F_MEM_PROT_4_BLOCK             0x0C
#define MX25V8035F_MEM_PROT_8_BLOCK             0x10
#define MX25V8035F_MEM_PROT_FULL                0x14
///@}

/// @name MX25V1635F-specific definitions
///@{
/** MX25V1635F-specific */
#define MX25V1635F_MAN_DEV_ID                   0xC215
#define MX25V1635F_JEDEC_ID                     0xC22315
#define MX25V1635F_CHIP_SIZE                    0x200000
#define MX25V1635F_MEM_PROT_1_BLOCK             0x04
#define MX25V1635F_MEM_PROT_2_BLOCK             0x08
#define MX25V1635F_MEM_PROT_4_BLOCK             0x0C
#define MX25V1635F_MEM_PROT_8_BLOCK             0x10
#define MX25V1635F_MEM_PROT_16_BLOCK            0x14
#define MX25V1635F_MEM_PROT_ALL                 0x18
///@}

/// @name MX25VxxxxF-specific definitions
///@{
/** MX25VxxxxF-specific */
#define MX25V_F_MAN_TYP_ID 0xC223
///@}

/// @name P25Q10U-specific definitions
///@{
/** P25Q10U-specific */
#define P25Q10U_MAN_DEV_ID                      0x8510
#define P25Q10U_JEDEC_ID                        0x856011
#define P25Q10U_CHIP_SIZE                       0x20000
#define P25Q10U_MEM_PROT_UPPER_HALF             4
#define P25Q10U_MEM_PROT_LOWER_HALF             36
#define P25Q10U_MEM_PROT_ALL                    8
///@}

/// @name P25Q11U-specific definitions
///@{
/** P25Q11U-specific */
#define P25Q11U_MAN_DEV_ID                      0x8510
#define P25Q11U_JEDEC_ID                        0x854011
#define P25Q11U_CHIP_SIZE                       0x20000
///@}

/// @name P25Q40U-specific definitions
///@{
/** P25Q40U-specific */
#define P25Q40U_MAN_DEV_ID                      0x8512
#define P25Q40U_JEDEC_ID                        0x856013
#define P25Q40U_CHIP_SIZE                       0x80000
#define P25Q40U_MEM_PROT_UPPER_EIGHTH           4
#define P25Q40U_MEM_PROT_UPPER_QUARTER          8
#define P25Q40U_MEM_PROT_UPPER_HALF             12
#define P25Q40U_MEM_PROT_LOWER_EIGHTH           36
#define P25Q40U_MEM_PROT_LOWER_QUARTER          40
#define P25Q40U_MEM_PROT_LOWER_HALF             44
#define P25Q40U_MEM_PROT_ALL                    16
///@}

/// @name GD25WD10-specific definitions
///@{
/** GD25WD10-specific */
#define GD25WD10_MAN_DEV_ID                     0xC810
#define GD25WD10_JEDEC_ID                       0xC86411
#define GD25WD10_CHIP_SIZE                      0x20000
#define GD25WD10_MEM_PROT_LOWER_HALF            16
#define GD25WD10_MEM_PROT_ALL                   20
///@}

/// @name GD25WD20-specific definitions
///@{
/** GD25WD20-specific */
#define GD25WD20_MAN_DEV_ID                     0xC811
#define GD25WD20_JEDEC_ID                       0xC86412
#define GD25WD20_CHIP_SIZE                      0x40000
#define GD25WD20_MEM_PROT_LOWER_HALF            20
#define GD25WD20_MEM_PROT_ALL                   24
///@}


/// Device index of supported SPI flash memories
typedef enum
{
    /// W25x10CL device index
    W25X10CL_DEV_INDEX = 1,

    /// W25x20CL device index
    W25X20CL_DEV_INDEX,

    /// AT25DN011/AT25DF011 device index
    AT25DX011_DEV_INDEX,

    /// MX25V1006E device index
    MX25V1006E_DEV_INDEX,

    /// MX25R1035F device index
    MX25R1035F_DEV_INDEX,

    /// MX25R2035F device index
    MX25R2035F_DEV_INDEX,

    /// MX25R4035F device index
    MX25R4035F_DEV_INDEX,

    /// MX25R8035F device index
    MX25R8035F_DEV_INDEX,

    /// MX25R1635F device index
    MX25R1635F_DEV_INDEX,

    /// MX25V1035F device index
    MX25V1035F_DEV_INDEX,

    /// MX25V2035F device index
    MX25V2035F_DEV_INDEX,

    /// MX25V4035F device index
    MX25V4035F_DEV_INDEX,

    /// MX25V8035F device index
    MX25V8035F_DEV_INDEX,

    /// MX25V1635F device index
    MX25V1635F_DEV_INDEX,

    /// P25Q10U device index
    P25Q10U_DEV_INDEX,

    /// P25Q40U device index
    P25Q40U_DEV_INDEX,

    /// GD25WD10 device index
    GD25WD10_DEV_INDEX,

    /// GD25WD20 device index
    GD25WD20_DEV_INDEX,

    /// AT25DF021A device index
    AT25DF021A_DEV_INDEX,

    /// AT25EU0021A device index
    AT25EU0021A_DEV_INDEX,

    /// AT25XE041D device index
    AT25XE041D_DEV_INDEX,
} spi_flash_dev_index_t;

/// @name Timeout definitions
///@{
/** Timeout */
#define SPI_FLASH_WAIT                          2000000
///@}

/// @name SPI Flash error codes
///@{
/** SPI Flash error code */
#define SPI_FLASH_ERR_OK                        (0)
#define SPI_FLASH_ERR_TIMEOUT                   (-1)
#define SPI_FLASH_ERR_NOT_ERASED                (-2)
#define SPI_FLASH_ERR_PROTECTED                 (-3)
#define SPI_FLASH_ERR_INVAL                     (-4)
#define SPI_FLASH_ERR_ALIGN                     (-5)
#define SPI_FLASH_ERR_UNKNOWN_FLASH_VENDOR      (-6)
#define SPI_FLASH_ERR_UNKNOWN_FLASH_TYPE        (-7)
#define SPI_FLASH_ERR_PROG_ERROR                (-8)
#define SPI_FLASH_ERR_READ_ERROR                (-9)
#define SPI_FLASH_ERR_NOT_DETECTED              (-10)
#define SPI_FLASH_ERR_AUTODETECT_ERROR          (-11)
#define SPI_FLASH_ERR_WEL_ERROR                 (-12)
#define SPI_FLASH_ERR_ERASE_ERROR               (-13)
#define SPI_FLASH_ERR_BUSY                      (-14)
///@}

/// @name SPI Flash Status Register bit fields
///@{
/** SPI Flash Status Register bit field */
#define SPI_FLASH_SR_BUSY                       0x01
#define SPI_FLASH_SR_WEL                        0x02
#define SPI_FLASH_SR_BP0                        0x04
#define SPI_FLASH_SR_BP1                        0x08
#define SPI_FLASH_SR_BP2                        0x10
#define SPI_FLASH_SR_BP3                        0x20
#define SPI_FLASH_SR_BP4                        0x40
#define SPI_FLASH_SR_TB                         0x20
#define SPI_FLASH_SR_SRP                        0x80
///@}

/// @name SPI Flash Security Register bit fields
///@{
/** SPI Flash Security Register bit field */
#define SPI_FLASH_SCR_PFAIL                     5
#define SPI_FLASH_SCR_EFAIL                     6
///@}

/// @name SPI Flash power modes
///@{
/** SPI Flash Power Mode */
#define SPI_FLASH_PM_LPM                        0x00
#define SPI_FLASH_PM_HPM                        0x02
///@}

/// @name Standard SPI Flash commands
///@{
/** Write Enable */
#define SPI_FLASH_OP_WREN                       0x06
/** Write Enable volatile */
#define SPI_FLASH_OP_VWREN                      0x50
/** Write Disable */
#define SPI_FLASH_OP_WRDI                       0x04
/** Read Status Register */
#define SPI_FLASH_OP_RDSR                       0x05
/** Write Status Register */
#define SPI_FLASH_OP_WRSR                       0x01
/** Read Configuration Register */
#define SPI_FLASH_OP_RDCFGR                     0x15
/** Read Security Register */
#define SPI_FLASH_OP_RDSCUR                     0x2B
/** Power Down */
#define SPI_FLASH_OP_DP                         0xB9
/** Release from Power Down */
#define SPI_FLASH_OP_RDP                        0xAB
/** Read Manufacturer and Device ID */
#define SPI_FLASH_OP_REMS                       0x90
/** Read Unique ID */
#define SPI_FLASH_OP_RUID                       0x4B
/** Read JEDEC ID */
#define SPI_FLASH_OP_RDID                       0x9F
/** Page Program */
#define SPI_FLASH_OP_PP                         0x02
/** Read Data */
#define SPI_FLASH_OP_READ                       0x03
/** Page Erase */
#define SPI_FLASH_OP_PE                         0x81
/** Chip Erase */
#define SPI_FLASH_OP_CE                         0xC7
/** Chip Erase Alternative */
#define SPI_FLASH_OP_CE2                        0x60
/** Ultra Deep Power-Down */
#define SPI_FLASH_OP_UDPD                       0x79
///@}

/// Block/Sector erase commands
typedef enum
{
    /// Sector erase
    SPI_FLASH_OP_SE   =                         0x20,

    /// Block32 erase
    SPI_FLASH_OP_BE32  =                        0x52,

    /// Block64 erase
    SPI_FLASH_OP_BE64  =                        0xD8,
} spi_flash_op_t;

#if defined (__DA14586__)
    /// @name DA14586 SPI Flash configuration
    ///@{
    /** SPI_EN port */
    #define SPI_EN_PORT                         GPIO_PORT_2
    /** SPI_EN pin */
    #define SPI_EN_PIN                          GPIO_PIN_3

    /** SPI_CLK port */
    #define SPI_CLK_PORT                        GPIO_PORT_2
    /** SPI_CLK pin */
    #define SPI_CLK_PIN                         GPIO_PIN_0

    /** SPI_DO port */
    #define SPI_DO_PORT                         GPIO_PORT_2
    /** SPI_DO pin */
    #define SPI_DO_PIN                          GPIO_PIN_9

    /** SPI_DI port */
    #define SPI_DI_PORT                         GPIO_PORT_2
    /** SPI_DI pin */
    #define SPI_DI_PIN                          GPIO_PIN_4

    /** Memory size in bytes */
    #define SPI_FLASH_DEV_SIZE                  MX25R2035F_CHIP_SIZE
    ///@}
#endif

/// SPI Flash configuration struct
typedef struct
{
    /// Device index on supported device list
    uint8_t dev_index;

    /// JEDEC ID
    uint32_t jedec_id;

    /// SPI Flash total size in bytes
    uint32_t chip_size;
} spi_flash_cfg_t;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if SPI Flash is busy
 * @return Status       SPI_FLASH_ERR_OK if device is ready,
 *                      SPI_FLASH_ERR_BUSY if device is busy or not attached
 ****************************************************************************************
 */
int8_t spi_flash_is_busy(void);

/**
 ****************************************************************************************
 * @brief Wait until SPI Flash is ready
 * @details Reads the busy signal of the Status Register until it is ready. If Status
 *          Register is still busy after SPI_FLASH_WAIT reads it will time out.
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_wait_till_ready(void);

/**
 ****************************************************************************************
 * @brief Enable the SPI flash memory. The user has to provide the SPI flash memory
 *        configuration.
 * @param[in] spi_cfg           Pointer to the configuration struct of the SPI block
 * @param[in] spi_flash_cfg     Pointer to the struct with the SPI flash configuration
 * @return Error code
 ****************************************************************************************
*/
int8_t spi_flash_enable(const spi_cfg_t *spi_cfg, const spi_flash_cfg_t *spi_flash_cfg);

/**
 ****************************************************************************************
 * @brief Enable the SPI flash memory. The SPI flash memory configuration will be
 *        auto-detected.
 * @param[in] spi_cfg           Pointer to the configuration struct of the SPI block
 * @param[out] dev_id           Index in the look-up table which contains the known SPI
 *                              flash memory configurations.
 * @return Error code
 ****************************************************************************************
*/
int8_t spi_flash_enable_with_autodetect(const spi_cfg_t *spi_cfg, uint8_t *dev_id);

/**
 ****************************************************************************************
 * @brief Configure SPI Flash environment with device's parameters
 * @param[in] spi_flash_cfg Pointer to the struct with SPI Flash device's parameters
 ****************************************************************************************
 */
void spi_flash_configure_env(const spi_flash_cfg_t *spi_flash_cfg);

/**
 ****************************************************************************************
 * @brief Try to auto-detect an SPI Flash device
 * @details Auto-detect is performed based on the JEDEC ID. If the device has been
 *          successfully identified then its parameters (device index and chip size) will
 *          be derived from a known devices look-up table.
 * @param[out] dev_id       Device index found
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_auto_detect(uint8_t *dev_id);

/**
 ****************************************************************************************
 * @brief Sends the Power-Down instruction
 * @note The function spi_flash_release_from_powerdown() is used to enable the IC again
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_power_down(void);

/**
 ****************************************************************************************
 * @brief Sends the Release from Power-Down instruction
 * @details This function is used to restore the IC from power-down mode
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_release_from_power_down(void);

/**
 ****************************************************************************************
 * @brief Sends the Ultra Deep Power-Down instruction
 * @details This function is used to set the IC to UDPD mode
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_ultra_deep_power_down(void);

/**
 ****************************************************************************************
 * @brief Read SPI Flash Status Register
 * @return Status Register value
 ****************************************************************************************
 */
uint16_t spi_flash_read_status_reg(void);

/**
 ****************************************************************************************
 * @brief Issue a Write Enable Command
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_set_write_enable(void);

/**
 ****************************************************************************************
 * @brief Issue a Write Enable Volatile Command
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_write_enable_volatile(void);

/**
 ****************************************************************************************
 * @brief Issue a Write Disable Command
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_set_write_disable(void);

/**
 ****************************************************************************************
 * @brief Write SPI Flash Status Register
 * @param[in] data          Value to be written to Status Register
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_write_status_reg(uint8_t data);

/**
 ****************************************************************************************
 * @brief Read Configuration Register
 * @param[out] data         Content of Configuration Register
 ****************************************************************************************
 */
void spi_flash_read_config_reg(uint32_t *data);

/**
 ****************************************************************************************
 * @brief Write Status & Config register
 * @param[in] data          Value to be written to Status and Configuration Register
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_write_status_config_reg(uint32_t data);

/**
 ****************************************************************************************
 * @brief Read Security Register
 * @param[out] data         Content of Configuration Register
 ****************************************************************************************
 */
void spi_flash_read_security_reg(uint32_t *data);

/**
 ****************************************************************************************
 * @brief Get Power Mode
 * @param[out] data         Current power mode
 ****************************************************************************************
 */
void spi_flash_get_power_mode(uint32_t *data);

/**
 ****************************************************************************************
 * @brief Set Power Mode
 * @param[in] data          Low Power Mode (LPM) or High Performance Mode (HPM)
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_set_power_mode(uint8_t data);

/**
 ****************************************************************************************
 * @brief Get SPI Flash Manufacturer/Device ID
 * @param[out] data         Pointer to the Manufacturer/Device ID
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_read_rems_id(uint16_t *data);

/**
 ****************************************************************************************
 * @brief Get SPI Flash Unique ID Number
 * @param[out] data         Pointer to the Unique ID Number
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_read_unique_id(uint64_t *data);

/**
 ****************************************************************************************
 * @brief Get SPI Flash JEDEC ID
 * @param[out] data         Pointer to the JEDEC ID
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_read_jedec_id(uint32_t *data);

/**
 ****************************************************************************************
 * @brief Set the memory protection configuration
 * @param[in] data          Memory protection setting
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_configure_memory_protection(uint8_t data);

/**
 ****************************************************************************************
 * @brief Issue a Page Erase command
 * @param[in] page_address  Address of the page to erase
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_page_erase(uint32_t page_address);

/**
 ****************************************************************************************
 * @brief Issue a comamnd to erase a given sector or block
 * @param[in] address       Address that belongs to the block64/block32/sector range
 * @param[in] erase_op      Erase operation type (Sector, Block32 or Block64)
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_block_erase(uint32_t address, spi_flash_op_t erase_op);

/**
 ****************************************************************************************
 * @brief Issue a command to erase a given sector or block without waiting for the
 *        operation to complete
 * @param[in] address       Address that belongs to the block64/block32/sector range
 * @param[in] erase_op      Erase operation type (Sector, Block32 or Block64)
 * @return Error code
****************************************************************************************
*/
int8_t spi_flash_block_erase_no_wait(uint32_t address, spi_flash_op_t erase_op);

/**
 ****************************************************************************************
 * @brief Erase chip
 * @note In order for the erase to succeed, all locking options must be disabled.
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_chip_erase(void);

/**
 ****************************************************************************************
 * @brief Erase chip even if some blocks are protected
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_chip_erase_forced(void);

/**
 ****************************************************************************************
 * @brief Check E_fail bit
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_erase_fail(void);

/**
 ****************************************************************************************
 * @brief Program page (up to @ref SPI_FLASH_PAGE_SIZE bytes) starting at given address
 * @details This operation performs SPI transfers on a byte level using spi_access().
 * It is recommended to use this function when code size is critical to the application.
 * @param[in] wr_data_ptr   Pointer to the data to be written
 * @param[in] address       Starting address of data to be written
 * @param[in] size          Size of the data to be written (should not be larger than
 *                          SPI Flash page size)
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_page_program(uint8_t *wr_data_ptr, uint32_t address, uint16_t size);

/**
 ****************************************************************************************
 * @brief Program page (up to @ref SPI_FLASH_PAGE_SIZE bytes) starting at given address
 * @details This operation performs SPI transfers on a buffer level using spi_send().
 * It is recommended to use this function when performance is critical to the application.
 * @param[in] wr_data_ptr   Pointer to the data to be written
 * @param[in] address       Starting address of data to be written
 * @param[in] size          Size of the data to be written (should not be larger than
 *                          SPI Flash page size)
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_page_program_buffer(uint8_t *wr_data_ptr, uint32_t address, uint16_t size);

#if defined (CFG_SPI_DMA_SUPPORT)
/**
 ****************************************************************************************
 * @brief Program page (up to @ref SPI_FLASH_PAGE_SIZE bytes) starting at given address
 * @details This operation performs SPI transfers using the DMA engine.
 * It is recommended to use this function when performance is critical to the application.
 * @param[in] wr_data_ptr   Pointer to the data to be written
 * @param[in] address       Starting address of data to be written
 * @param[in] size          Size of the data to be written (should not be larger than
 *                          SPI Flash page size)
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_page_program_dma(uint8_t *wr_data_ptr, uint32_t address, uint16_t size);
#endif

/**
 ****************************************************************************************
 * @brief Write data to SPI Flash across page boundaries and at any starting address
 * @details This operation performs SPI transfers on a byte level using spi_access().
 * It is recommended to use this function when code size is critical to the application.
 * @param[in] wr_data_ptr   Pointer to the data to be written
 * @param[in] address       Starting address of page to be written (must be a multiple
 *                          of SPI Flash page size)
 * @param[in] size          Size of the data to be written (can be larger than SPI Flash
 *                          page size)
 * @param[out] actual_size  Actual size of written data
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_write_data(uint8_t *wr_data_ptr, uint32_t address,
                            uint32_t size, uint32_t *actual_size);

/**
 ****************************************************************************************
 * @brief Write data to SPI Flash across page boundaries and at any starting address
 * @details This operation performs SPI transfers on a buffer level using spi_send().
 * It is recommended to use this function when performance is critical to the application.
 * @param[in] wr_data_ptr   Pointer to the data to be written
 * @param[in] address       Starting address of page to be written (must be a multiple
 *                          of SPI Flash page size)
 * @param[in] size          Size of the data to be written (can be larger than SPI Flash
 *                          page size)
 * @param[out] actual_size  Actual size of written data
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_write_data_buffer(uint8_t *wr_data_ptr, uint32_t address,
                                   uint32_t size, uint32_t *actual_size);

#if defined (CFG_SPI_DMA_SUPPORT)
/**
 ****************************************************************************************
 * @brief Write data to SPI Flash across page boundaries and at any starting address
 * @details This operation performs SPI transfers using the DMA engine.
 * It is recommended to use this function when performance is critical to the application.
 * @param[in] wr_data_ptr   Pointer to the data to be written
 * @param[in] address       Starting address of page to be written (must be a multiple
 *                          of SPI Flash page size)
 * @param[in] size          Size of the data to be written (can be larger than SPI Flash
 *                          page size)
 * @param[out] actual_size  Actual size of written data
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_write_data_dma(uint8_t *wr_data_ptr, uint32_t address,
                                uint32_t size, uint32_t *actual_size);
#endif

/**
 ****************************************************************************************
 * @brief Fill memory page (up to @ref SPI_FLASH_PAGE_SIZE bytes) with a given 1-byte value
 *        starting at given address
 * @param[in] data          Value used to fill memory
 * @param[in] address       Starting address
 * @param[in] size          Size of the area to be filled (should not be larger than SPI
 *                          Flash page size)
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_page_fill(uint8_t data, uint32_t address, uint16_t size);

/**
 ****************************************************************************************
 * @brief Fill memory with a 1-byte value, across page boundaries and at any starting
 *        address
 * @param[in] value         The value with which memory will be filled
 * @param[in] address       Starting address of page to be written (must be a multiple of
 *                          SPI Flash page size)
 * @param[in] size          Size of the area to be filled (can be larger than SPI Flash
 *                          page size)
 * @param[out] actual_size  Actual size of written data
 * @return Error code
 ****************************************************************************************
*/
int8_t spi_flash_fill(uint8_t value, uint32_t address,
                      uint32_t size, uint32_t *actual_size);

/**
 ****************************************************************************************
 * @brief Check P_fail bit
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_program_fail(void);

/**
 ****************************************************************************************
 * @brief Read data from a given starting address (up to the end of the flash)
 * @details This operation performs SPI transfers on a byte level using spi_access().
 * It is recommended to use this function when code size is critical to the application.
 * @param[in] rd_data_ptr   Points to the position the read data will be stored
 * @param[in] address       Starting address of data to be read
 * @param[in] size          Size of the data to be read
 * @param[out] actual_size  Actual size of read data
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_read_data(uint8_t *rd_data_ptr, uint32_t address,
                           uint32_t size, uint32_t *actual_size);

/**
 ****************************************************************************************
 * @brief Read data from a given starting address (up to the end of the flash)
 * @details This operation performs SPI transfers on a buffer level using spi_receive().
 * It is recommended to use this function when performance is critical to the application.
 * @param[in] rd_data_ptr   Points to the position the read data will be stored
 * @param[in] address       Starting address of data to be read
 * @param[in] size          Size of the data to be read
 * @param[out] actual_size  Actual size of read data
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_read_data_buffer(uint8_t *rd_data_ptr, uint32_t address,
                                  uint32_t size, uint32_t *actual_size);

#if defined (CFG_SPI_DMA_SUPPORT)
/**
 ****************************************************************************************
 * @brief Read data from a given starting address (up to the end of the flash)
 * @details This operation performs SPI transfers using the DMA engine.
 * It is recommended to use this function when performance is critical to the application.
 * @param[in] rd_data_ptr   Points to the position the read data will be stored
 * @param[in] address       Starting address of data to be read
 * @param[in] size          Size of the data to be read
 * @param[out] actual_size  Actual size of read data
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_read_data_dma(uint8_t *rd_data_ptr, uint32_t address,
                               uint32_t size, uint32_t *actual_size);
#endif

/**
 ****************************************************************************************
 * @brief Check if a page is erased
 * @param[in] page_address  Starting address of the page
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_is_page_empty(uint32_t page_address);

/**
 ****************************************************************************************
 * @brief Check if a sector is erased
 * @param[in] sector_address    Starting address of the sector
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_is_sector_empty(uint32_t sector_address);

/**
 ****************************************************************************************
 * @brief Check if the whole chip is erased
 * @return Error code
 ****************************************************************************************
 */
int8_t spi_flash_is_empty(void);

/**
 ****************************************************************************************
 * @brief Enable, using a GPIO, the power of the externally connected SPI flash memory.
 * @return True if GPIO was already enabled (high), otherwise false.
 ****************************************************************************************
 */
bool spi_flash_enable_power_pin(void);

/**
 ****************************************************************************************
 * @brief Disable, using a GPIO, the power of the externally connected SPI flash memory.
 ****************************************************************************************
 */
void spi_flash_disable_power_pin(void);

/**
 ****************************************************************************************
 * @brief Get the status of the GPIO which is used to enable/disable the power of the
 *        externally connected SPI flash memory.
 * @return True if GPIO is high, else false.
 ****************************************************************************************
 */
bool spi_flash_get_power_pin_status(void);

#if defined (__DA14586__)
/**
 ****************************************************************************************
 * @brief Power down on-chip Flash. Supported only in DA14586 devices.
 ****************************************************************************************
*/
void power_down_da14586_flash(void);
#endif

#endif //_SPI_FLASH_H_

///@}
///@}
