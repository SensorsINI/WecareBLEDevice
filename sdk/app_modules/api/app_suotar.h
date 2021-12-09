/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup SUOTAR
 * @brief Software Update Over the Air Receiver Application API
 * @{
 *
 * @file app_suotar.h
 *
 * @brief SUOTA Receiver Application header file
 *
 * Copyright (C) 2013-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_SUOTAR_H_
#define APP_SUOTAR_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "user_profiles_config.h"

#if BLE_SUOTA_RECEIVER

// #include <stdint.h>          // standard integer definition
// #include <co_bt.h>

#include "gpio.h"
#include "spi.h"
#include "spi_flash.h"
#include "i2c.h"
#include "i2c_eeprom.h"
#include "suotar.h"
// #include "suotar_task.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#if defined (CFG_SPI_FLASH_ENABLE)
    /// SPI Flash enabled
    #define SUOTAR_SPI_DISABLE                  0
    /// I2C EEPROM disabled
    #define SUOTAR_I2C_DISABLE                  1
    /// SPI Flash sector erase enabled
    #define SUOTAR_SPI_DELATE_SECTOR_ERASE      1
#elif defined (CFG_I2C_EEPROM_ENABLE)
    /// SPI Flash disabled
    #define SUOTAR_SPI_DISABLE                  1
    /// I2C EEPROM enabled
    #define SUOTAR_I2C_DISABLE                  0
#else
    /// SPI Flash disabled
    #define SUOTAR_SPI_DISABLE                  1
    /// I2C EEPROM disabled
    #define SUOTAR_I2C_DISABLE                  1
#endif

#if defined (CFG_SUOTAR_UPDATE_DISABLE)
    /// SUOTAR update disabled
    #define SUOTAR_UPDATE_DISABLE               1
#else
    /// SUOTAR update enabled
    #define SUOTAR_UPDATE_DISABLE               0
#endif

#if ((SUOTAR_SPI_DISABLE && SUOTAR_I2C_DISABLE) && !SUOTAR_UPDATE_DISABLE)
    #error "SUOTA application is enabled but both I2C EEPROM and SPI Flash are disabled"
#endif

/// @name SUOTAR status indications
///@{
/** SUOTAR start indication */
#define SUOTAR_START    1
/** SUOTAR end indication */
#define SUOTAR_END      0
///@}

/// @name SUOTAR application types
///@{
/** Software patch over the air */
#define SUOTAR_PATCH            0
/** Software image update over the air */
#define SUOTAR_IMAGE            1
///@}

/// Helper macro to check whether the software update is a patch
#define SUOTAR_IS_FOR_PATCH(mem_dev) ( (((mem_dev) & 0x10) >> 4 ) == SUOTAR_PATCH)
/// Helper macro to check whether the software update is a new image
#define SUOTAR_IS_FOR_IMAGE(mem_dev) ( (((mem_dev) & 0x10) >> 4 ) == SUOTAR_IMAGE)

/// Helper macro to check whether the memory device is valid
#define SUOTAR_IS_VALID_MEM_DEV(mem_dev) ( (mem_dev) < SUOTAR_MEM_INVAL_DEV )

/// @name SUOTAR flags
///@{
/** SUOTAR flag */
#define SUOTAR_READ_MEM_DEV_TYPE    0xFF000000
#define SUOTAR_READ_MEM_BASE_ADD    0x0000FFFF
#define SUOTAR_READ_NUM_OF_PATCHES  0xFFFF0000
#define SUOTAR_READ_MEM_PATCH_SIZE  0x0000FFFF
///@}

/// @name Example FLASH/EEPROM base address and GPIO mapping
///@{
/** Flash patch base address */
#define  SUOTAR_FLASH_PATCH_BASE_ADDR   0x03002800
/** Flash GPIO mapping */
#define  SUOTAR_FLASH_GPIO_MAP          0x05060300
/** EEPROM patch base address */
#define  SUOTAR_EEPROM_PATCH_BASE_ADDR  0x02000800
/** EEPROM GPIO mapping */
#define  SUOTAR_EEPROM_GPIO_MAP         0x00500203
///@}

/// @name SUOTAR definitions for image manipulation
///@{
/** SUOTAR definition for image manipulation */
#define PRODUCT_HEADER_POSITION     0x38000
#define PRODUCT_HEADER_SIGNATURE1   0x70
#define PRODUCT_HEADER_SIGNATURE2   0x52
#define IMAGE_HEADER_SIGNATURE1     0x70
#define IMAGE_HEADER_SIGNATURE2     0x51
#define CODE_OFFSET                 64
#define ADDITINAL_CRC_SIZE          1
#define IMAGE_HEADER_VERSION_SIZE   16
#define STATUS_VALID_IMAGE          0xAA
#define IMAGE_HEADER_OK             0
#define IMAGE_HEADER_INVALID        1
#define IMAGE_HEADER_SAME_VERSION   2
#define ANY_IMAGE_BANK              0
#define FISRT_IMAGE_BANK            1
#define SECOND_IMAGE_BANK           2
#define IMAGE_ID_0                  0
#define IMAGE_ID_1                  1
#define IMAGE_ID_2                  2
///@}

/// SUOTAR app state
typedef struct
{
    /// Memory device
    uint8_t     mem_dev;

    /// Memory base address
    uint32_t    mem_base_add;

    /// GPIO map
    uint32_t    gpio_map;

    /// New patch length
    uint32_t    new_patch_len;

    /// PD index
    uint8_t     suota_pd_idx;

    /// Image bank
    uint8_t     suota_image_bank;

    /// Block index
    uint32_t    suota_block_idx;

    /// Image index
    uint32_t    suota_img_idx;

    /// CRC
    uint8_t     crc_calc;

    /// Image length
    uint32_t    suota_image_len;

    /// Status indication function
    void (*status_ind_func) (const uint8_t);

    /// Reboot requested
    uint8_t     reboot_requested;
} app_suota_state;

#if (!SUOTAR_SPI_DISABLE)
/// GPIO configuration for SPI
typedef struct
{
    /// CS pin
    SPI_Pad_t cs;

    /// MOSI pin
    SPI_Pad_t mosi;

    /// MISO pin
    SPI_Pad_t miso;

    /// CLK pin
    SPI_Pad_t clk;
} spi_gpio_config_t;
#endif

/// I2C pad
typedef struct
{
    /// GPIO port
    GPIO_PORT port;

    /// GPIO pin
    GPIO_PIN pin;
} I2C_Pad_t;

/// GPIO configuration for I2C
typedef struct
{
    /// SCL pin
    I2C_Pad_t scl;

    /// SDA pin
    I2C_Pad_t sda;

    /// Slave address
    uint32_t  slave_addr;
} i2c_gpio_config_t;

/// Product header for SUOTA
typedef struct
{
    /// Signature
    uint8_t signature[2];

    /// Version
    uint8_t version[2];

    /// Start address of first image header
    uint32_t offset1;

    /// Start address of second image header
    uint32_t offset2;
} product_header_t;

/// Image header for SUOTA
typedef struct
{
    /// Signature
    uint8_t signature[2];

    /// Valid; set to STATUS_VALID_IMAGE at the end of the image update
    uint8_t validflag;

    /// ID to determine which image is the newest one
    uint8_t imageid;

    /// Image size
    uint32_t code_size;

    /// Image CRC
    uint32_t CRC ;

    /// Image version
    uint8_t version[16];

    /// Timestamp
    uint32_t timestamp;

    /// Encryption
    uint8_t encryption;

    uint8_t reserved[31];
}image_header_t;

/// SUOTAR Physical memory device selection and commands
enum
{
    // SUOTAR is used to send patches
    SUOTAR_MEM_INT_SYSRAM = 0x00,
    SUOTAR_MEM_INT_RETRAM = 0x01,
    SUOTAR_MEM_I2C_EEPROM = 0x02,
    SUOTAR_MEM_SPI_FLASH  = 0x03,

    // SUOTAR is used to send entire image
    SUOTAR_IMG_INT_SYSRAM = 0x10,
    SUOTAR_IMG_INT_RETRAM = 0x11,
    SUOTAR_IMG_I2C_EEPROM = 0x12,
    SUOTAR_IMG_SPI_FLASH  = 0x13,

    SUOTAR_MEM_INVAL_DEV  = 0x14, // DO NOT move. Must be before commands

    /* SUOTAR commands
    */
    SUOTAR_REBOOT         = 0xFD,
    SUOTAR_IMG_END        = 0xFE,
    // When initiator selects 0xff, it wants to exit SUOTAR service.
    // This is used in case of unexplained failures. If SUOTAR process
    // finishes correctly it will exit automatically.
    SUOTAR_MEM_SERVICE_EXIT   = 0xFF,
};

/// SUOTAR APP callbacks
struct app_suotar_cb
{
    /// Callback upon 'status_change'
    void (*on_suotar_status_change)(const uint8_t suotar_event);
};

/*
 * EXTERNAL DEFINITIONS
 ****************************************************************************************
 */

/// SUOTAR state
extern app_suota_state suota_state;
/// SUOTAR PD array
extern uint8_t suota_all_pd[SUOTA_OVERALL_PD_SIZE];


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes SUOTAR Application.
 ****************************************************************************************
 */
void app_suotar_init(void);

/**
 ****************************************************************************************
 * @brief Resets SUOTAR Application.
 ****************************************************************************************
 */
void app_suotar_reset(void);

/**
 ****************************************************************************************
 * @brief Creates SUOTAR service database.
 ****************************************************************************************
 */
void app_suotar_create_db(void);

/**
 ****************************************************************************************
 * @brief Reads memory device and writes memory info.
 * @param[in] mem_dev_type MSbyte holds the Memory device type, rest is the base address.
 * @param[in] mem_info 16MSbits hold number of patches, 16LSbits hold overall mem len.
 ****************************************************************************************
 */
void app_suotar_read_mem(uint32_t mem_dev_type, uint32_t* mem_info);

/**
 ****************************************************************************************
 * @brief Starts SUOTAR service and disables sleep.
 ****************************************************************************************
 */
void app_suotar_start(void);

/**
 ****************************************************************************************
 * @brief Stops SUOTAR service and resets application
 ****************************************************************************************
 */
void app_suotar_stop(void);

/**
 ****************************************************************************************
 * @brief Handles patch execution. Should be called at system start up and after deep sleep.
 ****************************************************************************************
 */
void app_suotar_exec_patch(void);

/**
 ****************************************************************************************
 * @brief Updates SUOTAR status characteristic.
 * @param[in] status SUOTAR application status.
 ****************************************************************************************
 */
void suotar_send_status_update_req(uint8_t status);

/**
 ****************************************************************************************
 * @brief Updates SUOTAR memory info characteristic.
 * @param[in] mem_info Patch memory info. Number of patches and overall patch length.
 ****************************************************************************************
 */
void suotar_send_mem_info_update_req(uint32_t mem_info);

/**
 ****************************************************************************************
 * @brief SUOTA image block handler. Validates image block and stores it to
 *        external memory device.
 ****************************************************************************************
 */
void app_suotar_img_hdlr(void);

#endif // BLE_SUOTA_RECEIVER

#endif // APP_H_

///@}
///@}
///@}
