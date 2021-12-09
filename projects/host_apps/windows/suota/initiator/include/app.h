/**
 ****************************************************************************************
 *
 * @file app.h
 *
 * @brief Header file for Main application.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_H_
#define APP_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>  // standard integer
#include "co_bt.h"
#include "gapc_task.h"
#include "gapm_task.h"


/*
 * DEFINES
 ****************************************************************************************
 */
#define APP_VERSION     "1.3"

#define MAX_IMAGE_SIZE (0x18000) // 96Kb
#define CHECKSUM_SIZE  (1)       // in bytes

#define MAX_SCAN_DEVICES      (9)
#define MAX_SCANNING_ATTEMPTS (3)

#define CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTOR_UUID 0x2902

#define MAX_LE_PKT_SIZE         (251)
#define MAX_LE_TX_TIME          (2120)
#define MAX_GATT_MTU_SIZE       (247)
#define ATT_HEADER_SIZE         (3)
#define DEFAULT_DATA_CHUNK_SIZE (20)

#define SUOTA_VERSION_1_3       (13)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

// 128-bit UUID (stored in LE order)
typedef uint8_t uuid_128_t[16];

#define UUID_128_LE(b15, b14, b13, b12, b11, b10, b9, b8, b7, b6, b5, b4, b3, b2, b1, b0) \
    {b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15}


// SUOTA service UUIDs
#define SUOTA_PRIMARY_SERVICE_UUID 0xFEF5  // SUOTA standard UUID

extern const uuid_128_t SUOTA_MEM_DEV_UUID;
extern const uuid_128_t SUOTA_GPIO_MAP_UUID;
extern const uuid_128_t SUOTA_MEM_INFO_UUID;
extern const uuid_128_t SUOTA_PATCH_LEN_UUID;
extern const uuid_128_t SUOTA_PATCH_DATA_UUID;
extern const uuid_128_t SUOTA_SERV_STATUS_UUID;
extern const uuid_128_t SUOTA_VERSION_UUID;
extern const uuid_128_t SUOTA_MTU_UUID;
extern const uuid_128_t SUOTA_PD_CHAR_SIZE_UUID;

// SUOTA_SERV_STATUS values
enum {
    SUOTA_STATUS_SRV_STARTED    = 0x01,
    SUOTA_STATUS_CMP_OK         = 0x02,
    SUOTA_STATUS_SRV_EXIT       = 0x03,
    SUOTA_STATUS_CRC_ERR        = 0x04,
    SUOTA_STATUS_PATCH_LEN_ERR  = 0x05,
    SUOTA_STATUS_EXT_MEM_ERR    = 0x06,
    SUOTA_STATUS_INT_MEM_ERR    = 0x07,
    SUOTA_STATUS_INVAL_MEM_TYPE = 0x08,
    SUOTA_STATUS_APP_ERROR      = 0x09,
    SUOTA_STATUS_IMG_STARTED    = 0x10,     // SUOTA started for downloading image (SUOTA application)
};

// SUOTA memory types
enum
{
    SUOTA_MEM_DEV_I2C = 0x12,
    SUOTA_MEM_DEV_SPI = 0x13
};


// GPIO's
enum
{
    INVALID_GPIO = 0xFF,

    P0_0 = 0x00,
    P0_1 = 0x01,
    P0_2 = 0x02,
    P0_3 = 0x03,
    P0_4 = 0x04,
    P0_5 = 0x05,
    P0_6 = 0x06,
    P0_7 = 0x07,

    P1_0 = 0x10,
    P1_1 = 0x11,
    P1_2 = 0x12,
    P1_3 = 0x13,

    P2_0 = 0x20,
    P2_1 = 0x21,
    P2_2 = 0x22,
    P2_3 = 0x23,
    P2_4 = 0x24,
    P2_5 = 0x25,
    P2_6 = 0x26,
    P2_7 = 0x27,
    P2_8 = 0x28,
    P2_9 = 0x29,

    P3_0 = 0x30,
    P3_1 = 0x31,
    P3_2 = 0x32,
    P3_3 = 0x33,
    P3_4 = 0x34,
    P3_5 = 0x35,
    P3_6 = 0x36,
    P3_7 = 0x37
};

typedef struct {
    char *name;
    uint8_t code;
} gpio_name_t;


extern uint16_t suota_block_size;


extern int patch_length; // counts bytes
extern uint8_t patch_data[MAX_IMAGE_SIZE + CHECKSUM_SIZE];

extern uint32_t block_offset;
extern uint16_t block_length;
extern uint32_t patch_chunck_offset;
extern uint8_t patch_chunck_length;

extern int expected_write_completion_events_counter;

typedef struct {
    uint32_t patch_base_address;
    uint16_t i2c_device_address;
    uint8_t scl_gpio;
    uint8_t sda_gpio;
} suota_i2c_options_t;

typedef struct {
    uint32_t patch_base_address;
    uint8_t miso_gpio;
    uint8_t mosi_gpio;
    uint8_t cs_gpio;
    uint8_t sck_gpio;
} suota_spi_options_t;


typedef struct {
    int comport;
    struct bd_addr target_suotar_bdaddr;
    char *bin_file_name;

    uint8_t mem_type;
    union {
        suota_i2c_options_t i2c_options;
        suota_spi_options_t spi_options;
    };

    uint8_t *tk;
    uint8_t tk_len;
} suota_options_t;


extern  suota_options_t suota_options;


typedef struct
{
    unsigned char  free;
    struct bd_addr adv_addr;
    uint8_t adv_addr_type;
    unsigned short conidx;
    unsigned short conhdl;
    unsigned char idx;
    char  rssi;
    unsigned char  data_len;
    unsigned char  data[ADV_DATA_LEN + 1];

} ble_dev;


typedef struct {
    uint8_t svc_found;
    uint16_t svc_start_handle;
    uint16_t svc_end_handle;

    // SUOTA_MEM_DEV value descriptor handle
    uint16_t mem_dev_handle;
    // SUOTA_GPIO_MAP value descriptor handle
    uint16_t gpio_map_handle;
    // SUOTA_MEM_INFO value descriptor handle
    uint16_t mem_info_handle;
    // SUOTA_PATCH_LEN value descriptor handle
    uint16_t patch_len_handle;
    // SUOTA_PATCH_DATA value descriptor handle
    uint16_t patch_data_handle;
    // SUOTA_SERV_STATUS value descriptor handle
    uint16_t serv_status_handle;
    // SUOTA_PATCH_LEN value descriptor handle
    uint16_t suota_version_handle;
    // SUOTA_PATCH_DATA value descriptor handle
    uint16_t suota_mtu_handle;
    // SUOTA_SERV_STATUS value descriptor handle
    uint16_t pd_char_size_handle;

    // SUOTA_SERV_STATUS Client Characteristic Configuration Descriptor handle
    uint16_t serv_status_cli_char_cfg_desc_handle;

    uint8_t mem_dev_value[4];
    uint8_t gpio_map_value[4];
    uint8_t mem_info_value[4];
    uint8_t patch_len_value[2];
    uint8_t patch_data_value[20];
    uint8_t serv_status_value;
    uint8_t suota_version_value;
    uint8_t suota_mtu_value[2];
    uint8_t pd_char_size_value[2];

} suota_env;

// SUOTA Receiver connected device
typedef struct
{
    ble_dev device;
    unsigned char bonded;
    unsigned short ediv;
    struct rand_nb rand_nb[RAND_NB_LEN];
    struct gapc_ltk ltk;
    struct gapc_irk irk;
    struct gap_sec_key csrk;

    suota_env suota;
} peer_dev;


// application environment structure
struct app_env_tag
{
    unsigned char state;
    unsigned char num_of_devices;
    ble_dev devices[MAX_SCAN_DEVICES];
    peer_dev peer_device;

    uint8_t scan_attempts_made;
    int target_idx; // index of target in devices array
};

/// Device name structure
struct app_device_name
{
    /// Length for name
    uint8_t length;
    /// Array of bytes for name
    uint8_t name[GAP_MAX_NAME_SIZE];
};

/// Device info structure
struct app_device_info
{
    /// Device Name
    struct app_device_name dev_name;

    /// Device Appearance Icon
    uint16_t appearance;
};

#define MS_TO_BLESLOTS(x)       ((int)((x)/0.625))
#define MS_TO_DOUBLESLOTS(x)    ((int)((x)/1.25))
#define MS_TO_TIMERUNITS(x)     ((int)((x)/10))

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Device name write permissions requirements
enum device_name_write_perm
{
   NAME_WRITE_DISABLE  = (0x00 << GAPM_POS_ATT_NAME_PERM),
   NAME_WRITE_ENABLE   = (0x01 << GAPM_POS_ATT_NAME_PERM),
   NAME_WRITE_UNAUTH   = (0x02 << GAPM_POS_ATT_NAME_PERM),
   NAME_WRITE_AUTH     = (0x03 << GAPM_POS_ATT_NAME_PERM),
   /// NAME_WRITE_SECURE is not supported by SDK
};

/// Device appearance write permissions requirements
enum device_appearance_write_perm
{
   APPEARANCE_WRITE_DISABLE  = (0x00 << GAPM_POS_ATT_APPEARENCE_PERM),
   APPEARANCE_WRITE_ENABLE   = (0x01 << GAPM_POS_ATT_APPEARENCE_PERM),
   APPEARANCE_WRITE_UNAUTH   = (0x02 << GAPM_POS_ATT_APPEARENCE_PERM),
   APPEARANCE_WRITE_AUTH     = (0x03 << GAPM_POS_ATT_APPEARENCE_PERM),
   /// APPEARANCE_WRITE_SECURE is not supported by SDK
};

/*
 * EXTERNAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

// Application environment
extern struct app_env_tag app_env;

extern bool gapm_reset_cmd_completed;

/// Device info
extern struct app_device_info device_info;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

void app_set_mode(void);

void app_inq(void);

void app_connect(unsigned char indx);

void app_connect_confirm(uint8_t auth);

void app_disconnect(void);

unsigned char app_device_recorded(struct bd_addr *addr);

void app_cancel(void);

void app_security_enable(void);

void app_start_encryption(void);

void app_read_rem_fetures(void);

void app_rst_gap(void);

void app_gap_bond_cfm(void);

void app_gap_bond_tk_cfm(uint8_t tk_type);

void app_read_rssi(void);

bool bdaddr_compare(struct bd_addr *bd_address1, struct bd_addr *bd_address2);

void app_discover_svc_by_uuid_16(uint16_t conidx, uint16_t uuid);

void app_discover_all_char(uint16_t conidx, uint16_t start_handle, uint16_t end_handle);

void app_characteristic_write(uint16_t conidx, uint16_t char_val_handle, uint8_t *value, uint8_t value_length);

void app_characteristic_read(uint16_t conidx, uint16_t char_val_handle);

void app_discover_char_desc(uint16_t conidx, uint16_t start_handle, uint16_t end_handle);

void app_exit();

// SUOTA related functions
bool is_last_block(void);
void next_block(void);
bool is_last_chunk(void);
void next_chunk(void);
void app_suota_write_mem_dev(void);
void app_suota_write_gpio_map();
void app_suota_write_patch_len();
void app_suota_write_chunks(void);
void app_suota_end(void);
void app_suota_start_gatt_mtu_negotiation(void);
void app_suota_start_dle_negotiation(void);
void app_suota_set_data_params(uint16_t);
void print_time(void);

#endif // APP_H_
