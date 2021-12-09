/**
 ****************************************************************************************
 *
 * @file app.h
 *
 * @brief Header file Main application.
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
#include <stdint.h>            // standard integer
#include "gapc_task.h"
#include "gapm_task.h"
#include "co_bt.h"
#include "sdk_version.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define MAX_SCAN_DEVICES 9

/// Local address type
#define APP_ADDR_TYPE                   0
/// Advertising channel map
#define APP_ADV_CHMAP                   0x07
/// Advertising filter policy
#define APP_ADV_POL                     0
/// Advertising minimum interval
#define APP_ADV_INT_MIN                 0x44C //(687.5 ms)
/// Advertising maximum interval
#define APP_ADV_INT_MAX                 0x44C //(687.5 ms)
/// Advertising data maximal length
#define APP_ADV_DATA_MAX_SIZE           (ADV_DATA_LEN - 3)
/// Scan Response data maximal length
#define APP_SCAN_RESP_DATA_MAX_SIZE     (SCAN_RSP_DATA_LEN)

#define APP_DFLT_ADV_DATA               "\x07\x03\x03\x18\x02\x18\x04\x18"
#define APP_DFLT_ADV_DATA_LEN           (8)

#define APP_SCNRSP_DATA                 "\x09\xFF\x00\x60\x52\x57\x2D\x42\x4C\x45"
#define APP_SCNRSP_DATA_LENGTH          (10)

/*
 ****************************************************************************************
 * DISS application profile configuration
 ****************************************************************************************
 */

#define APP_DIS_FEATURES                (DIS_MANUFACTURER_NAME_CHAR_SUP | \
                                        DIS_MODEL_NB_STR_CHAR_SUP | \
                                        DIS_SYSTEM_ID_CHAR_SUP | \
                                        DIS_SW_REV_STR_CHAR_SUP | \
                                        DIS_FIRM_REV_STR_CHAR_SUP | \
                                        DIS_PNP_ID_CHAR_SUP)

/// Manufacturer Name (up to 18 chars)
#define APP_DIS_MANUFACTURER_NAME       ("Dialog Semi")
#define APP_DIS_MANUFACTURER_NAME_LEN   (11)

/// Model Number String (up to 18 chars)
#ifdef __DA14586__
#define APP_DIS_MODEL_NB_STR            ("DA14586")
#else
#define APP_DIS_MODEL_NB_STR            ("DA14585")
#endif
#define APP_DIS_MODEL_NB_STR_LEN        (7)

/// System ID - LSB -> MSB
#define APP_DIS_SYSTEM_ID               ("\x12\x34\x56\xFF\xFE\x9A\xBC\xDE")
#define APP_DIS_SYSTEM_ID_LEN           (8)

/// Serial Number
#define APP_DIS_SERIAL_NB_STR           ("1.0.0.0-LE")
#define APP_DIS_SERIAL_NB_STR_LEN       (10)

/// Hardware Revision String
#define APP_DIS_HARD_REV_STR            ("x.y.z")
#define APP_DIS_HARD_REV_STR_LEN        (5)

/// Firmware Revision
#define APP_DIS_FIRM_REV_STR            SDK_VERSION
#define APP_DIS_FIRM_REV_STR_LEN        (sizeof(APP_DIS_FIRM_REV_STR) - 1)

/// Software Revision String
#define APP_DIS_SW_REV_STR              SDK_VERSION
#define APP_DIS_SW_REV_STR_LEN          (sizeof(APP_DIS_FIRM_REV_STR) - 1)

/// IEEE
#define APP_DIS_IEEE                    ("\xFF\xEE\xDD\xCC\xBB\xAA")
#define APP_DIS_IEEE_LEN                (6)

/**
 * PNP ID Value - LSB -> MSB
 *      Vendor ID Source : 0x02 (USB Implementers Forum assigned Vendor ID value)
 *      Vendor ID : 0x045E      (Microsoft Corp)
 *      Product ID : 0x0040
 *      Product Version : 0x0300
 * e.g. #define APP_DIS_PNP_ID          ("\x02\x5E\x04\x40\x00\x00\x03")
 */
#define APP_DIS_PNP_ID                  ("\x01\xD2\x00\x80\x05\x00\x01")
#define APP_DIS_PNP_ID_LEN              (7)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef struct 
{
    unsigned char  free;
    struct bd_addr adv_addr;
    unsigned short conidx;
    unsigned short conhdl;
    unsigned char idx;
    unsigned char  rssi;
    unsigned char  data_len;
    unsigned char  data[ADV_DATA_LEN + 1];

} ble_dev;

//Proximity Reporter connected device
typedef struct 
{
    ble_dev device;
    unsigned char bonded;
    struct gapc_ltk ltk;
    struct gapc_irk irk;
    struct gap_sec_key csrk;
    unsigned char llv;
    char txp;
    unsigned char rssi;
    unsigned char alert;

} proxr_dev;

/// application environment structure
struct app_env_tag
{
    unsigned char state;
    unsigned char num_of_devices;
    ble_dev devices[MAX_SCAN_DEVICES];
    proxr_dev proxr_device;
};

struct app_env_tag_adv
{
    /// Connection handle
    uint16_t conhdl;

    ///  Advertising data
    uint8_t app_adv_data[32];
    ///  Advertising data length
    uint8_t app_adv_data_length;
    /// Scan response data
    uint8_t app_scanrsp_data[32];
    /// Scan response data length
    uint8_t app_scanrsp_data_length;
};
extern struct app_env_tag_adv app_env_adv;

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

typedef struct
{
    uint32_t blink_timeout;
    uint8_t blink_toggle;
    uint8_t lvl;
    uint8_t ll_alert_lvl;
    int8_t  txp_lvl;
    uint8_t adv_toggle; 

} app_alert_state;

/*
 * DEFINES
 ****************************************************************************************
 */
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

/// Application environment
extern struct app_env_tag app_env;
extern app_alert_state alert_state;

/// Device info
extern struct app_device_info device_info;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Exit application.
 ****************************************************************************************
*/
void app_exit(void);

/**
 ****************************************************************************************
 * @brief Set Bondable mode.
 ****************************************************************************************
 */
void app_set_mode(void);

/**
 ****************************************************************************************
 * @brief Send Reset request to GAPM task.
 ****************************************************************************************
 */
void app_rst_gap(void);

/**
 ****************************************************************************************
 * @brief Send Inquiry (devices discovery) request to GAPM task.
 ****************************************************************************************
 */
void app_inq(void);

/**
 ****************************************************************************************
 * @brief Send Start Advertising command to GAPM task.
 ****************************************************************************************
 */
void app_adv_start(void);

/**
 ****************************************************************************************
 * @brief Send Connect request to GAPM task.
 *
 * @param[in] indx  Peer device's index in discovered devices list.
 ****************************************************************************************
 */
void app_connect(unsigned char indx);

/**
 ****************************************************************************************
 * @brief Send the GAPC_DISCONNECT_IND message to a task.
 *
 * @param[in] dst     Task id of the destination task.
 * @param[in] conhdl  The conhdl parameter of the GAPC_DISCONNECT_IND message.
 * @param[in] reason  The reason parameter of the GAPC_DISCONNECT_IND message.
 ****************************************************************************************
 */
void app_send_disconnect(uint16_t dst, uint16_t conhdl, uint8_t reason);

/**
 ****************************************************************************************
 * @brief Send Read rssi request to GAPC task instance for the current connection.
 ****************************************************************************************
 */
void app_read_rssi(void);

/**
 ****************************************************************************************
 * @brief Send disconnect request to GAPC task instance for the current connection.
 ****************************************************************************************
 */
void app_disconnect(void);

/**
 ****************************************************************************************
 * @brief Send pairing request.
 ****************************************************************************************
 */
void app_security_enable(void);

/**
 ****************************************************************************************
 * @brief Send connection confirmation.
 *
 * @param[in] auth  Authentication requirements.
 * @param[in] svc_changed_ind_enable    Service changed indication enable/disable.
 ****************************************************************************************
 */
void app_connect_confirm(enum gap_auth auth, uint8_t svc_changed_ind_enable);

/**
 ****************************************************************************************
 * @brief generate csrk key.
 ****************************************************************************************
 */
void app_gen_csrk(void);

/**
 ****************************************************************************************
 * @brief generate ltk key.
 ****************************************************************************************
 */
void app_sec_gen_ltk(uint8_t key_size);

/**
 ****************************************************************************************
 * @brief Generate Temporary Key.
 *
 * @return temporary key value
 ****************************************************************************************
 */
uint32_t app_gen_tk(void);

/**
 ****************************************************************************************
 * @brief Confirm bonding.
 *
 *  param[in] param  The parameters of GAPC_BOND_REQ_IND to be confirmed.
 ****************************************************************************************
 */
void app_gap_bond_cfm(struct gapc_bond_req_ind *param);

/**
 ****************************************************************************************
 * @brief Start Encryption with pre-agreed key.
 ****************************************************************************************
 */
void app_start_encryption(void);

/**
 ****************************************************************************************
 * @brief bd address compare.
 *
 *  @param[in] bd_address1  Pointer to bd_address 1.
 *  @param[in] bd_address2  Pointer to bd_address 2.
 *
 * @return true if addresses are equal / false if not.
 ****************************************************************************************
 */
bool bdaddr_compare(struct bd_addr *bd_address1,
                    struct bd_addr *bd_address2);

/**
 ****************************************************************************************
 * @brief Check if device is in application's discovered device list.
 *
 * @param[in] padv_addr  Pointer to devices bd_addr.
 *
 * @return Index in list. if return value equals MAX_SCAN_DEVICES device is not listed.
 ****************************************************************************************
 */
unsigned char app_device_recorded(struct bd_addr *padv_addr);

/**
 ****************************************************************************************
 * @brief Add Proximity Reporter Service instance in the DB.
 ****************************************************************************************
 */
void app_proxr_create_db(void);

/**
 ****************************************************************************************
 * @brief Add a Device Information Service instance in the DB.
 ****************************************************************************************
 */
void app_diss_create_db(void);

/**
 ****************************************************************************************
 * @brief Handles commands sent from console interface.
 ****************************************************************************************
 */
void ConsoleEvent(void);

/**
 ****************************************************************************************
 * @brief Initialization of application environment
 ****************************************************************************************
 */
void app_env_init(void);

/// @} APP

#endif // APP_H_
