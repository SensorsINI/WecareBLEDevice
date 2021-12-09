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
#include "co_bt.h"
#include "disc.h"
#include "gapc_task.h"
#include "gapm_task.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#define MAX_SCAN_DEVICES 9   
#define RSSI_SAMPLES	 5   
#define MAX_CONN_NUMBER  8

/// Maximal length for Characteristic values - 18
#define DIS_VAL_MAX_LEN                         (0x12)

///System ID string length
#define DIS_SYS_ID_LEN                          (0x08)
///IEEE Certif length (min 6 bytes)
#define DIS_IEEE_CERTIF_MIN_LEN                 (0x06)
///PnP ID length
#define DIS_PNP_ID_LEN                          (0x07)

typedef struct 
{
    unsigned char free;
    struct bd_addr adv_addr;
    unsigned short conidx;
    unsigned short conhdl;
    unsigned char idx;
    char  rssi;
    unsigned char  data_len;
    unsigned char  data[ADV_DATA_LEN + 1];
} ble_dev;

typedef struct 
{
    uint16_t    val_hdl;
    char        val[DIS_VAL_MAX_LEN + 1];
    uint16_t    len;
} dis_char;


//DIS information
typedef struct 
{
    dis_char chars[DISC_CHAR_MAX];
} dis_env;

//Proximity Reporter connected device
typedef struct 
{
    ble_dev device;
    unsigned char bonded;
    unsigned short ediv;
    struct rand_nb rand_nb[RAND_NB_LEN];
    struct gapc_ltk ltk;
    struct gapc_irk irk;
    struct gap_sec_key csrk;
    unsigned char llv;
    char txp;
    char rssi[RSSI_SAMPLES];
    char rssi_indx;
    char avg_rssi;
    unsigned char alert;
    dis_env dis;
    unsigned char isactive;
} proxr_dev;

/// application environment structure
struct app_env_tag
{
    unsigned char state;
    unsigned char num_of_devices;
    unsigned int cur_dev;
    ble_dev devices[MAX_SCAN_DEVICES];
    proxr_dev proxr_device[MAX_CONN_NUMBER];
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
 * @brief DISC add profile.
 ****************************************************************************************
 */
void app_disc_create(void);

/**
 ****************************************************************************************
 * @brief PROXM add profile.
 ****************************************************************************************
 */
void app_proxm_create(void);

/**
 ****************************************************************************************
 * @brief Configure device mode.
 ****************************************************************************************
 */
void app_set_mode(void);

/**
 ****************************************************************************************
 * @brief Send Reset request to GAP task.
 ****************************************************************************************
 */
void app_rst_gap(void);

/**
 ****************************************************************************************
 * @brief Send Inquiry (devices discovery) request to GAP task.
 ****************************************************************************************
 */
void app_inq(void);

/**
 ****************************************************************************************
 * @brief Send Connect request to GAP task.
 *
 * @param[in] indx  Peer device's index in discovered devices list.
 ****************************************************************************************
 */
void app_connect(unsigned char indx);

/**
 ****************************************************************************************
 * @brief Send Read rssi request to GAP task.
 ****************************************************************************************
 */
void app_read_rssi(void);

/**
 ****************************************************************************************
 * @brief Send disconnect request to GAP task.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_disconnect(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send pairing request.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_security_enable(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send connection confirmation.
 *
 *  @param[in] auth  Authentication requirements.
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_connect_confirm(uint8_t auth,uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief generate key.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_gen_csrk(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Confirm bonding.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_gap_bond_cfm(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Start Encryption with pre-agreed key.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_start_encryption(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send enable request to proximity monitor profile task.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_proxm_enable(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send read request for Link Loss Alert Level characteristic to proximity monitor profile task.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_proxm_read_llv(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send read request for Tx Power Level characteristic to proximity monitor profile task.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_proxm_read_txp(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send write request to proximity monitor profile task.
 *
 *  @param[in] chr Characteristic to be written.
 *  @param[in] val Characteristic's Value.
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_proxm_write(unsigned int chr, unsigned char val, uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send enable request to DIS client profile task.
 *
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_disc_enable(uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief Send read request to DIS Client profile task.
 *
 *  @param[in] char_code Characteristic to be retrieved.
 *  @param[in] con_idx	Index in connected devices list.
 ****************************************************************************************
 */
void app_disc_rd_char(uint8_t char_code,uint8_t con_idx);

/**
 ****************************************************************************************
 * @brief bd addresses compare.
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
 * @brief Check if device is in application's discovered devices list.
 *
 *  @param[in] padv_addr  Pointer to devices bd_addr.
 *
 * @return Index in list. if return value equals MAX_SCAN_DEVICES device is not listed.
 ****************************************************************************************
 */
unsigned char app_device_recorded(struct bd_addr *padv_addr);

/**
 ****************************************************************************************
 * @brief Cancel the ongoing air operation.
 ****************************************************************************************
 */
void app_cancel(void); 

/**
 ****************************************************************************************
 * @brief Read COM port number from console
 *
 * @return the COM port number.
 ****************************************************************************************
 */
unsigned int ConsoleReadPort(void);

/**
 ****************************************************************************************
 * @brief Find first available slot in device list.
 *
 * @return device list index
 ****************************************************************************************
 */
unsigned int rtrn_fst_avail(void);

/**
 ****************************************************************************************
 * @brief Find previously available connection device and return it.
 *
 * @return valid connection index
 ****************************************************************************************
 **/
unsigned int rtrn_prev_avail_conn(void);

/**
 ****************************************************************************************
 * @brief Find the number of active connections
 *
 * @return the number of active connections
 ****************************************************************************************
 **/
unsigned int rtrn_con_avail(void);

/**
 ****************************************************************************************
 * @brief Cancel last GAPM command
 ****************************************************************************************
 */
void app_cancel_gap(void);

/**
 ****************************************************************************************
 * @brief Resolve connection index (cidx).
 * 
 *  @param[in] cidx Connection index
 *
 * @return the index in connected devices list.
 ****************************************************************************************
 */
unsigned short res_conidx(unsigned short cidx);

/**
 **************************************************************************************** 
 * @brief Returns the index that corresponds to the given connection handle
 *
 *  @param[in] conhdl Connection handler
 *
 * @return the index in connected devices list.
 ****************************************************************************************
 */
unsigned short res_conhdl(uint16_t conhdl);

#endif // APP_H_
