/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup DISS
 * @brief Device Information Service Server Application API
 * @{
 *
 * @file app_diss.h
 *
 * @brief Device Information Service Application entry point.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef _APP_DISS_H_
#define _APP_DISS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_DIS_SERVER)

/*
 * GLOBAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Device Information Service Application
 ****************************************************************************************
 */
void app_dis_init(void);

/**
 ****************************************************************************************
 * @brief Add a Device Information Service instance in the DB
 ****************************************************************************************
 */
void app_diss_create_db(void);

#endif // (BLE_DIS_SERVER)

#endif // _APP_DISS_H_

///@}
///@}
///@}
