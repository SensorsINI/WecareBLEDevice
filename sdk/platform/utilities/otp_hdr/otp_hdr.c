/**
 ****************************************************************************************
 *
 * @file otp_hdr.c
 *
 * @brief OTP Header source file.
 *
 * Copyright (C) 2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "otp_hdr.h"
#include "co_bt.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void otp_hdr_get_bd_address(uint8_t *bd_addr)
{
    memcpy(bd_addr, (uint8_t *) OTP_HDR_BDADDR_ADDR, BD_ADDR_LEN * sizeof(uint8_t));
}
