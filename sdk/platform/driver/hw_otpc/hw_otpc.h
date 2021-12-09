/**
 ****************************************************************************************
 *
 * @file hw_otpc.h
 *
 * @brief OTP Controller driver generic header file.
 *
 * @note This is the header file to be included by any module that is to call
 * (DA14585/DA14586 OR DA14531) OTP Controller functions.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _HW_OTPC_H_
#define _HW_OTPC_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if defined (__DA14531__)
    #include "hw_otpc_531.h"
#else
    #include "hw_otpc_58x.h"
#endif

#endif /* _HW_OTPC_H_ */
