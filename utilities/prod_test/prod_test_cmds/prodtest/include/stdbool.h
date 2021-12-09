/**
 ****************************************************************************************
 *
 * @file stdbool.h
 *
 * @brief stdbool.h implementation for C compilers that do not support this header.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef STDBOOL_H_
#define STDBOOL_H_

#ifndef __cplusplus
    #define bool char
    #define true 1
    #define false 0
#endif

#endif // STDBOOL_H_
