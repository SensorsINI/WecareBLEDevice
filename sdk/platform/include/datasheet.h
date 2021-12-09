/**
 ****************************************************************************************
 *
 * @file datasheet.h
 *
 * @brief Register definitions header file.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _DATASHEET_H_
#define _DATASHEET_H_

#if defined (__DA14531__)
    #include "da14531.h"
    #include "core_cm0plus.h"
    #include "system_DA14531.h"
#else
    #include "da14585_586.h"
    #include "core_cm0.h"
    #include "system_DA14585_586.h"
#endif

#endif
