/**
 ****************************************************************************************
 *
 * @file compiler.h
 *
 * @brief Definitions of compiler specific directives.
 *
 * Copyright (C) 2018-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * Arm Compiler 4/5
 */
#if   defined ( __CC_ARM )
    #include "ARM/compiler.h"

/*
 * GNU Compiler
 */
#elif defined ( __GNUC__ )
    #include "GCC/compiler.h"

/*
 * IAR Compiler
 */
#elif defined ( __ICCARM__ )
    #include "IAR/compiler.h"

#else
    #warning "Unsupported compiler."

#endif
