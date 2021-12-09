/**
 ****************************************************************************************
 *
 * @file compiler.h
 *
 * @brief Overrides the DA14585 SDK compiler.h header for MSVC compilation.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef COMPILER_H_
#define COMPILER_H_

#ifdef __GNUC__
    #define __STATIC_FORCEINLINE  __attribute__((always_inline)) static inline
#endif

#define __ARRAY_EMPTY   1

#endif
