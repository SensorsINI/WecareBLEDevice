/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup KERNEL  Real Time Kernel
 * @brief The Real Time Kernel module
 * @{
 *
 * The Kernel is responsible for providing essential OS features like time management,
 * inter-task communication, task management and message handling and administration.
 *
 * @file ke_config.h
 *
 * @brief This file contains all the constant that can be changed in order to
 * tailor the kernel.
 *
 * Copyright (C) RivieraWaves 2009-2014
 *
 *
 ****************************************************************************************
 */

#ifndef _KE_CONFIG_H_
#define _KE_CONFIG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"       // stack configuration

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// @name Kernel Configuration
///@{
/** Kernel Settings and Configuration */
#define KE_MEM_RW       1
#define KE_MEM_LINUX    0
#define KE_MEM_LIBC     0

#define KE_FULL         1
#define KE_SEND_ONLY    0
///@}

#endif // _KE_CONFIG_H_

///@}
///@}
