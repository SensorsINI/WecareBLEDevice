/**
 ****************************************************************************************
 *
 * @file ring_buffer.h
 *
 * @brief Ring buffer API.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#ifndef _RING_BUFFER_H
#define _RING_BUFFER_H


#include <stdint.h>
#include <stdbool.h>


bool buffer_is_empty(void);

bool buffer_is_full(void);

void buffer_put_byte(uint8_t byte);

int buffer_get_byte(uint8_t *byte);

#endif // _RING_BUFFER_H
