#ifndef _UTIL_H_
#define _UTIL_H_

#include <stdint.h>

// SPI master API
#include "spi_531.h"
#include "spi.h"

//Uart
#include "uart_utils.h"

#include "math.h"


/**
 ****************************************************************************************
 * @brief Function declartion
 ****************************************************************************************
*/
void ftoa(float n, char* res, int afterpoint);

#endif // _UTIL_H_
