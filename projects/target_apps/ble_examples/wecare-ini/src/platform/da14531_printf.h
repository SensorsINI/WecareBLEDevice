#ifndef DA14531_PRINTF_H
#define DA14531_PRINTF_H
 
#include <stdint.h>

//Uart
#include "uart_utils.h"

#include "stdarg.h"
#include "ctype.h"

#include "utils.h"

/*----------------------------------------------------*/
/* Use the following parameter passing structure to   */
/* make xil_printf re-entrant.                        */
/*----------------------------------------------------*/

struct params_s;


/*---------------------------------------------------*/
/* The purpose of this routine is to output data the */
/* same as the standard printf function without the  */
/* overhead most run-time libraries involve. Usually */
/* the printf brings in many kilobytes of code and   */
/* that is unacceptable in most embedded systems.    */
/*---------------------------------------------------*/
typedef char char8;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;
typedef uint64_t u64;
typedef int sint32;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef char8* charptr;

typedef int32_t s32;
typedef struct params_s {
    s32 len;
    s32 num1;
    s32 num2;
    char8 pad_character;
    s32 do_padding;
    s32 left_flag;
    s32 unsigned_flag;
} params_t;
 
/**
 ****************************************************************************************
 * @brief Function declartion
 ****************************************************************************************
*/
void da14531_printf( const char8 *ctrl1, ...);

 #endif	/* end of protection macro */
