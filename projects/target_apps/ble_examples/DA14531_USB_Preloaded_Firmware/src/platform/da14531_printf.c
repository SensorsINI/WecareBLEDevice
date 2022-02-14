#include "da14531_printf.h"

static void padding( const s32 l_flag,const struct params_s *par);
static void outs(const charptr lp, struct params_s *par);
static s32 getnum( charptr* linep);

/*---------------------------------------------------*/
/* The purpose of this routine is to output data the */
/* same as the standard printf function without the  */
/* overhead most run-time libraries involve. Usually */
/* the printf brings in many kilobytes of code and   */
/* that is unacceptable in most embedded systems.    */
/*---------------------------------------------------*/

// Send byte in the original char format
static void outbyte(uart_t *uart, uint8_t ch)
{
    uart_send(uart, (uint8_t *)&ch, 1, UART_OP_BLOCKING);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine puts pad characters into the output  */
/* buffer.                                           */
/*                                                   */
static void padding( const s32 l_flag, const struct params_s *par)
{
    s32 i;

    if ((par->do_padding != 0) && (l_flag != 0) && (par->len < par->num1)) {
        i=(par->len);
        for (; i<(par->num1); i++) {
            outbyte(UART1, par->pad_character);
        }
    }
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a string to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */
static void outs(const charptr lp, struct params_s *par)
{
    charptr LocalPtr;
    LocalPtr = lp;
    /* pad on left if needed                         */
    if(LocalPtr != NULL) {
        par->len = (s32)strlen( LocalPtr);
    }
    padding( !(par->left_flag), par);

    /* Move string to the buffer                     */
    while (((*LocalPtr) != (char8)0) && ((par->num2) != 0)) {
        (par->num2)--;
        outbyte(UART1, *LocalPtr);
        LocalPtr += 1;
    }

    /* Pad on right if needed                        */
    /* CR 439175 - elided next stmt. Seemed bogus.   */
    /* par->len = strlen( lp)                      */
    padding( par->left_flag, par);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine gets a number from the format        */
/* string.                                           */
/*                                                   */
static s32 getnum( charptr* linep)
{
    s32 n;
    s32 ResultIsDigit = 0;
    charptr cptr;
    n = 0;
    cptr = *linep;
    if(cptr != NULL){
        ResultIsDigit = isdigit(((s32)*cptr));
    }
    while (ResultIsDigit != 0) {
        if(cptr != NULL){
            n = ((n*10) + (((s32)*cptr) - (s32)'0'));
            cptr += 1;
            if(cptr != NULL){
                ResultIsDigit = isdigit(((s32)*cptr));
            }
        }
        ResultIsDigit = isdigit(((s32)*cptr));
    }
    *linep = ((charptr )(cptr));
    return(n);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a number to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */

static void outnum( const s32 n, const s32 base, struct params_s *par)
{
    s32 negative;
    s32 i;
    char8 outbuf[32];
    const char8 digits[] = "0123456789ABCDEF";
    u32 num;
    for(i = 0; i<32; i++) {
        outbuf[i] = '0';
    }

    /* Check if number is negative                   */
    if ((par->unsigned_flag == 0) && (base == 10) && (n < 0L)) {
        negative = 1;
        num =(-(n));
    }
    else{
        num = n;
        negative = 0;
    }

    /* Build number (backwards) in outbuf            */
    i = 0;
    do {
        outbuf[i] = digits[(num % base)];
        i++;
        num /= base;
    } while (num > 0);

    if (negative != 0) {
        outbuf[i] = '-';
        i++;
    }

    outbuf[i] = 0;
    i--;

    /* Move the converted number to the buffer and   */
    /* add in the padding where needed.              */
    par->len = (s32)strlen(outbuf);
    padding( !(par->left_flag), par);
    while (&outbuf[i] >= outbuf) {
        outbyte(UART1, outbuf[i] );
        i--;
    }
    padding( par->left_flag, par);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a string which is converted    */
/* from a float number to the output buffer as       */
/* directed by the padding and positioning flags.    */
/*                                                   */
void outfloat(double n, int afterpoint, struct params_s *par)
{
	  char tmpStr[20];
		ftoa((float)n, tmpStr, 4);
	  outs(tmpStr, par);		
}


/*---------------------------------------------------*/
/*                                                   */
/* This routine operates just like a printf/sprintf  */
/* routine. It outputs a set of data under the       */
/* control of a formatting string. Not all of the    */
/* standard C format control are supported. The ones */
/* provided are primarily those needed for embedded  */
/* systems work. Primarily the floating point        */
/* routines are omitted. Other formats could be      */
/* added easily by following the examples shown for  */
/* the supported formats.                            */
/*                                                   */

/* void esp_printf( const func_ptr f_ptr,
   const charptr ctrl1, ...) */
void da14531_printf( const char8 *ctrl1, ...)
{
    s32 Check;
    s32 dot_flag;

    params_t par;

    char8 ch;
    va_list argp;
    char8 *ctrl = (char8 *)ctrl1;
	  
    va_start( argp, ctrl1);

    while ((ctrl != NULL) && (*ctrl != (char8)0)) {

        /* move format string chars to buffer until a  */
        /* format control is found.                    */
        if (*ctrl != '%') {
            outbyte(UART1, *ctrl);
            ctrl += 1;
            continue;
        }

        /* initialize all the flags for this format.   */
        dot_flag = 0;
        par.unsigned_flag = 0;
        par.left_flag = 0;
        par.do_padding = 0;
        par.pad_character = ' ';
        par.num2=32767;
        par.num1=0;
        par.len=0;

try_next:
        if(ctrl != NULL) {
            ctrl += 1;
        }
        if(ctrl != NULL) {
            ch = *ctrl;
        }
        else {
            ch = *ctrl;
        }

        if (isdigit((s32)ch) != 0) {
            if (dot_flag != 0) {
                par.num2 = getnum(&ctrl);
            }
            else {
                if (ch == '0') {
                    par.pad_character = '0';
                }
                if(ctrl != NULL) {
                    par.num1 = getnum(&ctrl);
                }
                par.do_padding = 1;
            }
            if(ctrl != NULL) {
                ctrl -= 1;
            }
            goto try_next;
        }

        switch (tolower((s32)ch)) {
            case '%':
                outbyte(UART1, '%');
                Check = 1;
                break;

            case '-':
                par.left_flag = 1;
                Check = 0;
                break;

            case '.':
                dot_flag = 1;
                Check = 0;
                break;

            case 'l':
                Check = 0;
                break;

            case 'u':
                par.unsigned_flag = 1;
                /* fall through */
            case 'i':
            case 'd':
                outnum( va_arg(argp, s32), 10L, &par);
                Check = 1;
                break;
						case 'f':
						    outfloat(va_arg(argp, double), 4, &par);
						    Check = 1;
								break;
            case 'p':
            case 'X':
            case 'x':
                par.unsigned_flag = 1;
                outnum((s32)va_arg(argp, s32), 16L, &par);
                Check = 1;
                break;

            case 's':
                outs( va_arg( argp, char *), &par);
                Check = 1;
                break;

            case 'c':
                outbyte(UART1, va_arg( argp, s32));
                Check = 1;
                break;

            case '\\':
                switch (*ctrl) {
                    case 'a':
                        outbyte(UART1, ((char8)0x07));
                        break;
                    case 'h':
                        outbyte(UART1, ((char8)0x08));
                        break;
                    case 'r':
                        outbyte(UART1, ((char8)0x0D));
                        break;
                    case 'n':
                        outbyte(UART1, ((char8)0x0D));
                        outbyte(UART1, ((char8)0x0A));
                        break;
                    default:
                        outbyte(UART1, *ctrl);
                        break;
                }
                ctrl += 1;
                Check = 0;
                break;

            default:
                Check = 1;
                break;
        }
        if(Check == 1) {
            if(ctrl != NULL) {
                ctrl += 1;
            }
            continue;
        }
        goto try_next;
    }
    va_end( argp);
}
/*---------------------------------------------------*/
