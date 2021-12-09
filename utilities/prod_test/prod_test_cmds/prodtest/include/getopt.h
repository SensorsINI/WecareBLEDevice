/**
 ****************************************************************************************
 *
 * @file getopt.h
 *
 * @brief get opt header file
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _GETOPT_H_
#define _GETOPT_H_

extern char *__progname;

int getopt( int nargc, char* const *nargv, const char*ostr); 

extern int
    opterr,          /* if error message should be printed */
    optind,          /* index into parent argv vector */
    optopt,          /* character checked for validity */
    optreset;        /* reset getopt */

extern char *optarg; /* argument associated with option */

#endif /* _GETOPT_H_ */
