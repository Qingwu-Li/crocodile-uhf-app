/**
  ******************************************************************************
  * @file    hal_comm_imx.c
  * 
  * 
  ******************************************************************************
  */

#include "unistd.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdarg.h>
#include "hal_interface.h"
/**
* @brief   Number of milliseconds since the device booted.
* @param  None.
* @retval The number of milliseconds indicates success.
*/
unsigned long GetTickCount()
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
/*
int hal_debug(const char *__restrict __fmt, ...)
{
    va_list arg;
    int done;

    va_start (arg, __fmt);
    done = printf ( __fmt, arg);
    va_end (arg);

    return done;
}*/