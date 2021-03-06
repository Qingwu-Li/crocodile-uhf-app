// hal_interface.h

#ifndef __HAL_INTERFACE_H
#define __HAL_INTERFACE_H


/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Types.h"
#include <stdint.h>
#include "SPIRIT_Config.h"
#include "hal_spi_imx.h"
#include "hal_gpio_imx.h"



#ifdef __cplusplus
extern "C" {
#endif



#define IRQ_DEBUG 0
#define LINEFIFO_DEBUG 0
#define PKTBASIC_DEBUG 0
#define PKTCOMMON_DEBUG 0
#define APP_DEBUG  0
#define P2P_APP_DEBUG 0
#define PKTSTACK_DEBUG 0


#define X_NUCLEO_IDS01A4
#define NO_EEPROM
#define XTAL_FREQUENCY 50000000

typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;






#define __IO    volatile

unsigned long GetTickCount();
#define hal_debug printf

#define hal_debug_msg(cond,printf_exp)   ((void)((cond)?(printf printf_exp),1:0))

#ifdef __cplusplus
}
#endif

#endif

