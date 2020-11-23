/**
******************************************************************************
* @file    hal_gpio_imx.h
* @author  
* @version
* @date
* @brief   
******************************************************************************
*/

#ifndef __HAL_GPIO_IMX_H__
#define __HAL_GPIO_IMX_H__
#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/

#include "SPIRIT_Types.h"
#include "hal_interface.h"
#include <fcntl.h>
  
/**
 * @addtogroup BSP
 * @{
 */
typedef enum
{ 
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;

/* Exported types ------------------------------------------------------------*/
  /* MCU GPIO pin working mode for GPIO */
typedef enum                                                                                          
{
    RADIO_MODE_GPIO_IN  = 0x00,   /*!< Work as GPIO input */
    RADIO_MODE_EXTI_IN,           /*!< Work as EXTI */
    RADIO_MODE_GPIO_OUT,          /*!< Work as GPIO output */
}RadioGpioMode;  

 /* MCU GPIO pin enumeration for GPIO */
typedef enum 
{

  HOST_GPIO_TO_RADIO_IRQ   = 85,
  HOST_GPIO_TO_RADIO_SDN   = 84, /*!< GPIO_SDN selected */
} 
RadioGpioPin;   

#define __IO volatile

typedef struct
{
  __IO uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
  __IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset registerBSRR,        Address offset: 0x18      */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function register,            Address offset: 0x20-0x24 */
} GPIO_TypeDef;



/* Exported functions ------------------------------------------------------- */
int RadioGpioGetLevel(RadioGpioPin xGpio);
void RadioGpioSetLevel(RadioGpioPin xGpio, GPIO_PinState xState);
void SdkEvalEnterShutdown(void);
void SdkEvalExitShutdown(void);
int SdkEvalCheckShutdown(void);
void RadioGpioInit(RadioGpioPin xGpio, RadioGpioMode xGpioMode);
void RadioGpioInterruptCmd(RadioGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState);



#define SpiritEnterShutdown                                  SdkEvalEnterShutdown
#define SpiritExitShutdown                                   SdkEvalExitShutdown
#define SpiritCheckShutdown                                  (SpiritFlagStatus)SdkEvalCheckShutdown

#define RadioEnterShutdown                                  SdkEvalEnterShutdown
#define RadioExitShutdown                                   SdkEvalExitShutdown

#ifdef __cplusplus
}
#endif
#endif /*__HAL_GPIO_IMX_H__ */


