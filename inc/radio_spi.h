/**
******************************************************************************
* @file    radio_spi.h
* @author  System Lab - NOIDA
* @version V1.0.0
* @date    15-May-2014
* @brief   This file contains all the functions prototypes for SPI .
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADIO_SPI_H
#define __RADIO_SPI_H
#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "hal_gpio_imx.h"
#include "SPIRIT_Config.h"
#include "radio_spi.h" 



/* SPIRIT1_Spi_config_Headers */
#define HEADER_WRITE_MASK     0x00                                /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01                                /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00                                /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80                                /*!< Command mask for header byte*/
  
#define LINEAR_FIFO_ADDRESS 0xFF                                  /*!< Linear FIFO address*/
  
/* SPIRIT1_Spi_config_Private_FunctionPrototypes */
#define SPI_ENTER_CRITICAL()         IRQ_DISABLE()
#define SPI_EXIT_CRITICAL()          IRQ_ENABLE()
  
/* SPIRIT1_Spi_config_Private_Functions */

  
/* SPIRIT1_Spi_config_Private_Macros */
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)                             /*!< macro to build the header byte*/
#define WRITE_HEADER        BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write 
                                                                                                         header byte*/
#define READ_HEADER         BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read 
                                                                                                         header byte*/
#define COMMAND_HEADER      BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command 
                                                                                                         header byte*/
  
  
  
/* Exported Variables --------------------------------------------------------*/
  
  
/* Exported functions ------------------------------------------------------- */ 
void SdkEvalSpiInit(void);
void SpiCSGpioSetLevel(GPIO_PinState xState);
StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode);
StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

  
#ifdef __cplusplus
}
#endif
#endif /*__RADIO_SPI_H */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
