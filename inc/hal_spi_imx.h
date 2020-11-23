
/**
******************************************************************************
* @file    hal_spi_imx.h
* @author  
* @version
* @date
* @brief   
******************************************************************************
*/


#ifndef __HAL_SPI_IMX__
#define __HAL_SPI_IMX__
#ifdef __cplusplus
extern "C" {
#endif

#include "hal_gpio_imx.h"
#include "SPIRIT_Config.h"



#define HEADER_WRITE_MASK     0x00                                /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01                                /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00                                /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80                                /*!< Command mask for header byte*/
  
#define LINEAR_FIFO_ADDRESS 0xFF                                  /*!< Linear FIFO address*/

#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)                             /*!< macro to build the header byte*/
#define WRITE_HEADER        BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER         BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER      BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command */
     
typedef SpiritStatus StatusBytes;

void SdkEvalSpiInit(void);
StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode);
StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);





#define SpiritSpiInit                                                  SdkEvalSpiInit
#define SpiritSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)       SdkEvalSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)
#define SpiritSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)        SdkEvalSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)
#define SpiritSpiCommandStrobes(cCommandCode)                          SdkEvalSpiCommandStrobes(cCommandCode)
#define SpiritSpiWriteLinearFifo(cNbBytes, pcBuffer)                   SdkEvalSpiWriteFifo(cNbBytes, pcBuffer)
#define SpiritSpiReadLinearFifo(cNbBytes, pcBuffer)                    SdkEvalSpiReadFifo(cNbBytes, pcBuffer)


#define RadioSpiInit                                                   SdkEvalSpiInit
#define RadioSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)        SdkEvalSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)
#define RadioSpiWriteFifo(cNbBytes, pcBuffer)                   SdkEvalSpiWriteFifo(cNbBytes, pcBuffer)
#define RadioSpiReadFifo(cNbBytes, pcBuffer)                    SdkEvalSpiReadFifo(cNbBytes, pcBuffer)
void SdkEvalSpiDumpReg(void);

#ifdef __cplusplus
}
#endif

#endif