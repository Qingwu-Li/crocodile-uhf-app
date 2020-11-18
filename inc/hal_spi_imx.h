
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


#ifdef __cplusplus
}
#endif

#endif