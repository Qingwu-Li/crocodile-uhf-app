/**
******************************************************************************
* @file    radio_spi.c
* @author  System Lab - NOIDA
* @version V1.0.0
* @date    15-May-2014
* @brief   This file provides code for the configuration of the SPI instances.                     
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
*     without specific prior written permission.
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


/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/stat.h>
#include <sys/ioctl.h>  
#include <stdio.h>
#include <pthread.h>
#define DEV_SPI	"/dev/spidev1.0"
#include "radio_spi.h"
#include "hal_gpio_imx.h"

static int fd = -1;

#define SPI_MODE 0
#define SPI_SPEED 5000000   //10MHZ Max
#define SPI_DEBUG 0


pthread_mutex_t spi_lock=PTHREAD_MUTEX_INITIALIZER;

void SdkEvalSpiInit(void)
{
  int res = 0;
  unsigned char mode=  SPI_MODE;
  unsigned int speed = SPI_SPEED;

  if (fd >0)
  {
    return;
  }

  fd = open(DEV_SPI, O_RDWR);

	if (fd < 0) {
		hal_debug("Error:cannot open device "
		       "(Maybe not present in your board?)\n");
		return ;
	}

  res =ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (res == -1) {
		hal_debug("can't set spi mode");
	}

  res = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (res == -1) {
		hal_debug("can't set max speed hz");
	}
  
}

int transfer(int fd, char *tbuf, char *rbuf, int bytes)
{
	int ret;
  int i=0;

  if (SPI_DEBUG) hal_debug("transfer %d\n",bytes);

  SdkEvalSpiInit();
  
  if (SPI_DEBUG) hal_debug("tbuf:\n");
  for(i=0;i<bytes;i++)
  {
    if (SPI_DEBUG) hal_debug("%02x ",tbuf[i]);
  }
  if (SPI_DEBUG) hal_debug("\n");

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tbuf,
		.rx_buf = (unsigned long)rbuf,
		.len = (unsigned int)bytes,
	};
      
  
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  

	if (ret == 1)
		hal_debug("can't send spi message");


  if (SPI_DEBUG) hal_debug("rbuf:\n");

  for(i=0;i<bytes;i++)
  {
    if (SPI_DEBUG) hal_debug("%02x ",rbuf[i]);
  }
  if (SPI_DEBUG) hal_debug("\n");

	return ret;
}





/**
* @brief  Write single or multiple RF Transceivers register
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
    if (SPI_DEBUG) hal_debug("SdkEvalSpiWriteRegisters %x %d %x\n",cRegAddress,cNbBytes,*pcBuffer);

    pthread_mutex_lock(&spi_lock);
    StatusBytes Status;
    uint8_t  *p_status=(uint8_t  *)&Status;
    memset(p_status,0,sizeof(Status));
    
    char *tx_buffer = (char *)malloc(cNbBytes+2);
    memset(tx_buffer, 0, cNbBytes+2);
    tx_buffer[0] = WRITE_HEADER;
    tx_buffer[1] = cRegAddress;
    memcpy(&tx_buffer[2],pcBuffer,cNbBytes);

    char *rx_buffer = (char *)malloc(cNbBytes+2);
    memset(rx_buffer, 0, cNbBytes+2);

    transfer(fd, tx_buffer, rx_buffer, cNbBytes+2);
    
    p_status[0]=rx_buffer[1];
    p_status[1]=rx_buffer[0];
    pthread_mutex_unlock(&spi_lock);

    
    
    return Status;
  
  
}


/**
* @brief  Read single or multiple SPIRIT1 register
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{

    if (SPI_DEBUG) hal_debug("SdkEvalSpiReadRegisters %x %d\n",cRegAddress,cNbBytes);

    pthread_mutex_lock(&spi_lock);
    StatusBytes Status;
    uint8_t  *p_status=(uint8_t  *)&Status;
    memset(p_status,0,sizeof(Status));
    
    
    char * tx_buffer = (char *)malloc(cNbBytes+2);
    memset(tx_buffer, 0, cNbBytes+2);

    tx_buffer[0] = READ_HEADER;
    tx_buffer[1] = cRegAddress;

    char *rx_buffer = (char *)malloc(cNbBytes+2);
    memset(rx_buffer, 0, cNbBytes+2);

    transfer(fd, tx_buffer, rx_buffer, cNbBytes+2);
    

    p_status[0]=rx_buffer[1];
    p_status[1]=rx_buffer[0];

    memcpy(pcBuffer,&rx_buffer[2],cNbBytes);

    if (SPI_DEBUG)
    {
      hal_debug("pcBuffer :\n");
      for(int i=0;i<cNbBytes;i++)
      {
        hal_debug("%02x ",pcBuffer[i]);
      }
      hal_debug("\n");
    }

    if (SPI_DEBUG) 
    {
      hal_debug("Status.XO_ON= %x\n",Status.XO_ON);
      hal_debug("Status.MC_STATE= %x\n",Status.MC_STATE);
      hal_debug("Status.ERROR_LOCK= %x\n",Status.ERROR_LOCK);
      hal_debug("Status.RX_FIFO_EMPTY= %x\n",Status.RX_FIFO_EMPTY);
      hal_debug("Status.TX_FIFO_FULL= %x\n",Status.TX_FIFO_FULL);
      hal_debug("Status.ANT_SELECT= %x\n",Status.ANT_SELECT);
      hal_debug("Status.RESVD= %x\n",Status.RESVD);
      
    }
    pthread_mutex_unlock(&spi_lock);
    
    return Status;  
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode)
{
  if (0) hal_debug("SdkEvalSpiCommandStrobes %x \n",cCommandCode);

    pthread_mutex_lock(&spi_lock);
    StatusBytes Status;
    uint8_t  *p_status=(uint8_t  *)&Status;
    memset(p_status,0,sizeof(Status));   

    
    char tx_buffer[2]={0};
    tx_buffer[0] = COMMAND_HEADER;
    tx_buffer[1] = cCommandCode;

    char rx_buffer[2]={0};


    transfer(fd, tx_buffer, rx_buffer, 2);
    
    p_status[0]=rx_buffer[1];
    p_status[1]=rx_buffer[0];
    pthread_mutex_unlock(&spi_lock);

    return Status;
  
}

/**
* @brief  Write data into TX FIFO
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  if (SPI_DEBUG) hal_debug("SdkEvalSpiWriteFifo %d \n",cNbBytes);
  
   if (SPI_DEBUG)
    {
      hal_debug("SdkEvalSpiWriteFifo\n");
      for(int i=0;i<cNbBytes;i++)
      {
        hal_debug("%02x ",pcBuffer[i]);
      }
      hal_debug("\n");
    }
  StatusBytes ret=SdkEvalSpiWriteRegisters(LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
  return ret;
}

/**
* @brief  Read data from RX FIFO
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  return SdkEvalSpiReadRegisters(LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
}