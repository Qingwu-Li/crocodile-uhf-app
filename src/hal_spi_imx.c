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
#include "hal_spi_imx.h"
#include "hal_gpio_imx.h"

static int fd = -1;

#define SPI_MODE 0
#define SPI_SPEED 1000000   //10MHZ Max
#define SPI_DEBUG 0

#define DEV_SPI	"/dev/spidev1.0"
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

  hal_debug_msg(SPI_DEBUG,("transfer %d\n",bytes));

  SdkEvalSpiInit();
  
  hal_debug_msg(SPI_DEBUG,("tbuf:\n"));
  for(i=0;i<bytes;i++)
  {
    hal_debug_msg(SPI_DEBUG,("%02x ",tbuf[i]));
  }
  hal_debug_msg(SPI_DEBUG,("\n"));

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tbuf,
		.rx_buf = (unsigned long)rbuf,
		.len = (unsigned int)bytes,
	};
      
  
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  

	if (ret == 1)
		hal_debug("can't send spi message");


  hal_debug_msg(SPI_DEBUG,("rbuf:\n"));

  for(i=0;i<bytes;i++)
  {
    hal_debug_msg(SPI_DEBUG,("%02x ",rbuf[i]));
  }
  hal_debug_msg(SPI_DEBUG,("\n"));

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
    hal_debug_msg(SPI_DEBUG,("SdkEvalSpiWriteRegisters %x %d %x\n",cRegAddress,cNbBytes,*pcBuffer));

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

    hal_debug_msg(SPI_DEBUG,("SdkEvalSpiReadRegisters %x %d\n",cRegAddress,cNbBytes));

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
  hal_debug_msg(SPI_DEBUG,("SdkEvalSpiWriteFifo %d \n",cNbBytes));
  
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

static uint8_t Registers[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
                    0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
                    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x26,0x27,
                    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,
                    0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,
                    0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,
                    0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,
                    0x90,0x91,0x92,0x93,0x9E,0x9F,
                    0xA1,0xA3,0xA4,0xA5,0xA6,0xA7,0xB4,
                    0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,
                    0xD0,0xD1,0xD2,0xD3,0xE4,0xE5,0xE6,0xE7,0xF0,0xF1};

void SdkEvalSpiDumpReg(void)
{
  uint8_t tmpvalue=0;
  int cnt= sizeof(Registers)/sizeof(Registers[0]);
  printf("cnt=%d\r\n",cnt);
  printf("<Registers>\r\n");
  printf("  <Device>SPIRIT1</Device>\r\n");
  printf("  <NumberFormat>Hexadecimal</NumberFormat>\r\n");


  for(int i=0;i<cnt;i++)
  {
    SdkEvalSpiReadRegisters(Registers[i], 1, &tmpvalue);
    printf("  <Register>\r\n");
    printf("    <Address>0x%02X</Address>\r\n",Registers[i]);
    printf("    <Value>0x%02X</Value>\r\n",tmpvalue);
    printf("    </Register>\r\n");
  }
  printf("</Registers>\r\n");
}