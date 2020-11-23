#include "stdio.h"
#include "SPIRIT1_Util.h"
#include "SPIRIT_Gpio.h"
#include "p2p_app.h"
#include "unistd.h"
#include <stdint.h>
#include <string.h>
#include "hal_gpio_imx.h"
#include "cmdline.h" 
#include "p2p_app.h"
#include "hal_interface.h"

#include <pthread.h>
#include <iostream>
#include <sys/time.h>
#include <fcntl.h>
#include <poll.h>
#include "hal_spi_imx.h"

using namespace std;
static int verbose=0;
static int crc_error_cnt=0;


/**
* @brief RadioDriver_t structure fitting
*/
RadioDriver_t spirit_cb =
{
  .Init = Spirit1InterfaceInit, 
  .GpioIrq = Spirit1GpioIrqInit,
  .RadioInit = Spirit1RadioInit,
  .SetRadioPower = Spirit1SetPower,
  .PacketConfig = Spirit1PacketConfig,
  .SetPayloadLen = Spirit1SetPayloadlength,
  .SetDestinationAddress = Spirit1SetDestinationAddress,
  .EnableTxIrq = Spirit1EnableTxIrq,
  .EnableRxIrq = Spirit1EnableRxIrq,
  .DisableIrq = Spirit1DisableIrq,
  .SetRxTimeout = Spirit1SetRxTimeout,
  .EnableSQI = Spirit1EnableSQI,
  .SetRssiThreshold = Spirit1SetRssiTH,
  .ClearIrqStatus = Spirit1ClearIRQ,
  .StartRx = Spirit1StartRx,
  .StartTx = Spirit1StartTx,
  .GetRxPacket = Spirit1GetRxPacket
};


#if defined(USE_STack_PROTOCOL)

#elif defined(USE_BASIC_PROTOCOL)

/**
* @brief Packet Basic structure fitting
*/
PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};


/**
* @brief Address structure fitting
*/

PktBasicAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  SENDER_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

#endif

#ifdef CSMA_ENABLE
  /**
  * @brief CSMA structure fitting
  */
  CsmaInit xCsmaInit={
    PERSISTENT_MODE_EN, /* CCA may optionally be persistent, i.e., rather than 
    entering backoff when the channel is found busy, CCA continues until the 
    channel becomes idle or until the MCU stops it.
    The thinking behind using this option is to give the MCU the possibility of 
    managing the CCA by itself, for instance, with the allocation of a 
    transmission timer: this timer would start when MCU finishes sending out 
    data to be transmitted, and would end when MCU expects that its transmission
    takes place, which would occur after a period of CCA.
    The choice of making CCA persistent should come from trading off 
    transmission latency,under the direct control of the MCU, and power 
    consumption, which would be greater due to a busy wait in reception mode.
                          */
    CS_PERIOD,
    CS_TIMEOUT,
    MAX_NB,
    BU_COUNTER_SEED,
    CU_PRESCALER
  };
#endif 
  
/* Private define ------------------------------------------------------------*/
#define TIME_UP                                         0x01

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RadioDriver_t *pRadioDriver;

/*IRQ status struct declaration*/
SpiritIrqs xIrqStatus;


RadioIrq_event radio_irq_event={PTHREAD_MUTEX_INITIALIZER,PTHREAD_COND_INITIALIZER,PTHREAD_COND_INITIALIZER};





void*  P2PInterrupt_thread(void* arg) 
{
  struct pollfd fdPoll;
  std::string pollingFile = std::string("/sys/class/gpio/gpio") + std::to_string(HOST_GPIO_TO_RADIO_IRQ)+"/value";
  int fd = open (pollingFile.c_str(), O_RDONLY );
  if(fd < 0)
  {
    hal_debug("open %s failed\n",pollingFile.c_str());
    return 0;
  }

  fdPoll.fd = fd;
  fdPoll.events = POLLPRI | POLLERR;
  fdPoll.revents = 0;
  while(true)
  {
    int c;
    int ret=read(fd,&c, 1);
    if(ret==-1)
    {

    }
    
    int res = poll(&fdPoll, 1, 5000);
    if(res < 0)
    {
      hal_debug("poll error %d\n",res);
      return 0;
    }
    if(res == 0)
    {
      hal_debug("timeout\n");
    }
    if (fdPoll.revents & POLLPRI)
    {
      lseek(fdPoll.fd, 0, SEEK_SET);
      P2PInterruptHandler();
    }

  }
  return 0;
}

/**
* @brief  Initializes RF Transceiver's HAL.
* @param  None
* @retval None.
*/
void HAL_Spirit1_Init(void)
{
  pRadioDriver = &spirit_cb;
  pRadioDriver->Init( ); 
}


/**
* @brief  This function handles the point-to-point packet transmission
* @param  uint8_t *DataBuff = Pointer to data buffer to send
*         uint8_t cTxlen = Length of aTransmitBuffer
* @retval None
*/

void AppliSendBuffRaw(uint8_t *DataBuff, uint8_t cTxlen)
{
  pRadioDriver = &spirit_cb; 
  
#ifdef USE_STack_PROTOCOL
  
  PktStackAddressesInit xAddressInit=
  {
    .xFilterOnMyAddress = S_ENABLE,
    .cMyAddress = SENDER_ADDRESS,
    .xFilterOnMulticastAddress = S_DISABLE,
    .cMulticastAddress = MULTICAST_ADDRESS,
    .xFilterOnBroadcastAddress = S_ENABLE,
    .cBroadcastAddress = BROADCAST_ADDRESS
  };
  
  SpiritPktStackAddressesInit(&xAddressInit);
  
#ifdef USE_STack_LLP

  /* LLP structure fitting */
  PktStackLlpInit xStackLLPInit=
  {
    .xAutoAck = EN_AUTOACK,                
    .xPiggybacking = EN_PIGGYBACKING,              
    .xNMaxRetx = MAX_RETRANSMISSIONS
  }; 
  
#else
  
  /* LLP structure fitting */
  PktStackLlpInit xStackLLPInit=
  {
    .xAutoAck = S_DISABLE,                
    .xPiggybacking = S_DISABLE,              
    .xNMaxRetx = PKT_DISABLE_RETX
  }; 
#endif
  
  SpiritPktStackLlpInit(&xStackLLPInit);
#endif

#ifdef USE_BASIC_PROTOCOL

  xAddressInit.cMyAddress=SENDER_ADDRESS;
  
  SpiritPktBasicAddressesInit(&xAddressInit);
  
#endif
  
  /* Spirit IRQs disable */
  pRadioDriver->DisableIrq();
      /* Spirit IRQs enable */
  pRadioDriver->EnableTxIrq();

  /* payload length config */
  pRadioDriver->SetPayloadLen(cTxlen);  
  /* rx timeout config */
  pRadioDriver->SetRxTimeout(RECEIVE_TIMEOUT);
  /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus();
  /* destination address */
  pRadioDriver->SetDestinationAddress(READER_ADDRESS);
  

  /* send the TX command */
  pRadioDriver->StartTx(DataBuff, cTxlen);
}


/**
* @brief  This function handles the point-to-point packet reception
* @param  None
* @retval None
*/
void StartReceive()
{ 

#ifdef USE_STack_PROTOCOL
  
#ifdef USE_STack_LLP
    /* LLP structure fitting */
  PktStackLlpInit xStackLLPInit=
  {
    .xAutoAck = EN_AUTOACK,                
    .xPiggybacking = EN_PIGGYBACKING,              
    .xNMaxRetx = MAX_RETRANSMISSIONS
  }; 
#else
  /* LLP structure fitting */
  PktStackLlpInit xStackLLPInit=
  {
    .xAutoAck = S_DISABLE,                
    .xPiggybacking = S_DISABLE,              
    .xNMaxRetx = PKT_DISABLE_RETX
  }; 
#endif
  SpiritPktStackLlpInit(&xStackLLPInit);
  
  
  PktStackAddressesInit xAddressInit=
  {
    .xFilterOnMyAddress = S_ENABLE,
    .cMyAddress = READER_ADDRESS,
    .xFilterOnMulticastAddress = S_DISABLE,
    .cMulticastAddress = MULTICAST_ADDRESS,
    .xFilterOnBroadcastAddress = S_ENABLE,
    .cBroadcastAddress = BROADCAST_ADDRESS
  };
  
  SpiritPktStackAddressesInit(&xAddressInit);
  
  if(EN_FILT_SOURCE_ADDRESS)
  {
    
    SpiritPktStackFilterOnSourceAddress(S_ENABLE);
    SpiritPktStackSetRxSourceMask(SOURCE_ADDR_MASK);
    SpiritPktStackSetSourceReferenceAddress(SOURCE_ADDR_REF);
    
  }
  else
  {
    
    SpiritPktStackFilterOnSourceAddress(S_DISABLE);    
  }
#endif

#ifdef USE_BASIC_PROTOCOL


    
    xAddressInit.cMyAddress=READER_ADDRESS;  
  
    SpiritPktBasicAddressesInit(&xAddressInit);

#endif  
  
  pRadioDriver = &spirit_cb;  
  /* Spirit IRQs disable */
  pRadioDriver->DisableIrq();
  /* Spirit IRQs enable */
  pRadioDriver->EnableRxIrq();
  /* payload length config */
  pRadioDriver->SetPayloadLen(PAYLOAD_LEN);
  /* rx timeout config */
  pRadioDriver->SetRxTimeout(RECEIVE_TIMEOUT);
  /* destination address */
  pRadioDriver->SetDestinationAddress(SENDER_ADDRESS);
  /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus(); 
  /* RX command */

  pRadioDriver->StartRx();      
}

/**
* @brief  This function initializes the protocol for point-to-point 
* communication
* @param  None
* @retval None
*/
void P2P_Init(SRadioInit* pxSRadioInitStruct)
{
  
  pRadioDriver = &spirit_cb;


  /* Spirit Radio config */    
  pRadioDriver->RadioInit(pxSRadioInitStruct);
  
  /* Spirit Radio set power */
  pRadioDriver->SetRadioPower(POWER_INDEX, pxSRadioInitStruct->lMaxPwrLevel);  
  
  /* Spirit Packet config */  
  pRadioDriver->PacketConfig();
  
  pRadioDriver->EnableSQI();
  
  pRadioDriver->SetRssiThreshold(RSSI_THRESHOLD);
}

/**
* @brief  This function initializes the STack Packet handler of spirit1
* @param  None
* @retval None
*/
void STackProtocolInit(void)
{ 
  
   PktStackInit xStackInit=
   {
     .xPreambleLength = PREAMBLE_LENGTH,
     .xSyncLength = SYNC_LENGTH,
     .lSyncWords = SYNC_WORD,
     .xFixVarLength = LENGTH_TYPE,
     .cPktLengthWidth = 8,
     .xCrcMode = CRC_MODE,
     .xControlLength = CONTROL_LENGTH,
     .xFec = EN_FEC,
     .xDataWhitening = EN_WHITENING
   };  
   
   /* Spirit Packet config */
   SpiritPktStackInit(&xStackInit);
}  
    
#ifdef USE_BASIC_PROTOCOL               
/*xDataWhitening;  *
* @brief  This function initializes the BASIC Packet handler of spirit1
* @param  None
* @retval None
*/
void BasicProtocolInit(void)
{
  /* Spirit Packet config */
  SpiritPktBasicInit(&xBasicInit);
}
#endif


/**
* @brief  This function will turn on the radio and waits till it enters the Ready state.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerON(void)
{
  SpiritCmdStrobeReady();   
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_READY);
}


/**
* @brief  This function will Shut Down the radio.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerOFF(void)
{
  SdkEvalEnterShutdown();
}


/**
* @brief  This function will put the radio in standby state.
* @param  None. 
* @retval None
*                       
*/
void RadioStandBy(void)
{
  SpiritCmdStrobeStandby();  
}

/**
* @brief  This function will put the radio in sleep state.
* @param  None. 
* @retval None
*                       
*/
void RadioSleep(void)
{
  SpiritCmdStrobeSleep(); 
}



/**
* @brief  This function handles External interrupt request. In this application it is used
*         to manage the Spirit IRQ configured to be notified on the Spirit GPIO_3.
* @param  None
* @retval None
*/
void P2PInterruptHandler(void)
{
   
  SpiritIrqGetStatus(&xIrqStatus);
  
  /* Check the SPIRIT TX_DATA_SENT IRQ flag */
  if(
     (xIrqStatus.IRQ_TX_DATA_SENT) 
       
#ifdef CSMA_ENABLE
       ||(xIrqStatus.IRQ_MAX_BO_CCA_REACH)
#endif
    )
  {
#ifdef CSMA_ENABLE
      SpiritCsma(S_DISABLE);  
      SpiritRadioPersistenRx(S_ENABLE);
      SpiritRadioCsBlanking(S_ENABLE);
      
      if(xIrqStatus.IRQ_MAX_BO_CCA_REACH)
      {
        SpiritCmdStrobeSabort();
      }

      SpiritQiSetRssiThresholddBm(RSSI_THRESHOLD);
#endif

      pthread_mutex_lock(&radio_irq_event.thread_lock);
      pthread_cond_signal(&radio_irq_event.tx_data_sent);
      pthread_mutex_unlock(&radio_irq_event.thread_lock);
  }
  
  /* Check the SPIRIT RX_DATA_READY IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_READY)// ||xIrqStatus.IRQ_RX_FIFO_ALMOST_FULL))
  {
    
    pthread_mutex_lock(&radio_irq_event.thread_lock);
    pthread_cond_signal(&radio_irq_event.rx_data_ready);  
    pthread_mutex_unlock(&radio_irq_event.thread_lock);
  }
  
  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    SpiritCmdStrobeRx();
  }
  
  /* Check the SPIRIT RX_DATA_DISC IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_DISC)
  {      
    /* RX command - to ensure the device will be ready for the next reception */
    SpiritCmdStrobeRx();
  }
  if(xIrqStatus.IRQ_CRC_ERROR) 
  {
    crc_error_cnt++;
    SpiritCmdStrobeRx();
  }
   if(xIrqStatus.IRQ_RX_FIFO_ERROR) 
  {
    SpiritCmdStrobeRx();
  }

  
}


int radio_send_data(uint8_t *data,uint8_t len,uint8_t timeout_s)
{
  hal_debug_msg(verbose,("[%s]\n",data));
  uint8_t tx_fifo_size=0;
  time_t T;                                                                     
  struct timespec t;
  int wait_ret=0;

  do
  {
    tx_fifo_size=SpiritLinearFifoReadNumElementsTxFifo();
  } while (tx_fifo_size!=0);  

  time(&T);
  
  unsigned long tick=GetTickCount();
  pthread_mutex_lock(&radio_irq_event.thread_lock);
  t.tv_sec = T;
  t.tv_nsec=100;
  wait_ret=pthread_cond_timedwait(&radio_irq_event.tx_data_sent, &radio_irq_event.thread_lock,&t);
  t.tv_sec = T + timeout_s;
  t.tv_nsec=0;
  AppliSendBuffRaw(data, len);
  wait_ret=pthread_cond_timedwait(&radio_irq_event.tx_data_sent, &radio_irq_event.thread_lock,&t);
  pthread_mutex_unlock(&radio_irq_event.thread_lock);
  tick=GetTickCount()-tick;


  hal_debug("takes=%d ms %d bytes %d bps\n",(int)(tick),(int)len,(int)(len*8/tick));
  if(0!=wait_ret)
  {
    hal_debug("pthread_cond_timedwait failed %d\n",wait_ret);
    return 0;
  }  
  return  len;
}



uint8_t recive_data(uint8_t *pRxBuff,uint8_t *Rxlen,uint8_t timeout_s)
{
  uint8_t readlen=0;
  time_t T;                                                                     
  struct timespec t;
  int wait_ret=0;
  

  time(&T);                                                    
  t.tv_sec = T + timeout_s;
  t.tv_nsec=0;
  
  pthread_mutex_lock(&radio_irq_event.thread_lock);
  wait_ret=pthread_cond_timedwait(&radio_irq_event.rx_data_ready, &radio_irq_event.thread_lock,&t);
  pthread_mutex_unlock(&radio_irq_event.thread_lock);

  if(ETIMEDOUT==wait_ret)
  {
    return 0;
  } 
  
  Spirit1GetRxPacket(pRxBuff,&readlen);
  
  if(0==readlen)
  {
   *Rxlen=0;
  }
  else
  {
    hal_debug_msg(verbose, ("[%s]\n",pRxBuff));
    *Rxlen=readlen;
  } 
  return *Rxlen;
}


int main(int argc, char *argv[])
{
  uint8_t rxdata[MAX_BUFFER_LEN]={0};
  uint8_t rx_len=0;
  uint8_t tx_len=0;

  uint32_t test_iterates=1;
  uint32_t sent_bytes=0;
  uint32_t totall_recieved_bytes=0;

  uint32_t waittime=0;

  pthread_t tid1;
  uint8_t test_data=0x5A;
  uint8_t test_buf_len=10;

  cmdline::parser a;

  SRadioInit xRadioInit = {
  (int16_t)XTAL_OFFSET_PPM,
  (uint32_t)BASE_FREQUENCY,
  (uint32_t)CHANNEL_SPACE,
  (uint8_t)DEFAULT_CHANNEL_NUMBER,
  MODULATION_SELECT,
  (uint32_t)DEFAULT_DATARATE,
  (uint32_t)FREQ_DEVIATION,
  (uint32_t)BANDWIDTH
};

  a.add<int>("channel",   'c', "Channel number",        false, 0, cmdline::range(0, 32));
  a.add<int>("datarate",  'e', "Air dataratE [100, 500000]",         false, 38400, cmdline::range(100, 500000));
  a.add<int>("bandwidth", 'b', "Bandwidth [1100, 800100]",         false, 100000, cmdline::range(1100, 800100));
  a.add<int>("addr",      'a', "my Address",            false, 0x34, cmdline::range(1, 256));
  a.add<int>("dest",      'd', "Destination address",   false, 0x44, cmdline::range(1, 256));
  a.add<int>("iterate",   'i', "iterates of the test",    false, 1, cmdline::range(1, 65535));
  a.add<int>("waittime",  'w', "the wait ms between two packages(transmitter)",    false, 1, cmdline::range(0, 300));
  a.add<int>("powerlevel",  'p', "Output power leve[-34,11]",    false, 0, cmdline::range(-34, 11));
  a.add<int>("transmitlength",  't', "Transmit buffer length[2,60]",    false, 2, cmdline::range(2, 60));
  a.add("receiver",       'r', "as a receiver");
  a.add("verbose",       'v', "enable verbose message");
  //a.add<string>("transmitlength",   't', "Transmit buffer length[2,60]", 10,false, "");

  
  a.parse_check(argc, argv);

  //max_power_level=a.get<int>("powerlevel");
  xRadioInit.cChannelNumber=a.get<int>("channel");
  xRadioInit.lDatarate=a.get<int>("datarate");
  xRadioInit.lBandwidth=a.get<int>("bandwidth");
  xRadioInit.lMaxPwrLevel=a.get<int>("powerlevel");

  test_buf_len=a.get<int>("transmitlength");
  
  waittime=a.get<int>("waittime");
  test_iterates=a.get<int>("iterate");
  hal_debug("test_iterates %d\n",test_iterates);
  verbose=a.exist("verbose");
  //string databuffer = a.get<string>("data");


  hal_debug("iterate %d\n",a.get<int>("iterate"));
  hal_debug("channel %d\n",xRadioInit.cChannelNumber);
  hal_debug("addr %d\n",a.get<int>("addr"));
  hal_debug("dest %d\n",a.get<int>("dest"));
  hal_debug("powerlevel %d\n",xRadioInit.lMaxPwrLevel);

  RadioGpioInit(HOST_GPIO_TO_RADIO_IRQ,RADIO_MODE_EXTI_IN);
  
  SGpioInit radio_gpio_0={SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_1={SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_2={SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_3={SPIRIT_GPIO_3, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_IRQ};


  
  HAL_Spirit1_Init();
  P2P_Init(&xRadioInit);
  
  if (pthread_create(&tid1, NULL,P2PInterrupt_thread, (void*)"new thread:") != 0) 
  {
      hal_debug("pthread_create error.");
  }
  SpiritGpioInit(&radio_gpio_0);
  SpiritGpioInit(&radio_gpio_1);
  SpiritGpioInit(&radio_gpio_2);
  SpiritGpioInit(&radio_gpio_3);

  if (a.exist("receiver"))
  {
    StartReceive();
    int errorcnt=0;
    while(1)
    {
      rx_len=0;
      memset(rxdata,0,sizeof(rxdata));
      uint32_t bytes=recive_data(rxdata,&rx_len,20);
      if(bytes)
      {
        totall_recieved_bytes+=bytes;
        hal_debug("Recieved=%d Discard packages %d error %d\n",totall_recieved_bytes,crc_error_cnt,errorcnt);
        for(uint32_t i=0;i<bytes;i++)
        {
          if(rxdata[i]!=test_data)
          {
            errorcnt++;
            printf("ERROR %d [0x%02x]\n",errorcnt,rxdata[i]);
          }
        }
        //SpiritQiGetRssi();

      }
      Spirit1StartRx();
    }
    
  }
  else
  {
    //uint8_t* txdata= (uint8_t*)a.get<string>("data").data();
    uint8_t txdata[MAX_BUFFER_LEN]={0};
    tx_len=test_buf_len;
    unsigned long tick=GetTickCount();
    memset(txdata,test_data,tx_len);
    int i=0;
    for(;test_iterates>0;test_iterates--)
    {
      
      sent_bytes+=radio_send_data(txdata,tx_len,10);
      usleep(waittime*1000);
      i++;
      hal_debug("%d\n",sent_bytes);
      
    }
    tick=GetTickCount()-tick;
    if (tick>0)
    {
      hal_debug("send %d bytes take %ld ms speed= %ld kbps \n",sent_bytes,tick,(sent_bytes*8)/tick);
    }    
  }
  return 0;
}

/////////////////////////////////
