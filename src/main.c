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

using namespace std;

void*  P2PInterrupt_dummy(void* arg) 
{
  struct pollfd fdPoll;
  std::string pollingFile = std::string("/sys/class/gpio/gpio") + std::to_string(HOST_GPIO_TO_RADIO_IRQ)+"/value";
  int fd = open (pollingFile.c_str(), O_RDONLY );
  if(fd < 0)
  {
    std::cerr << "fd poll error" << std::endl;
    return 0;
  }

  fdPoll.fd = fd;
  fdPoll.events = POLLPRI | POLLERR;
  fdPoll.revents = 0;
  while(true)
  {
    int c;
    read(fd,&c, 1);
    int res = poll(&fdPoll, 1, 5000);
    if(res < 0)
    {
      std::cerr << "poll error" << std::endl;
      return 0;
    }
    if(res == 0)
    {
      std::cerr << "timeout" << std::endl;
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



/**
* @brief RadioLowPowerMode_t structure fitting
*/
RadioLowPowerMode_t Radio_LPM_cb =
{
  .RadioShutDown = RadioPowerOFF,
  .RadioStandBy = RadioStandBy,
  .RadioSleep = RadioSleep,
  .RadioPowerON = RadioPowerON
};

/**
* @brief GPIO structure fitting
*/

int16_t           nXtalOffsetPpm;
uint32_t          lFrequencyBase;
uint32_t          nChannelSpace;
uint8_t           cChannelNumber;
ModulationSelect  xModulationSelect;
uint32_t          lDatarate;
uint32_t          lFreqDev;
uint32_t          lBandwidth;    





#if defined(USE_STack_PROTOCOL)
/**
* @brief Packet Basic structure fitting
*/
PktStackInit xStackInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_FEC,
  EN_WHITENING
};

/* LLP structure fitting */
PktStackLlpInit xStackLLPInit ={
  EN_AUTOACK,
  EN_PIGGYBACKING,
  MAX_RETRANSMISSIONS
};

/**
* @brief Address structure fitting
*/

PktStackAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  SENDER_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

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
MCULowPowerMode_t *pMCU_LPM_Comm;
RadioLowPowerMode_t  *pRadio_LPM_Comm;
/*Flags declarations*/
volatile FlagStatus xRxDoneFlag = RESET, xTxDoneFlag=RESET, cmdFlag=RESET;
volatile FlagStatus xStartRx=RESET, rx_timeout=RESET, exitTime=RESET;
volatile FlagStatus PushButtonStatusWakeup=RESET;
volatile FlagStatus PushButtonStatusData=RESET;
/*IRQ status struct declaration*/
SpiritIrqs xIrqStatus;
__IO uint32_t KEYStatusData = 0x00;
static AppliFrame_t xTxFrame;
uint8_t TxFrameBuff[MAX_BUFFER_LEN] = {0x00};
uint8_t RxFrameBuff[MAX_BUFFER_LEN] = {0x00};
uint16_t exitCounter = 0;
uint16_t txCounter = 0;
uint16_t wakeupCounter = 0;
uint16_t dataSendCounter = 0x00;

RadioIrq_event radio_irq_event={PTHREAD_MUTEX_INITIALIZER,PTHREAD_COND_INITIALIZER,PTHREAD_COND_INITIALIZER};



/** @defgroup SPIRIT1_APPLI_Private_Functions
* @{
*/

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
* @param  AppliFrame_t *xTxFrame = Pointer to AppliFrame_t structure 
*         uint8_t cTxlen = Length of aTransmitBuffer
* @retval None
*/
void AppliSendBuff(AppliFrame_t *xTxFrame, uint8_t cTxlen)
{
  uint8_t xIndex = 0;
  uint8_t trxLength = 0;
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
    .xAutoAck = S_DISABLE,                
    .xPiggybacking = S_DISABLE,              
    .xNMaxRetx = PKT_N_RETX_2
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
   
  TxFrameBuff[0] = xTxFrame->Cmd;
  TxFrameBuff[1] = xTxFrame->CmdLen;
  TxFrameBuff[2] = xTxFrame->Cmdtag;
  TxFrameBuff[3] = xTxFrame->CmdType;
  TxFrameBuff[4] = xTxFrame->DataLen;
  
  for(; xIndex < cTxlen; xIndex++)
  {
    TxFrameBuff[xIndex+5] =  xTxFrame->DataBuff[xIndex];
  }
  
  
  trxLength = (xIndex+5);
  
  /* Spirit IRQs disable */
  pRadioDriver->DisableIrq();
  /* Spirit IRQs enable */
  pRadioDriver->EnableTxIrq();
  /* payload length config */
  pRadioDriver->SetPayloadLen(trxLength);  
  /* rx timeout config */
  pRadioDriver->SetRxTimeout(RECEIVE_TIMEOUT);
  /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus();
  /* destination address */
  pRadioDriver->SetDestinationAddress(READER_ADDRESS);
  
  /* send the TX command */
  pRadioDriver->StartTx(TxFrameBuff, trxLength);
}


/**
* @brief  This function handles the point-to-point packet reception
* @param  uint8_t *RxFrameBuff = Pointer to ReceiveBuffer
*         uint8_t cRxlen = length of ReceiveBuffer
* @retval None
*/
//void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen)
void StartReceive()
{
  /*float rRSSIValue = 0;*/
  exitTime = SET;
  exitCounter = TIME_TO_EXIT_RX;

#ifdef USE_STack_PROTOCOL
  
#ifdef USE_STack_LLP
    /* LLP structure fitting */
  PktStackLlpInit xStackLLPInit=
  {
    .xAutoAck = S_ENABLE,                
    .xPiggybacking = S_ENABLE,              
    .xNMaxRetx = PKT_DISABLE_RETX
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

  /**
* @brief Radio structure fitting
*/

   
     /* Spirit IRQ config */
  //pRadioDriver->GpioIrq(&xGpioIRQ);
  
  /* Spirit Radio config */    
  pRadioDriver->RadioInit(pxSRadioInitStruct);
  
  /* Spirit Radio set power */
  pRadioDriver->SetRadioPower(POWER_INDEX, POWER_DBM);  
  
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

      
      xTxDoneFlag = SET;
  }
  
  /* Check the SPIRIT RX_DATA_READY IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_READY)// ||xIrqStatus.IRQ_RX_FIFO_ALMOST_FULL))
  {
    
    pthread_mutex_lock(&radio_irq_event.thread_lock);
    pthread_cond_broadcast(&radio_irq_event.rx_data_ready);  
    pthread_mutex_unlock(&radio_irq_event.thread_lock);
    xRxDoneFlag = SET; 

  }
  
  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    rx_timeout = SET; 
    SpiritCmdStrobeRx();
  }
  
  /* Check the SPIRIT RX_DATA_DISC IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_DISC)
  {      
    /* RX command - to ensure the device will be ready for the next reception */
    SpiritCmdStrobeRx();
  }
}


int radio_send_data(uint8_t *data,uint8_t len,uint8_t timeout_s)
{
  hal_debug("[%s]\n",data);

  xTxFrame.Cmd = ACK_OK;
  xTxFrame.CmdLen = 0x01;
  xTxFrame.Cmdtag = xTxFrame.Cmdtag;
  xTxFrame.CmdType = APPLI_CMD;
  xTxFrame.DataBuff = data;
  xTxFrame.DataLen = len;
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
  AppliSendBuff(&xTxFrame, xTxFrame.DataLen);
  wait_ret=pthread_cond_timedwait(&radio_irq_event.tx_data_sent, &radio_irq_event.thread_lock,&t);
  pthread_mutex_unlock(&radio_irq_event.thread_lock);
  tick=GetTickCount()-tick;


  hal_debug("takes=%d ms %d bps\n",(int)(tick),(int)(len*8/tick));
  if(0!=wait_ret)
  {
    hal_debug("pthread_cond_timedwait failed %d\n",wait_ret);
    return 0;
  }  
  return  xTxFrame.DataLen;
}



uint8_t recive_data(uint8_t *pRxBuff,uint8_t *Rxlen,uint8_t timeout_s)
{
  uint8_t readlen=0;
  uint8_t ptempbuffer[1024];

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
    hal_debug("pthread_cond_timedwait timeout\n");
    return 0;
  }
  
  Spirit1GetRxPacket(ptempbuffer,&readlen);
  if(0==readlen)
  {
   *Rxlen=0;
  }
  else
  {
  memcpy(pRxBuff,ptempbuffer+5,readlen-5);
  *Rxlen=readlen-5;
  }
   

 
  return *Rxlen;
}


int main(int argc, char *argv[])
{
  //uint8_t txdata[1024]={0};
  uint8_t rxdata[1024]={0};


  uint8_t rx_len=96;
  uint8_t tx_len=96;

  uint32_t test_iterates=1;
  uint32_t sent_bytes=0;
  uint32_t recieved_bytes=0;

  pthread_t tid1;


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

  a.add<int>("channel",   'c', "channel number",        false, 0, cmdline::range(0, 32));
  a.add<int>("datarate",   'b', "air datarate",         false, 38400, cmdline::range(100, 500000));
  a.add<int>("addr",      'a', "my address",            false, 0x34, cmdline::range(1, 256));
  a.add<int>("dest",      'd', "destination address",   false, 0x44, cmdline::range(1, 256));
  a.add<string>("data",   't', "string to send via uhf", false, "");
  a.add<int>("iterate",   'i', "iterates of the test",    false, 1, cmdline::range(1, 65535));
  a.add("receiver", 'r', "As a receiver");

  
  a.parse_check(argc, argv);

  hal_debug("iterate %d\n",a.get<int>("iterate"));
  hal_debug("channel %d\n",a.get<int>("channel"));
  hal_debug("addr %d\n",a.get<int>("addr"));
  hal_debug("dest %d\n",a.get<int>("dest"));

  
  string databuffer = a.get<string>("data");



  xRadioInit.cChannelNumber=a.get<int>("channel");
  xRadioInit.lDatarate=a.get<int>("datarate");
  test_iterates=a.get<int>("iterate");
  hal_debug("test_iterates %d\n",test_iterates);


  RadioGpioInit(HOST_GPIO_TO_RADIO_IRQ,RADIO_MODE_EXTI_IN);
  
  SGpioInit radio_gpio_0={SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_1={SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_2={SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_3={SPIRIT_GPIO_3, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_IRQ};


  
  HAL_Spirit1_Init();
  P2P_Init(&xRadioInit);
  if (pthread_create(&tid1, NULL,P2PInterrupt_dummy, (void*)"new thread:") != 0) 
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
    while(1)
    {
      rx_len=0;
      memset(rxdata,0,sizeof(rxdata));
      recieved_bytes+=recive_data(rxdata,&rx_len,10);
      hal_debug("[%s]\n",rxdata);
      hal_debug("recieved_bytes=%d\n",recieved_bytes);
      Spirit1StartRx();
    }
    
  }
  else
  {
    uint8_t* txdata= (uint8_t*)a.get<string>("data").data();
    tx_len=strlen((char*)txdata);
    unsigned long tick=GetTickCount();
    int i=0;
    for(;test_iterates>0;test_iterates--)
    {
      txdata[0]=0x30+i%10;
      sent_bytes+=radio_send_data(txdata,tx_len,10);
      i++;

      hal_debug("%d\n",sent_bytes);
      
    }
    tick=GetTickCount()-tick;
    if (tick>0)
    {
      hal_debug("send %d bytes take %ld ms speed= %ld kbps \n",sent_bytes,tick,(sent_bytes*8)/tick);
    }
    usleep(100);

    

    //recive_data(rxdata,&rx_len,10);
 

    if(rx_len==tx_len)
    {
      hal_debug("OK\n");
    }
    
  }
  return 0;
}

/////////////////////////////////

int mainaa(int argc, char *argv[])
{
  int i=0;
  RadioGpioInit(HOST_GPIO_TO_RADIO_IRQ,RADIO_MODE_GPIO_OUT);
  Spirit1InterfaceInit();
  
  SGpioInit radio_gpio_0={SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_1={SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_2={SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};
  SGpioInit radio_gpio_3={SPIRIT_GPIO_3, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_VDD};

  SpiritGpioInit(&radio_gpio_0);
  SpiritGpioInit(&radio_gpio_1);
  SpiritGpioInit(&radio_gpio_2);
  SpiritGpioInit(&radio_gpio_3);
  

  while(1)
  {
    i++;
    if(i%2)
    {
      SpiritGpioSetLevel(SPIRIT_GPIO_0, LOW);
      SpiritGpioSetLevel(SPIRIT_GPIO_1, LOW);
      SpiritGpioSetLevel(SPIRIT_GPIO_2, LOW);
      SpiritGpioSetLevel(SPIRIT_GPIO_3, LOW);
    }
    else
    {
      SpiritGpioSetLevel(SPIRIT_GPIO_0, HIGH);
      SpiritGpioSetLevel(SPIRIT_GPIO_1, HIGH);
      SpiritGpioSetLevel(SPIRIT_GPIO_2, HIGH);
      SpiritGpioSetLevel(SPIRIT_GPIO_3, HIGH);
    }
    sleep(1);

    RadioGpioGetLevel(HOST_GPIO_TO_RADIO_IRQ);
    sleep(1);
  }

}