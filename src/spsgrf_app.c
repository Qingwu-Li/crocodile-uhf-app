/**
******************************************************************************
  * @file    spsgrf_app.c
  * @author  MCD Application Team
  * @brief   This file provides some abstraction (upper layer) to SPSGRF driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "hal_gpio_imx.h"
#include "hal_interface.h"
#include "p2p_app.h"
#include "hal_spi_imx.h"
#include "SPIRIT1_Util.h"

/**
* @addtogroup ST_SPIRIT1
* @{
*/


/**
* @addtogroup SPIRIT1_Util
* @{
*/


/**
* @defgroup SPIRIT1_Util_Private_TypesDefinitions       SPIRIT1_Util Private Types Definitions
* @{
*/

/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Defines                SPIRIT1_Util Private Defines
* @{
*/

/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Macros                 SPIRIT1_Util Private Macros
* @{
*/

#define SPIRIT_VERSION          SPIRIT_VERSION_3_0
#define RANGE_TYPE              RANGE_EXT_NONE       /*RANGE_EXT_SKYWORKS*/
/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Variables              SPIRIT1_Util Private Variables
* @{
*/

/**
* @brief A map that contains the SPIRIT version
*/
const SpiritVersionMap xSpiritVersionMap[] =
{
  /* The Control Board frame handler functions */
  {CUT_2_1v4, SPIRIT_VERSION_2_1},
  {CUT_2_1v3, SPIRIT_VERSION_2_1},
  {CUT_3_0, SPIRIT_VERSION_3_0},
};
static RangeExtType xRangeExtType = RANGE_EXT_NONE;
static uint8_t s_RfModuleBand = 0;
static uint8_t s_eeprom = 0;
static int32_t s_RfModuleOffset=0;


/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_FunctionPrototypes     SPIRIT1_Util Private Function Prototypes
* @{
*/

/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Functions              SPIRIT1_Util Private Functions
* @{
*/

/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
void Spirit1InterfaceInit(void)
{ 
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  /* Initialize the SDN pin micro side */
  RadioGpioInit(HOST_GPIO_TO_RADIO_SDN,RADIO_MODE_GPIO_OUT);

  RadioSpiInit();
  
#if defined(SPIRIT1_HAS_EEPROM)
  EepromSpiInitialization();
#endif
     
  /* Board management */   
  RadioEnterShutdown(); 
  RadioExitShutdown();   
    
  SpiritManagementIdentificationRFBoard();
    
  /* Initialize the signals to drive the range extender application board */
  SpiritManagementRangeExtInit(); 
  

  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

#if defined(SPIRIT1_HAS_EEPROM)
/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
uint8_t EepromIdentification(void)
{
  uint8_t status;
  status = EepromSetSrwd();
  status = EepromStatus();
  if((status&0xF0) == EEPROM_STATUS_SRWD) 
  { 
    /*0xF0 mask [SRWD 0 0 0]*/
    status = 1;
    EepromResetSrwd();
  }
  else
    status = 0;
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));
  
  return status;
}

#endif

#if defined(SPIRIT1_HAS_EEPROM)

/**
* @brief  Identifies the SPIRIT1 Xtal frequency and version.
* @param  None
* @retval Status
*/
void SpiritManagementIdentificationRFBoard(void)
{
  tmpBuffer[128];
  hal_debug_msg(APP_DEBUG,("++ %s SPIRIT1_HAS_EEPROM  %s\n",__FUNCTION__,__FILE__));
    do{
      /* Delay for state transition */
      for(volatile uint8_t i=0; i!=0xFF; i++);
      
      /* Reads the MC_STATUS register */
     SpiritRefreshStatus();
    }while(g_xStatus.MC_STATE!=MC_STATE_READY);

  SdkEvalSetHasEeprom(EepromIdentification());
  
  if(!SdkEvalGetHasEeprom()) /* EEPROM is not present*/
  {    
    SpiritManagementComputeSpiritVersion();
    SpiritManagementComputeXtalFrequency(); 
  }
  else  /* EEPROM found*/
  {
    /*read the memory and set the variable*/
    EepromRead(0x0000, 32, tmpBuffer);
    uint32_t xtal;
    if(tmpBuffer[0]==0 || tmpBuffer[0]==0xFF) {
      SpiritManagementComputeSpiritVersion();
      SpiritManagementComputeXtalFrequency();
      return;
    }
    switch(tmpBuffer[1]) {
    case 0:
      xtal = 24000000;
      SpiritRadioSetXtalFrequency(xtal);
      break;
    case 1:
      xtal = 25000000;
      SpiritRadioSetXtalFrequency(xtal);
      break;
    case 2:
      xtal = 26000000;
      SpiritRadioSetXtalFrequency(xtal);
      break;
    case 3:
      xtal = 48000000;
      SpiritRadioSetXtalFrequency(xtal);
      break;
    case 4:
      xtal = 50000000;
      SpiritRadioSetXtalFrequency(xtal);
      break;
    case 5:
      xtal = 52000000;
      SpiritRadioSetXtalFrequency(xtal);
      break;
    default:
      SpiritManagementComputeXtalFrequency();
      break;
    }
    
    SpiritVersion spiritVersion;
    if(tmpBuffer[2]==0 || tmpBuffer[2]==1) {
      spiritVersion = SPIRIT_VERSION_2_1;
      //SpiritGeneralSetSpiritVersion(spiritVersion);
    }
    else if(tmpBuffer[2]==2) {
      spiritVersion = SPIRIT_VERSION_3_0;
      //SpiritGeneralSetSpiritVersion(spiritVersion);
    }
    else {
      SpiritManagementComputeSpiritVersion();
    }
    if(tmpBuffer[14]==1) {
      spiritVersion = SPIRIT_VERSION_3_0_D1;                        
      //SpiritGeneralSetSpiritVersion(spiritVersion);
    }
    
    RangeExtType range;
    if(tmpBuffer[5]==0) {
      range = RANGE_EXT_NONE;
    }
    else if(tmpBuffer[5]==1) {
      range = RANGE_EXT_SKYWORKS_169;
    }
    else if(tmpBuffer[5]==2) {
      range = RANGE_EXT_SKYWORKS_868;
    }
    else {
      range = RANGE_EXT_NONE;
    }
    SpiritManagementSetRangeExtender(range);
    
    SpiritManagementSetBand(tmpBuffer[3]);
    
  }
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}
#endif

#if defined(NO_EEPROM)
/**
* @brief  Identifies the SPIRIT1 Xtal frequency and version.
* @param  None
* @retval Status
*/
void SpiritManagementIdentificationRFBoard(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s NO_EEPROM  %s\n",__FUNCTION__,__FILE__));
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE!=MC_STATE_READY);

    SpiritRadioSetXtalFrequency(XTAL_FREQUENCY);        
    //SpiritGeneralSetSpiritVersion(SPIRIT_VERSION); 
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}
#endif


/**
* @brief  Sets the SPIRIT frequency band
* @param  uint8_t value: RF FREQUENCY
* @retval None
*/
void SpiritManagementSetBand(uint8_t value)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  s_RfModuleBand = value;
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}


/**
* @brief  returns the SPIRIT frequency band
* @param  None
* @retval uint8_t value: RF FREQUENCY
*/
uint8_t SpiritManagementGetBand(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  return s_RfModuleBand;
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @defgroup RANGE_EXT_MANAGEMENT_FUNCTIONS              SDK SPIRIT Management Range Extender Functions
* @{
*/
void SpiritManagementRangeExtInit(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  RangeExtType range_type = SpiritManagementGetRangeExtender();
  
  if(range_type==RANGE_EXT_SKYWORKS_169) {
    /* TCXO optimization power consumption */
    SpiritGeneralSetExtRef(MODE_EXT_XIN);
    uint8_t tmp = 0x01; 
    RadioSpiWriteRegisters(0xB6,1,&tmp);

    SGpioInit radio_gpio_0={SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_RX_MODE};
    SGpioInit radio_gpio_1={SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_STATE};
    SGpioInit radio_gpio_2={SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_RX_STATE};

    SpiritGpioInit(&radio_gpio_0);
    SpiritGpioInit(&radio_gpio_1);
    SpiritGpioInit(&radio_gpio_2);
    
    /* CSD control */
    //SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_RX_MODE});
    
    /* CTX/BYP control */
    //SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_STATE});
    
    /* Vcont control */
    //SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_RX_STATE});
  }
  else if(range_type==RANGE_EXT_SKYWORKS_868) {   

    SGpioInit radio_gpio_0={SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_RX_MODE};
    SGpioInit radio_gpio_1={SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_RX_STATE};
    SGpioInit radio_gpio_2={SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_STATE};

    SpiritGpioInit(&radio_gpio_0);
    SpiritGpioInit(&radio_gpio_1);
    SpiritGpioInit(&radio_gpio_2);
    /* CSD control */
    //SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_RX_MODE});
    
    /* CTX/BYP control */
    //SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_RX_STATE});
    
    /* Vcont control */
    //SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_STATE});
  }
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @brief  returns the spirit1 range extender type
* @param  None
* @retval RangeExtType
*/
RangeExtType SpiritManagementGetRangeExtender(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  return xRangeExtType;
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @brief  Sets the spirit1 range extender type
* @param  RangeExtType
* @retval None
*/
void SpiritManagementSetRangeExtender(RangeExtType xRangeType)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  xRangeExtType = xRangeType;
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @brief  this function returns the value to indicate that EEPROM is present or not
* @param  None
* @retval uint8_t: 0 or 1
*/
uint8_t SdkEvalGetHasEeprom(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  return s_eeprom;
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @brief  this function setc the value to indicate that EEPROM is present or not
* @param  None
* @retval uint8_t: 0 or 1
*/
void SdkEvalSetHasEeprom(uint8_t eeprom)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  s_eeprom = eeprom;
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @brief  this function intializes the spirit1 gpio irq for TX and Rx
* @param  None
* @retval None
*/
void Spirit1GpioIrqInit(SGpioInit *pGpioIRQ)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  /* Spirit IRQ config */
  SpiritGpioInit(pGpioIRQ);
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @brief  this function used to receive RX packet
* @param  None
* @retval None
*/
void Spirit1RadioInit(SRadioInit *pRadioInit)
{    
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  /* Spirit Radio config */
  SpiritRadioInit(pRadioInit);
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));


}

/**
* @brief  this function sets the radio power
* @param  uint8_t cIndex, float fPowerdBm
* @retval None
*/
void Spirit1SetPower(uint8_t cIndex, float fPowerdBm)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  /* Spirit Radio set power */
  SpiritRadioSetPALeveldBm(cIndex,fPowerdBm);
  SpiritRadioSetPALevelMaxIndex(cIndex);
  hal_debug_msg(APP_DEBUG,("-- %s %s\n",__FUNCTION__,__FILE__));

}

/**
* @brief  this function sets the packet configuration according to the protocol used
* @param  None
* @retval None
*/
void Spirit1PacketConfig(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
#if defined(USE_STack_PROTOCOL)
  
  STackProtocolInit();
   
#elif defined(USE_BASIC_PROTOCOL)
  
  BasicProtocolInit();
  
#endif
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function sets the payload length
* @param  uint8_t length
* @retval None
*/
void Spirit1SetPayloadlength(uint8_t length)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
#if defined(USE_STack_PROTOCOL)
    /* Payload length config */
  SpiritPktStackSetPayloadLength(length);
  
#elif defined(USE_BASIC_PROTOCOL)
  /* payload length config */
  SpiritPktBasicSetPayloadLength(length);
#endif
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function sets the destination address
* @param  uint8_t adress
* @retval None
*/
void Spirit1SetDestinationAddress(uint8_t address)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s 0x%02X\n",__FUNCTION__,__FILE__,address));
#if defined(USE_STack_PROTOCOL)
  /* Destination address */
  SpiritPktStackSetDestinationAddress(address);
#elif defined(USE_BASIC_PROTOCOL)
  /* destination address */
  SpiritPktBasicSetDestinationAddress(address);
#else
  Must define something
#endif
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function enables the Tx IRQ
* @param  None
* @retval None
*/
void Spirit1EnableTxIrq(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  /* Spirit IRQs enable */
  SpiritIrq(TX_DATA_SENT, S_ENABLE); 
#if defined(USE_STack_LLP)
  SpiritIrq(MAX_RE_TX_REACH, S_ENABLE);
#endif  
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function enables the Rx IRQ
* @param  None
* @retval None
*/
void Spirit1EnableRxIrq(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
    /* Spirit IRQs enable */
  SpiritIrq(RX_DATA_READY, S_ENABLE);
  SpiritIrq(RX_DATA_DISC, S_ENABLE); 
  SpiritIrq(RX_TIMEOUT, S_ENABLE);
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function disable IRQs
* @param  None
* @retval None
*/
void Spirit1DisableIrq(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  /* Spirit IRQs enable */
  SpiritIrqDeInit(NULL);
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}
/**
* @brief  this function set the receive timeout period
* @param  None
* @retval None
*/
void Spirit1SetRxTimeout(float cRxTimeOut)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  if(cRxTimeOut == 0)
  {
    /* rx timeout config */
    SET_INFINITE_RX_TIMEOUT();
    SpiritTimerSetRxTimeoutStopCondition(ANY_ABOVE_THRESHOLD);
  }
  else
  {
    /* RX timeout config */
    SpiritTimerSetRxTimeoutMs(cRxTimeOut);
    Spirit1EnableSQI();
    SpiritTimerSetRxTimeoutStopCondition(RSSI_AND_SQI_ABOVE_THRESHOLD);  
  }
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function sets the RSSI threshold
* @param  int dbmValue
* @retval None
*/
void Spirit1SetRssiTH(int dbmValue)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  SpiritQiSetRssiThresholddBm(dbmValue);
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function sets the RSSI threshold
* @param  int dbmValue
* @retval None
*/
float Spirit1GetRssiTH(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  float dbmValue=0;
  dbmValue = SpiritQiGetRssidBm();

  return dbmValue;
}

/**
* @brief  this function enables SQI check
* @param  None
* @retval None
*/
void Spirit1EnableSQI(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  /* enable SQI check */
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));

}

/**
* @brief  this function starts the RX process
* @param  None
* @retval None
*/
void Spirit1StartRx(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  if(g_xStatus.MC_STATE==MC_STATE_RX)
  {
    SpiritCmdStrobeSabort();
  }
  /* RX command */
  SpiritCmdStrobeRx();
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));
}

/**
* @brief  this function receives the data
* @param  None
* @retval None
*/
void Spirit1GetRxPacket(uint8_t *buffer, uint8_t *cRxData )
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  uint8_t noofbytes = 0;
  /* when rx data ready read the number of received bytes */

  noofbytes=SpiritLinearFifoReadNumElementsRxFifo();
  hal_debug_msg(APP_DEBUG,("noofbytes %d\n",noofbytes));
  *cRxData=noofbytes;
  if(0==noofbytes) return ;
    /* read the RX FIFO */
  RadioSpiReadFifo(noofbytes, buffer);
  SpiritCmdStrobeFlushRxFifo();
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));
}

/**
* @brief  this function starts the TX process
* @param  None
* @retval None
*/
void Spirit1StartTx(uint8_t *buffer, uint8_t size )
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  
  if(g_xStatus.MC_STATE==MC_STATE_RX)
  {
    SpiritCmdStrobeSabort();
  }
  
#ifdef CSMA_ENABLE
    
  
    /* Enable CSMA */
    SpiritRadioPersistenRx(S_DISABLE);
    SpiritRadioCsBlanking(S_DISABLE);

    SpiritCsmaInit(&xCsmaInit);
    SpiritCsma(S_ENABLE);
    SpiritQiSetRssiThresholddBm(CSMA_RSSI_THRESHOLD);
    
#endif 
  
  /* fit the TX FIFO */
  SpiritCmdStrobeFlushTxFifo();
  
  RadioSpiWriteFifo(size, buffer);
  
  /* send the TX command */
  SpiritCmdStrobeTx();
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));
}

/**
* @brief  this function clear the IRQ status
* @param  None
* @retval None
*/
void Spirit1ClearIRQ(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  SpiritIrqClearStatus();
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));
}


void SpiritManagementSetOffset(int32_t value)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  s_RfModuleOffset=value;
  hal_debug_msg(APP_DEBUG,("-- %s\n",__FUNCTION__));
}

int32_t SpiritManagementGetOffset(void)
{
  hal_debug_msg(APP_DEBUG,("++ %s %s\n",__FUNCTION__,__FILE__));
  return s_RfModuleOffset;
}

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
