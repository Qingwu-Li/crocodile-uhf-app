/**
  ******************************************************************************
  * @file    SPIRIT_Irq.c
  * @author  VMA division - AMS
  * @version 3.2.2
  * @date    08-July-2015
  * @brief   Configuration and management of SPIRIT IRQs.
  * @details
  *
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Irq.h"
#include "hal_interface.h"



/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_Irq
 * @{
 */


/**
 * @defgroup Irq_Private_TypesDefinitions       IRQ Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Defines                IRQ Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Macros                 IRQ Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Variables              IRQ Private Variables
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup Irq_Private_FunctionPrototypes     IRQ Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Functions              IRQ Private Functions
 * @{
 */


/**
 * @brief  De initializate the SpiritIrqs structure setting all the bitfield to 0.
 *         Moreover, it sets the IRQ mask registers to 0x00000000, disabling all IRQs.
 * @param  pxIrqInit pointer to a variable of type @ref SpiritIrqs, in which all the
 *         bitfields will be settled to zero.
 * @retval None.
 */
void SpiritIrqDeInit(SpiritIrqs* pxIrqInit)
{
  uint8_t tempRegValue[4]={0x00,0x00,0x00,0x00};

  if(pxIrqInit!=NULL)
  {
    uint32_t tempValue = 0x00000000;
    
    /* Sets the bitfields of passed structure to one */
    *pxIrqInit = (*(SpiritIrqs*)&tempValue);
  }

  /* Writes the IRQ_MASK registers */
  g_xStatus = SpiritSpiWriteRegisters(IRQ_MASK3_BASE, 4, tempRegValue);
}


/**
 * @brief  Enables all the IRQs according to the user defined pxIrqInit structure.
 * @param  pxIrqInit pointer to a variable of type @ref SpiritIrqs, through which the
 *         user enable specific IRQs. This parameter is a pointer to a SpiritIrqs.
 *         For example suppose to enable only the two IRQ Low Battery Level and Tx Data Sent:
 * @code
 * SpiritIrqs myIrqInit = {0};
 * myIrqInit.IRQ_LOW_BATT_LVL = 1;
 * myIrqInit.IRQ_TX_DATA_SENT = 1;
 * SpiritIrqInit(&myIrqInit);
 * @endcode
 * @retval None.
 */
void SpiritIrqInit(SpiritIrqs* pxIrqInit)
{
  uint8_t tempRegValue[4];
  uint8_t* tmpPoint;

  /* Cast the bitfields structure in an array of char using */
  tmpPoint = (uint8_t*)(pxIrqInit);
  for(uint8_t i=0; i<4; i++)
  {
    tempRegValue[3-i]= tmpPoint[i];
  }

  /* Writes the IRQ_MASK registers */
  g_xStatus = SpiritSpiWriteRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

}


/**
 * @brief  Enables or disables a specific IRQ.
 * @param  xIrq IRQ to enable or disable.
 *         This parameter can be any value of @ref IrqList.
 * @param  xNewState new state for the IRQ.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritIrq(IrqList xIrq, SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue[4];
  uint32_t tempValue = 0;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_IRQ_LIST(xIrq));
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Reads the IRQ_MASK registers */
  g_xStatus = SpiritSpiReadRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

  /* Build the IRQ mask word */
  for(uint8_t i=0; i<4; i++)
  {
    tempValue += ((uint32_t)tempRegValue[i])<<(8*(3-i));
  }
  
  /* Rebuild the new mask according to user request */
  if(xNewState == S_DISABLE)
  {
    tempValue &= (~xIrq);
  }
  else
  {
    tempValue |= (xIrq);
  }

  /* Build the array of bytes to write in the IRQ_MASK registers */
  for(uint8_t j=0; j<4; j++)
  {
    tempRegValue[j] = (uint8_t)(tempValue>>(8*(3-j)));
  }
  
  /* Writes the new IRQ mask in the corresponding registers */
  g_xStatus = SpiritSpiWriteRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

}


/**
 * @brief  Fills a pointer to a structure of SpiritIrqs type reading the IRQ_MASK registers.
 * @param  pxIrqMask pointer to a variable of type @ref SpiritIrqs, through which the
 *         user can read which IRQs are enabled. All the bitfields equals to zero correspond
 *         to enabled IRQs, while all the bitfields equals to one correspond to disabled IRQs.
 *         This parameter is a pointer to a SpiritIrqs.
 *         For example suppose that the Power On Reset and RX Data ready are the only enabled IRQs.
 * @code
 * SpiritIrqs myIrqMask;
 * SpiritIrqGetStatus(&myIrqMask);
 * @endcode
 * Then
 * myIrqMask.IRQ_POR and myIrqMask.IRQ_RX_DATA_READY are equal to 0
 * while all the other bitfields are equal to one.
 * @retval None.
 */
void SpiritIrqGetMask(SpiritIrqs* pxIrqMask)
{
  uint8_t tempRegValue[4];
  uint8_t* pIrqPointer = (uint8_t*)pxIrqMask;

  /* Reads IRQ_MASK registers */
  g_xStatus = SpiritSpiReadRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

  /* Build the IRQ mask word */
  for(char i=0; i<4; i++)
  {
    *pIrqPointer = tempRegValue[3-i];
    pIrqPointer++;
  }

}


/**
 * @brief  Filla a pointer to a structure of SpiritIrqs type reading the IRQ_STATUS registers.
 * @param  pxIrqStatus pointer to a variable of type @ref SpiritIrqs, through which the
 *         user can read the status of all the IRQs. All the bitfields equals to one correspond
 *         to the raised interrupts. This parameter is a pointer to a SpiritIrqs.
 *         For example suppose that the XO settling timeout is raised as well as the Sync word
 *         detection.
 * @code
 * SpiritIrqs myIrqStatus;
 * SpiritIrqGetStatus(&myIrqStatus);
 * @endcode
 * Then
 * myIrqStatus.IRQ_XO_COUNT_EXPIRED and myIrqStatus.IRQ_VALID_SYNC are equals to 1
 * while all the other bitfields are equals to zero.
 * @retval None.
 */
void SpiritIrqGetStatus(SpiritIrqs* pxIrqStatus)
{
  uint8_t tempRegValue[4];
  uint8_t* pIrqPointer = (uint8_t*)pxIrqStatus;
  uint32_t* pIrqPointer32=(uint32_t*)pxIrqStatus;

  uint8_t anydebugmessage=0;
  
  /* Reads IRQ_STATUS registers */
  g_xStatus = SpiritSpiReadRegisters(IRQ_STATUS3_BASE, 4, tempRegValue);

  /* Build the IRQ Status word */
  for(uint8_t i=0; i<4; i++)
  {
    *pIrqPointer = tempRegValue[3-i];
     pIrqPointer++;
  } 

  if(IRQ_DEBUG)
  {
    //if(pxIrqStatus->IRQ_RX_DATA_READY) {printf("RX_DATA_READY\t"); anydebugmessage=1;} 
    //if(pxIrqStatus->IRQ_TX_DATA_SENT) {printf("IRQ_TX_DATA_SENT\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_MAX_RE_TX_REACH) {printf("IRQ_MAX_RE_TX_REACH\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_CRC_ERROR) {printf("IRQ_CRC_ERROR\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_TX_FIFO_ERROR) {printf("IRQ_TX_FIFO_ERROR\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_RX_FIFO_ERROR) {printf("IRQ_RX_FIFO_ERROR\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_TX_FIFO_ALMOST_FULL) {printf("IRQ_TX_FIFO_ALMOST_FULL\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_RX_FIFO_ALMOST_FULL) {printf("IRQ_RX_FIFO_ALMOST_FULL\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_RX_FIFO_ALMOST_EMPTY) {printf("IRQ_RX_FIFO_ALMOST_EMPTY\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_TX_FIFO_ALMOST_EMPTY) {printf("IRQ_TX_FIFO_ALMOST_EMPTY\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_MAX_BO_CCA_REACH) {printf("IRQ_MAX_BO_CCA_REACH\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_VALID_PREAMBLE) {printf("IRQ_VALID_PREAMBLE\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_VALID_SYNC) {printf("IRQ_VALID_SYNC\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_RSSI_ABOVE_TH) {printf("IRQ_RSSI_ABOVE_TH\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_WKUP_TOUT_LDC) {printf("IRQ_WKUP_TOUT_LDC\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_READY) {printf("IRQ_READY\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_STANDBY_DELAYED) {printf("IRQ_STANDBY_DELAYED\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_LOW_BATT_LVL) {printf("IRQ_LOW_BATT_LVL\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_POR) {printf("IRQ_POR\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_BOR) {printf("IRQ_BOR\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_LOCK) {printf("IRQ_LOCK\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_PM_COUNT_EXPIRED) {printf("IRQ_PM_COUNT_EXPIRED\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_XO_COUNT_EXPIRED) {printf("IRQ_XO_COUNT_EXPIRED\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_SYNTH_LOCK_TIMEOUT) {printf("IRQ_SYNTH_LOCK_TIMEOUT\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_SYNTH_LOCK_STARTUP) {printf("IRQ_SYNTH_LOCK_STARTUP\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_SYNTH_CAL_TIMEOUT) {printf("IRQ_SYNTH_CAL_TIMEOUT\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_TX_START_TIME) {printf("IRQ_TX_START_TIME\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_RX_START_TIME) {printf("IRQ_RX_START_TIME\t"); anydebugmessage=1;}
    //if(pxIrqStatus->IRQ_RX_TIMEOUT) {printf("IRQ_RX_TIMEOUT\t"); anydebugmessage=1;}
    if(pxIrqStatus->IRQ_AES_END) {printf("IRQ_AES_END\t"); anydebugmessage=1;}
    if(anydebugmessage) printf("\n\n");
  }


  
  
}


/**
 * @brief  Clear the IRQ status registers.
 * @param  None.
 * @retval None.
 */
void SpiritIrqClearStatus(void)
{
  uint8_t tempRegValue[4];

  /* Reads the IRQ_STATUS registers clearing all the flags */
  g_xStatus = SpiritSpiReadRegisters(IRQ_STATUS3_BASE, 4, tempRegValue);

}


/**
 * @brief  Verifies if a specific IRQ has been generated.
 *         The call resets all the IRQ status, so it can't be used in case of multiple raising interrupts.
 * @param  xFlag IRQ flag to be checked.
 *         This parameter can be any value of @ref IrqList.
 * @retval SpiritBool S_TRUE or S_FALSE.
 */
SpiritBool SpiritIrqCheckFlag(IrqList xFlag)
{
  uint8_t tempRegValue[4];
  uint32_t tempValue = 0;
  SpiritBool flag;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_IRQ_LIST(xFlag));

  /* Reads registers and build the status word */
  g_xStatus = SpiritSpiReadRegisters(IRQ_STATUS3_BASE, 4, tempRegValue);
  for(uint8_t i=0; i<4; i++)
  {
    tempValue += ((uint32_t)tempRegValue[i])<<(8*(3-i));
  }
  
  if(tempValue & xFlag)
  {
    flag = S_TRUE;
  }
  else
  {
    flag = S_FALSE;
  }

  return flag;

}


/**
 *@}
 */


/**
 *@}
 */


/**
 *@}
 */




/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
