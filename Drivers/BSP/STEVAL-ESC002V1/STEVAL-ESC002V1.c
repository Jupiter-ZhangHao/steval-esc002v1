/**
 ******************************************************************************
 * @file    STEVAL-ESC002V1.c
 * @author  IPC Rennes
 * @version V2.0.0
 * @date    November 9th, 2018
 * @brief   This file provides a set of functions to manage the STEVAL-ESC002V1
 * board
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
#include "STEVAL-ESC002V1.h"

/* External variables --------------------------------------------------------*/
extern SipMotorDriver_TypeDef STSPIN32F0MotorDriver;
/// STEVAL-ESC002V1 timer handler for IF_TIMx
extern TIM_HandleTypeDef IF_TIMx;
/// STEVAL-ESC002V1 timer handler for HF_TIMx
extern TIM_HandleTypeDef HF_TIMx;
/// STEVAL-ESC002V1 timer handler for LF_TIMx
extern TIM_HandleTypeDef LF_TIMx;
/// STEVAL-ESC002V1 adc handler
extern ADC_HandleTypeDef ADCx;
/// STEVAL-ESC002V1 uart handler
extern UART_HandleTypeDef huart;
   
/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP     BSP 
  * @brief  BSP Layer
  * @{ 
  */

/** @addtogroup STEVAL-ESC002V1   STEVAL-ESC002V1
  * @brief  STSPIN32F0 evaluation board
  * @{ 
  */

/******************************************************//**
 * @brief     Turns selected LED On
 * @retval    None
 **********************************************************/
void BSP_BOARD_FAULT_LED_ON(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_FAULT_LED_PORT,BSP_BOARD_FAULT_LED_PIN,GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief     Turns selected LED Off
 * @retval    None
 **********************************************************/
void BSP_BOARD_FAULT_LED_OFF(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_FAULT_LED_PORT,BSP_BOARD_FAULT_LED_PIN,GPIO_PIN_SET);
}

/******************************************************//**
 * @brief     Turns RED LED On
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB1_LED_ON(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_RGB1_LED_PORT,BSP_BOARD_RGB1_LED_PIN,GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief     Turns RED LED Off
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB1_LED_OFF(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_RGB1_LED_PORT,BSP_BOARD_RGB1_LED_PIN,GPIO_PIN_SET);
}

/******************************************************//**
 * @brief     Toggle RED LED
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB1_LED_TOGGLE(void)
{
  HAL_GPIO_TogglePin(BSP_BOARD_RGB1_LED_PORT,BSP_BOARD_RGB1_LED_PIN);
}

/******************************************************//**
 * @brief     Turns GREEN LED On
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB2_LED_ON(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_RGB2_LED_PORT,BSP_BOARD_RGB2_LED_PIN,GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief     Turns GREEN LED Off
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB2_LED_OFF(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_RGB2_LED_PORT,BSP_BOARD_RGB2_LED_PIN,GPIO_PIN_SET);
}

/******************************************************//**
 * @brief     Toggle GREEN LED
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB2_LED_TOGGLE(void)
{
  HAL_GPIO_TogglePin(BSP_BOARD_RGB2_LED_PORT,BSP_BOARD_RGB2_LED_PIN);
}

/******************************************************//**
 * @brief     Turns BLUE LED On
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB3_LED_ON(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_RGB3_LED_PORT,BSP_BOARD_RGB3_LED_PIN,GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief     Turns BLUE LED Off
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB3_LED_TOGGLE(void)
{
  HAL_GPIO_WritePin(BSP_BOARD_RGB3_LED_PORT,BSP_BOARD_RGB3_LED_PIN,GPIO_PIN_SET);
}

/******************************************************//**
 * @brief     Toggle BLUE LED
 * @retval    None
 **********************************************************/
void BSP_BOARD_RGB3_LED_OFF(void)
{
  HAL_GPIO_TogglePin(BSP_BOARD_RGB3_LED_PORT,BSP_BOARD_RGB3_LED_PIN);
}

/******************************************************//**
 * @brief     Enable channels CH1 and CH2 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(uint8_t steps1to3)
{
   STSPIN32F0MotorDriver.EnableInput_CH1_E_CH2_E_CH3_D(steps1to3);
}

/******************************************************//**
 * @brief     Enable channels CH1 and CH3 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(uint8_t steps1to3)
{
  STSPIN32F0MotorDriver.EnableInput_CH1_E_CH2_D_CH3_E(steps1to3);
}

/******************************************************//**
 * @brief     Enable channels CH2 and CH3 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(uint8_t steps1to3)
{
  STSPIN32F0MotorDriver.EnableInput_CH1_D_CH2_E_CH3_E(steps1to3);
}

/******************************************************//**
 * @brief     Enable channels CH1, CH2 and CH3 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_E(void)
{
  STSPIN32F0MotorDriver.EnableInput_CH1_E_CH2_E_CH3_E();
}

/******************************************************//**
 * @brief     Disable channels CH1, CH2 and CH3 for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D(void)
{
  STSPIN32F0MotorDriver.DisableInput_CH1_D_CH2_D_CH3_D();
}

/******************************************************//**
 * @brief     Enable PWM channels for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_Start_PWM_driving(void)
{
  STSPIN32F0MotorDriver.Start_PWM_driving();
}

/******************************************************//**
 * @brief     Disable PWM channels for STSPIN32F0
 * @retval    None
 **********************************************************/
void MC_SixStep_Stop_PWM_driving(void)
{
  STSPIN32F0MotorDriver.Stop_PWM_driving();
}

/******************************************************//**
 * @brief     Set the Duty Cycle value for CH1
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle_CH1(CCR_value);
}

/******************************************************//**
 * @brief     Set the Duty Cycle value for CH2
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle_CH2(CCR_value);
}

/******************************************************//**
 * @brief     Set the Duty Cycle value for CH3
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle_CH3(CCR_value);
}

/******************************************************//**
 * @brief     Set the Duty Cycle value for CH4
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle_CH4(uint16_t CCR_value)
{ 
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle_CH4(CCR_value);
}

/******************************************************//**
 * @brief     Set the value for the voltage to be applied on motor phases
 * @retval    None
 **********************************************************/
void MC_SixStep_HF_TIMx_SetDutyCycle(uint16_t Iref, uint8_t stepNumber)
{
  STSPIN32F0MotorDriver.HF_TIMx_SetDutyCycle(Iref, stepNumber);
}

/******************************************************//**
 * @brief     Select the new ADC Channel
 * @param[in] adc_ch 
 * @retval    None
 **********************************************************/
void MC_SixStep_ADC_Channel(uint32_t adc_ch)
{ 
  STSPIN32F0MotorDriver.ADC_Channel(adc_ch);
}

/**
  * @}  end STEVAL-ESC002V1
  */

/**
  * @}  end BSP
  */

/**
  * @}  end DRIVERS
  */
