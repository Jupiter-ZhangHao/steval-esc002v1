/**
 ******************************************************************************
 * @file    MC_Common.h
 * @author  IPC Rennes
 * @version V2.0.0
 * @date    November 9th, 2018
 * @brief   This header file is a common file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_COMMON_H
#define __MC_COMMON_H

/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP    BSP
  * @brief  BSP Layer
  * @{ 
  */

/** @addtogroup Sip_Motor_Driver_handler     Sip_Motor_Driver_handler  
  * @brief  Handler for Silicon In Package Motor Driver such as STSPIN32F0
  * @{ 
  */  

typedef struct
{
  void (*EnableInput_CH1_E_CH2_E_CH3_D)(uint8_t); /*!< Enable the channel 1,2 and Disable the channel 3 */
  void (*EnableInput_CH1_E_CH2_D_CH3_E)(uint8_t); /*!< Enable the channel 1,3 and Disable the channel 2 */
  void (*EnableInput_CH1_D_CH2_E_CH3_E)(uint8_t); /*!< Enable the channel 2,3 and Disable the channel 1 */
  void (*EnableInput_CH1_E_CH2_E_CH3_E)(void);    /*!< Enable all channels */
  void (*DisableInput_CH1_D_CH2_D_CH3_D)(void);   /*!< Disable all channels */
  void (*Start_PWM_driving)(void);                /*!< Start PWM generation */
  void (*Stop_PWM_driving)(void);                 /*!< Stop PWM generation */  
  void (*HF_TIMx_SetDutyCycle_CH1)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH1 */    
  void (*HF_TIMx_SetDutyCycle_CH2)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH2 */
  void (*HF_TIMx_SetDutyCycle_CH3)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH3 */
  void (*HF_TIMx_SetDutyCycle_CH4)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH4 */
  void (*HF_TIMx_SetDutyCycle)(uint16_t, uint8_t);/*!< Set motor phases voltage value for closed loop control */
  void (*Overcurrent_Selection)(uint8_t);         /*!< Select the overcurrent comparator output signal visibility */
  void (*Overcurrent_Threshold_Setvalue)(uint8_t);/*!< Set the overcurrent threshold value and commands standby mode */ 
  void (*ADC_Channel)(uint32_t);                  /*!< Select the ADC channel */  
  uint32_t (*Get_UART_Data)(void);                /*!< Get the UART value from DR register */
} SipMotorDriver_TypeDef;                            /*!< MC driver handler */

/**
  * @}  end Sip_Motor_Driver_handler 
  */

/** @addtogroup Motor_Driver_handler     Motor_Driver_handler  
  * @brief  Handler for generic Motor Driver
  * @{ 
  */  

typedef struct
{
  void (*EnableInput_CH1_E_CH2_E_CH3_D)(uint8_t); /*!< Enable the channel 1,2 and Disable the channel 3 */
  void (*EnableInput_CH1_E_CH2_D_CH3_E)(uint8_t); /*!< Enable the channel 1,3 and Disable the channel 2 */
  void (*EnableInput_CH1_D_CH2_E_CH3_E)(uint8_t); /*!< Enable the channel 2,3 and Disable the channel 1 */
  void (*EnableInput_CH1_E_CH2_E_CH3_E)(void);    /*!< Enable all channels */
  void (*DisableInput_CH1_D_CH2_D_CH3_D)(void);   /*!< Disable all channels */
  void (*Start_PWM_driving)(void);                /*!< Start PWM generation */
  void (*Stop_PWM_driving)(void);                 /*!< Stop PWM generation */
  void (*HF_TIMx_SetDutyCycle_CH1)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH1 */    
  void (*HF_TIMx_SetDutyCycle_CH2)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH2 */
  void (*HF_TIMx_SetDutyCycle_CH3)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH3 */
  void (*HF_TIMx_SetDutyCycle_CH4)(uint16_t);     /*!< High Frequency Timer - Change DutyCycle value for CH4 */  
  void (*HF_TIMx_SetDutyCycle)(uint16_t, uint8_t);/*!< Set motor phases voltage value for closed loop control */
} MotorDriver_TypeDef;                            /*!< MC driver handler */

/**
  * @}  end Motor_Driver_handler 
  */

/**
  * @}  end BSP 
  */

/**
  * @}  end DRIVERS
  */

#endif

