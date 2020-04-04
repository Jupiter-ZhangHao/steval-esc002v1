/**
 ******************************************************************************
 * @file    STEVAL-ESC002V1.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STEVAL_ESC002V1_H
#define __STEVAL_ESC002V1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "stspin32f0.h"

/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP
  * @{
  */   

/** @addtogroup STEVAL-ESC002V1
  * @{   
  */

/* Exported Constants --------------------------------------------------------*/
/** @defgroup STEVAL-ESC002V1_Exported_Constants STEVAL-ESC002V1 Exported Constants
  * @{
  */

/** @defgroup STEVAL-ESC002V1_TIMER_Related_Constants STEVAL-ESC002V1 Timer Related Constants
  * @{
  */

#define BSP_BOARD_SYSTICK_PRIORITY           (3)

#define BSP_BOARD_HF_TIMx_SLAVE_MODE         (TIM_SLAVEMODE_TRIGGER)
#define BSP_BOARD_HF_TIMx_TS_ITR             (TIM_TS_ITR2)
#define BSP_BOARD_HF_TIMx_TRGO               (TIM_TRGO_OC4REF)
#define BSP_BOARD_HF_TIMx_BREAK_STATE        (TIM_BREAK_ENABLE)
#define BSP_BOARD_HF_TIMx_BREAK_POL          (TIM_BREAKPOLARITY_HIGH)
#define BSP_BOARD_HF_TIMx_BREAK_OUT          (TIM_AUTOMATICOUTPUT_DISABLE)

#define BSP_BOARD_LF_TIMx                    TIM3
#define BSP_BOARD_LF_TIMx_COUNTER_MODE       TIM_COUNTERMODE_UP
#define BSP_BOARD_LF_TIMx_CLK_ENABLE()       __HAL_RCC_TIM3_CLK_ENABLE()
#define BSP_BOARD_LF_TIMx_CLK_DISABLE()      __HAL_RCC_TIM3_CLK_DISABLE()
#define BSP_BOARD_LF_TIMx_FREEZE_DBGMCU()    __HAL_DBGMCU_FREEZE_TIM3()    
#define BSP_BOARD_LF_TIMx_IRQn               TIM3_IRQn
#define BSP_BOARD_LF_TIMx_PRIORITY           (0)
#define BSP_BOARD_LF_TIMx_SUB_PRIORITY       (1)
#define BSP_BOARD_LF_TIMx_TRGO               (TIM_TRGO_UPDATE)

#define BSP_BOARD_ZC_TIMx                    TIM2
#define BSP_BOARD_ZC_TIMx_COUNTER_MODE       TIM_COUNTERMODE_UP
#define BSP_BOARD_ZC_TIMx_CLK_ENABLE()       __HAL_RCC_TIM2_CLK_ENABLE()
#define BSP_BOARD_ZC_TIMx_CLK_DISABLE()      __HAL_RCC_TIM2_CLK_DISABLE()
#define BSP_BOARD_ZC_TIMx_FREEZE_DBGMCU()    __HAL_DBGMCU_FREEZE_TIM2()    
#define BSP_BOARD_ZC_TIMx_IRQn               TIM2_IRQn
#define BSP_BOARD_ZC_TIMx_PRIORITY           (0)
#define BSP_BOARD_ZC_TIMx_SUB_PRIORITY       (0)
#define BSP_BOARD_ZC_TIMx_SLAVE_MODE         (TIM_SLAVEMODE_RESET)
#define BSP_BOARD_ZC_TIMx_TS_ITR             (TIM_TS_ITR0)

#define BSP_BOARD_IF_TIMx                    TIM16
#define BSP_BOARD_IF_TIMx_COUNTER_MODE       TIM_COUNTERMODE_UP
#define BSP_BOARD_IF_TIMx_CLK_ENABLE()       __HAL_RCC_TIM16_CLK_ENABLE()
#define BSP_BOARD_IF_TIMx_CLK_DISABLE()      __HAL_RCC_TIM16_CLK_DISABLE()
#define BSP_BOARD_IF_TIMx_FREEZE_DBGMCU()    __HAL_DBGMCU_FREEZE_TIM16()    
#define BSP_BOARD_IF_TIMx_IRQn               TIM16_IRQn
#define BSP_BOARD_IF_TIMx_PRIORITY           (2)
#define BSP_BOARD_IF_TIMx_PSC                (1)

#define BSP_BOARD_IF_COUNTER_CYCLE_TIME      (((1<<16)/(SYSCLOCK_FREQUENCY/1000000))*(BSP_BOARD_IF_TIMx_PSC+1)) //(2^-16) us    
#define BSP_BOARD_IF_TIMx_ARR                (0xFFFF)
    
#if defined(ST_PWM_INTERFACE)
#define BSP_BOARD_IF_TIMx_STOP_MS            (1500) 
#define BSP_BOARD_IF_TIMx_PERIOD_US          (((BSP_BOARD_IF_TIMx_ARR+1)*BSP_BOARD_IF_COUNTER_CYCLE_TIME)>>16)
#define BSP_BOARD_IF_TIMx_STOP_PERIODS       ((BSP_BOARD_IF_TIMx_STOP_MS*1000)/(BSP_BOARD_IF_TIMx_PERIOD_US))
#define BSP_BOARD_IF_TIMx_ARMING_VALID_TON   (10)
#define BSP_BOARD_IF_TIMx_START_VALID_TON    (10)
#define BSP_BOARD_IF_TIMx_STOP_VALID_TON     (1000)
#define BSP_BOARD_IF_TIMx_MIN2MAX_BITS       (10)    
#define BSP_BOARD_IF_TIMx_MIN_SPEED_TON_US   (1060)
#define BSP_BOARD_IF_TIMx_MAX_SPEED_TON_US   (BSP_BOARD_IF_TIMx_MIN_SPEED_TON_US+(1<<BSP_BOARD_IF_TIMx_MIN2MAX_BITS))
#define BSP_BOARD_IF_TIMx_MIN_SPEED_TON      (BSP_BOARD_IF_TIMx_MIN_SPEED_TON_US<<16) //(2^-16) us
#define BSP_BOARD_IF_TIMx_MAX_SPEED_TON      (BSP_BOARD_IF_TIMx_MAX_SPEED_TON_US<<16) //(2^-16) us

#endif
    
    


/**
  * @} end STEVAL-ESC002V1 TIMER Related Constants
  */

/** @defgroup STEVAL-ESC002V1_ADC_Related_Constants STEVAL-ESC002V1 ADC Related Constants
  * @{
  */

#ifdef POTENTIOMETER
  #define ADC_CH_1              ADC_CHANNEL_6   /*SPEED*/
#endif

#ifdef CURRENT_SENSE_ADC
  #define ADC_CH_2              ADC_CHANNEL_4   /*CURRENT*/
#endif

#ifdef VBUS_SENSE_ADC
  #define ADC_CH_3              ADC_CHANNEL_3   /*VBUS*/
#endif

#ifdef TEMP_SENSE_ADC
  #define ADC_CH_4              ADC_CHANNEL_16  /*TEMP*/
#endif

#ifdef POTENTIOMETER
  #define ADC_CH_1_ST           ADC_SAMPLETIME_7CYCLES_5 /*SPEED sampling time*/
#endif

#ifdef CURRENT_SENSE_ADC
  #define ADC_CH_2_ST           ADC_SAMPLETIME_1CYCLE_5 /*CURRENT sampling time */
#endif

#ifdef VBUS_SENSE_ADC
  #define ADC_CH_3_ST           ADC_SAMPLETIME_1CYCLE_5 /*VBUS sampling time*/
#endif

#ifdef TEMP_SENSE_ADC
  #define ADC_CH_4_ST           ADC_SAMPLETIME_1CYCLE_5 /*TEMP sampling time*/
#endif

#define ADC_CH_INIT           ADC_CH_1
#define ADC_CH_INIT_ST        ADC_CH_1_ST

/**
  * @} end STEVAL-ESC002V1 ADC Related Constants
  */

/** @defgroup STEVAL-ESC002V1_GPIO_Related_Constants STEVAL-ESC002V1 GPIO Related Constants
  * @{
  */

#define BSP_BOARD_ADCx_GPIOA            (/*GPIO_PIN_3|*/GPIO_PIN_4|GPIO_PIN_6)
    
#define BSP_BOARD_ZCU_PIN               (GPIO_PIN_1)
#define BSP_BOARD_ZCU_PORT              (GPIOF)   
#define BSP_BOARD_ZCV_PIN               (GPIO_PIN_0)
#define BSP_BOARD_ZCV_PORT              (GPIOF)   
#define BSP_BOARD_ZCW_PIN               (GPIO_PIN_1)
#define BSP_BOARD_ZCW_PORT              (GPIOB)

#define GPIO_PORT_ARR                   (GPIOB)       /*!< GPIO port name for zero crossing detection */
#define GPIO_CH_ARR                     (GPIO_PIN_6)  /*!< GPIO pin name for zero crossing detection */
#define GPIO_PORT_TRIG                  (GPIOA)       /*!< GPIO port name for zero crossing detection */
#define GPIO_CH_TRIG                    (GPIO_PIN_5)  /*!< GPIO pin name for zero crossing detection */
#define GPIO_PORT_ZCR                   (GPIOA)       /*!< GPIO port name for zero crossing detection */
#define GPIO_CH_ZCR                     (GPIO_PIN_5)  /*!< GPIO pin name for zero crossing detection */
#define GPIO_PORT_COMM                  (GPIOA)       /*!< GPIO port name for 6Step commutation */
#define GPIO_CH_COMM                    (GPIO_PIN_7)  /*!< GPIO pin name for 6Step commutation */

#define BSP_BOARD_RGB1_LED_PIN          (GPIO_PIN_0)
#define BSP_BOARD_RGB1_LED_PORT         (GPIOA)
#define BSP_BOARD_RGB2_LED_PIN          (GPIO_PIN_1)
#define BSP_BOARD_RGB2_LED_PORT         (GPIOA)
#define BSP_BOARD_RGB3_LED_PIN          (GPIO_PIN_2)
#define BSP_BOARD_RGB3_LED_PORT         (GPIOA)
    
#define BSP_BOARD_FAULT_LED_PIN         (BSP_BOARD_RGB1_LED_PIN)
#define BSP_BOARD_FAULT_LED_PORT        (BSP_BOARD_RGB1_LED_PORT)

#define BSP_BOARD_IF_TIMx_CH1_PIN       (GPIO_PIN_6)
#define BSP_BOARD_IF_TIMx_CH1_PORT      (GPIOA)
#define BSP_BOARD_IF_TIMx_AF            (GPIO_AF5_TIM16)
#define BSP_BOARD_IF_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

/**
  * @} end STEVAL-ESC002V1 GPIO Related Constants
  */

/** @defgroup STEVAL-ESC002V1_UART_Related_Constants STEVAL-ESC002V1 UART Related Constants
  * @{
  */

#define STARTM_CMD             0     /*!<  Start Motor command received */
#define STOPMT_CMD             1     /*!<  Stop Motor command received */
#define SETSPD_CMD             2     /*!<  Set the new speed value command received */
#define GETSPD_CMD             3     /*!<  Get Mechanical Motor Speed command received */
#define INIREF_CMD             4     /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */
#define POLESP_CMD             5     /*!<  Set the Pole Pairs value command received */
#define ACCELE_CMD             6     /*!<  Set the Accelleration for Start-up of the motor command received */
#define DMGCTR_CMD             7     /*!<  Enable the DEMAG dynamic control command received */
#define MAXDMG_CMD             8     /*!<  Set the BEMF Demagn MAX command received */
#define MINDMG_CMD             9     /*!<  Set the BEMF Demagn MIN command received */
#define KP_PRM_CMD             10    /*!<  Set the KP PI param command received */
#define KI_PRM_CMD             11    /*!<  Set the KI PI param command received */
#define MEASEL_CMD             12    /*!<  Set the continuous measurement to be performed */
#define HELP_CMD               13    /*!<  Help command received */
#define STATUS_CMD             14    /*!<  Get the Status of the system command received */
#define DIRECT_CMD             15    /*!<  Set the motor direction */
#define SETDCY_CMD             16    /*!<  Set the HF PWM duty cycle */

/**
  * @} end STEVAL-ESC002V1 UART Related Constants
  */

/**
  * @} end STEVAL-ESC002V1 Exported Constants
  */

/* Exported Functions  -------------------------------------------------------*/
/** @defgroup STEVAL-ESC002V1_Exported_Functions STEVAL-ESC002V1 Exported Functions
  * @{
  */

void BSP_BOARD_FAULT_LED_ON(void);
void BSP_BOARD_FAULT_LED_OFF(void);
void BSP_BOARD_RGB1_LED_ON(void);
void BSP_BOARD_RGB1_LED_OFF(void);
void BSP_BOARD_RGB1_LED_TOGGLE(void);
void BSP_BOARD_RGB2_LED_ON(void);
void BSP_BOARD_RGB2_LED_OFF(void);
void BSP_BOARD_RGB2_LED_TOGGLE(void);
void BSP_BOARD_RGB3_LED_ON(void);
void BSP_BOARD_RGB3_LED_OFF(void);
void BSP_BOARD_RGB3_LED_TOGGLE(void);
#ifndef VOLTAGE_MODE
void MC_SixStep_Current_Reference_Start(void);
void MC_SixStep_Current_Reference_Stop(void);
void MC_SixStep_Current_Reference_Setvalue(uint16_t, uint8_t);
#endif
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(uint8_t);
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(uint8_t);
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(uint8_t);
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_E(void);
void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D(void);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH4(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle(uint16_t, uint8_t);
void MC_SixStep_ADC_Channel(uint32_t);

/**
  * @} end STEVAL-ESC002V1 Exported Functions
  */

/**
  * @} end STEVAL-ESC002V1
  */

/**
  * @} end BSP
  */

/**
  * @} end DRIVERS
  */
  
#endif /* __STEVAL_ESC002V1_H */

