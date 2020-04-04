/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @author  IPC Rennes
  * @version V2.0.0
  * @date    November 9th, 2018
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stspin32f0_it.h"
#include "6Step_Lib.h"
#ifdef UART_COMM
#include "UART_UI.h"
#endif

/* External variables --------------------------------------------------------*/
extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters; /*!< Main SixStep structure*/
extern ADC_HandleTypeDef ADCx;
extern TIM_HandleTypeDef IF_TIMx;
extern TIM_HandleTypeDef HF_TIMx;
extern TIM_HandleTypeDef LF_TIMx;
extern TIM_HandleTypeDef ZC_TIMx;
extern UART_HandleTypeDef huart;

void MC_Zero_Crossing_Read(void);

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
#ifdef UART_COMM
  uint32_t isrflags   = READ_REG(huart.Instance->ISR);
  uint32_t cr1its     = READ_REG(huart.Instance->CR1);  
  /* UART in mode Receiver ---------------------------------------------------*/
  if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
  { 
    if ((huart.gState!=HAL_UART_STATE_TIMEOUT)&&(huart.gState!=HAL_UART_STATE_ERROR))
    {
      huart.gState = HAL_UART_STATE_READY;
      huart.RxState &= ~HAL_UART_STATE_READY;
      huart.RxState |= HAL_UART_STATE_BUSY_RX;
      if (UART_Receive_IT(&huart)==HAL_OK)
      {
        UART_Set_Value();
      }
    }
    /* Clear RXNE interrupt flag */
    __HAL_UART_SEND_REQ(&huart, UART_RXDATA_FLUSH_REQUEST);
  }
  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
    UART_EndTransmit_IT(&huart);
    return;
  }  
#endif
}

/**
* @brief This function handles DMA USART1 TX interrupt.
*/
void DMA1_Channel2_3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart.hdmatx);
}

/**
* @brief This function handles TIM1 Break, Update, Trigger and Commutation Interrupts.
*/
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* TIM Break input event */
  if(__HAL_TIM_GET_FLAG(&HF_TIMx, TIM_FLAG_BREAK) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(&HF_TIMx, TIM_IT_BREAK) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(&HF_TIMx, TIM_IT_BREAK);
      __HAL_TIM_DISABLE(&HF_TIMx);
      HAL_TIMEx_BreakCallback(&HF_TIMx);
    }
  }
}

/**
* @brief This function handles ADC global interrupt.
*/
void ADC1_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&ADCx);
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&IF_TIMx);
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&LF_TIMx);
}

/**
* @brief This function handles TIM2 Interrupts.
*/
void TIM2_IRQHandler(void)
{
  MC_Zero_Crossing_Read();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
