/**
  ******************************************************************************
  * @file    stm32f0xx_hal_msp.c
  * @author  IPC Rennes
  * @version V2.0.0
  * @date    November 9th, 2018
  * @brief   HAL MSP module.    
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
#include "main_32F0.h"
#include "stm32f0xx_hal.h"

/* Imported variables --------------------------------------------------------*/
extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;

/* Private variables ---------------------------------------------------------*/
#ifdef TEST
extern uint8_t stop;
#endif

#if defined(ST_PWM_INTERFACE)
/* Captured Values */
uint16_t pwmInput1stRisingEdge = 0;
uint16_t pwmInputFallingEdge = 0;
uint16_t pwmInput2ndRisingEdge = 0; 
uint32_t pwmInputTonCapture = 0;
uint32_t pwmInputToffCapture = 0;

/* Capture index */
uint16_t pwmInputCaptureIndex = 0;

/* Time Values in (2^-16) us */
uint32_t pwmInputTon = 0;
uint32_t pwmInputPeriod = 0;

uint16_t armingCnt = 0;
uint16_t startCnt = 0;
uint16_t stopCnt = 0;
uint16_t blankStopCnt = 0;

uint16_t speedCnt = 0;
int32_t prev_speed_command = 0;
int32_t speed_command = 0;
uint32_t if_cnt_cycle_time = BSP_BOARD_IF_COUNTER_CYCLE_TIME;
uint32_t min_value = BSP_BOARD_IF_TIMx_MIN_SPEED_TON;
uint32_t max_value = BSP_BOARD_IF_TIMx_MAX_SPEED_TON;
uint32_t speed_range = (BSP_BOARD_IF_TIMx_MAX_SPEED_TON - BSP_BOARD_IF_TIMx_MIN_SPEED_TON)>>16;
#endif

/* Private function prototypes -----------------------------------------------*/
extern void MC_TIMx_SixStep_timebase(void);
extern void MC_SysTick_SixStep_MediumFrequencyTask(void);
extern void MC_ADCx_SixStep_User(void);
extern void UART_Send_Reply(void);

/** @defgroup MSP_module
  * @brief HAL MSP module.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief     Initializes the Global MSP 
  * @retval    None
  */
void HAL_MspInit(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, BSP_BOARD_SYSTICK_PRIORITY, 0);
}

/**
  * @brief     Initializes the ADC MSP.
  * @param[in] hadc ADC handle pointer
  * @retval    None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
#if (defined(POTENTIOMETER)||defined(CURRENT_SENSE_ADC)||defined(VBUS_SENSE_ADC)||defined(TEMP_SENSE_ADC))
  ADC_ChannelConfTypeDef sConfig;
#endif  
  if(hadc->Instance==BSP_ADCx)
  { 
    /* Peripheral clock enable */
    BSP_ADCx_CLK_ENABLE();
    
    /* GPIOs Clocks Enable */
    BSP_ADCx_GPIO_CLK_ENABLE();

#ifdef BSP_BOARD_ADCx_GPIOA
    /* GPIO Port Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();    
    /* GPIOs Init */
    GPIO_InitStruct.Pin = BSP_BOARD_ADCx_GPIOA;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
    
    /* System interrupt init*/ 
    HAL_NVIC_SetPriority(BSP_ADCx_IRQn, BSP_ADCx_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_ADCx_IRQn);
    
    /* Configuration of ADC channels */
#ifdef POTENTIOMETER     
    sConfig.Channel = ADC_CH_1; /* Potentiometer */
    sConfig.SamplingTime = ADC_CH_1_ST;
    sConfig.Rank = 1;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
#endif
#ifdef CURRENT_SENSE_ADC  
    sConfig.Channel = ADC_CH_2; /* Current feedabck */
    sConfig.SamplingTime = ADC_CH_2_ST;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
#endif   
#ifdef VBUS_SENSE_ADC    
    sConfig.Channel = ADC_CH_3; /* Bus voltage */
    sConfig.SamplingTime = ADC_CH_3_ST;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
#endif
#ifdef TEMP_SENSE_ADC
    sConfig.Channel = ADC_CH_4; /* Temperature feedback */
    sConfig.SamplingTime = ADC_CH_4_ST;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
#endif  
  }
}

/**
  * @brief     DeInitializes the ADC MSP.
  * @param[in] hadc ADC handle pointer
  * @retval    None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==BSP_ADCx)
  {
    /* Peripheral clock disable */
    BSP_ADCx_CLK_DISABLE();

#ifdef BSP_BOARD_ADCx_GPIOA    
    HAL_GPIO_DeInit(GPIOA, BSP_BOARD_ADCx_GPIOA);
#endif

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(BSP_ADCx_IRQn);
  }
}

/**
  * @brief     Timer MSP Initialization 
  * @param[in] htim_base timer handle pointer
  * @retval    None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==BSP_BOARD_LF_TIMx)
  {
    /* Peripheral clock enable */
    BSP_BOARD_LF_TIMx_CLK_ENABLE();   

    /* System interrupt init*/
    HAL_NVIC_SetPriority(BSP_BOARD_LF_TIMx_IRQn, BSP_BOARD_LF_TIMx_PRIORITY, BSP_BOARD_LF_TIMx_SUB_PRIORITY);
    HAL_NVIC_EnableIRQ(BSP_BOARD_LF_TIMx_IRQn);
    
    /* Stop TIM during Breakpoint */
    BSP_BOARD_LF_TIMx_FREEZE_DBGMCU();    
  }
}

/**
  * @brief     Timer MSP DeInitialization 
  * @param[in] htim_base timer handle pointer
  * @retval    None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==BSP_BOARD_LF_TIMx)
  {
    /* Peripheral clock disable */
    BSP_BOARD_LF_TIMx_CLK_DISABLE();

    /* Peripheral interrupt DeInit */
    HAL_NVIC_DisableIRQ(BSP_BOARD_LF_TIMx_IRQn);
  }
}

#if defined(ST_PWM_INTERFACE)
/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  BSP_BOARD_IF_TIMx_CLK_ENABLE();
  
  /* Enable GPIO channels Clock */
  BSP_BOARD_IF_GPIO_CLK_ENABLE();

  /* Configure  (TIMx_Channel) in Alternate function, push-pull and high speed */
  GPIO_InitStruct.Pin = BSP_BOARD_IF_TIMx_CH1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = BSP_BOARD_IF_TIMx_AF;
  HAL_GPIO_Init(BSP_BOARD_IF_TIMx_CH1_PORT , &GPIO_InitStruct);
  
  /*##-2- Configure the NVIC for TIMx #########################################*/

  HAL_NVIC_SetPriority(BSP_BOARD_IF_TIMx_IRQn, BSP_BOARD_IF_TIMx_PRIORITY, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(BSP_BOARD_IF_TIMx_IRQn);

}
#endif

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  
  if(htim->Instance==BSP_HF_TIMx)
  { 
    /* Peripheral clock enable */
    BSP_HF_TIMx_CLK_ENABLE();

    /* GPIOs Clocks Enable */
    BSP_HF_GPIO_CLK_ENABLE();    

    /* HF_TIMx Break Input Configuration */
    GPIO_InitStruct.Pin = BSP_HF_TIMx_BKIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = BSP_HF_TIMx_BKIN_AF;
    HAL_GPIO_Init(BSP_HF_TIMx_BKIN_PORT, &GPIO_InitStruct);
    
    /* HF_TIMx OUTPUTs Configuration */
    GPIO_InitStruct.Pin = BSP_HF_TIMx_CH1_PIN|BSP_HF_TIMx_CH2_PIN|BSP_HF_TIMx_CH3_PIN;
    GPIO_InitStruct.Mode = BSP_HF_TIMx_MODE;
    GPIO_InitStruct.Pull = BSP_HF_TIMx_PULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = BSP_HF_TIMx_AF;
    HAL_GPIO_Init(BSP_HF_TIMx_CHx_PORT, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = BSP_HF_TIMx_CH1N_PIN|BSP_HF_TIMx_CH2N_PIN|BSP_HF_TIMx_CH3N_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#ifdef COMPLEMENTARY_DRIVE
    GPIO_InitStruct.Mode = BSP_HF_TIMx_MODE;
    GPIO_InitStruct.Pull = BSP_HF_TIMx_PULL;      
    GPIO_InitStruct.Alternate = BSP_HF_TIMx_AF;
#else
    GPIO_InitStruct.Mode = BSP_HF_TIMx_ENx_MODE;
    GPIO_InitStruct.Pull = BSP_HF_TIMx_ENx_PULL;     
#endif
    HAL_GPIO_Init(BSP_HF_TIMx_CHxN_PORT, &GPIO_InitStruct);

    /* Enable and set priority for the HF_TIMx interrupt */
    HAL_NVIC_SetPriority(BSP_HF_TIMx_IRQn, BSP_HF_TIMx_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_HF_TIMx_IRQn);
    
    /* Stop TIM during Breakpoint */
    BSP_HF_TIMx_FREEZE_DBGMCU();

    /* Enable TIM break interrupt */
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_BREAK);
    
  }
  if(htim->Instance==BSP_BOARD_ZC_TIMx)
  {
    /* TIMx Peripheral clock enable */
    BSP_BOARD_ZC_TIMx_CLK_ENABLE();
  
    /* Set the TIMx priority */
    HAL_NVIC_SetPriority(BSP_BOARD_ZC_TIMx_IRQn, BSP_BOARD_ZC_TIMx_PRIORITY, BSP_BOARD_ZC_TIMx_SUB_PRIORITY);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_BOARD_ZC_TIMx_IRQn);
  }
}

/**
  * @brief  DeInitializes TIM PWM MSP.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==BSP_HF_TIMx)
  {
    /* Peripheral clock disable */
    BSP_HF_TIMx_CLK_DISABLE();
    
    HAL_GPIO_DeInit(BSP_HF_TIMx_CHx_PORT, BSP_HF_TIMx_CH1_PIN|BSP_HF_TIMx_CH2_PIN|BSP_HF_TIMx_CH3_PIN);
    HAL_GPIO_DeInit(BSP_HF_TIMx_CHxN_PORT, BSP_HF_TIMx_CH1N_PIN|BSP_HF_TIMx_CH2N_PIN|BSP_HF_TIMx_CH3N_PIN);
     
    HAL_GPIO_DeInit(BSP_HF_TIMx_BKIN_PORT, BSP_HF_TIMx_BKIN_PIN);  

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(BSP_HF_TIMx_IRQn);
  }
  else if(htim->Instance==BSP_BOARD_ZC_TIMx) 
  {
    /* Peripheral clock disable */
    BSP_BOARD_ZC_TIMx_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(BSP_HF_TIMx_IRQn);    
  }
}
 
/**
  * @brief     UART MSP Initialization 
  * @param[in] huart UART handle pointer
  * @retval    None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  static DMA_HandleTypeDef hdma_tx;  
  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==BSP_UART)
  { 
    /* DMA Clock Enable */
    BSP_DMA_CLK_ENABLE();
    
    /* Peripheral clock enable */
    BSP_UART_CLK_ENABLE();
    
    /* GPIO Ports Clock Enable */
    BSP_UART_GPIO_CLK_ENABLE();

    /* USART GPIO Configuration */
    GPIO_InitStruct.Pin = BSP_UART_PINS;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_UART_AF;
    HAL_GPIO_Init(BSP_UART_PORT, &GPIO_InitStruct);

    /* DMA TX handle configuration */    
    hdma_tx.Instance = BSP_DMA;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);
    
    /* System interrupt init*/
    HAL_NVIC_SetPriority(BSP_DMA_IRQn, BSP_DMA_PRIORITY, BSP_DMA_PRIORITY_SUB);
    HAL_NVIC_EnableIRQ(BSP_DMA_IRQn);    
    HAL_NVIC_SetPriority(BSP_UART_IRQn, BSP_UART_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BSP_UART_IRQn);
  }
}

/**
  * @brief     UART MSP DeInitialization 
  * @param[in] huart UART handle pointer
  * @retval    None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  static DMA_HandleTypeDef hdma_tx;
  if(huart->Instance==BSP_UART)
  {
    /* DMA Clock Disable */
    BSP_DMA_CLK_DISABLE();
    
    /* Peripheral clock disable */
    BSP_UART_CLK_DISABLE();
  
    /* UART GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_UART_PORT, BSP_UART_PINS);
    
    /* De-Initialize the DMA Channel associated to transmission process */
    HAL_DMA_DeInit(&hdma_tx);    

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(BSP_DMA_IRQn);
    HAL_NVIC_DisableIRQ(BSP_UART_IRQn);
  }
}

/**
  * @}
  */

/* Callbacks -----------------------------------------------------------------*/

/** @defgroup HAL_MSP_Callbacks
  * @{
  */

/**
  * @brief     ADC conversion complete callback 
  * @param[in] hadcADC handle pointer  
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  MC_ADCx_SixStep_User();
}

/**
  * @brief     timer period elapsed callback 
  * @param[in] htim TIM handle pointer  
  * @retval    None
  */  
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==BSP_BOARD_LF_TIMx)
  {
    MC_TIMx_SixStep_timebase();   
  }
#if defined(ST_PWM_INTERFACE)
  else if (htim->Instance==BSP_BOARD_IF_TIMx)
  {
    if (SIXSTEP_parameters.STATUS!=STOP)
    {
      if (blankStopCnt>BSP_BOARD_IF_TIMx_STOP_PERIODS)
      {
        MC_StopMotor();
      }
      else
      {
        blankStopCnt++;
      }
    }
  }
#endif
}

/**
  * @brief  Break detection callback in non blocking mode 
  * @param[in] htim : TIM handle
  * @retval None
  */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  MC_StopMotor();
  SIXSTEP_parameters.overcurrent++;
  //HAL_GPIO_WritePin(GPIO_PORT_TRIG,GPIO_CH_TRIG,GPIO_PIN_SET);
}

/**
  * @brief     Systick callback
  * @retval None
  */
void HAL_SYSTICK_Callback()
{
    MC_SysTick_SixStep_MediumFrequencyTask();
}

/**
  * @brief  UART transmit callback 
  * @param  huart
  * @retval None
*/
#ifdef UART_COMM
/**
  * @brief     UART transmit complete callback
  * @param[in] huart UART handle pointer 
  * @retval    None
  */
  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  {
    if (SIXSTEP_parameters.UART_TX_REPLY != 0)
    {
      SIXSTEP_parameters.UART_TX_REPLY = 0;
    }
    else if (SIXSTEP_parameters.UART_TX_DIFFERED_REPLY != 0)
    {
      SIXSTEP_parameters.UART_TX_DIFFERED_REPLY = 0;
      UART_Send_Reply();
    }    
    if (SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_MODE != 0)
    {
      SIXSTEP_parameters.UART_CONTINUOUS_TX_SPEED_ALLOWED = 1;
    }
    else if (SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_MODE != 0)
    {
      SIXSTEP_parameters.UART_CONTINUOUS_TX_BEMF_ALLOWED = 1;
    }
  }
#endif

#if defined(ST_PWM_INTERFACE)
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  htim : hadc handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance==BSP_BOARD_IF_TIMx)&&(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    //if(pwmInputCaptureIndex == 0)
    if (LL_TIM_IC_GetPolarity(htim->Instance,LL_TIM_CHANNEL_CH1)==LL_TIM_IC_POLARITY_RISING)
    {
      /* Get the 1st Input Capture value */
      pwmInput1stRisingEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      
      /* Capture computation */
      if (pwmInput1stRisingEdge > pwmInputFallingEdge)
      {
        pwmInputToffCapture = (pwmInput1stRisingEdge - pwmInputFallingEdge); 
      }
      else if (pwmInput1stRisingEdge < pwmInputFallingEdge)
      {
        /* 0xFFFF is max TIM1_CCRx value */
        pwmInputToffCapture = ((0xFFFF - pwmInputFallingEdge) + pwmInput1stRisingEdge) + 1;
      }
      else
      {
        /* If capture values are equal, we have reached the limit of frequency
           measures */
        Error_Handler();
      }
      pwmInputPeriod = pwmInputTon + pwmInputToffCapture*if_cnt_cycle_time;
      //pwmInputCaptureIndex = 1;
      LL_TIM_IC_SetPolarity(htim->Instance, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_FALLING);
      blankStopCnt = 0;
    }
    //else if(pwmInputCaptureIndex == 1)
    else if (LL_TIM_IC_GetPolarity(htim->Instance,LL_TIM_CHANNEL_CH1)!=LL_TIM_IC_POLARITY_RISING)
    {
      /* Get the 2nd Input Capture value */
      pwmInputFallingEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

      /* Capture computation */
      if (pwmInputFallingEdge > pwmInput1stRisingEdge)
      {
        pwmInputTonCapture = (pwmInputFallingEdge - pwmInput1stRisingEdge); 
      }
      else if (pwmInputFallingEdge < pwmInput1stRisingEdge)
      {
        /* 0xFFFF is max TIM1_CCRx value */
        pwmInputTonCapture = ((0xFFFF - pwmInput1stRisingEdge) + pwmInputFallingEdge) + 1;
      }
      else
      {
        /* If capture values are equal, we have reached the limit of frequency
           measures */
        Error_Handler();
      }
      /* Period computation */    
      pwmInputTon = pwmInputTonCapture*if_cnt_cycle_time;

      if (pwmInputTon<min_value)
      {
        if (SIXSTEP_parameters.STATUS!=STOP)
        {
          if (stopCnt>BSP_BOARD_IF_TIMx_STOP_VALID_TON)
          {
            MC_StopMotor();
          }
          else
          {
            stopCnt++;
          }
        }
        else if (armingCnt==BSP_BOARD_IF_TIMx_ARMING_VALID_TON)
        {
          startCnt=1;
        }
        else if (SIXSTEP_parameters.STATUS==STOP)
        {
          armingCnt++;
        }
      }
      else if (armingCnt!=0)
      {
        if ((pwmInputTon>=min_value)&&(pwmInputTon<=max_value))
        {
          if (startCnt>BSP_BOARD_IF_TIMx_START_VALID_TON)
          {
            armingCnt=0;
            startCnt=0;
            stopCnt=0;
            speedCnt = 0;
            MC_StartMotor();
          }
          else if (startCnt!=0)
          {
            startCnt++;
          }
        }
      }
      else if (startCnt==0)
      {
        prev_speed_command = speed_command;
        speed_command = (pwmInputTon - min_value)>>16;
        if (speed_command==prev_speed_command)
        {
          speedCnt++;
        }
        else
        {
          speedCnt=0;
        }
        if (speedCnt>=10)
        {
          speedCnt = 0;
#if defined(OPEN_LOOP)          
          if ((speed_command>=0)&&(speed_command<=speed_range))
#else          
          if ((speed_command>=0)&&(speed_command<=speed_range)&&(PI_parameters.ReferenceToBeUpdated==0))
#endif            
          {
            stopCnt=0;
#if defined(OPEN_LOOP)
            SIXSTEP_parameters.pulse_command = ((speed_command*(SIXSTEP_parameters.HF_TIMx_ARR-SIXSTEP_parameters.startup_reference))>>BSP_BOARD_IF_TIMx_MIN2MAX_BITS)+SIXSTEP_parameters.startup_reference;
#else
            SIXSTEP_parameters.speed_target = ((speed_command*(MAX_SPEED-MIN_SPEED))>>BSP_BOARD_IF_TIMx_MIN2MAX_BITS)+MIN_SPEED;
            if (SIXSTEP_parameters.speed_target>MAX_SPEED)
            {
              SIXSTEP_parameters.speed_target=MAX_SPEED;
            }
            PI_parameters.ReferenceToBeUpdated++;
#endif            
          }
        }
      }
      //pwmInputCaptureIndex = 0;
      LL_TIM_IC_SetPolarity(htim->Instance, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    }
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
