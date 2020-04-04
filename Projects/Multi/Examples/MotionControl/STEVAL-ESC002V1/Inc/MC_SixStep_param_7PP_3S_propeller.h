/**
 ******************************************************************************
 * @file    MC_SixStep_param_7PP_3S_propeller.h
 * @author  IPC Rennes
 * @version V2.0.0
 * @date    November 9th, 2018
 * @brief   This header file provides all parameters to driver a motor with 6Step
            library
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

/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
*/

/** @addtogroup MC_6-STEP_LIB       MC_6-STEP LIB 
  * @brief  Motor Control driver
  * @{ 
*/

/** @defgroup Main_Motor_parameters    Main_Motor_parameters
  *  @{
    * @brief All motor parameters for 6Step driving
*/
 
/* **************************************************************************** 
 ==============================================================================   
           ###### BASIC PARAMETERS ######
 ============================================================================== 
**************************************************************************** */   
#define NUM_POLE_PAIRS                       7      /*!< Number of Motor Pole pairs */
#define DIRECTION                            0      /*!< Set motor direction CW = 0 and CCW = 1*/
#define TARGET_SPEED_OPEN_LOOP            2000      /*!< Target speed in open loop control */
#define TARGET_SPEED                     20000      /*!< Target speed in closed loop control */

#define MIN_SPEED                         3000
#define MAX_SPEED                        20000      /*!< Motor rated max speed */

/* **************************************************************************** 
 ==============================================================================   
           ###### ADVANCED PARAMETERS VOLTAGE MODE ######
 ============================================================================== 
**************************************************************************** */
/*!< ********************* Open loop control *********************************/
#if defined(FAST_DEMAG)
  #define STARTUP_DUTY_CYCLE                   50  /*!< Tenths of percentage of PWM on time */
#else
  #define STARTUP_DUTY_CYCLE                   70  /*!< Tenths of percentage of PWM on time */
#endif

/*!< ********************* Closed Loop control *********************************/
#if defined(PID_V2)
    #define KP_GAIN                           32       /*!< Kp parameter for PID regulator */
    #define KI_GAIN                            4       /*!< Ti parameter for PID regulator */
    #define KD_GAIN                            1      /*!< Td parameter for PID regulator */             
#else
  #define KP_GAIN                         4096     /*!< Kp parameter for PI regulator */
#endif
#define K_GAIN_SCALING                      16     /*!< Kp, Ki, (Kd) scaling for PI(D) regulator */
#define LOWER_OUT_LIMIT  ((5*HF_TIMX_ARR)/100)     /*!< Low Out value of PI regulator */
#define UPPER_OUT_LIMIT           (HF_TIMX_ARR)     /*!< High Out value of PI regulator */

/* **************************************************************************** 
 ==============================================================================   
           ###### ADVANCED PARAMETERS COMMON ######
 ============================================================================== 
**************************************************************************** */
/*!< ********************* Gate driving **************************************/
#define GATE_DRIVING_PWM_FREQUENCY      30000     /*!< Hz */
#define SYSCLOCK_FREQUENCY            48000000     /*!< Hz */
#define HF_TIMX_PSC                          0
#define HF_TIMX_ARR (SYSCLOCK_FREQUENCY/(GATE_DRIVING_PWM_FREQUENCY*2*(HF_TIMX_PSC+1))-1)
#define DEAD_TIME_NS        (400)
#define DEAD_TIME           ((DEAD_TIME_NS/HF_COUNTER_CYCLE_TIME_NS)+1)
#define PULSE               ((4*HF_TIMX_ARR)/5)

/*!< ********************* Step timer ****************************************/
#define LF_TIMX_PSC                      11
#define LF_TIMX_ARR                      65535
#define LF_COUNTER_CYCLE_TIME_NS         ((1000000000/(SYSCLOCK_FREQUENCY))*(LF_TIMX_PSC+1))
#define LF_TIMX_ARR_GUARD_TIME_NS        (700)
#define LF_TIMX_ARR_GUARD_TIME_CYC       ((LF_TIMX_ARR_GUARD_TIME_NS/LF_COUNTER_CYCLE_TIME_NS)+1)

/*!< ********************* Open loop control *********************************/
#define ACC                              70000     /*!< Mechanical acceleration rate (setting available in manual mode, LOAD_TYPE = 0) */
#define MINIMUM_ACC                        500     /*!< Mechanical acceleration rate for BIG load application */
#define NUMBER_OF_STEPS                  10000     /*!< Number of elements for motor start-UP (max value 65535)*/
#define TIME_FOR_ALIGN                    200     /*!< Time for alignment (msec)*/
#define BUTTON_DELAY                      1000     /*!< Delay time to enable push button for new command (1 = 1msec)*/

/*!< ********************* Closed Loop control *******************************/
/*!< Zero Crossing parameters */
#define ZCD_TO_COMM                       200    /*!< Zero Crossing detection to commutation delay in 15/128 degrees */
#define MIN_ZCD_TO_COMM                       (20)
#define VARIABLE_ADVANCE_MUL                  (9)
#define VARIABLE_ADVANCE_SHIFT                (10)
#if defined(ST_PWM_INTERFACE)  
  #define ZC_TIMX_FREQUENCY_HZ                (150000)
#else
  #define ZC_TIMX_FREQUENCY_HZ                (200000)
#endif  
#define ZC_TIMX_PSC                         (0)
#define ZC_COUNTER_CYCLE_TIME_NS            ((1000000000/(SYSCLOCK_FREQUENCY))*(ZC_TIMX_PSC+1))
#define PWM_EDGE_TO_ZC_READ_EXTRA_DELAY_NS  (0)  /*!< Additional delay between the gate driver PWM edge and the Zero Crossing reading in ns */
#define PWM_EDGE_TO_ZC_READ_EXTRA_DELAY_CYC (PWM_EDGE_TO_ZC_READ_EXTRA_DELAY_NS/ZC_COUNTER_CYCLE_TIME_NS) /*!< Additional delay between the gate driver PWM edge and the Zero Crossing reading in ZC counter cycles */
#define HF_COUNTER_CYCLE_TIME_NS            ((1000000000/(SYSCLOCK_FREQUENCY))*(HF_TIMX_PSC+1))
#define ZC_READ_TO_PWM_EDGE_PRE_GUARD_TIME_NS   (1500)//(500)
#define ZC_READ_TO_PWM_EDGE_PRE_GUARD_TIME_CYC  (ZC_READ_TO_PWM_EDGE_PRE_GUARD_TIME_NS/HF_COUNTER_CYCLE_TIME_NS)
#define ZC_READ_TO_PWM_EDGE_POST_GUARD_TIME_NS   (2000) // (1000)
#define ZC_READ_TO_PWM_EDGE_POST_GUARD_TIME_CYC  (ZC_READ_TO_PWM_EDGE_POST_GUARD_TIME_NS/HF_COUNTER_CYCLE_TIME_NS)

/*!< Speed parameters */
#define SPEED_LOOP_TIME                      1     /*!< Speed Loop time (1 = 1msec) */
#define FILTER_DEEP_SHIFT                    4
#define FILTER_DEEP     (1<<FILTER_DEEP_SHIFT)     /*!< Number of bits for digital filter */

/*!< Motor stall detection parameters */
#define BEMF_CONSEC_DOWN_MAX                10     /*!< Maximum value of BEMF Consecutive Threshold Falling Crossings Counter in closed loop */

/*!< ********************* Debug *********************************************/
/*!< Debug pin */
#define GPIO_ZERO_CROSS                      0     /*!< Enable (1) the GPIO toggling for zero crossing sensing */
#define GPIO_COMM                            0     /*!< Enable (1) the GPIO toggling for commutation */

/*!< ********************* Demo **********************************************/
/*!< Demo mode parameters */
#define DEMO_START_TIME                  10000     /*!< Time (msec) to keep the motor in run mode */
#define DEMO_STOP_TIME                    5000     /*!< Time (msec) to keep the motor in stop mode */

/*!< ********************* General *******************************************/
#define TRUE                                 1     /*!< Define TRUE */
#define FALSE                                0     /*!< Define FALSE */

/**
  * @}   Main_Motor_parameters
  */


/**
  * @}  end MC_6-STEP_LIB 
  */

/**
  * @}  end MIDDLEWARES
  */
