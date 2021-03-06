/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "functions.h"
#include "config.h"
#include <stdlib.h>
typedef int lineState;
enum lineState { hardL, shallowL, cont, shallowR, hardR };

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t oldDutyCycle;
uint16_t newDutyCycle;
volatile TIM_HandleTypeDef * motorTimer = &htim4;
volatile TIM_HandleTypeDef * servoTimer = &htim5;
uint16_t INIT_STATE = 1;
uint16_t LINE_SENSE_F = 1;
lineState lState = cont;
uint8_t servoAngle = 0;
uint8_t servoLeftAngle = 0;
uint8_t servoRightAngle = 0;
uint8_t DEBUG_MODE = 1;
uint16_t increment = 0;
uint16_t timer = 0;
uint16_t timer2 = 0;
int lBias = -15;
int rBias = -15;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//MX_ADC_1_Init();

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  struct lineData *lineData = checked_malloc(sizeof(struct lineData));
  initLineSensor(lineData, LS1, LS1NUM,
                           LS2, LS2NUM,
                           LS3, LS3NUM,
                           LS4, LS4NUM,
                           LS5, LS5NUM,
                           LS6, LS6NUM,
                           LS7, LS7NUM,
                           LS8, LS8NUM);

  // Start Timers after initialization
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  passiveTimer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  while (DEBUG_MODE == 1) {
      passiveTimer();
      int lBias = -15;
      int rBias = -15;
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
          INIT_STATE = 0;
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED Off
          drive(0,0);
          //turnServo(0);
          continue;
      }
      while (INIT_STATE == 0) {
          updateLineData(lineData);
          //forwardLineFollowingPrecise(lineData, &lBias, &rBias);
          forwardLineFollowing(lineData, &lBias, &rBias);
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // LED On
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // LED On
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
              INIT_STATE = 1;
              drive(0, 0);
              //turnServo(0);
              break;
          }
          else {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED Off
          }

      }
      while (INIT_STATE == 1) {
          HAL_Delay(1500);
          //loadServo();
          INIT_STATE = 2;
          //turnServo(0);
          break;
      }
      while (INIT_STATE == 2) {
          drive(-100, -100);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) > 6) {
              INIT_STATE = 3;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 3) {
          drive(-100, -100);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) < 4) {
              INIT_STATE = 4;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 4) {
          drive(-100, -100);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) > 6) {
              INIT_STATE = 5;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 5) {
          timer++;
          drive(80, 80);
          //turnServo(0);
          if (timer > 6500) {
              timer = 0;
              timer2++;
          }
          if (timer2 > 75) { // 125 275
              timer = 0;
              timer2 = 0;
              INIT_STATE = 6;
              drive(0, 0);
          }
      }
      while (INIT_STATE == 6) {
          drive(-40, 40);
          //turnServo(0);
          updateLineData(lineData);
          if (lineOnCount(lineData) <= 1) {
              INIT_STATE = 7;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 7) {
          drive(-40, 40);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) == 2 && lineData->status[2] == true && lineData->status[3] == true) {
              INIT_STATE = 8;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 8) {
          updateLineData(lineData);
          forwardLineFollowing2(lineData, &lBias, &rBias);
          //turnServo(0);
          if (lineOnCount(lineData) > 6) {
              INIT_STATE = 9;
          }
      }
      while (INIT_STATE == 9) {
          updateLineData(lineData);
          //turnServo(0);
          //forwardLineWobble(lineData, &lBias, &rBias);
          forwardLineFollowing2(lineData, &lBias, &rBias);
          //drive(20, 20);
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // LED On
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // LED On
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
              INIT_STATE = 10;
              drive(0, 0);
              HAL_Delay(50);
          }
          else {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED Off
          }
      }
      while (INIT_STATE == 10) {
          offloadServo();
          INIT_STATE = 11;
      }
      while (INIT_STATE == 11) {
          drive(-100, -100);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) > 6) {
              INIT_STATE = 12;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 12) {
          drive(-100, -100);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) < 4) {
              INIT_STATE = 13;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 13) {
          drive(-100, -100);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) > 6) {
              INIT_STATE = 14;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 14) {
          timer++;
          drive(80, 80);
          //turnServo(0);
          if (timer > 6500) {
              timer = 0;
              timer2++;
          }
          if (timer2 > 75) { // 125 275
              timer = 0;
              timer2 = 0;
              INIT_STATE = 15;
              drive(0, 0);
          }
      }
      while (INIT_STATE == 15) {
          drive(40, -40);
          //turnServo(0);
          updateLineData(lineData);
          if (lineOnCount(lineData) <= 1) {
              INIT_STATE = 1;
              drive(0, 0);
              break;
          }
      }
      while (INIT_STATE == 16) {
          drive(40, -40);
          updateLineData(lineData);
          //turnServo(0);
          if (lineOnCount(lineData) == 2 && lineData->status[5] == true && lineData->status[6] == true) {
              INIT_STATE = 0;
              drive(0, 0);
              break;
          }
      }
  }
  while (DEBUG_MODE == 2) {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
          INIT_STATE = 0;
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED Off
          drive(0,0);
          continue;
      }
      while (INIT_STATE == 0) {
          updateLineData(lineData);
          forwardLineFollowing2(lineData, &lBias, &rBias);
          if (lineOnCount(lineData) > 6) {
              INIT_STATE = 1;
              drive(0, 0);
              HAL_Delay(500);
          }
      }
      while (INIT_STATE == 1) {
          updateLineData(lineData);
          forwardLineWobble(lineData, &lBias, &rBias);
          forwardLineFollowing(lineData, &lBias, &rBias);
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // LED On
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // LED On
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
              INIT_STATE = 2;
              drive(0, 0);
              HAL_Delay(100);
              break;
          }
          else {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED Off
          }
      }
      while (INIT_STATE == 2) {
          //drive(0, 0);
          offloadServo();
          //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
          drive(0, 0);
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // LED On
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // LED On
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
          }
          else {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED Off
          }
          HAL_Delay(50);
      }
      /*
        timer++;
        if (increment < 85) {
            drive(15+increment, 15+increment);
        }
        else {
            drive(0, 0);
        }
        if (timer > 6500) {
            timer = 0;
            //timer2++;
            increment++;
        }
        */
        /*
        if (timer2 > 175) {
            timer = 0;
            timer2 = 0;
        }
        */
      /*
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED Off
      }
      else {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED Off
      }
      */
  }
  /*
  while (DEBUG_MODE == 3) {
      drive(0, 0);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // LED On
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // LED On
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
      }
      else {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED Off
      }
    HAL_Delay(50);

>>>>>>> .merge_file_0t3nie
  }
  */
  }
  return 0;
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 180;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 360;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_1_LEFT_Pin|DIR_2_RIGHT_Pin|Front_Left_BTNC11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Front_Right_BTND2_GPIO_Port, Front_Right_BTND2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LS2_CH6_Pin LS_2_CH5_Pin LS2_CH1_Pin LS2_CH2_Pin 
                           LS1_CH7_Pin */
  GPIO_InitStruct.Pin = LS2_CH6_Pin|LS_2_CH5_Pin|LS2_CH1_Pin|LS2_CH2_Pin 
                          |LS1_CH7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LS2_CH4_Pin LS_1_CH6_Pin LS1_CH8_Pin LS1_CH1_Pin 
                           LS2_CH8_Pin LS2_CH7_Pin */
  GPIO_InitStruct.Pin = LS2_CH4_Pin|LS_1_CH6_Pin|LS1_CH8_Pin|LS1_CH1_Pin 
                          |LS2_CH8_Pin|LS2_CH7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LS2_CH3_Pin LS1_CH5_Pin LS1_CH2_Pin LS1_CH4_Pin 
                           LS1_CH3_Pin */
  GPIO_InitStruct.Pin = LS2_CH3_Pin|LS1_CH5_Pin|LS1_CH2_Pin|LS1_CH4_Pin 
                          |LS1_CH3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_1_LEFT_Pin DIR_2_RIGHT_Pin Front_Left_BTNC11_Pin */
  GPIO_InitStruct.Pin = DIR_1_LEFT_Pin|DIR_2_RIGHT_Pin|Front_Left_BTNC11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Front_Left_BTN_Pin Front_Right_BTN_Pin */
  GPIO_InitStruct.Pin = Front_Left_BTN_Pin|Front_Right_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Front_Right_BTND2_Pin */
  GPIO_InitStruct.Pin = Front_Right_BTND2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Front_Right_BTND2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//---------------Robot Private Functions-----------------------------------------------

/*void Drive_All_Motors(uint16_t speed){
//---Example for using Timer 13---------------------------
    // oldDutyCycle = __HAL_TIM_GET_AUTORELOAD(&htim13);         // Gets the Period set for PWm
    // oldDutyCycle = __HAL_TIM_GET_COMPARE(&htim13, TIM_CHANNEL_1); // Get reading for Timer 13
    // oldDutyCycle = TIM13 -> CCR1;                             // Set Capture Compare register directly

    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
