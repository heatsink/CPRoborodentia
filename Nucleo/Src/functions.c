#include "main.h"
#include "stm32f4xx_hal.h"
#include "functions.h"

//extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct lineStatus {
    //int16_t line1[] = int16_t[7];
};
extern uint16_t oldDutyCycle;
extern uint16_t newDutyCycle;
extern TIM_HandleTypeDef * motorTimer;

void Drive_Forward(uint16_t speed){
//---Example for using Timer 13---------------------------
    // oldDutyCycle = __HAL_TIM_GET_AUTORELOAD(&htim13);         // Gets the Period set for PWm
    // oldDutyCycle = __HAL_TIM_GET_COMPARE(&htim13, TIM_CHANNEL_1); // Get reading for Timer 13
    // oldDutyCycle = TIM13 -> CCR1;                             // Set Capture Compare register directly

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);         // DIR Pin 1 for Motor Driver
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);         // DIR Pin 2 for Motor Driver

    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}

void Drive_Left(uint16_t speed){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}

void Drive_Right(uint16_t speed){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}

void Drive_Back(uint16_t speed){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}
void initialize_ADC(ADC_HandleTypeDef hadc) {
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 100);
}

bool lineDetected(ADC_HandleTypeDef hadc, uint16_t logicLevel) {
    initialize_ADC(hadc);
    //if (HAL_ADC_GetValue(&hadc) > logicLevel) {
    if (HAL_ADC_GetValue(&hadc) < logicLevel) {
        return true;
    }
    return false;
}

void lineFollowerCallback(ADC_HandleTypeDef hadc1, ADC_HandleTypeDef hadc2, uint16_t logicLevel) {
    if (lineDetected(hadc1, LINE_LOGIC_LEVEL) && lineDetected(hadc2, LINE_LOGIC_LEVEL)) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
        Drive_Forward(DEFAULT_SPEED);
    }
    else if (lineDetected(hadc1, LINE_LOGIC_LEVEL)) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
        Drive_Left(DEFAULT_SPEED);
    }
    else if (lineDetected(hadc2, LINE_LOGIC_LEVEL)) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED On
        Drive_Right(DEFAULT_SPEED);
    }
    else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED Off
        Drive_Forward(0);
    }

}