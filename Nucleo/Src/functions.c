#include <stdlib.h>
#include <unistd.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "functions.h"
#include <math.h>

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern uint16_t oldDutyCycle;
extern uint16_t newDutyCycle;
extern TIM_HandleTypeDef * motorTimer;
extern TIM_HandleTypeDef * servoTimer;
extern uint8_t servoAngle;
//extern uint8_t servoRightAngle;
extern uint8_t servoRightAngle;
extern uint8_t servoLeftAngle;
/*
 * Checks for a valid memory allocation before returning allocation
 */
void *checked_malloc(size_t size) {
    void *p = malloc(size);
    if (p == NULL){ 
        //perror("broken allocation\n");
        exit(-1);
    }
    return p;
}

/*
 * Checks for a valid memory reallocation before returning allocation
 */
void *checked_realloc(void *pointer, size_t size) {
    void *p = realloc(pointer, size);
    if (p == NULL){ 
        //perror("broken reallocation\n");
        exit(-1);
    }
    return p;
}

void stateInit(struct lineData *lineData, int *state) {
    if (*state == 0) {
    }
}

/*
 * State 1
 * Normal F_Linesense until front buttons are pressed
 * TODO: Ensure button code & button data update is complete
 */
void stateOne(struct lineData *FLineData, struct buttonData *buttonData, int *state) {
    int lBias = -15;
    int rBias = -15;
    while (*state == 1) {
        // Normal forward line following
        updateLineData(FLineData);
        updateButtonData(buttonData);
        forwardLineFollowing(FLineData, &lBias, &rBias);
        // Front buttons are pressed
        if (buttonOnCount(buttonData) >= 2) {
            *state = 2;
            break;
        }
    }
}

/*
 * TODO: Function to turn a specific servo and wait as long as necessary
 * until the turn is complete
 */
void turnRingServoCC(int wait, int *state) {
    // Turn the servo
    // Wait for a timer
    *state = 3;
}

void stateTwo(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    while (*state == 2) {
        // Turn ring and wait 2 seconds
        turnRingServoCC(2, state); // This function updates state
    }
}

void stateThree(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    int lBias = 15;
    int rBias = 15;
    while (*state == 3) {
        updateLineData(BLineData);
        backwardLineFollowing(BLineData, &lBias, &rBias);
        // 6-8 black points detected
        if (lineOnCount(BLineData) > 6) {
            *state = 4;
        }
    }
}

/*
 * Turns right 90 degree starting when the back of the line sensor hits the intersection
 */
void turnRight90(struct lineData *FLineData, int *state) {
    uint16_t timer = 0;
    uint16_t timer2 = 0;
    while (timer2 < 175) {
        timer++;
        drive(-15, -15);
        if (timer > 6500) {
            timer = 0;
            timer2++;
        }
    }
    drive(0, 0);
    updateLineData(FLineData);
    drive(25, -25);
    if (lineOnCount(FLineData) == 2 && (FLineData->status[0] == true && FLineData->status[1] == true)) {
        drive(0, 0);
        *state = 5;
    }

}

void stateFour(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    turnRight90(FLineData, state); // This function updates state
}

void stateFive(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    int lBias = -15;
    int rBias = -15;
    while (*state == 5) {
        updateLineData(FLineData);
        forwardLineFollowing(FLineData, &lBias, &rBias);
        if (lineOnCount(FLineData) == 0) {
            *state = 6;
            break;
        }
    }
}

void stateSix(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    while (*state == 6) {
        drive(20, 20);
        if (lineOnCount(FLineData) > 1 && lineOnCount(FLineData) < 3) {
            *state = 7;
            break;
        }
    }
}

void stateSeven(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    int lBias = -15;
    int rBias = -15;
    while (*state == 7) {
        updateLineData(FLineData);
        forwardLineFollowing(FLineData, &lBias, &rBias);
        if (lineOnCount(BLineData) > 6) {
            *state = 8;
            break;
        }
    }
}

/*
 * TODO: Create Special Wobbly Drive Forward Code Here
 * TODO: IR Logic
 */
void stateEight(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    int lBias = 15;
    int rBias = 15;
    while (*state == 8) {
        updateLineData(FLineData);
        forwardLineFollowing(FLineData, &lBias, &rBias);
        // if IR Data Triggered { 
        *state = 9;
        // }
    }
}

/*
 * TODO: Servo Code to turn servo CW
 * TODO: Timer delay
 */
void turnRingServoCW(int wait, int *state) {
    // Turn the servo
    // Wait for a timer
    *state = 10;
}

void stateNine(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    turnRingServoCW(5, state); // Function modifies the state
}

void stateTen(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    int lBias = 15;
    int rBias = 15;
    while (*state == 10) {
        updateLineData(BLineData);
        backwardLineFollowing(BLineData, &lBias, &rBias);
        if (lineOnCount(BLineData) < 1) {
            *state = 11;
            break;
        }
    }
}

void stateEleven(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    while (*state == 11) {
        drive(-20, -20);
        if (lineOnCount(BLineData) > 0 && lineOnCount(BLineData) < 3) {
            *state = 12;
            break;
        }
    }
}

void stateTwelve(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    int lBias = 15;
    int rBias = 15;
    while (*state == 12) {
        updateLineData(BLineData);
        backwardLineFollowing(BLineData, &lBias, &rBias);
        if (lineOnCount(BLineData) > 6) {
            *state = 13;
            break;
        }
    }
}

/*
 * Turns right 90 degree starting when the back of the line sensor hits the intersection
 */
void turnLeft90(struct lineData *FLineData, int *state) {
    uint16_t timer = 0;
    uint16_t timer2 = 0;
    while (timer2 < 175) {
        timer++;
        drive(-15, -15);
        if (timer > 6500) {
            timer = 0;
            timer2++;
        }
    }
    drive(0, 0);
    updateLineData(FLineData);
    drive(-25, 25);
    if (lineOnCount(FLineData) == 2 && (FLineData->status[0] == true && FLineData->status[1] == true)) {
        drive(0, 0);
        *state = 14;
    }
}

void stateThirteen(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    turnLeft90(FLineData, state); // This function modified state
}

void stateFourteen(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    int lBias = -15;
    int rBias = -15;
    while (*state == 14) {
        updateLineData(FLineData);
        forwardLineFollowing(FLineData, &lBias, &rBias);
        if (lineOnCount(FLineData) > 6) {
            *state = 15;
            break;
        }
    }
}

/*
 * TODO: Drive forward for a set interval to get past the full line
 * This is supposed to use an iteration of "Drive Forward"
 */
void stateFifteen(struct lineData *FLineData, struct lineData *BLineData, int *state) {
    while (*state == 15) {
        drive(20, 20);
        *state = 1;
    }
}

void forwardLineFollowing(struct lineData *lineData, int *lBias, int *rBias) {
    if (lineOnCount(lineData) > 1) {
        *lBias = leftBias(lineData);
        *rBias = rightBias(lineData);
    }
    else {
        *lBias = *lBias;
        *rBias = *rBias;
    }
    if (lineOnCount(lineData) > 6) {
        drive(32, 32);
    }
        else {
        if (*lBias < *rBias) {
            if (22-*lBias> 0 && 22-*lBias < 15) {
                drive(32+*rBias, 15);
            }
            else if (22-*lBias < 0 && 22-*lBias > -15) {
                drive(32+*rBias, -15);
            }
            else {
                drive(32+*rBias, 22-*lBias);
            }
        }
        else if (*lBias > *rBias) {
            if (22-*rBias > 0 && 22-*rBias < 15) {
                drive(15, 32+*lBias);
            }
            else if (22-*rBias < 0 && 22-*rBias > -15) {
                drive(-15, 32+*lBias);
            }
            else {
                drive(22-*rBias, 32+*lBias);
            }
        }
        else {
            drive(32, 32);
        }
    }
}

void forwardLineFollowingVariant(struct lineData *lineData, int *lBias, int *rBias, int max, int threshhold) {
    if (lineOnCount(lineData) > 1) {
        *lBias = leftBias(lineData);
        *rBias = rightBias(lineData);
    }
    else {
        *lBias = *lBias;
        *rBias = *rBias;
    }
    if (*lBias < *rBias) {
        if (threshhold-*lBias> 0 && threshhold-*lBias < 15) {
            drive(max+*rBias, 15);
        }
        else if (threshhold-*lBias < 0 && threshhold-*lBias > -15) {
            drive(max+*rBias, -15);
        }
        else {
            drive(max+*rBias, threshhold-*lBias);
        }
    }
    else if (*lBias > *rBias) {
        if (threshhold-*rBias > 0 && threshhold-*rBias < 15) {
            drive(15, max+*lBias);
        }
        else if (threshhold-*rBias < 0 && threshhold-*rBias > -15) {
            drive(-15, max+*lBias);
        }
        else {
            drive(threshhold-*rBias, max+*lBias);
        }
    }
    else {
        drive(max, max);
    }
}


void forwardLineWobble(struct lineData *lineData, int *lBias, int *rBias) {
    if (lineOnCount(lineData) > 1) {
        *lBias = leftBias(lineData);
        *rBias = rightBias(lineData);
    }
    else {
        *lBias = *lBias;
        *rBias = *rBias;
    }
    if (*lBias < *rBias) {
        if (22-*lBias> 0 && 22-*lBias < 15) {
            drive(32+*rBias, 15);
        }
        else if (22-*lBias < 0 && 22-*lBias > -15) {
            drive(32+*rBias, -15);
        }
        else {
            drive(32+*rBias, 22-*lBias);
        }
    }
    else if (*lBias > *rBias) {
        if (22-*rBias > 0 && 22-*rBias < 15) {
            drive(15, 32+*lBias);
        }
        else if (22-*rBias < 0 && 22-*rBias > -15) {
            drive(-15, 32+*lBias);
        }
        else {
            drive(22-*rBias, 32+*lBias);
        }
    }
    else {
        drive(32, 32);
    }
}

void forwardLineFollowing2(struct lineData *lineData, int *lBias, int *rBias) {
    if (lineOnCount(lineData) > 1) {
        *lBias = leftBias(lineData);
        *rBias = rightBias(lineData);
    }
    else {
        *lBias = *lBias;
        *rBias = *rBias;
    }
    if (*lBias < *rBias) {
        if (DEFAULT_THRESHHOLD-*lBias> 0 && DEFAULT_THRESHHOLD-*lBias < 15) {
            drive(DEFAULT_SPEED+*rBias, 15);
        }
        else if (DEFAULT_THRESHHOLD-*lBias < 0 && DEFAULT_THRESHHOLD-*lBias > -15) {
            drive(DEFAULT_SPEED+*rBias, -15);
        }
        else {
            drive(DEFAULT_SPEED+*rBias, DEFAULT_THRESHHOLD-*lBias);
        }
    }
    else if (*lBias > *rBias) {
        if (DEFAULT_THRESHHOLD-*rBias > 0 && DEFAULT_THRESHHOLD-*rBias < 15) {
            drive(15, DEFAULT_SPEED+*lBias);
        }
        else if (DEFAULT_THRESHHOLD-*rBias < 0 && DEFAULT_THRESHHOLD-*rBias > -15) {
            drive(-15, DEFAULT_SPEED+*lBias);
        }
        else {
            drive(DEFAULT_THRESHHOLD-*rBias, DEFAULT_SPEED+*lBias);
        }
    }
    else {
        drive(DEFAULT_SPEED, DEFAULT_SPEED);
    }
}

void forwardLineFollowingPrecise(struct lineData *lineData, int *lBias, int *rBias) {
    int lCount = lineOnCount(lineData);
    /*
    if (lCount == 2 && lineData->status[3] == true && lineData->status[4] == true) {
        drive(10, 10);
    }
    */
    /*
    else if (lCount == 3 && lineData->status[2] && lineData->status[3] && lineData->status[4]) {
        drive(12, 20);
    }
    else if (lCount == 3 && lineData->status[3] && lineData->status[4] && lineData->status[5]) {
        drive(20, 12);
    }
    */
    if (lCount > 4) {
        drive(12, 12);
    }
    else if (leftBias(lineData) > rightBias(lineData)) {
        drive(0, 12);
    }
    else if (leftBias(lineData) < rightBias(lineData)) {
        drive(12, 0);
    }
    else {
        drive(12, 12);
    }
}

void forwardLineFollowingSlow(struct lineData *lineData, int *lBias, int *rBias) {
    int lCount = lineOnCount(lineData);
    //if (lineOnCount(lineData) > 1) {
    if (lCount > 1) {
        *lBias = leftBias(lineData);
        *rBias = rightBias(lineData);
    }
    else {
        *lBias = *lBias;
        *rBias = *rBias;
    }
    if (lCount > 5) {
        drive(15, 15);
    }
    else {
        if (*lBias < *rBias) {
            if (SLOW_THRESHHOLD-*lBias> 0 && SLOW_THRESHHOLD-*lBias < 15) {
                drive(SLOW_SPEED+*rBias, 15);
            }
            else if (SLOW_THRESHHOLD-*lBias < 0 && SLOW_THRESHHOLD-*lBias > -15) {
                drive(SLOW_SPEED+*rBias, -15);
            }
            else {
                drive(SLOW_SPEED+*rBias, SLOW_THRESHHOLD-*lBias);
            }
        }
        else if (*lBias > *rBias) {
            if (SLOW_THRESHHOLD-*rBias > 0 && SLOW_THRESHHOLD-*rBias < 15) {
                drive(15, SLOW_SPEED+*lBias);
            }
            else if (SLOW_THRESHHOLD-*rBias < 0 && SLOW_THRESHHOLD-*rBias > -15) {
                drive(-15, SLOW_SPEED+*lBias);
            }
            else {
                drive(SLOW_THRESHHOLD-*rBias, SLOW_SPEED+*lBias);
            }
        }
        else {
            drive(SLOW_SPEED, SLOW_SPEED);
        }
    }
}



void backwardLineFollowing(struct lineData *lineData, int *lBias, int *rBias) {
    if (lineOnCount(lineData) > 1) {
        *lBias = leftBias(lineData);
        *rBias = rightBias(lineData);
    }
    else {
        *lBias = *lBias;
        *rBias = *rBias;
    }
    if (-42+*rBias > -10) {
        *rBias = 30;
    }
    if (-42+*lBias > -10) {
        *lBias = 30;
    }
    drive(-42+*rBias, -42+*lBias);
}

void initLineDataStruct(struct lineData *lineData) {
    for (int i = 0; i < LSENSOR_COUNT; i++) {
        lineData->pName[i] = NULL;
        lineData->status[i] = 0;
        lineData->status[i] = false;
    }
}

void initLineSensor(struct lineData *lineData, 
       GPIO_TypeDef *pName1, uint16_t pNum1,
       GPIO_TypeDef *pName2, uint16_t pNum2,
       GPIO_TypeDef *pName3, uint16_t pNum3,
       GPIO_TypeDef *pName4, uint16_t pNum4,
       GPIO_TypeDef *pName5, uint16_t pNum5,
       GPIO_TypeDef *pName6, uint16_t pNum6,
       GPIO_TypeDef *pName7, uint16_t pNum7,
       GPIO_TypeDef *pName8, uint16_t pNum8
        ) {
    initLineDataStruct(lineData);
    lineData->pName[0] = pName1;
    lineData->pName[1] = pName2;
    lineData->pName[2] = pName3;
    lineData->pName[3] = pName4;
    lineData->pName[4] = pName5;
    lineData->pName[5] = pName6;
    lineData->pName[6] = pName7;
    lineData->pName[7] = pName8;
    lineData->pNum[0] = pNum1;
    lineData->pNum[1] = pNum2;
    lineData->pNum[2] = pNum3;
    lineData->pNum[3] = pNum4;
    lineData->pNum[4] = pNum5;
    lineData->pNum[5] = pNum6;
    lineData->pNum[6] = pNum7;
    lineData->pNum[7] = pNum8;
}
void initButtonDataStruct(struct buttonData *buttonData) {
    for (int i = 0; i < BSENSOR_COUNT; i++) {
        buttonData->pName[i] = NULL;
        buttonData->status[i] = 0;
        buttonData->status[i] = false;
    }
}

void initButtons(struct buttonData *buttonData, 
       GPIO_TypeDef *pName1, uint16_t pNum1,
       GPIO_TypeDef *pName2, uint16_t pNum2
        ) {
    initButtonDataStruct(buttonData);
    buttonData->pName[0] = pName1;
    buttonData->pName[1] = pName2;
    buttonData->pNum[0] = pNum1;
    buttonData->pNum[1] = pNum2;
}

void updateLineData(struct lineData *lineData) {
    for (int i = 0; i < LSENSOR_COUNT; i++) {
        if (HAL_GPIO_ReadPin(lineData->pName[i], (uint16_t) lineData->pNum[i]) == GPIO_PIN_SET) {
            lineData->status[i] = true;
            continue;
        }
        lineData->status[i] = false;
    }
}

void updateButtonData(struct buttonData *buttonData) {
    for (int i = 0; i < BSENSOR_COUNT; i++) {
        if (HAL_GPIO_ReadPin(buttonData->pName[i], (uint16_t) buttonData->pNum[i]) == GPIO_PIN_SET) {
            buttonData->status[i] = true;
            continue;
        }
        buttonData->status[i] = false;
    }
}

int buttonOnCount(struct buttonData *buttonData) {
    int onCount = 0;
    for (int i = 0; i < BSENSOR_COUNT; i++) {
        if (buttonData->status[i] == true) {
            onCount++;
        }
    }
    return onCount;
}


int lineOnCount(struct lineData *lineData) {
    int onCount = 0;
    for (int i = 0; i < LSENSOR_COUNT; i++) {
        if (lineData->status[i] == true) {
            onCount++;
        }
    }
    return onCount;
}

int leftBias(struct lineData *lineData) {
    int bias = 0;
    int j = 32;
    //int j = 7;
    for (int i=0; i < 4; i++, j/=8) {
    //for (int i=0; i < 4; i++, j--) {
        if (lineData->status[i] == true) {
            bias+=j;
        }
    }
    return bias;
}

int rightBias(struct lineData *lineData) {
    int bias = 0;
    int j = 2;
    //int j=2;
    //for (int i = 5; i <= LSENSOR_COUNT; i++, j++) {
    //for (int i = 4; i <= LSENSOR_COUNT; i++, j+=8) {
    for (int i = 5; i <= LSENSOR_COUNT; i++, j*=8) {
        if (lineData->status[i] == true) {
            bias+=j;
        }
    }
    return bias;
}

int leftBiasWobble(struct lineData *lineData) {
    int bias = 0;
    int j = 7;
    for (int i=0; i < 5; i++, j--) {
        if (lineData->status[i] == true) {
            bias+=j;
        }
    }
    return bias;
}

int rightBiasWobble(struct lineData *lineData) {
    int bias = 0;
    int j = 4;
    for (int i = 5; i <= LSENSOR_COUNT; i++, j++) {
        if (lineData->status[i] == true) {
            bias+=j;
        }
    }
    return bias;
}


void drive(int lSpeed, int rSpeed) {
    if(lSpeed < 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        lSpeed*=-1;
    }
    else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    if (rSpeed < 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        rSpeed*=-1;
    }
    else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    }
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, lSpeed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, rSpeed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, rSpeed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, lSpeed); // Set new Pulse to Channel
}

void driveForward(uint16_t speed){
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

void driveLeft(uint16_t speed){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}

void driveShallow(uint16_t speedL, uint16_t speedR){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speedL); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speedR); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speedR); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speedL); // Set new Pulse to Channel
}

void driveShallowBack(uint16_t speedL, uint16_t speedR){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speedR); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speedL); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speedL); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speedR); // Set new Pulse to Channel
}


void driveRight(uint16_t speed){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}

void driveBack(uint16_t speed){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_1, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_2, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_3, speed); // Set new Pulse to Channel
    __HAL_TIM_SET_COMPARE(motorTimer, TIM_CHANNEL_4, speed); // Set new Pulse to Channel
}

/*
 * angle between 0 and 180
 * Servo uses 50Hz signal
 */
void turnServo(uint8_t angle){
    
    if (angle > 180) {
        angle = 180;
    }
    //slope = (output_end - output_start) / (input_end - input_start)
    // servoSlope and minimum period defined in Header file 
    //output = output_start + slope * (input - input_start)
    int val = round(servoMinPeriod + servoSlope * angle); 

    __HAL_TIM_SET_COMPARE(servoTimer, TIM_CHANNEL_1, val);
}
void offloadServo(){

      drive(0, 0);
      turnServo(0);
      //HAL_Delay(16);
      HAL_Delay(8);
      typedef int servoNum;
      enum servoNum {inc, dec, fluc};
      servoNum servoState = inc;
      int county = 0;
      int cycles = 0;
      while(servoLeftAngle < 135 || servoRightAngle > 45){
                  servoLeftAngle++;
                  servoRightAngle--;
          turnLeftServo(servoLeftAngle);
          turnRightServo(servoRightAngle);
          HAL_Delay(8);  //180 deg * 16ms/deg = 2.88 sec to complete sweep
      }
      while (1) {
          if (cycles > 1) {
              break;
          }
          if (servoState == inc) {
              if (servoAngle < 100){
                servoAngle++;
              }
              if (servoAngle == 100){
                  servoState = fluc;
              }
          }
          if (servoState == dec) {
              if (servoAngle > 0){
                  servoAngle--;
              }
              if (servoAngle == 0){
                  cycles++;
                  servoState = inc;
              }
          }
          if (servoState == fluc){
              if (county == 3) {
                  servoState = dec;
                  county = 0;
              }
             
              if (servoAngle > 40){
                  servoAngle--;
              }
              else if(servoAngle <= 40){
                  county++;
                  servoState = inc;
              }
              else{
                  servoState = dec;
              }
          }
          turnServo(servoAngle);
          HAL_Delay(8);  //180 deg * 16ms/deg = 2.88 sec to complete sweep
      }
      while (servoLeftAngle < 180 || servoRightAngle > 0){
                  servoLeftAngle++;
                  servoRightAngle--;
          turnLeftServo(servoLeftAngle);
          turnRightServo(servoRightAngle);
          HAL_Delay(8);  //180 deg * 16ms/deg = 2.88 sec to complete sweep
      }
}
void turnRightServo(uint8_t angle){
    
    if (angle > 180) {
        angle = 180;
    }
    //slope = (output_end - output_start) / (input_end - input_start)
    // servoSlope and minimum period defined in Header file 
    //output = output_start + slope * (input - input_start)
    int val = round(servoMinPeriod + servoSlope * angle); 

    __HAL_TIM_SET_COMPARE(servoTimer, TIM_CHANNEL_3, val);
}
void turnLeftServo(uint8_t angle){
    
    if (angle > 180) {
        angle = 180;
    }
    //slope = (output_end - output_start) / (input_end - input_start)
    // servoSlope and minimum period defined in Header file 
    //output = output_start + slope * (input - input_start)
    int val = round(servoMinPeriod + servoSlope * angle); 

    __HAL_TIM_SET_COMPARE(servoTimer, TIM_CHANNEL_2, val);
}

void loadServo(){
      drive(0, 0);
      turnServo(0);
      HAL_Delay(16);
      //typedef int servoNum;
      //enum servoNum {turnGrab, turnClosed,turnVert};
      //servoNum servoState = turnClosed;
      //int cycles = 0;
      /*while (1) {
          
          if (cycles > 1) {
              break;
          }
          
          if (servoState == turnGrab) {*/
              while (servoLeftAngle > 25 || servoRightAngle < 135){
                servoLeftAngle--;
                servoRightAngle++;
              }/*
              if (servoLeftAngle <= 45 && servoRightAngle >= 135){
                  //cycles++;
                  servoState = turnVert;
              }
          }
          else if (servoState == turnClosed) {
              if (servoLeftAngle < 180 && servoRightAngle > 0){
                  servoLeftAngle++;
                  servoRightAngle--;
              }
              if (servoLeftAngle >= 180 && servoRightAngle <= 0){
                  servoState = turnGrab;
              }
          }
          else if (servoState == turnVert){
              if(servoLeftAngle < 135 && servoRightAngle > 45){
                  servoLeftAngle++;
                  servoRightAngle--;
              } 
              if(servoLeftAngle >= 135 && servoRightAngle <= 45){
                  HAL_Delay(3000);
                  servoState = turnClosed;
              }
          }
          else{
             servoLeftAngle = 0;
             servoRightAngle = 180;
          }*/
          turnLeftServo(servoLeftAngle);
          turnServo(servoRightAngle);
          HAL_Delay(16);  //180 deg * 16ms/deg = 2.88 sec to complete sweep
      //}
}
/*
void turnRightServo(){
      drive(0, 0);
      turnServo(0);
      HAL_Delay(16);
      typedef int servoNum;
      enum servoNum {turnGrab, turnClosed,turnVert};
      servoNum servoState = turnClosed;
      while (1) {
          if (servoState == turnGrab) {
              if (servoRightAngle < 135){
                servoRightAngle++;
              }
              if (servoRightAngle == 135){
                  servoState = turnVert;
              }
          }
          if (servoState == turnClosed) {
              if (servoRightAngle > 0){
                  servoRightAngle--;
              }
              if (servoRightAngle == 0){
                  servoState = turnGrab;
              }
          }
          if (servoState == turnVert){
              if(servoRightAngle > 45){
                  servoRightAngle--;
              } 
              if(servoRightAngle == 45){
                  HAL_Delay(3000);
                  servoState = turnClosed;
              }
          }
          turnServo(servoRightAngle);
          HAL_Delay(16);  //180 deg * 16ms/deg = 2.88 sec to complete sweep
      } 
}
*/

void passiveTimer() {
  __HAL_TIM_SET_COMPARE(servoTimer, TIM_CHANNEL_1, 0);
}



/*
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
*/
