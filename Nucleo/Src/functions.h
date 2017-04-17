#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define DEFAULT_SPEED 25
#define LINE_LOGIC_LEVEL 3000
#define LSENSOR_COUNT 8
//#define servoMinPeriod 500
//#define servoMaxPeriod 1000
#define servoMinPeriod 100
#define servoMaxPeriod 800
//slope = (output_end - output_start) / (input_end - input_start)
#define servoSlope (servoMaxPeriod-servoMinPeriod)/(180-0)
#define BSENSOR_COUNT 2
typedef int bool;
enum bool { false, true };
struct lineData {
    GPIO_TypeDef *pName[LSENSOR_COUNT];
    uint16_t pNum[LSENSOR_COUNT];
    bool status[LSENSOR_COUNT];
};

struct buttonData {
    GPIO_TypeDef *pName[BSENSOR_COUNT];
    uint16_t pNum[BSENSOR_COUNT];
    bool status[LSENSOR_COUNT];
};
void driveForward(uint16_t speed);
void driveBack(uint16_t speed);
void driveLeft(uint16_t speed);
void driveRight(uint16_t speed);
void driveShallow(uint16_t speedL, uint16_t speedR);
void driveShallowBack(uint16_t speedL, uint16_t speedR);
/*
void initialize_ADC(ADC_HandleTypeDef hadc);
bool lineDetected(ADC_HandleTypeDef hadc, uint16_t logicLevel);
void lineFollowerCallback(ADC_HandleTypeDef hadc1, ADC_HandleTypeDef hadc2, uint16_t logicLevel);
*/
void *checked_malloc(size_t size);
void *checked_realloc(void *pointer, size_t size);
void initLineDataStruct(struct lineData *lineData);
void initLineSensor(struct lineData *lineData, 
       GPIO_TypeDef *pName1, uint16_t pNum1,
       GPIO_TypeDef *pName2, uint16_t pNum2,
       GPIO_TypeDef *pName3, uint16_t pNum3,
       GPIO_TypeDef *pName4, uint16_t pNum4,
       GPIO_TypeDef *pName5, uint16_t pNum5,
       GPIO_TypeDef *pName6, uint16_t pNum6,
       GPIO_TypeDef *pName7, uint16_t pNum7,
       GPIO_TypeDef *pName8, uint16_t pNum8
        );
int buttonOnCount(struct buttonData *buttonData);
void initButtonDataStruct(struct buttonData *buttonData);
void initButtons(struct buttonData *buttonData, 
       GPIO_TypeDef *pName1, uint16_t pNum1,
       GPIO_TypeDef *pName2, uint16_t pNum2
        );

void updateButtonData(struct buttonData *buttonData);
void updateLineData(struct lineData *lineData);
int lineOnCount(struct lineData *lineData);
void drive(int lSpeed, int rSpeed);
int leftBias(struct lineData *lineData);
int rightBias(struct lineData *lineData);
void forwardLineFollowing(struct lineData *lineData, int *lBias, int *rBias);
void turnServo(uint16_t angle);
void offloadServo();
void turnRightServo();
void backwardLineFollowing(struct lineData *lineData, int *lBias, int *rBias);
void turnRight90(struct lineData *FLineData, int *state);
void turnLeft90(struct lineData *FLineData, int *state);
void turnRingServoCC(int wait, int *state);
void turnRingServoCW(int wait, int *state);

/*
 * The Final FSM
 */
void stateOne(struct lineData *FLineData, struct buttonData *buttonData, int *state);
void stateTwo(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateThree(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateFour(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateFive(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateSix(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateSeven(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateEight(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateNine(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateTen(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateEleven(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateTwelve(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateThirteen(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateFourteen(struct lineData *FLineData, struct lineData *BLineData, int *state);
void stateFifteen(struct lineData *FLineData, struct lineData *BLineData, int *state);
#endif
