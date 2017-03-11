#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define DEFAULT_SPEED 25
#define LINE_LOGIC_LEVEL 3000
#define LSENSOR_COUNT 8
typedef int bool;
enum bool { false, true };
struct lineData {
    GPIO_TypeDef *pName[LSENSOR_COUNT];
    uint16_t pNum[LSENSOR_COUNT];
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
void updateLineData(struct lineData *lineData);
int lineOnCount(struct lineData *lineData);
#endif
