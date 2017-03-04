#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define DEFAULT_SPEED 25
#define LINE_LOGIC_LEVEL 3000
typedef int bool;
enum bool { false, true };
void Drive_Forward(uint16_t speed);
void Drive_Back(uint16_t speed);
void Drive_Left(uint16_t speed);
void Drive_Right(uint16_t speed);
void initialize_ADC(ADC_HandleTypeDef hadc);
bool lineDetected(ADC_HandleTypeDef hadc, uint16_t logicLevel);
void lineFollowerCallback(ADC_HandleTypeDef hadc1, ADC_HandleTypeDef hadc2, uint16_t logicLevel);
#endif
