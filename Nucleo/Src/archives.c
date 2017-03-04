/*
// Steps through PWM signals in increments of 10
oldDutyCycle = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1); // Get CC Reg value from CH. 1
if (oldDutyCycle < 100){
    newDutyCycle = oldDutyCycle + 10;
} else {
    newDutyCycle = 10;
}
Drive_Forward(newDutyCycle);
HAL_Delay(3000);        // Delay 3 second
Drive_Left(newDutyCycle);
HAL_Delay(3000);        // Delay 3 second
Drive_Right(newDutyCycle);
HAL_Delay(3000);        // Delay 3 seconds
Drive_Back(newDutyCycle);
HAL_Delay(3000);        // Delay 3 seconds
*/
