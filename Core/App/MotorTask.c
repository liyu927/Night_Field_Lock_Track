#include "SimpleFOC_Motor.h"

void StartMotorTask(void *argument)
{
    SimpleFOC_Motor_RunPositionTask(argument);
}
