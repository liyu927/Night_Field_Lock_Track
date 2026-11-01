#include "app_system.h"

/* 上电只准备控制任务，不下发运动目标。 */
void StartMotorTask(void *argument)
{
    (void)argument;
    SimpleFOC_Motor_RunTask(SIMPLEFOC_CONTROL_NONE, 0.0f);
}
