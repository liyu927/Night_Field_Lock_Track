#include "app_system.h"

#define MOTOR_START_ANGLE_DEG 90.0f /* 上电后两轴各顺时针转动的相对角度。 */

/* 电机任务上电后向两轴下发相同的相对位置目标。 */
void StartMotorTask(void *argument)
{
    (void)argument;
    App_MotorPosition_Run(MOTOR_START_ANGLE_DEG);
}
