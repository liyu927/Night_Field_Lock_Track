#include "app_system.h"

#define MOTOR_START_SPEED_RPS 0.5f /* 上电默认正向目标速度，单位圈/秒。 */

/* 电机任务只调用应用层入口；需要位置测试时切换下面两行。 */
void StartMotorTask(void *argument)
{
    (void)argument;
    App_MotorSpeed_Run(MOTOR_START_SPEED_RPS);
    // App_MotorPosition_Run(360.0f); /* 位置测试：注释上一行，再启用本行。 */
}
