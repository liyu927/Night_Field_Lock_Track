#include "SimpleFOC_Motor.h"

/*
 * 电机任务只负责按固定周期更新两个轴的 AS5600 角度和位置闭环。
 * 具体的 PWM、校准和故障保护逻辑集中在 SimpleFOC_Motor.c。
 */
void StartMotorTask(void *argument)
{
    SimpleFOC_Motor_Task(argument);
}
