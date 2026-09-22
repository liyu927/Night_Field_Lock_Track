#include "SimpleFOC_Motor.h"
#include "cmsis_os2.h"

/*
 * 当前阶段只完成无刷电机驱动初始化，任务入口故意不执行实际控制：
 * 1. 不读取 AS5600；
 * 2. 不使能上下轴或左右轴；
 * 3. 不修改三相 PWM；
 * 4. 不调用 Clarke、Park、逆 Park 和 SVPWM。
 *
 * 保留任务入口是为了不破坏现有 FreeRTOS 任务创建结构，后续完成控制策略后
 * 再把采样、变换、控制器和占空比更新放入这里。
 */
void StartMotorTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        osDelay(1U);
    }
}
