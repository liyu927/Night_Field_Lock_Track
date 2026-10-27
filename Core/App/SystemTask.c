//
// Created by liweicong on 2026/10/27.
//
//系统唯一状态拥有者：维护手动/自动/异常模式枚举、异常标志位、停止电机标志位、手动命令清除、3 s 自动模式进入倒计时、15 s 低光倒计时、低功耗与唤醒；异常模式锁存至复位。

#include "app_system.h"
#include "cmsis_os2.h"

void StartSystemStateTask(void *argument)
{
    (void)argument; /* 明确该任务不使用入口参数，避免编译器产生未使用参数告警。 */


    for (;;)
    {

        osDelay(10); /* 让出 CPU，保持固定的采样与发布周期。 */
    }
}
