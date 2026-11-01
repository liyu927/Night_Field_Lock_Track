//
// Created by liweicong on 2026/10/27.
//
// 系统状态任务目前只处理手动方向与停车；自动、异常和低功耗状态仍待接入。

#include "app_system.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#define MANUAL_SPEED_DPS       15.0f  /* 手动按住时的目标机械角速度。 */
#define MANUAL_HOLD_TIMEOUT_MS 200U   /* 长时间收不到按住续发事件时停车。 */
#define MANUAL_UP_SIGN         1.0f   /* 实机方向相反时只需修改对应符号。 */
#define MANUAL_RIGHT_SIGN      1.0f

static AppManualDirection pending_direction = APP_MANUAL_STOP;
static uint32_t pending_tick;
static uint32_t pending_sequence;

/* UI 只投递方向，电机命令由系统任务统一下发。 */
void App_ManualControl_Send(AppManualDirection direction)
{
    if (direction > APP_MANUAL_RIGHT) return;
    taskENTER_CRITICAL();
    pending_direction = direction;
    pending_tick = osKernelGetTickCount();
    ++pending_sequence;
    taskEXIT_CRITICAL();
}

static SimpleFOC_MotorAxis manual_axis(AppManualDirection direction)
{
    return direction == APP_MANUAL_UP || direction == APP_MANUAL_DOWN
        ? SIMPLEFOC_AXIS_VERTICAL : SIMPLEFOC_AXIS_HORIZONTAL;
}

void StartSystemStateTask(void *argument)
{
    AppManualDirection active = APP_MANUAL_STOP;
    uint32_t seen_sequence = 0U;
    uint32_t last_tick = 0U;
    (void)argument;
    for (;;)
    {
        AppManualDirection direction = APP_MANUAL_STOP;
        uint32_t sequence, tick;
        taskENTER_CRITICAL();
        direction = pending_direction;
        tick = pending_tick;
        sequence = pending_sequence;
        taskEXIT_CRITICAL();

        if (sequence != seen_sequence)
        {
            seen_sequence = sequence;
            last_tick = tick;
            if (direction != active)
            {
                if (active != APP_MANUAL_STOP
                    && (direction == APP_MANUAL_STOP
                        || manual_axis(direction) != manual_axis(active)))
                    (void)MotorPosition_HoldCurrent(manual_axis(active));
                SimpleFOC_MotorAxis axis = manual_axis(direction);
                active = APP_MANUAL_STOP;
                if (direction != APP_MANUAL_STOP)
                {
                    float sign = direction == APP_MANUAL_UP ? MANUAL_UP_SIGN
                        : direction == APP_MANUAL_DOWN ? -MANUAL_UP_SIGN
                        : direction == APP_MANUAL_RIGHT ? MANUAL_RIGHT_SIGN
                        : -MANUAL_RIGHT_SIGN;
                    if (MotorSpeed_SetTargetSpeed(axis, sign * MANUAL_SPEED_DPS / 360.0f) != 0U)
                        active = direction;
                    else
                        (void)MotorPosition_HoldCurrent(axis);
                }
            }
        }
        if (active != APP_MANUAL_STOP
            && (uint32_t)(osKernelGetTickCount() - last_tick) > MANUAL_HOLD_TIMEOUT_MS)
        {
            (void)MotorPosition_HoldCurrent(manual_axis(active));
            active = APP_MANUAL_STOP;
        }
        osDelay(10U);
    }
}
