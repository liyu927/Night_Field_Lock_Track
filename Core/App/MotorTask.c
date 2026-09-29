#include "SimpleFOC_Motor.h"
#include "cmsis_os2.h"

/*
 * TIM8 对应的水平轴开环参数。
 *
 * 这里使用的是“虚拟电角度 + SVPWM”的无位置传感器开环驱动：
 * 不读取 AS5600，不读取 ADC 电流，也没有速度/位置反馈闭环。
 * 15 rpm 对于调试来说是相对低的初始转速，实机确认安全后再逐步提高。
 *
 * 注意：软件中的正方向由相序决定，不保证在所有电机接线和观察方向下都叫
 * “顺时针”。如果实机观察到逆时针，把下面的方向宏从 1 改成 -1。
 */
#define TIM8_OPEN_LOOP_SPEED_RPS       0.25f
#define TIM8_OPEN_LOOP_VOLTAGE         2.0f
#define TIM8_OPEN_LOOP_POLE_PAIRS      7U
#define TIM8_OPEN_LOOP_DIRECTION       1
#define TIM8_OPEN_LOOP_RAMP_TIME_MS    1500U
/* 开环电角度偏移量，单位为度；先以 0 度作为实机调试基准。 */
#define TIM8_OPEN_LOOP_ANGLE_OFFSET_DEG 0.0f
#define MOTOR_TASK_PERIOD_MS           1U

void StartMotorTask(void *argument)
{
    const SimpleFOC_OpenLoopConfig tim8_open_loop_config = {
        TIM8_OPEN_LOOP_SPEED_RPS,
        TIM8_OPEN_LOOP_VOLTAGE,
        TIM8_OPEN_LOOP_POLE_PAIRS,
        TIM8_OPEN_LOOP_DIRECTION,
        TIM8_OPEN_LOOP_RAMP_TIME_MS,
        TIM8_OPEN_LOOP_ANGLE_OFFSET_DEG
    };
    uint32_t next_wake_tick;

    (void)argument;

    /*
     * main() 已经完成 PWM 外设初始化和中性占空比设置。
     * 这里仅启动 TIM8 对应的水平轴，TIM1 对应的竖直轴继续保持关闭。
     */
    if (SimpleFOC_Motor_OpenLoopStart(SIMPLEFOC_AXIS_HORIZONTAL,
                                      &tim8_open_loop_config) != HAL_OK)
    {
        /* 参数错误时保持电机关闭，并周期性等待，避免任务空转占满 CPU。 */
        SimpleFOC_Motor_OpenLoopStop(SIMPLEFOC_AXIS_HORIZONTAL);
        for (;;)
        {
            osDelay(100U);
        }
    }

    /*
     * 使用绝对唤醒时间，让电机更新周期尽量稳定在 1 ms。
     * 开环角度由 SimpleFOC_Motor_OpenLoopUpdate() 根据实际 HAL 时间推进。
     */
    next_wake_tick = osKernelGetTickCount();
    for (;;)
    {
        SimpleFOC_Motor_OpenLoopUpdate(SIMPLEFOC_AXIS_HORIZONTAL);
        next_wake_tick += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);
    }
}
