#include "app_system.h"
#include "AppMotorCommand.h"
#include "SmaplingTypes.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* 手动按住时的目标机械角速度，单位度/秒。 */
#define MANUAL_SPEED_DPS          15.0f
/* 手动按住事件超过此时间未续发时改为保持当前位置。 */
#define MANUAL_HOLD_TIMEOUT_MS    200U
/* 上键和右键对应的电机正方向，实机方向不符时改符号。 */
#define MANUAL_UP_SIGN            1.0f
#define MANUAL_RIGHT_SIGN         1.0f
/* 自动模式进入后等待三秒，再允许光强环输出。 */
#define AUTO_ENTER_DELAY_MS       3000U
/* 超过此时间没有新光照帧时停止追光并保持当前位置。 */
#define AUTO_SAMPLE_TIMEOUT_MS    100U
/* 水平光强差处于此范围时锁定水平轴当前位置。 */
#define AUTO_HORIZONTAL_BAND      50
/* 竖直光强差处于此范围时锁定竖直轴当前位置。 */
#define AUTO_VERTICAL_BAND        100
/* 光强比例环增益：差值 1500 对应相对目标 150 度。 */
#define AUTO_LIGHT_KP             0.1f
/* 四路滤波 ADC 均严格高于此值时认为环境较暗，停止追光并保持位置。 */
#define AUTO_ALL_DARK_ADC         1500U
/* 单次相对角度命令的最大绝对值，单位度。 */
#define AUTO_TARGET_LIMIT_DEG     360.0f
/* 水平轴追向左侧亮光、竖直轴追向上方亮光的方向系数。 */
#define AUTO_HORIZONTAL_SIGN      (-MANUAL_RIGHT_SIGN)
#define AUTO_VERTICAL_SIGN        MANUAL_UP_SIGN

/* 光照快照的派生量由系统任务集中计算。 */
typedef struct
{
    int32_t horizontal_diff; /* 左上加左下，减右上加右下。 */
    int32_t vertical_diff;   /* 左上加右上，减左下加右下。 */
    uint16_t average;        /* 四路滤波 ADC 的平均值。 */
    uint16_t total_diff;     /* 四路滤波 ADC 的最大值减最小值。 */
} AppLightMetrics;

/* UI 发送的最新手动方向与时间戳。 */
static AppManualDirection pending_direction = APP_MANUAL_STOP;
static uint32_t pending_tick;
static uint32_t pending_sequence;
/* UI 发送的模式请求及系统任务发布的当前模式。 */
static AppMode pending_mode = APP_MODE_MANUAL;
static uint32_t pending_mode_sequence;
static AppMode current_mode = APP_MODE_MANUAL;
/* 故障请求与一次性弹窗通知，异常模式锁存到复位。 */
static uint8_t fault_requested;
static uint8_t error_popup_pending;
/* CubeMX 创建的采样队列和自动角度命令队列。 */
extern osMessageQueueId_t SmapToSystemHandle;
extern osMessageQueueId_t SmapSystemToMotorHandle;

/* UI 只投递方向，实际电机命令由系统任务发出。 */
void App_ManualControl_Send(AppManualDirection direction)
{
    if (direction > APP_MANUAL_RIGHT) return;
    taskENTER_CRITICAL();
    pending_direction = direction;
    pending_tick = osKernelGetTickCount();
    ++pending_sequence;
    taskEXIT_CRITICAL();
}

/* UI 投递模式切换请求，不直接改变当前模式。 */
void App_System_RequestMode(AppMode mode)
{
    if (mode != APP_MODE_MANUAL && mode != APP_MODE_AUTO) return;
    taskENTER_CRITICAL();
    if (fault_requested == 0U && current_mode != APP_MODE_ERROR)
    {
        pending_mode = mode;
        ++pending_mode_sequence;
    }
    taskEXIT_CRITICAL();
}

/* 故障报告只设置请求，不在其他任务中修改模式或调用 LVGL。 */
void App_System_ReportFault(void)
{
    taskENTER_CRITICAL();
    fault_requested = 1U;
    taskEXIT_CRITICAL();
}

/* 弹窗通知仅由 UI 任务领取一次，关闭弹窗不清除异常锁存。 */
uint8_t App_System_TakeErrorPopup(void)
{
    uint8_t pending;
    taskENTER_CRITICAL();
    pending = error_popup_pending;
    error_popup_pending = 0U;
    taskEXIT_CRITICAL();
    return pending;
}

/* 电机任务读取系统任务发布的当前模式。 */
uint8_t App_System_IsAutoMode(void)
{
    uint8_t enabled;
    taskENTER_CRITICAL();
    enabled = current_mode == APP_MODE_AUTO;
    taskEXIT_CRITICAL();
    return enabled;
}

/* 根据手动按钮选择竖直轴或水平轴。 */
static SimpleFOC_MotorAxis manual_axis(AppManualDirection direction)
{
    return direction == APP_MANUAL_UP || direction == APP_MANUAL_DOWN
        ? SIMPLEFOC_AXIS_VERTICAL : SIMPLEFOC_AXIS_HORIZONTAL;
}

/* 限制光强环给出的单次相对目标角度。 */
static float auto_limit_target(float target_deg)
{
    if (target_deg > AUTO_TARGET_LIMIT_DEG) return AUTO_TARGET_LIMIT_DEG;
    if (target_deg < -AUTO_TARGET_LIMIT_DEG) return -AUTO_TARGET_LIMIT_DEG;
    return target_deg;
}

/* 计算四路滤波光照的平均、极差和两个方向差。 */
static AppLightMetrics light_analyze(const SmaplingData_t *sample)
{
    /* 四路值、最大值、最小值与总和。 */
    uint16_t values[4] = {sample->left_top, sample->left_bottom,
                          sample->right_top, sample->right_bottom};
    uint16_t maximum = values[0], minimum = values[0];
    uint32_t sum = 0U;
    /* 本次采样对应的派生结果。 */
    AppLightMetrics result;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (values[i] > maximum) maximum = values[i];
        if (values[i] < minimum) minimum = values[i];
        sum += values[i];
    }
    result.horizontal_diff = (int32_t)values[0] + values[1] - values[2] - values[3];
    result.vertical_diff = (int32_t)values[0] + values[2] - values[1] - values[3];
    result.average = (uint16_t)(sum / 4U);
    result.total_diff = maximum - minimum;
    return result;
}

/* 队列满时替换最旧角度命令，使电机任务最终使用最新光照。 */
static void auto_send_target(const AppMotorCommand *command)
{
    if (SmapSystemToMotorHandle == NULL) return;
    if (osMessageQueuePut(SmapSystemToMotorHandle, command, 0U, 0U) != osOK)
    {
        /* 被覆盖的旧命令不再代表当前光照。 */
        AppMotorCommand old_command;
        (void)osMessageQueueGet(SmapSystemToMotorHandle, &old_command, NULL, 0U);
        (void)osMessageQueuePut(SmapSystemToMotorHandle, command, 0U, 0U);
    }
}

/* 系统任务独占模式与光照分析，手动模式只更新数据而不发布光强命令。 */
void StartSystemStateTask(void *argument)
{
    /* 手动命令的当前方向、事件序号和最后续发时间。 */
    AppManualDirection active = APP_MANUAL_STOP;
    uint32_t seen_sequence = 0U, last_tick = 0U;
    /* 模式事件序号、自动模式进入时间和最新采样时间。 */
    uint32_t seen_mode_sequence = 0U, auto_enter_tick = 0U, last_sample_tick = 0U;
    /* 采样超时后只锁定一次保持点。 */
    uint8_t sample_stale = 0U;
    /* 最新光照与其派生结果。 */
    SmaplingData_t sample;
    AppLightMetrics light = {0};
    (void)argument;

    for (;;)
    {
        /* 从 UI 邮箱取出本周期的模式与方向请求。 */
        AppMode requested_mode;
        AppManualDirection direction;
        uint32_t mode_sequence, sequence, tick;
        uint8_t sample_received = 0U, fault;
        taskENTER_CRITICAL();
        fault = fault_requested;
        requested_mode = pending_mode;
        mode_sequence = pending_mode_sequence;
        direction = pending_direction;
        tick = pending_tick;
        sequence = pending_sequence;
        taskEXIT_CRITICAL();

        if (fault != 0U && current_mode != APP_MODE_ERROR)
        {
            taskENTER_CRITICAL();
            current_mode = APP_MODE_ERROR;
            error_popup_pending = 1U;
            taskEXIT_CRITICAL();
            if (SmapSystemToMotorHandle != NULL)
                (void)osMessageQueueReset(SmapSystemToMotorHandle);
            MotorSpeed_Stop(SIMPLEFOC_AXIS_VERTICAL);
            MotorSpeed_Stop(SIMPLEFOC_AXIS_HORIZONTAL);
            active = APP_MANUAL_STOP;
        }

        if (current_mode != APP_MODE_ERROR && mode_sequence != seen_mode_sequence)
        {
            seen_mode_sequence = mode_sequence;
            if (requested_mode != current_mode)
            {
                taskENTER_CRITICAL();
                current_mode = requested_mode;
                taskEXIT_CRITICAL();
                if (SmapSystemToMotorHandle != NULL)
                    (void)osMessageQueueReset(SmapSystemToMotorHandle);
                (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_VERTICAL);
                (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_HORIZONTAL);
                active = APP_MANUAL_STOP;
                auto_enter_tick = osKernelGetTickCount();
                last_sample_tick = auto_enter_tick;
                sample_stale = 0U;
            }
            seen_sequence = sequence;
        }

        /* 手动模式沿用按住速度控制，自动模式完全忽略方向事件。 */
        if (current_mode == APP_MODE_MANUAL && sequence != seen_sequence)
        {
            seen_sequence = sequence;
            last_tick = tick;
            if (direction != active)
            {
                if (active != APP_MANUAL_STOP
                    && (direction == APP_MANUAL_STOP
                        || manual_axis(direction) != manual_axis(active)))
                    (void)MotorPosition_HoldCurrent(manual_axis(active));
                active = APP_MANUAL_STOP;
                if (direction != APP_MANUAL_STOP)
                {
                    /* 方向符号与手动页保持一致。 */
                    float sign = direction == APP_MANUAL_UP ? MANUAL_UP_SIGN
                        : direction == APP_MANUAL_DOWN ? -MANUAL_UP_SIGN
                        : direction == APP_MANUAL_RIGHT ? MANUAL_RIGHT_SIGN
                        : -MANUAL_RIGHT_SIGN;
                    /* 本次按钮要控制的物理轴。 */
                    SimpleFOC_MotorAxis axis = manual_axis(direction);
                    if (MotorSpeed_SetTargetSpeed(axis, sign * MANUAL_SPEED_DPS / 360.0f) != 0U)
                        active = direction;
                    else
                        (void)MotorPosition_HoldCurrent(axis);
                }
            }
        }
        if (current_mode == APP_MODE_MANUAL && active != APP_MANUAL_STOP
            && (uint32_t)(osKernelGetTickCount() - last_tick) > MANUAL_HOLD_TIMEOUT_MS)
        {
            (void)MotorPosition_HoldCurrent(manual_axis(active));
            active = APP_MANUAL_STOP;
        }
        if (current_mode == APP_MODE_AUTO) seen_sequence = sequence;

        /* 排空采样队列，只分析本周期最新的一帧。 */
        if (SmapToSystemHandle != NULL)
        {
            while (osMessageQueueGet(SmapToSystemHandle, &sample, NULL, 0U) == osOK)
                sample_received = 1U;
        }
        if (sample_received != 0U)
        {
            light = light_analyze(&sample);
            last_sample_tick = osKernelGetTickCount();
            sample_stale = 0U;
        }

        if (current_mode == APP_MODE_AUTO && sample_stale == 0U
            && (uint32_t)(osKernelGetTickCount() - last_sample_tick) > AUTO_SAMPLE_TIMEOUT_MS)
        {
            if (SmapSystemToMotorHandle != NULL)
                (void)osMessageQueueReset(SmapSystemToMotorHandle);
            (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_VERTICAL);
            (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_HORIZONTAL);
            sample_stale = 1U;
        }

        if (current_mode == APP_MODE_AUTO && sample_received != 0U
            && (uint32_t)(osKernelGetTickCount() - auto_enter_tick) >= AUTO_ENTER_DELAY_MS)
        {
            /* 两轴都发送最新目标；电机任务只在首次进入零区间时锁定位置。 */
            AppMotorCommand command = {0};
            /* 四路都偏暗时保留双轴零度目标；否则沿用方向差比例控制。 */
            if (!(sample.left_top > AUTO_ALL_DARK_ADC
                && sample.left_bottom > AUTO_ALL_DARK_ADC
                && sample.right_top > AUTO_ALL_DARK_ADC
                && sample.right_bottom > AUTO_ALL_DARK_ADC))
            {
                if (light.horizontal_diff < -AUTO_HORIZONTAL_BAND
                    || light.horizontal_diff > AUTO_HORIZONTAL_BAND)
                    command.horizontal_deg = auto_limit_target(
                        AUTO_HORIZONTAL_SIGN * AUTO_LIGHT_KP * light.horizontal_diff);
                if (light.vertical_diff < -AUTO_VERTICAL_BAND
                    || light.vertical_diff > AUTO_VERTICAL_BAND)
                    command.vertical_deg = auto_limit_target(
                        AUTO_VERTICAL_SIGN * AUTO_LIGHT_KP * light.vertical_diff);
            }
            auto_send_target(&command);
        }
        osDelay(10U);
    }
}
