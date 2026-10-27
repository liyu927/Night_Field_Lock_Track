#include "pid.h"

/* 把控制输出限制在配置区间，防止目标速度或 q 轴电压越界。 */
static float pid_limit(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/* 初始化增益和限幅，并让首次 PID_Update 从无历史状态开始。 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    /* 上下限非法时不修改原控制器，避免后续裁剪方向颠倒。 */
    if ((pid == 0) || (output_min >= output_max))
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    /* 新参数启用前清除旧积分和微分历史。 */
    PID_Reset(pid);
}

/* 只清运行历史，增益和输出范围保持不变。 */
void PID_Reset(PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->initialized = 0U;
}

/* 一次离散 PID 更新：试算积分、判断积分饱和、重算并裁剪输出。 */
float PID_Update(PID_Controller_t *pid,
                 float target,
                 float measurement,
                 float interval_s)
{
    float error;
    float derivative = 0.0f;
    float integral_candidate;
    float output;

    if ((pid == 0) || (interval_s <= 0.0f))
    {
        return 0.0f;
    }

    /* 正误差表示测量值低于目标；输入单位由位置环或速度环决定。 */
    error = target - measurement;
    /* 首次更新没有上一误差，故不计算微分，避免启动瞬间突变。 */
    if (pid->initialized != 0U)
    {
        derivative = (error - pid->previous_error) / interval_s;
    }

    /* 先试算本次积分和输出，再决定是否接受积分，避免越限时持续累积。 */
    integral_candidate = pid->integral + error * interval_s;
    output = pid->kp * error + pid->ki * integral_candidate + pid->kd * derivative;

    /* 输出饱和且误差仍朝饱和方向增加时，不更新积分项，防止积分饱和。 */
    /* 高限且正误差、或低限且负误差时，本次误差只会让输出进一步饱和。 */
    if (!((output > pid->output_max && error > 0.0f)
          || (output < pid->output_min && error < 0.0f)))
    {
        pid->integral = integral_candidate;
    }

    /* 无论积分是否冻结，都保存当前误差供下一周期计算误差变化率。 */
    pid->previous_error = error;
    pid->initialized = 1U;
    /* 用最终接受的积分重算输出，最后再按配置范围裁剪。 */
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return pid_limit(output, pid->output_min, pid->output_max);
}
