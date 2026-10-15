#include "pid.h"

/* 将数值限制在指定范围，避免 PID 输出越界。 */
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

void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if ((pid == 0) || (output_min >= output_max))
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    PID_Reset(pid);
}

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

    error = target - measurement;
    if (pid->initialized != 0U)
    {
        derivative = (error - pid->previous_error) / interval_s;
    }

    integral_candidate = pid->integral + error * interval_s;
    output = pid->kp * error + pid->ki * integral_candidate + pid->kd * derivative;

    /* 输出饱和且误差仍朝饱和方向增加时，不更新积分项，防止积分饱和。 */
    if (!((output > pid->output_max && error > 0.0f)
          || (output < pid->output_min && error < 0.0f)))
    {
        pid->integral = integral_candidate;
    }

    pid->previous_error = error;
    pid->initialized = 1U;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return pid_limit(output, pid->output_min, pid->output_max);
}
