#ifndef PID_H
#define PID_H

#include <stdint.h>

/* PID 控制器运行状态，输入、输出和积分项均使用浮点数。 */
typedef struct
{
    float kp;                /* 比例系数。 */
    float ki;                /* 积分系数。 */
    float kd;                /* 微分系数。 */
    float integral;          /* 已累计的积分项。 */
    float previous_error;    /* 上一周期误差，用于计算微分项。 */
    float output_min;        /* 输出下限。 */
    float output_max;        /* 输出上限。 */
    uint8_t initialized;     /* 首次更新标记，防止微分突变。 */
} PID_Controller_t;

/* 初始化 PID 参数和输出限幅。 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

/* 清空积分项和历史误差，保留 PID 参数。 */
void PID_Reset(PID_Controller_t *pid);

/* 根据目标值、测量值和控制周期计算限幅后的 PID 输出。 */
float PID_Update(PID_Controller_t *pid,
                 float target,
                 float measurement,
                 float interval_s);

#endif /* PID_H */
