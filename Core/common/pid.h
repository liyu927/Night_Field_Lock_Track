#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 离散 PID 参数与历史状态；同一实例只能由其所属控制环按周期更新。 */
typedef struct
{
    float kp;                /* 比例增益：直接作用于当前误差。 */
    float ki;                /* 积分增益：作用于按秒累计的误差。 */
    float kd;                /* 微分增益：作用于误差变化率。 */
    float integral;          /* 已累计的误差积分，尚未乘 ki。 */
    float previous_error;    /* 上一周期误差，用于差分求微分项。 */
    float output_min;        /* 输出下限；位置环为 rad/s，速度环为 V。 */
    float output_max;        /* 输出上限；必须大于 output_min。 */
    uint8_t initialized;     /* 首次更新不计算微分，避免初值造成尖峰。 */
} PID_Controller_t;

/* 设置三项增益和输出范围，再清空积分与历史误差；参数不合法时直接返回。 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

/* 清空积分、上一误差与首次更新标记，保留增益和输出限幅。 */
void PID_Reset(PID_Controller_t *pid);

/* error=target-measurement；interval_s 为本次实际控制周期，单位秒。 */
/* 先试算积分，若输出已饱和且误差继续推向饱和，则冻结积分；最后限制输出。 */
/* 返回值的单位由所属控制环决定；空指针或非正周期返回 0。 */
float PID_Update(PID_Controller_t *pid,
                 float target,
                 float measurement,
                 float interval_s);

#endif /* PID_H */
