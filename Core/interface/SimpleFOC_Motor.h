#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 双轴电机枚举，用于选择对应的 PWM 和使能引脚。 */
typedef enum
{
    SIMPLEFOC_AXIS_VERTICAL = 0,   /* 竖直轴，使用 TIM1。 */
    SIMPLEFOC_AXIS_HORIZONTAL      /* 水平轴，使用 TIM8。 */
} SimpleFOC_MotorAxis;

/* alpha/beta 静止坐标系中的两相电压。 */
typedef struct
{
    float alpha;
    float beta;
} SimpleFOC_AlphaBeta;

/* d/q 旋转坐标系中的两相电压。 */
typedef struct
{
    float d;
    float q;
} SimpleFOC_DirectQuadrature;

/* 启动两组电机的三相 PWM，并保持两个电机禁止状态。 */
HAL_StatusTypeDef SimpleFOC_Motor_Init(void);

/* 允许或禁止指定轴的电调输出。 */
void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis);
void SimpleFOC_Motor_Disable(SimpleFOC_MotorAxis axis);

/*
 * 以实际转子电角度输出 d/q 电压，并经逆 Park 和 SVPWM 写入三相 PWM。
 * supply_voltage 是直流母线电压，调用者必须限制 d/q 指令不超过安全范围。
 */
void SimpleFOC_Motor_SetPhaseVoltage(SimpleFOC_MotorAxis axis,
                                     float voltage_d,
                                     float voltage_q,
                                     float electrical_angle,
                                     float supply_voltage);

/* 根据 AS5600 的机械角度得到用于 FOC 的电角度。 */
float SimpleFOC_Motor_CalculateElectricalAngle(float mechanical_angle,
                                               float electrical_offset,
                                               uint8_t pole_pairs,
                                               int8_t direction);

/*
 * 转子锁定在 alignment_angle 后，根据 AS5600 实测机械角度计算电角零位偏移。
 * 该函数只计算偏移；施加锁定电压、等待和读取传感器由上层任务按安全时序执行。
 */
float SimpleFOC_Motor_CalculateElectricalOffset(float mechanical_angle,
                                                float alignment_angle,
                                                uint8_t pole_pairs,
                                                int8_t direction);

/* 使用电角度将 d/q 电压转换到 alpha/beta 电压。 */
void SimpleFOC_InvPark(const SimpleFOC_DirectQuadrature *input,
                       float electrical_angle,
                       SimpleFOC_AlphaBeta *output);

/* 将 alpha/beta 电压转换为三相 PWM 占空比。 */
void SimpleFOC_SVPWM(const SimpleFOC_AlphaBeta *input,
                     float supply_voltage,
                     float *duty_a,
                     float *duty_b,
                     float *duty_c);

#ifdef __cplusplus
}
#endif

#endif /* SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H */
