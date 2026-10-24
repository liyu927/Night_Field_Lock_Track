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

/* 从调用时的实际位置再转 delta_deg；新调用立即覆盖未完成的旧目标。正数顺时针，负数逆时针。 */
/* 返回 1 表示接收；未完成上电对齐、传感器故障或参数超限时返回 0。仅在任务上下文调用。 */
uint8_t MotorPosition_SetTargetDegrees(float delta_deg);

/* TIM8 上电保持禁用；每次上电准备转动前在任务上下文调用此函数。 */
/* 首次无偏移量时会锁转子对齐一次并保存双备份；以后只恢复偏移和位置原点。 */
/* 返回 HAL_OK 后才能启动 TIM8 的闭环转动；函数本身不下发转动目标。 */
HAL_StatusTypeDef SimpleFOC_Motor_PrepareTIM8(void);

/* 电机任务入口调用；应用层无需直接调用。 */
void SimpleFOC_Motor_RunPositionTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H */
