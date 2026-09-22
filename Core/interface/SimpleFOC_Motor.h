#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 双轴电机枚举仅用于选择对应的使能引脚。 */
typedef enum
{
    SIMPLEFOC_AXIS_VERTICAL = 0,
    SIMPLEFOC_AXIS_HORIZONTAL
} SimpleFOC_MotorAxis;

/* Clarke 变换后的两相静止坐标。 */
typedef struct
{
    float alpha;
    float beta;
} SimpleFOC_AlphaBeta;

/* Park 变换后的两相旋转坐标。 */
typedef struct
{
    float d;
    float q;
} SimpleFOC_DirectQuadrature;

/*
 * 启动 TIM1 和 TIM8 的六路 PWM，并将占空比设置为中性值。
 * 初始化结束后两个电机仍保持关闭，只有显式调用 Enable 才会使能。
 */
HAL_StatusTypeDef SimpleFOC_Motor_Init(void);

/* 按轴控制低电平有效的电机使能引脚。 */
void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis);
void SimpleFOC_Motor_Disable(SimpleFOC_MotorAxis axis);

/* 将三相量转换到 alpha/beta 静止坐标系。 */
void SimpleFOC_Clarke(float current_a,
                      float current_b,
                      float current_c,
                      SimpleFOC_AlphaBeta *output);

/* 使用电角度（弧度）将 alpha/beta 转换到 d/q 旋转坐标系。 */
void SimpleFOC_Park(const SimpleFOC_AlphaBeta *input,
                    float electrical_angle,
                    SimpleFOC_DirectQuadrature *output);

/* 使用电角度（弧度）将 d/q 转换回 alpha/beta 静止坐标系。 */
void SimpleFOC_InvPark(const SimpleFOC_DirectQuadrature *input,
                       float electrical_angle,
                       SimpleFOC_AlphaBeta *output);

/* 将 alpha/beta 电压参考转换为限制在 0 到 1 之间的三相占空比。 */
void SimpleFOC_SVPWM(const SimpleFOC_AlphaBeta *input,
                     float supply_voltage,
                     float *duty_a,
                     float *duty_b,
                     float *duty_c);

#ifdef __cplusplus
}
#endif

#endif /* SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H */
