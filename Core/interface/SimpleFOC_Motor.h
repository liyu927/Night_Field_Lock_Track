#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 两个云台轴的基础闭环驱动接口：
 * TIM1 + I2C1 + AS5600 对应上下轴；
 * TIM8 + I2C3 + AS5600 对应左右轴。
 */
void SimpleFOC_Motor_Init(void);
void SimpleFOC_Motor_Update(void);
void SimpleFOC_Motor_Task(void *argument);

/*
 * 目标角度单位为弧度，使用 AS5600 上电后建立的连续机械角坐标。
 * vertical_angle 对应上下轴，horizontal_angle 对应左右轴。
 */
void SimpleFOC_Motor_SetTargetAngles(float vertical_angle, float horizontal_angle);

#ifdef __cplusplus
}
#endif

#endif /* SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H */
