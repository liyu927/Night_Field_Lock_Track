#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 电机物理轴编号；底层据此选择定时器、三路 PWM 通道和 EN 引脚。 */
typedef enum
{
    SIMPLEFOC_AXIS_VERTICAL = 0,   /* 竖直轴：TIM1 三相 PWM、PE7 使能、I2C1 编码器。 */
    SIMPLEFOC_AXIS_HORIZONTAL      /* 水平轴：TIM8 三相 PWM、PE8 使能、I2C3 编码器。 */
} SimpleFOC_MotorAxis;

/* TIM1 控制模式；任务一次只运行其中一个控制路径。 */
typedef enum
{
    SIMPLEFOC_CONTROL_NONE = 0,
    SIMPLEFOC_CONTROL_POSITION,
    SIMPLEFOC_CONTROL_SPEED
} SimpleFOC_ControlMode;

/* 逆 Park 变换后的静止坐标系电压；两分量合成定子电压矢量。 */
typedef struct
{
    float alpha;   /* α 轴电压，单位 V。 */
    float beta;    /* β 轴电压，单位 V。 */
} SimpleFOC_AlphaBeta;

/* 随转子电角度旋转的坐标系电压；本工程闭环运行时 d=0、q 为速度环输出。 */
typedef struct
{
    float d;       /* d 轴电压，单位 V；校准时用于固定磁场方向。 */
    float q;       /* q 轴电压，单位 V；控制转矩方向与强度，但不是电流值。 */
} SimpleFOC_DirectQuadrature;

/* 启动两轴各三路 PWM，写入 50% 中性占空比并保持 EN 关闭；失败返回 HAL_ERROR。 */
HAL_StatusTypeDef SimpleFOC_Motor_Init(void);

/* 将指定轴 EN 置高；调用前应先写好安全的三相占空比。 */
void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis);
/* 将指定轴 EN 置低并把三路比较值恢复为中性占空比。 */
void SimpleFOC_Motor_Disable(SimpleFOC_MotorAxis axis);

/*
 * 用 electrical_angle 将 d/q 电压逆 Park 变换，再经 SVPWM 写入指定轴三相比较值。
 * voltage_d/voltage_q/supply_voltage 的单位均为 V；supply_voltage <= 0 时关闭该轴。
 * 本函数只更新 PWM，不负责使能；调用者必须按实际供电和硬件能力限制电压指令。
 */
void SimpleFOC_Motor_SetPhaseVoltage(SimpleFOC_MotorAxis axis,
                                     float voltage_d,
                                     float voltage_q,
                                     float electrical_angle,
                                     float supply_voltage);

/* 电角度 = direction × pole_pairs × 机械角度 + electrical_offset，结果归一化到 [0, 2π)。 */
/* mechanical_angle/electrical_offset 均为弧度；direction 只能取 +1 或 -1。 */
float SimpleFOC_Motor_CalculateElectricalAngle(float mechanical_angle,
                                               float electrical_offset,
                                               uint8_t pole_pairs,
                                               int8_t direction);

/*
 * 由固定磁场的电角度 alignment_angle 和锁定后的机械角度反推电角零位偏移。
 * 四个角度参数单位均为弧度；该函数只计算，不会使能 PWM 或读取 AS5600。
 */
float SimpleFOC_Motor_CalculateElectricalOffset(float mechanical_angle,
                                                float alignment_angle,
                                                uint8_t pole_pairs,
                                                int8_t direction);

/* 逆 Park：把随转子旋转的 d/q 电压投影到定子静止的 α/β 轴，角度单位为弧度。 */
void SimpleFOC_InvPark(const SimpleFOC_DirectQuadrature *input,
                       float electrical_angle,
                       SimpleFOC_AlphaBeta *output);

/* SVPWM：由 α/β 电压求三相电压，注入公共偏置后得到 [0,1] 占空比。 */
/* supply_voltage 为直流母线电压；三个 duty 输出分别对应 A/B/C 相。 */
void SimpleFOC_SVPWM(const SimpleFOC_AlphaBeta *input,
                     float supply_voltage,
                     float *duty_a,
                     float *duty_b,
                     float *duty_c);

/* TIM1 相对位置命令：每次调用以最近反馈位置为零点，再转 delta_deg 度。 */
/* 新命令立即覆盖旧目标，不重置编码器角度或速度反馈。 */
/* 未就绪、故障、非有限数或超过 MOTOR_MAX_COMMAND_DEG 时返回 0；接收后返回 1。 */
/* 内部使用 FreeRTOS 临界区，仅可从任务上下文调用；接收命令并不表示已经到位。 */
uint8_t MotorPosition_SetTargetDegrees(float delta_deg);

/* TIM1 速度命令：speed_rps 单位圈/秒，正负号控制方向，0 为零速但不关闭 EN。 */
/* 只运行速度环，绝对值超过 MOTOR_MAX_SPEED_DPS/360、未就绪或故障时返回 0。 */
uint8_t MotorSpeed_SetTargetSpeed(float speed_rps);

/* TIM8 上电保持禁用；准备使用该轴时在任务上下文显式调用。 */
/* 已有偏移时只读取当前位置建立原点；无偏移时会锁转子校准并保存双备份。 */
/* 校准会让电机实际移动；HAL_OK 表示准备成功，本函数不会下发位置目标。 */
HAL_StatusTypeDef SimpleFOC_Motor_PrepareTIM8(void);

/* 长期运行入口：完成 TIM1 校准，并以 initial_mode 和 initial_target 启动对应控制环。 */
/* 位置模式的目标单位度，速度模式的目标单位圈/秒；任务上下文调用后不返回。 */
void SimpleFOC_Motor_RunTask(SimpleFOC_ControlMode initial_mode, float initial_target);

#ifdef __cplusplus
}
#endif

#endif /* SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H */
