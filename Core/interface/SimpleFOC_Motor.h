#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_SIMPLEFOC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 双轴电机枚举仅用于选择对应的使能引脚。 */
typedef enum
{
    SIMPLEFOC_AXIS_VERTICAL = 0,//竖直轴，使用TIM1
    SIMPLEFOC_AXIS_HORIZONTAL   //水平轴，使用TIM8
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
 * 无位置传感器开环运行参数。
 *
 * 这里的“开环”不是读取 AS5600 后再纠正角度，而是由软件自己按照
 * 目标机械转速推进电角度，再使用这个电角度产生三相 PWM。
 */
typedef struct
{
    /* 目标机械转速，单位为转/秒，必须填写非负数。 */
    float mechanical_speed_rps;
    /* q 轴电压幅值，单位为伏特，决定电机产生的平均转矩。d轴固定为 0V */
    float voltage;
    /* 电机极对数，例如 14 极电机通常填写 7。 */
    uint8_t pole_pairs;
    /*
     * 电角度推进方向：
     * 1 表示按当前 U/V/W 相序正向推进；
     * -1 表示反向推进。
     * 正向是否对应机械上的“顺时针”，还取决于相线接法和观察方向。
     */
    int8_t direction;
    /* 从静止逐渐升到目标转速所需的时间，单位为毫秒。 */
    uint32_t ramp_time_ms;
    /*
     * 电角度偏移量，单位为度。
     * 这是磁场相位校准量，不是占空比偏移，也不会改变目标速度。
     */
    float electrical_angle_offset_deg;
} SimpleFOC_OpenLoopConfig;

/*
 * 启动 TIM1 和 TIM8 的六路 PWM，并将占空比设置为中性值。
 * 初始化结束后两个电机仍保持关闭，只有显式调用 Enable 才会使能。
 */
HAL_StatusTypeDef SimpleFOC_Motor_Init(void);

/* 按轴控制高电平有效的电机使能引脚。 */
void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis);
void SimpleFOC_Motor_Disable(SimpleFOC_MotorAxis axis);

/*
 * 启动指定轴的开环运行。
 * 函数会先输出初始三相电压，再打开对应 EN 引脚。
 */
HAL_StatusTypeDef SimpleFOC_Motor_OpenLoopStart(SimpleFOC_MotorAxis axis,
                                                const SimpleFOC_OpenLoopConfig *config);

/*
 * 更新指定轴的开环输出。
 * 应在周期性任务中持续调用，调用间隔可以有轻微变化，函数会使用
 * HAL_GetTick() 计算实际经过的时间。
 */
void SimpleFOC_Motor_OpenLoopUpdate(SimpleFOC_MotorAxis axis);

/* 停止指定轴的开环输出，并恢复到安全的中性 PWM。 */
void SimpleFOC_Motor_OpenLoopStop(SimpleFOC_MotorAxis axis);

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
