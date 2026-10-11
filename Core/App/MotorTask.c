#include "AS5600.h"
#include "SimpleFOC_Motor.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "pid.h"

#define MOTOR_PI                         3.14159265358979323846f
#define MOTOR_TWO_PI                     (2.0f * MOTOR_PI)
#define MOTOR_TASK_PERIOD_MS             5U

/* 两个电机的已确认硬件参数。 */
#define MOTOR_POLE_PAIRS                 7U
#define MOTOR_SUPPLY_VOLTAGE             12.0f
#define MOTOR_DIRECTION_CLOCKWISE        1

/* 自动电角零位校准参数：低 d 轴电压锁定转子后读取 AS5600。 */
#define MOTOR_ALIGNMENT_VOLTAGE          1.0f
#define MOTOR_ALIGNMENT_TIME_MS          800U
#define MOTOR_ALIGNMENT_ANGLE_RAD        0.0f
/* TIM8 当前必须静止；机械安全后改为 1U 可在启动时执行同样的零位校准。 */
#define TIM8_AUTO_CALIBRATE_ON_BOOT      0U

/* TIM1 演示轨迹：先顺时针慢速，5 秒后逆时针以约两倍速度运行。 */
#define TIM1_CLOCKWISE_SPEED_RPS         0.10f
#define TIM1_COUNTERCLOCKWISE_SPEED_RPS  (-0.20f)
#define TIM1_DIRECTION_SWITCH_TIME_MS    5000U

/* 位置环输出速度上限，速度环输出 q 轴电压上限。 */
#define MOTOR_MAX_SPEED_RPS              0.35f
#define MOTOR_MAX_Q_VOLTAGE              2.5f

/* 一个轴的 AS5600 反馈、闭环状态和 PID 控制器。 */
typedef struct
{
    SimpleFOC_MotorAxis axis;
    AS5600_Handle_t encoder;
    PID_Controller_t position_pid;
    PID_Controller_t speed_pid;
    float electrical_offset;
    float raw_angle;
    float mechanical_position;
    float mechanical_speed;
    uint8_t feedback_initialized;
} MotorControl_t;

static MotorControl_t g_tim1_control;
static MotorControl_t g_tim8_control;

/* 将相邻两次绝对角度的差值限制到 -π 至 π，跨越 0 度时保持连续。 */
static float motor_angle_difference(float current, float previous)
{
    float difference = current - previous;

    while (difference > MOTOR_PI)
    {
        difference -= MOTOR_TWO_PI;
    }
    while (difference < -MOTOR_PI)
    {
        difference += MOTOR_TWO_PI;
    }
    return difference;
}

/* 初始化一个电机轴的控制状态；不会使能 PWM 或驱动电机。 */
static void motor_control_init(MotorControl_t *control,
                               SimpleFOC_MotorAxis axis,
                               I2C_HandleTypeDef *i2c)
{
    control->axis = axis;
    control->encoder.i2c = i2c;
    control->encoder.address = AS5600_I2C_ADDRESS;
    control->electrical_offset = 0.0f;
    control->raw_angle = 0.0f;
    control->mechanical_position = 0.0f;
    control->mechanical_speed = 0.0f;
    control->feedback_initialized = 0U;

    /* 位置环输出速度目标，速度环输出 q 轴电压，即无电流传感器时的力矩指令。 */
    PID_Init(&control->position_pid,
             4.0f, 0.0f, 0.02f,
             -MOTOR_TWO_PI * MOTOR_MAX_SPEED_RPS,
             MOTOR_TWO_PI * MOTOR_MAX_SPEED_RPS);
    PID_Init(&control->speed_pid,
             1.0f, 3.0f, 0.0f,
             -MOTOR_MAX_Q_VOLTAGE,
             MOTOR_MAX_Q_VOLTAGE);
}

/* 读取 AS5600 并更新连续机械位置与机械角速度。 */
static HAL_StatusTypeDef motor_update_feedback(MotorControl_t *control,
                                                float interval_s)
{
    float current_angle;
    float angle_increment;

    if (AS5600_ReadMechanicalAngle(&control->encoder, &current_angle) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (control->feedback_initialized == 0U)
    {
        control->raw_angle = current_angle;
        control->mechanical_position = current_angle;
        control->mechanical_speed = 0.0f;
        control->feedback_initialized = 1U;
        return HAL_OK;
    }

    angle_increment = motor_angle_difference(current_angle, control->raw_angle);
    control->raw_angle = current_angle;
    control->mechanical_position += angle_increment;
    control->mechanical_speed = angle_increment / interval_s;
    return HAL_OK;
}

/*
 * 自动校准电角零位：固定输出已知 d 轴电压，待转子对齐后读取 AS5600，
 * 再计算 FOC 所需的电角偏移。校准期间转子会移动到锁定角，必须先卸载机构。
 */
static HAL_StatusTypeDef motor_auto_calibrate(MotorControl_t *control)
{
    float mechanical_angle;

    SimpleFOC_Motor_SetPhaseVoltage(control->axis,
                                    MOTOR_ALIGNMENT_VOLTAGE,
                                    0.0f,
                                    MOTOR_ALIGNMENT_ANGLE_RAD,
                                    MOTOR_SUPPLY_VOLTAGE);
    SimpleFOC_Motor_Enable(control->axis);
    osDelay(MOTOR_ALIGNMENT_TIME_MS);

    if (AS5600_ReadMechanicalAngle(&control->encoder, &mechanical_angle) != HAL_OK)
    {
        SimpleFOC_Motor_Disable(control->axis);
        return HAL_ERROR;
    }

    control->electrical_offset = SimpleFOC_Motor_CalculateElectricalOffset(
        mechanical_angle,
        MOTOR_ALIGNMENT_ANGLE_RAD,
        MOTOR_POLE_PAIRS,
        MOTOR_DIRECTION_CLOCKWISE);
    SimpleFOC_Motor_Disable(control->axis);

    /* 用校准时的真实角度建立位置原点，避免第一周期计算出虚假的速度。 */
    control->raw_angle = mechanical_angle;
    control->mechanical_position = mechanical_angle;
    control->mechanical_speed = 0.0f;
    control->feedback_initialized = 1U;
    PID_Reset(&control->position_pid);
    PID_Reset(&control->speed_pid);
    return HAL_OK;
}

/* 执行位置环、速度环并把 q 轴电压指令输出到三相 PWM。 */
static HAL_StatusTypeDef motor_run_closed_loop(MotorControl_t *control,
                                                float target_position,
                                                float interval_s)
{
    float target_speed;
    float q_voltage;
    float electrical_angle;

    if (motor_update_feedback(control, interval_s) != HAL_OK)
    {
        SimpleFOC_Motor_Disable(control->axis);
        return HAL_ERROR;
    }

    target_speed = PID_Update(&control->position_pid,
                              target_position,
                              control->mechanical_position,
                              interval_s);
    q_voltage = PID_Update(&control->speed_pid,
                           target_speed,
                           control->mechanical_speed,
                           interval_s);
    electrical_angle = SimpleFOC_Motor_CalculateElectricalAngle(
        control->raw_angle,
        control->electrical_offset,
        MOTOR_POLE_PAIRS,
        MOTOR_DIRECTION_CLOCKWISE);

    SimpleFOC_Motor_SetPhaseVoltage(control->axis,
                                    0.0f,
                                    q_voltage,
                                    electrical_angle,
                                    MOTOR_SUPPLY_VOLTAGE);
    return HAL_OK;
}

void StartMotorTask(void *argument)
{
    uint32_t next_wake_tick;
    uint32_t previous_tick;
    uint32_t elapsed_ms;
    uint32_t running_time_ms = 0U;
    float target_position;
    float target_speed_rps;
    float interval_s;

    (void)argument;

    /* I2C1/TIM1 与 I2C3/TIM8 都建立完整控制对象；TIM8 不做校准和使能，因此不会转动。 */
    motor_control_init(&g_tim1_control, SIMPLEFOC_AXIS_VERTICAL, &hi2c1);
    motor_control_init(&g_tim8_control, SIMPLEFOC_AXIS_HORIZONTAL, &hi2c3);
    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_HORIZONTAL);

    if ((TIM8_AUTO_CALIBRATE_ON_BOOT != 0U)
        && (motor_auto_calibrate(&g_tim8_control) != HAL_OK))
    {
        SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_HORIZONTAL);
    }

    if (motor_auto_calibrate(&g_tim1_control) != HAL_OK)
    {
        SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
        for (;;)
        {
            osDelay(100U);
        }
    }

    target_position = g_tim1_control.mechanical_position;
    SimpleFOC_Motor_Enable(SIMPLEFOC_AXIS_VERTICAL);
    next_wake_tick = osKernelGetTickCount();
    previous_tick = next_wake_tick;

    for (;;)
    {
        next_wake_tick += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);

        elapsed_ms = osKernelGetTickCount() - previous_tick;
        previous_tick += elapsed_ms;
        if (elapsed_ms == 0U)
        {
            continue;
        }
        if (elapsed_ms > 20U)
        {
            elapsed_ms = 20U;
        }
        interval_s = (float)elapsed_ms * 0.001f;
        running_time_ms += elapsed_ms;

        if (running_time_ms < TIM1_DIRECTION_SWITCH_TIME_MS)
        {
            target_speed_rps = TIM1_CLOCKWISE_SPEED_RPS;
        }
        else
        {
            target_speed_rps = TIM1_COUNTERCLOCKWISE_SPEED_RPS;
        }
        target_position += target_speed_rps * MOTOR_TWO_PI * interval_s;

        if (motor_run_closed_loop(&g_tim1_control, target_position, interval_s) != HAL_OK)
        {
            /* I2C 反馈失效时立即关断 TIM1，禁止使用旧角度继续驱动。 */
            SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
        }
    }
}
