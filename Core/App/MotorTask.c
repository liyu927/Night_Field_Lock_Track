#include "AS5600.h"
#include "SimpleFOC_Motor.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "pid.h"
#include "MotorPosition.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

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

/* D32/D33 实测有效的双闭环参数；力矩环不测电流，只限制 q 轴电压。 */
#define MOTOR_POSITION_KP                2.0f
#define MOTOR_MAX_SPEED_DPS              20.0f
#define MOTOR_SPEED_KP                   2.5f
#define MOTOR_SPEED_KI                   5.0f
#define MOTOR_SPEED_FILTER_S             0.08f
#define MOTOR_MAX_Q_VOLTAGE              1.5f
#define MOTOR_Q_SLEW_VPS                 5.0f
#define MOTOR_MAX_STEP_DEG               8.0f
#define MOTOR_MAX_OVERSHOOT_DEG         10.0f
#define MOTOR_MAX_COMMAND_DEG          180.0f

/* 一个轴的 AS5600 反馈、闭环状态和 PID 控制器。 */
typedef struct
{
    SimpleFOC_MotorAxis axis;
    AS5600_Handle_t encoder;
    PID_Controller_t position_pid;
    PID_Controller_t speed_pid;
    float electrical_offset;
    float raw_angle;
    float mechanical_origin;
    float mechanical_position;
    float mechanical_speed;
    float applied_q_voltage;
    uint8_t feedback_initialized;
} MotorControl_t;

static MotorControl_t g_tim1_control;
static MotorControl_t g_tim8_control;
typedef struct
{
    float target_deg;
    float actual_deg;
    uint8_t ready;
    uint8_t fault;
} MotorPositionState;

static MotorPositionState g_motor_state;
static float g_command_start_deg;
static uint8_t g_command_valid;

uint8_t MotorPosition_SetTargetDegrees(float delta_deg)
{
    if (!isfinite(delta_deg) || fabsf(delta_deg) > MOTOR_MAX_COMMAND_DEG)
        return 0U;

    taskENTER_CRITICAL();
    if (g_motor_state.ready == 0U || g_motor_state.fault != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    g_command_start_deg = g_motor_state.actual_deg;
    g_motor_state.target_deg = g_command_start_deg + delta_deg;
    g_command_valid = 1U;
    taskEXIT_CRITICAL();
    return 1U;
}

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
    control->mechanical_origin = 0.0f;
    control->mechanical_position = 0.0f;
    control->mechanical_speed = 0.0f;
    control->applied_q_voltage = 0.0f;
    control->feedback_initialized = 0U;

    /* 位置环输出速度目标，速度环输出 q 轴电压，即无电流传感器时的力矩指令。 */
    PID_Init(&control->position_pid,
             MOTOR_POSITION_KP, 0.0f, 0.0f,
             -MOTOR_MAX_SPEED_DPS * MOTOR_PI / 180.0f,
             MOTOR_MAX_SPEED_DPS * MOTOR_PI / 180.0f);
    PID_Init(&control->speed_pid,
             MOTOR_SPEED_KP, MOTOR_SPEED_KI, 0.0f,
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
    if (fabsf(angle_increment) > MOTOR_MAX_STEP_DEG * MOTOR_PI / 180.0f)
        return HAL_ERROR;
    control->raw_angle = current_angle;
    control->mechanical_position += angle_increment;
    control->mechanical_speed += interval_s / (MOTOR_SPEED_FILTER_S + interval_s)
        * (angle_increment / interval_s - control->mechanical_speed);
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
    control->mechanical_origin = mechanical_angle;
    control->mechanical_position = mechanical_angle;
    control->mechanical_speed = 0.0f;
    control->applied_q_voltage = 0.0f;
    control->feedback_initialized = 1U;
    PID_Reset(&control->position_pid);
    PID_Reset(&control->speed_pid);
    return HAL_OK;
}

/* 执行位置环、速度环并把 q 轴电压指令输出到三相 PWM。 */
static void motor_run_closed_loop(MotorControl_t *control,
                                  float target_position,
                                  float interval_s)
{
    float target_speed;
    float q_voltage;
    float electrical_angle;
    float previous_integral;
    float max_change;
    float upper;
    float lower;

    target_speed = PID_Update(&control->position_pid,
                              target_position,
                              control->mechanical_position,
                              interval_s);
    previous_integral = control->speed_pid.integral;
    q_voltage = PID_Update(&control->speed_pid,
                           target_speed,
                           control->mechanical_speed,
                           interval_s);
    max_change = MOTOR_Q_SLEW_VPS * interval_s;
    upper = fmaxf(control->applied_q_voltage, 0.0f) + max_change;
    lower = fminf(control->applied_q_voltage, 0.0f) - max_change;
    q_voltage = fmaxf(lower, fminf(upper, q_voltage));
    if (((q_voltage >= upper && target_speed > control->mechanical_speed)
         || (q_voltage <= lower && target_speed < control->mechanical_speed))
        && fabsf(control->speed_pid.integral) >= fabsf(previous_integral))
        control->speed_pid.integral = previous_integral;
    control->applied_q_voltage = q_voltage;
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
}

void StartMotorTask(void *argument)
{
    uint32_t next_wake_tick;
    uint32_t previous_tick;
    uint32_t elapsed_ms;
    float target_position;
    float interval_s;
    float target_deg;
    float command_start_deg;
    float actual_deg;
    uint8_t command_valid;
    uint8_t driver_enabled = 0U;

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
        taskENTER_CRITICAL();
        g_motor_state.fault = 1U;
        taskEXIT_CRITICAL();
        for (;;)
        {
            osDelay(100U);
        }
    }

    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
    taskENTER_CRITICAL();
    g_motor_state.ready = 1U;
    taskEXIT_CRITICAL();
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
        if (motor_update_feedback(&g_tim1_control, interval_s) != HAL_OK)
        {
            SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
            taskENTER_CRITICAL();
            g_motor_state.fault = 1U;
            taskEXIT_CRITICAL();
            for (;;) osDelay(100U);
        }

        actual_deg = (g_tim1_control.mechanical_position
            - g_tim1_control.mechanical_origin) * 180.0f / MOTOR_PI;
        taskENTER_CRITICAL();
        g_motor_state.actual_deg = actual_deg;
        target_deg = g_motor_state.target_deg;
        command_start_deg = g_command_start_deg;
        command_valid = g_command_valid;
        taskEXIT_CRITICAL();
        if (command_valid == 0U) continue;

        if ((target_deg > command_start_deg
             && actual_deg > target_deg + MOTOR_MAX_OVERSHOOT_DEG)
            || (target_deg < command_start_deg
                && actual_deg < target_deg - MOTOR_MAX_OVERSHOOT_DEG))
        {
            SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
            taskENTER_CRITICAL();
            g_motor_state.fault = 1U;
            taskEXIT_CRITICAL();
            for (;;) osDelay(100U);
        }

        if (driver_enabled == 0U)
        {
            PID_Reset(&g_tim1_control.position_pid);
            PID_Reset(&g_tim1_control.speed_pid);
            SimpleFOC_Motor_Enable(SIMPLEFOC_AXIS_VERTICAL);
            driver_enabled = 1U;
        }
        target_position = g_tim1_control.mechanical_origin
            + target_deg * MOTOR_PI / 180.0f;
        motor_run_closed_loop(&g_tim1_control, target_position, interval_s);
    }
}
