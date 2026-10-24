#include "SimpleFOC_Motor.h"

#include "arm_math.h"
#include "tim.h"
#include "AS5600.h"
#include "W25Q64.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define SFOC_PI                    3.14159265358979323846f
#define SFOC_TWO_PI                (2.0f * SFOC_PI)
#define SFOC_RAD_TO_DEG            (180.0f / SFOC_PI)
#define SFOC_HALF                  0.5f
#define SFOC_SQRT3_BY_TWO          0.86602540378443864676f

/* 一个电机轴所使用的定时器、使能引脚和三相 PWM 映射。 */
typedef struct
{
    TIM_HandleTypeDef *timer;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    uint32_t channel_a;
    uint32_t channel_b;
    uint32_t channel_c;
} SFOC_AxisHardware;

static const SFOC_AxisHardware g_vertical_axis = {
    &htim1, TIM1_EN_GPIO_Port, TIM1_EN_Pin,
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
};

static const SFOC_AxisHardware g_horizontal_axis = {
    &htim8, TIM8_EN_GPIO_Port, TIM8_EN_Pin,
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
};

static float sfoc_limit(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float sfoc_normalize_angle(float angle)
{
    while (angle >= SFOC_TWO_PI)
    {
        angle -= SFOC_TWO_PI;
    }
    while (angle < 0.0f)
    {
        angle += SFOC_TWO_PI;
    }
    return angle;
}

static const SFOC_AxisHardware *sfoc_get_axis(SimpleFOC_MotorAxis axis)
{
    return axis == SIMPLEFOC_AXIS_HORIZONTAL ? &g_horizontal_axis : &g_vertical_axis;
}

static void sfoc_set_neutral_pwm(const SFOC_AxisHardware *axis)
{
    uint32_t neutral_compare = __HAL_TIM_GET_AUTORELOAD(axis->timer) / 2U;

    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_a, neutral_compare);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_b, neutral_compare);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_c, neutral_compare);
}

static void sfoc_stop_pwm(const SFOC_AxisHardware *axis)
{
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_a);
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_b);
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_c);
}

static HAL_StatusTypeDef sfoc_start_pwm(const SFOC_AxisHardware *axis)
{
    if ((HAL_TIM_PWM_Start(axis->timer, axis->channel_a) != HAL_OK)
        || (HAL_TIM_PWM_Start(axis->timer, axis->channel_b) != HAL_OK)
        || (HAL_TIM_PWM_Start(axis->timer, axis->channel_c) != HAL_OK))
    {
        return HAL_ERROR;
    }

    sfoc_set_neutral_pwm(axis);
    HAL_GPIO_WritePin(axis->en_port, axis->en_pin, GPIO_PIN_RESET);
    return HAL_OK;
}

HAL_StatusTypeDef SimpleFOC_Motor_Init(void)
{
    HAL_GPIO_WritePin(g_vertical_axis.en_port, g_vertical_axis.en_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_horizontal_axis.en_port, g_horizontal_axis.en_pin, GPIO_PIN_RESET);

    if (sfoc_start_pwm(&g_vertical_axis) != HAL_OK)
    {
        sfoc_stop_pwm(&g_vertical_axis);
        return HAL_ERROR;
    }
    if (sfoc_start_pwm(&g_horizontal_axis) != HAL_OK)
    {
        sfoc_stop_pwm(&g_vertical_axis);
        sfoc_stop_pwm(&g_horizontal_axis);
        return HAL_ERROR;
    }
    return HAL_OK;
}

void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis)
{
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);
    HAL_GPIO_WritePin(hardware->en_port, hardware->en_pin, GPIO_PIN_SET);
}

void SimpleFOC_Motor_Disable(SimpleFOC_MotorAxis axis)
{
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);
    HAL_GPIO_WritePin(hardware->en_port, hardware->en_pin, GPIO_PIN_RESET);
    sfoc_set_neutral_pwm(hardware);
}

void SimpleFOC_Motor_SetPhaseVoltage(SimpleFOC_MotorAxis axis,
                                     float voltage_d,
                                     float voltage_q,
                                     float electrical_angle,
                                     float supply_voltage)
{
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);
    SimpleFOC_DirectQuadrature direct_quadrature = {voltage_d, voltage_q};
    SimpleFOC_AlphaBeta alpha_beta = {0.0f, 0.0f};
    float duty_a;
    float duty_b;
    float duty_c;
    uint32_t period;

    if (supply_voltage <= 0.0f)
    {
        SimpleFOC_Motor_Disable(axis);
        return;
    }

    SimpleFOC_InvPark(&direct_quadrature,
                       sfoc_normalize_angle(electrical_angle),
                       &alpha_beta);
    SimpleFOC_SVPWM(&alpha_beta, supply_voltage, &duty_a, &duty_b, &duty_c);

    period = __HAL_TIM_GET_AUTORELOAD(hardware->timer) + 1U;
    __HAL_TIM_SET_COMPARE(hardware->timer, hardware->channel_a,
                          (uint32_t)(duty_a * (float)period));
    __HAL_TIM_SET_COMPARE(hardware->timer, hardware->channel_b,
                          (uint32_t)(duty_b * (float)period));
    __HAL_TIM_SET_COMPARE(hardware->timer, hardware->channel_c,
                          (uint32_t)(duty_c * (float)period));
}

float SimpleFOC_Motor_CalculateElectricalAngle(float mechanical_angle,
                                               float electrical_offset,
                                               uint8_t pole_pairs,
                                               int8_t direction)
{
    if ((pole_pairs == 0U) || ((direction != 1) && (direction != -1)))
    {
        return 0.0f;
    }

    return sfoc_normalize_angle((float)direction * (float)pole_pairs
                                * mechanical_angle + electrical_offset);
}

float SimpleFOC_Motor_CalculateElectricalOffset(float mechanical_angle,
                                                float alignment_angle,
                                                uint8_t pole_pairs,
                                                int8_t direction)
{
    if ((pole_pairs == 0U) || ((direction != 1) && (direction != -1)))
    {
        return 0.0f;
    }

    return sfoc_normalize_angle(alignment_angle
                                - (float)direction * (float)pole_pairs
                                * mechanical_angle);
}

void SimpleFOC_InvPark(const SimpleFOC_DirectQuadrature *input,
                       float electrical_angle,
                       SimpleFOC_AlphaBeta *output)
{
    float sine;
    float cosine;

    if ((input == 0) || (output == 0))
    {
        return;
    }

    arm_sin_cos_f32(electrical_angle * SFOC_RAD_TO_DEG, &sine, &cosine);
    output->alpha = input->d * cosine - input->q * sine;
    output->beta = input->d * sine + input->q * cosine;
}

void SimpleFOC_SVPWM(const SimpleFOC_AlphaBeta *input,
                     float supply_voltage,
                     float *duty_a,
                     float *duty_b,
                     float *duty_c)
{
    float phase_voltage[3];
    float maximum;
    float minimum;
    float common_mode;
    uint32_t maximum_index;
    uint32_t minimum_index;

    if ((input == 0) || (duty_a == 0) || (duty_b == 0) || (duty_c == 0)
        || (supply_voltage <= 0.0f))
    {
        return;
    }

    phase_voltage[0] = input->alpha;
    phase_voltage[1] = -SFOC_HALF * input->alpha
                       + SFOC_SQRT3_BY_TWO * input->beta;
    phase_voltage[2] = -SFOC_HALF * input->alpha
                       - SFOC_SQRT3_BY_TWO * input->beta;
    arm_max_f32(phase_voltage, 3U, &maximum, &maximum_index);
    arm_min_f32(phase_voltage, 3U, &minimum, &minimum_index);
    common_mode = -SFOC_HALF * (maximum + minimum);

    *duty_a = sfoc_limit(SFOC_HALF + (phase_voltage[0] + common_mode) / supply_voltage,
                         0.0f, 1.0f);
    *duty_b = sfoc_limit(SFOC_HALF + (phase_voltage[1] + common_mode) / supply_voltage,
                         0.0f, 1.0f);
    *duty_c = sfoc_limit(SFOC_HALF + (phase_voltage[2] + common_mode) / supply_voltage,
                         0.0f, 1.0f);
}

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
/* 角度源改为 0x0E 后更新记录版本，旧偏移量需重新测量。 */
#define MOTOR_OFFSET_SIGNATURE          0x32464F43UL
#define MOTOR_OFFSET_CONFIG             ((MOTOR_POLE_PAIRS << 8) | (MOTOR_DIRECTION_CLOCKWISE & 0xFF))
#define MOTOR_OFFSET_TIM1_VALID         0x01U
#define MOTOR_OFFSET_TIM8_VALID         0x02U

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
/* TIM8 只记住偏移量是否有效；上电加载不会启动 PWM。 */
static uint8_t g_tim8_offset_valid;
/* 两轴共用一个参数包，由 W25Q64 驱动在原有两个扇区做双备份。 */
typedef struct
{
    uint32_t signature;    /* 数据格式版本。 */
    uint32_t config;       /* 极对数与方向，避免误用其他电机配置的偏移量。 */
    uint32_t valid_flags;  /* 分别标记 TIM1、TIM8 是否已完成校准。 */
    float tim1_offset;
    float tim8_offset;
} MotorOffsetRecord;

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

/* 每次上电首次使用 TIM8 前调用；已有有效偏移时只建立当前位置。 */
/* 尚未校准时才短暂使能 TIM8 对齐，校准成功后立即关闭并保存双备份。 */
HAL_StatusTypeDef SimpleFOC_Motor_PrepareTIM8(void)
{
    MotorOffsetRecord record;
    HAL_StatusTypeDef status;

    if (g_motor_state.ready == 0U || g_motor_state.fault != 0U)
        return HAL_BUSY;
    if (g_tim8_offset_valid != 0U)
    {
        /* 从 Flash 恢复偏移后，还需读取本次上电的机械角度作为位置原点。 */
        if (g_tim8_control.feedback_initialized != 0U) return HAL_OK;
        status = motor_update_feedback(&g_tim8_control, 0.005f);
        if (status == HAL_OK)
            g_tim8_control.mechanical_origin = g_tim8_control.mechanical_position;
        return status;
    }

    /* 再读一次整包，保存 TIM8 时保留原有的 TIM1 偏移量。 */
    status = W25Q64_LoadParameters(&record, sizeof(record));
    if (status != HAL_OK) return status;
    if (record.signature != MOTOR_OFFSET_SIGNATURE
        || record.config != MOTOR_OFFSET_CONFIG
        || (record.valid_flags & MOTOR_OFFSET_TIM1_VALID) == 0U)
        return HAL_ERROR;
    if ((record.valid_flags & MOTOR_OFFSET_TIM8_VALID) != 0U
        && isfinite(record.tim8_offset)
        && record.tim8_offset >= 0.0f
        && record.tim8_offset < MOTOR_TWO_PI)
    {
        /* 处理上一次保存中途断电但已有一份新记录的情况。 */
        g_tim8_control.electrical_offset = record.tim8_offset;
        g_tim8_offset_valid = 1U;
        status = motor_update_feedback(&g_tim8_control, 0.005f);
        if (status == HAL_OK)
            g_tim8_control.mechanical_origin = g_tim8_control.mechanical_position;
        return status;
    }

    /* 首次显式准备时才会真正对齐，启动任务本身绝不调用这里。 */
    status = motor_auto_calibrate(&g_tim8_control);
    if (status != HAL_OK) return status;
    record.tim8_offset = g_tim8_control.electrical_offset;
    record.valid_flags |= MOTOR_OFFSET_TIM8_VALID;
    status = W25Q64_SaveParameters(&record, sizeof(record));
    if (status == HAL_OK) g_tim8_offset_valid = 1U;
    return status;
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

void SimpleFOC_Motor_RunPositionTask(void *argument)
{
    MotorOffsetRecord offset_record = {0};
    HAL_StatusTypeDef storage_status;
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

    /* 两轴建立控制对象；TIM8 只读取已保存的偏移量，不使能电机。 */
    motor_control_init(&g_tim1_control, SIMPLEFOC_AXIS_VERTICAL, &hi2c1);
    motor_control_init(&g_tim8_control, SIMPLEFOC_AXIS_HORIZONTAL, &hi2c3);
    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_HORIZONTAL);

    /* 先校验双备份；旧版 0x0C 偏移量不适用于现在的 0x0E 角度源。 */
    storage_status = W25Q64_Init();
    if (storage_status == HAL_OK)
        storage_status = W25Q64_LoadParameters(&offset_record, sizeof(offset_record));
    if (storage_status == HAL_BUSY || (storage_status == HAL_OK
        && (offset_record.signature != MOTOR_OFFSET_SIGNATURE
            || offset_record.config != MOTOR_OFFSET_CONFIG)))
    {
        /* 新芯片或记录版本不匹配：建立空记录，后面只校准 TIM1。 */
        offset_record.signature = MOTOR_OFFSET_SIGNATURE;
        offset_record.config = MOTOR_OFFSET_CONFIG;
        offset_record.valid_flags = 0U;
        storage_status = HAL_OK;
    }

    /* TIM8 只载入偏移量，不读传感器、不使能驱动，也不发转动目标。 */
    if (storage_status == HAL_OK
        && (offset_record.valid_flags & MOTOR_OFFSET_TIM8_VALID) != 0U
        && isfinite(offset_record.tim8_offset)
        && offset_record.tim8_offset >= 0.0f
        && offset_record.tim8_offset < MOTOR_TWO_PI)
    {
        g_tim8_control.electrical_offset = offset_record.tim8_offset;
        g_tim8_offset_valid = 1U;
    }

    /* TIM1 有有效偏移时用当前 AS5600 角度建立位置原点，跳过锁转子。 */
    if (storage_status == HAL_OK
        && (offset_record.valid_flags & MOTOR_OFFSET_TIM1_VALID) != 0U
        && isfinite(offset_record.tim1_offset)
        && offset_record.tim1_offset >= 0.0f
        && offset_record.tim1_offset < MOTOR_TWO_PI)
    {
        g_tim1_control.electrical_offset = offset_record.tim1_offset;
        storage_status = motor_update_feedback(&g_tim1_control, 0.005f);
        g_tim1_control.mechanical_origin = g_tim1_control.mechanical_position;
    }
    else if (storage_status == HAL_OK)
    {
        /* 只有 TIM1 没有可用偏移时才执行一次对齐并写入参数包。 */
        storage_status = motor_auto_calibrate(&g_tim1_control);
        if (storage_status == HAL_OK)
        {
            offset_record.tim1_offset = g_tim1_control.electrical_offset;
            offset_record.valid_flags |= MOTOR_OFFSET_TIM1_VALID;
            storage_status = W25Q64_SaveParameters(&offset_record, sizeof(offset_record));
        }
    }

    /* 读写 Flash、传感器或校准失败时保持驱动关闭，避免错误电角度输出。 */
    if (storage_status != HAL_OK)
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
    /* 保留原来的上电测试动作：TIM1 从当前机械位置顺时针转 180 度。 */
    (void)MotorPosition_SetTargetDegrees(180.0f);
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
