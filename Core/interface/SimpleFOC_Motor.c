#include "SimpleFOC_Motor.h"

#include "cmsis_os2.h"
#include "i2c.h"
#include "tim.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define SFOC_PI                         3.14159265358979323846f
#define SFOC_TWO_PI                     (2.0f * SFOC_PI)
#define SFOC_SQRT3                      1.73205080756887729353f

/* 这些参数与样例工程保持一致。 */
#define SFOC_SUPPLY_VOLTAGE             12.0f
#define SFOC_VOLTAGE_LIMIT              10.0f
#define SFOC_MOTOR_DIRECTION            1
#define SFOC_MOTOR_POLE_PAIRS           7

/* 位置环和校准参数。后续调试时只需要调整这些宏。 */
#define SFOC_POSITION_KP                3.0f
#define SFOC_POSITION_VOLTAGE_LIMIT     3.0f
#define SFOC_ALIGNMENT_VOLTAGE          2.0f
#define SFOC_ALIGNMENT_HOLD_MS          500U
#define SFOC_UPDATE_PERIOD_MS           2U

#define SFOC_AS5600_ADDRESS             (0x36U << 1)
#define SFOC_AS5600_ANGLE_REGISTER      0x0CU
#define SFOC_AS5600_RESOLUTION          4096.0f
#define SFOC_I2C_TIMEOUT_MS             5U

typedef enum
{
    SFOC_AXIS_WAITING = 0,
    SFOC_AXIS_ALIGNING,
    SFOC_AXIS_RUNNING
} SFOC_AxisState;

typedef struct
{
    TIM_HandleTypeDef *timer;
    I2C_HandleTypeDef *i2c;
    uint32_t en_pin;
    GPIO_TypeDef *en_port;
    uint32_t channel_a;
    uint32_t channel_b;
    uint32_t channel_c;

    float target_angle;
    float angle_raw;
    float angle_continuous;
    float last_angle_raw;
    float zero_electric_angle;
    uint32_t alignment_start_tick;
    bool angle_valid;
    bool sensor_online;
    SFOC_AxisState state;
} SFOC_Axis;

static SFOC_Axis g_vertical_axis;
static SFOC_Axis g_horizontal_axis;

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

static float sfoc_limit(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static bool sfoc_read_angle(SFOC_Axis *axis)
{
    uint8_t data[2] = {0};
    uint16_t raw_value;
    float new_angle;

    if (HAL_I2C_Mem_Read(axis->i2c,
                         SFOC_AS5600_ADDRESS,
                         SFOC_AS5600_ANGLE_REGISTER,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         sizeof(data),
                         SFOC_I2C_TIMEOUT_MS) != HAL_OK)
    {
        axis->sensor_online = false;
        return false;
    }

    /* AS5600 的角度值是 12 位，低四位必须清零。 */
    raw_value = (uint16_t)((((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU);
    new_angle = ((float)raw_value / SFOC_AS5600_RESOLUTION) * SFOC_TWO_PI;

    if (!axis->angle_valid)
    {
        axis->angle_raw = new_angle;
        axis->last_angle_raw = new_angle;
        axis->angle_continuous = new_angle;
        axis->angle_valid = true;
    }
    else
    {
        float delta = new_angle - axis->last_angle_raw;

        /* 处理 AS5600 从 4095 跳回 0 或从 0 跳到 4095 的情况。 */
        if (delta > SFOC_PI)
        {
            delta -= SFOC_TWO_PI;
        }
        else if (delta < -SFOC_PI)
        {
            delta += SFOC_TWO_PI;
        }

        axis->angle_continuous += delta;
        axis->angle_raw = new_angle;
        axis->last_angle_raw = new_angle;
    }

    axis->sensor_online = true;
    return true;
}

static void sfoc_set_enabled(SFOC_Axis *axis, bool enabled)
{
    HAL_GPIO_WritePin(axis->en_port,
                      axis->en_pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void sfoc_set_neutral_pwm(SFOC_Axis *axis)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(axis->timer);
    uint32_t neutral = period / 2U;

    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_a, neutral);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_b, neutral);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_c, neutral);
}

static void sfoc_set_torque(SFOC_Axis *axis, float uq, float electrical_angle)
{
    float u_alpha;
    float u_beta;
    float u_a;
    float u_b;
    float u_c;
    float duty_a;
    float duty_b;
    float duty_c;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(axis->timer);

    uq = sfoc_limit(uq,
                    -SFOC_POSITION_VOLTAGE_LIMIT,
                    SFOC_POSITION_VOLTAGE_LIMIT);
    electrical_angle = sfoc_normalize_angle(electrical_angle);

    /*
     * 逆 Park 变换：仅使用 Uq，Ud 固定为 0。
     * 随后加入半母线电压偏置，使三相 PWM 在无转矩时保持约 50%。
     */
    u_alpha = -uq * sinf(electrical_angle);
    u_beta = uq * cosf(electrical_angle);

    u_a = u_alpha + SFOC_VOLTAGE_LIMIT / 2.0f;
    u_b = (-u_alpha + SFOC_SQRT3 * u_beta) / 2.0f
          + SFOC_VOLTAGE_LIMIT / 2.0f;
    u_c = (-u_alpha - SFOC_SQRT3 * u_beta) / 2.0f
          + SFOC_VOLTAGE_LIMIT / 2.0f;

    duty_a = sfoc_limit(u_a / SFOC_SUPPLY_VOLTAGE, 0.0f, 1.0f);
    duty_b = sfoc_limit(u_b / SFOC_SUPPLY_VOLTAGE, 0.0f, 1.0f);
    duty_c = sfoc_limit(u_c / SFOC_SUPPLY_VOLTAGE, 0.0f, 1.0f);

    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_a,
                          (uint32_t)(duty_a * (float)period));
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_b,
                          (uint32_t)(duty_b * (float)period));
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_c,
                          (uint32_t)(duty_c * (float)period));
}

static void sfoc_start_pwm(SFOC_Axis *axis)
{
    HAL_TIM_PWM_Start(axis->timer, axis->channel_a);
    HAL_TIM_PWM_Start(axis->timer, axis->channel_b);
    HAL_TIM_PWM_Start(axis->timer, axis->channel_c);
    sfoc_set_neutral_pwm(axis);
    sfoc_set_enabled(axis, false);
}

static void sfoc_initialize_axis(SFOC_Axis *axis,
                                  TIM_HandleTypeDef *timer,
                                  I2C_HandleTypeDef *i2c,
                                  GPIO_TypeDef *en_port,
                                  uint32_t en_pin,
                                  uint32_t channel_a,
                                  uint32_t channel_b,
                                  uint32_t channel_c)
{
    memset(axis, 0, sizeof(*axis));
    axis->timer = timer;
    axis->i2c = i2c;
    axis->en_port = en_port;
    axis->en_pin = en_pin;
    axis->channel_a = channel_a;
    axis->channel_b = channel_b;
    axis->channel_c = channel_c;
    axis->state = SFOC_AXIS_WAITING;

    sfoc_start_pwm(axis);

    /*
     * 先读一次传感器并把当前角度设为目标角。
     * 这样校准结束后不会主动跳到某个未经用户指定的绝对角度。
     */
    if (sfoc_read_angle(axis))
    {
        axis->target_angle = axis->angle_continuous;
    }
}

static void sfoc_update_axis(SFOC_Axis *axis)
{
    uint32_t now;
    float electrical_angle;
    float position_error;
    float uq;

    if (!sfoc_read_angle(axis))
    {
        sfoc_set_enabled(axis, false);
        sfoc_set_neutral_pwm(axis);
        axis->state = SFOC_AXIS_WAITING;
        return;
    }

    if (axis->state == SFOC_AXIS_WAITING)
    {
        /*
         * 用较低电压把转子吸到电角度 0，读取此时机械角并建立电角零位。
         * 这是闭环 FOC 必需的零位校准，校准期间云台可能有小幅动作。
         */
        sfoc_set_enabled(axis, true);
        sfoc_set_torque(axis, SFOC_ALIGNMENT_VOLTAGE, 0.0f);
        axis->alignment_start_tick = HAL_GetTick();
        axis->state = SFOC_AXIS_ALIGNING;
        return;
    }

    if (axis->state == SFOC_AXIS_ALIGNING)
    {
        now = HAL_GetTick();
        if ((now - axis->alignment_start_tick) < SFOC_ALIGNMENT_HOLD_MS)
        {
            sfoc_set_torque(axis, SFOC_ALIGNMENT_VOLTAGE, 0.0f);
            return;
        }

        if (!sfoc_read_angle(axis))
        {
            sfoc_set_enabled(axis, false);
            sfoc_set_neutral_pwm(axis);
            axis->state = SFOC_AXIS_WAITING;
            return;
        }

        axis->zero_electric_angle = sfoc_normalize_angle(
            (float)(SFOC_MOTOR_DIRECTION * SFOC_MOTOR_POLE_PAIRS)
            * axis->angle_raw);
        axis->target_angle = axis->angle_continuous;
        axis->state = SFOC_AXIS_RUNNING;
        sfoc_set_torque(axis, 0.0f, 0.0f);
        return;
    }

    position_error = axis->target_angle - axis->angle_continuous;
    uq = sfoc_limit(SFOC_POSITION_KP * position_error,
                    -SFOC_POSITION_VOLTAGE_LIMIT,
                    SFOC_POSITION_VOLTAGE_LIMIT);
    electrical_angle = sfoc_normalize_angle(
        (float)(SFOC_MOTOR_DIRECTION * SFOC_MOTOR_POLE_PAIRS)
        * axis->angle_raw - axis->zero_electric_angle);
    sfoc_set_torque(axis, uq, electrical_angle);
}

void SimpleFOC_Motor_Init(void)
{
    sfoc_initialize_axis(&g_vertical_axis,
                         &htim1,
                         &hi2c1,
                         TIM1_EN_GPIO_Port,
                         TIM1_EN_Pin,
                         TIM_CHANNEL_1,
                         TIM_CHANNEL_2,
                         TIM_CHANNEL_3);

    sfoc_initialize_axis(&g_horizontal_axis,
                         &htim8,
                         &hi2c3,
                         TIM8_EN_GPIO_Port,
                         TIM8_EN_Pin,
                         TIM_CHANNEL_1,
                         TIM_CHANNEL_2,
                         TIM_CHANNEL_3);
}

void SimpleFOC_Motor_Update(void)
{
    sfoc_update_axis(&g_vertical_axis);
    sfoc_update_axis(&g_horizontal_axis);
}

void SimpleFOC_Motor_SetTargetAngles(float vertical_angle, float horizontal_angle)
{
    g_vertical_axis.target_angle = vertical_angle;
    g_horizontal_axis.target_angle = horizontal_angle;
}

void SimpleFOC_Motor_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        SimpleFOC_Motor_Update();
        osDelay(SFOC_UPDATE_PERIOD_MS);
    }
}
