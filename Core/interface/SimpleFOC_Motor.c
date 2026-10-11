#include "SimpleFOC_Motor.h"

#include "arm_math.h"
#include "tim.h"

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
