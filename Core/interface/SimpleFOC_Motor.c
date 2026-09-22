#include "SimpleFOC_Motor.h"

#include "arm_math.h"
#include "tim.h"

/* 圆周率，用于角度换算以及电机空间矢量计算。 */
#define SFOC_PI                    3.14159265358979323846f
/* 弧度转角度的换算系数，CMSIS-DSP 的正弦余弦函数需要角度输入。 */
#define SFOC_RAD_TO_DEG            (180.0f / SFOC_PI)
/* 根号 3 的倒数，用于 Clarke 变换中的 beta 轴计算。 */
#define SFOC_INV_SQRT3             0.57735026918962576451f
/* 0.5，用于计算中点电压、50% 中性占空比和三相投影。 */
#define SFOC_HALF                  0.5f
/* 根号 3 除以 2，用于把 beta 轴量投影到三相电压。 */
#define SFOC_SQRT3_BY_TWO          0.86602540378443864676f

/* 保存一个电机轴所使用的定时器、使能引脚和三相 PWM 通道映射。 */
typedef struct
{
    /* 当前电机轴使用的定时器句柄，负责产生三相 PWM 波形。 */
    TIM_HandleTypeDef *timer;
    /* 当前电机轴的电调使能引脚所在 GPIO 端口。 */
    GPIO_TypeDef *en_port;
    /* 当前电机轴的电调使能引脚编号。 */
    uint16_t en_pin;
    /* A 相绕组对应的定时器 PWM 通道。 */
    uint32_t channel_a;
    /* B 相绕组对应的定时器 PWM 通道。 */
    uint32_t channel_b;
    /* C 相绕组对应的定时器 PWM 通道。 */
    uint32_t channel_c;
} SFOC_AxisHardware;

/* 竖直轴的底层硬件资源映射：TIM1 三相 PWM 和 TIM1_EN 使能引脚。 */
static const SFOC_AxisHardware g_vertical_axis = {
    &htim1,
    TIM1_EN_GPIO_Port,
    TIM1_EN_Pin,
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3
};

/* 水平轴的底层硬件资源映射：TIM8 三相 PWM 和 TIM8_EN 使能引脚。 */
static const SFOC_AxisHardware g_horizontal_axis = {
    &htim8,
    TIM8_EN_GPIO_Port,
    TIM8_EN_Pin,
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3
};

/*
 * 将输入数值限制在指定的最小值和最大值之间。
 * 作用是防止计算出的 PWM 占空比超出 0.0 到 1.0 的有效范围。
 *
 * value：需要限制的原始数值。
 * minimum：允许的最小值。
 * maximum：允许的最大值。
 * 返回值：限制后的数值。
 */
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

/*
 * 根据轴枚举值取得对应的硬件资源映射。
 * 这样上层只需要传入“水平轴”或“竖直轴”，不需要直接操作定时器和 GPIO。
 *
 * axis：需要查询的电机轴。
 * 返回值：对应轴的硬件配置指针。
 */
static const SFOC_AxisHardware *sfoc_get_axis(SimpleFOC_MotorAxis axis)
{
    return axis == SIMPLEFOC_AXIS_HORIZONTAL
               ? &g_horizontal_axis
               : &g_vertical_axis;
}

/*
 * 把三相 PWM 设置为中性状态，即三个通道都输出约 50% 占空比。
 * 50% 是定时器周期的中点，可作为电机不产生有效线电压时的安全初始状态。
 *
 * axis：需要设置中性 PWM 的电机轴硬件配置。
 */
static void sfoc_set_neutral_pwm(const SFOC_AxisHardware *axis)
{
    /* 自动重装载值的一半，对应定时器计数周期的中点比较值。 */
    uint32_t neutral_compare = __HAL_TIM_GET_AUTORELOAD(axis->timer) / 2U;

    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_a, neutral_compare);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_b, neutral_compare);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_c, neutral_compare);
}

/*
 * 关闭指定电机轴的电调输出。
 * 本工程 EN 为低电平有效，因此写入高电平后电调处于禁止工作状态。
 *
 * axis：需要关闭的电机轴硬件配置。
 */
static void sfoc_disable_axis(const SFOC_AxisHardware *axis)
{
    /*
     * 本工程的 EN 是低电平有效：高电平表示电调关闭，低电平表示电调允许工作。
     * 这里统一写高电平，确保初始化失败或用户主动 Disable 时不会驱动电机。
     */
    HAL_GPIO_WritePin(axis->en_port, axis->en_pin, GPIO_PIN_SET);
}

/*
 * 停止指定电机轴的三个 PWM 通道。
 * 该函数用于初始化失败或停止输出时，撤销定时器通道的 PWM 运行状态。
 *
 * axis：需要停止 PWM 的电机轴硬件配置。
 */
static void sfoc_stop_pwm(const SFOC_AxisHardware *axis)
{
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_a);
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_b);
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_c);
}

/*
 * 启动指定电机轴的三个 PWM 通道，并设置安全的初始输出状态。
 * 三个通道全部启动成功后，才会设置 50% 中性占空比并保持电调关闭。
 *
 * axis：需要启动 PWM 的电机轴硬件配置。
 * 返回值：HAL_OK 表示全部通道启动成功，HAL_ERROR 表示至少一个通道启动失败。
 */
static HAL_StatusTypeDef sfoc_start_pwm(const SFOC_AxisHardware *axis)
{
    if (HAL_TIM_PWM_Start(axis->timer, axis->channel_a) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(axis->timer, axis->channel_b) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(axis->timer, axis->channel_c) != HAL_OK)
    {
        return HAL_ERROR;
    }

    sfoc_set_neutral_pwm(axis);
    sfoc_disable_axis(axis);
    return HAL_OK;
}

/*
 * 初始化无刷电机驱动硬件。
 * 该函数只完成两组电机轴的三相 PWM 启动、中性占空比设置和电调关闭，
 * 不会在实际任务中执行 Clarke、Park、逆 Park 或 SVPWM 控制，也不会使能电机。
 *
 * 返回值：HAL_OK 表示上下两个电机轴初始化成功，HAL_ERROR 表示初始化失败。
 */
HAL_StatusTypeDef SimpleFOC_Motor_Init(void)
{
    /*
     * 初始化阶段只做硬件准备：
     * 1. 启动上下轴和左右轴的三相 PWM；
     * 2. 把三相比较值放到周期中点，得到约 50% 占空比；
     * 3. 把两个 EN 写成关闭电平。
     *
     * 这样定时器已经可以输出稳定的中性 PWM，但电调仍不会带动电机。
     */
    sfoc_disable_axis(&g_vertical_axis);
    sfoc_disable_axis(&g_horizontal_axis);

    if (sfoc_start_pwm(&g_vertical_axis) != HAL_OK)
    {
        sfoc_stop_pwm(&g_vertical_axis);
        sfoc_set_neutral_pwm(&g_vertical_axis);
        sfoc_disable_axis(&g_vertical_axis);
        sfoc_disable_axis(&g_horizontal_axis);
        return HAL_ERROR;
    }

    if (sfoc_start_pwm(&g_horizontal_axis) != HAL_OK)
    {
        sfoc_stop_pwm(&g_horizontal_axis);
        sfoc_stop_pwm(&g_vertical_axis);
        sfoc_set_neutral_pwm(&g_vertical_axis);
        sfoc_set_neutral_pwm(&g_horizontal_axis);
        sfoc_disable_axis(&g_vertical_axis);
        sfoc_disable_axis(&g_horizontal_axis);
        return HAL_ERROR;
    }

    return HAL_OK;
}

/*
 * 允许指定电机轴的电调工作。
 * 由于 EN 为低电平有效，写入低电平后电调才会响应 PWM；
 * 调用者应当只在控制任务已经准备好输出数据后调用此函数。
 *
 * axis：需要使能的电机轴。
 */
void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis)
{
    /* 根据轴枚举值取得对应的定时器和 EN 引脚配置。 */
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);

    /* 只有后续真正的控制任务明确调用此函数时，才允许电调工作。 */
    HAL_GPIO_WritePin(hardware->en_port, hardware->en_pin, GPIO_PIN_RESET);
}

/*
 * 禁止指定电机轴的电调工作，并把三相 PWM 恢复到中性状态。
 * 这样可以同时切断电调输出权限和当前三相电压指令。
 *
 * axis：需要关闭的电机轴。
 */
void SimpleFOC_Motor_Disable(SimpleFOC_MotorAxis axis)
{
    /* 根据轴枚举值取得对应的定时器和 EN 引脚配置。 */
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);

    sfoc_disable_axis(hardware);
    sfoc_set_neutral_pwm(hardware);
}

/*
 * Clarke 变换：把三相静止坐标系中的电流 A、B、C 转换为二维 alpha/beta 电流。
 * 三相绕组彼此相差 120 度，而 alpha/beta 只需要两个互相垂直的量即可表示它们，
 * 因此该变换可以减少后续控制计算量。
 *
 * current_a/current_b/current_c：采样得到的 A、B、C 三相电流。
 * output：保存变换结果的 alpha/beta 输出结构体。
 */
void SimpleFOC_Clarke(float current_a,
                      float current_b,
                      float current_c,
                      SimpleFOC_AlphaBeta *output)
{
    /* 三相电流的总和，用于消除三相传感器共同的零点偏移。 */
    float current_sum;

    if (output == NULL)
    {
        return;
    }

    /*
     * 三相电流（A、B、C）相差 120 度。对于理想平衡系统，三相和接近 0，
     * 但这里仍减去三相平均值，使传感器零偏不会直接污染变换结果。
     * alpha、beta 是固定不动的二维坐标，保存了三相量的主要信息。
     */
    current_sum = current_a + current_b + current_c;
    output->alpha = current_a - current_sum / 3.0f;
    output->beta = (current_b - current_c) * SFOC_INV_SQRT3;
}

/*
 * Park 变换：把固定 alpha/beta 坐标系中的电流转换到跟随转子旋转的 d/q 坐标系。
 * d 轴通常与转子磁链方向对齐，q 轴与 d 轴垂直；d、q 分量更适合分别进行电流控制。
 *
 * input：Clarke 变换得到的 alpha/beta 输入电流。
 * electrical_angle：转子当前电角度，单位为弧度。
 * output：保存变换结果的 d/q 输出结构体。
 */
void SimpleFOC_Park(const SimpleFOC_AlphaBeta *input,
                    float electrical_angle,
                    SimpleFOC_DirectQuadrature *output)
{
    /* 电角度的正弦值，用于把 alpha/beta 分量投影到旋转坐标轴。 */
    float sine;
    /* 电角度的余弦值，用于把 alpha/beta 分量投影到旋转坐标轴。 */
    float cosine;
    /* CMSIS-DSP 正弦余弦函数所需的角度制输入值。 */
    float angle_degree;

    if ((input == NULL) || (output == NULL))
    {
        return;
    }

    /*
     * CMSIS-DSP 的 arm_sin_cos_f32 输入单位是角度，而接口角度使用弧度，
     * 所以先转换单位。DSP 库同时计算正弦和余弦，避免自行实现三角函数。
     */
    angle_degree = electrical_angle * SFOC_RAD_TO_DEG;
    arm_sin_cos_f32(angle_degree, &sine, &cosine);

    /*
     * Park 变换相当于让坐标轴跟着转子一起旋转：
     * d 轴对准磁链方向，q 轴与它垂直，q 轴分量通常主要决定电磁转矩。
     * 后续电流环可以分别调节 d、q 两个更直观的量。
     */
    output->d = input->alpha * cosine + input->beta * sine;
    output->q = -input->alpha * sine + input->beta * cosine;
}

/*
 * 逆 Park 变换：把旋转 d/q 坐标系中的电压还原为固定 alpha/beta 坐标系电压。
 * 变换结果会继续交给 SVPWM，转换成三相 PWM 占空比。
 *
 * input：控制器输出的 d/q 电压指令。
 * electrical_angle：转子当前电角度，单位为弧度。
 * output：保存还原结果的 alpha/beta 输出电压。
 */
void SimpleFOC_InvPark(const SimpleFOC_DirectQuadrature *input,
                       float electrical_angle,
                       SimpleFOC_AlphaBeta *output)
{
    /* 电角度的正弦值，用于把 d/q 分量旋转回固定坐标系。 */
    float sine;
    /* 电角度的余弦值，用于把 d/q 分量旋转回固定坐标系。 */
    float cosine;
    /* CMSIS-DSP 正弦余弦函数所需的角度制输入值。 */
    float angle_degree;

    if ((input == NULL) || (output == NULL))
    {
        return;
    }

    angle_degree = electrical_angle * SFOC_RAD_TO_DEG;
    arm_sin_cos_f32(angle_degree, &sine, &cosine);

    /*
     * 逆 Park 把旋转坐标中的 d/q 电压还原到固定 alpha/beta 坐标。
     * 这样得到的二维电压参考才能继续交给 SVPWM 生成三相占空比。
     */
    output->alpha = input->d * cosine - input->q * sine;
    output->beta = input->d * sine + input->q * cosine;
}

/*
 * SVPWM：把 alpha/beta 两相电压参考转换为 A、B、C 三相 PWM 占空比。
 * 先将二维电压投影为三相电压，再加入共同模式电压以提高母线电压利用率，
 * 最后归一化到 0.0 到 1.0，结果可直接换算成定时器比较值。
 *
 * input：逆 Park 变换得到的 alpha/beta 电压参考。
 * supply_voltage：电机驱动器使用的直流母线电压，必须大于 0。
 * duty_a/duty_b/duty_c：输出的 A、B、C 三相占空比指针。
 */
void SimpleFOC_SVPWM(const SimpleFOC_AlphaBeta *input,
                     float supply_voltage,
                     float *duty_a,
                     float *duty_b,
                     float *duty_c)
{
    /* 暂存 alpha/beta 电压投影得到的三相电压参考值。 */
    float phase_voltage[3];
    /* 三相电压参考中的最大值。 */
    float maximum;
    /* 三相电压参考中的最小值。 */
    float minimum;
    /* 加到三相电压上的共同模式偏置，用于扩大可用调制范围。 */
    float common_mode;
    /* 最大三相电压在 phase_voltage 数组中的下标，DSP 函数会同时输出。 */
    uint32_t maximum_index;
    /* 最小三相电压在 phase_voltage 数组中的下标，DSP 函数会同时输出。 */
    uint32_t minimum_index;

    if (duty_a != NULL)
    {
        *duty_a = SFOC_HALF;
    }
    if (duty_b != NULL)
    {
        *duty_b = SFOC_HALF;
    }
    if (duty_c != NULL)
    {
        *duty_c = SFOC_HALF;
    }

    if ((input == NULL) || (supply_voltage <= 0.0f)
        || (duty_a == NULL) || (duty_b == NULL) || (duty_c == NULL))
    {
        return;
    }

    /*
     * alpha/beta 是二维电压，先投影成三相电压参考 Ua、Ub、Uc。
     * 三个参考值的共同偏置不会改变线电压，却可以更充分利用直流母线电压。
     */
    phase_voltage[0] = input->alpha;
    phase_voltage[1] = -SFOC_HALF * input->alpha
                       + SFOC_SQRT3_BY_TWO * input->beta;
    phase_voltage[2] = -SFOC_HALF * input->alpha
                       - SFOC_SQRT3_BY_TWO * input->beta;

    /* CMSIS-DSP 负责三项数据的最大值和最小值，减少手写比较分支。 */
    arm_max_f32(phase_voltage, 3U, &maximum, &maximum_index);
    arm_min_f32(phase_voltage, 3U, &minimum, &minimum_index);
    common_mode = -SFOC_HALF * (maximum + minimum);

    /*
     * 把电压参考除以母线电压并加上 50% 中点偏置：
     * 0 V 对应 0.5，占空比范围被限制在 0 到 1，最终可直接写入定时器 CCR。
     */
    *duty_a = sfoc_limit(SFOC_HALF
                             + (phase_voltage[0] + common_mode) / supply_voltage,
                         0.0f,
                         1.0f);
    *duty_b = sfoc_limit(SFOC_HALF
                             + (phase_voltage[1] + common_mode) / supply_voltage,
                         0.0f,
                         1.0f);
    *duty_c = sfoc_limit(SFOC_HALF
                             + (phase_voltage[2] + common_mode) / supply_voltage,
                         0.0f,
                         1.0f);
}
