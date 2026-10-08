#include "SimpleFOC_Motor.h"

#include "arm_math.h"
#include "tim.h"

/* 圆周率，用于角度换算以及电机空间矢量计算。 */
#define SFOC_PI                    3.14159265358979323846f
/* 当前电机驱动母线电压，供开环电压指令归一化到 PWM 占空比。 */
#define SFOC_SUPPLY_VOLTAGE        12.0f
/* 弧度转角度的换算系数，CMSIS-DSP 的正弦余弦函数需要角度输入。 */
#define SFOC_RAD_TO_DEG            (180.0f / SFOC_PI)
/* 角度转弧度的换算系数，用于把用户配置的偏移量转换为计算角度。 */
#define SFOC_DEG_TO_RAD            (SFOC_PI / 180.0f)
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

/* 保存一个电机轴的开环状态，角度由软件虚拟推进，不依赖编码器反馈。 */
typedef struct
{
    /* 本次开环运行使用的参数副本。 */
    SimpleFOC_OpenLoopConfig config;
    /* 当前虚拟电角度，单位为弧度，始终限制在 0 到 2π 之间。 */
    float electrical_angle;
    /* 当前实际使用的机械转速，启动时从 0 逐渐增加到目标值。 */
    float current_speed_rps;
    /* 上一次更新开环输出时的 HAL 时间戳。 */
    uint32_t last_update_tick;
    /* 1 表示已经启动，0 表示未启动。 */
    uint8_t active;
} SFOC_OpenLoopState;

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

/* 两个轴各自保存一份开环状态，当前任务只会启动 TIM8 对应的水平轴。 */
static SFOC_OpenLoopState g_open_loop_state[2];

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
 * 将任意电角度限制到 0 到 2π 之间，便于后续三角函数计算和观察调试。
 */
static float sfoc_normalize_angle(float angle)
{
    while (angle >= 2.0f * SFOC_PI)
    {
        angle -= 2.0f * SFOC_PI;
    }
    while (angle < 0.0f)
    {
        angle += 2.0f * SFOC_PI;
    }
    return angle;
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
 * 把轴枚举转换成开环状态数组下标。
 * 当前枚举从 0 开始连续排列，因此可以直接使用枚举值作为下标。
 */
static uint32_t sfoc_get_axis_index(SimpleFOC_MotorAxis axis)
{
    return axis == SIMPLEFOC_AXIS_HORIZONTAL ? 1U : 0U;
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
 * 把 alpha/beta 电压参考转换成三相 PWM，并写入指定轴的三个比较寄存器。
 * 开环控制时 d 轴电压固定为 0，q 轴电压由调用者给出，电角度由软件推进。
 */
static void sfoc_set_phase_voltage(const SFOC_AxisHardware *axis,
                                   float voltage_q,
                                   float electrical_angle)
{
    SimpleFOC_DirectQuadrature direct_quadrature = {0.0f, voltage_q}; /* d 轴固定为零，q 轴使用调用者给定的电压。 */
    SimpleFOC_AlphaBeta alpha_beta = {0.0f, 0.0f};                    /* 保存逆 Park 变换得到的 alpha/beta 两相电压。 */
    float duty_a;                                                      /* 保存 A 相 PWM 占空比，范围为 0.0 到 1.0。 */
    float duty_b;                                                      /* 保存 B 相 PWM 占空比，范围为 0.0 到 1.0。 */
    float duty_c;                                                      /* 保存 C 相 PWM 占空比，范围为 0.0 到 1.0。 */
    uint32_t period;                                                   /* 保存定时器完整 PWM 周期对应的计数值。 */

    /* 先做逆 Park，再由 SVPWM 生成三相占空比。 */
    SimpleFOC_InvPark(&direct_quadrature, electrical_angle, &alpha_beta);
    SimpleFOC_SVPWM(&alpha_beta,
                    SFOC_SUPPLY_VOLTAGE,
                    &duty_a,
                    &duty_b,
                    &duty_c);

    /*
     * ARR 表示最大计数值，因此一个完整 PWM 周期的计数数量是 ARR+1。
     * 将 0.0 到 1.0 的占空比转换为比较值，并在最后再次保护边界。
     */
    period = __HAL_TIM_GET_AUTORELOAD(axis->timer) + 1U;
    __HAL_TIM_SET_COMPARE(axis->timer,
                          axis->channel_a,
                          (uint32_t)(duty_a * (float)period));
    __HAL_TIM_SET_COMPARE(axis->timer,
                          axis->channel_b,
                          (uint32_t)(duty_b * (float)period));
    __HAL_TIM_SET_COMPARE(axis->timer,
                          axis->channel_c,
                          (uint32_t)(duty_c * (float)period));
}

/*
 * 关闭指定电机轴的电调输出。
 * 本工程 EN 为高电平有效，因此写入低电平后电调处于禁止工作状态。
 *
 * axis：需要关闭的电机轴硬件配置。
 */
static void sfoc_disable_axis(const SFOC_AxisHardware *axis)
{
    /*
     * 本工程的 EN 是高电平有效：低电平表示电调关闭，高电平表示电调允许工作。
     * 这里统一写低电平，确保初始化失败或用户主动 Disable 时不会驱动电机。
     */
    HAL_GPIO_WritePin(axis->en_port, axis->en_pin, GPIO_PIN_RESET);
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
 * 启动指定轴的开环电压驱动。
 *
 * 开环控制的完整思路是：
 * 1. 不读取 AS5600，也不读取 ADC 电流；
 * 2. 按目标机械转速计算电角速度；
 * 3. 在每次任务更新时推进虚拟电角度；
 * 4. 使用 q 轴电压和虚拟电角度生成三相 SVPWM；
 * 5. 通过 EN 引脚让电调开始响应 PWM。
 *
 * 由于没有位置反馈，电机如果负载过大、启动电压不足或相序不正确，
 * 可能出现抖动、失步或实际转向与软件方向相反。
 */
HAL_StatusTypeDef SimpleFOC_Motor_OpenLoopStart(SimpleFOC_MotorAxis axis,
                                                const SimpleFOC_OpenLoopConfig *config)
{
    const SFOC_AxisHardware *hardware;
    SFOC_OpenLoopState *state;
    uint32_t state_index;

    if ((config == NULL)
        || (config->mechanical_speed_rps < 0.0f)
        || (config->voltage < 0.0f)
        || (config->pole_pairs == 0U)
        || ((config->direction != 1) && (config->direction != -1)))
    {
        return HAL_ERROR;
    }

    hardware = sfoc_get_axis(axis);                     /* 根据轴编号获取对应的定时器、PWM 通道和使能引脚。 */
    state_index = sfoc_get_axis_index(axis);            /* 将轴编号转换为开环状态数组下标。 */
    state = &g_open_loop_state[state_index];            /* 获取当前轴专属的开环运行状态。 */
    state->config = *config;                             /* 保存调用者传入的开环参数副本。 */
    state->electrical_angle = 0.0f;                      /* 从零电角度开始生成旋转磁场。 */
    state->current_speed_rps = 0.0f;                     /* 从静止开始，后续按斜坡逐步加速。 */
    state->last_update_tick = HAL_GetTick();             /* 记录首次更新的时间基准。 */
    state->active = 1U;                                  /* 标记当前轴已启动，允许周期任务刷新 PWM。 */

    /* 先写入初始电压，再拉高 EN，避免电调使能时三相输出尚未准备好。 */
    /*
     * 虚拟角度从 0 度开始，但输出磁场先加上用户配置的电角度偏移量。
     * 这样偏移量只改变磁场相位，不会改变后续电角速度。
     */
    sfoc_set_phase_voltage(hardware,
                           0.0f,
                           sfoc_normalize_angle(state->electrical_angle
                                                + state->config.electrical_angle_offset_deg
                                                  * SFOC_DEG_TO_RAD));
    SimpleFOC_Motor_Enable(axis);
    return HAL_OK;
}

/*
 * 更新指定轴的开环电角度和三相 PWM。
 * 该函数不阻塞，适合由 FreeRTOS 电机任务周期性调用。
 */
void SimpleFOC_Motor_OpenLoopUpdate(SimpleFOC_MotorAxis axis)
{
    const SFOC_AxisHardware *hardware;
    SFOC_OpenLoopState *state;
    uint32_t state_index;
    uint32_t now_tick;
    uint32_t elapsed_ms;
    float elapsed_s;
    float electrical_speed;

    state_index = sfoc_get_axis_index(axis);
    state = &g_open_loop_state[state_index];
    if (state->active == 0U)
    {
        return;
    }

    hardware = sfoc_get_axis(axis);
    now_tick = HAL_GetTick();
    elapsed_ms = now_tick - state->last_update_tick;
    state->last_update_tick = now_tick;

    /* 任务被调度延迟时限制单次角度跳变，避免瞬间跨过大量电角度。 */
    if (elapsed_ms > 20U)
    {
        elapsed_ms = 20U;
    }
    elapsed_s = (float)elapsed_ms * 0.001f;

    /* 按设定的斜坡时间逐渐增加速度，降低上电瞬间的机械冲击。 */
    if (state->config.ramp_time_ms == 0U)
    {
        state->current_speed_rps = state->config.mechanical_speed_rps;
    }
    else
    {
        float speed_step = state->config.mechanical_speed_rps
                           * (float)elapsed_ms
                           / (float)state->config.ramp_time_ms;
        state->current_speed_rps += speed_step;
        if (state->current_speed_rps > state->config.mechanical_speed_rps)
        {
            state->current_speed_rps = state->config.mechanical_speed_rps;
        }
    }

    /*
     * 机械转速乘以极对数得到电转速：
     * electrical_speed = mechanical_speed × pole_pairs × 2π。
     */
    electrical_speed = state->current_speed_rps
                       * (float)state->config.pole_pairs
                       * 2.0f
                       * SFOC_PI;
    state->electrical_angle += (float)state->config.direction
                               * electrical_speed
                               * elapsed_s;

    /* 角度回绕，避免长时间运行后浮点数不断增大。 */
    while (state->electrical_angle >= 2.0f * SFOC_PI)
    {
        state->electrical_angle -= 2.0f * SFOC_PI;
    }
    while (state->electrical_angle < 0.0f)
    {
        state->electrical_angle += 2.0f * SFOC_PI;
    }

    sfoc_set_phase_voltage(hardware,
                           state->config.voltage,
                           sfoc_normalize_angle(state->electrical_angle
                                                + state->config.electrical_angle_offset_deg
                                                  * SFOC_DEG_TO_RAD));
}

/*
 * 停止指定轴的开环运行。
 * 先关闭 EN，再把三相输出恢复到 50% 中性状态，降低停机时的意外驱动风险。
 */
void SimpleFOC_Motor_OpenLoopStop(SimpleFOC_MotorAxis axis)
{
    uint32_t state_index = sfoc_get_axis_index(axis);

    g_open_loop_state[state_index].active = 0U;
    g_open_loop_state[state_index].current_speed_rps = 0.0f;
    SimpleFOC_Motor_Disable(axis);
}

/*
 * 允许指定电机轴的电调工作。
 * 由于 EN 为高电平有效，写入高电平后电调才会响应 PWM；
 * 调用者应当只在控制任务已经准备好输出数据后调用此函数。
 *
 * axis：需要使能的电机轴。
 */
void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis)
{
    /* 根据轴枚举值取得对应的定时器和 EN 引脚配置。 */
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);

    /* 只有后续真正的控制任务明确调用此函数时，才允许电调工作。 */
    HAL_GPIO_WritePin(hardware->en_port, hardware->en_pin, GPIO_PIN_SET);
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
    sfoc_set_neutral_pwm(hardware);//设置50%占空比
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

    //初始化必须保证三者都在一个安全值
    //三相都是 50% 时，三相平均电压相同，电机绕组之间没有有效线电压
    //因此通常不会产生旋转转矩
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
     * Ua = Uα
     * Ub = -½Uα + (√3 / 2)Uβ
     * Uc = -½Uα - (√3 / 2)Uβ
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
