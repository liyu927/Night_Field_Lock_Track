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

/* FOC 电压变换只使用弧度；CMSIS-DSP 的 arm_sin_cos_f32 输入改用角度。 */
#define SFOC_PI                    3.14159265358979323846f
#define SFOC_TWO_PI                (2.0f * SFOC_PI)
#define SFOC_RAD_TO_DEG            (180.0f / SFOC_PI)
#define SFOC_HALF                  0.5f
#define SFOC_SQRT3_BY_TWO          0.86602540378443864676f      //  (根号3)/2



//************************************************************************************************

#define MOTOR_PI                         3.14159265358979323846f
#define MOTOR_TWO_PI                     (2.0f * MOTOR_PI)
#define MOTOR_TASK_PERIOD_MS             5U  /* 闭环任务的名义周期；实际 dt 由系统节拍计算。 */

/* 两个电机的已确认硬件参数。 */
#define MOTOR_POLE_PAIRS                 7U     //极对数
#define MOTOR_SUPPLY_VOLTAGE             12.0f  //母线电压
#define MOTOR_DIRECTION_CLOCKWISE        1      //方向系数（顺逆相反时把这个取反）

/* 自动电角零位校准参数：低 d 轴电压锁定转子后读取 AS5600。 */
#define MOTOR_ALIGNMENT_VOLTAGE          1.0f
#define MOTOR_ALIGNMENT_TIME_MS          800U
#define MOTOR_ALIGNMENT_ANGLE_RAD        0.0f
/* 角度源改为 0x0E 后更新记录版本，旧偏移量需重新测量。 */
#define MOTOR_OFFSET_SIGNATURE          0x32464F43UL//这个项目用 MOTOR_OFFSET_SIGNATURE 充当这张标签。程序把标签与电角偏移一起存入 W25Q64；以后上电读取时，只有标签与当前程序要求的值相同，才继续考虑使用该偏移
/* 配置字把极对数放在高位、方向放在低 8 位，防止误加载不匹配的校准记录。 */
#define MOTOR_OFFSET_CONFIG             ((MOTOR_POLE_PAIRS << 8) | (MOTOR_DIRECTION_CLOCKWISE & 0xFF))
#define MOTOR_OFFSET_TIM1_VALID         0x01U  /* TIM1 偏移量已校准并可恢复。 */
#define MOTOR_OFFSET_TIM8_VALID         0x02U  /* TIM8 偏移量已校准并可恢复。 */

/* 实测确定的双闭环参数；力矩环不测电流，只限制 q 轴电压。 */
#define MOTOR_POSITION_KP                2.5f   /* 位置误差转为目标角速度的比例系数，单位 1/s。 */
#define MOTOR_MAX_SPEED_DPS             180.0f  /* 目标机械角速度上限，单位度/秒，不保证实际能达到。 */
#define MOTOR_SPEED_KP                   0.18f  /* 速度误差生成 q 轴电压的比例系数。 */
#define MOTOR_SPEED_KI                   4.47f  /* 速度误差的积分系数，用于减小稳态速度误差。 */
#define MOTOR_SPEED_FILTER_S             0.02f  /* 速度反馈低通滤波时间常数，单位秒；越大越平稳但滞后越大。 */
#define MOTOR_MAX_Q_VOLTAGE              3.5f   /* q 轴电压指令的绝对值上限，单位伏；不是电流上限。 */
#define MOTOR_Q_SLEW_VPS                 30.0f   /* q 轴电压指令每秒允许变化的最大值，单位伏/秒。 */
#define MOTOR_MAX_STEP_DEG               8.0f   /* 相邻两次传感器读数允许的最大角度变化，超出则故障停机。 */
#define MOTOR_MAX_OVERSHOOT_DEG         10.0f   /* 沿指令方向越过目标角度的容许量，超出则故障停机。 */
#define MOTOR_MAX_COMMAND_DEG          360.0f   /* 单次相对转角指令的绝对值上限，超出则拒绝指令。 */
#define MOTOR_MAX_SPEED_RPS             (MOTOR_MAX_SPEED_DPS / 360.0f)

/* 单轴闭环运行态：传感器给出单圈角，软件另行累计跨圈位置。 */
typedef struct
{
    SimpleFOC_MotorAxis axis;       /* 物理轴编号，决定 PWM 与 EN 的硬件映射。 */
    AS5600_Handle_t encoder;        /* 该轴的 I2C 总线与编码器地址。 */
    PID_Controller_t position_pid; /* 位置误差输入，目标角速度输出。 */
    PID_Controller_t speed_pid;    /* 角速度误差输入，q 轴电压输出。 */
    float electrical_offset;       /* 机械角转换为电角度时的校准偏移，单位 rad。 */
    float raw_angle;               /* 上一次 AS5600 单圈机械角，范围 [0, 2π)。 */
    float mechanical_origin;       /* 本次上电的位置零点，单位 rad。 */
    float mechanical_position;     /* 跨圈累计的机械角，单位 rad。 */
    float mechanical_speed;        /* 低通后的机械角速度，单位 rad/s。 */
    float applied_q_voltage;      /* 上周期实际下发的 q 轴电压，用于斜率限制。 */
    uint8_t feedback_initialized;  /* 首次采样已建立基准，后续才能计算增量。 */
} MotorControl_t;

/* TIM1 由本任务持续控制；TIM8 当前仅准备校准状态，不在本任务内运行闭环。 */
static MotorControl_t g_tim1_control;
static MotorControl_t g_tim8_control;
/* TIM8 只记住偏移量是否有效；上电加载不会启动 PWM。 */
static uint8_t g_tim8_offset_valid;
/* 两轴电角零位共用一份 Flash 记录，由 W25Q64 模块负责双扇区备份。 */
typedef struct
{
    uint32_t signature;    /* 当前记录格式和角度源的识别值。 */
    uint32_t config;       /* 极对数与方向，避免误用其他电机配置的偏移量。 */
    uint32_t valid_flags;  /* 位标志分别表示 TIM1、TIM8 偏移是否有效。 */
    float tim1_offset;     /* 竖直轴电角偏移，单位 rad。 */
    float tim8_offset;     /* 水平轴电角偏移，单位 rad。 */
} MotorOffsetRecord;

/* TIM1 位置命令与故障状态，在任务和外部命令调用者之间共享。 */
typedef struct
{
    float target_deg;  /* 相对最近一条位置命令起点的目标角度，单位度。 */
    float actual_deg;  /* 相对最近一条位置命令起点的反馈角度，单位度。 */
    float target_speed_rps; /* 速度命令，单位圈/秒。 */
    SimpleFOC_ControlMode mode; /* 当前只执行位置环或速度环中的一种。 */
    uint8_t ready;     /* TIM1 编码器、电角偏移与存储准备完成。 */
    uint8_t fault;     /* 传感器、存储或越界故障已锁存。 */
} MotorPositionState;

static MotorPositionState g_motor_state;
static uint32_t g_command_sequence; /* 每条新命令更新一次，供控制任务重设位置零点。 */

//*******************************************************************************************************



/* 底层硬件映射：只描述三相输出和使能脚，不保存控制环状态。 */
typedef struct
{
    TIM_HandleTypeDef *timer;  /* 产生三路 PWM 的定时器。 */
    GPIO_TypeDef *en_port;     /* 电调使能 GPIO 端口。 */
    uint16_t en_pin;           /* 电调使能 GPIO 引脚，高电平使能。 */
    uint32_t channel_a;        /* A 相 PWM 通道。 */
    uint32_t channel_b;        /* B 相 PWM 通道。 */
    uint32_t channel_c;        /* C 相 PWM 通道。 */
} SFOC_AxisHardware;

static const SFOC_AxisHardware g_vertical_axis = {
    &htim1, TIM1_EN_GPIO_Port, TIM1_EN_Pin,
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
};

static const SFOC_AxisHardware g_horizontal_axis = {
    &htim8, TIM8_EN_GPIO_Port, TIM8_EN_Pin,
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
};

/* 对占空比等连续量做闭区间裁剪；不改变输入单位。 */
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

/* 把任意弧度角折回 [0, 2π)，便于三角函数和电角零位计算。 */
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

/* 按轴编号返回固定的定时器/EN 映射；非水平枚举会落到竖直轴。 */
static const SFOC_AxisHardware *sfoc_get_axis(SimpleFOC_MotorAxis axis)
{
    return axis == SIMPLEFOC_AXIS_HORIZONTAL ? &g_horizontal_axis : &g_vertical_axis;
}

/* 三相均置为计数周期一半；关闭 EN 时保留确定的 PWM 比较值。 */
static void sfoc_set_neutral_pwm(const SFOC_AxisHardware *axis)
{
    /* 自动重装值 ARR 对应计数上限，取一半作为约 50% 中性占空比。 */
    uint32_t neutral_compare = __HAL_TIM_GET_AUTORELOAD(axis->timer) / 2U;

    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_a, neutral_compare);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_b, neutral_compare);
    __HAL_TIM_SET_COMPARE(axis->timer, axis->channel_c, neutral_compare);
}

/* 初始化失败时关闭该轴三路 PWM，避免只启动了部分相。 */
static void sfoc_stop_pwm(const SFOC_AxisHardware *axis)
{
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_a);
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_b);
    (void)HAL_TIM_PWM_Stop(axis->timer, axis->channel_c);
}

/* 启动三相 PWM 后立即写中性比较值并关闭 EN；任一相启动失败则上报。 */
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

/* 两轴 PWM 初始化入口：先断 EN，再逐轴启动三相输出；失败则撤销已启动通道。 */
HAL_StatusTypeDef SimpleFOC_Motor_Init(void)
{
    /* 在启动定时器前先关闭两轴 EN，避免初始化窗口误使能。 */
    HAL_GPIO_WritePin(g_vertical_axis.en_port, g_vertical_axis.en_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_horizontal_axis.en_port, g_horizontal_axis.en_pin, GPIO_PIN_RESET);

    if (sfoc_start_pwm(&g_vertical_axis) != HAL_OK)//写50%，关EN
    {
        //失败处理
        sfoc_stop_pwm(&g_vertical_axis);
        return HAL_ERROR;
    }
    if (sfoc_start_pwm(&g_horizontal_axis) != HAL_OK)
    {
        /* 第二轴失败时撤销第一轴已启动的三路 PWM。 */
        sfoc_stop_pwm(&g_vertical_axis);
        sfoc_stop_pwm(&g_horizontal_axis);
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* 只拉高指定轴使能脚；不会修改三相占空比，也不会检查控制环状态。 */
void SimpleFOC_Motor_Enable(SimpleFOC_MotorAxis axis)
{
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);
    HAL_GPIO_WritePin(hardware->en_port, hardware->en_pin, GPIO_PIN_SET);
}

/* 先关闭指定轴使能，再把三相 PWM 设为中性比较值。 */
void SimpleFOC_Motor_Disable(SimpleFOC_MotorAxis axis)
{
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);//判断是哪个电机
    /* 先撤使能，再复位比较值，防止更新占空比时继续输出。 */
    HAL_GPIO_WritePin(hardware->en_port, hardware->en_pin, GPIO_PIN_RESET);
    sfoc_set_neutral_pwm(hardware);
}

/* d/q 电压到三相比较值的完整路径；本函数不拉高 EN。 */
void SimpleFOC_Motor_SetPhaseVoltage(SimpleFOC_MotorAxis axis,
                                     float voltage_d,
                                     float voltage_q,
                                     float electrical_angle,
                                     float supply_voltage)
{
    const SFOC_AxisHardware *hardware = sfoc_get_axis(axis);//判断是哪个电机
    SimpleFOC_DirectQuadrature direct_quadrature = {voltage_d, voltage_q};
    SimpleFOC_AlphaBeta alpha_beta = {0.0f, 0.0f};
    float duty_a;
    float duty_b;
    float duty_c;
    uint32_t period;

    /* 无有效母线电压时无法换算占空比，直接关闭本轴。 */
    if (supply_voltage <= 0.0f)
    {
        SimpleFOC_Motor_Disable(axis);
        return;
    }

    /* 先把 d/q 电压转到定子坐标，再由 SVPWM 得到三相占空比。 */
    SimpleFOC_InvPark(&direct_quadrature,
                       sfoc_normalize_angle(electrical_angle),
                       &alpha_beta);
    SimpleFOC_SVPWM(&alpha_beta, supply_voltage, &duty_a, &duty_b, &duty_c);

    /* ARR+1 是计数周期长度；三个 [0,1] 占空比分别换算为比较寄存器值。 */
    period = __HAL_TIM_GET_AUTORELOAD(hardware->timer) + 1U;
    __HAL_TIM_SET_COMPARE(hardware->timer, hardware->channel_a,
                          (uint32_t)(duty_a * (float)period));
    __HAL_TIM_SET_COMPARE(hardware->timer, hardware->channel_b,
                          (uint32_t)(duty_b * (float)period));
    __HAL_TIM_SET_COMPARE(hardware->timer, hardware->channel_c,
                          (uint32_t)(duty_c * (float)period));
}

/* 将单圈机械角乘极对数、叠加校准零位，形成当前 FOC 换相电角度。 */
float SimpleFOC_Motor_CalculateElectricalAngle(float mechanical_angle,//机械角
                                               float electrical_offset,//电角零位（偏移量）
                                               uint8_t pole_pairs,//极对数
                                               int8_t direction)//方向
{
    if ((pole_pairs == 0U) || ((direction != 1) && (direction != -1)))
    {
        return 0.0f;
    }

    /* 机械转一圈，磁极对使电角转 pole_pairs 圈；方向决定角度正负。 */
    return sfoc_normalize_angle((float)direction * (float)pole_pairs
                                * mechanical_angle + electrical_offset);
}

/* 锁转子时用已知磁场角减去机械角对应的电角，反推出零位偏移。 */
float SimpleFOC_Motor_CalculateElectricalOffset(float mechanical_angle,
                                                float alignment_angle,//锁定时施加的固定磁场电角度（本工程 0.0f）。
                                                uint8_t pole_pairs,
                                                int8_t direction)
{
    if ((pole_pairs == 0U) || ((direction != 1) && (direction != -1)))
    {
        return 0.0f;
    }

    /* 已知锁定磁场角和实测机械角，移项求出日后复用的电角零位。 */
    //θe(alignment_angle) = dir·pp·θm + offset移项得 offset = alignment_angle − dir·pp·θm。
    return sfoc_normalize_angle(alignment_angle
                                - (float)direction * (float)pole_pairs
                                * mechanical_angle);
}

/* 逆 Park 仅做数学变换，不读取硬件，也不执行电压限幅。 */
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

    /* CMSIS-DSP 接口接收角度制，故先把调用者提供的弧度换为度。 */
    arm_sin_cos_f32(electrical_angle * SFOC_RAD_TO_DEG, &sine, &cosine);
    /* 逆 Park：d 轴沿磁场、q 轴垂直磁场，投影回静止 α/β 轴。 */
    output->alpha = input->d * cosine - input->q * sine;
    output->beta = input->d * sine + input->q * cosine;
}

/* 使用公共模电压平移三相参考值，输出各自 0～1 的 PWM 占空比。 */
void SimpleFOC_SVPWM(const SimpleFOC_AlphaBeta *input,
                     float supply_voltage,
                     float *duty_a,
                     float *duty_b,
                     float *duty_c)
{
    float phase_voltage[3];//三相参考电压（范围-6~6V）
    float maximum;//三相中的最大值
    float minimum;//三相中的最小值
    float common_mode;//公共偏置（零序电压）
    uint32_t maximum_index;
    uint32_t minimum_index;

    if ((input == 0) || (duty_a == 0) || (duty_b == 0) || (duty_c == 0)
        || (supply_voltage <= 0.0f))//这是指针为空，不是数值为0
    {
        return;
    }

    /* 逆 Clarke：由两轴电压重建相隔 120° 的三相参考电压。 */
    phase_voltage[0] = input->alpha; /* A 相位于 α 轴。 */
    /* B 相相对 A 相旋转 120°，因此 α 系数为 -1/2、β 系数为 +√3/2。 */
    phase_voltage[1] = -SFOC_HALF * input->alpha
                       + SFOC_SQRT3_BY_TWO * input->beta;
    /* C 相相对 A 相旋转 -120°，β 分量与 B 相相反。 */
    phase_voltage[2] = -SFOC_HALF * input->alpha
                       - SFOC_SQRT3_BY_TWO * input->beta;
    /* 公共偏置取最高、最低相电压中点的相反数，平移三相而不改变线电压。 */
    arm_max_f32(phase_voltage, 3U, &maximum, &maximum_index);
    arm_min_f32(phase_voltage, 3U, &minimum, &minimum_index);
    common_mode = -SFOC_HALF * (maximum + minimum);

    /* 以 50% 为零电压中心，再按母线电压归一化；限幅避免非法比较值。 */
    /* A 相参考电压除以母线电压后叠加 50% 中心占空比。 */
    *duty_a = sfoc_limit(SFOC_HALF + (phase_voltage[0] + common_mode) / supply_voltage,
                         0.0f, 1.0f);
    /* B 相使用同一个公共偏置，保持三相线电压关系。 */
    *duty_b = sfoc_limit(SFOC_HALF + (phase_voltage[1] + common_mode) / supply_voltage,
                         0.0f, 1.0f);
    /* C 相同样限制在硬件可表示的 0～100% 占空比。 */
    *duty_c = sfoc_limit(SFOC_HALF + (phase_voltage[2] + common_mode) / supply_voltage,
                         0.0f, 1.0f);
}


/* 接收 TIM1 相对位置指令；每条新命令以最近一次反馈位置为零点。 */
uint8_t MotorPosition_SetTargetDegrees(float delta_deg)
{
    /* 拒绝 NaN/无穷和超过单次转角上限的命令，避免目标污染控制环。 */
    if (!isfinite(delta_deg) || fabsf(delta_deg) > MOTOR_MAX_COMMAND_DEG)
        return 0U;

    /* 命令调用者与电机任务共享实际位置、目标和故障标志，整组读写必须原子化。 */
    taskENTER_CRITICAL();
    if (g_motor_state.ready == 0U || g_motor_state.fault != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    g_motor_state.actual_deg = 0.0f;
    g_motor_state.target_deg = delta_deg;
    g_motor_state.mode = SIMPLEFOC_CONTROL_POSITION;
    ++g_command_sequence;
    taskEXIT_CRITICAL();
    return 1U;
}

/* 接收 TIM1 速度命令；只设置速度环目标，不产生位置目标。 */
uint8_t MotorSpeed_SetTargetSpeed(float speed_rps)
{
    if (!isfinite(speed_rps) || fabsf(speed_rps) > MOTOR_MAX_SPEED_RPS)
        return 0U;

    taskENTER_CRITICAL();
    if (g_motor_state.ready == 0U || g_motor_state.fault != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    g_motor_state.target_speed_rps = speed_rps;
    g_motor_state.mode = SIMPLEFOC_CONTROL_SPEED;
    ++g_command_sequence;
    taskEXIT_CRITICAL();
    return 1U;
}

/* 将相邻两次绝对角度的差值限制到 -π 至 π，跨越 0 度时保持连续。 */
static float motor_angle_difference(float current, float previous)
{
    float difference = current - previous;

    /* 例如读数从 359° 跳到 1°，应视为正向 2°，而不是反向 358°。 */
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

    /* 位置环的输入单位 rad，输出限定为 rad/s；本环只用比例项。 */
    PID_Init(&control->position_pid,
             MOTOR_POSITION_KP, 0.0f, 0.0f,
             -MOTOR_MAX_SPEED_DPS * MOTOR_PI / 180.0f,
             MOTOR_MAX_SPEED_DPS * MOTOR_PI / 180.0f);
    /* 速度环的输入单位 rad/s，输出限定为 q 轴电压 V；无相电流反馈。 */
    PID_Init(&control->speed_pid,
             MOTOR_SPEED_KP, MOTOR_SPEED_KI, 0.0f,
             -MOTOR_MAX_Q_VOLTAGE,
             MOTOR_MAX_Q_VOLTAGE);
}

/* 读取 AS5600 并更新连续机械位置与机械角速度。 */
static HAL_StatusTypeDef motor_update_feedback(MotorControl_t *control,
                                                float interval_s)//距上次采样的时间间隔（秒）
{
    float current_angle;//本次读到的单圈机械角
    float angle_increment;//解缠后的角度增量

    if (AS5600_ReadMechanicalAngle(&control->encoder, &current_angle) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 首帧只建立角度基准，不计算速度，避免把初始绝对角误判为位移。 */
    if (control->feedback_initialized == 0U)
    {
        control->raw_angle = current_angle;
        control->mechanical_position = current_angle;
        control->mechanical_speed = 0.0f;
        control->feedback_initialized = 1U;
        return HAL_OK;
    }

    /* 单圈角先解缠，再检查相邻采样是否出现不可信的大跳变。 */
    angle_increment = motor_angle_difference(current_angle, control->raw_angle);
    if (fabsf(angle_increment) > MOTOR_MAX_STEP_DEG * MOTOR_PI / 180.0f)//不能大于8度
        return HAL_ERROR;
    control->raw_angle = current_angle;
    /* 累加后得到可跨越 0/2π 的连续机械位置。 */
    control->mechanical_position += angle_increment;
    /* 一阶低通：新速度估计=(本次角度增量/dt)，系数=dt/(时间常数+dt)。 */
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

    /* 固定电角度只输出 d 轴电压，使转子停在已知磁场方向。 */
    SimpleFOC_Motor_SetPhaseVoltage(control->axis,
                                    MOTOR_ALIGNMENT_VOLTAGE,
                                    0.0f,
                                    MOTOR_ALIGNMENT_ANGLE_RAD,
                                    MOTOR_SUPPLY_VOLTAGE);
    /* 对齐时间内电机会实际转动；等待稳定后才读取编码器。 */
    SimpleFOC_Motor_Enable(control->axis);
    osDelay(MOTOR_ALIGNMENT_TIME_MS);

    if (AS5600_ReadMechanicalAngle(&control->encoder, &mechanical_angle) != HAL_OK)
    {
        SimpleFOC_Motor_Disable(control->axis);
        return HAL_ERROR;
    }

    /* 把锁定电角和机械角关联起来，保存供以后每个控制周期使用。 */
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

    /* 主轴尚未完成上电准备或已有故障时，不启动第二轴校准。 */
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
    /* 重读整包，避免只保存 TIM8 时丢失原来有效的 TIM1 偏移。 */
    status = W25Q64_LoadParameters(&record, sizeof(record));
    if (status != HAL_OK) return status;
    if (record.signature != MOTOR_OFFSET_SIGNATURE
        || record.config != MOTOR_OFFSET_CONFIG
        || (record.valid_flags & MOTOR_OFFSET_TIM1_VALID) == 0U)
        return HAL_ERROR;
    /* Flash 可能已有上次成功提交的 TIM8 校准值，优先直接恢复。 */
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
    /* 只有无有效偏移才真正使能并锁转子，成功后追加 TIM8 标志。 */
    status = motor_auto_calibrate(&g_tim8_control);
    if (status != HAL_OK) return status;
    record.tim8_offset = g_tim8_control.electrical_offset;
    record.valid_flags |= MOTOR_OFFSET_TIM8_VALID;
    status = W25Q64_SaveParameters(&record, sizeof(record));
    if (status == HAL_OK) g_tim8_offset_valid = 1U;
    return status;
}

/* 速度环直接接收目标角速度，并把 q 轴电压输出到三相 PWM。 */
static void motor_run_speed_loop(MotorControl_t *control,
                                 float target_speed,
                                 float interval_s)
{
    float q_voltage;//周期要下发的 q 轴电压
    float electrical_angle;//换相电角度
    float previous_integral;//PID 更新前的积分值，用于抗积分饱和的二次判断
    float max_change;//本周期允许的电压变化量
    float upper;//本周期允许的电压上界
    float lower;//本周期允许的电压下界

    /* 内环：速度误差产生 q 轴电压；先记下积分，供后续斜率限幅时判断是否回退。 */
    previous_integral = control->speed_pid.integral;
    q_voltage = PID_Update(&control->speed_pid,
                           target_speed,
                           control->mechanical_speed,
                           interval_s);
    /* 每周期只允许 q 轴电压变化 max_change，减轻突然加速或换向。 */
    max_change = MOTOR_Q_SLEW_VPS * interval_s;//30 V/s × 0.005 s = 0.15 V
    /* 上下界都包含 0：允许立即向停车方向收敛，但限制重新施加电压的速度。 */
    upper = fmaxf(control->applied_q_voltage, 0.0f) + max_change;
    /* 下界从较小的“旧电压/零电压”起算，换向时允许先跨向零附近。 */
    lower = fminf(control->applied_q_voltage, 0.0f) - max_change;
    /* 最终下发值只能落在本周期允许的变化区间内。 */
    q_voltage = fmaxf(lower, fminf(upper, q_voltage));
    /* 若斜率限幅已经挡住输出，且积分继续推向限幅方向，就撤回本周期积分。 */
    if (((q_voltage >= upper && target_speed > control->mechanical_speed)
         || (q_voltage <= lower && target_speed < control->mechanical_speed))
        && fabsf(control->speed_pid.integral) >= fabsf(previous_integral))
        control->speed_pid.integral = previous_integral;
    control->applied_q_voltage = q_voltage;
    /* FOC 换相角由最新 AS5600 单圈角、电机方向、极对数和电角零位共同决定。 */
    electrical_angle = SimpleFOC_Motor_CalculateElectricalAngle(
        control->raw_angle,
        control->electrical_offset,
        MOTOR_POLE_PAIRS,
        MOTOR_DIRECTION_CLOCKWISE);

    /* 闭环正常运行时 d 轴置零，仅把速度环的 q 轴电压写入三相 PWM。 */
    SimpleFOC_Motor_SetPhaseVoltage(control->axis,
                                    0.0f,
                                    q_voltage,
                                    electrical_angle,
                                    MOTOR_SUPPLY_VOLTAGE);
}

/* 位置模式先由位置环求目标速度，再交给共同的速度环。 */
static void motor_run_position_loop(MotorControl_t *control,
                                    float target_position,
                                    float interval_s)
{
    float target_speed = PID_Update(&control->position_pid,
                                    target_position,
                                    control->mechanical_position,
                                    interval_s);
    motor_run_speed_loop(control, target_speed, interval_s);
}

/* 电机任务主循环：上电恢复/校准 TIM1，再按命令模式运行对应控制环。 */
void SimpleFOC_Motor_RunTask(SimpleFOC_ControlMode initial_mode, float initial_target)//模式、目标
{
    MotorOffsetRecord offset_record = {0};
    HAL_StatusTypeDef storage_status;
    uint32_t next_wake_tick;//下一次绝对唤醒时刻（tick）。
    uint32_t previous_tick;//上一周期的 tick，用于算 dt。
    uint32_t elapsed_ms;//实际经过的毫秒数。
    float target_position;//位置模式下换算后的绝对目标角(rad)。
    float interval_s;//dt（秒）
    float target_deg;//从共享状态取出的目标角度。
    float target_speed_rps;//取出的目标速度。
    float actual_deg;//本周期算出的“已转角度”。
    uint32_t command_sequence = 0U;//任务本地副本的命令序号。初值 0 保证第一条命令（序号变成 1）一定被识别为新命令。
    SimpleFOC_ControlMode mode;//本周期的模式（临界区内读取）。
    SimpleFOC_ControlMode previous_mode = SIMPLEFOC_CONTROL_NONE;//上一周期模式,用于判断“是否发生了模式切换”。初值 NONE 保证第一次进入任何模式都会触发一次 PID 复位。
    uint8_t driver_enabled = 0U; /* 仅在本任务收到首条有效命令后置 1。 */

    /* 两轴建立控制对象；TIM8 只读取已保存的偏移量，不使能电机。 */
    motor_control_init(&g_tim1_control, SIMPLEFOC_AXIS_VERTICAL, &hi2c1);
    motor_control_init(&g_tim8_control, SIMPLEFOC_AXIS_HORIZONTAL, &hi2c3);
    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_HORIZONTAL);

    /* 先校验双备份；旧版 0x0C 偏移量不适用于现在的 0x0E 角度源。 */
    storage_status = W25Q64_Init();
    if (storage_status == HAL_OK)
        storage_status = W25Q64_LoadParameters(&offset_record, sizeof(offset_record));
    /* 空 Flash 用 HAL_BUSY 表示无记录；版本/硬件配置不符时也从空记录开始。 */
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
    /* 仅记录 TIM8 可用偏移，绝不在上电阶段读取其角度或使能该轴。 */
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
    /* TIM1 偏移有效时，只读取当前位置建立本次上电的位置原点。 */
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
        /* 只有 TIM1 没有可用偏移时才对齐一次，并写入双备份参数包。 */
        storage_status = motor_auto_calibrate(&g_tim1_control);
        if (storage_status == HAL_OK)
        {
            offset_record.tim1_offset = g_tim1_control.electrical_offset;
            offset_record.valid_flags |= MOTOR_OFFSET_TIM1_VALID;
            storage_status = W25Q64_SaveParameters(&offset_record, sizeof(offset_record));
        }
    }

    /* 读写 Flash、传感器或校准失败时保持驱动关闭，避免错误电角度输出。 */
    /* 故障一旦锁存，任务停在延时循环；外部新命令也会因 fault 被拒绝。 */
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
    /* 启动时只接收所选模式的一条命令，后续可由应用层覆盖。 */
    if (initial_mode == SIMPLEFOC_CONTROL_SPEED)
        (void)MotorSpeed_SetTargetSpeed(initial_target);
    else if (initial_mode == SIMPLEFOC_CONTROL_POSITION)
        (void)MotorPosition_SetTargetDegrees(initial_target);
    next_wake_tick = osKernelGetTickCount();
    previous_tick = next_wake_tick;

    for (;;)
    {
        /* 绝对节拍延时避免循环工作耗时不断累积到下一周期。 */
        next_wake_tick += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);

        /* 用实际经过的毫秒数换算 dt，任务偶尔延迟时速度计算仍按真实间隔。 */
        elapsed_ms = osKernelGetTickCount() - previous_tick;
        previous_tick += elapsed_ms;
        /* 同一系统节拍内重复醒来时没有有效 dt，跳过速度差分。 */
        if (elapsed_ms == 0U)
        {
            continue;
        }
        /* dt 最大按 20 ms 处理，避免长时间阻塞后 PID 一次跨步过大。 */
        if (elapsed_ms > 20U)
        {
            elapsed_ms = 20U;
        }
        interval_s = (float)elapsed_ms * 0.001f;
        /* I2C 失联或相邻角度跳变过大时立即关 EN 并锁存故障。 */
        if (motor_update_feedback(&g_tim1_control, interval_s) != HAL_OK)
        {
            SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
            taskENTER_CRITICAL();
            g_motor_state.fault = 1U;
            taskEXIT_CRITICAL();
            for (;;) osDelay(100U);
        }

        /* 新位置命令只重设本条命令的零点，编码器原始角和速度历史保持连续。 */
        taskENTER_CRITICAL();
        mode = g_motor_state.mode;
        if (mode == SIMPLEFOC_CONTROL_POSITION
            && command_sequence != g_command_sequence)
        {
            g_tim1_control.mechanical_position = g_tim1_control.raw_angle;
            g_tim1_control.mechanical_origin = g_tim1_control.raw_angle;
        }
        command_sequence = g_command_sequence;
        if (mode == SIMPLEFOC_CONTROL_POSITION)
            actual_deg = (g_tim1_control.mechanical_position
                - g_tim1_control.mechanical_origin) * 180.0f / MOTOR_PI;
        else
        {
            /* 速度模式只用单圈角度差求速度，不让跨圈位置无限累加。 */
            g_tim1_control.mechanical_position = g_tim1_control.raw_angle;
            g_tim1_control.mechanical_origin = g_tim1_control.raw_angle;
            actual_deg = 0.0f;
        }
        g_motor_state.actual_deg = actual_deg;
        target_deg = g_motor_state.target_deg;
        target_speed_rps = g_motor_state.target_speed_rps;
        taskEXIT_CRITICAL();
        if (mode == SIMPLEFOC_CONTROL_NONE) continue;

        /* 位置命令有固定终点；速度命令没有终点，不做位置越界检查。 */
        if (mode == SIMPLEFOC_CONTROL_POSITION
            && ((target_deg > 0.0f
                 && actual_deg > target_deg + MOTOR_MAX_OVERSHOOT_DEG)
                || (target_deg < 0.0f
                    && actual_deg < target_deg - MOTOR_MAX_OVERSHOOT_DEG)))
        {
            SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
            taskENTER_CRITICAL();
            g_motor_state.fault = 1U;
            taskEXIT_CRITICAL();
            for (;;) osDelay(100U);
        }

        /* 切换控制模式才重置 PID 历史；同模式频繁更新目标不会反复清零积分。 */
        if (mode != previous_mode)
        {
            PID_Reset(&g_tim1_control.position_pid);
            PID_Reset(&g_tim1_control.speed_pid);
            previous_mode = mode;
        }
        if (driver_enabled == 0U)
        {
            SimpleFOC_Motor_Enable(SIMPLEFOC_AXIS_VERTICAL);
            driver_enabled = 1U;
        }
        if (mode == SIMPLEFOC_CONTROL_POSITION)
        {
            target_position = g_tim1_control.mechanical_origin
                + target_deg * MOTOR_PI / 180.0f;
            motor_run_position_loop(&g_tim1_control, target_position, interval_s);
        }
        else
        {
            motor_run_speed_loop(&g_tim1_control,
                                 target_speed_rps * MOTOR_TWO_PI,
                                 interval_s);
        }
    }
}
