#include "SimpleFOC_Motor.h"
#include "AppMotorCommand.h"
#include "app_system.h"

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
#define MOTOR_ALIGNMENT_CHECK_MS         10U   /* 校准期间检查松手停车的周期。 */
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
#define MOTOR_MAX_Q_VOLTAGE              5.5f   /* q 轴电压指令的绝对值上限，单位伏；不是电流上限。 */
#define MOTOR_Q_SLEW_VPS                 20.0f   /* q 轴电压指令每秒允许变化的最大值，单位伏/秒。 */
#define MOTOR_MAX_STEP_DEG               12.0f   /* 驱动运行期间相邻两次角度变化上限；空闲时不检查。 */
#define MOTOR_MAX_OVERSHOOT_DEG         10.0f   /* 沿指令方向越过目标角度的容许量，超出则故障停机。 */
#define MOTOR_MAX_COMMAND_DEG          360.0f   /* 单次相对转角指令的绝对值上限，超出则拒绝指令。 */
#define MOTOR_MAX_SPEED_RPS             (MOTOR_MAX_SPEED_DPS / 360.0f)

/* 系统任务发给电机任务的自动模式角度队列。 */
extern osMessageQueueId_t SmapSystemToMotorHandle;

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

/* 两轴共用同一控制流程，各自保存编码器、PID 与电角偏移。 */
static MotorControl_t g_motor_control[2];
/* 两轴电角零位共用一份 Flash 记录，由 W25Q64 模块负责双扇区备份。 */
typedef struct
{
    uint32_t signature;    /* 当前记录格式和角度源的识别值。 */
    uint32_t config;       /* 极对数与方向，避免误用其他电机配置的偏移量。 */
    uint32_t valid_flags;  /* 位标志分别表示 TIM1、TIM8 偏移是否有效。 */
    float tim1_offset;     /* 竖直轴电角偏移，单位 rad。 */
    float tim8_offset;     /* 水平轴电角偏移，单位 rad。 */
} MotorOffsetRecord;

/* 两轴各自的位置命令与故障状态，在任务和外部命令调用者之间共享。 */
typedef struct
{
    float target_deg;  /* 相对最近一条位置命令起点的目标角度，单位度。 */
    float actual_deg;  /* 相对最近一条位置命令起点的反馈角度，单位度。 */
    float target_speed_rps; /* 速度命令，单位圈/秒。 */
    SimpleFOC_ControlMode mode; /* 当前只执行位置环或速度环中的一种。 */
    uint8_t ready;     /* 本轴编码器、电角偏移与存储准备完成。 */
    uint8_t calibrated; /* 电角偏移已恢复或校准，可安全进入位置保持。 */
    uint8_t fault;     /* 传感器、存储或越界故障已锁存。 */
} MotorPositionState;

static MotorPositionState g_motor_state[2];
static uint8_t recovery_state;
static uint32_t g_command_sequence[2]; /* 每条新命令更新一次，供控制任务重设位置零点。 */

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

/* 逆 Clarke：由静止坐标系电压还原三相参考值。 */
void SimpleFOC_InvClarke(const SimpleFOC_AlphaBeta *input, float phase_voltage[3])
{
    if (input == NULL || phase_voltage == NULL) return;
    phase_voltage[0] = input->alpha;
    phase_voltage[1] = -SFOC_HALF * input->alpha + SFOC_SQRT3_BY_TWO * input->beta;
    phase_voltage[2] = -SFOC_HALF * input->alpha - SFOC_SQRT3_BY_TWO * input->beta;
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

    SimpleFOC_InvClarke(input, phase_voltage);
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


/* 接收指定轴相对位置指令；每条新命令以最近一次反馈位置为零点。 */
uint8_t MotorPosition_SetTargetDegrees(SimpleFOC_MotorAxis axis, float delta_deg)
{
    MotorPositionState *state;
    /* 拒绝 NaN/无穷和超过单次转角上限的命令，避免目标污染控制环。 */
    if ((axis != SIMPLEFOC_AXIS_VERTICAL && axis != SIMPLEFOC_AXIS_HORIZONTAL)
        || !isfinite(delta_deg)
        || fabsf(delta_deg) > MOTOR_MAX_COMMAND_DEG)
        return 0U;
    state = &g_motor_state[axis];

    /* 命令调用者与电机任务共享实际位置、目标和故障标志，整组读写必须原子化。 */
    taskENTER_CRITICAL();
    if (state->ready == 0U || state->fault != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    state->actual_deg = 0.0f;
    state->target_deg = delta_deg;
    state->mode = SIMPLEFOC_CONTROL_POSITION;
    ++g_command_sequence[axis];
    taskEXIT_CRITICAL();
    return 1U;
}

/* 接收指定轴速度命令；只设置速度环目标，不产生位置目标。 */
uint8_t MotorSpeed_SetTargetSpeed(SimpleFOC_MotorAxis axis, float speed_rps)
{
    MotorPositionState *state;
    if ((axis != SIMPLEFOC_AXIS_VERTICAL && axis != SIMPLEFOC_AXIS_HORIZONTAL)
        || !isfinite(speed_rps)
        || fabsf(speed_rps) > MOTOR_MAX_SPEED_RPS)
        return 0U;
    state = &g_motor_state[axis];

    taskENTER_CRITICAL();
    if (state->ready == 0U || state->fault != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    state->target_speed_rps = speed_rps;
    state->mode = SIMPLEFOC_CONTROL_SPEED;
    ++g_command_sequence[axis];
    taskEXIT_CRITICAL();
    return 1U;
}

/* 松手后在下一控制周期以最新反馈角度为零点，持续运行位置保持。 */
uint8_t MotorPosition_HoldCurrent(SimpleFOC_MotorAxis axis)
{
    MotorPositionState *state;
    uint8_t accepted;
    if (axis != SIMPLEFOC_AXIS_VERTICAL && axis != SIMPLEFOC_AXIS_HORIZONTAL) return 0U;
    state = &g_motor_state[axis];
    taskENTER_CRITICAL();
    accepted = state->ready != 0U && state->fault == 0U && state->calibrated != 0U;
    state->target_speed_rps = 0.0f;
    state->target_deg = 0.0f;
    state->mode = accepted != 0U ? SIMPLEFOC_CONTROL_POSITION : SIMPLEFOC_CONTROL_NONE;
    ++g_command_sequence[axis];
    taskEXIT_CRITICAL();
    return accepted;
}

/* 清除目标并请求电机任务关闭指定轴驱动。 */
void MotorSpeed_Stop(SimpleFOC_MotorAxis axis)
{
    MotorPositionState *state;
    if (axis != SIMPLEFOC_AXIS_VERTICAL && axis != SIMPLEFOC_AXIS_HORIZONTAL) return;
    state = &g_motor_state[axis];
    taskENTER_CRITICAL();
    state->target_speed_rps = 0.0f;
    state->mode = SIMPLEFOC_CONTROL_NONE;
    ++g_command_sequence[axis];
    taskEXIT_CRITICAL();
}

/* UI 只登记请求，避免在 UI 或系统任务中直接操作电机硬件。 */
uint8_t SimpleFOC_Motor_RequestRecovery(void)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (recovery_state == 0U)
    {
        recovery_state = 1U;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint8_t SimpleFOC_Motor_TakeRecoveryResult(void)
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (recovery_state > 1U)
    {
        result = recovery_state == 2U ? 1U : 2U;
        recovery_state = 0U;
    }
    taskEXIT_CRITICAL();
    return result;
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
    uint32_t started_sequence;

    taskENTER_CRITICAL();
    started_sequence = g_command_sequence[control->axis];
    taskEXIT_CRITICAL();

    /* 固定电角度只输出 d 轴电压，使转子停在已知磁场方向。 */
    SimpleFOC_Motor_SetPhaseVoltage(control->axis,
                                    MOTOR_ALIGNMENT_VOLTAGE,
                                    0.0f,
                                    MOTOR_ALIGNMENT_ANGLE_RAD,
                                    MOTOR_SUPPLY_VOLTAGE);
    /* 对齐时间内电机会实际转动；等待稳定后才读取编码器。 */
    SimpleFOC_Motor_Enable(control->axis);
    for (uint32_t elapsed = 0U; elapsed < MOTOR_ALIGNMENT_TIME_MS;
         elapsed += MOTOR_ALIGNMENT_CHECK_MS)
    {
        osDelay(MOTOR_ALIGNMENT_CHECK_MS);
        taskENTER_CRITICAL();
        uint8_t cancelled = g_command_sequence[control->axis] != started_sequence;
        taskEXIT_CRITICAL();
        if (cancelled != 0U)
        {
            SimpleFOC_Motor_Disable(control->axis);
            return HAL_BUSY;
        }
    }

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

/* 为指定轴恢复电角偏移；没有有效记录时校准并保存整包。 */
static HAL_StatusTypeDef motor_prepare(MotorControl_t *control,
                                       MotorOffsetRecord *record, uint8_t allow_calibration)
{
    MotorOffsetRecord updated;
    uint32_t flag = control->axis == SIMPLEFOC_AXIS_VERTICAL
        ? MOTOR_OFFSET_TIM1_VALID : MOTOR_OFFSET_TIM8_VALID;
    float offset = control->axis == SIMPLEFOC_AXIS_VERTICAL
        ? record->tim1_offset : record->tim8_offset;
    HAL_StatusTypeDef status;

    if ((record->valid_flags & flag) != 0U && isfinite(offset)
        && offset >= 0.0f && offset < MOTOR_TWO_PI)
    {
        control->electrical_offset = offset;
        status = motor_update_feedback(control, 0.005f);
        if (status == HAL_OK)
            control->mechanical_origin = control->mechanical_position;
        return status;
    }

    if (allow_calibration == 0U) return HAL_BUSY;
    status = motor_auto_calibrate(control);
    if (status != HAL_OK) return status;
    updated = *record;
    if (control->axis == SIMPLEFOC_AXIS_VERTICAL)
        updated.tim1_offset = control->electrical_offset;
    else
        updated.tim8_offset = control->electrical_offset;
    updated.valid_flags |= flag;
    status = W25Q64_SaveParameters(&updated, sizeof(updated));
    if (status == HAL_OK) *record = updated;
    return status;
}

/* 单轴故障只关闭该轴；另一轴继续执行自己的控制环。 */
static void motor_fault(SimpleFOC_MotorAxis axis)
{
    SimpleFOC_Motor_Disable(axis);
    taskENTER_CRITICAL();
    g_motor_state[axis].fault = 1U;
    g_motor_state[axis].ready = 0U;
    g_motor_state[axis].calibrated = 0U;
    taskEXIT_CRITICAL();
    App_System_ReportFault();
}

/* 两轴均通过存储记录和编码器检查后才重新开放命令；检查期间保持 EN 关闭。 */
static uint8_t motor_try_recover(MotorOffsetRecord *record,
                                 uint8_t driver_enabled[2], uint8_t offset_ready[2],
                                 SimpleFOC_ControlMode previous_mode[2])
{
    MotorOffsetRecord checked_record;
    MotorControl_t checked[2];
    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_HORIZONTAL);
    driver_enabled[0] = driver_enabled[1] = 0U;
    if (W25Q64_Init() != HAL_OK
        || W25Q64_LoadParameters(&checked_record, sizeof(checked_record)) != HAL_OK
        || checked_record.signature != MOTOR_OFFSET_SIGNATURE
        || checked_record.config != MOTOR_OFFSET_CONFIG
        || (checked_record.valid_flags & (MOTOR_OFFSET_TIM1_VALID | MOTOR_OFFSET_TIM8_VALID))
            != (MOTOR_OFFSET_TIM1_VALID | MOTOR_OFFSET_TIM8_VALID)) return 0U;

    motor_control_init(&checked[0], SIMPLEFOC_AXIS_VERTICAL, &hi2c1);
    motor_control_init(&checked[1], SIMPLEFOC_AXIS_HORIZONTAL, &hi2c3);
    if (motor_prepare(&checked[0], &checked_record, 0U) != HAL_OK
        || motor_prepare(&checked[1], &checked_record, 0U) != HAL_OK) return 0U;

    *record = checked_record;
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        g_motor_control[i] = checked[i];
        offset_ready[i] = 1U;
        previous_mode[i] = SIMPLEFOC_CONTROL_NONE;
        taskENTER_CRITICAL();
        g_motor_state[i].target_deg = 0.0f;
        g_motor_state[i].actual_deg = 0.0f;
        g_motor_state[i].target_speed_rps = 0.0f;
        g_motor_state[i].mode = SIMPLEFOC_CONTROL_NONE;
        g_motor_state[i].ready = 1U;
        g_motor_state[i].calibrated = 1U;
        g_motor_state[i].fault = 0U;
        ++g_command_sequence[i];
        taskEXIT_CRITICAL();
    }
    return 1U;
}

/* 每个周期按同一逻辑处理指定电机。 */
static void motor_run_axis(SimpleFOC_MotorAxis axis, float interval_s,
                           uint32_t command_sequence[2],
                           SimpleFOC_ControlMode previous_mode[2],
                           uint8_t driver_enabled[2],
                           uint8_t offset_ready[2], MotorOffsetRecord *record)
{
    MotorControl_t *control = &g_motor_control[axis];
    MotorPositionState *state = &g_motor_state[axis];
    SimpleFOC_ControlMode mode;
    float target_deg, target_speed_rps, actual_deg;

    if (state->ready == 0U || state->fault != 0U) return;
    taskENTER_CRITICAL();
    mode = state->mode;
    taskEXIT_CRITICAL();
    if (mode == SIMPLEFOC_CONTROL_NONE)
    {
        if (driver_enabled[axis] != 0U)
        {
            SimpleFOC_Motor_Disable(axis);
            driver_enabled[axis] = 0U;
        }
        control->applied_q_voltage = 0.0f;
        /* 空闲期间允许外力转轴；下次启动先以当前编码器角度重建速度基准。 */
        control->feedback_initialized = 0U;
        control->mechanical_speed = 0.0f;
        previous_mode[axis] = SIMPLEFOC_CONTROL_NONE;
        return;
    }
    /* 首次手动命令到来后才允许进行可能使转子移动的电角校准。 */
    if (offset_ready[axis] == 0U)
    {
        HAL_StatusTypeDef prepare_status = motor_prepare(control, record, 1U);
        if (prepare_status == HAL_BUSY) return;
        if (prepare_status != HAL_OK)
        {
            motor_fault(axis);
            return;
        }
        offset_ready[axis] = 1U;
        taskENTER_CRITICAL();
        state->calibrated = 1U;
        mode = state->mode;
        taskEXIT_CRITICAL();
        if (mode == SIMPLEFOC_CONTROL_NONE) return;
    }
    if (motor_update_feedback(control, interval_s) != HAL_OK)
    {
        motor_fault(axis);
        return;
    }

    taskENTER_CRITICAL();
    mode = state->mode;
    if (mode == SIMPLEFOC_CONTROL_POSITION
        && command_sequence[axis] != g_command_sequence[axis])
    {
        control->mechanical_position = control->raw_angle;
        control->mechanical_origin = control->raw_angle;
    }
    command_sequence[axis] = g_command_sequence[axis];
    if (mode == SIMPLEFOC_CONTROL_POSITION)
        actual_deg = (control->mechanical_position - control->mechanical_origin)
            * 180.0f / MOTOR_PI;
    else
    {
        control->mechanical_position = control->raw_angle;
        control->mechanical_origin = control->raw_angle;
        actual_deg = 0.0f;
    }
    state->actual_deg = actual_deg;
    target_deg = state->target_deg;
    target_speed_rps = state->target_speed_rps;
    taskEXIT_CRITICAL();
    if (mode == SIMPLEFOC_CONTROL_NONE) return;

    if (mode == SIMPLEFOC_CONTROL_POSITION
        && ((target_deg > 0.0f && actual_deg > target_deg + MOTOR_MAX_OVERSHOOT_DEG)
            || (target_deg < 0.0f && actual_deg < target_deg - MOTOR_MAX_OVERSHOOT_DEG)))
    {
        motor_fault(axis);
        return;
    }
    if (mode != previous_mode[axis])
    {
        PID_Reset(&control->position_pid);
        PID_Reset(&control->speed_pid);
        previous_mode[axis] = mode;
    }
    if (mode == SIMPLEFOC_CONTROL_POSITION)
        motor_run_position_loop(control,
            control->mechanical_origin + target_deg * MOTOR_PI / 180.0f,
            interval_s);
    else
        motor_run_speed_loop(control, target_speed_rps * MOTOR_TWO_PI, interval_s);
    /* 先写入本周期三相占空比，再打开驱动使能。 */
    if (driver_enabled[axis] == 0U)
    {
        SimpleFOC_Motor_Enable(axis);
        driver_enabled[axis] = 1U;
    }
}

/* 上电依次准备两轴，随后在同一任务内分别执行闭环。 */
void SimpleFOC_Motor_RunTask(SimpleFOC_ControlMode initial_mode, float initial_target)
{
    MotorOffsetRecord record = {0};
    HAL_StatusTypeDef status;
    uint32_t next_wake_tick, previous_tick, elapsed_ms;
    uint32_t command_sequence[2] = {0U, 0U};
    SimpleFOC_ControlMode previous_mode[2] = {
        SIMPLEFOC_CONTROL_NONE, SIMPLEFOC_CONTROL_NONE};
    uint8_t driver_enabled[2] = {0U, 0U};
    uint8_t offset_ready[2] = {0U, 0U};
    /* 两轴是否已在当前光照平衡区间锁定保持点。 */
    uint8_t auto_hold_sent[2] = {0U, 0U};

    motor_control_init(&g_motor_control[0], SIMPLEFOC_AXIS_VERTICAL, &hi2c1);
    motor_control_init(&g_motor_control[1], SIMPLEFOC_AXIS_HORIZONTAL, &hi2c3);
    status = W25Q64_Init();
    if (status == HAL_OK)
        status = W25Q64_LoadParameters(&record, sizeof(record));
    if (status == HAL_BUSY || (status == HAL_OK
        && (record.signature != MOTOR_OFFSET_SIGNATURE
            || record.config != MOTOR_OFFSET_CONFIG)))
    {
        record.signature = MOTOR_OFFSET_SIGNATURE;
        record.config = MOTOR_OFFSET_CONFIG;
        record.valid_flags = 0U;
        status = HAL_OK;
    }
    if (status == HAL_OK)
    {
        for (uint32_t i = 0U; i < 2U; ++i)
        {
            status = motor_prepare(&g_motor_control[i], &record, 0U);
            if (status == HAL_OK || status == HAL_BUSY)
            {
                SimpleFOC_Motor_Disable((SimpleFOC_MotorAxis)i);
                offset_ready[i] = status == HAL_OK ? 1U : 0U;
                taskENTER_CRITICAL();
                g_motor_state[i].ready = 1U;
                g_motor_state[i].calibrated = offset_ready[i];
                taskEXIT_CRITICAL();
            }
            else
                motor_fault((SimpleFOC_MotorAxis)i);
        }
    }
    else
    {
        motor_fault(SIMPLEFOC_AXIS_VERTICAL);
        motor_fault(SIMPLEFOC_AXIS_HORIZONTAL);
    }

    for (uint32_t i = 0U; i < 2U; ++i)
    {
        if (initial_mode == SIMPLEFOC_CONTROL_SPEED)
            (void)MotorSpeed_SetTargetSpeed((SimpleFOC_MotorAxis)i, initial_target);
        else if (initial_mode == SIMPLEFOC_CONTROL_POSITION)
            (void)MotorPosition_SetTargetDegrees((SimpleFOC_MotorAxis)i, initial_target);
    }
    next_wake_tick = osKernelGetTickCount();
    previous_tick = next_wake_tick;
    for (;;)
    {
        next_wake_tick += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);
        elapsed_ms = osKernelGetTickCount() - previous_tick;
        previous_tick += elapsed_ms;
        if (elapsed_ms == 0U) continue;
        if (elapsed_ms > 20U) elapsed_ms = 20U;
        if (recovery_state == 1U)
        {
            uint8_t recovered = motor_try_recover(&record, driver_enabled,
                                                   offset_ready, previous_mode);
            taskENTER_CRITICAL();
            recovery_state = recovered != 0U ? 2U : 3U;
            taskEXIT_CRITICAL();
            continue;
        }
        /* 在本电机任务中只取最新自动角度，避免追赶过时光照。 */
        if (SmapSystemToMotorHandle != NULL)
        {
            AppMotorCommand command;
            uint8_t received = 0U;
            while (osMessageQueueGet(SmapSystemToMotorHandle, &command, NULL, 0U) == osOK)
                received = 1U;
            if (App_System_IsAutoMode() == 0U)
            {
                auto_hold_sent[0] = 0U;
                auto_hold_sent[1] = 0U;
            }
            else if (received != 0U)
            {
                for (uint32_t i = 0U; i < 2U; ++i)
                {
                    /* 两轴分别使用上下差和左右差计算出的目标。 */
                    float target = i == SIMPLEFOC_AXIS_VERTICAL
                        ? command.vertical_deg : command.horizontal_deg;
                    /* 未校准时超时停车可能进入空闲，恢复采样后需重新锁定零度。 */
                    if (g_motor_state[i].mode == SIMPLEFOC_CONTROL_NONE)
                        auto_hold_sent[i] = 0U;
                    if (target != 0.0f)
                    {
                        (void)MotorPosition_SetTargetDegrees((SimpleFOC_MotorAxis)i, target);
                        auto_hold_sent[i] = 0U;
                    }
                    else if (auto_hold_sent[i] == 0U)
                    {
                        auto_hold_sent[i] = MotorPosition_SetTargetDegrees(
                            (SimpleFOC_MotorAxis)i, 0.0f);
                    }
                }
            }
        }
        for (uint32_t i = 0U; i < 2U; ++i)
            motor_run_axis((SimpleFOC_MotorAxis)i, (float)elapsed_ms * 0.001f,
                           command_sequence, previous_mode, driver_enabled,
                           offset_ready, &record);
    }
}
