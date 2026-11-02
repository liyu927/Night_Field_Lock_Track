#include "app_system.h"
#include "AppMotorCommand.h"
#include "SmaplingTypes.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rtc.h"
#include "tim.h"
#include "main.h"
#include "adc.h"
#include "lv_port_lcd_stm32.h"

/* 全部页面连续无触摸 15 秒后关背光；手动模式同时进入 STOP。 */
#define POWER_IDLE_MS             15000U
/* 正常背光占空比，范围为 0～100。 */
#define POWER_NORMAL_BRIGHTNESS   100U
/* 关屏前五秒将背光降为正常亮度的 30%。 */
#define POWER_DIM_MS              5000U
#define POWER_DIM_PERCENT         30U
/* 自动模式 A 或 B 连续成立 15 秒后进入 STOP，与关背光分别计时。 */
#define POWER_AUTO_MS             15000U
/* 自动模式 B 的 ADC 平均值门槛：严格大于 1800 才成立。 */
#define POWER_DARK_AVG            1800U
/* RTC 使用 1 Hz 的 ck_spre；计数值 2 对应约 3 秒一次唤醒。 */
#define POWER_RTC_COUNTER         2U

/* STOP 前暂停这些任务；系统任务负责恢复，不在中断里操作任务。 */
extern osThreadId_t UiTaskHandle, MotorTaskHandle, SmaplingTaskHandle, DebugTaskHandle;
/* STM32F407 退出 STOP 后先运行 HSI，需要重新建立原来的系统时钟。 */
extern void SystemClock_Config(void);

/* 触摸中断只递增序号；系统任务比较序号以识别新的触摸事件。 */
static volatile uint32_t touch_sequence;
/* RTC 中断只递增序号；系统任务据此决定是否进行一轮光照检查。 */
static volatile uint32_t rtc_sequence;
/* LVGL 轮询到的手指接触状态；持续按住时禁止空闲入睡。 */
static volatile uint8_t touch_held;
/* 黑屏后的首次触摸需要整次屏蔽，避免点亮时同时触发按钮。 */
static volatile uint8_t touch_suppressed;
/* 连续两次读到松手才解除屏蔽，避免一次 I2C 读取失败造成误点。 */
static volatile uint8_t touch_release_count;
/* 背光状态独立于 MCU STOP：自动追光可以在黑屏下继续运行。 */
static volatile uint8_t screen_on = 1U;
/* 记录 UI 任务是否已暂停，防止对同一任务重复暂停或恢复。 */
static uint8_t ui_suspended;

/* 触摸 EXTI 回调入口：只记录事件；黑屏时标记本次触摸无效。 */
void App_System_TouchInterrupt(void)
{
    /* 用序号而不是单个布尔标志，防止处理前又来一次触摸时丢事件。 */
    ++touch_sequence;
    if (screen_on == 0U)
    {
        /* 即使触摸落在“保存”按钮上，也只允许它唤醒背光。 */
        touch_suppressed = 1U;
        /* 本次手势尚未确认松开，重新开始松手判定。 */
        touch_release_count = 0U;
    }
}

/* RTC 唤醒中断回调：只留下标记，采样和状态判断放在系统任务。 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    /* HAL 回调要求传入句柄，本处不需要在中断里访问 RTC 寄存器。 */
    (void)hrtc;
    ++rtc_sequence;
}

/* LVGL 输入层报告手指是否仍按在屏幕上，供空闲计时使用。 */
void App_System_TouchHeld(uint8_t held)
{
    touch_held = held;
}

/* 返回 1 时 LVGL 必须上报“松开”，直到唤醒手势真正结束。 */
uint8_t App_System_SuppressTouch(uint8_t pressed)
{
    /* 正常亮屏触摸直接传给 LVGL，不改变页面操作。 */
    if (touch_suppressed == 0U) return 0U;
    /* 仍在按住就重新等待松手，整次拖动也不能触发页面控件。 */
    if (pressed != 0U) touch_release_count = 0U;
    /* 连续两次未按下才解除屏蔽；本次调用仍返回屏蔽状态。 */
    else if (++touch_release_count >= 2U) touch_suppressed = 0U;
    return 1U;
}

/* 手动按住时的目标机械角速度，单位度/秒。 */
#define MANUAL_SPEED_DPS          15.0f
/* 手动按住事件超过此时间未续发时改为保持当前位置。 */
#define MANUAL_HOLD_TIMEOUT_MS    200U
/* 上键和右键对应的电机正方向，实机方向不符时改符号。 */
#define MANUAL_UP_SIGN            1.0f
#define MANUAL_RIGHT_SIGN         1.0f
/* 自动模式进入后等待三秒，再允许光强环输出。 */
#define AUTO_ENTER_DELAY_MS       3000U
/* 超过此时间没有新光照帧时停止追光并保持当前位置。 */
#define AUTO_SAMPLE_TIMEOUT_MS    100U
/* 水平光强差处于此范围时锁定水平轴当前位置。 */
#define AUTO_HORIZONTAL_BAND      50
/* 竖直光强差处于此范围时锁定竖直轴当前位置。 */
#define AUTO_VERTICAL_BAND        100
/* 光强比例环增益：差值 1500 对应相对目标 150 度。 */
#define AUTO_LIGHT_KP             0.1f
/* 四路滤波 ADC 均严格高于此值时认为环境较暗，停止追光并保持位置。 */
#define AUTO_ALL_DARK_ADC         1500U
/* 单次相对角度命令的最大绝对值，单位度。 */
#define AUTO_TARGET_LIMIT_DEG     360.0f
/* 水平轴追向左侧亮光、竖直轴追向上方亮光的方向系数。 */
#define AUTO_HORIZONTAL_SIGN      (-MANUAL_RIGHT_SIGN)
#define AUTO_VERTICAL_SIGN        MANUAL_UP_SIGN

/* 光照快照的派生量由系统任务集中计算。 */
typedef struct
{
    int32_t horizontal_diff; /* 左上加左下，减右上加右下。 */
    int32_t vertical_diff;   /* 左上加右上，减左下加右下。 */
    uint16_t average;        /* 四路滤波 ADC 的平均值。 */
    uint16_t total_diff;     /* 四路滤波 ADC 的最大值减最小值。 */
} AppLightMetrics;

/* UI 发送的最新手动方向与时间戳。 */
static AppManualDirection pending_direction = APP_MANUAL_STOP;
static uint32_t pending_tick;
static uint32_t pending_sequence;
/* UI 发送的模式请求及系统任务发布的当前模式。 */
static AppMode pending_mode = APP_MODE_MANUAL;
static uint32_t pending_mode_sequence;
static AppMode current_mode = APP_MODE_MANUAL;
/* 故障请求与一次性弹窗通知，异常模式锁存到复位。 */
static uint8_t fault_requested;
static uint8_t error_popup_pending;
/* CubeMX 创建的采样队列和自动角度命令队列。 */
extern osMessageQueueId_t SmapToSystemHandle;
extern osMessageQueueId_t SmapSystemToMotorHandle;
extern osMessageQueueId_t SmaplingHandle;

/* UI 只投递方向，实际电机命令由系统任务发出。 */
void App_ManualControl_Send(AppManualDirection direction)
{
    if (direction > APP_MANUAL_RIGHT) return;
    taskENTER_CRITICAL();
    pending_direction = direction;
    pending_tick = osKernelGetTickCount();
    ++pending_sequence;
    taskEXIT_CRITICAL();
}

/* UI 投递模式切换请求，不直接改变当前模式。 */
void App_System_RequestMode(AppMode mode)
{
    if (mode != APP_MODE_MANUAL && mode != APP_MODE_AUTO) return;
    taskENTER_CRITICAL();
    if (fault_requested == 0U && current_mode != APP_MODE_ERROR)
    {
        pending_mode = mode;
        ++pending_mode_sequence;
    }
    taskEXIT_CRITICAL();
}

/* 故障报告只设置请求，不在其他任务中修改模式或调用 LVGL。 */
void App_System_ReportFault(void)
{
    taskENTER_CRITICAL();
    fault_requested = 1U;
    taskEXIT_CRITICAL();
}

/* 弹窗通知仅由 UI 任务领取一次，关闭弹窗不清除异常锁存。 */
uint8_t App_System_TakeErrorPopup(void)
{
    uint8_t pending;
    taskENTER_CRITICAL();
    pending = error_popup_pending;
    error_popup_pending = 0U;
    taskEXIT_CRITICAL();
    return pending;
}

/* 电机任务读取系统任务发布的当前模式。 */
uint8_t App_System_IsAutoMode(void)
{
    uint8_t enabled;
    taskENTER_CRITICAL();
    enabled = current_mode == APP_MODE_AUTO;
    taskEXIT_CRITICAL();
    return enabled;
}

/* 根据手动按钮选择竖直轴或水平轴。 */
static SimpleFOC_MotorAxis manual_axis(AppManualDirection direction)
{
    return direction == APP_MANUAL_UP || direction == APP_MANUAL_DOWN
        ? SIMPLEFOC_AXIS_VERTICAL : SIMPLEFOC_AXIS_HORIZONTAL;
}

/* 限制光强环给出的单次相对目标角度。 */
static float auto_limit_target(float target_deg)
{
    if (target_deg > AUTO_TARGET_LIMIT_DEG) return AUTO_TARGET_LIMIT_DEG;
    if (target_deg < -AUTO_TARGET_LIMIT_DEG) return -AUTO_TARGET_LIMIT_DEG;
    return target_deg;
}

/* 计算四路滤波光照的平均、极差和两个方向差。 */
static AppLightMetrics light_analyze(const SmaplingData_t *sample)
{
    /* 四路值、最大值、最小值与总和。 */
    uint16_t values[4] = {sample->left_top, sample->left_bottom,
                          sample->right_top, sample->right_bottom};
    uint16_t maximum = values[0], minimum = values[0];
    uint32_t sum = 0U;
    /* 本次采样对应的派生结果。 */
    AppLightMetrics result;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (values[i] > maximum) maximum = values[i];
        if (values[i] < minimum) minimum = values[i];
        sum += values[i];
    }
    result.horizontal_diff = (int32_t)values[0] + values[1] - values[2] - values[3];
    result.vertical_diff = (int32_t)values[0] + values[2] - values[1] - values[3];
    result.average = (uint16_t)(sum / 4U);
    result.total_diff = maximum - minimum;
    return result;
}

/* 自动入睡条件：A 为两轴光差都在停转区间，B 为四路平均值偏暗。 */
static uint8_t power_auto_idle(const AppLightMetrics *light)
{
    /* 四个比较必须同时成立才算 A；任一轴超限都表示可能需要追光。 */
    uint8_t balanced = light->horizontal_diff >= -AUTO_HORIZONTAL_BAND
        && light->horizontal_diff <= AUTO_HORIZONTAL_BAND
        && light->vertical_diff >= -AUTO_VERTICAL_BAND
        && light->vertical_diff <= AUTO_VERTICAL_BAND;
    /* A 或 B 满足即可继续累计同一只 15 秒计时器。 */
    return balanced != 0U || light->average > POWER_DARK_AVG;
}

/* TIM3 的 ARR 为 99，CCR4 取 0～100 正好对应 0～100% 占空比。 */
static void power_backlight_set(uint8_t percent)
{
    if (percent > 100U) percent = 100U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, percent);
}

/* STOP 会停止 TIM3，黑屏时用 GPIO 保证 PB1 持续为低电平。 */
static void power_backlight_pin_set(uint8_t pwm)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_1;
    gpio.Pull = GPIO_NOPULL;
    if (pwm != 0U)
    {
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF2_TIM3;
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
    }
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* 只切换背光和 UI 任务；黑屏不等于已进入 MCU STOP。 */
static void power_screen_set(uint8_t on)
{
    /* 状态未变化时不重复操作背光和任务。 */
    if (on == screen_on) return;
    if (on == 0U)
    {
        /* 先公布黑屏状态，之后到来的触摸中断才会屏蔽首次手势。 */
        screen_on = 0U;
        if (UiTaskHandle != NULL && ui_suspended == 0U)
        {
            /* 保留 LVGL 对象和页面栈，只停止刷新；草稿不会被销毁。 */
            (void)osThreadSuspend(UiTaskHandle);
            ui_suspended = 1U;
        }
        /* 等待已开始的 SPI DMA 刷屏结束，再关闭背光或进入 STOP。 */
        lv_port_disp_wait_idle();
        power_backlight_set(0U);
        power_backlight_pin_set(0U);
    }
    else
    {
        /* 唤醒时恢复正常占空比。 */
        power_backlight_set(POWER_NORMAL_BRIGHTNESS);
        power_backlight_pin_set(1U);
        screen_on = 1U;
        if (ui_suspended != 0U)
        {
            /* 继续刷新原页面，LVGL 不重新创建页面或参数控件。 */
            (void)osThreadResume(UiTaskHandle);
            ui_suspended = 0U;
        }
    }
}

/* 读取 RTC 的当日毫秒数，用于补偿 MCU STOP 期间暂停的 RTOS 节拍。 */
static uint32_t power_rtc_ms(void)
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    /* HAL 要求读完时间再读日期，才能释放日历影子寄存器。 */
    (void)HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    (void)HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
    /* 秒字段转毫秒；亚秒寄存器向下计数，所以用预分频值减它。 */
    return ((uint32_t)time.Hours * 3600U + (uint32_t)time.Minutes * 60U
        + time.Seconds) * 1000U
        + ((uint32_t)(hrtc.Init.SynchPrediv - time.SubSeconds) * 1000U)
          / (hrtc.Init.SynchPrediv + 1U);
}

/* RTC 唤醒后短暂恢复 ADC，读取多次完整四路数据后返回最新滤波结果。 */
static uint8_t power_probe(SmaplingData_t *sample, AppLightMetrics *light)
{
    uint8_t received = 0U;
    /* 先清空 STOP 前的队列，不能拿睡前旧值决定是否重新睡眠。 */
    if (SmapToSystemHandle != NULL) (void)osMessageQueueReset(SmapToSystemHandle);
    if (SmaplingHandle != NULL) (void)osMessageQueueReset(SmaplingHandle);
    /* 重置中值、低通滤波历史并重新启动四通道循环 DMA。 */
    App_Smapling_Init();
    /* 每隔 5 ms 处理一次，一共取三帧；首帧只使用新的 ADC 缓冲。 */
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        osDelay(5U);
        App_Smapling_Run();
    }
    /* 排空队列，留下这一轮检查中时间最近的完整光照帧。 */
    if (SmapToSystemHandle != NULL)
        while (osMessageQueueGet(SmapToSystemHandle, sample, NULL, 0U) == osOK)
            received = 1U;
    /* 没有新帧时不更新指标，调用者保持电机停转并回到 STOP。 */
    if (received != 0U) *light = light_analyze(sample);
    return received;
}

/* 执行一次或多次 STOP 周期：手动只等触摸，自动每次由 RTC 检查 A/B。 */
/* automatic 为 1 时启用 RTC；sample/light 用来带回最后一轮新光照结果。 */
/* 返回 1 表示触摸唤醒，2 表示 A/B 全不满足或 RTC 配置失败，3 表示故障请求。 */
static uint8_t power_stop(uint8_t automatic, SmaplingData_t *sample,
                          AppLightMetrics *light)
{
    /* 保存入睡前事件序号；中断只改序号，不直接恢复任务或点亮屏幕。 */
    uint32_t seen_touch = touch_sequence;
    uint32_t seen_rtc = rtc_sequence;
    /* adc_running 表示最后一次探测后 ADC 是否仍运行，避免重复启动 DMA。 */
    uint8_t result = 0U, adc_running = 0U;
    /* 先清除两轴运动目标和排队中的自动命令，防止唤醒后旧命令重启。 */
    MotorSpeed_Stop(SIMPLEFOC_AXIS_VERTICAL);
    MotorSpeed_Stop(SIMPLEFOC_AXIS_HORIZONTAL);
    if (SmapSystemToMotorHandle != NULL)
        (void)osMessageQueueReset(SmapSystemToMotorHandle);
    /* 停止电机周期任务后再关闭两轴 EN，避免任务重新使能驱动。 */
    if (MotorTaskHandle != NULL) (void)osThreadSuspend(MotorTaskHandle);
    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_VERTICAL);
    SimpleFOC_Motor_Disable(SIMPLEFOC_AXIS_HORIZONTAL);
    /* 停止采样任务及 DMA；STOP 中不能继续沿用旧的 ADC 缓冲。 */
    if (SmaplingTaskHandle != NULL) (void)osThreadSuspend(SmaplingTaskHandle);
    App_Smapling_Stop();
    if (SmapToSystemHandle != NULL) (void)osMessageQueueReset(SmapToSystemHandle);
    if (SmaplingHandle != NULL) (void)osMessageQueueReset(SmaplingHandle);
    /* 调试打印不参与唤醒判断，STOP 期间一并暂停。 */
    if (DebugTaskHandle != NULL) (void)osThreadSuspend(DebugTaskHandle);
    /* 保留原 LVGL 页面，只关闭背光并暂停 UI 刷新。 */
    power_screen_set(0U);
    do
    {
        /* 每轮 STOP 前后各读一次 RTC，计算真实经过时间以补偿系统节拍。 */
        uint32_t before, after, elapsed;
        /* 异常优先于低功耗；故障请求到来后退出本循环交还状态机处理。 */
        if (fault_requested != 0U)
        {
            result = 3U;
            break;
        }
        /* 先停用上次的唤醒定时器，自动模式再重装完整约三秒周期。 */
        (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
        if (automatic != 0U
            && HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, POWER_RTC_COUNTER,
                                           RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
        {
            /* RTC 设置失败时不冒险进入无法定时检查的 STOP。 */
            result = 2U;
            break;
        }
        seen_rtc = rtc_sequence;
        before = power_rtc_ms();
        /* 分别暂停 HAL 的 TIM6 节拍和 FreeRTOS 的 SysTick 中断。 */
        HAL_SuspendTick();
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
        /* 清除已有 SysTick 待处理中断，避免刚入 STOP 就被旧节拍唤醒。 */
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
        /* 关中断后复查触摸，消除“检查完触摸、尚未 WFI”之间的竞态。 */
        __disable_irq();
        if (touch_sequence == seen_touch)
            HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        /* 待处理的 RTC/触摸中断在这里执行回调并更新事件序号。 */
        __enable_irq();
        /* STOP 退出先恢复 HAL 节拍和 PLL 系统时钟，之后才使用常规外设。 */
        HAL_ResumeTick();
        SystemClock_Config();
        after = power_rtc_ms();
        /* 处理 RTC 跨午夜回零；一次 STOP 最多约三秒。 */
        elapsed = after >= before ? after - before : 86400000U - before + after;
        /* SysTick 中断暂停期间由 RTC 补回 RTOS 毫秒，保持任务延时语义。 */
        if (elapsed != 0U) (void)xTaskCatchUpTicks(elapsed);
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
        /* 同时收到触摸和 RTC 时触摸优先：只有触摸能点亮屏幕。 */
        if (touch_sequence != seen_touch)
        {
            result = 1U;
            break;
        }
        if (automatic != 0U && rtc_sequence != seen_rtc)
        {
            /* RTC 只短暂启动 ADC；没有新样本时不允许启动自动电机。 */
            if (power_probe(sample, light) != 0U)
            {
                adc_running = 1U;
                /* A、B 均不成立：退出 STOP 恢复追光，但保持黑屏。 */
                if (power_auto_idle(light) == 0U)
                {
                    result = 2U;
                    break;
                }
            }
            /* A 或 B 仍成立：关 ADC 后立即继续下一轮 STOP，不重计 15 秒。 */
            App_Smapling_Stop();
            adc_running = 0U;
        }
    } while (result == 0U);
    /* 真正离开 STOP 流程后取消 RTC 周期中断，正常运行不必每三秒检查。 */
    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    /* 丢弃本次探测留在队列中的旧帧，恢复运行后只处理持续采样的新帧。 */
    if (SmapToSystemHandle != NULL) (void)osMessageQueueReset(SmapToSystemHandle);
    if (SmaplingHandle != NULL) (void)osMessageQueueReset(SmaplingHandle);
    /* RTC 探测可能已经启动 ADC；只在未运行时重新初始化。 */
    if (adc_running == 0U) App_Smapling_Init();
    /* 电机先前已清除目标，恢复任务不会自动执行睡前运动命令。 */
    if (SmaplingTaskHandle != NULL) (void)osThreadResume(SmaplingTaskHandle);
    if (MotorTaskHandle != NULL) (void)osThreadResume(MotorTaskHandle);
    if (DebugTaskHandle != NULL) (void)osThreadResume(DebugTaskHandle);
    /* 背光是否开启由外层按唤醒原因决定；RTC 分支在此始终黑屏。 */
    return result;
}

/* 队列满时替换最旧角度命令，使电机任务最终使用最新光照。 */
static void auto_send_target(const AppMotorCommand *command)
{
    if (SmapSystemToMotorHandle == NULL) return;
    if (osMessageQueuePut(SmapSystemToMotorHandle, command, 0U, 0U) != osOK)
    {
        /* 被覆盖的旧命令不再代表当前光照。 */
        AppMotorCommand old_command;
        (void)osMessageQueueGet(SmapSystemToMotorHandle, &old_command, NULL, 0U);
        (void)osMessageQueuePut(SmapSystemToMotorHandle, command, 0U, 0U);
    }
}

/* 系统任务独占模式与光照分析，手动模式只更新数据而不发布光强命令。 */
void StartSystemStateTask(void *argument)
{
    /* 手动命令的当前方向、事件序号和最后续发时间。 */
    AppManualDirection active = APP_MANUAL_STOP;
    uint32_t seen_sequence = 0U, last_tick = 0U;
    /* 模式事件序号、自动模式进入时间和最新采样时间。 */
    uint32_t seen_mode_sequence = 0U, auto_enter_tick = 0U, last_sample_tick = 0U;
    /* 上次处理过的触摸序号及最近操作时间，手动 STOP 和全局黑屏共用后者。 */
    uint32_t seen_touch = touch_sequence, last_touch_tick = osKernelGetTickCount();
    /* 自动 A/B 首次连续成立的时间，与无触摸黑屏计时独立。 */
    uint32_t auto_idle_tick = 0U;
    /* auto_counting 表示自动 15 秒计时有效；was_held 用于捕获松手瞬间。 */
    uint8_t auto_counting = 0U, was_held = 0U;
    /* 采样超时后只锁定一次保持点。 */
    uint8_t sample_stale = 0U;
    /* 最新光照与其派生结果。 */
    SmaplingData_t sample;
    AppLightMetrics light = {0};
    (void)argument;
    /* 初始化背光 PWM；ARR 为 99 时 CCR 为 100 即持续高电平。 */
    power_backlight_set(POWER_NORMAL_BRIGHTNESS);
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    /* CubeMX 初始化时已启动 RTC；平时关闭周期中断，仅自动 STOP 时启用。 */
    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    for (;;)
    {
        /* 从 UI 邮箱取出本周期的模式与方向请求。 */
        AppMode requested_mode;
        AppManualDirection direction;
        uint32_t mode_sequence, sequence, tick;
        uint8_t sample_received = 0U, fault;
        /* 统一用本周期的 RTOS 毫秒值作倒计时基准，避免多处读数不一致。 */
        uint32_t now = osKernelGetTickCount();
        taskENTER_CRITICAL();
        fault = fault_requested;
        requested_mode = pending_mode;
        mode_sequence = pending_mode_sequence;
        direction = pending_direction;
        tick = pending_tick;
        sequence = pending_sequence;
        taskEXIT_CRITICAL();

        /* 触摸中断只记录序号；在任务上下文重置计时并决定是否点亮。 */
        if (touch_sequence != seen_touch)
        {
            seen_touch = touch_sequence;
            last_touch_tick = now;
            /* 自动 STOP 倒计时因用户触摸从零重新计算。 */
            auto_counting = 0U;
            power_screen_set(1U);
        }
        /* 长按期间不睡；松开时重新起算完整的无触摸 15 秒。 */
        if (was_held != 0U && touch_held == 0U) last_touch_tick = now;
        was_held = touch_held;

        if (fault != 0U && current_mode != APP_MODE_ERROR)
        {
            taskENTER_CRITICAL();
            current_mode = APP_MODE_ERROR;
            error_popup_pending = 1U;
            taskEXIT_CRITICAL();
            if (SmapSystemToMotorHandle != NULL)
                (void)osMessageQueueReset(SmapSystemToMotorHandle);
            MotorSpeed_Stop(SIMPLEFOC_AXIS_VERTICAL);
            MotorSpeed_Stop(SIMPLEFOC_AXIS_HORIZONTAL);
            active = APP_MANUAL_STOP;
            auto_counting = 0U;
        }

        if (current_mode != APP_MODE_ERROR && mode_sequence != seen_mode_sequence)
        {
            seen_mode_sequence = mode_sequence;
            if (requested_mode != current_mode)
            {
                taskENTER_CRITICAL();
                current_mode = requested_mode;
                taskEXIT_CRITICAL();
                if (SmapSystemToMotorHandle != NULL)
                    (void)osMessageQueueReset(SmapSystemToMotorHandle);
                (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_VERTICAL);
                (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_HORIZONTAL);
                active = APP_MANUAL_STOP;
                auto_enter_tick = osKernelGetTickCount();
                last_sample_tick = auto_enter_tick;
                sample_stale = 0U;
                /* 切换模式时不能继承上一个模式的自动倒计时或触摸时间。 */
                auto_counting = 0U;
                last_touch_tick = now;
            }
            seen_sequence = sequence;
        }

        /* 手动模式沿用按住速度控制，自动模式完全忽略方向事件。 */
        if (current_mode == APP_MODE_MANUAL && sequence != seen_sequence)
        {
            seen_sequence = sequence;
            last_tick = tick;
            if (direction != active)
            {
                if (active != APP_MANUAL_STOP
                    && (direction == APP_MANUAL_STOP
                        || manual_axis(direction) != manual_axis(active)))
                    (void)MotorPosition_HoldCurrent(manual_axis(active));
                active = APP_MANUAL_STOP;
                if (direction != APP_MANUAL_STOP)
                {
                    /* 方向符号与手动页保持一致。 */
                    float sign = direction == APP_MANUAL_UP ? MANUAL_UP_SIGN
                        : direction == APP_MANUAL_DOWN ? -MANUAL_UP_SIGN
                        : direction == APP_MANUAL_RIGHT ? MANUAL_RIGHT_SIGN
                        : -MANUAL_RIGHT_SIGN;
                    /* 本次按钮要控制的物理轴。 */
                    SimpleFOC_MotorAxis axis = manual_axis(direction);
                    if (MotorSpeed_SetTargetSpeed(axis, sign * MANUAL_SPEED_DPS / 360.0f) != 0U)
                        active = direction;
                    else
                        (void)MotorPosition_HoldCurrent(axis);
                }
            }
        }
        if (current_mode == APP_MODE_MANUAL && active != APP_MANUAL_STOP
            && (uint32_t)(osKernelGetTickCount() - last_tick) > MANUAL_HOLD_TIMEOUT_MS)
        {
            (void)MotorPosition_HoldCurrent(manual_axis(active));
            active = APP_MANUAL_STOP;
        }
        if (current_mode == APP_MODE_AUTO) seen_sequence = sequence;

        /* 排空采样队列，只分析本周期最新的一帧。 */
        if (SmapToSystemHandle != NULL)
        {
            while (osMessageQueueGet(SmapToSystemHandle, &sample, NULL, 0U) == osOK)
                sample_received = 1U;
        }
        if (sample_received != 0U)
        {
            light = light_analyze(&sample);
            last_sample_tick = osKernelGetTickCount();
            sample_stale = 0U;
        }

        /* 自动入睡只依据新鲜光照；进入自动后的三秒等待也不累计。 */
        if (current_mode == APP_MODE_AUTO)
        {
            /* 超过采样超时的旧帧不能继续维持 A/B 成立。 */
            uint8_t fresh = sample_stale == 0U
                && (uint32_t)(now - last_sample_tick) <= AUTO_SAMPLE_TIMEOUT_MS;
            if (fresh != 0U && touch_held == 0U
                && (uint32_t)(now - auto_enter_tick) >= AUTO_ENTER_DELAY_MS
                && power_auto_idle(&light) != 0U)
            {
                /* A 或 B 从不成立变为成立时记录起点；两者交替成立不重计。 */
                if (auto_counting == 0U)
                {
                    auto_counting = 1U;
                    auto_idle_tick = now;
                }
            }
            /* 同一帧数据使 A、B 都不成立，或采样失效时显示无穷秒。 */
            else auto_counting = 0U;
        }
        else auto_counting = 0U;

        /* 无触摸关屏前五秒变暗；触摸或持续手动运动会恢复正常亮度。 */
        if (screen_on != 0U)
        {
            if (touch_held == 0U && active == APP_MANUAL_STOP
                && (uint32_t)(now - last_touch_tick) >= POWER_IDLE_MS - POWER_DIM_MS)
            {
                if ((uint32_t)(now - last_touch_tick) >= POWER_IDLE_MS)
                    power_screen_set(0U);
                else power_backlight_set(POWER_NORMAL_BRIGHTNESS * POWER_DIM_PERCENT / 100U);
            }
            else power_backlight_set(POWER_NORMAL_BRIGHTNESS);
        }

        /* 手动无触摸到期或自动 A/B 连续成立到期，才真正进入 STOP。 */
        if ((current_mode == APP_MODE_MANUAL && touch_held == 0U
             && active == APP_MANUAL_STOP
             && (uint32_t)(now - last_touch_tick) >= POWER_IDLE_MS)
            || (current_mode == APP_MODE_AUTO && auto_counting != 0U
                && (uint32_t)(now - auto_idle_tick) >= POWER_AUTO_MS))
        {
            /* STOP 内部可能经历多次 RTC 短暂唤醒；返回才是状态转换。 */
            uint8_t reason = power_stop(current_mode == APP_MODE_AUTO,
                                        &sample, &light);
            now = osKernelGetTickCount();
            /* 旧光照时间和旧手动命令作废，恢复后等待新采样或新按键。 */
            last_sample_tick = now;
            auto_counting = 0U;
            active = APP_MANUAL_STOP;
            seen_sequence = pending_sequence;
            if (reason == 1U)
            {
                /* 触摸唤醒才开背光；RTC 唤醒返回时继续保持黑屏。 */
                seen_touch = touch_sequence;
                last_touch_tick = now;
                power_screen_set(1U);
            }
            continue;
        }

        if (current_mode == APP_MODE_AUTO && sample_stale == 0U
            && (uint32_t)(osKernelGetTickCount() - last_sample_tick) > AUTO_SAMPLE_TIMEOUT_MS)
        {
            if (SmapSystemToMotorHandle != NULL)
                (void)osMessageQueueReset(SmapSystemToMotorHandle);
            (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_VERTICAL);
            (void)MotorPosition_HoldCurrent(SIMPLEFOC_AXIS_HORIZONTAL);
            sample_stale = 1U;
        }

        if (current_mode == APP_MODE_AUTO && sample_received != 0U
            && (uint32_t)(osKernelGetTickCount() - auto_enter_tick) >= AUTO_ENTER_DELAY_MS)
        {
            /* 两轴都发送最新目标；电机任务只在首次进入零区间时锁定位置。 */
            AppMotorCommand command = {0};
            /* 四路都偏暗时保留双轴零度目标；否则沿用方向差比例控制。 */
            if (!(sample.left_top > AUTO_ALL_DARK_ADC
                && sample.left_bottom > AUTO_ALL_DARK_ADC
                && sample.right_top > AUTO_ALL_DARK_ADC
                && sample.right_bottom > AUTO_ALL_DARK_ADC))
            {
                if (light.horizontal_diff < -AUTO_HORIZONTAL_BAND
                    || light.horizontal_diff > AUTO_HORIZONTAL_BAND)
                    command.horizontal_deg = auto_limit_target(
                        AUTO_HORIZONTAL_SIGN * AUTO_LIGHT_KP * light.horizontal_diff);
                if (light.vertical_diff < -AUTO_VERTICAL_BAND
                    || light.vertical_diff > AUTO_VERTICAL_BAND)
                    command.vertical_deg = auto_limit_target(
                        AUTO_VERTICAL_SIGN * AUTO_LIGHT_KP * light.vertical_diff);
            }
            auto_send_target(&command);
        }
        osDelay(10U);
    }
}
