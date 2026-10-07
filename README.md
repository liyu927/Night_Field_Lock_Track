# Solar Light Tracking System

这是一个基于 STM32F407、FreeRTOS 和 HAL 库的太阳光追踪系统工程。当前工程包含两组三相无刷电机 PWM 资源：

- TIM1：竖直轴，三相输出为 `PE9 / PE11 / PE13`，使能脚为 `PE7`。
- TIM8：水平轴，三相输出为 `PC6 / PC7 / PC8`，使能脚为 `PE8`。

## 当前实现

### 光照采样任务

已完成：

- 新增 `Core/App/SmaplingTask.c`。`SmaplingTask` 由 FreeRTOS 创建，启动 ADC1 的 PA0~PA3 DMA 循环采样。
- 新增 `Core/Types/SmaplingTypes.h`。`SmaplingData_t` 按 `left_top`、`left_bottom`、`right_top`、`right_bottom` 保存 PA0、PA1、PA2、PA3 的滤波值。
- 新增 `Core/common/filter.c/.h`，提供一阶低通、标量卡尔曼和三点中值滤波函数。
- 在 `Solar_Light_Tracking_System.ioc` 中创建 `SmaplingTask` 和 `Smapling` 消息队列；队列长度为 16，元素类型为 `SmaplingData_t`。
- `SmaplingTask` 每 20 ms 对四路 DMA 数据执行一阶低通后写入 `Smapling` 队列。`UiTask` 不再启动 ADC，也不再直接读取 DMA 缓冲。
- 观察者页面每 200 ms 清空 `Smapling` 队列并显示其中最新的四路滤波结果，避免屏幕显示积压的旧数据。

关键采样代码：

```c
ADC1_DMA_Start();

smapling_data.left_top = Filter_LowPass(&left_top_filter, adc1_dma_buf[0], 3U);
smapling_data.left_bottom = Filter_LowPass(&left_bottom_filter, adc1_dma_buf[1], 3U);
smapling_data.right_top = Filter_LowPass(&right_top_filter, adc1_dma_buf[2], 3U);
smapling_data.right_bottom = Filter_LowPass(&right_bottom_filter, adc1_dma_buf[3], 3U);
osMessageQueuePut(SmaplingHandle, &smapling_data, 0U, 0U);
```

一阶低通的系数为 `1/8`，兼顾光敏电阻读数稳定性和追光响应。三点中值滤波适合抑制偶发尖峰，已提供函数，暂未放入实时采样链路。卡尔曼滤波同样已提供，但需要依据实测噪声调整过程噪声和测量噪声，当前不建议直接启用。

### 触摸轮询与 CTP_INT 中断

已完成：

- 保留 FT6336 的 I2C 轮询模式，LVGL 仍通过 `lv_port_indev.c` 的输入设备回调读取触摸坐标和按下状态。
- `PE15` 配置为 `CTP_INT`，上拉输入、下降沿 EXTI15_10 中断，NVIC 优先级为 5。
- `EXTI15_10_IRQHandler()` 调用 `HAL_GPIO_EXTI_IRQHandler(CTP_INT_Pin)`，再进入 `HAL_GPIO_EXTI_Callback()`。
- 回调会识别 `CTP_INT_Pin`，当前不执行计数、I2C、FreeRTOS 或 LVGL 操作；这是后续低功耗触摸唤醒逻辑的扩展点。

关键中断代码：

```c
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(CTP_INT_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == CTP_INT_Pin)
    {
        /* 后续低功耗触摸唤醒逻辑放在这里。 */
    }
}
```

未完成：低功耗进入、触摸唤醒后的时钟和外设恢复、屏幕背光恢复，以及系统状态机与低功耗流程的联动。

### 后续未完成任务

- 四路光照的平均值、总差异、左右差和上下差计算。
- 光敏电阻开路、短路或满量程异常检测。
- 自动追光状态机、阈值判断和双轴电机控制命令。
- 参数保存、Flash 双备份与参数校验。
- 正式的自动模式、手动模式、异常模式和低功耗 UI 页面。

本次已经在实际 `MotorTask` 中加入 TIM8 水平轴的无刷电机开环驱动：

1. 上电完成外设初始化后，`main()` 调用 `SimpleFOC_Motor_Init()`。
2. TIM1 和 TIM8 的三路 PWM 都启动，但两个电机的 EN 脚先保持低电平，电机处于禁止状态。
3. FreeRTOS 启动后，`StartMotorTask()` 只启动 TIM8 对应的水平轴。
4. 软件生成一个虚拟电角度，不读取 AS5600，也不读取 ADC 电流。
5. 虚拟电角度按照设定机械转速推进，通过逆 Park 和 SVPWM 生成三相占空比。
6. TIM8 的 `PE8` 拉高后，电调开始响应 `PC6 / PC7 / PC8` 的三相 PWM。
7. 电机速度从 0 逐渐升高到约 15 rpm，避免刚上电时突然施加较大的转速指令。

这属于**电压型无位置传感器开环驱动**，不是力矩、速度、位置三级闭环。当前控制不使用 AS5600 的位置反馈，也不使用 ADC 电流采样保护。

## 开环参数

参数集中在 `Core/App/MotorTask.c` 文件顶部：

| 宏 | 当前值 | 作用 |
|---|---:|---|
| `TIM8_OPEN_LOOP_SPEED_RPS` | `0.25f` | 目标机械转速，0.25 转/秒约等于 15 rpm |
| `TIM8_OPEN_LOOP_VOLTAGE` | `2.0f` | q 轴电压幅值，电压越大，平均转矩通常越大 |
| `TIM8_OPEN_LOOP_POLE_PAIRS` | `7U` | 电机极对数，14 极电机通常填写 7 |
| `TIM8_OPEN_LOOP_DIRECTION` | `1` | 软件电角度方向，改为 `-1` 可反向 |
| `TIM8_OPEN_LOOP_RAMP_TIME_MS` | `1500U` | 从静止升到目标速度的时间 |
| `TIM8_OPEN_LOOP_ANGLE_OFFSET_DEG` | `0.0f` | 电角度偏移量，单位为度，用于调整磁场相位 |
| `MOTOR_TASK_PERIOD_MS` | `1U` | 电机任务更新周期 |

开环的核心关系为：

```text
电角速度 = 机械转速 × 极对数 × 2π
电角度   = 电角度 + 电角速度 × 时间间隔
```

代码使用虚拟电角度和 q 轴电压生成电压矢量，再使用现有的 `SimpleFOC_SVPWM()` 把电压矢量变为 TIM8 三相 PWM。

### 电角度偏移量怎么调

当前代码在 `Core/App/MotorTask.c` 顶部定义：

```c
#define TIM8_OPEN_LOOP_ANGLE_OFFSET_DEG 0.0f
```

它会在虚拟电角度送入逆 Park 之前加入一个固定相位。例如：

- `0.0f`：当前相位基准；
- `30.0f`：在当前基准上提前 30 个电角度；
- `-30.0f`：在当前基准上滞后 30 个电角度；
- `90.0f`：相位相差四分之一电周期。

这个值不能只根据电机机械结构直接推算出来，因为它同时受电机 U/V/W 相线顺序、驱动板相位定义、转轴观察方向以及电机的电气零位影响。建议先从 `0.0f` 开始，电机脱离负载，在低电压和低转速下每次改变 10 到 15 度，观察启动是否更平稳、声音是否更小、抖动是否减轻、同样电压下是否更容易带动负载。找到较好区域后，再用 2 到 5 度步进细调。

偏移量应在顺时针和逆时针测试中保持不变；反转只修改 `TIM8_OPEN_LOOP_DIRECTION`。如果电机实际方向相反，先改方向宏，不要用偏移量代替方向设置。

### 能不能用正转和反转速度是否相同来判断偏移量

不能仅靠“正转和反转的机械速度是否相同”准确判断偏移量。当前是开环，软件按照设定的电角速度推进磁场，正反转的目标速度本来就由同一个 `TIM8_OPEN_LOOP_SPEED_RPS` 决定；只要没有明显失步，测速结果接近是正常现象。固定偏移量主要影响的是磁场与转子之间的相位关系、启动转矩、振动、噪声和失步裕量，而不是直接改变软件给定速度。

正反转对比可以作为辅助检查：在相同的电压、目标速度、加速时间和负载条件下，如果某个方向明显更容易启动、噪声更小、实际转速更接近指令，说明相位、相序、机械负载或装配存在方向性差异。但这不能单独证明偏移量已经正确。更可靠的调法是固定一个方向，逐步扫描偏移量并比较启动电流、振动、噪声、带载能力和失步情况；当前代码未启用 ADC 电流采样，因此只能使用外部电流表、示波器或驱动器反馈进行电流比较。

## TIM8 接线

| STM32F407 引脚 | 外设功能 | 作用 |
|---|---|---|
| `PC6` | `TIM8_CH1` | 电机 U/A 相 PWM |
| `PC7` | `TIM8_CH2` | 电机 V/B 相 PWM |
| `PC8` | `TIM8_CH3` | 电机 W/C 相 PWM |
| `PE8` | `TIM8_EN` | 电调使能，高电平有效 |

三路 PWM 应连接到电调或三相驱动板的三个输入端，不能直接连接无刷电机绕组。电机绕组应接在驱动板的 U/V/W 或 A/B/C 输出端，驱动板还必须与 STM32 共地。

TIM8 的 PWM 周期来自 `Core/Src/tim.c`：

- 定时器时钟：约 168 MHz
- 预分频：`0`
- 自动重装载值：`11200 - 1`
- PWM 频率：约 15 kHz

## 顺时针方向说明

程序中的 `TIM8_OPEN_LOOP_DIRECTION = 1` 表示按当前 U/V/W 相序推进电角度。电机实际是顺时针还是逆时针，还与以下因素有关：

- 电机 U/V/W 三相线的接线顺序；
- 电机轴的观察方向；
- 电机安装方向；
- 驱动板输入与输出相位定义。

因此第一次上电必须让电机脱离负载，并准备好立即断电。若观察到实际方向与期望相反，优先将 `TIM8_OPEN_LOOP_DIRECTION` 改为 `-1` 重新编译；也可以交换任意两根电机相线，但不要同时修改软件方向和相线，否则方向会再次翻转。

## 安全调试建议

1. 第一次测试时拆下负载，确保电机轴不会夹伤人员或碰撞机构。
2. 先确认驱动板电源、电机电源和 STM32 地线连接正确。
3. 如果电机只抖动不转，先检查极对数、三相线序和 `TIM8_OPEN_LOOP_VOLTAGE`。
4. 如果电机启动困难，可以小幅提高电压，例如从 `2.0f` 调到 `2.5f`；不要一次调得过大。
5. 如果电机转速过高，降低 `TIM8_OPEN_LOOP_SPEED_RPS`；如果实际转向相反，把方向改为 `-1`。
6. 开环没有位置反馈，负载增大时可能失步；不能把当前实现当作具有堵转保护的闭环驱动。
7. TIM1 轴目前没有在 `MotorTask` 中启动，仍保持关闭。

## 相关文件

- `Core/App/MotorTask.c`：FreeRTOS 电机任务，配置并更新 TIM8 开环运行。
- `Core/interface/SimpleFOC_Motor.h`：电机驱动接口和开环参数结构体。
- `Core/interface/SimpleFOC_Motor.c`：PWM、逆 Park、SVPWM 和开环电角度推进实现。
- `Core/Src/tim.c`：TIM1/TIM8 三相 PWM 初始化。
- `Core/Src/gpio.c`：TIM1/TIM8 使能脚初始化。
- `Solar_Light_Tracking_System.ioc`：STM32CubeMX 外设和引脚配置。

## 构建

工程使用 CMake 和 `arm-none-eabi-gcc`。在已经配置好 ARM GCC 工具链的环境中，可以执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

构建成功后生成的 ELF 文件位于 `build/Debug/Solar_Light_Tracking_System` 对应的构建目录中。
