# 太阳能追光系统与双轴无刷电机驱动说明

## 1. 当前阶段与工作边界

本项目基于 STM32F407VETx、FreeRTOS CMSIS-RTOS V2 和 LVGL，目标是完成太阳能追光系统，并通过两个无刷电机驱动双轴云台。

本轮工作的范围是：

- 阅读项目 `word` 文件夹和云台配套资料；
- 核对当前工程的 `.ioc`、初始化代码、`SimpleFOC_Motor` 代码和配套 STM32 例程；
- 只整理和更新本 README；
- 不修改 `.c`、`.h`、`.ioc` 或 FreeRTOS 任务代码；
- 当前只完成无刷电机驱动初始化和 FOC 数学接口准备，不在实际任务中运行电机。

因此，当前代码不能被称为已经完成的闭环 FOC。AS5600 位置反馈、相电流采样、电流环、位置环、速度环和实际电机运行验证仍未完成。

## 2. 工程与资料路径

### 2.1 当前工程

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System
```

关键文件：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Solar_Light_Tracking_System.ioc
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\interface\SimpleFOC_Motor.c
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\interface\SimpleFOC_Motor.h
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\Src\tim.c
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\Src\gpio.c
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\Src\adc.c
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\Src\MotorTask.c
```

### 2.2 云台资料

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\云台资料V1.0-M创动工坊\云台资料V1.0-M创动工坊
```

主要资料：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\云台资料V1.0-M创动工坊\云台资料V1.0-M创动工坊\说明.txt
```

配套 STM32 例程：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\云台资料V1.0-M创动工坊\云台资料V1.0-M创动工坊\测试源码\STM32HALFOCdouble双电机驱动HAL库\STM32HALFOCdouble双电机驱动HAL库
```

例程中重点核对了：

```text
...\STM32HALFOCdouble双电机驱动HAL库.ioc
...\Core\Src\tim.c
...\Core\Src\adc.c
...\User\Device\foc_dev.c
...\libs\lib\foc_dev.c
```

## 3. 双轴电机硬件映射

| 轴 | PWM 定时器 | 三相 PWM 引脚 | EN 引脚 |
| --- | --- | --- | --- |
| 上下轴 | TIM1 CH1/CH2/CH3 | PE9、PE11、PE13 | PE7，`TIM1_EN` |
| 左右轴 | TIM8 CH1/CH2/CH3 | PC6、PC7、PC8 | PE8，`TIM8_EN` |

当前定时器配置为预分频 `0`、自动重装值 `11200 - 1`、PWM1、高电平有效，PWM 频率约为 15 kHz。

## 4. 三路 PWM 与互补 PWM 的结论

### 4.1 资料和例程已经确认的内容

资料文件：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\云台资料V1.0-M创动工坊\云台资料V1.0-M创动工坊\说明.txt
```

其中明确写有：

```text
其他MCU，只要能每路驱动板（电机）能提供3路独立的PWM，即可使用
```

配套 STM32 例程的 `.ioc` 只配置了：

```text
TIM2_CH1、TIM2_CH2、TIM2_CH3、TIM2_CH4
TIM3_CH1、TIM3_CH2
```

例程文件 `User/Device/foc_dev.c` 启动 PWM 时只使用：

```c
HAL_TIM_PWM_Start(tim, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(tim, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(tim, TIM_CHANNEL_3);
```

没有发现以下配置或调用：

```c
TIMx_CHxN
HAL_TIMEx_PWMN_Start()
```

所以可以确认：电调的 MCU 输入侧要求每个电机提供三路独立的普通 PWM。当前双轴系统就是两个定时器、每个定时器三路 PWM。

### 4.2 不能直接从资料确认的内容

目前没有找到驱动板完整原理图、功率级驱动芯片型号或芯片数据手册。因此，不能仅凭“三路 PWM 输入”百分之百确认驱动板内部具体使用了哪种互补逻辑。

合理的硬件解释是：驱动板接收三路相位 PWM 后，在板内完成三相半桥的上下管驱动、互补逻辑和防直通死区；也可能由集成式三相电机驱动芯片内部完成这些工作。但具体方案仍需要以下材料或实测确认：

- 驱动板原理图；
- 栅极驱动器或电机驱动芯片型号；
- 芯片数据手册；
- 示波器观察功率管上下管栅极波形。

因此，当前结论应写成：三路普通 PWM 输入已被资料和例程确认；驱动板内部是否生成互补 PWM、由哪一个芯片生成以及死区具体是多少，尚未被完整原理图或波形证实。

## 5. 当前 `.ioc` 是否配置了互补 PWM

当前工程文件：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Solar_Light_Tracking_System.ioc
```

配置的是：

```text
PC6.Signal=S_TIM8_CH1
PC7.Signal=S_TIM8_CH2
PC8.Signal=S_TIM8_CH3

PE9.Signal=S_TIM1_CH1
PE11.Signal=S_TIM1_CH2
PE13.Signal=S_TIM1_CH3
```

没有发现：

```text
TIM1_CH1N
TIM1_CH2N
TIM1_CH3N
TIM8_CH1N
TIM8_CH2N
TIM8_CH3N
```

当前 `Core/interface/SimpleFOC_Motor.c` 也只调用：

```c
HAL_TIM_PWM_Start(axis->timer, axis->channel_a);
HAL_TIM_PWM_Start(axis->timer, axis->channel_b);
HAL_TIM_PWM_Start(axis->timer, axis->channel_c);
```

没有调用：

```c
HAL_TIMEx_PWMN_Start(...)
```

结论：当前 MCU 没有配置和输出 TIM1/TIM8 的 CHxN 互补 PWM，MCU 侧只输出六路普通 PWM，即每个电机三路。

## 6. DeadTime、硬件死区和软件死区补偿

### 6.1 当前 MCU 的硬件 DeadTime 状态

当前文件：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\Src\tim.c
```

TIM1 和 TIM8 都存在：

```c
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
HAL_TIMEx_ConfigBreakDeadTime(...)
```

但是实际配置为：

```c
sBreakDeadTimeConfig.DeadTime = 0;
```

这说明代码调用了高级定时器的 Break/DeadTime 配置接口，但当前 DeadTime 数值为零。由于 CHxN 也没有配置，因此不能把它理解为当前 MCU 已经输出了带死区的互补 PWM。

### 6.2 硬件死区的作用

对于一个半桥，上管和下管不能在同一时刻导通。上管关闭后，要等待一小段时间，确认它已经真正关断，再打开下管；下管切换时也一样。这段空白时间就是硬件 DeadTime。

它的主要作用是防止上下功率管同时导通，避免直流母线被直接短路，这种故障通常称为桥臂直通。

### 6.3 软件死区补偿的作用

软件死区补偿不是用来替代硬件死区保护，而是用来补偿死区带来的平均电压误差。

死区期间，PWM 指令与实际电机端电压之间会产生偏差。这个偏差在低速、小电压和小电流时尤其明显，可能造成：

- 电流波形畸变；
- 转矩脉动；
- 低速运行不平稳；
- 电流 PI 控制器误差增大；
- 电机出现啸叫或抖动。

软件补偿通常需要知道相电流方向，根据电流正负修正对应相的占空比。因此，没有相电流采样时不能可靠地做基于电流方向的补偿。

### 6.4 当前是否需要加入软件死区补偿

当前硬件只向电调输入三路 PWM，驱动板很可能在内部完成互补驱动和死区控制。此时 MCU 不应在没有驱动板参数、没有相电流采样、没有实测波形的情况下盲目加入软件死区补偿，否则可能把驱动板已经完成的处理再次补偿，导致占空比误差。

当前阶段的合理结论是：

- MCU 侧互补 PWM：未配置；
- MCU 侧硬件 DeadTime：未启用，当前 `DeadTime=0`；
- 驱动板内部互补和死区：需要原理图、芯片手册或示波器确认；
- 软件死区补偿：当前未实现，也暂时不具备可靠实现条件。

## 7. EN 电平有效性的证据范围

当前工程的 EN 初始化位于：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\Src\gpio.c
```

初始化时写入：

```c
HAL_GPIO_WritePin(GPIOE, TIM1_EN_Pin|TIM8_EN_Pin, GPIO_PIN_SET);
```

当前 `SimpleFOC_Motor.c` 按以下逻辑设计：

```text
GPIO_PIN_SET   = 关闭电调
GPIO_PIN_RESET = 允许电调工作
```

也就是说，当前代码假设 EN 为低电平有效。

但是，在配套资料、例程 `.ioc`、`User/Device/foc_dev.c` 和 `libs/lib/foc_dev.c` 中没有找到明确的 EN 引脚控制或极性说明。因此，不能说“EN 低电平有效”已经由例程代码确认。

必须通过以下一种方式最终确认：

- 查看驱动板丝印和接口定义；
- 查看驱动芯片数据手册；
- 查看驱动板原理图；
- 上电后使用示波器或万用表结合电机是否使能进行实测。

当前 README 中的准确表述是：代码按 EN 低有效编写，但硬件极性尚未由配套例程完全证实。

## 8. AS5600 上电零位校准是否合理

AS5600 用于测量转子的机械角度，不能直接测量电流，也不能自动告诉控制器“当前机械角对应哪一个电角度零点”。

配套例程中的参考函数位于：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\云台资料V1.0-M创动工坊\云台资料V1.0-M创动工坊\测试源码\STM32HALFOCdouble双电机驱动HAL库\STM32HALFOCdouble双电机驱动HAL库\User\Device\foc_dev.c
```

函数：

```c
void FOC_AlignmentSensor(FOC_t *hfoc)
```

其核心思路是：

1. 给定一个固定的电压或磁场方向；
2. 让转子吸合并等待稳定；
3. 读取 AS5600 当前角度；
4. 根据固定电角度和传感器读数计算电角度零点；
5. 保存零点，并根据机械角变化判断方向。

因此，上电时做一次电角度零点校准是合理的，尤其适合传感器安装角度不确定或每次上电机械位置可能变化的系统。

它主要解决的是：

- AS5600 磁铁与定子之间的安装偏移；
- 电机相序与传感器正方向的关系；
- 机械角到电角度的换算起点；
- 极对数和电角度方向配置。

它不应简单理解为每次都在修正 AS5600 芯片本身的零漂。若机械结构和传感器安装稳定，可以首次校准后保存到 Flash；若装配位置可能变化，则上电重新校准更稳妥。

当前主工程还没有接入 AS5600 读取和零位校准，因此这项功能属于未完成任务。

## 9. 三相电流如何采样

AS5600 只负责位置角度测量，不能代替三相电流采样。

三相电流采样一般通过电机驱动板上的采样电阻、运算放大器和 ADC 输入完成。也就是说，需要在硬件上另外接 ADC 采样线，或者使用驱动板已经引出的电流反馈接口。

配套 STM32 例程中可以看到 ADC 输入：

```text
PB0 -> M0_ADC1
PB1 -> M0_ADC2
PA4 -> M1_ADC1
PA5 -> M1_ADC2
```

例程使用 ADC1 DMA 采样。在配套 `libs/lib/foc_dev.c` 中，电流控制使用了至少两路相电流：

```c
float I_alpha = CurrentData.I_a;
float I_beta = _1_SQRT3 * CurrentData.I_a
             + _2_SQRT3 * CurrentData.I_b;
```

对于理想三相平衡系统，可以用：

```text
Ic = -(Ia + Ib)
```

由两路相电流推导第三路，所以工程不一定必须使用三个 ADC 通道，但必须确认驱动板的采样拓扑、采样时刻、放大倍数、零点偏置和 ADC 量程。

当前主工程的 ADC 主要用于光敏电阻输入，不是电机相电流采样，因此当前还不能实现可靠的电流环和基于电流方向的软件死区补偿。

## 10. `SimpleFOC_Motor` 代码来源与是否可以引入官方 SimpleFOC

当前文件：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\interface\SimpleFOC_Motor.c
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\Core\interface\SimpleFOC_Motor.h
```

这部分代码是针对本项目 STM32 HAL、TIM1/TIM8、两个 EN 引脚和 CMSIS-DSP 环境编写的项目适配代码，不是直接复制官方 Arduino SimpleFOC 完整运行库。

当前已经准备的内容包括：

- 双轴 PWM 硬件资源映射；
- PWM 启动、停止和中性占空比输出；
- EN 控制；
- Clarke 变换；
- Park 变换；
- 逆 Park 变换；
- SVPWM 数学计算；
- 使用 CMSIS-DSP 的正弦/余弦、最大值和最小值函数。

理论上可以引入官方 SimpleFOC 的算法或参考实现，但不能把 Arduino 版本直接当作 STM32 HAL 工程代码使用。仍然需要适配：

- STM32 定时器和三路 PWM 输出；
- 当前电调的三路 PWM 输入协议；
- EN 极性和停止状态；
- AS5600 的 I2C 读取；
- ADC 相电流采样；
- FreeRTOS 调度；
- 极对数、方向和电角度零点；
- 故障保护、限流和机械限位。

所以，当前项目可以借鉴 SimpleFOC 的控制算法，但是否直接引入完整官方库，要等硬件接口和任务架构确定后再决定。

## 11. 当前初始化行为

`SimpleFOC_Motor_Init()` 在 `main()` 完成 TIM1、TIM8 初始化后调用，主要执行：

1. 启动 TIM1 的 CH1、CH2、CH3 普通 PWM；
2. 启动 TIM8 的 CH1、CH2、CH3 普通 PWM；
3. 把各相比较值设置为定时器周期的一半，即约 50% 占空比；
4. 将两个 EN 保持在关闭状态；
5. 初始化失败时停止已启动的 PWM，并恢复安全输出。

50% 占空比这里只是中性初始化值，不代表已经给电机施加了有效转矩。当前 `MotorTask` 只保留任务入口和周期休眠，不调用使能、不修改 PWM，也不运行 FOC 数学接口。

## 12. FOC 数学接口说明

数学函数位于 `Core/interface/SimpleFOC_Motor.c`。能使用 CMSIS-DSP 的部分优先使用 DSP，简单标量组合和占空比限幅保留直接计算。

### 12.1 Clarke 变换

输入三相量 `Ia、Ib、Ic`，输出静止坐标系中的 `alpha、beta`：

```text
alpha = Ia - (Ia + Ib + Ic) / 3
beta  = (Ib - Ic) / sqrt(3)
```

三相绕组彼此相差 120 度。Clarke 变换把三相信息转换成二维表示，减少后续计算量。`alpha、beta` 仍然完整描述三相系统，只是换成了更适合程序处理的坐标。

### 12.2 Park 变换

输入 `alpha、beta` 和电角度，输出跟随转子旋转的 `d、q` 坐标：

```text
d = alpha × cos(angle) + beta × sin(angle)
q = -alpha × sin(angle) + beta × cos(angle)
```

通常可以把 d 轴理解为与转子磁链方向相关的轴，q 轴为与磁链垂直的轴。q 轴分量通常主要影响电磁转矩。代码使用 `arm_sin_cos_f32()` 计算正弦和余弦，并处理弧度与角度单位的转换。

### 12.3 逆 Park 变换

逆 Park 把控制器在 d/q 坐标中得到的电压指令恢复为静止坐标系：

```text
alpha = d × cos(angle) - q × sin(angle)
beta  = d × sin(angle) + q × cos(angle)
```

完成这一步后，`alpha、beta` 电压参考才能交给 SVPWM 生成三相 PWM 占空比。

### 12.4 SVPWM

SVPWM 将 `alpha、beta` 电压参考转换为三相占空比：

```text
Ua = alpha
Ub = -0.5 × alpha + 0.8660254 × beta
Uc = -0.5 × alpha - 0.8660254 × beta
common = -0.5 × (max(Ua, Ub, Uc) + min(Ua, Ub, Uc))
duty = 0.5 + (U + common) / supply_voltage
```

`common` 是三相共同偏置。三相同时增加或减少相同电压，不改变相间电压，但可以提高母线电压利用率。代码使用 `arm_max_f32()` 和 `arm_min_f32()` 获取三相最大值和最小值，再将占空比限制在 `0.0` 到 `1.0`。

当母线电压小于等于零，或输入输出指针为空时，SVPWM 会把能够写入的占空比设置为 `0.5`，避免非法除法和异常输出。

## 13. 无刷电机驱动完整流程

当前只完成了初始化和数学接口，下面是后续完整闭环控制应遵循的顺序：

```text
上电安全初始化
    -> PWM 输出中性值
    -> EN 保持关闭
    -> ADC、I2C、定时器和保护初始化
    -> AS5600 读取
    -> 电角度零位校准
    -> 读取相电流并去除 ADC 零点偏置
    -> Clarke 变换
    -> Park 变换
    -> d/q 电流 PI 控制
    -> 逆 Park 变换
    -> SVPWM
    -> 更新三相 PWM 比较值
    -> 保护判断和故障处理
```

各步骤作用如下：

1. **安全初始化**：确保 MCU 复位、初始化失败或任务尚未启动时，电机不会突然转动。
2. **PWM 中性输出**：给三路 PWM 设置安全的中性值，避免初始化期间产生明显相间电压。
3. **EN 保持关闭**：电机驱动板未完成校准和参数检查前，不允许功率级工作。
4. **AS5600 读取**：获得转子的机械角度。
5. **电角度零位校准**：建立传感器机械角与电机电角度之间的对应关系。
6. **相电流采样**：获得电机实际电流，用于闭环控制和过流保护。
7. **Clarke 变换**：把三相电流变为 `alpha、beta` 二维静止坐标。
8. **Park 变换**：把静止坐标变为跟随转子的 d/q 坐标，使磁链和转矩控制更容易分开。
9. **电流 PI 控制**：比较目标电流和实际电流，计算需要施加的 d/q 电压。
10. **逆 Park 变换**：把 d/q 电压转换回 `alpha、beta` 电压。
11. **SVPWM**：把二维电压参考转换为三相 PWM 占空比。
12. **更新 PWM**：将占空比写入定时器比较寄存器，驱动电调。
13. **保护判断**：检查过流、过压、欠压、传感器异常、通信异常和机械限位，异常时关闭 EN 或输出安全值。

## 14. 当前完成情况

| 模块当前状态 | 状态 |
| --- | --- |
| GPIO 初始化 | 已完成 |
| TIM1 三路普通 PWM 初始化 | 已完成 |
| TIM8 三路普通 PWM 初始化 | 已完成 |
| TIM1/TIM8 互补 PWM | 未配置 |
| MCU CHxN 输出 | 未配置 |
| MCU 硬件 DeadTime | 未启用，当前 `DeadTime=0` |
| 驱动板内部互补 PWM/死区 | 需原理图、芯片手册或波形确认 |
| 电调 EN 控制代码 | 已完成，但 EN 极性仍需硬件确认 |
| 中性 PWM 输出 | 已完成 |
| Clarke 变换 | 已完成 |
| Park 变换 | 已完成 |
| 逆 Park 变换 | 已完成 |
| SVPWM 数学计算 | 已完成 |
| CMSIS-DSP 基础函数使用 | 已完成 |
| AS5600 位置读取 | 当前工程未接入 |
| AS5600 上电零位校准 | 当前工程未实现，配套例程有参考 |
| 三相电流采样 | 当前工程未接入 |
| 电流 PI 控制器 | 尚未实现 |
| 软件死区补偿 | 尚未实现，当前也缺少相电流方向输入 |
| 实际电机控制任务 | 尚未实现 |
| PWM 占空比写入控制任务 | 尚未接入 |
| 位置环 | 尚未实现 |
| 速度环 | 尚未实现 |
| 完整闭环 FOC | 尚未完成 |
| 实际电机安全验证 | 尚未完成 |

## 15. 关键代码

### 15.1 PWM 启动

```c
HAL_TIM_PWM_Start(axis->timer, axis->channel_a);
HAL_TIM_PWM_Start(axis->timer, axis->channel_b);
HAL_TIM_PWM_Start(axis->timer, axis->channel_c);
```

当前只启动每个定时器的三个普通通道，没有启动 CHxN 互补通道。

### 15.2 中性 PWM

```c
neutral_compare = (period + 1U) / 2U;
```

将比较值设置为周期的一半，得到约 50% 占空比，作为初始化阶段的中性输出。

### 15.3 EN 安全状态

```c
HAL_GPIO_WritePin(GPIOE, TIM1_EN_Pin | TIM8_EN_Pin, GPIO_PIN_SET);
```

当前代码把高电平当作关闭状态，把低电平当作允许驱动状态。硬件极性仍需确认。

### 15.4 CMSIS-DSP 调用

```c
arm_sin_cos_f32();
arm_max_f32();
arm_min_f32();
```

分别用于 Park/逆 Park 的正弦余弦计算，以及 SVPWM 三相参考值的最大值和最小值计算。

### 15.5 两相电流推导第三相

```text
Ic = -(Ia + Ib)
```

这是三相电流和为零时的理论关系。实际使用前必须确认采样拓扑、零点偏置、采样同步和硬件增益。

## 16. 后续实施顺序

建议按以下顺序继续开发：

1. 获取驱动板原理图、芯片型号和接口定义；
2. 确认 EN 电平、PWM 停止值、PWM 中性值和内部死区；
3. 用示波器确认三路 PWM 输入和驱动板功率级波形；
4. 接入 AS5600 I2C 读取；
5. 完成电机方向、极对数和电角度零位校准；
6. 确认电机驱动板的相电流采样接口；
7. 接入 ADC 触发、DMA 和电流零点校准；
8. 完成 Clarke、Park、逆 Park 到 PWM 更新的定时同步；
9. 实现 d/q 电流 PI 控制器；
10. 在低电压、限流和机械限位条件下进行单轴测试；
11. 再接入位置环、速度环和太阳能追光逻辑；
12. 最后进行双轴联动和整机安全验证。

## 17. 本轮更新说明

本轮只更新了：

```text
D:\A_RTOS_MCU\project\Solar_Light_Tracking_System\Solar_Light_Tracking_System\word\README.md
```

本轮没有修改任何 `.c`、`.h`、`.ioc` 或 FreeRTOS 任务代码。README 中已经同步当前聊天中形成的主要结论，并明确区分了：

- 已由工程代码和配套例程确认的事实；
- 根据三路 PWM 架构做出的合理推断；
- 仍需驱动板原理图、芯片手册或示波器确认的硬件内容；
- 当前已经完成和尚未完成的工作。
