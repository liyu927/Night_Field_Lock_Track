# 太阳能追光系统文档说明

## 文档入口

- 产品方案：[Product_Plan/product_plan.md](Product_Plan/product_plan.md)
- 技术设计方案：[Technical_Design/太阳能追光系统技术设计方案.docx](Technical_Design/太阳能追光系统技术设计方案.docx)

## 当前完成情况

技术设计方案已经完成，并已根据产品方案、`Solar_Light_Tracking_System.ioc`、CubeMX 生成代码和当前 LVGL 工程基线进行复核。技术方案描述的是代码实现设计，不代表应用层代码已经全部编写完成。

当前已确认的工程基础包括：

- STM32F407VETx、168 MHz 时钟、FreeRTOS CMSIS-RTOS V2 和 LVGL 工程基线。
- 四路光敏电阻 ADC1 DMA 采样，以及手动、自动、异常和低功耗状态的产品设计。
- TIM1/TIM8 各三路 PWM 和两个电机 EN 引脚的硬件映射。
- W25Q64 参数保存、LVGL 页面和后续自动追光控制的设计文档。

当前电机代码只完成“无刷电机驱动初始化”和数学接口准备，不在 FreeRTOS 任务中实际驱动电机。AS5600 读取、零位校准、位置环、运行时 SVPWM 更新和运行时使能均未接入当前任务。

## 双轴电机硬件映射

| 轴 | PWM 定时器 | 三相 PWM 引脚 | EN 引脚 |
| --- | --- | --- | --- |
| 上下轴 | TIM1 CH1/CH2/CH3 | PE9、PE11、PE13 | PE7，`TIM1_EN` |
| 左右轴 | TIM8 CH1/CH2/CH3 | PC6、PC7、PC8 | PE8，`TIM8_EN` |

当前定时器配置为预分频 `0`、自动重装值 `11200 - 1`、PWM1、高电平有效，PWM 频率约为 15 kHz。EN 为高电平关闭、低电平允许电调工作的逻辑。

## 当前初始化行为

`SimpleFOC_Motor_Init()` 在 `main()` 完成 TIM1、TIM8 初始化后调用，执行以下步骤：

1. 启动 TIM1 和 TIM8 的六路 PWM。
2. 将每个通道的比较值设置为定时器周期的一半，即约 50% 占空比。这个值是中性占空比，不代表已经给电机施加转矩。
3. 将上下轴和左右轴的 EN 都写为高电平，确保电调保持关闭。
4. 如果任一路 PWM 启动失败，停止已经启动的通道，恢复中性比较值，并保持两个 EN 关闭，然后返回 `HAL_ERROR`。

只有后续控制任务显式调用 `SimpleFOC_Motor_Enable()` 才会把对应 EN 拉低。当前 `MotorTask` 只保留任务入口并周期休眠，不调用使能函数、不修改 PWM，也不调用任何 FOC 数学接口。

## DSP 数学接口

数学函数位于 `Core/interface/SimpleFOC_Motor.c`，使用工程中已经加入的 CMSIS-DSP 库。能使用 DSP 的计算优先使用 DSP，简单的标量组合和占空比限幅保留为直接代码。

### 1. Clarke 变换

输入是三相量 `Ia、Ib、Ic`，输出是固定不旋转的二维坐标 `alpha、beta`：

```text
alpha = Ia - (Ia + Ib + Ic) / 3
beta  = (Ib - Ic) / sqrt(3)
```

三相绕组相差 120 度。Clarke 变换把三相信息压缩到二维平面，减少后续计算量。`alpha` 和 `beta` 仍然表示同一个三相系统，只是换了一种更适合程序计算的坐标表达。

### 2. Park 变换

输入 `alpha、beta` 和电角度，输出旋转坐标 `d、q`：

```text
d = alpha × cos(angle) + beta × sin(angle)
q = -alpha × sin(angle) + beta × cos(angle)
```

可以把它理解为“让坐标轴跟着转子一起旋转”。`d` 轴通常对准转子磁链方向，`q` 轴与磁链垂直，q 轴分量通常主要影响电磁转矩。代码使用 `arm_sin_cos_f32()` 计算正弦和余弦，接口输入角度单位是弧度，但 CMSIS-DSP 函数输入单位是角度，所以代码会先换算。

### 3. 逆 Park 变换

逆 Park 将控制器得到的 `d、q` 电压参考恢复为固定坐标 `alpha、beta`：

```text
alpha = d × cos(angle) - q × sin(angle)
beta  = d × sin(angle) + q × cos(angle)
```

它是 Park 变换的反方向。后续如果有电流环或电压控制器，就可以在 d/q 坐标中计算，再通过逆 Park 得到 SVPWM 所需的二维电压参考。

### 4. SVPWM

SVPWM 将 `alpha、beta` 电压参考转换为三相 PWM 占空比：

```text
Ua = alpha
Ub = -0.5 × alpha + 0.8660254 × beta
Uc = -0.5 × alpha - 0.8660254 × beta
common = -0.5 × (max(Ua, Ub, Uc) + min(Ua, Ub, Uc))
duty = 0.5 + (U + common) / supply_voltage
```

其中 `common` 是同时加到三相上的共同偏置。共同偏置不会改变相间电压，但能帮助三相参考更充分地利用直流母线电压。代码使用 `arm_max_f32()` 和 `arm_min_f32()` 得到三相最大值和最小值，最后把每个占空比限制在 `0.0` 到 `1.0`。

输入母线电压小于等于 0，或输入输出指针为空时，SVPWM 会把能够写入的占空比设为 `0.5` 并直接返回，避免产生非法除法或异常输出。

## 当前代码边界

本阶段已完成：

- 删除旧 `SimpleFOC_Motor` 中的 AS5600 读取、位置环、零位校准、自动使能和 FreeRTOS 控制循环。
- 使用 CMSIS-DSP 的正弦/余弦、最大值和最小值函数实现 Park、逆 Park 和 SVPWM 所需的底层计算。
- 提供 Clarke、Park、逆 Park、SVPWM、PWM 初始化、按轴使能和按轴禁用接口。
- 在主启动流程中初始化双轴 PWM，并在初始化失败时进入 `Error_Handler()`。

本阶段尚未完成：

- 没有相电流采样，因此当前还不是完整的电流环 FOC。
- 没有读取 AS5600，也没有机械角到电角度的运行时计算。
- 没有实现零位校准、位置环、速度环或电流环。
- 没有在任务中调用 Clarke、Park、逆 Park 或 SVPWM。
- 没有在任务中调用 `SimpleFOC_Motor_Enable()`。
- 没有完成电调启动值、方向、机械限位和整机实测验证。

后续实现完整控制时，推荐的数据流为：

```text
相电流采样
    -> Clarke
    -> Park
    -> d/q 电流控制器
    -> 逆 Park
    -> SVPWM
    -> 三相 PWM 比较值
```

AS5600 位置反馈和光敏电阻追光逻辑属于后续的位置/外环功能，不能替代相电流采样，也不能单独说明已经完成完整 FOC。

## 验证状态

- DOCX ZIP 结构检查：通过，`testzip` 返回 `OK`。
- DOCX 表格：17 个表格首行已设置为重复表头。
- DOCX 可访问性检查：通过，当前审计结果为 `high=0, medium=0, low=0`。
- 页面 PNG 视觉检查：尚未完成，因为当前环境未找到 LibreOffice，Microsoft Word COM 自动化也未能完成渲染。
- 本次 SimpleFOC 数学接口和固件构建验证以源代码测试、静态搜索和 ARM 工程构建结果为准。

## 后续工作边界

后续继续开发前，应先确认电调实际接口协议、EN 极性、三相 PWM 的停止值和启动值，再接入相电流采样、ADC 触发、定时器同步控制、AS5600 位置反馈、位置环和自动追光外环。
