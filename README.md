# Solar Light Tracking System

这是一个基于 STM32F407、FreeRTOS 与 HAL 的太阳光追踪系统。工程当前包含触摸屏、四路光照采集和两组无刷电机 FOC 电压驱动。

## 目录职责

- `Core/common`：可复用的软件算法。目前有滤波器 `filter.c/.h` 和本次新增的 PID 算法 `pid.c/.h`。
- `Core/interface`：底层硬件接口。本次新增 AS5600 的 I2C 读取接口 `AS5600.c/.h`，并将 `SimpleFOC_Motor.c/.h` 改为闭环 FOC 所需的三相 PWM、电角度、逆 Park 与 SVPWM 输出接口。
- `Core/App`：FreeRTOS 任务。本次更新 `MotorTask.c`，在任务中运行自动校准、位置环和速度环。
- `Core/Types`：RTOS 队列、消息和任务之间传递数据所需的枚举、结构体。目前包含光照队列的 `SmaplingTypes.h`。

## 电机硬件映射

| 轴 | PWM | 使能 | 编码器 |
|---|---|---|---|
| TIM1，竖直轴 | `PE9 / PE11 / PE13` | `PE7`，高有效 | I2C1，PB6/PB7，AS5600 |
| TIM8，水平轴 | `PC6 / PC7 / PC8` | `PE8`，高有效 | I2C3，PA8/PC9，AS5600 |

两个电机均为 7 极对，母线电压为 12 V。`MOTOR_DIRECTION_CLOCKWISE` 设为 `1`，表示当前相序定义下 q 轴正电压对应顺时针。

## 本次完成：编码器反馈闭环电机控制

已彻底删除虚拟电角度推进、开环启动、开环斜坡和 `SimpleFOC_OpenLoop*` 接口。现在的控制链路为：

```text
目标机械位置
  → 位置 PID
目标机械速度
  → 速度 PID
q 轴电压（力矩指令）
  → AS5600 实测角度换算电角度 → 逆 Park → SVPWM → TIM1/TIM8 三相 PWM
```

`Core/App/MotorTask.c` 每 5 ms 从 AS5600 读取角度。它会处理 0/360° 回绕，累积连续机械位置，并由相邻采样差分得到机械速度。I2C 读取失败会立即关闭对应轴的 EN 并将三相 PWM 恢复为中性占空比，禁止继续按旧角度驱动。

### 关于“力矩环”

本硬件没有相电流或母线电流采样，因此无法测量电磁转矩，也无法形成物理意义上的闭环力矩/电流环。本工程按已确认的硬件条件实现为：位置环、速度环闭环，速度环输出受 `±2.5 V` 限制的 q 轴电压作为力矩指令。这不是开环换相，转子角度始终来自 AS5600；但 q 轴实际电流和实际转矩不能被校正。若以后增加电流采样，才可在速度环与 PWM 之间加入真正的 `Iq PID` 力矩环。

### 自动电角零位校准

可以自动校准，代码已在 `MotorTask.c` 的 `motor_auto_calibrate()` 中实现：

1. 在已知电角度 `0 rad` 输出 1 V 的 d 轴锁定电压；
2. 等待 800 ms，使转子与磁场对齐；
3. 经 I2C 读取 AS5600 的机械角度；
4. 计算 `电角偏移 = 锁定电角度 - 极对数 × 机械角度`；
5. 关闭 EN、清零 PID 历史，再开始闭环控制。

校准时转子会向锁定位置移动，因此首次上电必须脱开机构或确保运动范围安全。TIM1 每次启动自动校准。TIM8 目前绝不校准也不使能，保持静止；其 I2C、PID 和 PWM 均已初始化。只有在机械安全时，将 `TIM8_AUTO_CALIBRATE_ON_BOOT` 从 `0U` 改成 `1U` 才会为 TIM8 执行同样的校准。

### TIM1 上电演示轨迹

`MotorTask` 将 TIM1 当前实测位置作为初始目标，随后持续生成位置目标：

- 前 5 秒：顺时针 `0.10 rps`，约 6 rpm；
- 5 秒后：逆时针 `-0.20 rps`，约 12 rpm，约为顺时针速度的两倍；
- TIM8：EN 始终为低，电机不会转动。

待上层任务接入时，只需将同样的“目标位置”替换为追光算法给出的目标，无需改动 AS5600、PID、FOC 或 PWM 输出层。

关键控制代码如下：

```c
target_speed = PID_Update(&control->position_pid,
                          target_position,
                          control->mechanical_position,
                          interval_s);
q_voltage = PID_Update(&control->speed_pid,
                       target_speed,
                       control->mechanical_speed,
                       interval_s);
electrical_angle = SimpleFOC_Motor_CalculateElectricalAngle(
    control->raw_angle, control->electrical_offset, 7U, 1);
SimpleFOC_Motor_SetPhaseVoltage(control->axis, 0.0f, q_voltage,
                                electrical_angle, 12.0f);
```

## PID 文件

本次新增：

- `Core/common/pid.h`：`PID_Controller_t`、初始化、清零和更新函数声明；
- `Core/common/pid.c`：带输出限幅和积分抗饱和的 PID 实现。

位置 PID 输出速度目标，速度 PID 输出 q 轴电压。所有需要实机调节的电机参数集中在 `Core/App/MotorTask.c` 顶部的宏中，包括极对数、母线电压、校准电压、PID 输出限制和演示速度。

## 光照采样

- `SmaplingTask` 使用 ADC1 的 PA0~PA3 DMA 循环采集四路光敏电阻；
- 每 20 ms 经一阶低通后，将 `SmaplingData_t` 写入 `Smapling` 消息队列；
- UI 每 200 ms 读取最新数据，避免显示积压；
- `Core/common/filter.c/.h` 提供一阶低通、标量卡尔曼、三点中值滤波。

ADC1 的四个通道全部用于光照数据，因此本项目没有电流采样，也没有电流保护功能。

## 触摸屏

- FT6336 保持 I2C2 轮询方式，LVGL 在 `lv_port_indev.c` 读取触摸坐标与按下状态；
- `PE15` 是 `CTP_INT` 下降沿 EXTI；当前中断仅保留扩展点，不执行 I2C、FreeRTOS 或 LVGL 操作。

## 未完成任务和限制

- 未接入光照差值、追光状态机与电机目标位置；当前 TIM1 使用固定测试轨迹。
- 未做 Flash 保存。电角偏移量会在每次 TIM1 启动时重新校准，掉电后不保留。
- 未提供相电流、过流、堵转、母线欠压或过温保护；上电测试必须卸载机构并准备立即断电。
- 无电流采样时，不能实现真实闭环力矩控制。
- 触摸屏、电机和光照采集的实际工作状态必须通过编译及上板测试确认；源代码静态检查不能替代硬件验证。

## 构建

工程使用 CMake、Ninja 和 `arm-none-eabi-gcc`：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```
