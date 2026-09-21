# 太阳能追光系统文档说明

## 文档入口

- 产品方案：[Product_Plan/product_plan.md](Product_Plan/product_plan.md)
- 技术设计方案：[Technical_Design/太阳能追光系统技术设计方案.docx](Technical_Design/太阳能追光系统技术设计方案.docx)

## 当前完成情况

技术设计方案已经完成，并已根据产品方案、`Solar_Light_Tracking_System.ioc`、CubeMX 生成代码和当前 LVGL 工程基线进行逐项复核。技术方案描述的是代码实现设计，不代表应用层代码已经全部编写完成。

已覆盖的产品功能和技术实现包括：

- STM32F407VETx、168 MHz 时钟、FreeRTOS CMSIS-RTOS V2 和 LVGL 工程基线。
- 四路光敏电阻的左上、左下、右上、右下布局，ADC1 四通道 DMA 采样、一阶低通滤波、平均值、总差异、左右差和上下差计算。
- 光敏电阻模板开孔、四路隔板降低斜向光串扰的机械前提，以及隔板安装完成后再进行参数标定和验收的约束。
- 手动模式、自动模式、异常模式和独立低功耗状态；自动模式 3 秒进入倒计时、低光 15 秒倒计时、低光进入低功耗及光照/触摸唤醒。
- 自动模式阈值关系：光照最低阈值用于停止判断，光照唤醒阈值用于低功耗唤醒，光照最小差异值用于方向追光判断，且唤醒阈值必须高于最低阈值。
- 手动模式长按上、下、左、右方向控制，按键释放后的及时停止，以及切换模式和保存后必须重新按键才能恢复手动运动。
- 单路 ADC 边界异常持续超过 2 秒的检测、全 0/全满量程组合例外、异常标志锁存、一次性异常窗口、复位恢复和异常状态下的电机安全禁止。
- 异常状态下允许关闭窗口、查看产品说明、进入参数页面和执行保存，但不能清除异常、切换模式启动电机或进入低功耗。
- TIM1/TIM8 各三路 PWM 和 EN 引脚的当前 IOC 映射，以及双轴电机驱动抽象层。
- 双轴电机闭环映射已经确认：TIM1 + I2C1 + AS5600 用于上下轴，TIM8 + I2C3 + AS5600 用于左右轴。三路 PWM 负责向对应电调发送三相控制量，AS5600 通过独立 I2C 总线提供转子机械角度反馈。
- 两个无刷电机分别配套电调，每个轴由三路 PWM 输入控制。当前按样例工程和已确认参数使用：预分频为 0、PWM1、高电平有效、当前 STM32F407 的周期为 `11200 - 1`，频率约 15 kHz，EN 为高电平有效，电机供电为 12 V，极对数为 7。样例工程的 `4800 - 1` 直接对应其 72 MHz 定时器时钟；本工程 TIM1/TIM8 定时器时钟为 168 MHz，因此换算为 `11200 - 1`。
- 低噪声云台选型和验收约束，包含平稳启动、斜坡限幅、机械共振和换相噪声的联调要求。
- 以光照差分为外环的自动追光比例控制、方向映射、速度限幅和斜坡限制；电机控制使用 AS5600 位置反馈，双轴属于位置反馈闭环控制，不是开环控制。若后续实现完整电流环 FOC，还需要补充相电流采样。
- W25Q64 Record A/Record B 双备份、`magic/version/length/sequence/payload/crc32/commit` 数据包、回读校验、最后写 `commit`、掉电恢复和默认参数回退。
- LVGL `lv_tileview` 手动页/自动页上下切换、页面栈、嵌套返回、普通页面底部实时数据显示、低功耗黑屏和手机 App 空白占位页。
- 当前生效参数与页面草稿分离、范围裁剪、阈值关系校验、保存/放弃/取消交互和保存期间电机停止。
- 产品方案中的标定流程：隔板和光敏电阻固定完成后，用手机电筒在约 1 m 处采集四路 ADC，计算平均光照和总差异，再确定五项参数的初始范围、默认值和现场验收数据。
- 保存成功/失败结果事件、参数草稿快照、普通返回时的保存/放弃/取消选择，以及模式切换或低功耗离开时保留草稿但不自动保存。
- FreeRTOS 任务拆分、快照通信、状态机优先级、启动流程、主机逻辑测试、板级外设测试和整机联调验收项目。
- ADC 模拟看门狗尚未配置，`CTP_INT=PE15` 的 EXTI/NVIC 和触摸驱动仍需核对。AS5600 闭环仍需实机校准磁铁同心度、零位和方向；极对数已按样例及确认值固定为 7。

## 双轴闭环电机映射

| 轴 | PWM 定时器 | AS5600 通信接口 | 闭环反馈 |
| --- | --- | --- | --- |
| 上下轴 | TIM1 三路 PWM | I2C1 | 通过 AS5600 读取转子机械角度，参与电角度计算和闭环控制 |
| 左右轴 | TIM8 三路 PWM | I2C3 | 通过 AS5600 读取转子机械角度，参与电角度计算和闭环控制 |

两台 AS5600 均可使用默认 I2C 地址 `0x36`，因为分别连接在 I2C1 和 I2C3 上，不会发生总线地址冲突。每个电机都有独立的三路 PWM 输出和独立的角度反馈通道，因此当前方案是基于转子位置反馈的双轴闭环控制，不是无位置反馈的开环控制。三路 PWM 与 AS5600 的关系分别是：PWM 输出驱动对应电调，AS5600 经 I2C 返回角度，控制器根据反馈角度更新下一次 PWM 控制量。

## 电机部分当前实现

### 1. 已完成的硬件和软件映射

当前工程按照已经确认的 `.ioc` 配置实现了两路独立云台电机：

| 功能 | 上下轴 | 左右轴 |
| --- | --- | --- |
| PWM 定时器 | TIM1 | TIM8 |
| 三相 PWM 引脚 | PE9、PE11、PE13 | PC6、PC7、PC8 |
| 电调 EN 引脚 | PE7，`TIM1_EN` | PE8，`TIM8_EN` |
| AS5600 总线 | I2C1 | I2C3 |
| I2C 引脚 | PB6=SCL、PB7=SDA | PA8=SCL、PC9=SDA |
| AS5600 地址 | `0x36` | `0x36` |
| 机械轴 | 上下摇摆 | 左右摇摆 |

两个 AS5600 虽然使用相同的默认地址 `0x36`，但它们分别挂在 I2C1 和 I2C3 上，因此不会发生同一总线上的地址冲突。

已经完成的相关文件如下：

- `Core/interface/SimpleFOC_Motor.c`：双轴电机控制、AS5600 读取、零位校准、位置环和三相 PWM 计算。
- `Core/interface/SimpleFOC_Motor.h`：电机控制接口和上下轴/左右轴映射说明。
- `Core/App/MotorTask.c`：FreeRTOS 电机任务入口。
- `Core/Src/i2c.c`、`Core/Inc/i2c.h`：I2C1 和 I2C3 初始化及 GPIO 复用。
- `Core/Src/tim.c`：TIM1、TIM8 三路 PWM 初始化。
- `Core/Src/main.c`：在启动时初始化两个电机控制轴。
- `Core/Src/freertos.c`：创建并运行 `MotorTask`。
- `Solar_Light_Tracking_System.ioc`：同步保存 TIM1、TIM8、I2C1、I2C3 和 EN 引脚配置。

### 2. PWM 配置

当前 TIM1 和 TIM8 均配置为：

- 计数器模式：向上计数。
- 预分频：`0`。
- 自动重装值：`11200 - 1`。
- PWM 模式：`PWM1`。
- 输出极性：高电平有效。
- 输出通道：CH1、CH2、CH3。
- 定时器时钟：168 MHz。
- PWM 频率：`168 MHz / 11200 = 15 kHz`。

样例工程使用 STM32F1，定时器时钟为 72 MHz，所以样例中的 `4800 - 1` 对应约 15 kHz。当前芯片是 STM32F407，TIM1/TIM8 定时器时钟为 168 MHz，因此已将 ARR 适配为 `11200 - 1`，保持相同的约 15 kHz PWM 频率。

TIM1 的三路输出为：

```text
PE9  -> TIM1_CH1
PE11 -> TIM1_CH2
PE13 -> TIM1_CH3
```

TIM8 的三路输出为：

```text
PC6 -> TIM8_CH1
PC7 -> TIM8_CH2
PC8 -> TIM8_CH3
```

电机控制启动时，程序调用 `HAL_TIM_PWM_Start()` 启动对应定时器的三个 PWM 通道，并先把三个通道设置到约 50% 的中性占空比。EN 引脚采用高电平有效逻辑；如果 AS5600 读取失败，程序会关闭对应 EN，并恢复三个通道的中性占空比。

### 3. AS5600 角度反馈原理

AS5600 通过 I2C 返回 12 位磁编码器角度。程序每次读取两个字节：

```c
#define SFOC_AS5600_ADDRESS        (0x36U << 1)
#define SFOC_AS5600_ANGLE_REGISTER 0x0CU
#define SFOC_AS5600_RESOLUTION     4096.0f

uint8_t data[2] = {0};

HAL_I2C_Mem_Read(axis->i2c,
                 SFOC_AS5600_ADDRESS,
                 SFOC_AS5600_ANGLE_REGISTER,
                 I2C_MEMADD_SIZE_8BIT,
                 data,
                 sizeof(data),
                 SFOC_I2C_TIMEOUT_MS);

raw_value = (uint16_t)((((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU);
new_angle = ((float)raw_value / SFOC_AS5600_RESOLUTION) * SFOC_TWO_PI;
```

具体过程如下：

1. 通过 I2C1 读取上下轴 AS5600，或通过 I2C3 读取左右轴 AS5600。
2. 从 AS5600 的角度寄存器 `0x0C` 读取两个字节。
3. 保留低 12 位，得到 `0~4095` 的原始角度值。
4. 将原始值换算成 `0~2PI` 弧度。
5. 对 `4095 -> 0` 和 `0 -> 4095` 的跳变进行跨零点处理，生成连续机械角度。
6. 连续机械角度用于位置误差计算，当前角度用于电角度计算。

当前实现是轮询读取，不使用 AS5600 中断。电机任务每 2 ms 读取一次两个传感器，因此理论控制更新周期为 2 ms。AS5600 的 I2C 读取失败时会立即禁止对应电机输出，等待下一次读取成功后重新进入初始化和校准流程。

### 4. 电角度和极对数

样例工程中的初始化调用为：

```c
FOC_Closeloop_Init(&FOC0, &htim2, 12, 10, 1, 7);
FOC_Closeloop_Init(&FOC1, &htim2, 12, 10, 1, 7);
```

其中最后一个参数确认是电机极对数 `7`。当前工程已经对应写入：

```c
#define SFOC_SUPPLY_VOLTAGE   12.0f
#define SFOC_VOLTAGE_LIMIT    10.0f
#define SFOC_MOTOR_DIRECTION  1
#define SFOC_MOTOR_POLE_PAIRS 7
```

机械角度和电角度的关系为：

```text
电角度 = 机械角度 × 电机方向 × 极对数 - 电角度零位
```

因此当前程序使用：

```c
electrical_angle = normalize_angle(
    SFOC_MOTOR_DIRECTION * SFOC_MOTOR_POLE_PAIRS * axis->angle_raw
    - axis->zero_electric_angle);
```

### 5. 上电零位校准

每个轴都有三个状态：

```c
typedef enum
{
    SFOC_AXIS_WAITING = 0,
    SFOC_AXIS_ALIGNING,
    SFOC_AXIS_RUNNING
} SFOC_AxisState;
```

状态流程如下：

1. `WAITING`：等待 AS5600 读取成功。
2. 读取成功后拉高对应 EN，使能电调。
3. 使用较小的校准电压 `2.0 V`，把转子吸到电角度 0。
4. 保持 `500 ms`，让转子稳定。
5. 再次读取 AS5600，计算并保存 `zero_electric_angle`。
6. 把当前机械角设为目标角，避免上电后云台突然跳向一个未指定的目标位置。
7. 进入 `RUNNING`，开始位置闭环控制。

关键校准代码如下：

```c
sfoc_set_enabled(axis, true);
sfoc_set_torque(axis, SFOC_ALIGNMENT_VOLTAGE, 0.0f);
axis->alignment_start_tick = HAL_GetTick();
axis->state = SFOC_AXIS_ALIGNING;
```

校准完成后：

```c
axis->zero_electric_angle = sfoc_normalize_angle(
    SFOC_MOTOR_DIRECTION * SFOC_MOTOR_POLE_PAIRS * axis->angle_raw);

axis->target_angle = axis->angle_continuous;
axis->state = SFOC_AXIS_RUNNING;
```

校准期间电机可能出现小幅吸合或转动，这是建立磁场电角度零位的过程。实际方向、磁铁同心度和校准电压仍需上电后确认。

### 6. 位置闭环和三路 PWM 计算原理

当前控制器不是只输出固定 PWM 的开环控制，而是每个控制周期都使用 AS5600 反馈值计算位置误差：

```c
position_error = axis->target_angle - axis->angle_continuous;

uq = sfoc_limit(SFOC_POSITION_KP * position_error,
                -SFOC_POSITION_VOLTAGE_LIMIT,
                SFOC_POSITION_VOLTAGE_LIMIT);
```

当前参数为：

```c
#define SFOC_POSITION_KP            3.0f
#define SFOC_POSITION_VOLTAGE_LIMIT 3.0f
#define SFOC_ALIGNMENT_VOLTAGE      2.0f
#define SFOC_ALIGNMENT_HOLD_MS      500U
#define SFOC_UPDATE_PERIOD_MS       2U
```

得到 `Uq` 和电角度后，程序使用仅含 `Uq` 的逆 Park/三相电压计算，将三相电压转换成三个 PWM 占空比：

```c
u_alpha = -uq * sinf(electrical_angle);
u_beta  =  uq * cosf(electrical_angle);

u_a = u_alpha + SFOC_VOLTAGE_LIMIT / 2.0f;
u_b = (-u_alpha + SFOC_SQRT3 * u_beta) / 2.0f
      + SFOC_VOLTAGE_LIMIT / 2.0f;
u_c = (-u_alpha - SFOC_SQRT3 * u_beta) / 2.0f
      + SFOC_VOLTAGE_LIMIT / 2.0f;

duty_a = limit(u_a / SFOC_SUPPLY_VOLTAGE, 0.0f, 1.0f);
duty_b = limit(u_b / SFOC_SUPPLY_VOLTAGE, 0.0f, 1.0f);
duty_c = limit(u_c / SFOC_SUPPLY_VOLTAGE, 0.0f, 1.0f);
```

最后将占空比写入对应定时器的三个比较寄存器：

```c
__HAL_TIM_SET_COMPARE(axis->timer, axis->channel_a,
                      duty_a * period);
__HAL_TIM_SET_COMPARE(axis->timer, axis->channel_b,
                      duty_b * period);
__HAL_TIM_SET_COMPARE(axis->timer, axis->channel_c,
                      duty_c * period);
```

完整数据流可以概括为：

```text
AS5600 磁铁转动
    ↓
I2C1/I2C3 读取 12 位角度
    ↓
机械角度、连续角度和位置误差
    ↓
极对数 7 换算电角度
    ↓
位置比例环计算 Uq
    ↓
逆 Park 和三相电压计算
    ↓
TIM1/TIM8 的 CH1、CH2、CH3 更新占空比
    ↓
对应电调接收三路 PWM
    ↓
无刷电机转动
    ↓
AS5600 再次反馈角度
```

因此，当前实现属于“AS5600 位置反馈闭环 + 电压换相控制”。它还不是带相电流采样、Clark/Park 电流环、硬件过流保护和 PWM 同步采样的完整电流环 FOC。

### 7. FreeRTOS 任务和启动顺序

启动顺序为：

```c
MX_TIM1_Init();
MX_TIM8_Init();
MX_I2C1_Init();
MX_I2C3_Init();
SimpleFOC_Motor_Init();
```

之后创建 `MotorTask`：

```c
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

MotorTaskHandle = osThreadNew(StartMotorTask, NULL,
                              &MotorTask_attributes);
```

任务主体每 2 ms 更新两个轴：

```c
void SimpleFOC_Motor_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        SimpleFOC_Motor_Update();
        osDelay(SFOC_UPDATE_PERIOD_MS);
    }
}
```

当前电机控制没有使用 `TIM1_UP`、`TIM8_UP` 或 PWM 更新中断；PWM 只是由定时器硬件持续输出，AS5600 读取和占空比更新由 `MotorTask` 完成。这种方式适合当前的基础位置闭环验证，但后续若实现高速电流环，需要改为定时器同步中断或其他确定性快速控制结构。

## 电机部分已完成和待完成清单

### 已完成

- 确认并实现 TIM1 上下轴、TIM8 左右轴的双轴映射。
- 配置 TIM1、TIM8 各三路 PWM。
- 配置 PWM1、高电平有效、预分频 0。
- 根据 STM32F407 的 168 MHz 定时器时钟，将 ARR 适配为 `11200 - 1`，得到约 15 kHz。
- 配置 I2C1 和 I2C3 连接两个 AS5600。
- 实现 AS5600 地址、角度寄存器和 12 位角度读取。
- 实现角度跨零点连续化。
- 实现极对数 7 的电角度计算。
- 实现上电零位校准。
- 实现位置误差比例控制。
- 实现三相电压到三路 PWM 占空比的计算。
- 实现 EN 高电平使能和 I2C 读取失败时的关闭处理。
- 创建 FreeRTOS `MotorTask`，控制周期为 2 ms。
- 已完成 CMake 编译验证，当前固件 Flash 使用约 63.5%，RAM 使用约 58.3%。

### 待完成

- 用示波器实测 TIM1 的 PE9/PE11/PE13 和 TIM8 的 PC6/PC7/PC8 波形，确认频率、幅值、相位和占空比范围。
- 确认电调实际接口协议确实是三路相控制 PWM；如果电调实际只接受单路油门 PWM，则当前三路换相方式不适用。
- 确认电调的启动最小占空比、停止值、刹车方式和 EN 时序。
- 实测两个 AS5600 是否能稳定返回角度，检查 3.3 V 供电、I2C 上拉、电气连接和磁铁同心度。
- 分别校准两个轴的机械零位、正方向、电机方向和 UI 方向映射。
- 根据实际负载调整 `SFOC_POSITION_KP`、`SFOC_POSITION_VOLTAGE_LIMIT` 和 `SFOC_ALIGNMENT_VOLTAGE`。
- 增加机械限位、启动保护、堵转判断、角度越界和更明确的故障锁存。
- 将光敏电阻自动追光结果和手动页面目标角度接入 `SimpleFOC_Motor_SetTargetAngles()`。
- 如果后续需要完整 FOC，增加相电流采样、ADC 触发、Clark/Park 电流环、SVPWM、过流保护和定时器同步控制。
- 增加串口调试输出或调试变量，用于观察 AS5600 原始角度、连续角度、目标角度、位置误差和电机状态。

## 验证状态

- DOCX ZIP 结构检查：通过，`testzip` 返回 `OK`。
- DOCX 表格：17 个表格首行已设置为重复表头。
- DOCX 可访问性检查：通过，当前审计结果为 `high=0, medium=0, low=0`。
- 技术方案更新脚本已支持重复运行检查，不会重复插入补充章节或日期信息。
- 页面 PNG 视觉检查：尚未完成。官方 `render_docx.py` 需要 LibreOffice `soffice.exe`，当前环境未找到该程序；已安装的 Microsoft Word 存在，但 Word COM 自动化因登录会话错误 `80070520` 未能完成渲染。因此不能把当前文档描述为已经通过页面级视觉验收。

## 仍需实机验证

以下项目仍需要结合实际硬件或后续功能确认：

- 电调的具体型号、启动最小占空比、停止安全值和刹车方式仍需通过实机验证；当前 PWM 基本时序、EN 极性、供电电压和极对数按已确认参数执行。
- 低噪声验收指标、允许的机械共振和负载工况，需结合实际云台、电机和电调测试确认。
- TIM1、TIM8 的电机正方向和 UI 方向映射，以及 AS5600 磁铁安装同心度和零位，需要实机校准。
- W25Q64 是否确实连接 SPI2，片选是否为 `PC1`，以及允许的 SPI 时钟。
- 屏幕控制器、触摸控制器型号、LVGL 版本、`CTP_INT` 中断极性和是否必须使用 `PE15` EXTI。
- 五项参数的实际最小值和最大值：光照最小差异值、光照最低阈值、光照唤醒阈值、手动模式电机基础速度、自动模式电机基础速度。

## 后续工作边界

当前工程已经具备双轴 AS5600 位置反馈闭环驱动的基础实现：TIM1 + I2C1 对应上下轴，TIM8 + I2C3 对应左右轴。当前代码是基于转子角度反馈的位置环和电压换相控制，不等同于带相电流采样的完整电流环 FOC。后续重点是完成实机零位、方向、停止值和机械限位校准，再继续完善 ADC、状态机、参数存储、LVGL 页面、测试、手机 App 和 OTA。
