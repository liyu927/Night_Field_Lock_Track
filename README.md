> **特别标注：RTOS 任务应用层入口统一放在 `Core/App/app_system.c/.h`。** 任务只负责初始化调用、周期调度和选择运行模式；ADC、滤波、LVGL、电机控制等底层调用由对应的 `App_*` 函数封装。`app_system.c/.h` 使用 `//============` 按任务分隔函数，并在声明处说明用法、参数及注意事项。以后新增 RTOS 任务的底层调用也按此约定封装。

# 太阳能追光系统统一设计说明

本文是 `word` 文件夹内产品方案、技术设计方案、既有说明和框架图的统一归档。保留本文件及其引用的两张 PNG 后，可以删除本文件列出的两份 `.docx`，而不丢失其中的产品需求、技术实现、表格、公式、状态规则和后续工作边界。

## 1. 归档范围与使用原则

已归档的来源：

- `Product_Plan/太阳能追光系统_最终修改版.docx`
- `Technical_Design/太阳能追光系统技术设计方案.docx`
- `Product_Plan/product_plan.md`
- 本文件的历史版本
- `Product_Plan/太阳能追光系统_框架图.png`

产品方案 Word 中的嵌入式图也已单独保留为 [内嵌框架图](Product_Plan/太阳能追光系统_最终修改版_内嵌框架图.png)，因此删除 `.docx` 不会丢失图示。

资料形成于不同阶段，存在历史配置与当前工程不一致的情况。后续实施按下列优先级决策：

1. 实机测量和硬件原理图/数据手册；
2. 当前 `.ioc`、生成代码和已经通过板级验证的行为；
3. 本文的产品规则与技术设计；
4. 早期方案中的具体数值和待办备注。

例如，早期技术方案中记录过 PWM 周期 `4800 - 1`、上电 EN 高电平使能、应用层尚未展开；当前工程的 `tim.c` 使用 `11200 - 1`，`SimpleFOC_Motor.c` 中 EN 高电平为使能，当前已具备触摸、四路采样和 TIM8 开环驱动。不要把旧参数直接覆盖到已验证工程中。

## 2. 项目目标与总体框架

系统以 STM32F407VETx 为主控，使用 FreeRTOS/CMSIS-RTOS V2 调度任务，LVGL 提供触摸 UI。四个光敏电阻按左上、左下、右上、右下布置，经过 ADC1 和 DMA 采样；二维云台的上下、左右两轴由两组三相 PWM 驱动；W25Q64 用于保存可调参数。

第一版交付目标：手动模式、自动追光模式、异常模式、软件低功耗状态、实时数据展示、参数编辑与双备份保存。完整电流环 FOC、LQR、OTA 与手机 App 是后续增强项，不阻塞第一版闭环。

![太阳能追光系统框架图](Product_Plan/太阳能追光系统_框架图.png)

框架图表达的用户流程为：

- 上电进入手动模式，电机默认停止；可进入手动控制、参数调节、产品说明和手机 App 观测占位页。
- 自动模式包含明亮追光/保持、暗淡倒计时和低功耗；光照恢复或触摸可以唤醒。
- 异常状态弹出一次提示，停止所有电机，复位前不能解除。

## 3. 已知当前工程基线

用户已确认触摸、四路光敏模块和无刷电机均可在板上工作。当前代码的主要实现为：

| 模块 | 当前状态 | 说明 |
| --- | --- | --- |
| 触摸与 LVGL | 已可用 | FT6336 采用 I2C 轮询；`PE15` 配置为 CTP_INT 下降沿 EXTI，当前仅保留中断扩展点，不在中断中调用 I2C、RTOS 或 LVGL。 |
| 光照采样 | 已可用 | `SmaplingTask` 启动 ADC1 DMA，按 PA0、PA1、PA2、PA3 读取左上、左下、右上、右下；先三点中值滤波，再做系数 `1/8` 的一阶低通，每 20 ms 发布一帧。 |
| UI 数据显示 | 已可用 | UI 每 200 ms 清空采样队列并显示最新四路数据。 |
| 电机 | 已可驱动 | `MotorTask` 当前只启动 TIM8 左右轴的电压型无位置传感器开环控制，使用虚拟电角度、逆 Park 和 SVPWM，默认 15 rpm 左右。 |
| TIM1 上下轴 | 资源已初始化 | 三相 PWM 已准备，但当前任务没有启动该轴。 |
| AS5600 | 未接入当前控制代码 | 方案规定 I2C1 对应上下轴、I2C3 对应左右轴；代码尚未读取角度。 |
| 相电流采样/电流环 FOC | 未实现 | 光敏 ADC 不能替代相电流采样。 |
| 产品状态机、参数保存、正式 UI | 界面预览已完成，业务未接入 | `user_home` 已有页面与本地交互；仍需要把触摸事件、光照和电机统一到事件/状态机中。 |

当前开环电机并不是完整闭环 FOC：没有 AS5600 反馈、相电流采样、电流 PI、速度环或位置环。现有 Clarke/Park/逆 Park/SVPWM 函数是后续电机后端的计算基础。

## 4. 硬件与资源映射

| 模块 | 配置与用途 |
| --- | --- |
| MCU/时钟 | STM32F407VETx，HSE 8 MHz，SYSCLK 168 MHz。 |
| 光敏 ADC | ADC1、PA0/PA1/PA2/PA3、12 位四通道扫描、DMA2 Stream0 循环；顺序固定为左上、左下、右上、右下。 |
| 上下轴 | TIM1 CH1/CH2/CH3：PE9/PE11/PE13；EN：PE7；规划使用 I2C1 的 AS5600。 |
| 左右轴 | TIM8 CH1/CH2/CH3：PC6/PC7/PC8；EN：PE8；规划使用 I2C3 的 AS5600。 |
| AS5600 | 默认地址均为 `0x36` 时，可因 I2C1/I2C3 分属不同总线而不冲突。两轴极对数记录为 7，供电记录为 12 V。 |
| 显示 | SPI1：PA5 SCK、PA7 MOSI、PA4 DC、PA6 RST、PB0 CS、PB1 BLK；需核对 RGB565 的 DMA 位宽与对齐。 |
| 触摸 | I2C2：PB10 SCL、PB11 SDA，CTP_RST 为 PB12，CTP_INT 为 PE15。 |
| 外部 Flash | SPI2：PB13 SCK、PC2 MISO、PC3 MOSI、PC1 CS 是 W25Q64 的候选映射，必须由 PCB 网名确认。 |

当前 TIM1/TIM8 使用三路普通 PWM 输入，不配置 MCU 的 `CHxN` 互补输出。资料和配套例程支持“每电机三路独立 PWM”的结论；驱动板内部如何产生互补驱动和死区仍需原理图、驱动芯片数据手册或示波器确认。当前 `DeadTime=0`，不能把它当作 MCU 已启用硬件死区。

当前代码中 EN 高电平为使能、低电平为关闭；早期说明中曾写过相反极性，因此实机仍必须确认电调 EN 的安全逻辑。上电、复位、初始化失败、异常、保存和低功耗时，必须关闭 EN 或输出经实测确认的安全 PWM。

## 5. 光敏结构、采样与追光计算

### 5.1 机械前提

四个光敏电阻应安装为左上、左下、右上、右下；模板留出受光孔位，模块之间安装隔板以减少斜向光串扰。可采用打孔、热熔胶和固定条等方式保证不松动。所有阈值、滤波参数和验收测试必须在机械位置、隔板和固定方式完成后标定。

建议以手机手电筒距传感器约 1 m 的多方向照射作为首次标定记录，记录四路滤波 ADC、平均光照和总差异；该数据只用于定义初始范围，不能替代真实环境验证。

### 5.2 派生量与滤波

第一版使用中值滤波去除尖峰，再使用一阶低通：

```text
filtered = filtered + ((raw - filtered) >> 3)
```

即 `alpha = 1/8`。响应慢时可试 `1/4`，抖动明显时可试 `1/16`；滤波系数必须集中为宏定义。早期不建议直接使用卡尔曼滤波，因为它需要可靠的噪声模型与实测调参。

| 符号 | 定义 | 用途 |
| --- | --- | --- |
| `lt/lb/rt/rb` | 左上/左下/右上/右下的滤波 ADC 值 | 原始方向信息 |
| `avg` | `(lt + lb + rt + rb) / 4` | 低光和唤醒判断 |
| `total_diff` | `max4 - min4` | 是否需要追光 |
| `x_diff` | `(lt + lb) - (rt + rb)` | 水平轴方向 |
| `y_diff` | `(lt + rt) - (lb + rb)` | 上下轴方向 |

自动速度可先采用归一化比例控制：

```text
sum = lt + lb + rt + rb + 1
err_x = ((lt + lb) - (rt + rb)) / sum
err_y = ((lt + rt) - (lb + rb)) / sum
speed_x = auto_base_speed * err_x
speed_y = auto_base_speed * err_y
```

执行前应进行轴向映射、最小启动速度、最大速度和斜坡限幅。若实机方向相反，只改 `MOTOR_X_REVERSE` / `MOTOR_Y_REVERSE` 一类校准宏，不交换传感器含义，避免 UI 与故障诊断混乱。

### 5.3 三个光照阈值

| 参数 | 用途 | 关系 |
| --- | --- | --- |
| `light_low_threshold` | 自动模式停止追光并开始低光倒计时 | `wake_threshold` 必须大于它 |
| `light_wakeup_threshold` | 低功耗下允许光照唤醒 | 不参与方向差判断 |
| `light_diff_min` | 是否存在足够方向差 | 不参与低功耗唤醒 |

当 `total_diff <= light_diff_min` 时保持当前位置；只有平均光照高于最低阈值且总差异大于最小差异值时允许自动追光。五个可调参数还包括手动基础速度和自动基础速度；每项均应有最小值、最大值和默认值，先做单值裁剪，再校验 `wake_threshold > low_threshold`。

## 6. 模式、状态与安全规则

系统模式只有三种：手动 `APP_MODE_MANUAL`、自动 `APP_MODE_AUTO`、异常 `APP_MODE_ERROR`。低功耗是独立电源状态 `POWER_LOW`，不是第四个模式。自动子状态为进入倒计时、明亮保持、明亮追光和低光倒计时。

`SystemStateTask` 是系统状态的唯一拥有者：维护手动/自动/异常模式枚举、异常标志位、保存锁和电机停止标志位 `motor_stop`；在手动按键松开、切换模式、保存或唤醒等需要停车的场景清除手动命令；管理自动模式进入前 3 s 倒计时、低光持续 15 s 倒计时，以及软件低功耗的进入与触摸/光照唤醒。异常模式一旦进入便锁存至复位，关闭弹窗、切换页面或唤醒均不能解除。UI 回调、Flash 任务和电机任务均不得直接修改模式或清除停止标志。

停止优先级从高到低如下：

| 优先级 | 条件 | 处理 | 电机结果 |
| --- | --- | --- | --- |
| 1 | 异常已锁存 | 进入异常模式，清除命令和倒计时，只请求一次弹窗 | 立即停止，禁止重启 |
| 2 | 正在保存参数 | 保存锁有效 | 停止 |
| 3 | 低功耗 | 黑屏，等待光照或触摸 | 停止并关闭使能 |
| 4 | 自动模式进入 3 s 倒计时 | 继续采样、显示倒计时 | 停止 |
| 5 | 自动低光 15 s 倒计时 | 光照恢复则取消，否则转低功耗 | 停止 |
| 6 | 手动模式无有效方向 | 保持页面/草稿 | 停止 |
| 7 | 自动满足条件或手动方向有效 | 发布运行快照 | 按命令输出 |

状态机每周期统一重算 `motor_stop`。只有非异常、非低功耗、非保存、倒计时结束，且自动追光条件成立或收到新的有效手动方向时，才允许其为 0。切换模式、保存结束、手动松开或从低功耗唤醒后，都不能恢复旧的手动运动命令。

### 6.1 手动模式

- 上电默认进入手动主页，电机停止。
- 上、下、左、右按钮只发送方向事件；按住期间按手动基础速度运行，松开后下一次控制周期内停止。
- 返回手动控制页时清除方向命令并停止。
- 异常、保存或停止标志有效时，方向命令无效。

### 6.2 自动模式

切换自动模式后先清除手动命令、置停止原因、启动 3 s 倒计时；倒计时内只采样和显示。

| 条件 | 自动子状态/结果 |
| --- | --- |
| 倒计时未结束 | 显示倒计时，禁止电机输出 |
| `avg <= low_threshold` | 停止追光，开始 15 s 低光倒计时 |
| `avg > low_threshold` 且 `total_diff <= diff_min` | 明亮保持，电机停止 |
| `avg > low_threshold` 且 `total_diff > diff_min` | 明亮追光，按 `x_diff/y_diff` 输出两轴命令 |

低光倒计时期间，平均光照恢复到高于最低阈值或切换手动模式时，取消倒计时并重新判断。自动页面显示平均光照、总差异、状态和倒计时。

### 6.3 异常模式

非低功耗时，若某一路滤波 ADC 连续超过 2 s 为 0 或满量程，且其余至少一路处于正常范围，则判定单路传感器异常；四路全 0 或全满不按此规则判定。每路维护 `abnormal_ms`，条件不成立立即清零，达到 2000 ms 后上报事件。

异常一旦成立即锁存：停止所有电机、取消倒计时、禁用自动/手动输出，首次进入时只请求一次“设备异常”窗口。窗口可关闭，用户仍可查看说明、调参和保存，但只能通过复位解除异常，不能通过页面返回、低功耗或保存解除。

### 6.4 软件低功耗

第一版采用软件低功耗：关闭背光、隐藏实时数据、停止电机、降低 UI 刷新频率，同时保持低频 ADC 采样，因此仍可由光照唤醒。直接进入 MCU STOP 模式会停止 ADC，除非额外设计定时唤醒/外部比较器，否则不满足光照唤醒需求。

满足 `avg >= wake_threshold` 或检测到触摸时退出低功耗，恢复进入前页面、模式和参数草稿后重新执行状态优先级。自动模式重新判断光照和差异；手动模式保持停止等待新的按键；低功耗期间不弹异常窗口且异常计时不累加。异常状态不允许借助低功耗绕过。

## 7. 电机控制路线

当前双轴设计使用 TIM1 + I2C1 + AS5600 控制上下轴，TIM8 + I2C3 + AS5600 控制左右轴。AS5600 测量机械角度，不测量相电流，不能独立完成完整电流环 FOC。

电机后端应与上层解耦：上层只输出两轴速度/停止命令，`MotorDriver` 决定实际三相 PWM、驱动器协议或后续 FOC。推荐第一版顺序：先完成安全停止、手动方向、光照比例速度和 AS5600 零位/方向校准；相电流硬件确认后再增加电流环 FOC、速度 PID 或 LQR。

完整闭环 FOC 所需链路为：

```text
安全初始化 -> PWM 中性/安全值 -> EN 关闭
-> AS5600 读取 -> 电角度零位与方向校准
-> 相电流 ADC/DMA 和零点校准 -> Clarke -> Park
-> d/q 电流 PI -> 逆 Park -> SVPWM -> PWM 比较值更新
-> 过流/过欠压/传感器/机械限位保护
```

关键变换保留如下：

```text
Clarke:
alpha = Ia - (Ia + Ib + Ic) / 3
beta  = (Ib - Ic) / sqrt(3)

Park:
d = alpha * cos(angle) + beta * sin(angle)
q = -alpha * sin(angle) + beta * cos(angle)

逆 Park:
alpha = d * cos(angle) - q * sin(angle)
beta  = d * sin(angle) + q * cos(angle)

SVPWM:
Ua = alpha
Ub = -0.5 * alpha + 0.8660254 * beta
Uc = -0.5 * alpha - 0.8660254 * beta
common = -0.5 * (max(Ua, Ub, Uc) + min(Ua, Ub, Uc))
duty = 0.5 + (U + common) / supply_voltage
```

理想三相平衡时可由 `Ic = -(Ia + Ib)` 推导第三相，但仍必须确认采样拓扑、触发时刻、放大倍数、零点偏置、ADC 量程和过流保护。没有这些条件时，不能实现可靠电流环或基于电流方向的软件死区补偿。

AS5600 上电对齐的目标是建立机械角、电角度、相序与方向的映射：施加固定磁场方向，等待转子稳定，读取角度，计算并保存电角零点，同时识别方向。机械安装稳定时可保存到 Flash；装配可能变化时可每次上电重新校准。还必须校准磁铁同心度、极对数、机械限位、停止安全值和最大安全速度。

## 8. FreeRTOS 架构与通信

建议新增业务模块集中在 `Core/App`，CubeMX 文件只保留初始化和 USER CODE 钩子：

```text
app_config.h        参数默认值、范围、可调宏
app_types.h         模式、事件、快照、命令类型
light_sensor.c/.h   DMA 数据、滤波、派生量、异常候选
motor_driver.c/.h   双轴 PWM、使能、停止和后端封装
motor_control.c/.h  手动/自动速度命令生成
system_state.c/.h   状态机、倒计时、停止原因、异常锁存
param_store.c/.h    默认参数、校验、双记录读写
w25q64.c/.h         SPI Flash 基础操作
ui_app.c/.h         LVGL 初始化和主页面
ui_pages.c/.h       手动、自动、参数、说明、App 页面
debug_task.c/.h     串口调试输出
```

| 优先级 | 任务 | 周期/触发 | 职责 |
| --- | --- | --- | --- |
| 中断级（后续） | PwmFastLoop | PWM 更新中断 | 仅完整 FOC 后启用；做同步电流采样和 PWM 更新，不调用 RTOS API。 |
| 高 | MotorControlTask | 20 ms | 消费快照并输出两轴命令；任何停止条件立即调用 `StopAll`。 |
| 中高 | LightSampleTask | DMA 完成或 10 ms 轮询、20 ms 发布 | 滤波、平均、差异和异常候选。 |
| 中 | SystemStateTask | 事件驱动 + 20 ms/1 s 节拍 | 唯一状态拥有者和停止仲裁者。 |
| 中低 | UiTask | 5-10 ms | 唯一调用 LVGL，刷新数据与页面；只投递事件。 |
| 低 | FlashTask | 保存事件 | 双记录擦写、回读、提交 `commit` 和返回结果。 |
| 低 | DebugTask | 100 ms/按需 | 串口输出状态、参数和故障；可由宏关闭。 |

DMA 原始数组仅由采样任务复制，光照和状态均以“最新快照”供 UI/状态机/电机使用。UI、Flash 和触摸中断向状态机投递事件；保存请求携带完整参数副本，Flash 返回结果事件。LVGL 非线程安全，任何其他任务和中断不得调用 LVGL。

## 9. UI、页面与触摸规则

UI 使用 `lv_tileview` 或同等页面容器组织手动页和自动页；滑动/按钮只发模式事件，真正模式切换由状态机完成。页面层保存返回栈、当前页、参数草稿和脏标志。

| 页面 | 控件/内容 | 规则 |
| --- | --- | --- |
| 手动主页 | 手动控制、参数、产品说明、App 观测、低功耗入口 | 默认页面。 |
| 手动控制 | 上/下/左/右长按、返回 | 按下发方向，释放发停止，返回清空命令。 |
| 参数调节 | 五项参数、保存、返回 | 维护草稿；普通返回且有修改时询问保存/放弃/取消。 |
| 产品说明 | 文本、返回 | 只展示，不改模式。 |
| App 观测 | 空白占位、返回 | 保留入口，不实现通信。 |
| 自动模式 | ADC、平均值、总差异、状态、倒计时 | 进入自动时启动 3 s 倒计时。 |
| 异常窗口 | 一次性设备异常提示 | 关闭不清异常。 |
| 低功耗 | 黑屏 | 关闭背光、隐藏实时数据。 |

模式切换或低功耗离开参数页不弹保存确认、不自动保存，草稿和页面状态应保留；复位或保存成功后清除草稿。非低功耗页面底部持续显示四路光照、平均值和总差异。

### 9.1 完整界面交互要求与当前阶段边界

- 使用 `lv_tileview` 布置手动、自动两个主页面，手动页在上、自动页在下，上电默认手动页，通过上下滑动切换。系统模式只有手动、自动、异常三种；低功耗是独立电源状态。进入参数、说明或 App 子页不改变模式。页面只呈现和投递事件，模式、倒计时、异常锁存及停车统一由 `SystemStateTask` 决定；异常优先级最高，页面切换、保存和唤醒均不能解除异常或启动电机。
- 手动主页依次提供手动控制、参数调节、产品说明和手机 App 观测入口。手动控制页有上、下、左、右按住/松开按钮：有效按住期间按手动基础速度移动，松开、返回、切换模式或停车条件成立时清空方向并停车。所有返回按钮回到进入前的页面，支持连续返回；普通子页返回不改变模式。
- 参数页编辑光照最小差异值、光照唤醒阈值、光照最低阈值、手动基础速度和自动基础速度，分别维护当前生效值、进入页面时的快照、草稿及脏标志。每项按明确的最小/最大范围裁剪，且保存前校验“唤醒阈值 > 最低阈值”。实际范围必须根据实测由用户提供，不能把界面演示值当作正式范围。普通返回有未保存修改时，提供保存、放弃、取消：保存提交草稿；放弃恢复进入页面前的值并清除草稿；取消留在当前页。模式切换或进入低功耗离开参数页时不提示、不保存，保留页面、返回栈及草稿；复位或真正保存成功后清除草稿。
- 真正保存时先停车，再由 `FlashTask` 写入 W25Q64 双备份；成功后新值生效并提示，失败或阈值关系错误时旧值继续生效、电机保持停止并提示失败，**未写入 Flash 的草稿继续保留供修改或重试**。保存结束或切回手动模式不恢复旧的方向命令。
- 自动页显示四路 ADC、平均光照、总差异、自动状态和倒计时。每次切入自动模式先清空手动命令并停车，再开始 3 s 倒计时；期间继续采样和显示但禁止电机输出。倒计时后，平均光照 `<=` 最低阈值则停车并开始 15 s 低光倒计时；平均光照 `>` 最低阈值且总差异 `<=` 最小差异值则显示明亮保持；两者均高于对应阈值才显示明亮追光。总差异固定为四路 ADC 的最大值减最小值，唤醒阈值只用于低功耗唤醒。低光期间光照恢复或切回手动时取消倒计时。对应文案为“光照不足，%d 秒后进入低功耗模式”“明亮模式，当前位置保持”“明亮模式，自动追光中”。
- 软件低功耗时停车、关闭背光、黑屏并隐藏实时数据；平均光照 `>=` 唤醒阈值或触摸可唤醒。唤醒后恢复原页面、模式、返回栈和草稿，并重新执行异常和自动条件判断；手动模式等待新的方向按键，不恢复旧命令。异常模式不能借低功耗退出，低功耗期间不弹异常窗口。
- 非低功耗的全部页面底部持续显示左上、左下、右上、右下四路滤波值、平均光照和总差异。产品说明页只显示文字；手机 App 观测页暂为空白占位。异常首次锁存时仅请求一次“设备异常”弹窗，关闭弹窗仍可查看说明、调参和保存，但电机入口始终无效，只有复位能解除异常。
- `motor_stop` 仅由 `SystemStateTask` 按异常、保存、低功耗、自动进入倒计时、低光倒计时、有效手动命令及自动追光条件统一重算。LVGL 回调不得直接操作电机、Flash、模式枚举或清除停止标志。

**当前界面预览阶段：**`user_home` 仅实现瓦片/子页导航、触摸按钮效果、五项演示数值增减、保存成功提示、异常弹窗预览和光照数据显示；演示保存不写 Flash、不改变生效参数。自动状态与倒计时保留占位显示。低功耗及所有电机/状态机联动尚未接入。

## 10. W25Q64 参数双备份

参数采用两个独立 4 KB 扇区的 A/B 记录。记录至少含：`magic`、`version`、`length`、`sequence`、`payload`、`crc32`、`commit`。CRC 覆盖头部与 payload，不覆盖最后写入的 `commit`。

启动时先加载并裁剪编译期默认参数，再分别验证 A/B 的 magic、版本、长度、CRC 和 commit：两条都有效取较新的 sequence；仅一条有效取该条；均无效则保留默认值并报告恢复默认。

保存时：

1. `SystemStateTask` 停止电机、清空手动命令、校验范围及阈值关系，并复制完整草稿；
2. `FlashTask` 选择非最新扇区，擦除后写入头部、payload 和 CRC，但不写 commit；
3. 回读并重新计算 CRC；
4. 通过后最后单独写入 commit，旧记录不立即擦除；
5. 成功才提交新生效参数并清除草稿脏标志；失败保留旧参数和停止状态，UI 明确提示。

保存中掉电时，新记录因 commit 缺失或 CRC 错误被忽略，旧记录仍可使用。序号递增需要处理回绕。Flash 擦写耗时，必须在低优先级 `FlashTask` 中执行。

## 11. 测试、验收与硬件确认清单

| 类别 | 测试 | 通过条件 |
| --- | --- | --- |
| 状态机 | 自动 3 s、低光 15 s、模式切换 | 顺序正确，所有非允许状态均停止电机。 |
| ADC | 通道顺序、滤波、阈值边界 | 单独遮挡/照射时数据和方向符合物理位置。 |
| 异常 | 单路 0/满量程、四路全 0/全满 | 仅符合规则时异常锁存。 |
| 手动 | 长按、松开、返回、保存中按键 | 运动连续，松开/返回在一个控制周期内停止。 |
| 自动 | 左强、右强、上强、下强、平衡、低光 | 方向正确，平衡不抖，低光进入低功耗流程。 |
| 保存 | 成功、CRC 损坏、中途掉电、越界 | 成功后生效，失败保留旧值，掉电不破坏旧记录。 |
| UI | 页面栈、草稿、异常窗口、低功耗恢复 | 交互规则符合第 9 节。 |
| 安全 | 上电、复位、任务阻塞、擦写期 | 电机默认停止，禁止状态无输出。 |

写代码或升级电机前，必须确认：电调型号和 PWM 语义、最小/启动/停止/中性占空比、EN 极性、内部互补与死区；AS5600 同心度/零位/方向/极限位置；W25Q64 接线与时钟；显示/触摸型号和 PE15 EXTI 要求；五项参数的实测范围。若启用完整 FOC，还需确认相电流采样接口、增益、零偏、同步触发和保护阈值。

## 12. 后续增强项

在基础手动/自动闭环稳定后，再评估速度 PID、位置环优化、完整电流环 FOC、LQR、外部电池管理、导电滑环、手机 App/远程控制、ESP8266 等 OTA 固件升级。导电滑环、电源拓扑和机械固定属于硬件范围，软件只保留接口边界；OTA 可预留 Flash 分区，但不应混入第一版追光控制实现。

具体实施顺序、每一步的验收条件和当前最适合先做的“触摸屏发事件 -> 自动/手动状态机 -> 电机执行”链路见 [Plan.md](Plan.md)。

## 13. 当前工程代码实现状态

本节以当前工程源码为准，补充说明已经实际写入和正在运行的功能。它不替换前文的产品需求与技术设计，而是用来区分“已有代码”和“下一步要实现的代码”。

### 13.1 当前已完成任务

| 功能 | 当前实现 | 关键文件 |
| --- | --- | --- |
| 系统启动与外设初始化 | `main()` 已依次初始化 GPIO、DMA、SPI1、USART1、I2C2、TIM1、TIM8、ADC1、SPI2、I2C1、I2C3，并在启动 RTOS 前调用电机 PWM 安全初始化。 | `Core/Src/main.c` |
| FreeRTOS 基础任务 | 已创建 `DebugTask`、`UiTask`、`MotorTask`、`SmaplingTask`，并创建长度为 16 的四路光照消息队列。 | `Core/Src/freertos.c` |
| 触摸屏驱动 | 已完成 FT6336 复位、I2C2 轮询触摸点数、首个触摸坐标读取和坐标转换；LVGL 输入设备初始化后可读取触摸。 | `Core/interface/FT6336.c`、`Core/App/UiTask.c` |
| 触摸中断入口 | `PE15` 已配置为上拉、下降沿 EXTI；`EXTI15_10_IRQHandler()` 能转入 `HAL_GPIO_EXTI_Callback()`。当前中断中不执行 I2C、RTOS 或 LVGL 操作。 | `Core/Src/gpio.c`、`Core/Src/stm32f4xx_it.c`、`Core/interface/FT6336.c` |
| 四路 ADC DMA 采样 | ADC1 以连续扫描模式采样 PA0--PA3，12 位、DMA 循环模式，缓冲区持续由硬件更新。 | `Core/Src/adc.c` |
| 光照滤波与任务通信 | `SmaplingTask` 每 20 ms 对四路数据依次执行三点中值滤波和 `1/8` 一阶低通，再写入 `Smapling` 消息队列。 | `Core/App/SmaplingTask.c`、`Core/common/filter.c` |
| 光照 UI 显示 | 当前 UI 的 LVGL 定时器每 200 ms 取空采样队列，用最新一帧更新底部四路、平均值、总差异以及自动页数据。 | `Core/ui/user_home.c` |
| 三相 PWM 初始化 | TIM1、TIM8 均已配置三路普通 PWM；`SimpleFOC_Motor_Init()` 启动六路 PWM、设置约 50% 中性占空比并关闭两个 EN。 | `Core/Src/tim.c`、`Core/interface/SimpleFOC_Motor.c` |
| TIM8 无刷电机开环驱动 | `MotorTask` 使用虚拟电角度、逆 Park 和 SVPWM 驱动水平轴；默认目标 `0.25 rps`，约 15 rpm，电压指令为 2 V，1500 ms 斜坡。 | `Core/App/MotorTask.c`、`Core/interface/SimpleFOC_Motor.c` |
| FOC 数学基础 | 已实现 Clarke、Park、逆 Park 和 SVPWM；SVPWM 输出会限幅到 0--1，再换算为定时器比较值。 | `Core/interface/SimpleFOC_Motor.c` |
| 基础 UI | `UiTask` 已完成 LVGL、显示端口和输入端口初始化；`user_home` 已替换演示页面，具备手动/自动瓦片、子页、参数演示控件、底部光照数据与异常弹窗预览。 | `Core/App/UiTask.c`、`Core/ui/user_home.c` |

### 13.2 当前实际数据流

光照数据已经形成如下可运行链路：

```text
PA0/PA1/PA2/PA3
    -> ADC1 四通道连续扫描
    -> DMA2 Stream0 循环写入 adc1_dma_buf[4]
    -> SmaplingTask（中值滤波 + 低通滤波，每 20 ms）
    -> Smapling 消息队列（长度 16）
    -> user_home 的 LVGL 定时器（每 200 ms 取最新帧）
    -> PA0--PA3 标签显示
```

当前电机链路为独立的开环演示链路：

```text
MotorTask（每 1 ms）
    -> 软件推进虚拟电角度
    -> d/q 电压指令（d=0，q=设定电压）
    -> 逆 Park
    -> SVPWM
    -> TIM8 CH1/CH2/CH3 比较寄存器
    -> PE8 拉高使能水平轴电调
```

因此，当前光照采样结果还没有传入 `MotorTask`，触摸事件也还没有产生电机命令；二者目前在功能上是独立的。

### 13.3 关键代码：启动顺序与安全 PWM 初始化

`main()` 在启动调度器前初始化相关外设，并准备两轴 PWM，但不直接让电机运行：

```c
MX_GPIO_Init();
MX_DMA_Init();
MX_SPI1_Init();
MX_USART1_UART_Init();
MX_I2C2_Init();
MX_TIM1_Init();
MX_TIM8_Init();
MX_ADC1_Init();
MX_SPI2_Init();
MX_I2C1_Init();
MX_I2C3_Init();

if (SimpleFOC_Motor_Init() != HAL_OK)
{
    Error_Handler();
}
```

`SimpleFOC_Motor_Init()` 对 TIM1、TIM8 的三个通道均执行 `HAL_TIM_PWM_Start()`；每个轴均先设置三相约 50% 的中性占空比，再将 EN 写为低电平。当前工程的 EN 逻辑为高电平使能、低电平关闭。

```c
sfoc_set_neutral_pwm(axis);
sfoc_disable_axis(axis);
```

### 13.4 关键代码：FreeRTOS 任务与采样队列

当前系统创建的采样队列和任务如下：

```c
SmaplingHandle = osMessageQueueNew(16,
                                   sizeof(SmaplingData_t),
                                   &Smapling_attributes);

DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);
UiTaskHandle = osThreadNew(StartUiTask, NULL, &UiTask_attributes);
MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);
SmaplingTaskHandle = osThreadNew(StartSmaplingTask, NULL,
                                  &SmaplingTask_attributes);
```

当前任务优先级中，`MotorTask` 和 `SmaplingTask` 为 `osPriorityNormal`，`UiTask` 和 `DebugTask` 为 `osPriorityLow`。`DebugTask` 目前只每 100 ms 延时，没有输出调试信息；它已经预留，可用于后续打印模式、光照差、两轴命令和故障原因。

### 13.5 关键代码：ADC、滤波与消息发布

ADC1 已配置为四通道扫描、连续转换和循环 DMA；DMA 缓冲定义为：

```c
uint16_t adc1_dma_buf[4];

void ADC1_DMA_Start(void)
{
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_buf, 4) != HAL_OK)
    {
        Error_Handler();
    }
}
```

`SmaplingTask` 在任务启动时只启动一次 DMA，然后每 20 ms 处理一帧。四个数组下标与物理位置固定对应：`0` 左上、`1` 左下、`2` 右上、`3` 右下。

```c
smapling_data.left_top = Filter_LowPass(&left_top_filter,
    Filter_Median3Update(&left_top_median, adc1_dma_buf[0]),
    LIGHT_LOW_PASS_SHIFT);

smapling_data.left_bottom = Filter_LowPass(&left_bottom_filter,
    Filter_Median3Update(&left_bottom_median, adc1_dma_buf[1]),
    LIGHT_LOW_PASS_SHIFT);

smapling_data.right_top = Filter_LowPass(&right_top_filter,
    Filter_Median3Update(&right_top_median, adc1_dma_buf[2]),
    LIGHT_LOW_PASS_SHIFT);

smapling_data.right_bottom = Filter_LowPass(&right_bottom_filter,
    Filter_Median3Update(&right_bottom_median, adc1_dma_buf[3]),
    LIGHT_LOW_PASS_SHIFT);

(void)osMessageQueuePut(SmaplingHandle, &smapling_data, 0U, 0U);
```

低通函数使用整数移位实现 `y(k) = y(k-1) + (x(k)-y(k-1))/2^shift`；当前 `shift=3`，即系数 `1/8`。三点中值滤波和标量卡尔曼滤波函数都已实现，但采样任务当前只使用中值加低通，未启用卡尔曼。

### 13.6 关键代码：触摸轮询、坐标转换和中断入口

FT6336 使用 I2C2、7 位地址 `0x38`。轮询到有效触摸点后读取首个触摸点，并转换为 240 x 320 屏幕坐标：

```c
raw_x = (uint16_t)(((buf[0] & 0x0FU) << 8) | buf[1]);
raw_y = (uint16_t)(((buf[2] & 0x0FU) << 8) | buf[3]);

*x = raw_x >= FT6336_LCD_WIDTH ? 0U : FT6336_LCD_WIDTH - 1U - raw_x;
*y = raw_y >= FT6336_LCD_HEIGHT ? FT6336_LCD_HEIGHT - 1U : raw_y;
```

触摸中断已经接入，但只作为后续低功耗唤醒的扩展点：

```c
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(CTP_INT_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == CTP_INT_Pin)
    {
        /* 后续低功耗唤醒逻辑放在这里。 */
    }
}
```

### 13.7 关键代码：当前开环无刷电机控制

当前 `MotorTask` 的运行方式是上电后自动启动**水平轴**。它还不是由触摸或光照控制的正式逻辑：

```c
#define TIM8_OPEN_LOOP_SPEED_RPS        0.25f
#define TIM8_OPEN_LOOP_VOLTAGE          2.0f
#define TIM8_OPEN_LOOP_POLE_PAIRS       7U
#define TIM8_OPEN_LOOP_DIRECTION        1
#define TIM8_OPEN_LOOP_RAMP_TIME_MS     1500U
#define MOTOR_TASK_PERIOD_MS            1U

if (SimpleFOC_Motor_OpenLoopStart(SIMPLEFOC_AXIS_HORIZONTAL,
                                  &tim8_open_loop_config) != HAL_OK)
{
    SimpleFOC_Motor_OpenLoopStop(SIMPLEFOC_AXIS_HORIZONTAL);
}
```

随后任务每 1 ms 调用更新函数，并用绝对唤醒时间尽量保持周期稳定：

```c
next_wake_tick = osKernelGetTickCount();
for (;;)
{
    SimpleFOC_Motor_OpenLoopUpdate(SIMPLEFOC_AXIS_HORIZONTAL);
    next_wake_tick += MOTOR_TASK_PERIOD_MS;
    (void)osDelayUntil(next_wake_tick);
}
```

开环更新中，电角速度按“机械转速 × 极对数 × `2π`”计算；软件推进电角度后，以 d 轴为 0、q 轴为设定电压进行逆 Park 和 SVPWM。运行中没有读取 AS5600、没有相电流采样、没有闭环速度/位置修正；负载过大、相序不对或电压不足时仍可能抖动或失步。

### 13.8 关键代码：当前 LVGL 任务和页面

`UiTask` 是唯一调用 LVGL 的任务，已完成显示、输入与页面初始化：

```c
lv_init();
lv_tick_set_cb(HAL_GetTick);
lv_delay_set_cb(vTaskDelay);
lv_port_disp_init();
lv_port_indev_init();
user_home_create();

for (;;)
{
    uint32_t delay_ms = lv_timer_handler();
    osDelay(delay_ms);
}
```

当前 `user_home_create()` 已创建手动/自动瓦片、手动控制页、参数页、产品说明页、App 占位页和异常弹窗预览。方向按键只更新界面文字；参数增减只修改页面演示值，“保存成功”提示不代表写入 Flash；自动状态与倒计时仍是占位文字。UI 还未投递业务事件，也未接入电机、状态机或低功耗控制。

### 13.9 当前未完成任务

下表仅列出尚未在当前源码中实现或尚未接入当前控制链路的事项：

| 模块 | 未完成内容 |
| --- | --- |
| 触摸到业务事件 | 已有手动/自动页面滑动与方向键触摸效果，但还没有投递手动/自动、方向按下/松开或触摸唤醒的业务事件。 |
| 系统状态机 | 已创建 `SystemStateTask` 任务入口，但当前仅循环延时；尚未实现模式枚举、异常标志位与锁存、手动命令清除、停止原因及统一的 `motor_stop`、3 s 自动倒计时、15 s 低光倒计时和低功耗唤醒状态管理。 |
| 自动追光 | 尚未计算 `avg`、`total_diff`、`x_diff`、`y_diff`；采样队列没有被电机控制任务使用。 |
| 手动控制 | 未按触摸命令控制两轴；当前水平轴会在 `MotorTask` 启动后直接开环旋转。 |
| 垂直轴控制 | TIM1 三相 PWM 已初始化，但当前没有任务调用 TIM1 的开环启动/更新接口。 |
| AS5600 | I2C1/I2C3 已初始化，但没有 AS5600 驱动、读取、零位校准、方向校准、通信故障处理或机械限位。 |
| 完整 FOC | 没有相电流采样、同步触发、d/q 电流 PI、过流保护、速度环或位置环。 |
| 异常与安全 | 未实现四路 ADC 的 0/满量程持续 2 s 异常判断、异常锁存、一次性弹窗、机械限位与电调通信故障停车。 |
| 参数保存 | 页面已有仅供预览的草稿增减和成功提示；尚未实现正式参数范围校验、W25Q64 驱动、A/B 双记录、CRC32、commit 和掉电恢复。 |
| 低功耗 | 未实现背光关闭、UI 降频、低频 ADC 唤醒、触摸唤醒后的页面与模式恢复。 |
| 调试和测试 | `DebugTask` 未输出运行数据；尚未形成自动追光、异常、保存和低功耗的板级验收程序。 |

### 13.10 现阶段最重要的改动边界

下一步应先实现“UI 事件 -> 状态机 -> 电机命令”而不是在 `user_home.c` 的按钮回调中直接操作 PWM 或 EN。这样可以保证自动模式倒计时、低光、参数保存、触摸松开、故障和低功耗都经过同一个停车规则。

在这个链路完成前，当前 `MotorTask` 的“上电自动运行 TIM8”仅适合作为脱载调试演示；正式产品逻辑应改为默认停止，只有状态机明确允许时才启动对应轴。
