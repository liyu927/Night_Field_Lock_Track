#ifndef NIGHT_FIELD_LOCK_TRACK_APP_PARAMETERS_H
#define NIGHT_FIELD_LOCK_TRACK_APP_PARAMETERS_H

#include "main.h"

/* 参数总数决定存储记录的长度；增加参数时需同时更新下方枚举、默认值和范围表。 */
#define APP_PARAMETER_COUNT 9U
/* 以下数值是编译进固件的出厂值，上电时会同步到 W25Q64 的出厂扇区。 */
#define APP_BRIGHTNESS_DEFAULT 100U          /* 背光 PWM 占空比，单位为百分比。 */
#define APP_MANUAL_SLEEP_DEFAULT 20U         /* 手动模式无触摸后熄屏并进入低功耗的秒数。 */
#define APP_AUTO_SLEEP_DEFAULT 15U           /* 自动模式熄屏及满足低功耗条件的等待秒数。 */
#define APP_HORIZONTAL_BAND_DEFAULT 50U      /* 左右光差的停转范围，单位为 ADC 差值。 */
#define APP_VERTICAL_BAND_DEFAULT 100U       /* 上下光差的停转范围，单位为 ADC 差值。 */
#define APP_LOW_POWER_LIGHT_DEFAULT 1800U    /* 四路 ADC 平均值高于此值时视为偏暗。 */
#define APP_WAKE_INTERVAL_DEFAULT 5U         /* 自动低功耗期间定时唤醒采光的秒数。 */
#define APP_MANUAL_SPEED_DEFAULT 60U         /* 手动控制目标速度，单位为度每秒。 */
#define APP_AUTO_KP_HUNDREDTHS_DEFAULT 12U   /* 自动追光比例系数的百分之一单位，10 表示 0.10。 */

/* 数组下标与界面顺序、默认值顺序、范围表顺序一致，不能单独调整其中一处。 */
typedef enum
{
    APP_PARAM_BRIGHTNESS,       /* 背光亮度，30～100%。 */
    APP_PARAM_MANUAL_SLEEP,     /* 手动模式熄屏时间，单位为秒。 */
    APP_PARAM_AUTO_SLEEP,       /* 自动模式熄屏与入睡等待时间，单位为秒。 */
    APP_PARAM_HORIZONTAL_BAND,  /* 左右光差小于等于此值时不调整水平轴。 */
    APP_PARAM_VERTICAL_BAND,    /* 上下光差小于等于此值时不调整垂直轴。 */
    APP_PARAM_LOW_POWER_LIGHT,  /* 用于自动低功耗判断的四路 ADC 平均值阈值。 */
    APP_PARAM_WAKE_INTERVAL,    /* 自动低功耗定时唤醒间隔，单位为秒。 */
    APP_PARAM_MANUAL_SPEED,     /* 手动电机目标速度，单位为度每秒。 */
    APP_PARAM_AUTO_KP           /* 自动追光比例系数乘以 100 后的整数值。 */
} AppParameterIndex;

/* 持久化参数的数据格式；九个 uint16_t 按枚举顺序写入用户参数槽。 */
typedef struct
{
    uint16_t values[APP_PARAMETER_COUNT];
} AppParameters;

/* 系统任务启动时调用：初始化 Flash、同步出厂值，并加载最新有效用户记录。 */
void App_Parameters_Init(void);
/* 原子复制当前生效参数；系统任务和 UI 通过返回值读取同一份运行时配置。 */
AppParameters App_Parameters_Get(void);
/* 返回指定参数的允许下限或上限；界面据此限制加减按钮与亮度滑块。 */
uint16_t App_Parameters_Min(AppParameterIndex index);
uint16_t App_Parameters_Max(AppParameterIndex index);
/* 校验所有字段范围，保存到用户双槽，读回确认后立即更新运行时参数。 */
HAL_StatusTypeDef App_Parameters_Save(const AppParameters *parameters);
/* 读取出厂扇区并作为新的用户版本保存，成功后无需重启即可生效。 */
HAL_StatusTypeDef App_Parameters_RestoreFactory(void);
/* 读取用户双槽中的较旧版本并重新保存；无旧版本时返回 HAL_BUSY。 */
HAL_StatusTypeDef App_Parameters_RestorePrevious(void);

#endif
