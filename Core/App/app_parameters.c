#include "app_parameters.h"
#include "W25Q64.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* 参数范围集中在此处维护，顺序通过下方两张范围表映射到枚举下标。 */
/* 背光占空比至少 30%，避免用户把滑块调到近乎看不见。 */
#define APP_BRIGHTNESS_MIN 30U
#define APP_BRIGHTNESS_MAX 100U
/* 手动和自动熄屏时间共用秒数范围；下限 10 秒大于熄屏前的 5 秒渐暗期。 */
#define APP_SLEEP_MIN 10U
#define APP_SLEEP_MAX 100U
/* 左右、上下光差分别以出厂值 50 和 100 为范围中点，单位均为 ADC 差值。 */
#define APP_HORIZONTAL_BAND_MIN 0U
#define APP_HORIZONTAL_BAND_MAX 100U
#define APP_VERTICAL_BAND_MIN 0U
#define APP_VERTICAL_BAND_MAX 200U
/* 四路光敏电阻的 ADC 为 12 位，平均值阈值允许覆盖完整的 0～4095。 */
#define APP_LOW_POWER_LIGHT_MIN 0U
#define APP_LOW_POWER_LIGHT_MAX 4095U
/* 自动 STOP 中的 RTC 周期唤醒间隔，单位为秒。 */
#define APP_WAKE_INTERVAL_MIN 3U
#define APP_WAKE_INTERVAL_MAX 100U
/* 手动运动速度以度每秒计，自动 Kp 以百分之一为保存和调节单位。 */
#define APP_MANUAL_SPEED_MIN 1U
#define APP_MANUAL_SPEED_MAX 180U
#define APP_AUTO_KP_MIN 1U
#define APP_AUTO_KP_MAX 100U

/* 固件内置的出厂快照：只用于上电同步和首次无用户记录时的初值。 */
static const AppParameters factory_parameters = {{
    APP_BRIGHTNESS_DEFAULT, APP_MANUAL_SLEEP_DEFAULT, APP_AUTO_SLEEP_DEFAULT,
    APP_HORIZONTAL_BAND_DEFAULT, APP_VERTICAL_BAND_DEFAULT,
    APP_LOW_POWER_LIGHT_DEFAULT, APP_WAKE_INTERVAL_DEFAULT,
    APP_MANUAL_SPEED_DEFAULT, APP_AUTO_KP_HUNDREDTHS_DEFAULT
}};
/* 当前真正供系统任务使用的参数；保存成功并读回后才整体替换。 */
static AppParameters active_parameters = {{
    APP_BRIGHTNESS_DEFAULT, APP_MANUAL_SLEEP_DEFAULT, APP_AUTO_SLEEP_DEFAULT,
    APP_HORIZONTAL_BAND_DEFAULT, APP_VERTICAL_BAND_DEFAULT,
    APP_LOW_POWER_LIGHT_DEFAULT, APP_WAKE_INTERVAL_DEFAULT,
    APP_MANUAL_SPEED_DEFAULT, APP_AUTO_KP_HUNDREDTHS_DEFAULT
}};
/* JEDEC 检查通过且启动加载流程结束后，才允许界面发起保存或恢复操作。 */
static uint8_t flash_ready;
/* 两张表与 AppParameterIndex 一一对应，避免界面和存储校验采用不同范围。 */
static const uint16_t parameter_minimum[APP_PARAMETER_COUNT] = {
    APP_BRIGHTNESS_MIN, APP_SLEEP_MIN, APP_SLEEP_MIN,
    APP_HORIZONTAL_BAND_MIN, APP_VERTICAL_BAND_MIN, APP_LOW_POWER_LIGHT_MIN,
    APP_WAKE_INTERVAL_MIN, APP_MANUAL_SPEED_MIN, APP_AUTO_KP_MIN
};
static const uint16_t parameter_maximum[APP_PARAMETER_COUNT] = {
    APP_BRIGHTNESS_MAX, APP_SLEEP_MAX, APP_SLEEP_MAX,
    APP_HORIZONTAL_BAND_MAX, APP_VERTICAL_BAND_MAX, APP_LOW_POWER_LIGHT_MAX,
    APP_WAKE_INTERVAL_MAX, APP_MANUAL_SPEED_MAX, APP_AUTO_KP_MAX
};

/* 检查整份参数是否可用；空指针或任意字段越界均拒绝写入和应用。 */
static uint8_t parameters_valid(const AppParameters *parameters)
{
    if (parameters == NULL) return 0U;
    for (uint32_t i = 0U; i < APP_PARAMETER_COUNT; ++i)
        if (parameters->values[i] < parameter_minimum[i]
            || parameters->values[i] > parameter_maximum[i])
            return 0U;
    return 1U;
}

/* 上电先确认 W25Q64，再核对出厂扇区，最后选取用户 A/B 中序号最新的有效记录。 */
/* 两个用户槽都无效时保留 active_parameters 的代码默认值，首次烧录无需预写用户槽。 */
void App_Parameters_Init(void)
{
    AppParameters loaded;
    HAL_StatusTypeDef status = HAL_BUSY;
    for (uint32_t attempt = 0U; attempt < 20U && status == HAL_BUSY; ++attempt)
    {
        status = W25Q64_Init();
        if (status == HAL_BUSY) osDelay(10U);
    }
    if (status != HAL_OK) return;
    /* 每次上电核对代码默认值，内容变化时才擦写出厂扇区。 */
    for (uint32_t attempt = 0U; attempt < 100U; ++attempt)
    {
        status = W25Q64_SyncFactoryParameters(&factory_parameters, sizeof(factory_parameters));
        if (status != HAL_BUSY) break;
        osDelay(10U);
    }
    status = W25Q64_LoadUserParameters(&loaded, sizeof(loaded), 0U);
    if (status == HAL_OK && parameters_valid(&loaded) != 0U)
    {
        taskENTER_CRITICAL();
        active_parameters = loaded;
        taskEXIT_CRITICAL();
    }
    flash_ready = 1U;
}

/* 临界区内一次性复制全部字段，避免界面保存时系统任务读到新旧混合值。 */
AppParameters App_Parameters_Get(void)
{
    AppParameters result;
    taskENTER_CRITICAL();
    result = active_parameters;
    taskEXIT_CRITICAL();
    return result;
}

/* 非法下标返回 0；正常调用应使用 AppParameterIndex 中的下标。 */
uint16_t App_Parameters_Min(AppParameterIndex index)
{
    return index < APP_PARAMETER_COUNT ? parameter_minimum[index] : 0U;
}

/* 非法下标返回 0；界面的上限与保存时的校验上限来自同一张表。 */
uint16_t App_Parameters_Max(AppParameterIndex index)
{
    return index < APP_PARAMETER_COUNT ? parameter_maximum[index] : 0U;
}

/* 先校验并写入较旧/无效槽，再从最新槽读回，确认无误后立即公布到运行时。 */
/* HAL_OK 表示写入和应用均成功；其他状态表示底层忙或记录校验失败。 */
HAL_StatusTypeDef App_Parameters_Save(const AppParameters *parameters)
{
    AppParameters checked;
    HAL_StatusTypeDef status;
    if (flash_ready == 0U || parameters_valid(parameters) == 0U) return HAL_ERROR;
    status = W25Q64_SaveUserParameters(parameters, sizeof(*parameters));
    if (status != HAL_OK) return status;
    status = W25Q64_LoadUserParameters(&checked, sizeof(checked), 0U);
    if (status != HAL_OK || parameters_valid(&checked) == 0U) return HAL_ERROR;
    taskENTER_CRITICAL();
    active_parameters = checked;
    taskEXIT_CRITICAL();
    return HAL_OK;
}

/* 恢复出厂值属于一次新的用户保存，原用户版本因此仍可作为旧版回归。 */
HAL_StatusTypeDef App_Parameters_RestoreFactory(void)
{
    AppParameters selected;
    HAL_StatusTypeDef status;
    if (flash_ready == 0U) return HAL_ERROR;
    status = W25Q64_LoadFactoryParameters(&selected, sizeof(selected));
    if (status != HAL_OK || parameters_valid(&selected) == 0U) return HAL_ERROR;
    return App_Parameters_Save(&selected);
}

/* 将较旧槽的内容重新保存为最新版本，形成可再次切换的双版本记录。 */
/* 旧版不存在或 Flash 正忙时可能返回 HAL_BUSY，界面需按操作未完成处理。 */
HAL_StatusTypeDef App_Parameters_RestorePrevious(void)
{
    AppParameters selected;
    HAL_StatusTypeDef status;
    if (flash_ready == 0U) return HAL_ERROR;
    status = W25Q64_LoadUserParameters(&selected, sizeof(selected), 1U);
    if (status != HAL_OK || parameters_valid(&selected) == 0U) return status == HAL_BUSY ? HAL_BUSY : HAL_ERROR;
    return App_Parameters_Save(&selected);
}
