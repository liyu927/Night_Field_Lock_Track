#include "app_system.h"

#include "adc.h"
#include "cmsis_os2.h"
#include "filter.h"
#include "FreeRTOS.h"
#include "lvgl.h"
#include "lv_port_indev.h"
#include "lv_port_lcd_stm32.h"
#include "main.h"
#include "SimpleFOC_Motor.h"
#include "SmaplingTypes.h"
#include "task.h"
#include "user_home.h"
#include <string.h>

#define LIGHT_LOW_PASS_SHIFT 3U /* 一阶低通系数为 1/8。 */

/* 四路滤波历史必须跨采样周期保留，由初始化函数统一清零。 */
static Median3Filter_t g_light_median[4];
static LowPassFilter_t g_light_low_pass[4];
extern osMessageQueueId_t SmaplingHandle;
/* 四路滤波结果同时提供给系统状态任务。 */
extern osMessageQueueId_t SmapToSystemHandle;

//============ 光照采样任务应用层函数 ============

/* 采样任务启动时清空滤波历史并启动循环 ADC DMA。 */
void App_Smapling_Init(void)
{
    memset(g_light_median, 0, sizeof(g_light_median));
    memset(g_light_low_pass, 0, sizeof(g_light_low_pass));
    ADC1_DMA_Start();
}

/* 进入 STOP 前停止连续 ADC DMA，唤醒后再由采样初始化函数启动。 */
void App_Smapling_Stop(void)
{
    /* 不改动滤波结果；STOP 流程会清队列，探测前重新初始化滤波器。 */
    (void)HAL_ADC_Stop_DMA(&hadc1);
}

/* 四路数据按 PA0～PA3 顺序滤波并发布最新快照。 */
void App_Smapling_Run(void)
{
    SmaplingData_t data;

    data.left_top = Filter_LowPass(&g_light_low_pass[0],
        Filter_Median3Update(&g_light_median[0], adc1_dma_buf[0]), LIGHT_LOW_PASS_SHIFT);
    data.left_bottom = Filter_LowPass(&g_light_low_pass[1],
        Filter_Median3Update(&g_light_median[1], adc1_dma_buf[1]), LIGHT_LOW_PASS_SHIFT);
    data.right_top = Filter_LowPass(&g_light_low_pass[2],
        Filter_Median3Update(&g_light_median[2], adc1_dma_buf[2]), LIGHT_LOW_PASS_SHIFT);
    data.right_bottom = Filter_LowPass(&g_light_low_pass[3],
        Filter_Median3Update(&g_light_median[3], adc1_dma_buf[3]), LIGHT_LOW_PASS_SHIFT);

    (void)osMessageQueuePut(SmaplingHandle, &data, 0U, 0U);
    if (SmapToSystemHandle != NULL
        && osMessageQueuePut(SmapToSystemHandle, &data, 0U, 0U) != osOK)
    {
        /* 队列满时丢弃最旧帧，保证系统任务能取得新光照。 */
        SmaplingData_t old_data;
        (void)osMessageQueueGet(SmapToSystemHandle, &old_data, NULL, 0U);
        (void)osMessageQueuePut(SmapToSystemHandle, &data, 0U, 0U);
    }
}

//============ UI 任务应用层函数 ============

/* 建立 LVGL 时钟、显示、输入设备和首页。 */
void App_Ui_Init(void)
{
    lv_init();
    lv_tick_set_cb(HAL_GetTick);
    lv_delay_set_cb(vTaskDelay);
    lv_port_disp_init();
    lv_port_indev_init();
    user_home_create();
}

/* 执行一轮 UI 更新并交还 LVGL 建议的延时。 */
uint32_t App_Ui_Run(void)
{
    /* LVGL 只在 UI 任务中操作；异常通知由系统任务锁存到首次领取。 */
    if (App_System_TakeErrorPopup() != 0U)
        user_home_show_error();
    uint8_t recovery_result = App_System_TakeRecoveryResult();
    if (recovery_result != 0U)
        user_home_show_recovery_result(recovery_result);
    return lv_timer_handler();
}

//============ 电机任务应用层函数 ============

/* 准备并启动两轴速度环；后续只用各自 AS5600 的相邻角度差求速度。 */
void App_MotorSpeed_Run(float speed_rps)
{
    SimpleFOC_Motor_RunTask(SIMPLEFOC_CONTROL_SPEED, speed_rps);
}

/* 准备并启动两轴位置、速度双环，目标角度相对各自命令起点。 */
void App_MotorPosition_Run(float delta_deg)
{
    SimpleFOC_Motor_RunTask(SIMPLEFOC_CONTROL_POSITION, delta_deg);
}

/* 运行期间改写速度目标；返回底层命令是否被接收。 */
uint8_t App_MotorSpeed_SetTargetSpeed(SimpleFOC_MotorAxis axis, float speed_rps)
{
    return MotorSpeed_SetTargetSpeed(axis, speed_rps);
}

/* 运行期间改写相对角度目标；返回底层命令是否被接收。 */
uint8_t App_MotorPosition_SetTargetDegrees(SimpleFOC_MotorAxis axis, float delta_deg)
{
    return MotorPosition_SetTargetDegrees(axis, delta_deg);
}
