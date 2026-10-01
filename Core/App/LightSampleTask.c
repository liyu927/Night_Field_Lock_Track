#include "main.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "filter.h"
#include "LightSamplingTypes.h"

#define LIGHT_SAMPLE_PERIOD_MS 20U /* 光照任务周期，符合设计文档的 20 ms 发布周期。 */
#define LIGHT_LOW_PASS_SHIFT    3U /* 一阶低通右移位数，等价于使用 1/8 的滤波系数。 */

/* 由 FreeRTOS 初始化函数创建，供采样任务写入、UI 任务读取。 */
extern osMessageQueueId_t LightSamplingHandle;

/**
 * @brief 启动 ADC1 DMA，周期性滤波四路光照数据并写入消息队列。
 * @param argument CMSIS-RTOS 任务入口保留参数，本任务不使用。
 */
void StartLightSampleTask(void *argument)
{
    LightData_t light_data;                    /* 本周期完成滤波后准备发布的四路光照快照。 */
    LowPassFilter_t left_top_filter = {0};     /* PA0 左上通道的一阶低通状态。 */
    LowPassFilter_t left_bottom_filter = {0};  /* PA1 左下通道的一阶低通状态。 */
    LowPassFilter_t right_top_filter = {0};    /* PA2 右上通道的一阶低通状态。 */
    LowPassFilter_t right_bottom_filter = {0}; /* PA3 右下通道的一阶低通状态。 */

    (void)argument; /* 明确该任务不使用入口参数，避免编译器产生未使用参数告警。 */

    ADC1_DMA_Start(); /* 只启动一次 DMA，之后硬件会循环更新 adc1_dma_buf。 */

    for (;;)
    {
        /* DMA 持续刷新缓冲区，此处读取 PA0 的左上原始值并完成一阶低通。 */
        light_data.left_top = Filter_LowPass(&left_top_filter, adc1_dma_buf[0], LIGHT_LOW_PASS_SHIFT);
        /* 读取 PA1 的左下原始值并完成一阶低通。 */
        light_data.left_bottom = Filter_LowPass(&left_bottom_filter, adc1_dma_buf[1], LIGHT_LOW_PASS_SHIFT);
        /* 读取 PA2 的右上原始值并完成一阶低通。 */
        light_data.right_top = Filter_LowPass(&right_top_filter, adc1_dma_buf[2], LIGHT_LOW_PASS_SHIFT);
        /* 读取 PA3 的右下原始值并完成一阶低通。 */
        light_data.right_bottom = Filter_LowPass(&right_bottom_filter, adc1_dma_buf[3], LIGHT_LOW_PASS_SHIFT);

        (void)osMessageQueuePut(LightSamplingHandle, &light_data, 0U, 0U); /* 非阻塞发布，队列满时不延误下一次采样。 */
        osDelay(LIGHT_SAMPLE_PERIOD_MS); /* 让出 CPU，保持固定的采样与发布周期。 */
    }
}
