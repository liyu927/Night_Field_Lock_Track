#include "main.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "filter.h"
#include "LightSamplingTypes.h"

#define LIGHT_SAMPLE_PERIOD_MS 20U
#define LIGHT_LOW_PASS_SHIFT    3U

extern osMessageQueueId_t LightSamplingHandle;

void StartLightSampleTask(void *argument)
{
    LightData_t light_data;
    LowPassFilter_t left_top_filter = {0};
    LowPassFilter_t left_bottom_filter = {0};
    LowPassFilter_t right_top_filter = {0};
    LowPassFilter_t right_bottom_filter = {0};

    (void)argument;

    ADC1_DMA_Start();

    for (;;)
    {
        /* DMA 持续刷新缓冲区，此处只复制本次四路采样值。 */
        light_data.left_top = Filter_LowPass(&left_top_filter, adc1_dma_buf[0], LIGHT_LOW_PASS_SHIFT);
        light_data.left_bottom = Filter_LowPass(&left_bottom_filter, adc1_dma_buf[1], LIGHT_LOW_PASS_SHIFT);
        light_data.right_top = Filter_LowPass(&right_top_filter, adc1_dma_buf[2], LIGHT_LOW_PASS_SHIFT);
        light_data.right_bottom = Filter_LowPass(&right_bottom_filter, adc1_dma_buf[3], LIGHT_LOW_PASS_SHIFT);

        (void)osMessageQueuePut(LightSamplingHandle, &light_data, 0U, 0U);
        osDelay(LIGHT_SAMPLE_PERIOD_MS);
    }
}
