//
// Created by liweicong on 2026/9/19.\
#include "main.h"
#include "cmsis_os2.h"
#include "com_Debug.h"
#include "SmaplingTypes.h"

/* 由 FreeRTOS 初始化函数创建，专门向 VOFA+ 调试任务发送滤波后的光照数据。 */
extern osMessageQueueId_t SmapToVofaHandle;

void StartDebugTask(void *argument)
{
    SmaplingData_t smapling_data;
    float light_wave[4];

    (void)argument;

    for(;;)
    {
        /* 阻塞等待最新滤波快照，避免空转占用 CPU。 */
        if (osMessageQueueGet(SmapToVofaHandle, &smapling_data, NULL, osWaitForever) == osOK)
        {
            /* 数组顺序固定为 PA0、PA1、PA2、PA3，对应 VOFA+ 的通道 0 到通道 3。 */
            light_wave[0] = (float)smapling_data.left_top;
            light_wave[1] = (float)smapling_data.left_bottom;
            light_wave[2] = (float)smapling_data.right_top;
            light_wave[3] = (float)smapling_data.right_bottom;

            /* 一次发送四路数据，确保 VOFA+ 解析为同一时刻的四条波形。 */
            (void)VOFA_SendValues(light_wave, 4U);
        }
    }
}

//
