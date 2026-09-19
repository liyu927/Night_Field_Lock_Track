//
// Created by liweicong on 2026/9/19.
#include "main.h"
#include "cmsis_os2.h"
#include "lvgl.h"
#include "FreeRTOS.h"

void StartUiTask(void *argument)
{
    /* USER CODE BEGIN StartUiTask */
    /* Infinite loop */
    for(;;)
    {
        //######################### LVGL ##############################
        // LVGL 主循环负责刷新显示、读取触摸和运行动画。
        uint32_t delay_ms = lv_timer_handler();
        if(delay_ms > 20U) delay_ms = 20U;
        if(delay_ms < 1U) delay_ms = 1U;
        osDelay(delay_ms);
        //######################### LVGL ##############################
    }
    /* USER CODE END StartUiTask */
}
//