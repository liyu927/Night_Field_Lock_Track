//
// Created by liweicong on 2026/9/19.\
#include "main.h"
#include "cmsis_os2.h"

void StartDebugTask(void *argument)
{
    /* USER CODE BEGIN StartDebugTask */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDebugTask */
}

//