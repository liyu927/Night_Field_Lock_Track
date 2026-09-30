//
// Created by liweicong on 2026/9/19.
#include "main.h"
#include "cmsis_os2.h"
#include "lvgl.h"
#include "FreeRTOS.h"
#include "lv_port_indev.h"
#include "lv_port_lcd_stm32.h"
#include "task.h"
#include "user_home.h"
#include "adc.h"

void StartUiTask(void *argument)
{
    /* USER CODE BEGIN StartUiTask */
    //############################### LVGL #####################################
    //LVGL_0:初始准备:
    //SPI要用DMA
    //cubemx的Project Manager 里面的Linker settings堆栈内存可以适当调大一点

    //LVGL_1.调用lv_init
    lv_init();

    //LVGL_2.设置时钟
    lv_tick_set_cb(HAL_GetTick);
    //声明使用的延时函数
    lv_delay_set_cb(vTaskDelay);

    //LVGL_3.创建一个显示屏
    lv_port_disp_init();

    //LVGL_4.创建一个输入设备（触摸屏输入坐标等初始化）
    lv_port_indev_init();

    //LVGL_5.创建UI界面（注意不要占满flash）
    //lv_demo_widgets();这个demo占满flash了，跑不动
    user_home_create();

    //启动ADC1多通道DMA循环采样（PA0~PA3），DMA会持续刷新adc1_dma_buf
    ADC1_DMA_Start();

    //############################### LVGL #####################################
    /* Infinite loop */
    for(;;)
    {
        //######################### LVGL ##############################
        // LVGL 主循环负责刷新显示、读取触摸和运行动画。
        uint32_t delay_ms = lv_timer_handler();
        osDelay(delay_ms);
        //######################### LVGL ##############################
    }
    /* USER CODE END StartUiTask */
}
//