#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_APP_SYSTEM_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_APP_SYSTEM_H

#include <stdint.h>

//============ 光照采样任务应用层函数 ============

/* 启动 ADC1 DMA，并清空四路滤波状态；采样任务启动时调用一次。 */
void App_Smapling_Init(void);

/* 读取四路 DMA 值，依次做中值和低通滤波，再非阻塞发布一帧；每 20 ms 调用一次。 */
void App_Smapling_Run(void);

//============ UI 任务应用层函数 ============

/* 初始化 LVGL、显示屏、触摸输入和首页；UI 任务启动时调用一次。 */
void App_Ui_Init(void);

/* 处理一次 LVGL 定时器；返回建议的下次调用间隔，单位毫秒。 */
uint32_t App_Ui_Run(void);

//============ 电机任务应用层函数 ============

/* 上电准备 TIM1 后进入速度模式；speed_rps 单位圈/秒，正负号决定方向。 */
/* 调用后长期运行；绝对值不得超过 0.5 圈/秒，0 表示目标速度为零但不会关闭 EN。 */
void App_MotorSpeed_Run(float speed_rps);

/* 上电准备 TIM1 后进入位置模式；delta_deg 为相对当前位置的角度，单位度。 */
/* 调用后长期运行；每次新命令重设位置命令零点，不清零编码器或速度反馈。 */
void App_MotorPosition_Run(float delta_deg);

/* 运行期间更新速度目标并切换到速度模式；未就绪、故障或超限返回 0，成功返回 1。 */
/* speed_rps 单位圈/秒，允许范围为 -0.5～0.5；0 不关闭 EN，仅可在任务上下文调用。 */
uint8_t App_MotorSpeed_SetTargetSpeed(float speed_rps);

/* 运行期间更新相对角度目标并切换到位置模式；未就绪、故障或超限返回 0。 */
/* delta_deg 单位度，范围为 -360～360；频繁调用会持续把目标推向当前位置前方。 */
uint8_t App_MotorPosition_SetTargetDegrees(float delta_deg);

#endif /* SOLAR_LIGHT_TRACKING_SYSTEM_APP_SYSTEM_H */
