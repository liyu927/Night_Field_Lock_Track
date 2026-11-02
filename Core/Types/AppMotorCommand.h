#ifndef APP_MOTOR_COMMAND_H
#define APP_MOTOR_COMMAND_H

/* 系统任务发给电机任务的双轴相对角度命令；零度表示锁定当前位置。 */
typedef struct
{
    float vertical_deg;      /* TIM1 竖直轴的相对目标角度。 */
    float horizontal_deg;    /* TIM8 水平轴的相对目标角度。 */
} AppMotorCommand;

#endif
