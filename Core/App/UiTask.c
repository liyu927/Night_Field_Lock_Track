#include "app_system.h"
#include "cmsis_os2.h"

/* UI 任务只负责调度；LVGL 初始化和刷新由应用层执行。 */
void StartUiTask(void *argument)
{
    (void)argument;
    App_Ui_Init();

    for (;;)
    {
        osDelay(App_Ui_Run());
    }
}
