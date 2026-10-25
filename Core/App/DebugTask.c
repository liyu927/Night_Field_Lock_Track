#include "cmsis_os2.h"

void StartDebugTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        osDelay(100U);
    }
}
