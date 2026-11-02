#ifndef _USER_HOME_H_
#define _USER_HOME_H_

#include "lvgl.h"

/* 创建主界面与全屏弹窗容器。 */
void user_home_create(void);
/* UI 任务根据系统快照更新手动和自动低功耗倒计时。 */
void user_home_update_power(void);

/* 仅由 UI 任务调用，在当前任意页面显示设备异常弹窗。 */
void user_home_show_error(void);

#endif
