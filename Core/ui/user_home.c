#include "user_home.h"
#include "app_system.h"
#include "app_parameters.h"
#include "cmsis_os2.h"
#include "SmaplingTypes.h"
#include <stdint.h>

/* 屏幕上方 260 像素为可切换页面，底部 60 像素固定显示光敏数据。 */
#define UI_SCREEN_W          240
#define UI_CONTENT_H         260
#define UI_FOOTER_H          60
/* 光敏数据标签的刷新周期，以及页面返回栈和参数瓦片的容量。 */
#define UI_ADC_REFRESH_MS    200
#define UI_PAGE_STACK_DEPTH  4
#define UI_PARAM_COUNT       APP_PARAMETER_COUNT
#define UI_PARAMS_PER_TILE   5U

extern osMessageQueueId_t SmaplingHandle;

/* UI 页面编号用于 pages 数组和返回栈；主页面本身由上方模式 Tileview 承载。 */
typedef enum {
    UI_PAGE_MAIN,
    UI_PAGE_CONTROL,
    UI_PAGE_PARAMS,
    UI_PAGE_ABOUT,
    UI_PAGE_APP
} UiPage;

/* 主 Tileview 在手动/自动模式间纵向切换；参数 Tileview 在两页参数间横向切换。 */
static lv_obj_t *tileview;
static lv_obj_t *manual_tile;
static lv_obj_t *recovery_button;
/* 自动模式瓦片用于识别用户的模式切换。 */
static lv_obj_t *auto_tile;
static lv_obj_t *pages[UI_PAGE_APP + 1];
static lv_obj_t *adc_labels[8];
static lv_obj_t *auto_avg_label;
static lv_obj_t *auto_diff_label;
static lv_obj_t *direction_label;
/* 每个参数的数值标签与亮度滑块；页码标签和底部按钮不随参数瓦片移动。 */
static lv_obj_t *param_labels[UI_PARAM_COUNT];
static lv_obj_t *param_slider;
static lv_obj_t *param_tileview;
static lv_obj_t *param_page_label;
static lv_obj_t *dialog_layer;
static lv_obj_t *dialog;
static lv_obj_t *dialog_text;
static lv_obj_t *dialog_buttons[3];
static UiPage page_stack[UI_PAGE_STACK_DEPTH];
static uint8_t page_stack_size;
static UiPage current_page;
/* demo_params 是屏幕上的待保存草稿，entry_params 是进入页面时的已生效快照。 */
static int32_t demo_params[UI_PARAM_COUNT];
static int32_t entry_params[UI_PARAM_COUNT];
static bool params_dirty;      /* 草稿变化后，返回时询问保存还是放弃。 */
static bool param_refreshing;  /* 程序设置滑块时阻止回调把操作误判为用户修改。 */
static bool save_then_back;    /* 从返回弹窗保存成功后再退出参数页。 */
static uint8_t dialog_action;  /* 0 为保存草稿，1 为恢复出厂，2 为回归旧版。 */
static AppManualDirection manual_direction;

/* 标签顺序必须与 AppParameterIndex 一致，索引同时用于取范围和读取参数值。 */
static const char *const param_names[UI_PARAM_COUNT] = {
    "亮度调节", "手动熄屏时间", "自动熄屏时间", "左右光最小差异", "上下光最小差异",
    "低功耗光强阈值", "自动唤醒读光时间", "手动基础速度", "自动Kp"
};

static void ui_param_refresh(uint8_t index);

/* 在指定父对象内创建文本标签，并用像素坐标确定位置和显示宽度。 */
static lv_obj_t *ui_label(lv_obj_t *parent, const char *text,
                          int32_t x, int32_t y, int32_t width)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_width(label, width);
    lv_label_set_text(label, text);
    return label;
}

/* 创建带居中标题的按钮；callback 非空时绑定单击事件并传入 user_data。 */
static lv_obj_t *ui_button(lv_obj_t *parent, const char *text,
                           int32_t x, int32_t y, int32_t width, int32_t height,
                           lv_event_cb_t callback, void *user_data)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, height);
    if (callback != NULL) {
        lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, user_data);
    }
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    return button;
}

/* 创建默认隐藏的内容页面，不覆盖屏幕底部固定的光敏数据栏。 */
static lv_obj_t *ui_subpage(const char *title)
{
    lv_obj_t *page = lv_obj_create(lv_screen_active());
    lv_obj_set_pos(page, 0, 0);
    lv_obj_set_size(page, UI_SCREEN_W, UI_CONTENT_H);
    lv_obj_set_style_pad_all(page, 0, 0);
    lv_obj_set_scrollable(page, false);
    lv_obj_set_hidden(page, true);
    ui_label(page, title, 8, 8, 164);
    return page;
}

/* 隐藏旧页面并显示目标页面；离开手动控制页时先发送停止命令。 */
static void ui_show_page(UiPage page)
{
    if (current_page == UI_PAGE_CONTROL && page != UI_PAGE_CONTROL) {
        manual_direction = APP_MANUAL_STOP;
        App_ManualControl_Send(APP_MANUAL_STOP);
    }
    lv_obj_set_hidden(tileview, true);
    if (current_page != UI_PAGE_MAIN) {
        lv_obj_set_hidden(pages[current_page], true);
    }
    current_page = page;
    if (page == UI_PAGE_MAIN) {
        lv_obj_set_hidden(tileview, false);
    } else {
        lv_obj_set_hidden(pages[page], false);
    }
}

/* 记录返回路径；进入参数页时从运行时配置重新装载草稿和数值标签。 */
static void ui_push_page(UiPage page)
{
    if (page_stack_size >= UI_PAGE_STACK_DEPTH) return;
    page_stack[page_stack_size++] = current_page;
    if (page == UI_PAGE_PARAMS && !params_dirty) {
        AppParameters active = App_Parameters_Get();
        for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) {
            entry_params[i] = active.values[i];
            demo_params[i] = active.values[i];
            if (param_labels[i] != NULL) ui_param_refresh(i);
        }
    }
    ui_show_page(page);
}

/* 从页面栈弹出上一页；没有历史记录时返回模式主页面。 */
static void ui_go_back(void)
{
    if (current_page == UI_PAGE_CONTROL) {
        lv_label_set_text(direction_label, "方向：未按下");
    }
    ui_show_page(page_stack_size ? page_stack[--page_stack_size] : UI_PAGE_MAIN);
}

/* 只隐藏弹窗遮罩，不改变其下方页面或参数草稿。 */
static void ui_dialog_hide(void)
{
    lv_obj_set_hidden(dialog_layer, true);
}

/* confirm 为真时显示操作按钮和取消按钮；为假时仅显示提示和关闭按钮。 */
static void ui_dialog_show(const char *text, bool confirm)
{
    dialog_action = 0U;
    lv_label_set_text(dialog_text, text);
    for (uint8_t i = 0; i < 2; i++) {
        if (confirm) lv_obj_set_hidden(dialog_buttons[i], false);
        else lv_obj_set_hidden(dialog_buttons[i], true);
    }
    lv_obj_set_pos(dialog_buttons[2], confirm ? 149 : 77, 100);
    lv_obj_t *close_label = lv_obj_get_child(dialog_buttons[2], 0);
    lv_label_set_text(close_label, confirm ? "取消" : "关闭");
    lv_obj_set_hidden(dialog_layer, false);
    lv_obj_move_foreground(dialog_layer);
}

/* 使用屏幕根对象上的遮罩层，当前页面无论显示什么都能弹出。 */
void user_home_show_error(void)
{
    ui_show_page(UI_PAGE_MAIN);
    page_stack_size = 0U;
    params_dirty = false;
    save_then_back = false;
    lv_tileview_set_tile(tileview, manual_tile, LV_ANIM_OFF);
    lv_obj_remove_state(recovery_button, LV_STATE_DISABLED);
    lv_obj_set_hidden(recovery_button, false);
    ui_dialog_show("设备异常，电机已停止", false);
}

void user_home_show_recovery_result(uint8_t result)
{
    if (result == 1U) lv_obj_set_hidden(recovery_button, true);
    else lv_obj_remove_state(recovery_button, LV_STATE_DISABLED);
    ui_dialog_show(result == 1U ? "恢复成功，一切恢复正常" : "保存失败", false);
}

/* 恢复按钮只登记请求，检查和故障清除分别由电机任务、系统任务执行。 */
static void ui_recovery_cb(lv_event_t *event)
{
    (void)event;
    App_System_RequestRecovery();
    lv_obj_add_state(recovery_button, LV_STATE_DISABLED);
}

/* 点击关闭或取消后收起弹窗，保留当前页面状态。 */
static void ui_close_dialog_cb(lv_event_t *event)
{
    (void)event;
    ui_dialog_hide();
}

/* 参数草稿未保存时先询问；没有修改时直接返回。 */
static void ui_back_cb(lv_event_t *event)
{
    (void)event;
    if (current_page == UI_PAGE_PARAMS && params_dirty) {
        save_then_back = true;
        ui_dialog_show("是否保存修改？", true);
        lv_label_set_text(lv_obj_get_child(dialog_buttons[0], 0), "保存");
        lv_obj_set_hidden(dialog_buttons[1], false);
    } else {
        ui_go_back();
    }
}

/* 主页面按钮将目标页编号放在 user_data 中，点击后压栈进入子页面。 */
static void ui_open_cb(lv_event_t *event)
{
    UiPage page = (UiPage)(uintptr_t)lv_event_get_user_data(event);
    ui_push_page(page);
}

/* 瓦片切换只请求改变模式，电机输出由系统任务决定。 */
static void ui_tile_changed_cb(lv_event_t *event)
{
    (void)event;
    App_System_RequestMode(lv_tileview_get_tile_active(tileview) == auto_tile
        ? APP_MODE_AUTO : APP_MODE_MANUAL);
}

/* 按下和长按时持续发送方向命令，松开或触摸取消时立刻停止。 */
/* 显示的目标速度取当前已生效参数，不使用写死的出厂速度。 */
static void ui_direction_cb(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    AppManualDirection direction = (AppManualDirection)(uintptr_t)lv_event_get_user_data(event);
    if (code == LV_EVENT_PRESSED) {
        static const char *const names[4] = {"上", "下", "左", "右"};
        manual_direction = direction;
        App_ManualControl_Send(direction);
        AppParameters active = App_Parameters_Get();
        lv_label_set_text_fmt(direction_label, "方向：%s  目标：%u°/s",
                              names[direction - APP_MANUAL_UP], active.values[APP_PARAM_MANUAL_SPEED]);
    } else if (code == LV_EVENT_PRESSING && manual_direction == direction) {
        App_ManualControl_Send(direction);
    } else if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST
                || code == LV_EVENT_CANCEL) && manual_direction == direction) {
        manual_direction = APP_MANUAL_STOP;
        App_ManualControl_Send(APP_MANUAL_STOP);
        lv_label_set_text(direction_label, "方向：未按下");
    }
}

/* 根据草稿刷新指定参数的数字；Kp 以两位小数显示，内部仍按百分之一存储。 */
/* 亮度同步到滑块时使用 param_refreshing 防止触发“用户修改”回调。 */
static void ui_param_refresh(uint8_t index)
{
    if (index == APP_PARAM_AUTO_KP)
        lv_label_set_text_fmt(param_labels[index], "%ld.%02ld",
                              (long)(demo_params[index] / 100), (long)(demo_params[index] % 100));
    else
        lv_label_set_text_fmt(param_labels[index], "%ld", (long)demo_params[index]);
    if (index == APP_PARAM_BRIGHTNESS && param_slider != NULL
        && lv_slider_get_value(param_slider) != demo_params[index]) {
        param_refreshing = true;
        lv_slider_set_value(param_slider, demo_params[index], LV_ANIM_OFF);
        param_refreshing = false;
    }
}

/* 加减按钮用 user_data 的低位区分方向，其余位表示参数下标。 */
/* 单击或长按重复事件每次调整一个整数单位，达到上下限后保持边界值。 */
static void ui_param_change_cb(lv_event_t *event)
{
    uintptr_t code = (uintptr_t)lv_event_get_user_data(event);
    uint8_t index = (uint8_t)(code / 2U);
    int32_t delta = (code & 1U) ? 1 : -1;
    int32_t value = demo_params[index] + delta;
    if (value < App_Parameters_Min(index)) value = App_Parameters_Min(index);
    if (value > App_Parameters_Max(index)) value = App_Parameters_Max(index);
    demo_params[index] = value;
    params_dirty = true;
    ui_param_refresh(index);
}

/* 用户拖动亮度滑块时更新草稿，实际 PWM 在保存成功后由系统任务应用。 */
static void ui_brightness_cb(lv_event_t *event)
{
    if (param_refreshing) return;
    demo_params[APP_PARAM_BRIGHTNESS] = lv_slider_get_value(lv_event_get_target(event));
    params_dirty = true;
    ui_param_refresh(APP_PARAM_BRIGHTNESS);
}

/* 把九项草稿写入用户双槽；底层读回成功后运行时参数立即生效。 */
/* 保存失败保留草稿供重试；从返回弹窗保存成功则先退页再提示成功。 */
static void ui_save_parameters(void)
{
    AppParameters changed;
    for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) changed.values[i] = (uint16_t)demo_params[i];
    if (App_Parameters_Save(&changed) != HAL_OK) {
        save_then_back = false;
        ui_dialog_show("保存失败，请重试", false);
        return;
    }
    params_dirty = false;
    for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) entry_params[i] = demo_params[i];
    if (save_then_back) {
        ui_go_back();
        save_then_back = false;
    }
    ui_dialog_show("保存成功", false);
}

/* 固定“保存”按钮直接保存当前草稿，保存后继续停留在参数页。 */
static void ui_save_cb(lv_event_t *event)
{
    (void)event;
    save_then_back = false;
    ui_save_parameters();
}

/* 弹窗“确定”：按 dialog_action 保存草稿、恢复出厂或回归旧版。 */
/* 恢复操作成功后重新读取已生效参数，刷新两页显示和草稿快照。 */
static void ui_confirm_save_cb(lv_event_t *event)
{
    (void)event;
    if (dialog_action == 0U) {
        ui_save_parameters();
        return;
    }
    HAL_StatusTypeDef status = dialog_action == 1U
        ? App_Parameters_RestoreFactory() : App_Parameters_RestorePrevious();
    if (status == HAL_OK) {
        AppParameters active = App_Parameters_Get();
        for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) {
            demo_params[i] = active.values[i];
            entry_params[i] = active.values[i];
            ui_param_refresh(i);
        }
        params_dirty = false;
        ui_dialog_show("已保存并生效", false);
    } else {
        ui_dialog_show(status == HAL_BUSY ? "旧版本不可用" : "操作失败，请重试", false);
    }
}

/* 恢复按钮只打开确定/取消弹窗；真正的 Flash 操作留给确认回调执行。 */
static void ui_restore_cb(lv_event_t *event)
{
    uint8_t action = (uint8_t)(uintptr_t)lv_event_get_user_data(event);
    ui_dialog_show(action == 1U ? "恢复出厂设置并保存？" : "回归旧版本并保存？", true);
    dialog_action = action;
    lv_label_set_text(lv_obj_get_child(dialog_buttons[0], 0), "确定");
    lv_obj_set_hidden(dialog_buttons[1], true);
}

/* 参数 Tileview 横向切页时更新固定角标，显示当前页/总页数。 */
static void ui_param_tile_cb(lv_event_t *event)
{
    (void)event;
    lv_obj_t *tile = lv_tileview_get_tile_active(param_tileview);
    lv_label_set_text(param_page_label, lv_obj_get_index(tile) == 0 ? "1/2" : "2/2");
}

/* 返回弹窗中选择“放弃”时恢复进入页面前的快照，再退出参数页。 */
static void ui_discard_cb(lv_event_t *event)
{
    (void)event;
    for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) {
        demo_params[i] = entry_params[i];
        ui_param_refresh(i);
    }
    params_dirty = false;
    save_then_back = false;
    ui_dialog_hide();
    ui_go_back();
}

/* 产品说明页保留的异常弹窗预览入口，不主动触发电机故障。 */
static void ui_error_preview_cb(lv_event_t *event)
{
    (void)event;
    ui_dialog_show("设备异常（弹窗预览）", false);
}

/* 从采样队列取最新一帧，计算均值、总差和两轴差值并刷新固定底栏。 */
/* 队列为空时沿用上次显示，不把旧采样误当成新的传感器读数。 */
static void ui_adc_timer_cb(lv_timer_t *timer)
{
    SmaplingData_t sample;
    bool received = false;
    (void)timer;
    if (SmaplingHandle == NULL) return;
    while (osMessageQueueGet(SmaplingHandle, &sample, NULL, 0U) == osOK) {
        received = true;
    }
    if (!received) return;

    uint16_t max = sample.left_top;
    uint16_t min = sample.left_top;
    uint16_t values[4] = {sample.left_top, sample.left_bottom,
                          sample.right_top, sample.right_bottom};
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (values[i] > max) max = values[i];
        if (values[i] < min) min = values[i];
        sum += values[i];
    }
    int32_t x_diff = (int32_t)values[0] + values[1] - values[2] - values[3];
    int32_t y_diff = (int32_t)values[0] + values[2] - values[1] - values[3];
    lv_label_set_text_fmt(adc_labels[0], "左上%u", (unsigned int)values[0]);
    lv_label_set_text_fmt(adc_labels[1], "左下%u", (unsigned int)values[1]);
    lv_label_set_text_fmt(adc_labels[2], "右上%u", (unsigned int)values[2]);
    lv_label_set_text_fmt(adc_labels[3], "右下%u", (unsigned int)values[3]);
    lv_label_set_text_fmt(adc_labels[4], "均%lu", (unsigned long)(sum / 4U));
    lv_label_set_text_fmt(adc_labels[5], "差%u", (unsigned int)(max - min));
    lv_label_set_text_fmt(adc_labels[6], "左右差%ld", (long)x_diff);
    lv_label_set_text_fmt(adc_labels[7], "上下差%ld", (long)y_diff);
    lv_label_set_text_fmt(auto_avg_label, "平均光照：%lu", (unsigned long)(sum / 4U));
    lv_label_set_text_fmt(auto_diff_label, "光照总差：%u", (unsigned int)(max - min));
}

/* 创建手动/自动主瓦片、参数双页、固定光敏底栏及全屏弹窗。 */
/* 参数页每页最多五项；保存、恢复出厂和回归旧版按钮创建在瓦片外。 */
void user_home_create(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_clean(screen);

    tileview = lv_tileview_create(screen);
    lv_obj_set_pos(tileview, 0, 0);
    lv_obj_set_size(tileview, UI_SCREEN_W, UI_CONTENT_H);
    manual_tile = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_BOTTOM);
    auto_tile = lv_tileview_add_tile(tileview, 0, 1, LV_DIR_TOP);
    lv_obj_add_event_cb(tileview, ui_tile_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_style_pad_all(manual_tile, 0, 0);
    lv_obj_set_style_pad_all(auto_tile, 0, 0);
    lv_obj_set_scrollable(manual_tile, false);
    lv_obj_set_scrollable(auto_tile, false);
    /* 手动标题缩短，右侧留给最后五秒的统一倒计时。 */
    ui_label(manual_tile, "手动模式", 8, 9, 100);
    recovery_button = ui_button(manual_tile, "恢复电机", 160, 5, 72, 29,
                                ui_recovery_cb, NULL);
    lv_obj_set_hidden(recovery_button, true);
    ui_label(manual_tile, "上滑切换自动", 8, 236, 224);
    ui_button(manual_tile, "手动控制", 16, 42, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_CONTROL);
    ui_button(manual_tile, "参数调节", 16, 90, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_PARAMS);
    ui_button(manual_tile, "产品说明", 16, 138, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_ABOUT);
    ui_button(manual_tile, "手机 App 观测", 16, 186, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_APP);

    ui_label(auto_tile, "自动模式  下滑返回手动", 8, 9, 224);
    auto_avg_label = ui_label(auto_tile, "平均光照：--", 12, 48, 216);
    auto_diff_label = ui_label(auto_tile, "光照总差：--", 12, 78, 216);
    ui_label(auto_tile, "进入自动后等待 3 s\n光差平衡时保持位置\n低光判断已接入", 12, 120, 216);
    ui_label(auto_tile, "光强差控制已接入", 12, 225, 216);

    pages[UI_PAGE_CONTROL] = ui_subpage("手动控制");
    ui_button(pages[UI_PAGE_CONTROL], "返回", 180, 5, 52, 29, ui_back_cb, NULL);
    ui_label(pages[UI_PAGE_CONTROL], "长按方向键，松开保持", 8, 48, 224);
    lv_obj_t *directions[4];
    directions[0] = ui_button(pages[UI_PAGE_CONTROL], "上", 94, 78, 52, 42, NULL, NULL);
    directions[1] = ui_button(pages[UI_PAGE_CONTROL], "下", 94, 174, 52, 42, NULL, NULL);
    directions[2] = ui_button(pages[UI_PAGE_CONTROL], "左", 28, 126, 52, 42, NULL, NULL);
    directions[3] = ui_button(pages[UI_PAGE_CONTROL], "右", 160, 126, 52, 42, NULL, NULL);
    for (uint8_t i = 0; i < 4; i++) {
        void *direction = (void *)(uintptr_t)(APP_MANUAL_UP + i);
        lv_obj_add_event_cb(directions[i], ui_direction_cb, LV_EVENT_PRESSED, direction);
        lv_obj_add_event_cb(directions[i], ui_direction_cb, LV_EVENT_PRESSING, direction);
        lv_obj_add_event_cb(directions[i], ui_direction_cb, LV_EVENT_RELEASED, direction);
        lv_obj_add_event_cb(directions[i], ui_direction_cb, LV_EVENT_PRESS_LOST, direction);
        lv_obj_add_event_cb(directions[i], ui_direction_cb, LV_EVENT_CANCEL, direction);
    }
    direction_label = ui_label(pages[UI_PAGE_CONTROL], "方向：未按下", 8, 230, 224);

    pages[UI_PAGE_PARAMS] = ui_subpage("参数调节");
    ui_button(pages[UI_PAGE_PARAMS], "返回", 180, 5, 52, 29, ui_back_cb, NULL);
    param_page_label = ui_label(pages[UI_PAGE_PARAMS], "1/2", 142, 11, 35);
    /* 参数 Tileview 只占页面中段，左右滑动不会移动标题、页码和底部操作按钮。 */
    param_tileview = lv_tileview_create(pages[UI_PAGE_PARAMS]);
    lv_obj_set_pos(param_tileview, 0, 39);
    lv_obj_set_size(param_tileview, 240, 160);
    lv_obj_set_style_pad_all(param_tileview, 0, 0);
    lv_obj_add_event_cb(param_tileview, ui_param_tile_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *param_tiles[2];
    param_tiles[0] = lv_tileview_add_tile(param_tileview, 0, 0, LV_DIR_RIGHT);
    param_tiles[1] = lv_tileview_add_tile(param_tileview, 1, 0, LV_DIR_LEFT);
    for (uint8_t i = 0; i < 2; i++) {
        lv_obj_set_style_pad_all(param_tiles[i], 0, 0);
        lv_obj_set_scrollable(param_tiles[i], false);
    }
    /* 枚举下标决定瓦片和行号；亮度为滑块，其余参数用加减按钮调节。 */
    for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) {
        lv_obj_t *tile = param_tiles[i / UI_PARAMS_PER_TILE];
        int32_t y = (i % UI_PARAMS_PER_TILE) * 31;
        ui_label(tile, param_names[i], 2, y + 5, 130);
        if (i == APP_PARAM_BRIGHTNESS) {
            param_slider = lv_slider_create(tile);
            lv_obj_set_pos(param_slider, 125, y + 10);
            lv_obj_set_size(param_slider, 72, 10);
            lv_slider_set_range(param_slider, App_Parameters_Min(i), App_Parameters_Max(i));
            lv_obj_add_event_cb(param_slider, ui_brightness_cb, LV_EVENT_VALUE_CHANGED, NULL);
            param_labels[i] = ui_label(tile, "100", 202, y + 5, 36);
        } else {
            lv_obj_t *minus = ui_button(tile, "-", 131, y, 25, 28,
                                        ui_param_change_cb, (void *)(uintptr_t)(i * 2U));
            param_labels[i] = ui_label(tile, "0", 158, y + 5, 48);
            lv_obj_t *plus = ui_button(tile, "+", 209, y, 25, 28,
                                       ui_param_change_cb, (void *)(uintptr_t)(i * 2U + 1U));
            lv_obj_add_event_cb(minus, ui_param_change_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                                (void *)(uintptr_t)(i * 2U));
            lv_obj_add_event_cb(plus, ui_param_change_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                                (void *)(uintptr_t)(i * 2U + 1U));
        }
    }
    ui_button(pages[UI_PAGE_PARAMS], "保存", 5, 207, 70, 42, ui_save_cb, NULL);
    ui_button(pages[UI_PAGE_PARAMS], "恢复出厂\n设置", 79, 207, 78, 42,
              ui_restore_cb, (void *)(uintptr_t)1U);
    ui_button(pages[UI_PAGE_PARAMS], "回归旧\n版本", 161, 207, 74, 42,
              ui_restore_cb, (void *)(uintptr_t)2U);

    pages[UI_PAGE_ABOUT] = ui_subpage("产品说明");
    ui_button(pages[UI_PAGE_ABOUT], "返回", 180, 5, 52, 29, ui_back_cb, NULL);
    ui_label(pages[UI_PAGE_ABOUT], "监控夜间光场自动锁定跟踪平台\n\n四路光敏电阻采集光照，\n双轴云台自动锁定跟踪。", 14, 48, 212);
    ui_button(pages[UI_PAGE_ABOUT], "预览异常提示", 35, 202, 170, 38, ui_error_preview_cb, NULL);

    pages[UI_PAGE_APP] = ui_subpage("手机 App 观测");
    ui_button(pages[UI_PAGE_APP], "返回", 180, 5, 52, 29, ui_back_cb, NULL);

    lv_obj_t *footer = lv_obj_create(screen);
    lv_obj_set_pos(footer, 0, UI_CONTENT_H);
    lv_obj_set_size(footer, UI_SCREEN_W, UI_FOOTER_H);
    lv_obj_set_style_pad_all(footer, 0, 0);
    lv_obj_set_scrollable(footer, false);
    /* 三行标签均继承同一字体；左右差为左减右，上下差为上减下。 */
    adc_labels[0] = ui_label(footer, "左上--", 3, 3, 77);
    adc_labels[1] = ui_label(footer, "左下--", 80, 3, 77);
    adc_labels[2] = ui_label(footer, "右上--", 157, 3, 80);
    adc_labels[3] = ui_label(footer, "右下--", 3, 21, 77);
    adc_labels[4] = ui_label(footer, "均--", 80, 21, 77);
    adc_labels[5] = ui_label(footer, "差--", 157, 21, 80);
    adc_labels[6] = ui_label(footer, "左右差--", 3, 39, 115);
    adc_labels[7] = ui_label(footer, "上下差--", 120, 39, 117);

    dialog_layer = lv_obj_create(screen);
    lv_obj_set_pos(dialog_layer, 0, 0);
    lv_obj_set_size(dialog_layer, UI_SCREEN_W, UI_CONTENT_H + UI_FOOTER_H);
    lv_obj_set_style_pad_all(dialog_layer, 0, 0);
    lv_obj_set_style_border_width(dialog_layer, 0, 0);
    lv_obj_set_style_bg_color(dialog_layer, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(dialog_layer, LV_OPA_50, 0);
    lv_obj_set_scrollable(dialog_layer, false);
    dialog = lv_obj_create(dialog_layer);
    lv_obj_set_pos(dialog, 10, 84);
    lv_obj_set_size(dialog, 220, 151);
    lv_obj_set_style_pad_all(dialog, 0, 0);
    lv_obj_set_scrollable(dialog, false);
    dialog_text = ui_label(dialog, "", 10, 16, 175);
    ui_button(dialog, "X", 185, 5, 28, 28, ui_close_dialog_cb, NULL);
    dialog_buttons[0] = ui_button(dialog, "保存", 5, 100, 66, 37, ui_confirm_save_cb, NULL);
    dialog_buttons[1] = ui_button(dialog, "放弃", 77, 100, 66, 37, ui_discard_cb, NULL);
    dialog_buttons[2] = ui_button(dialog, "取消", 149, 100, 66, 37, ui_close_dialog_cb, NULL);
    lv_obj_set_hidden(dialog_layer, true);

    current_page = UI_PAGE_MAIN;
    manual_direction = APP_MANUAL_STOP;
    lv_tileview_set_tile(tileview, manual_tile, LV_ANIM_OFF);
    lv_timer_create(ui_adc_timer_cb, UI_ADC_REFRESH_MS, NULL);
}
