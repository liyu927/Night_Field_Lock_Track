#include "user_home.h"
#include "app_system.h"
#include "cmsis_os2.h"
#include "SmaplingTypes.h"
#include <stdint.h>

#define UI_SCREEN_W          240
#define UI_CONTENT_H         260
#define UI_FOOTER_H          60
#define UI_ADC_REFRESH_MS    200
#define UI_PAGE_STACK_DEPTH  4
#define UI_PARAM_COUNT       5
#define UI_DEMO_PARAM_MIN    0
#define UI_DEMO_PARAM_MAX    9999

extern osMessageQueueId_t SmaplingHandle;

typedef enum {
    UI_PAGE_MAIN,
    UI_PAGE_CONTROL,
    UI_PAGE_PARAMS,
    UI_PAGE_ABOUT,
    UI_PAGE_APP
} UiPage;

static lv_obj_t *tileview;
static lv_obj_t *manual_tile;
/* 自动模式瓦片用于识别用户的模式切换。 */
static lv_obj_t *auto_tile;
static lv_obj_t *pages[UI_PAGE_APP + 1];
static lv_obj_t *adc_labels[8];
static lv_obj_t *auto_avg_label;
static lv_obj_t *auto_diff_label;
static lv_obj_t *direction_label;
static lv_obj_t *param_labels[UI_PARAM_COUNT];
static lv_obj_t *dialog_layer;
static lv_obj_t *dialog;
static lv_obj_t *dialog_text;
static lv_obj_t *dialog_buttons[3];
static UiPage page_stack[UI_PAGE_STACK_DEPTH];
static uint8_t page_stack_size;
static UiPage current_page;
static int32_t demo_params[UI_PARAM_COUNT];
static int32_t entry_params[UI_PARAM_COUNT];
static bool params_dirty;
static bool save_then_back;
static AppManualDirection manual_direction;

static const char *const param_names[UI_PARAM_COUNT] = {
    "光照最小差异", "光照唤醒阈值", "光照最低阈值",
    "手动基础速度", "自动基础速度"
};

static lv_obj_t *ui_label(lv_obj_t *parent, const char *text,
                          int32_t x, int32_t y, int32_t width)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_width(label, width);
    lv_label_set_text(label, text);
    return label;
}

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

static void ui_push_page(UiPage page)
{
    if (page_stack_size >= UI_PAGE_STACK_DEPTH) return;
    page_stack[page_stack_size++] = current_page;
    if (page == UI_PAGE_PARAMS && !params_dirty) {
        for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) {
            entry_params[i] = demo_params[i];
        }
    }
    ui_show_page(page);
}

static void ui_go_back(void)
{
    if (current_page == UI_PAGE_CONTROL) {
        lv_label_set_text(direction_label, "方向：未按下");
    }
    ui_show_page(page_stack_size ? page_stack[--page_stack_size] : UI_PAGE_MAIN);
}

static void ui_dialog_hide(void)
{
    lv_obj_set_hidden(dialog_layer, true);
}

static void ui_dialog_show(const char *text, bool confirm)
{
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
    ui_dialog_show("设备异常，电机已停止", false);
}

static void ui_close_dialog_cb(lv_event_t *event)
{
    (void)event;
    ui_dialog_hide();
}

static void ui_back_cb(lv_event_t *event)
{
    (void)event;
    if (current_page == UI_PAGE_PARAMS && params_dirty) {
        save_then_back = true;
        ui_dialog_show("是否保存修改？", true);
    } else {
        ui_go_back();
    }
}

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

static void ui_direction_cb(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    AppManualDirection direction = (AppManualDirection)(uintptr_t)lv_event_get_user_data(event);
    if (code == LV_EVENT_PRESSED) {
        static const char *const names[4] = {"上", "下", "左", "右"};
        manual_direction = direction;
        App_ManualControl_Send(direction);
        lv_label_set_text_fmt(direction_label, "方向：%s  目标：15°/s", names[direction - APP_MANUAL_UP]);
    } else if (code == LV_EVENT_PRESSING && manual_direction == direction) {
        App_ManualControl_Send(direction);
    } else if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST
                || code == LV_EVENT_CANCEL) && manual_direction == direction) {
        manual_direction = APP_MANUAL_STOP;
        App_ManualControl_Send(APP_MANUAL_STOP);
        lv_label_set_text(direction_label, "方向：未按下");
    }
}

static void ui_param_refresh(uint8_t index)
{
    lv_label_set_text_fmt(param_labels[index], "%ld", (long)demo_params[index]);
}

static void ui_param_change_cb(lv_event_t *event)
{
    uintptr_t code = (uintptr_t)lv_event_get_user_data(event);
    uint8_t index = (uint8_t)(code / 2U);
    int32_t delta = (code & 1U) ? 1 : -1;
    int32_t value = demo_params[index] + delta;
    if (value < UI_DEMO_PARAM_MIN) value = UI_DEMO_PARAM_MIN;
    if (value > UI_DEMO_PARAM_MAX) value = UI_DEMO_PARAM_MAX;
    demo_params[index] = value;
    params_dirty = true;
    ui_param_refresh(index);
}

static void ui_save_preview(void)
{
    params_dirty = false;
    for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) {
        entry_params[i] = demo_params[i];
    }
    if (save_then_back) {
        ui_go_back();
        save_then_back = false;
    }
    ui_dialog_show("保存成功（界面演示，未写入 Flash）", false);
}

static void ui_save_cb(lv_event_t *event)
{
    (void)event;
    save_then_back = false;
    ui_save_preview();
}

static void ui_confirm_save_cb(lv_event_t *event)
{
    (void)event;
    ui_save_preview();
}

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

static void ui_error_preview_cb(lv_event_t *event)
{
    (void)event;
    ui_dialog_show("设备异常（弹窗预览）", false);
}

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
    ui_label(manual_tile, "上滑切换自动", 8, 236, 224);
    ui_button(manual_tile, "手动控制", 16, 42, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_CONTROL);
    ui_button(manual_tile, "参数调节", 16, 90, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_PARAMS);
    ui_button(manual_tile, "产品说明", 16, 138, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_ABOUT);
    ui_button(manual_tile, "手机 App 观测", 16, 186, 208, 40, ui_open_cb, (void *)(uintptr_t)UI_PAGE_APP);

    ui_label(auto_tile, "自动模式  下滑返回手动", 8, 9, 224);
    auto_avg_label = ui_label(auto_tile, "平均光照：--", 12, 48, 216);
    auto_diff_label = ui_label(auto_tile, "光照总差：--", 12, 78, 216);
    ui_label(auto_tile, "进入自动后等待 3 s\n光差平衡时保持位置\n低光判断：待接入", 12, 120, 216);
    ui_label(auto_tile, "光强差控制已接入", 12, 225, 216);

    pages[UI_PAGE_CONTROL] = ui_subpage("手动控制");
    ui_button(pages[UI_PAGE_CONTROL], "返回", 180, 5, 52, 29, ui_back_cb, NULL);
    ui_label(pages[UI_PAGE_CONTROL], "长按方向键：15°/s，松开保持", 8, 48, 224);
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
    for (uint8_t i = 0; i < UI_PARAM_COUNT; i++) {
        int32_t y = 40 + i * 37;
        ui_label(pages[UI_PAGE_PARAMS], param_names[i], 6, y + 5, 105);
        ui_button(pages[UI_PAGE_PARAMS], "-", 112, y, 32, 29,
                  ui_param_change_cb, (void *)(uintptr_t)(i * 2U));
        param_labels[i] = ui_label(pages[UI_PAGE_PARAMS], "0", 151, y + 6, 49);
        ui_button(pages[UI_PAGE_PARAMS], "+", 200, y, 32, 29,
                  ui_param_change_cb, (void *)(uintptr_t)(i * 2U + 1U));
    }
    ui_button(pages[UI_PAGE_PARAMS], "保存", 70, 226, 100, 30, ui_save_cb, NULL);

    pages[UI_PAGE_ABOUT] = ui_subpage("产品说明");
    ui_button(pages[UI_PAGE_ABOUT], "返回", 180, 5, 52, 29, ui_back_cb, NULL);
    ui_label(pages[UI_PAGE_ABOUT], "太阳能追光系统\n\n四路光敏电阻采集光照，\n双轴云台规划用于追光。\n\n当前仅为界面预览。", 14, 48, 212);
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
