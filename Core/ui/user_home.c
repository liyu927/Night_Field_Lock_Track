#include "user_home.h"
#include "cmsis_os2.h"
#include "LightSamplingTypes.h"

lv_subject_t temp_subject;

/*==================== ADC 电压显示（观察者模式） ====================*/
static lv_subject_t adc_subject[4];      /* PA0~PA3 各自的主题 */
static const char *adc_fmt[4] = {"PA0:%d", "PA1:%d", "PA2:%d", "PA3:%d"};

extern osMessageQueueId_t LightSamplingHandle;

/**
 * @brief LVGL 定时器回调：把光照采样队列中的最新滤波值写入各主题
 * @note  值变化时自动刷新绑定的 label，运行在 UiTask 上下文
 */
static void adc_display_timer_cb(lv_timer_t *timer)
{
    LightData_t light_data;

    (void)timer;
    while (osMessageQueueGet(LightSamplingHandle, &light_data, NULL, 0U) == osOK)
    {
        lv_subject_set_int(&adc_subject[0], light_data.left_top);
        lv_subject_set_int(&adc_subject[1], light_data.left_bottom);
        lv_subject_set_int(&adc_subject[2], light_data.right_top);
        lv_subject_set_int(&adc_subject[3], light_data.right_bottom);
    }
}

//(1)：observer参数:同一个回调函数可以应用到多个观察者，通过东西一个参数的地址来表示当前触发回调的是哪个观察者,其值就是lv_subject_add_observer返回的值
//(2)：subject参数：：添加观察者的主题 =》 对应观察的对象（跟平常的回调函数那个参数一样）
void temp_obs_cb(lv_observer_t * observer, lv_subject_t *subject)
{
    int32_t pre_value = lv_subject_get_previous_int(subject);//获取上一次的值
    int32_t cur_value = lv_subject_get_int(subject);//获取当前的值
    printf("pre_value: %d, cur_value: %d\n", pre_value, cur_value);

}

void btn1_event_cb(lv_event_t *e)
{
    lv_subject_t *subject = lv_event_get_user_data(e);
    int32_t cur_value = lv_subject_get_int(subject);
    lv_subject_set_int(subject, cur_value + 1);
}

void btn2_event_cb(lv_event_t *e)
{
    lv_subject_t *subject = lv_event_get_user_data(e);
    int32_t cur_value = lv_subject_get_int(subject);
    lv_subject_set_int(subject, cur_value - 1);

}




static const char * const languages[] = {"en", "chinese", NULL};//支持语言
static const char * const tags[] = {"obs", "translation", NULL};//标签页
static const char * const translations[] = {
    //LVGL规定字符串只能写成UTF-8的形式，不然LVGL无法识别
    //keil编译器中若使用GBK，会出现不定期的乱码然后报错
    //可以在下面的字符串前后加空格解决，例如：" 观察者 "
    "obs",        " 观察者 ",   /* tag "obs":         [en, chinese] */
    "translation", "翻译"};   /* tag "translation": [en, chinese] */

void dropdown_event_cb(lv_event_t *e)
{
    //1.获取下拉菜单组件
    lv_obj_t *dropdown = lv_event_get_user_data(e);

    //2.获取当前选项下标
    uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    printf("selected_index:%d\n",selected_index);
    if(selected_index == 0)
    {
        lv_translation_set_language("chinese");
    }
    else if(selected_index == 1)
    {
        lv_translation_set_language("en");
    }

}


void language_change_event_cb(lv_event_t *e)
{
    lv_obj_t *label = lv_event_get_target_obj(e);//返回label0或label1
    const char *tag = lv_event_get_user_data(e);//返回tag对应的那个值
    lv_label_set_text(label,lv_tr(tag));//对标签对应的值切换语言

}

/**
 * @brief 观察者模式和翻译
 *
 */
void user_home_create(void)
{


    /*观察者模式
    lv_obj_t *scr = lv_screen_active();
    //1.创建一个标签视图
    lv_obj_t *tabview = lv_tabview_create(scr);
    //1.1设置标签视图的样式
    lv_obj_set_size(tabview, LV_PCT(100), LV_PCT(100));

    //1.2创建一个标签页
    lv_obj_t *obs_table = lv_tabview_add_tab(tabview, "Obs");//观察者页面
    lv_obj_t *translation = lv_tabview_add_tab(tabview, "translation");//翻译页面

    //添加对应的组件到obs里面

    //2.1外部参数的观察者模式

    //2.1.1初始化一个主题 =》 选择正确的类型和初始值
    lv_subject_init_int(&temp_subject,10);

    //2.1.2添加外部参数的观察者(回调)
    lv_subject_add_observer(&temp_subject,temp_obs_cb,NULL);//回调触发：当参数值（lv_subject_init_int的第二个值）改变时会触发


    //2.2创建按钮 =》 调整温度

    //加温度值
    lv_obj_t *btn1 = lv_btn_create(obs_table);
    lv_obj_set_size(btn1, 50, 50);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 50, 0);
    lv_obj_add_event_cb(btn1, btn1_event_cb, LV_EVENT_CLICKED, &temp_subject);


    //减温度值
    lv_obj_t *btn2 = lv_btn_create(obs_table);
    lv_obj_set_size(btn2, 50, 50);
    lv_obj_align(btn2, LV_ALIGN_CENTER, -50, 0);
    lv_obj_add_event_cb(btn2, btn2_event_cb, LV_EVENT_CLICKED, &temp_subject);


    //2.2.1创建标签 =》 显示加号减号
    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_center(label);

    lv_obj_t *label2 = lv_label_create(btn2);
    lv_label_set_text(label2, LV_SYMBOL_MINUS);
    lv_obj_center(label2);

    //2.3创建标签 =》 显示当前的温度值
    lv_obj_t *label3 = lv_label_create(obs_table);
    //lv_label_set_text_fmt(label3, "temp:%d",lv_subject_get_int(&temp_subject));不可用于观察者模式
    //代替：绑定观察者模式：
    lv_label_bind_text(label3,&temp_subject,"temp:%d");//%d绑定的是temp_subject的那个参数10
    lv_obj_align(label3, LV_ALIGN_CENTER, 0, 50);


    //2.4创建滑块 =》实现双向绑定(滑块和按钮都能控制温度值改变)
    lv_obj_t *slider = lv_slider_create(obs_table);
    lv_obj_set_size(slider, 200,20);
    lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_slider_set_range(slider, -100, 100);
    lv_slider_set_value(slider, 50, LV_ANIM_OFF);
    lv_slider_bind_value(slider,&temp_subject);//双向绑定

    //滑块和标签都可以绑定观察者模式

    */


    //观察者模式和翻译（中英切换）

    //开启静态翻译功能
    lv_translation_add_static(languages, tags, translations);

    //设置当前的语言为英语
    //在lv_conf.h文件中要先设置LV_FONT_DEFAULT &noto_sans_sc_16才能正确显示中文
    lv_translation_set_language("chinese");

    lv_obj_t *scr = lv_screen_active();
    //1.创建一个标签视图
    lv_obj_t *tabview = lv_tabview_create(scr);
    //1.1设置标签视图的样式
    lv_obj_set_size(tabview, LV_PCT(100), LV_PCT(100));

    //1.2创建一个标签页(创建时把 "obs" 转成 "观察者" 写死进 label，后续直接通过下拉菜单函数修改也改不了)
    lv_obj_t *obs_table = lv_tabview_add_tab(tabview, lv_tr("obs"));//观察者页面
    lv_obj_t *translation = lv_tabview_add_tab(tabview, lv_tr("translation"));//翻译页面

    //添加接收翻译修改的事件回调函数
    //(1):获取table bar
    lv_obj_t *table_bar = lv_tabview_get_tab_bar(tabview);//标签栏
    //(2):循环调用多个按钮
    for(uint8_t i = 0;i < 2;i++)
    {
        lv_obj_t *btn = lv_obj_get_child(table_bar,i);//获取标签栏的第i个按钮
        lv_obj_t *label = lv_obj_get_child(btn,0);//获取按钮上的标签文字
        //添加语言变更的回调
        lv_obj_add_event_cb(label,language_change_event_cb,LV_EVENT_TRANSLATION_LANGUAGE_CHANGED,(void *)tags[i]);//LV_EVENT_TRANSLATION_LANGUAGE_CHANGED:翻译语言变更事件
    }

    //添加对应的组件到obs里面

    //2.1外部参数的观察者模式

    //2.1.1初始化一个主题 =》 选择正确的类型和初始值
    lv_subject_init_int(&temp_subject,10);

    //2.1.2添加外部参数的观察者(回调)
    lv_subject_add_observer(&temp_subject,temp_obs_cb,NULL);//回调触发：当参数值（lv_subject_init_int的第二个值）改变时会触发


    //2.2创建按钮 =》 调整温度

    //加温度值
    lv_obj_t *btn1 = lv_btn_create(obs_table);
    lv_obj_set_size(btn1, 50, 50);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 50, 0);
    lv_obj_add_event_cb(btn1, btn1_event_cb, LV_EVENT_CLICKED, &temp_subject);


    //减温度值
    lv_obj_t *btn2 = lv_btn_create(obs_table);
    lv_obj_set_size(btn2, 50, 50);
    lv_obj_align(btn2, LV_ALIGN_CENTER, -50, 0);
    lv_obj_add_event_cb(btn2, btn2_event_cb, LV_EVENT_CLICKED, &temp_subject);


    //2.2.1创建标签 =》 显示加号减号
    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, LV_SYMBOL_PLUS);
    lv_obj_center(label);

    lv_obj_t *label2 = lv_label_create(btn2);
    lv_label_set_text(label2, LV_SYMBOL_MINUS);
    lv_obj_center(label2);

    //2.3创建标签 =》 显示当前的温度值
    lv_obj_t *label3 = lv_label_create(obs_table);
    //lv_label_set_text_fmt(label3, "temp:%d",lv_subject_get_int(&temp_subject));不可用于观察者模式
    //代替：绑定观察者模式：
    lv_label_bind_text(label3,&temp_subject,"temp:%d");//%d绑定的是temp_subject的那个参数10
    lv_obj_align(label3, LV_ALIGN_CENTER, 0, 50);


    //2.4创建滑块 =》实现双向绑定(滑块和按钮都能控制温度值改变)
    lv_obj_t *slider = lv_slider_create(obs_table);
    lv_obj_set_size(slider, 200,20);
    lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_slider_set_range(slider, -100, 100);
    lv_slider_set_value(slider, 50, LV_ANIM_OFF);
    lv_slider_bind_value(slider,&temp_subject);//双向绑定

    //滑块和标签都可以绑定观察者模式

    //2.5在观察者页面顶部空白区域显示PA0~PA3的ADC原始值(0~4095)
    //每个通道一个主题，label绑定主题后，主题值变化时自动刷新文本
    for (uint8_t i = 0; i < 4; i++)
    {
        lv_subject_init_int(&adc_subject[i], 0);

        lv_obj_t *adc_label = lv_label_create(obs_table);
        lv_label_bind_text(adc_label, &adc_subject[i], adc_fmt[i]);//格式串里的%d绑定主题的int值
        lv_obj_align(adc_label, LV_ALIGN_TOP_LEFT, 10, 8 + i * 20);//顶部空白区域纵向排列
    }

    //2.6创建定时器 =》周期性把DMA缓存中的ADC值写入主题，触发观察者刷新label
    lv_timer_create(adc_display_timer_cb, 200, NULL);//200ms刷新一次

    //在翻译页面中添加对应的下拉菜单  切换语言
    lv_obj_t *dropdown = lv_dropdown_create(translation);
    lv_dropdown_set_options(dropdown, "chinese\n"
                                      "en");

    //添加回调函数
    lv_obj_add_event_cb(dropdown, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, dropdown);

}
