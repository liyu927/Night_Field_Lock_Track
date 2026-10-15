#include "mylist.h"
#include "com_Debug.h"

static void mylist_callback(lv_event_t * e)
{
    //获取由lv_obj_add_event_cb函数传进来的user_data（mylist本身的指针）
    mylist_t * mylist = lv_event_get_user_data(e);

    //获取触发事件的对象(哪个对象触发了回调函数)（例如是按钮btn1）
    lv_obj_t * btn1 = lv_event_get_target(e);

    //累加（每次点击它都会进入这个函数然后+1）
    mylist->count++;

    printf("mylist_callback: count=%d\n", mylist->count);

}



/**
 * @brief 创建一个mylist=>类型一个按钮=>实现累加功能
 *
 * @param parent
 * @return lv_obj_t*
 */
mylist_t * lv_mylist_create(lv_obj_t * parent)
{
    //1.创建一个mylist_t结构体
    mylist_t * mylist = lv_malloc_zeroed(sizeof(mylist_t));

    //创建一个底层的结构体 =》屏幕
    lv_obj_t * obj = lv_obj_create(parent);

    //2.根据按钮需要的样式修改
    //2.1删除边框
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    //2.2修改颜色
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);//设置背景颜色
    //2.3设置默认大小和位置（默认值，后续可以通过下面的函数来更改）
    lv_obj_set_size(obj, 40, 20);
    lv_obj_set_pos(obj, 0, 0);

    //添加阴影
    lv_obj_set_style_shadow_color(obj, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(obj, 5, LV_PART_MAIN);
    lv_obj_set_style_shadow_offset_x(obj, 2, LV_PART_MAIN);
    lv_obj_set_style_shadow_offset_y(obj, 2, LV_PART_MAIN);

    //添加标志：可点击（屏幕变按键）
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    //生成事件
    lv_obj_add_event_cb(obj, mylist_callback, LV_EVENT_CLICKED, mylist);//点击事件回调函数，传入的参数是mylist本身的指针

    mylist->btn = obj;
    mylist->count = 0;

    return mylist;

}

/**
 * @brief 删除mylist
 * @param mylist
 */
void lv_mylist_delete(mylist_t * mylist)
{

    //删除按钮对象
    lv_obj_delete_async(mylist->btn);

    //释放mylist结构体内存
    lv_free(mylist);

}

/**
 * @brief 设置大小
 * @param mylist
 */
void lv_mylist_set_size(mylist_t * mylist, int32_t width, int32_t height)
{
    lv_obj_set_size(mylist->btn, width, height);
}


/**
 * @brief 设置位置
 * @param mylist
 * @param x
 * @param y
 */
void lv_mylist_set_pos(mylist_t * mylist, int32_t x, int32_t y)
{
    lv_obj_set_pos(mylist->btn, x, y);
}
