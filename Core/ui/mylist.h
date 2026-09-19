#ifndef MYLIST_H
#define MYLIST_H

#include "lvgl.h"
#include "stdio.h"

typedef struct {
    lv_obj_t * btn;//按钮
    uint32_t count;//值
} mylist_t;

/**
 * @brief 创建一个mylist
 *
 * @param parent
 * @return lv_obj_t*
 */
mylist_t * lv_mylist_create(lv_obj_t * parent);

/**
 * @brief 删除mylist
 * @param mylist
 */
void lv_mylist_delete(mylist_t * mylist);

/**
 * @brief 设置大小
 * @param mylist
 * @param width
 * @param height
 */
void lv_mylist_set_size(mylist_t * mylist, int32_t width, int32_t height);

/**
 * @brief 设置位置
 * @param mylist
 * @param x
 * @param y
 */
void lv_mylist_set_pos(mylist_t * mylist, int32_t x, int32_t y);

#endif /* MYLIST_H */
