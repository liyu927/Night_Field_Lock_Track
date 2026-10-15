#include "ui.h"
#include "com_Debug.h"

//按钮回调函数
static void btn_callback(lv_event_t * e)
{
    //获取由lv_obj_add_event_cb函数传进来的user_data（"hello"）
    char * str = lv_event_get_user_data(e);

    //获取触发事件的对象(哪个对象触发了回调函数)（例如是按钮btn1）
    lv_obj_t * btn1 = lv_event_get_target(e);

    printf("btn_callback: %s\n", str);

    //获取父类
    lv_obj_t * parent = lv_obj_get_parent(btn1);
    //从父类中获取一个滑块孩子
    lv_obj_t * slider = lv_obj_get_child(parent, 1);//第二个参数意义是获取父类的第几个孩子，0是第一个孩子，1是第二个孩子，2是第三个孩子
    //修改滑块的值（不是修改0-100范围）
    lv_slider_set_value(slider,lv_slider_get_value(slider)+1,LV_ANIM_ON);//获取滑块的当前值+1作为新的滑块的值，LV_ANIM_ON表示动画效果
    //手动发送事件函数（把更改进行实际部署）
    lv_obj_send_event(slider, LV_EVENT_VALUE_CHANGED, NULL);//手动发送滑块的值发生变化事件，触发滑块的回调函数，LV_EVENT_VALUE_CHANGED是值发生变化的事件,必须对应lv_obj_add_event_cb的参数
    //由于lv_obj_send_event手动发生的事滑块的LV_EVENT_VALUE_CHANGED事件，所以必须是LV_EVENT_VALUE_CHANGED的滑块lv_obj_add_event_cb对应的回调函数才会被执行
}

//滑块的回调函数
static void slider_callback(lv_event_t * e)
{
    //获取触发事件的对象(哪个对象触发了回调函数)（例如是滑块slider）
    lv_obj_t * slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);//默认范围0-100
    printf("slider_callback: %d\n", value);
}


static void scroll_callback(lv_event_t * e)
{
    //精准获取当前一次滚动=>滚动的方向和距离
    lv_obj_t * scroll = lv_event_get_target(e);

    //保存上次滚动位置
    static int32_t last_x = 0;
    static int32_t last_y = 0;

    //获取当前滚动位置
    uint32_t current_x = lv_obj_get_scroll_x(scroll);
    uint32_t current_y = lv_obj_get_scroll_y(scroll);

    int32_t diff_x = current_x - last_x;
    int32_t diff_y = current_y - last_y;

    if (diff_x > 0)
    {
        printf("scroll right: %d\n", diff_x);
    }
    else if(diff_x < 0)
    {
        printf("scroll left: %d\n", diff_x);
    }

    if (diff_y > 0)
    {
        printf("scroll down: %d\n", diff_y);
    }
    else if(diff_y < 0)
    {
        printf("scroll up: %d\n", -diff_y);
    }

    //更新上次滚动位置
    last_x = current_x;
    last_y = current_y;


}





void ui_init(void)
{
    /*一、初始属性介绍
    //自定义页面设备
    //1.先获取当前硬件的整个屏幕(是获取已有的屏幕保存在一个变量中。不是生成一个屏幕)
    lv_obj_t * scr = lv_screen_active();//指针类型函数，可跳转

    //2.在屏幕上创建一个子屏幕，并保存在obj变量中
    lv_obj_t *obj = lv_obj_create(scr);//在当前屏幕上创建一个子屏幕（默认大小100x100）

    //3.创建一个按钮
    lv_obj_t *btn = lv_btn_create(obj);//在子屏幕内创建一个按钮（组件都有默认值）

    //3.1设置按钮的位置和大小  x,y坐标,x向右为正，y向下为正
    lv_obj_set_pos(btn, 30,30);//按键左上角那个点位置
    lv_obj_set_size(btn, 40, 40);//大小

    //4.创建一个滑块
    //类似lv_obj_t *slider不推荐将组件设置为全局变量
    lv_obj_t *slider = lv_slider_create(obj);//在子屏幕内创建一个滑块

    //4.1设置滑块的位置和大小(通用属性)
    lv_obj_set_pos(slider, 10, 80);
    lv_obj_set_size(slider, 60, 10);
    60>10 所以滑块的宽度大于高度，所以滑块是横着的，参数调换：10<60，滑块是竖着的

    //4.2设置滑块的独有属性
    lv_slider_set_range(slider, 0, 5);//设置滑块的范围5
    lv_slider_set_value(slider, 4, LV_ANIM_ON);//设置滑块的初始值,LV_ANIM_ON表示动画效果(刚加载成功瞬间会有动画效果动到4)

    //5.延时删除滑块
    lv_obj_delete_delayed(slider,3000);
    */

    /*二、父级与子级图层关系
    //1.获取当前屏幕(父级 根)
    lv_obj_t * scr = lv_screen_active();

    //2.创建一个子屏幕(第一个孩子) 下标为0
    lv_obj_t * obj = lv_obj_create(scr);
    lv_obj_set_pos(obj, 20, 20);

    //3.创建第二个孩子 下标为1
    lv_obj_t * obj2 = lv_obj_create(scr);
    lv_obj_set_pos(obj2, 30, 30);

    //4.创建第三个孩子 下标为2
    lv_obj_t * obj3 = lv_obj_create(scr);
    lv_obj_set_pos(obj3, 40, 40);

    //显示默认父级在下面，子级在上面
    //尽量不要给组件set_name(),很占用内存，可以用下标的方式查找

    //5.如何获取3个子级的中间屏幕
    lv_obj_t * child = lv_obj_get_child(scr, 1);//获取父级scr的下标为1的孩子（由于是get_child，此时长子obj下标为0）
    //lv_obj_get_parent(child);//获取父级scr,很少用，一般都是父亲找孩子
    lv_obj_set_size(child, 50, 50);
    //这个适合在其他文件忘记第二个孩子叫obj2的情况下，直接获取下标为1的孩子，进行操作

    //只移动obj3到最下层的位置
    lv_obj_move_to_index(obj3, 0);//将obj3移动到下标为0的位置，obj3在最下面，其他正常排序不变->obj在中间，obj2在最上面
    lv_obj_swap(obj, obj3);//交换obj和obj2的位置，obj在最上面，obj2在中间，obj3在最下面

    */

    /*三、组件特性 位置和大小
    //1.获取当前屏幕(父级 根)
    lv_obj_t * scr = lv_screen_active();

    //2.创建一个子屏幕
    lv_obj_t * obj = lv_obj_create(scr);
    lv_obj_set_pos(obj, 30, 30);//设置子屏幕的位置

    //2.1修改子屏幕的边框大小
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);//设置边框宽度

    //2.2修改屏幕的圆角大小
    lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);//设置圆角大小

    //2.3修改屏幕内边距（设置子屏幕与按键之间的距离）
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);//设置内边距大小，LV_PART_MAIN是屏幕，LV_PART_SCROLLABLE_PART是滚动条等
    //不是所有的东西都有第三个参数里面全部的属性


    //3.创建一个按钮
    lv_obj_t * btn = lv_btn_create(obj);
    //lv_obj_set_pos(btn, 10, 20);//子元素的位置是相对于父元素的 不是绝对的坐标（上方可以设置内边距调整(0,0)的坐标位置）
    //当按键屏幕大小放不下按钮的位置时，例如屏幕大小为100，按键位置为120，实际运行后往下拉屏幕就能找到

    //lv_obj_set_x(btn, 10);//只设置x坐标
    //lv_obj_set_y(btn, 10);//只设置y坐标
    //坐标可以是负数（但是此时不能拖拉屏幕展示全部）

    //相对位置
    //lv_obj_align(btn,  LV_ALIGN_CENTER, 0, 0);//按钮居中对齐，x,y偏移量为0
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 10);//按钮父级的中间的最上边，然后y轴往下移10元素
    //lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);//按钮居中对齐，x,y偏移量为0

    //lv_obj_set_size(btn, 40, 40);
    // lv_obj_set_size也可以设置百分比大小（相对于上一父级）
    lv_obj_set_size(btn, LV_PCT(50),LV_PCT(50));//LV_PCT(50)表示50%的父级大小，不常用
    //常用点在：不同大小的屏幕（例如不同尺寸的手机看抖音），看到的都是与手机尺寸大小适配的视频页面
    */

    /*四、组件特性 部件和特性
    //1.获取当前屏幕(父级 根)
    lv_obj_t * scr = lv_screen_active();

    //2.创建一个子屏幕
    lv_obj_t * obj = lv_obj_create(scr);

    //2.1修改不同屏幕部件的颜色   背景绿色 滚动条红色
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);//设置背景颜色
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), LV_PART_SCROLLBAR);//设置滚动条颜色

    //2.2创建一个超出范围的按键，使得滚动条出现
    lv_obj_t * btn = lv_btn_create(obj);
    lv_obj_set_pos(btn, 200, 200);

    //3.创建一个滑块
    //在最底层的屏幕上创建一个滑块
    lv_obj_t * slider = lv_slider_create(scr);
    lv_obj_set_pos(slider, 200, 200);
    lv_obj_set_size(slider, 100, 10);

    //3.1修改一个部件不同颜色
    lv_obj_set_style_bg_color(slider, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);//设置滑块未占据颜色
    lv_obj_set_style_bg_color(slider, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR);//设置滑块占据颜色
    lv_obj_set_style_bg_color(slider, lv_palette_main(LV_PALETTE_BLUE), LV_PART_KNOB);//设置滑块按钮颜色

    //3.2同一个部件不同状态下颜色也可以设置不同（例如按键按下和松开时颜色不一样）
    lv_obj_t * btn2 = lv_btn_create(scr);
    lv_obj_set_pos(btn2, 60, 60);

    lv_obj_set_style_bg_color(btn2, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);//设置默认状态颜色 LV_STATE_DEFAULT和LV_PART_MAIN在这里是一样的
    lv_obj_set_style_bg_color(btn2, lv_palette_main(LV_PALETTE_RED), LV_STATE_PRESSED);//设置按下状态颜色
    //也可以合成下面一句：
    //lv_obj_set_style_bg_color(btn2, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN | LV_STATE_PRESSED);
    //但是前提是两者颜色相同都是蓝色

    */

    /*五、组件特性 动画

    //1.获取当前屏幕(父级 根)
    lv_obj_t * scr = lv_screen_active();
    //2.创建一个按钮
    lv_obj_t * btn = lv_btn_create(scr);

    //修改两种状态下按钮的颜色
    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);//设置默认状态颜色
    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_STATE_PRESSED);//设置按下状态颜色


    //begin官方网站复制过来
    static const lv_style_prop_t trans_props[] = {
                                                LV_STYLE_BG_OPA,//背景不透明度
                                                LV_STYLE_BG_COLOR,//背景颜色
                                                0,
    };

    static lv_style_transition_dsc_t trans1;//声明一个静态的lv_style_transition_dsc_t类型的变量trans1

    //初始化,(1)存储过渡效果的结构体，（2）存过渡效果的数组，（3）实现过渡的函数，（4）过渡效果的持续时间，（5）过渡效果的延时时间 ,（6）过渡函数的参数（可选）
    lv_style_transition_dsc_init(&trans1, trans_props, lv_anim_path_ease_out, 5000, 1000,NULL);

    static lv_style_t style1;//声明一个静态的lv_style_t类型的变量style1
    lv_style_init(&style1);//初始化style1

    //添加过渡效果给到样式
    lv_style_set_transition(&style1, &trans1);

    //将样式添加到按钮中 =》 从其他状态进入到默认状态时，按钮的背景颜色和不透明度会有过渡效果
    lv_obj_add_style(btn, &style1, LV_PART_MAIN);

    //end官方网站复制过来
    */

    /*六、组件特性 事件 (按钮触发)
    //1.获取当前屏幕(父级 根)
    lv_obj_t * scr = lv_screen_active();
    //2.创建一个按钮
    lv_obj_t * btn1 = lv_btn_create(scr);
    lv_obj_set_pos(btn1, 100, 100);
    lv_obj_set_size(btn1, 40, 40);

    //添加点击事件
    lv_obj_add_event_cb(btn1,
        btn_callback,//函数事件名，在本文件上方定义一个事件函数
        LV_EVENT_CLICKED,
        "hello");//传入的参数，传入的参数可以是任意类型的指针，传入的参数可以在回调函数中获取，也可以是NULL

    //3.创建一个滑块
    lv_obj_t * slider = lv_slider_create(scr);
    lv_obj_set_pos(slider, 50, 200);
    lv_obj_set_size(slider, 100, 10);

    //添加滑块事件（值发生变化的回调）
    lv_obj_add_event_cb(slider, slider_callback, LV_EVENT_VALUE_CHANGED, slider);//传入的参数是滑块本身的指针，LV_EVENT_VALUE_CHANGED连续更新最新值
    //lv_obj_add_event_cb(slider, slider_callback, LV_EVENT_CLICKED, slider);//传入的参数是滑块本身的指针，LV_EVENT_RELEASED或LV_EVENT_CLICKED只在松开时触发一次

    //创建mylist
    mylist_t * mylist = lv_mylist_create(scr);
    mylist_t * mylist2 = lv_mylist_create(scr);
    lv_mylist_set_pos(mylist, 100, 0);
    */

    /*七、展示弹性布局

    //1.设置屏幕为弹性布局的容器
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_layout(scr, LV_LAYOUT_FLEX);

    //2.弹性布局
    //lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_ROW);//设置为横向排列，不换行(官方网站有详细解释)
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_ROW_WRAP);//设置为横向排列，换行(官方网站有详细解释)

    //3.对齐
    lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);//设置为左对齐或上对齐
    //第二个参数是主轴对齐方式（横向排列时主轴是X轴，此时START是左对齐，END是右对齐，CENTER是居中对齐 反之）
    //第三个参数是交叉轴(与主轴垂直)对齐方式，逻辑同上，横向排列时，START是上对齐，END是下对齐，CENTER是居中对齐
    //第四个参数是交叉轴的轨道对齐方式，START代表是顶部对齐，END是底部对齐，CENTER是居中对齐，是多行多列时这些内容时在顶部还是底部

    //4.修改边距（一般不建议改）
    lv_obj_set_style_pad_all(scr, 15, LV_PART_MAIN);//设置所有边距（屏幕边距）
    lv_obj_set_style_pad_row(scr, 5, LV_PART_MAIN);//设置行间距（组件之间边距）
    lv_obj_set_style_pad_column(scr, 5, LV_PART_MAIN);//设置列间距（组件之间边距）

    //5.创建按钮进行布局
    // for(uint8_t i = 0; i < 20; i++)
    // {
    //     // 创建按钮并添加到弹性布局容器中
    //     lv_obj_t *btn = lv_btn_create(scr);//弹性按钮不需要设置位置，它会自己找适当位置布局
    //     lv_obj_set_size(btn, (i+1)*2, (i+1)*2);//设置按钮大小，随着i的增加，按钮越来越大
    // }

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_flex_grow(btn, 1);//设置为1，1为加权数
    lv_obj_t *btn1 = lv_btn_create(scr);
    lv_obj_set_flex_grow(btn1, 2);//设置为2，2为加权数（占比权重）
    //上述代码中，btn1的加权数是btn的两倍，所以btn1的大小是btn的两倍，且btn在btn1的前面
    lv_obj_t *btn2 = lv_btn_create(scr);
    lv_obj_set_flex_grow(btn2, 3);//设置为3，3为加权数（占比权重）

    //强制将btn2添加到新的轨道
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);//强制将btn2添加到新的轨道，btn2在btn1的下面
    //后面再添加新组件都会到新轨道内

    */

    /*八、展示网格布局

    //1.网格布局
    lv_obj_t *scr = lv_screen_active();
    //创建一个屏幕
    lv_obj_t *obj = lv_obj_create(scr);
    //设置屏幕为网格布局的容器
    lv_obj_set_layout(obj, LV_LAYOUT_GRID);
    //修改大小
    lv_obj_set_size(obj, 300, 400);
    //去除边距
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);//设置所有边距（屏幕边距）

    //2.设置网格行列数
    //static int32_t column_dsc[] = {LV_GRID_CONTENT,LV_GRID_CONTENT , LV_GRID_TEMPLATE_LAST};//两个参数代表两列（数字大小代表下一列间距）
    //static int32_t row_dsc[] = {LV_GRID_CONTENT,LV_GRID_CONTENT ,LV_GRID_CONTENT , LV_GRID_TEMPLATE_LAST};//三个参数代表三行（数字大小代表与下一行间距）
    static int32_t column_dsc[] = {LV_GRID_FR(1),LV_GRID_FR(2) , LV_GRID_TEMPLATE_LAST};//按比例划分1:4，（下面的lv_obj_set_grid_cell的LV_GRID_ALIGN_START代表从左边开始划分）
    static int32_t row_dsc[] = {LV_GRID_FR(1),LV_GRID_FR(2) ,LV_GRID_FR(3) , LV_GRID_TEMPLATE_LAST};


    //设置网格是两列三行的网格布局
    lv_obj_set_grid_dsc_array(obj, column_dsc, row_dsc);

    //两列三行一共六个格子，一个组件可以占据多个格子，这里统一只占一个
    // for(uint8_t i = 0; i < 6; i++)
    // {
    //     lv_obj_t *btn = lv_btn_create(obj);

    //     //（1）子组件 （2）列对齐->左右对齐 （3） 哪一列 （4）占几列 （5）行对齐->上下对齐 （6）哪一行 （7）占几行
    //     lv_obj_set_grid_cell(btn, LV_GRID_ALIGN_START, i % 2, 1, LV_GRID_ALIGN_START, i / 2, 1);//设置按钮在网格中的位置和占据的格子数
    // }



    for(uint8_t i = 0; i < 2; i++)
    {
        lv_obj_t *btn = lv_btn_create(obj);

        //（1）子组件 （2）列对齐->左右对齐 （3） 哪一列 （4）占几列 （5）行对齐->上下对齐 （6）哪一行 （7）占几行
        lv_obj_set_grid_cell(btn, LV_GRID_ALIGN_START, i % 2, 1, LV_GRID_ALIGN_START, i / 2, 1);//设置按钮在网格中的位置和占据的格子数
    }

    //创建一个屏幕（代码编写占剩下四个格子的网格）（上面两个按键的代码已经占了两个格子）（一共六个格子）
    lv_obj_t *obj1 = lv_obj_create(obj);
    lv_obj_set_grid_cell(obj1, LV_GRID_ALIGN_CENTER, 0, 2, LV_GRID_ALIGN_CENTER, 1, 2);//设置按钮在网格中的位置和占据的格子数

    //将子网格设置为网格布局
    lv_obj_set_layout(obj1, LV_LAYOUT_GRID);

    //设置子网格的行列数为NULL，表示子网格的行列数由父网格的行列数决定
    lv_obj_set_grid_dsc_array(obj1, NULL, NULL);//子网格的行列数由子网格的子组件决定

    //修改行列边距
    lv_obj_set_style_pad_all(obj1, 0, LV_PART_MAIN);//设置所有边距（屏幕边距）
    lv_obj_set_style_pad_column(obj1, 0, LV_PART_MAIN);//设置列边距
    lv_obj_set_style_pad_row(obj1, 0, LV_PART_MAIN);//设置行边距

    for(uint8_t i = 0; i < 4; i++)
    {
        lv_obj_t *btn = lv_btn_create(obj1);

        //（1）子组件 （2）列对齐->左右对齐 （3） 哪一列 （4）占几列 （5）行对齐->上下对齐 （6）哪一行 （7）占几行
        lv_obj_set_grid_cell(btn, LV_GRID_ALIGN_START, i % 2, 1, LV_GRID_ALIGN_START, i / 2, 1);//设置按钮在网格中的位置和占据的格子数
        //子网格继承父网格的行列数和比例，所以子网格的按钮也会按比例划分
        //LV_GRID_ALIGN_STRETCH表示组件铺满整个网格
    }
    */

    /*九、滚动基础

    //展示布局 网格布局
    lv_obj_t *scr = lv_screen_active();
    //创建一个屏幕
    lv_obj_t *obj = lv_obj_create(scr);
    lv_obj_set_size(obj, 200, 200);

    //关闭父屏幕的滚动动量
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLL_MOMENTUM);//关闭父屏幕的滚动动量，LV_OBJ_FLAG_SCROLL_MOMENTUM是滚动动量的标志位
    //滚动动量：当用户快速滑动并松开手指后，内容会继续惯性滑动一段距离，然后慢慢减速停止，类似智能手机的滚动效果。

    //重新开启父屏幕的滚动动量
    //lv_obj_add_flag(obj, LV_OBJ_FLAG_SCROLL_MOMENTUM);//重新开启（默认是开启的，但关闭需要手动）

    //关闭父屏幕的弹性滚动（弹性滚动是滚动条到头之后继续拉动下去会反弹回来，有一个反弹效果）
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLL_ELASTIC);//关闭父屏幕的弹性滚动，LV_OBJ_FLAG_SCROLL_ELASTIC是弹性滚动的标志位

    //重新开启父屏幕的弹性滚动
    //lv_obj_add_flag(obj, LV_OBJ_FLAG_SCROLL_ELASTIC);//重新开启（默认是开启的，但关闭需要手动）

    //创建一个子屏幕(触发链式滚动，父子屏幕都可以滚动)
    lv_obj_t *obj1 = lv_obj_create(obj);
    lv_obj_set_pos(obj1, 200, 200);

    // //创建按钮(层层嵌套会导致刚运行时就会滚动一下，影响后面lv_obj_scroll_to_x的判断，要先注释掉这个再测试lv_obj_scroll_to_x)
    //lv_obj_scroll_to_x(obj, 0, LV_ANIM_ON);也可以强制不让进度条一开始就自动滚动
    lv_obj_t *btn = lv_btn_create(obj1);
    lv_obj_set_pos(btn, 200, 200);//触发滚动屏幕

    //不显示滚动条
    //lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);//不显示滚动条(只是不显示，但是可以滚动，仍然有对应的功能)
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_ON);
    //LV_SCROLLBAR_MODE_AUTO（默认，超出屏幕时才显示滚动条），LV_SCROLLBAR_MODE_ACTIVE（只有滚动的时候才显示滚动条）
    //LV_SCROLLBAR_MODE_ON（一直显示滚动条）， LV_SCROLLBAR_MODE_OFF（不显示滚动条）

    //修改滚动条样式
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), LV_PART_SCROLLBAR);//设置水平滚动条背景颜色(LV_PART_SCROLLBAR是滚动条的部件，LV_PART_MAIN是背景的部件)
    //水平滚动条函数：lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), LV_PART_SCROLLBAR | LV_STATE_HORIZONTAL);//设置水平滚动条背景颜色
    //垂直滚动条函数：lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), LV_PART_SCROLLBAR | LV_STATE_VERTICAL);//设置垂直滚动条背景颜色

    //添加滚动结束的回调函数
    lv_obj_add_event_cb(obj, scroll_callback, LV_EVENT_SCROLL_END, NULL);//传入的参数是滑块本身的指针，LV_EVENT_SCROLL_END只在滚动结束时触发一次

    //手动调用一些滚动函数
    lv_obj_scroll_to_x(obj, 100, LV_ANIM_ON);//滚动到x轴100的位置
    lv_obj_scroll_to_y(obj, 100, LV_ANIM_ON);//滚动到y轴100的位置

    */

    /*十、滚动吸附（重要）
    //展示布局 网格布局
    lv_obj_t *scr = lv_screen_active();
    //创建一个屏幕
    lv_obj_t *obj = lv_obj_create(scr);
    lv_obj_set_size(obj, 200, 200);
    lv_obj_set_pos(obj, 60, 100);

    //设置为弹性布局
    lv_obj_set_layout(obj, LV_LAYOUT_FLEX);
    //水平布局（不换行）
    lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_ROW);

    for(uint8_t i = 0; i < 10; i++)
    {
        lv_obj_t *btn = lv_btn_create(obj);
        lv_obj_set_size(btn, 160, 160);

        //添加吸附的标签(让按钮带吸附功能)，，标记某个子对象为“可吸附”的锚点。
        lv_obj_add_flag(btn,LV_OBJ_FLAG_SNAPPABLE);

        //设置x轴上每次滚动完之后都保持按钮在中间，，设置容器的x轴吸附模式为“居中吸附”。
        lv_obj_set_scroll_snap_x(obj,LV_SCROLL_SNAP_CENTER);

        //添加只滚动一次的功能
        lv_obj_add_flag(obj,LV_OBJ_FLAG_SCROLL_ONE);
    }
    */


}
