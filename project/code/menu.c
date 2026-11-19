/* ======================================================================
 * >> OLED 菜单框架
 * 【面试问答】
 * Q1: 菜单框架的数据结构？
 * A: 树状多级菜单：option 节点含父指针、最多 8 个子节点指针、光标 cursor 和滚动首行
 * A: top；叶子要么绑定一个 Parameter，要么绑定一个 action 回调，新增菜单只需 Init +
 * A: AddChild 两行。
 * ----------------------------------------------------------------------
 * Q1: 多类型参数如何统一管理？
 * A: Parameter 用 union 分别保存 float/int16/uint16/uint8 的目标指针、步长和上下限，
 * A: 显示与加减按 type 分支，同一套代码支持所有参数。
 * ----------------------------------------------------------------------
 * Q1: 滚动和刷新怎么实现？
 * A: 每页 6 项，cursor 循环移动、top 跟踪首行；普通移动只擦画光标两行，翻页才整屏
 * A: 重绘，减少 SPI 刷屏量和闪烁。
 * ----------------------------------------------------------------------
 * Q1: 按键如何检测？为何菜单阶段安全？
 * A: 10ms 定时中断扫描消抖并区分短按，菜单读取后清除状态；调参阶段电机尚未上电，
 * A: 保证人员安全，退出菜单写 Flash 后才启动控制中断。
 * ----------------------------------------------------------------------
 * Q1: 参数编辑如何防止越界？
 * A: 每次加减后与 min/max 比较钳位，小数显示精度由 decimals 控制。
 * ----------------------------------------------------------------------
 * ====================================================================== */
#include "menu.h"
extern uint8  num_box2;
extern uint8  num_box3;
extern float ACCEL_LIMIT_TRIGGER_DMPS;
// 内部函数声明
static MenuKey Menu_ReadKey(void);
static void Menu_EditParameter(option *opt);
static void Menu_ShowParameterValue(Parameter *param,uint16 x,uint16 y);
static void Menu_ParamIncrease(Parameter *param);
static void Menu_ParamDecrease(Parameter *param);
static void Menu_MoveUp(option *menu);
static void Menu_MoveDown(option *menu);
static void Menu_UpdateCursor(option *menu,uint8 old_cursor,uint8 old_top);


static Parameter p_num_box2;
static Parameter p_num_box3;
static Parameter p_Limited;
static Parameter p_Xp;
static Parameter p_Xd;
static Parameter p_Yp;
static Parameter p_Yd;
// 菜单对象

static option menu_main;
static option opt_num_box2;
static option opt_num_box3;

static option Xk;
static option Yk;

static option opt_Limited;

static option X_Kp;
static option X_Kd;

static option Y_Kp;
static option Y_Kd;

void App_MenuInit(void)
{
    // 参数初始化   
		Menu_ParamUint8Init(&p_num_box2,&num_box2,1,1,255);
		Menu_ParamUint8Init(&p_num_box3,&num_box3,1,1,255);
		Menu_ParamFloatInit(&p_Xp,&X.kp,0.01,0.0,100.f,2);
		Menu_ParamFloatInit(&p_Xd,&X.kd,0.1,0.0,100.f,2);
		Menu_ParamFloatInit(&p_Yp,&Y.kp,0.01,0.0,100.f,2);
		Menu_ParamFloatInit(&p_Yd,&Y.kd,0.1,0.0,100.f,2);
		Menu_ParamFloatInit(&p_Limited,&ACCEL_LIMIT_TRIGGER_DMPS,0.5,0.0,100.f,1);
	
    // 菜单初始化
		Menu_OptionInit(&menu_main,"Main");
		Menu_OptionInit(&Xk,"Xk");
		Menu_OptionInit(&Yk,"Yk");

		Menu_ParamOptionInit(&opt_num_box2,"Num of Box2", &p_num_box2);
		Menu_ParamOptionInit(&opt_num_box3,"Num of Box3", &p_num_box3);
		Menu_ParamOptionInit(&opt_Limited,"Limited", &p_Limited);
		Menu_ParamOptionInit(&X_Kp,"Xp",&p_Xp);
		Menu_ParamOptionInit(&X_Kd,"Xd", &p_Xd);
		Menu_ParamOptionInit(&Y_Kp,"Yp", &p_Yp);
		Menu_ParamOptionInit(&Y_Kd,"Yd", &p_Yd);
    // Main
		Menu_AddChild(&menu_main,&opt_num_box2);
		Menu_AddChild(&menu_main,&opt_num_box3);
		Menu_AddChild(&menu_main,&Xk);
		Menu_AddChild(&menu_main,&Yk);
		Menu_AddChild(&menu_main,&opt_Limited);
		Menu_AddChild(&Xk,&X_Kp);
		Menu_AddChild(&Xk,&X_Kd);
		Menu_AddChild(&Yk,&Y_Kp);
		Menu_AddChild(&Yk,&Y_Kd);
}

// 普通菜单项初始化

void Menu_OptionInit(
    option *opt,
    char *name
)
{
    if(opt == NULL)
        return;

    opt->name = name;

    opt->parent = NULL;

    opt->child_count = 0;

    opt->cursor = 0;
    opt->top = 0;

    opt->param = NULL;

    opt->action = NULL;

    for(uint8 i = 0;
        i < MENU_MAX_CHILDREN;
        i++)
    {
        opt->children[i] = NULL;
    }
}

// 参数菜单项初始化

void Menu_ParamOptionInit(
    option *opt,
    char *name,
    Parameter *param
)
{
    Menu_OptionInit(
        opt,
        name
    );

    opt->param = param;
}

// 功能菜单项初始化

void Menu_ActionOptionInit(
    option *opt,
    char *name,
    void (*action)(void)
)
{
    Menu_OptionInit(
        opt,
        name
    );

    opt->action = action;
}

// float参数初始化

void Menu_ParamFloatInit(
    Parameter *param,
    float *target,
    float step,
    float min,
    float max,
    uint8 decimals
)
{
    if(param == NULL)
        return;

    param->type = PARAM_FLOAT;

    param->target.float_ptr = target;

    param->step.float_val = step;

    param->min.float_val = min;

    param->max.float_val = max;

    param->decimals = decimals;
}

// int16参数初始化

void Menu_ParamInt16Init(
    Parameter *param,
    int16 *target,
    int16 step,
    int16 min,
    int16 max
)
{
    if(param == NULL)
        return;

    param->type = PARAM_INT16;

    param->target.int16_ptr = target;

    param->step.int16_val = step;

    param->min.int16_val = min;

    param->max.int16_val = max;

    param->decimals = 0;
}

// uint16参数初始化

void Menu_ParamUint16Init(
    Parameter *param,
    uint16 *target,
    uint16 step,
    uint16 min,
    uint16 max
)
{
    if(param == NULL)
        return;

    param->type = PARAM_UINT16;

    param->target.uint16_ptr = target;

    param->step.uint16_val = step;

    param->min.uint16_val = min;

    param->max.uint16_val = max;

    param->decimals = 0;
}

// uint16参数初始化

void Menu_ParamUint8Init(
    Parameter *param,
    uint8 *target,
    uint8 step,
    uint8 min,
    uint8 max
)
{
    if(param == NULL)
        return;

    param->type = PARAM_UINT8;

    param->target.uint8_ptr = target;

    param->step.uint8_val = step;

    param->min.uint8_val = min;

    param->max.uint8_val = max;

    param->decimals = 0;
}

// 添加子菜单

uint8 Menu_AddChild(
    option *parent,
    option *child
)
{
    if(parent == NULL ||
       child == NULL)
    {
        return 0;
    }

    if(parent->child_count >=
       MENU_MAX_CHILDREN)
    {
        return 0;
    }

    parent->children[
        parent->child_count
    ] = child;

    parent->child_count++;

    child->parent = parent;

    return 1;
}

// 读取按键

static MenuKey Menu_ReadKey(void)
{
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);
        return MENU_KEY_UP;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        return MENU_KEY_DOWN;
    }

    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_3);
        return MENU_KEY_ENTER;
    }

    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_4);
        return MENU_KEY_BACK;
    }

    return MENU_KEY_NONE;
}

// 显示参数

static void Menu_ShowParameterValue(
    Parameter *param,
    uint16 x,
    uint16 y
)
{
    if(param == NULL)
        return;

    // 清除旧数字
    ips200_show_string(
        x,
        y,
        "          "
    );

    switch(param->type)
    {
        case PARAM_FLOAT:

            ips200_show_float(
                x,
                y,
                *param->target.float_ptr,
                6,
                param->decimals
            );

            break;


        case PARAM_INT16:

            ips200_show_int(
                x,
                y,
                *param->target.int16_ptr,
                6
            );

            break;


        case PARAM_UINT16:

            ips200_show_uint(
                x,
                y,
                *param->target.uint16_ptr,
                6
            );

            break;
				
				case PARAM_UINT8:

            ips200_show_uint(
                x,
                y,
                *param->target.uint8_ptr,
                6
            );

            break;


        default:

            break;
    }
}

// 显示菜单

void Menu_Display(option *menu)
{
    if(menu == NULL)
        return;

    ips200_clear();


    // ------------------------------
    // 标题
    // ------------------------------

    ips200_show_string(
        0,
        0,
        menu->name
    );


    // ------------------------------
    // 菜单项
    // ------------------------------

    uint8 display_count =
        menu->child_count - menu->top;

    if(display_count >
       MENU_VISIBLE_ITEMS)
    {
        display_count =
            MENU_VISIBLE_ITEMS;
    }


    for(uint8 row = 0;
        row < display_count;
        row++)
    {
        uint8 index =
            menu->top + row;

        option *child =
            menu->children[index];

        uint16 y =
            (row + 1) * 16;


        // 光标
        if(index == menu->cursor)
        {
            ips200_show_string(
                0,
                y,
                ">"
            );
        }
        else
        {
            ips200_show_string(
                0,
                y,
                " "
            );
        }


        // 名称
        ips200_show_string(
            16,
            y,
            child->name
        );


        // 参数值
        if(child->param != NULL)
        {
            Menu_ShowParameterValue(
                child->param,
                112,
                y
            );
        }
    }


    // ------------------------------
    // 最后一行
    // ------------------------------

    if(menu->parent != NULL)
    {
        ips200_show_string(
            0,
            112,
            "K4: Return"
        );
    }
    else
    {
        ips200_show_string(
            0,
            112,
            "K4: Start"
        );
    }
}

// 更新光标
//
// 如果没有翻页，只修改两个箭头。
// 如果发生翻页，再整屏刷新。

static void Menu_UpdateCursor(
    option *menu,
    uint8 old_cursor,
    uint8 old_top
)
{
    if(menu->top != old_top)
    {
        Menu_Display(menu);
        return;
    }


    // 删除原箭头
    uint16 old_y =
        (old_cursor - old_top + 1) * 16;

    ips200_show_string(
        0,
        old_y,
        " "
    );


    // 添加新箭头
    uint16 new_y =
        (menu->cursor - menu->top + 1) * 16;

    ips200_show_string(
        0,
        new_y,
        ">"
    );
}


// 光标向上

static void Menu_MoveUp(option *menu)
{
    if(menu == NULL ||
       menu->child_count == 0)
    {
        return;
    }

    uint8 old_cursor =
        menu->cursor;

    uint8 old_top =
        menu->top;


    if(menu->cursor == 0)
    {
        // 第一项 -> 最后一项
        menu->cursor =
            menu->child_count - 1;

        if(menu->child_count >
           MENU_VISIBLE_ITEMS)
        {
            menu->top =
                menu->child_count -
                MENU_VISIBLE_ITEMS;
        }
        else
        {
            menu->top = 0;
        }
    }
    else
    {
        menu->cursor--;

        if(menu->cursor <
           menu->top)
        {
            menu->top =
                menu->cursor;
        }
    }


    Menu_UpdateCursor(
        menu,
        old_cursor,
        old_top
    );
}

// 光标向下

static void Menu_MoveDown(option *menu)
{
    if(menu == NULL ||
       menu->child_count == 0)
    {
        return;
    }

    uint8 old_cursor =
        menu->cursor;

    uint8 old_top =
        menu->top;


    menu->cursor++;


    // 最后一项 -> 第一项
    if(menu->cursor >=
       menu->child_count)
    {
        menu->cursor = 0;
        menu->top = 0;
    }
    else
    {
        if(menu->cursor >=
           menu->top +
           MENU_VISIBLE_ITEMS)
        {
            menu->top++;
        }
    }


    Menu_UpdateCursor(
        menu,
        old_cursor,
        old_top
    );
}

// 参数增加

static void Menu_ParamIncrease(
    Parameter *param
)
{
    if(param == NULL)
        return;


    switch(param->type)
    {
        // ----------------------------------------------------
        // float
        // ----------------------------------------------------

        case PARAM_FLOAT:
        {
            float value =
                *param->target.float_ptr;

            value +=
                param->step.float_val;

            if(value >
               param->max.float_val)
            {
                value =
                    param->max.float_val;
            }

            *param->target.float_ptr =
                value;

            break;
        }


        // ----------------------------------------------------
        // int16
        // ----------------------------------------------------

        case PARAM_INT16:
        {
            int32 value =
                *param->target.int16_ptr;

            value +=
                param->step.int16_val;

            if(value >
               param->max.int16_val)
            {
                value =
                    param->max.int16_val;
            }

            *param->target.int16_ptr =
                (int16)value;

            break;
        }


        // ----------------------------------------------------
        // uint16
        // ----------------------------------------------------

        case PARAM_UINT16:
        {
            uint32 value =
                *param->target.uint16_ptr;

            value +=
                param->step.uint16_val;

            if(value >
               param->max.uint16_val)
            {
                value =
                    param->max.uint16_val;
            }

            *param->target.uint16_ptr =
                (uint16)value;

            break;
        }

				// ----------------------------------------------------
        // uint8
        // ----------------------------------------------------
        case PARAM_UINT8:
        {
            uint8 value =
                *param->target.uint8_ptr;

            value +=
                param->step.uint8_val;

            if(value >
               param->max.uint8_val)
            {
                value =
                    param->max.uint8_val;
            }

            *param->target.uint8_ptr =
                (uint8)value;

            break;
        }

        default:
            break;
    }
}

// 参数减小

static void Menu_ParamDecrease(
    Parameter *param
)
{
    if(param == NULL)
        return;


    switch(param->type)
    {
        // ----------------------------------------------------
        // float
        // ----------------------------------------------------

        case PARAM_FLOAT:
        {
            float value =
                *param->target.float_ptr;

            value -=
                param->step.float_val;

            if(value <
               param->min.float_val)
            {
                value =
                    param->min.float_val;
            }

            *param->target.float_ptr =
                value;

            break;
        }


        // ----------------------------------------------------
        // int16
        // ----------------------------------------------------

        case PARAM_INT16:
        {
            int16 value =
                *param->target.int16_ptr;

            value -=
                param->step.int16_val;

            if(value <
               param->min.int16_val)
            {
                value =
                    param->min.int16_val;
            }

            *param->target.int16_ptr =
                (int16)value;

            break;
        }


        // ----------------------------------------------------
        // uint16
        // ----------------------------------------------------

        case PARAM_UINT16:
        {
            int16 value =
                *param->target.uint16_ptr;

            value -=
                param->step.uint16_val;

            if(value <
               param->min.uint16_val)
            {
                value =
                    param->min.uint16_val;
            }

            *param->target.uint16_ptr =
                (uint16)value;

            break;
        }
        // ----------------------------------------------------
        // uint8
        // ----------------------------------------------------
        case PARAM_UINT8:
        {
            uint8 value =
                *param->target.uint8_ptr;

            value -=
                param->step.uint8_val;

            if(value <
               param->min.uint8_val)
            {
                value =
                    param->min.uint8_val;
            }

            *param->target.uint8_ptr =
                (uint8)value;

            break;
        }
				
        default:
            break;
    }
}

// 参数编辑

static void Menu_EditParameter(
    option *opt
)
{
    if(opt == NULL ||
       opt->param == NULL)
    {
        return;
    }


    Parameter *param =
        opt->param;


    // ------------------------------
    // 参数界面只绘制一次
    // ------------------------------

    ips200_clear();


    ips200_show_string(
        0,
        0,
        opt->name
    );


    ips200_show_string(
        0,
        32,
        "K1: Increase"
    );

    ips200_show_string(
        0,
        48,
        "K2: Decrease"
    );

    ips200_show_string(
        0,
        64,
        "K4: Return"
    );


    ips200_show_string(
        0,
        96,
        "Value:"
    );


    Menu_ShowParameterValue(
        param,
        64,
        96
    );


    // ------------------------------
    // 阻塞调参
    // ------------------------------

    while(1)
    {
        MenuKey key =
            Menu_ReadKey();


        // 增加
        if(key == MENU_KEY_UP)
        {
            Menu_ParamIncrease(
                param
            );

            // 只刷新数字
            Menu_ShowParameterValue(
                param,
                64,
                96
            );
        }


        // 减少
        else if(key ==
                MENU_KEY_DOWN)
        {
            Menu_ParamDecrease(
                param
            );

            // 只刷新数字
            Menu_ShowParameterValue(
                param,
                64,
                96
            );
        }


        // 返回
        else if(key ==
                MENU_KEY_BACK)
        {
            return;
        }
    }
}

// 菜单运行

void Menu_Run(option *root)
{
    if(root == NULL)
        return;


    option *current =
        root;


    Menu_Display(
        current
    );


    while(current != NULL)
    {
        MenuKey key =
            Menu_ReadKey();


        // 没按键
        if(key == MENU_KEY_NONE)
        {
            continue;
        }

        // 上
        if(key == MENU_KEY_UP)
        {
            Menu_MoveUp(
                current
            );

            continue;
        }

        // 下
        if(key == MENU_KEY_DOWN)
        {
            Menu_MoveDown(
                current
            );

            continue;
        }

        // Enter
        if(key == MENU_KEY_ENTER)
        {
            if(current->child_count == 0)
            {
                continue;
            }


            option *selected =
                current->children[
                    current->cursor
                ];


            if(selected == NULL)
            {
                continue;
            }

            // 子菜单
            if(selected->child_count > 0)
            {
                current = selected;

                Menu_Display(
                    current
                );

                continue;
            }

            // 参数
            if(selected->param != NULL)
            {
                Menu_EditParameter(
                    selected
                );


                // 返回以后重新绘制当前菜单
                Menu_Display(
                    current
                );

                continue;
            }
						
            // 功能
            if(selected->action != NULL)
            {
                selected->action();

                Menu_Display(
                    current
                );

                continue;
            }
        }


        // Back
        if(key == MENU_KEY_BACK)
        {
            // 子菜单
            if(current->parent != NULL)
            {
                current =
                    current->parent;

                Menu_Display(
                    current
                );
            }

            // 根菜单
            else
            {
                // 退出菜单
                current = NULL;
            }
        }
    }
}
void Menu_Run_Main(void)
{
	Menu_Run(&menu_main);
}