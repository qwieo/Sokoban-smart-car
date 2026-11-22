

#ifndef _MENU_H_
#define _MENU_H_

#include "motor&motion.h"

// 每个菜单最多拥有多少个直接子项
#define MENU_MAX_CHILDREN       8

// IPS200一页最多显示多少个菜单项
#define MENU_VISIBLE_ITEMS      6


// ============================================================
// 按键定义
// ============================================================

typedef enum
{
    MENU_KEY_NONE = 0,

    MENU_KEY_UP,       // KEY1
    MENU_KEY_DOWN,     // KEY2
    MENU_KEY_ENTER,    // KEY3
    MENU_KEY_BACK      // KEY4

} MenuKey;

// 参数类型
typedef enum
{
    PARAM_FLOAT,
    PARAM_INT16,
    PARAM_UINT16,
		PARAM_UINT8

} ParamType;

// 参数指针
typedef union
{
    float  *float_ptr;
    int16  *int16_ptr;
    uint16 *uint16_ptr;
		uint8	 *uint8_ptr;

} ParamPtr;

// 参数数值
typedef union
{
    float  float_val;
    int16  int16_val;
    uint16 uint16_val;
		uint8  uint8_val;
} ParamValue;


// 参数结构体
typedef struct
{
    ParamType type;

    ParamPtr target;

    // 调节步长
    ParamValue step;

    // 最小值
    ParamValue min;

    // 最大值
    ParamValue max;

    // float显示小数位
    uint8 decimals;

} Parameter;

// 菜单结构
typedef struct menu_option
{
    char *name;

    struct menu_option *parent;

    struct menu_option *children[MENU_MAX_CHILDREN];

    uint8 child_count;

    // 当前光标实际索引
    uint8 cursor;

    // 当前页面从第几个菜单项开始显示
    uint8 top;

    // 如果非NULL，则这是参数项
    Parameter *param;

    // 如果非NULL，则这是功能项
    void (*action)(void);

} option;

// 菜单项初始化

void Menu_OptionInit(
    option *opt,
    char *name
);

void Menu_ParamOptionInit(
    option *opt,
    char *name,
    Parameter *param
);

void Menu_ActionOptionInit(
    option *opt,
    char *name,
    void (*action)(void)
);


// ============================================================
// 参数初始化
void Menu_ParamFloatInit(
    Parameter *param,
    float *target,
    float step,
    float min,
    float max,
    uint8 decimals
);

void Menu_ParamInt16Init(
    Parameter *param,
    int16 *target,
    int16 step,
    int16 min,
    int16 max
);

void Menu_ParamUint16Init(
    Parameter *param,
    uint16 *target,
    uint16 step,
    uint16 min,
    uint16 max
);

void Menu_ParamUint8Init(
    Parameter *param,
    uint8 *target,
    uint8 step,
    uint8 min,
    uint8 max
);

// 菜单结构操作
uint8 Menu_AddChild(
    option *parent,
    option *child
);

// 阻塞运行菜单
// 根菜单KEY4退出
void App_MenuInit(void);
void Menu_Run(option *root);
void Menu_Run_Main(void);
// 显示
void Menu_Display(option *menu);


#endif