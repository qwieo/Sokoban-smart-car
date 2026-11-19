#ifndef SOKOBAN_MAP1_H
#define SOKOBAN_MAP1_H

#include "sokoban_types1.h"

// 宣告你的全局变量
/* 第一关解析结果。raw_map_safe1 只保留可通行底图和墙，动态元素另存为坐标。 */
extern char raw_map_safe1[12][16]; // 地图底图
extern int map_w1;   
extern int map_h1;   
extern Pos1 start_player1;

/* 初始箱子坐标及数量，容量受 MAX_BOXES1 限制。 */
extern Pos1 initial_boxes1[MAX_BOXES1];
extern int num_initial_boxes1;

/* 初始目标坐标及数量；map_parse_error1 表示箱子或目标数量超过容量。 */
extern Pos1 initial_targets1[MAX_BOXES1];
extern int num_initial_targets1;
extern int map_parse_error1;

// 地图解析接口
/**
 * @brief 解析规划前已经确定的第一关字符地图。
 *
 * 算法流程：
 * 1. 固定地图为 12 行、16 列并清空计数。
 * 2. 扫描墙、玩家、箱子和目标，将动态元素保存为坐标。
 * 3. 数量超过 MAX_BOXES1 时设置 map_parse_error1。
 *
 * @param vision_map 12x16 字符地图；函数名沿用现有接口，本函数本身不执行视觉识别。
 * @return 无。
 *
 * @note
 * - 识别字符以 sokoban_map1.c 的实际分支为准。
 * - 缺少玩家字符时使用源码中的固定回退坐标。
 */
void parse_map_with_vision1(const char vision_map[12][16]);

/* Reset all level-1 map parsing state. */
/**
 * @brief 清空第一关地图解析状态。
 *
 * @return 无。
 *
 * @note 清空底图、坐标数组、标签数组对应的计数以及解析错误标志。
 */
void clear_sokoban_map1_state(void);

#endif
