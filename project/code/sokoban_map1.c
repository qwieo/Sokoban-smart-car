#include "sokoban_types1.h"
#include <string.h>

/// ================= 全局变量物理分区 =================

/*
 * 地图静态底图。解析前填充为 '-'，只把墙 '#' 写入底图；玩家、箱子和目标
 * 分别存入独立坐标，因此这些动态元素所在格在底图中保持为普通可通行地面。
 */
char raw_map_safe1[12][16]; // 地图底图
int map_w1 = 0;  
int map_h1 = 0;  

/* 玩家起点。若输入中没有 '@'，解析函数按现有行为回退到 (1,6)。 */
Pos1 start_player1;   // 玩家初始位置

// 箱子初始
/*
 * 箱子坐标、标签和数量。box_labels 当前只记录识别到的 '$' 字符，
 * 第一关求解器按索引枚举箱子，不读取该标签数组进行大小写匹配。
 */
Pos1 initial_boxes1[MAX_BOXES1];
char box_labels[MAX_BOXES1];
int num_initial_boxes1 = 0;

// 目标初始
/*
 * 目标坐标、标签和数量。target_labels 当前只记录识别到的 '.' 字符；
 * map_parse_error1 仅在箱子或目标数量超过 MAX_BOXES1 时由本文件置位。
 */
Pos1 initial_targets1[MAX_BOXES1];
char target_labels[MAX_BOXES1];
int num_initial_targets1 = 0;
int map_parse_error1 = 0;

// 视觉

/**
 * @brief 将第一关 12x16 字符地图拆分为静态底图和动态元素坐标。
 *
 * 算法流程：
 * 1. 固定 map_h1=12、map_w1=16，清零箱子数、目标数和解析错误。
 * 2. 先用 '-' 填充 raw_map_safe1，使未识别字符和动态元素位置保持可通行。
 * 3. 按行扫描 '#', '@', '$', '.'，分别保存墙、玩家、箱子和目标。
 * 4. 箱子或目标超过 MAX_BOXES1 时设置 map_parse_error1。
 * 5. 未找到玩家时使用固定坐标 (1,6)。
 *
 * @param vision_map 已完成识别的 12x16 字符地图。
 * @return 无。
 *
 * @note
 * - 本函数名称保留“vision”，但当前代码只解析调用者提供的字符数组。
 * - 每行遇到空字符就停止该行扫描，其后格子保留为 '-'。
 * - 当前源码不解析大小写箱子/目标字符，只识别 '$' 和 '.'。
 */
void parse_map_with_vision1(const char vision_map[12][16]){
	// 1. 初始化
	 map_h1 = 12; 
   map_w1 = 16; 
   num_initial_boxes1 = 0; 
   num_initial_targets1 = 0;
   map_parse_error1 = 0;

	// 2. 刷
	
		/* 动态元素不写入底图，后续规划阶段再按当前箱子和目标阶段叠加。 */
memset(raw_map_safe1, '-', sizeof(raw_map_safe1));
    
  bool player_found = false;

	// 3. 扫描地图
    for (int r = 0; r < 12; r++){
        for (int c = 0; c < 16; c++){
            char ch = vision_map[r][c];
            if (ch == '\0') break; 
            
            // 4. 内容分类存储
            if (ch == '#') {
                raw_map_safe1[r][c] = '#'; 
            } else if (ch == '@') {
                start_player1 = (Pos1){(int8_t)c, (int8_t)r}; 
                player_found = true;
            } else if (ch == '$') {
                                /* 容量内保存坐标；超过容量时不再写数组，只记录解析错误。 */
if (num_initial_boxes1 < MAX_BOXES1) {
                    box_labels[num_initial_boxes1] = ch;
                    initial_boxes1[num_initial_boxes1++] = (Pos1){(int8_t)c, (int8_t)r};
                } else {
                    map_parse_error1 = 1;
                }
            } else if (ch == '.') {
                if (num_initial_targets1 < MAX_BOXES1) {
                    target_labels[num_initial_targets1] = ch;
                    initial_targets1[num_initial_targets1++] = (Pos1){(int8_t)c, (int8_t)r};
                } else {
                    map_parse_error1 = 1;
                }
            }
        }
    }
    
    // 5. 容错
        /* 缺少玩家不会设置 map_parse_error1，而是沿用既有回退行为。 */
if (!player_found) {
        start_player1 = (Pos1){1, 6}; 
    }
}


/**
 * @brief 复位第一关地图解析所使用的全部全局状态。
 *
 * 算法流程：
 * 1. 清空静态底图并复位地图宽高和玩家坐标。
 * 2. 清空箱子、目标及标签数组。
 * 3. 清零元素数量和 map_parse_error1。
 *
 * @return 无。
 *
 * @note 只处理第一关地图状态，不负责清空求解工作区和最终路径。
 */
void clear_sokoban_map1_state(void)
{
    memset(raw_map_safe1, 0, sizeof(raw_map_safe1));
    map_w1 = 0;
    map_h1 = 0;
    start_player1 = (Pos1){0, 0};

    memset(initial_boxes1, 0, sizeof(initial_boxes1));
    memset(box_labels, 0, sizeof(box_labels));
    num_initial_boxes1 = 0;

    memset(initial_targets1, 0, sizeof(initial_targets1));
    memset(target_labels, 0, sizeof(target_labels));
    num_initial_targets1 = 0;
    map_parse_error1 = 0;
}
