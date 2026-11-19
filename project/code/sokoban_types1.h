#ifndef SOKOBAN_TYPES1_H
#define SOKOBAN_TYPES1_H


#include <stdint.h>
#include <stdbool.h>
#include "sokoban_config.h"

/* Level-1 box storage capacity. This is not the box count of the current
 * map; solve_sokoban1() uses the count parsed from the map at runtime. Node
 * and hash capacities are owned by sokoban_workspace.h. */
#define MAX_BOXES1 SOKOBAN_BOX_CAPACITY     

// 坐标结构体
/**
 * @brief 第一关二维格点坐标。
 *
 * x 表示列坐标，y 表示行坐标。12x16 地图坐标能够放入 int8_t。
 * 求解阶段还使用 (-1,-1) 表示已经进入目标并从地图中消失的箱子。
 */
typedef struct { 
    int8_t x, y; 
} Pos1;

#endif