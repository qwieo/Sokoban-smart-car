#ifndef SOKOBAN_TYPES3_H
#define SOKOBAN_TYPES3_H

#include <stdint.h>
#include <stdbool.h>
#include "sokoban_config.h"

#define MAX_BOXES3 SOKOBAN_BOX_CAPACITY
#define MAX_BOMBS3 SOKOBAN_LEVEL3_BOMB_CAPACITY

typedef struct {
    int x;
    int y;
} Pos3;

typedef struct {
    char lable;
    Pos3 box_pos;
    Pos3 target_pos;
} BoxTargetPair3;

#endif
