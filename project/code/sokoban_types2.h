#ifndef SOKOBAN_TYPES2_H
#define SOKOBAN_TYPES2_H
#include <stdint.h>
#include <stdbool.h>
#include "sokoban_config.h"
#define MAX_BOXES2 SOKOBAN_BOX_CAPACITY
typedef struct { int x, y; } Pos2;
typedef struct { char label; Pos2 pos; } DynObj2;
#endif
