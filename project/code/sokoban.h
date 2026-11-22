#ifndef SOKOBAN_H
#define SOKOBAN_H

#include <stdint.h>

#define SOKOBAN_MAP_ROWS 12
#define SOKOBAN_MAP_COLS 16

typedef enum {
    SOKOBAN_LEVEL_1 = 1,
    SOKOBAN_LEVEL_2 = 2,
    SOKOBAN_LEVEL_3 = 3
} SokobanLevelId;

typedef enum {
    SOKOBAN_STAGE_PREPARE = 0,
    SOKOBAN_STAGE_PUSH = 1
} SokobanStage;

/* Run the planner for the requested level and stage.
 * The return value is the level-specific error code, or zero on success. */
int sokoban_plan(SokobanLevelId level,
                 SokobanStage stage,
                 char map[SOKOBAN_MAP_ROWS][SOKOBAN_MAP_COLS]);

/* Return the null-terminated command buffer produced by the requested stage. */
char *sokoban_commands(SokobanLevelId level, SokobanStage stage);

/* Apply recognition bytes to the targets collected by the prepare stage. */
void sokoban_apply_vision(SokobanLevelId level,
                          char map[SOKOBAN_MAP_ROWS][SOKOBAN_MAP_COLS],
                          const char *vision_stream,
                          uint8_t vision_count);

/* Return the number of recognition targets currently waiting for labels. */
int sokoban_expected_count(SokobanLevelId level);

/* Return the most recent level-specific error code. */
int sokoban_error(SokobanLevelId level);

/* Clear planner state and all level-owned command/recognition buffers. */
void sokoban_reset(SokobanLevelId level);

#endif
