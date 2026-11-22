#include "sokoban.h"

#include <stddef.h>

#include "sokoban_map1.h"
#include "sokoban_solver1.h"
#include "sokoban_solver2.h"
#include "sokoban_solver3.h"
#include "sokoban_workspace.h"

static char sokoban_empty_commands[] = "";

static int sokoban_is_valid_level(SokobanLevelId level)
{
    return level >= SOKOBAN_LEVEL_1 && level <= SOKOBAN_LEVEL_3;
}

int sokoban_plan(SokobanLevelId level,
                 SokobanStage stage,
                 char map[SOKOBAN_MAP_ROWS][SOKOBAN_MAP_COLS])
{
    if (!sokoban_is_valid_level(level) || map == NULL) {
        return -1;
    }

    switch (level) {
        case SOKOBAN_LEVEL_1:
            /* Level 1 parses the map before running its push planner. */
            parse_map_with_vision1(map);
            solve_sokoban1();
            break;

        case SOKOBAN_LEVEL_2:
            if (stage == SOKOBAN_STAGE_PREPARE) {
                plan_explore_phase2(map);
            } else {
                plan_push_phase2(map);
            }
            break;

        case SOKOBAN_LEVEL_3:
            if (stage == SOKOBAN_STAGE_PREPARE) {
                plan_bomb_explore_phase3(map);
            } else {
                plan_push_phase3(map);
            }
            break;

        default:
            return -1;
    }

    return sokoban_error(level);
}

char *sokoban_commands(SokobanLevelId level, SokobanStage stage)
{
    switch (level) {
        case SOKOBAN_LEVEL_1:
            return final_commands1;

        case SOKOBAN_LEVEL_2:
            return (stage == SOKOBAN_STAGE_PREPARE)
                       ? explore_commands2
                       : push_commands2;

        case SOKOBAN_LEVEL_3:
            return (stage == SOKOBAN_STAGE_PREPARE)
                       ? bomb_explore_commands3
                       : level3_commands3;

        default:
            return sokoban_empty_commands;
    }
}

static void sokoban_apply_level2_vision(
    char map[SOKOBAN_MAP_ROWS][SOKOBAN_MAP_COLS],
    const char *vision_stream,
    uint8_t vision_count)
{
    int count;

    if (map == NULL || vision_stream == NULL || expected_cnt2 <= 0 ||
        vision_count == 0U) {
        return;
    }

    count = expected_cnt2;
    if (count > (int)vision_count) {
        count = (int)vision_count;
    }

    for (int i = 0; i < count; i++) {
        int x = expected_targets2[i].x;
        int y = expected_targets2[i].y;

        if (x >= 0 && x < SOKOBAN_MAP_COLS &&
            y >= 0 && y < SOKOBAN_MAP_ROWS) {
            map[y][x] = vision_stream[i];
        }
    }
}

static void sokoban_apply_level3_vision(
    char map[SOKOBAN_MAP_ROWS][SOKOBAN_MAP_COLS],
    const char *vision_stream,
    uint8_t vision_count)
{
    int count;

    if (map == NULL || vision_stream == NULL || expected_cnt3 <= 0 ||
        vision_count == 0U) {
        return;
    }

    count = expected_cnt3;
    if (count > (int)vision_count) {
        count = (int)vision_count;
    }

    for (int i = 0; i < count; i++) {
        int x = expected_targets3[i].x;
        int y = expected_targets3[i].y;

        if (x >= 0 && x < SOKOBAN_MAP_COLS &&
            y >= 0 && y < SOKOBAN_MAP_ROWS) {
            map[y][x] = vision_stream[i];
        }
    }
}

void sokoban_apply_vision(SokobanLevelId level,
                          char map[SOKOBAN_MAP_ROWS][SOKOBAN_MAP_COLS],
                          const char *vision_stream,
                          uint8_t vision_count)
{
    switch (level) {
        case SOKOBAN_LEVEL_2:
            sokoban_apply_level2_vision(map, vision_stream, vision_count);
            break;

        case SOKOBAN_LEVEL_3:
            sokoban_apply_level3_vision(map, vision_stream, vision_count);
            break;

        default:
            break;
    }
}

int sokoban_expected_count(SokobanLevelId level)
{
    switch (level) {
        case SOKOBAN_LEVEL_2:
            return expected_cnt2;
        case SOKOBAN_LEVEL_3:
            return expected_cnt3;
        default:
            return 0;
    }
}

int sokoban_error(SokobanLevelId level)
{
    switch (level) {
        case SOKOBAN_LEVEL_1:
            return sokoban_error1;
        case SOKOBAN_LEVEL_2:
            return sokoban_error2;
        case SOKOBAN_LEVEL_3:
            return level3_error3 ? level3_error_code3 : 0;
        default:
            return -1;
    }
}

void sokoban_reset(SokobanLevelId level)
{
    switch (level) {
        case SOKOBAN_LEVEL_1:
            clear_sokoban_memory1();
            break;

        case SOKOBAN_LEVEL_2:
            clear_sokoban_memory2();
            break;

        case SOKOBAN_LEVEL_3:
            level3_reset3();
            sokoban_workspace_clear();
            break;

        default:
            break;
    }
}
