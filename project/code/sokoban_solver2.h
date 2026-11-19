#ifndef SOKOBAN_SOLVER2_H
#define SOKOBAN_SOLVER2_H

#include <stdint.h>
#include "sokoban_types2.h"

#define SOKOBAN_OK2 0
#define SOKOBAN_ERROR_INVALID_MAP2 1
#define SOKOBAN_ERROR_EXPLORE_PATH_CAPACITY2 2
#define SOKOBAN_ERROR_EXPECTED_TARGET_CAPACITY2 3
#define SOKOBAN_ERROR_SEARCH_STACK2 4
#define SOKOBAN_ERROR_PUSH_PATH_CAPACITY2 5
#define SOKOBAN_LEVEL2_OUTPUT_CAPACITY 1000
#define SOKOBAN_LEVEL2_RECOGNITION_CAPACITY (MAX_BOXES2 * 2)

#define PUSH_PLAN_NONE2 0
#define PUSH_PLAN_SAFE_DIRECT2 1
#define PUSH_PLAN_COOPERATIVE2 2
#define PUSH_ALLOW_COOPERATIVE_FALLBACK2 0

extern int sokoban_error2;
extern uint32_t explore_search_nodes2;
extern uint32_t push_search_nodes2;
extern int explore_steps2;
extern char explore_commands2[SOKOBAN_LEVEL2_OUTPUT_CAPACITY];
extern Pos2 expected_targets2[SOKOBAN_LEVEL2_RECOGNITION_CAPACITY];
extern int expected_cnt2;
extern int push_steps2;
extern char push_commands2[SOKOBAN_LEVEL2_OUTPUT_CAPACITY];
extern int push_plan_mode2;
extern char push_safe_order2[MAX_BOXES2 + 1];

/* Recognition-order DP diagnostics. */
extern volatile uint32_t g_l2_recog_move_bfs_calls2;
extern volatile uint32_t g_l2_recog_dp_states2;
extern volatile uint32_t g_l2_recog_dp_transitions2;
extern volatile uint32_t g_l2_recog_pose_count2;
extern volatile uint32_t g_l2_recog_compact_state_capacity2;
extern volatile int32_t g_l2_recog_best_cost2;
extern volatile int32_t g_l2_recog_result2;

/* Strict one-box-at-a-time subset-DP diagnostics. */
extern volatile uint32_t g_l2_strict_dp_states2;
extern volatile uint32_t g_l2_strict_transition_searches2;
extern volatile uint32_t g_l2_strict_transition_nodes2;
extern volatile int32_t g_l2_strict_best_cost2;
extern volatile int32_t g_l2_strict_result2;

void plan_explore_phase2(char external_map[12][16]);
void plan_push_phase2(char external_map[12][16]);
void clear_sokoban_memory2(void);

#endif
