#ifndef SOKOBAN_SOLVER1_H
#define SOKOBAN_SOLVER1_H

#include <stdint.h>
#include "sokoban_types1.h"

#define SOKOBAN_OK1 0
#define SOKOBAN_ERROR_INVALID_MAP1 1
#define SOKOBAN_ERROR_PATH_CAPACITY1 2
#define SOKOBAN_ERROR_SEARCH_CAPACITY1 3
#define SOKOBAN_ERROR_SEARCH_STACK1 SOKOBAN_ERROR_SEARCH_CAPACITY1 /* legacy alias */
#define SOKOBAN_ERROR_NO_SOLUTION1 4
#define SOKOBAN_LEVEL1_OUTPUT_CAPACITY 1000

extern int sokoban_error1;
extern uint32_t search_nodes1;
extern int final_steps1;
extern char final_commands1[SOKOBAN_LEVEL1_OUTPUT_CAPACITY];

/* Keil Watch diagnostics for the cooperative weighted-A* planner. */
extern volatile uint32_t g_l1_astar_attempts1;
extern volatile uint32_t g_l1_astar_nodes1;
extern volatile uint32_t g_l1_astar_pushes1;
extern volatile uint32_t g_l1_astar_reopens1;
extern volatile int32_t g_l1_astar_weight1;
extern volatile int32_t g_l1_astar_result1;

/* Small, volatile level-1 counters intended for Keil Watch and host tests. */
extern volatile uint32_t expanded_nodes;
extern volatile uint32_t generated_nodes;
extern volatile uint32_t duplicate_nodes;
extern volatile uint32_t deadlock_pruned;
extern volatile uint32_t max_open;
extern volatile uint32_t final_path_len;
extern volatile uint32_t g_l1_hash_lookups1;
extern volatile uint32_t g_l1_hash_probes1;
extern volatile uint32_t g_l1_hash_collisions1;
extern volatile uint32_t g_l1_hash_max_probe1;
extern volatile uint32_t g_l1_node_pool_hits1;
extern volatile uint32_t g_l1_hash_full_hits1;

/* Solve all generic boxes and goals jointly. A box disappears after entering
 * any unused goal. The returned commands contain only U/D/L/R. */
void solve_sokoban1(void);

/* Clear level-1 results, parsed map state and the shared OCRAM workspace. */
void clear_sokoban_memory1(void);

#endif
