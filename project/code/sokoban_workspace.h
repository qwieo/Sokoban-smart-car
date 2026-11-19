#ifndef SOKOBAN_WORKSPACE_H
#define SOKOBAN_WORKSPACE_H

#include <stdint.h>
#include "sokoban_types1.h"
#include "sokoban_types2.h"

/* 第一关单个箱子阶段的反向路径容量，同时约束分段路径重建。 */
#define SOKOBAN_LEVEL1_SEGMENT_PATH_CAPACITY 1000
#define SOKOBAN_LEVEL2_TSP_MAX_OBJECTS SOKOBAN_RECOGNITION_OBJECT_CAPACITY
#define SOKOBAN_LEVEL2_TSP_MAX_CANDIDATES 80
#define SOKOBAN_LEVEL2_INTERNAL_PATH_CAPACITY 1000
#define SOKOBAN_LEVEL2_BFS_PATH_CAPACITY 192
#define SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY 3000

#define SOKOBAN_LEVEL1_PUSH_MAX_NODES 12000
#define SOKOBAN_LEVEL1_PUSH_HASH_SIZE 32768
#define SOKOBAN_LEVEL1_PUSH_PATH_CAPACITY 3000
#define SOKOBAN_LEVEL1_CELL_COUNT (12 * 16)

#define SOKOBAN_LEVEL2_STRICT_EXACT_MAX_BOXES 6
#define SOKOBAN_LEVEL2_STRICT_BEAM_WIDTH 48
#define SOKOBAN_LEVEL2_STRICT_MASK_COUNT \
    (1u << SOKOBAN_LEVEL2_STRICT_EXACT_MAX_BOXES)
#define SOKOBAN_LEVEL2_STRICT_DP_STATES \
    (SOKOBAN_LEVEL2_STRICT_MASK_COUNT * 12u * 16u)
#define SOKOBAN_LEVEL2_STRICT_TRANSITION_STATES \
    (12u * 16u * 12u * 16u)


/* 第三关识别顺序优化器的固定容量。
 * 对象总容量由 SOKOBAN_RECOGNITION_OBJECT_CAPACITY 决定；为了避免 2^N
 * 静态 RAM 随箱数指数膨胀，只有不超过 EXACT_DP_MAX_OBJECTS 的场景走
 * 精确 subset-DP，更大场景自动切换固定内存的 scalable planner。 */
#define SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS SOKOBAN_RECOGNITION_EXACT_DP_MAX_OBJECTS
#define SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY SOKOBAN_RECOGNITION_OBJECT_CAPACITY
#define SOKOBAN_LEVEL3_RECOG_MAX_POSES \
    (SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY * 4)
#define SOKOBAN_LEVEL3_RECOG_MAX_MASKS \
    (1u << SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS)
#define SOKOBAN_LEVEL3_RECOG_MAX_STATES \
    (SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS * 4u * \
     (1u << (SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS - 1)))
#define SOKOBAN_LEVEL3_RECOG_PATH_CAPACITY 500

typedef struct {
    uint8_t object_index;   /* 对应 all_objs[] 下标。 */
    uint8_t local_index;    /* 该对象内部的第几个识别姿态。 */
    uint8_t stand_cell;     /* 小车识别时所在栅格，编码为 y*16+x。 */
    char view_dir;          /* 识别时车头方向：U/R/D/L。 */
} SokobanRecognitionPose3;

typedef struct {
    /* 压缩 DP：仅为 mask 中最后对象的合法姿态分配状态。 */
    uint16_t dp_cost[SOKOBAN_LEVEL3_RECOG_MAX_STATES];
    uint8_t dp_parent_pose[SOKOBAN_LEVEL3_RECOG_MAX_STATES];
    uint32_t mask_base[SOKOBAN_LEVEL3_RECOG_MAX_MASKS + 1u];

    uint16_t start_cost[SOKOBAN_LEVEL3_RECOG_MAX_POSES];
    uint16_t edge_cost[SOKOBAN_LEVEL3_RECOG_MAX_POSES]
                      [SOKOBAN_LEVEL3_RECOG_MAX_POSES];

    SokobanRecognitionPose3 poses[SOKOBAN_LEVEL3_RECOG_MAX_POSES];
    uint8_t object_pose_start[SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY];
    uint8_t object_pose_count[SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY];
    uint8_t chosen_pose_sequence[SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY];
    char path_scratch[SOKOBAN_LEVEL3_RECOG_PATH_CAPACITY];
} SokobanRecognitionWorkspace3;

/**
 * @brief 排列枚举的显式迭代帧。
 *
 * next_index 保存当前深度下一名候选的下标；active_index 保存本层已经交换
 * 到当前位置的候选下标，回溯时据此执行与递归版本相同的交换撤销。
 * 两个字段均为 uint8_t，第一关实际深度受 MAX_BOXES1 限制。
 */
typedef struct {
    uint8_t next_index;
    uint8_t active_index;
} SokobanPermutationFrame;

typedef struct {
    uint8_t sx, sy, ex, ey, type;
    int len;
    uint8_t turns;
    char first_dir, last_dir;
    char path[64];
} SokobanBfsCache2;

typedef struct {
    uint16_t parent, g_cost, f_cost, heap_pos;
    uint8_t player;
    uint8_t boxes[MAX_BOXES2];
    uint8_t move_box, move_dir;
    SokobanBoxMask active_box;
    uint8_t closed;
    /* Level 1 uses region as the canonical player-region key. */
    uint8_t region;
} SokobanPushNode2;

typedef struct {
    SokobanBoxMask completed_mask;
    uint16_t cost;
    uint16_t score;
    uint8_t player;
    uint8_t order[MAX_BOXES2];
    uint8_t start_player[MAX_BOXES2];
    uint8_t end_player[MAX_BOXES2];
} SokobanStrictBeamState2;

/* Compact candidate retained for every explicit TSP depth.  All represented
 * values are bounded by the 16x12 map, 600-entry cache and 64-byte cache path. */
typedef struct {
    uint16_t cache_idx;
    uint8_t next_i;
    uint8_t next_cell;
    uint8_t path_len;
    uint8_t move_turns;
    char first_move_dir;
    char last_move_dir;
    char target_dir;
    char rot_cmd;
    uint8_t rot_len;
    uint8_t rot_cost;
} SokobanTspCandidate2;

typedef struct {
    uint32_t unvisited_mask;
    int current_cost;
    int current_turns;
    uint16_t current_len;
    uint8_t curr_cell;
    char curr_dir;
    char last_move_dir;
    uint8_t next_candidate;
    uint8_t candidate_count;
    uint8_t entered;
} SokobanTspFrame2;

/*
 * 各关卡的大容量临时数组放在同一个联合体中复用静态 RAM。
 * 第一关求解期间只能使用 level1 成员；切换关卡前由
 * sokoban_workspace_clear() 清空整个联合体，不能并行保留多个关卡的工作状态。
 */
typedef union {
    /* Level 1 now uses one cooperative multi-box A* state instead of the
     * old box-order x target-order factorial enumeration.  active_box in
     * SokobanPushNode2 stores the mask of already consumed generic goals. */
    struct {
        SokobanPushNode2 push_nodes[SOKOBAN_LEVEL1_PUSH_MAX_NODES];
        uint16_t push_hash[SOKOBAN_LEVEL1_PUSH_HASH_SIZE];
        uint16_t push_heap[SOKOBAN_LEVEL1_PUSH_MAX_NODES];
        uint16_t solution_chain[SOKOBAN_LEVEL1_PUSH_PATH_CAPACITY];
        uint8_t wall[SOKOBAN_LEVEL1_CELL_COUNT];
        uint8_t goal_cells[MAX_BOXES1];
        int8_t goal_index_by_cell[SOKOBAN_LEVEL1_CELL_COUNT];
        uint8_t goal_push_dist[MAX_BOXES1][SOKOBAN_LEVEL1_CELL_COUNT];
        uint8_t walk_dist[SOKOBAN_LEVEL1_CELL_COUNT];
        int16_t walk_parent[SOKOBAN_LEVEL1_CELL_COUNT];
        char walk_move[SOKOBAN_LEVEL1_CELL_COUNT];
        SokobanBoxMask goal_reach_mask[SOKOBAN_LEVEL1_CELL_COUNT];
        uint8_t walk_queue[SOKOBAN_LEVEL1_CELL_COUNT];
        uint8_t occupied[SOKOBAN_LEVEL1_CELL_COUNT];
        char work_commands[SOKOBAN_LEVEL1_PUSH_PATH_CAPACITY];
        char reverse_path[SOKOBAN_LEVEL1_CELL_COUNT];
    } level1;

    struct {
        char dynamic_map[12][16];
        union {
            /* Explore and push planning are sequential.  Their large scratch
             * areas intentionally overlap; only the exported result buffers and
             * explore_end_pos survive the phase transition. */
            struct {
                uint16_t move_parent[12 * 16 * 5];
                uint16_t move_queue[12 * 16 * 5];
                uint8_t move_dist[12 * 16 * 5];
                uint8_t move_turns[12 * 16 * 5];
                char move_dir[12 * 16 * 5];
                /* The move BFS arrays must coexist with either planner.
                 * The old bounded DFS remains available for regression, while
                 * the default level-2 path uses the same compact recognition
                 * DP layout proven by level 3. */
                union {
                    struct {
                        SokobanBfsCache2 bfs_cache[600];
                        SokobanTspFrame2 tsp_stack[SOKOBAN_LEVEL2_TSP_MAX_OBJECTS];
                        SokobanTspCandidate2
                            tsp_candidates[SOKOBAN_LEVEL2_TSP_MAX_OBJECTS]
                                          [SOKOBAN_LEVEL2_TSP_MAX_CANDIDATES];
                        char tsp_current_path[SOKOBAN_LEVEL2_INTERNAL_PATH_CAPACITY];
                        char bfs_build_path[SOKOBAN_LEVEL2_BFS_PATH_CAPACITY];
                    } legacy;
                    SokobanRecognitionWorkspace3 recognition;
                } planner;
            } explore;

            struct {
                /* Data shared by the legacy A* and the strict subset DP. */
                uint8_t push_wall[12 * 16];
                uint8_t push_static_dist[MAX_BOXES2][12 * 16];
                uint8_t push_walk_dist[12 * 16];
                int16_t push_walk_parent[12 * 16];
                char push_walk_move[12 * 16];
                uint8_t push_walk_queue[12 * 16];
                uint8_t push_occupied[12 * 16];
                int16_t label_counts[2][SOKOBAN_ELEMENT_TYPE_CAPACITY];

                union {
                    /* Original implementation: enumerate every box order, then
                     * run a complete A* for each permutation. Retained only as
                     * a regression switch. */
                    struct {
                        SokobanPushNode2 push_nodes[12000];
                        uint16_t push_hash[32768];
                        uint16_t push_heap[12000];
                        uint16_t push_solution_chain[SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY];
                        char safe_best_commands[SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY];
                        char push_work_commands[SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY];
                        char push_reverse_path[12 * 16];
                        SokobanPermutationFrame safe_perm[MAX_BOXES2];
                    } legacy;

                    /* Strict one-box-at-a-time planner. A state is
                     * (completed-box mask, player cell). Unfinished boxes stay
                     * at their original cells, so common order prefixes are
                     * solved once instead of once per full permutation. */
                    struct {
                        uint16_t dp_cost[SOKOBAN_LEVEL2_STRICT_DP_STATES];
                        uint8_t dp_parent_player[SOKOBAN_LEVEL2_STRICT_DP_STATES];
                        uint8_t dp_parent_box[SOKOBAN_LEVEL2_STRICT_DP_STATES];
                        union {
                            struct {
                                uint16_t dist[SOKOBAN_LEVEL2_STRICT_TRANSITION_STATES];
                                uint16_t queue[SOKOBAN_LEVEL2_STRICT_TRANSITION_STATES];
                            } plan;
                            struct {
                                uint16_t parent[SOKOBAN_LEVEL2_STRICT_TRANSITION_STATES];
                                uint16_t queue[SOKOBAN_LEVEL2_STRICT_TRANSITION_STATES];
                                char move[SOKOBAN_LEVEL2_STRICT_TRANSITION_STATES];
                            } rebuild;
                        } transition;
                        uint8_t chosen_box[MAX_BOXES2];
                        uint8_t chosen_start_player[MAX_BOXES2];
                        uint8_t chosen_end_player[MAX_BOXES2];
                        uint8_t target_cell[MAX_BOXES2];
                        SokobanStrictBeamState2 beam_current[SOKOBAN_LEVEL2_STRICT_BEAM_WIDTH];
                        SokobanStrictBeamState2 beam_next[SOKOBAN_LEVEL2_STRICT_BEAM_WIDTH];
                        char work_commands[SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY];
                        char reverse_path[SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY];
                    } strict;
                } planner;
            } push;
        } stage;
    } level2;

    union {
        /* 保留原第三关旧工作区布局，避免未来旧辅助函数重新启用时失去容量。 */
        struct {
            uint16_t bfs_parent[65536];
            uint32_t bfs_queue[12000];
            uint8_t bfs_visit_mark[65536];
            char bfs_move_dir[65536];
            char dynamic_map[12][16];
            char current_map[12][16];
        } legacy;

        /* 第三关识别顺序 DP 与旧工作区、第一关、第二关互斥复用 OCRAM。 */
        SokobanRecognitionWorkspace3 recognition;
    } level3;
} SokobanSharedWorkspace;

/* 全工程唯一共享工作区实例；联合体布局和成员容量属于固定内存接口。 */
extern SokobanSharedWorkspace g_sokoban_workspace;
/**
 * @brief 将共享工作区的全部字节清零。
 *
 * @return 无。
 *
 * @note 会同时覆盖 level1、level2、level3 的联合体存储，不保留任何关卡临时数据。
 */
void sokoban_workspace_clear(void);

#endif
