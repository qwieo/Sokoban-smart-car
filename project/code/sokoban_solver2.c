#include "zf_common_headfile.h"
#include <string.h>
#include <stdlib.h>
#include "sokoban_solver2.h"
#include "sokoban_workspace.h"

#ifndef LEVEL2_USE_RECOGNITION_DP2
#define LEVEL2_USE_RECOGNITION_DP2 1
#endif

#ifndef LEVEL2_USE_STRICT_ORDER_DP2
#define LEVEL2_USE_STRICT_ORDER_DP2 1
#endif

#define LEVEL2_RECOG_INF2 0xFFFFu
#define LEVEL2_RECOG_PARENT_START2 0xFEu
#define LEVEL2_RECOG_PARENT_NONE2 0xFFu

#if defined(__GNUC__) || defined(__clang__)
#define SOKOBAN_UNUSED_FN2 __attribute__((unused))
#else
#define SOKOBAN_UNUSED_FN2
#endif


/* Level 2 shares its large scratch buffers with levels 1 and 3. */
#define dynamic_map          (g_sokoban_workspace.level2.dynamic_map)
#define move_parent_buf      (g_sokoban_workspace.level2.stage.explore.move_parent)
#define move_queue_buf       (g_sokoban_workspace.level2.stage.explore.move_queue)
#define move_dist_buf        (g_sokoban_workspace.level2.stage.explore.move_dist)
#define move_turns_buf       (g_sokoban_workspace.level2.stage.explore.move_turns)
#define move_dir_buf         (g_sokoban_workspace.level2.stage.explore.move_dir)
#define tsp_stack            (g_sokoban_workspace.level2.stage.explore.planner.legacy.tsp_stack)
#define tsp_candidates       (g_sokoban_workspace.level2.stage.explore.planner.legacy.tsp_candidates)
#define tsp_current_path     (g_sokoban_workspace.level2.stage.explore.planner.legacy.tsp_current_path)
#define bfs_build_path       (g_sokoban_workspace.level2.stage.explore.planner.legacy.bfs_build_path)
#define level2_recog_ws2      (g_sokoban_workspace.level2.stage.explore.planner.recognition)
#define push_nodes           (g_sokoban_workspace.level2.stage.push.planner.legacy.push_nodes)
#define push_hash            (g_sokoban_workspace.level2.stage.push.planner.legacy.push_hash)
#define push_heap            (g_sokoban_workspace.level2.stage.push.planner.legacy.push_heap)
#define push_solution_chain  (g_sokoban_workspace.level2.stage.push.planner.legacy.push_solution_chain)
#define push_wall            (g_sokoban_workspace.level2.stage.push.push_wall)
#define push_static_dist     (g_sokoban_workspace.level2.stage.push.push_static_dist)
#define push_walk_dist       (g_sokoban_workspace.level2.stage.push.push_walk_dist)
#define push_walk_parent     (g_sokoban_workspace.level2.stage.push.push_walk_parent)
#define push_walk_move       (g_sokoban_workspace.level2.stage.push.push_walk_move)
#define push_walk_queue      (g_sokoban_workspace.level2.stage.push.push_walk_queue)
#define push_occupied        (g_sokoban_workspace.level2.stage.push.push_occupied)
#define safe_best_commands   (g_sokoban_workspace.level2.stage.push.planner.legacy.safe_best_commands)
#define push_work_commands   (g_sokoban_workspace.level2.stage.push.planner.legacy.push_work_commands)
#define push_reverse_path    (g_sokoban_workspace.level2.stage.push.planner.legacy.push_reverse_path)
#define safe_perm_stack      (g_sokoban_workspace.level2.stage.push.planner.legacy.safe_perm)
#define push_box_counts      (g_sokoban_workspace.level2.stage.push.label_counts[0])
#define push_target_counts   (g_sokoban_workspace.level2.stage.push.label_counts[1])
#define strict_dp_cost2       (g_sokoban_workspace.level2.stage.push.planner.strict.dp_cost)
#define strict_dp_parent_player2 (g_sokoban_workspace.level2.stage.push.planner.strict.dp_parent_player)
#define strict_dp_parent_box2 (g_sokoban_workspace.level2.stage.push.planner.strict.dp_parent_box)
#define strict_plan_dist2     (g_sokoban_workspace.level2.stage.push.planner.strict.transition.plan.dist)
#define strict_plan_queue2    (g_sokoban_workspace.level2.stage.push.planner.strict.transition.plan.queue)
#define strict_rebuild_parent2 (g_sokoban_workspace.level2.stage.push.planner.strict.transition.rebuild.parent)
#define strict_rebuild_queue2 (g_sokoban_workspace.level2.stage.push.planner.strict.transition.rebuild.queue)
#define strict_rebuild_move2  (g_sokoban_workspace.level2.stage.push.planner.strict.transition.rebuild.move)
#define strict_chosen_box2    (g_sokoban_workspace.level2.stage.push.planner.strict.chosen_box)
#define strict_chosen_start2  (g_sokoban_workspace.level2.stage.push.planner.strict.chosen_start_player)
#define strict_chosen_end2    (g_sokoban_workspace.level2.stage.push.planner.strict.chosen_end_player)
#define strict_target_cell2   (g_sokoban_workspace.level2.stage.push.planner.strict.target_cell)
#define strict_beam_current2  (g_sokoban_workspace.level2.stage.push.planner.strict.beam_current)
#define strict_beam_next2     (g_sokoban_workspace.level2.stage.push.planner.strict.beam_next)
#define strict_work_commands2 (g_sokoban_workspace.level2.stage.push.planner.strict.work_commands)
#define strict_reverse_path2  (g_sokoban_workspace.level2.stage.push.planner.strict.reverse_path)


// ??????????
int explore_steps2 = 0;
char explore_commands2[SOKOBAN_LEVEL2_OUTPUT_CAPACITY];
Pos2 expected_targets2[SOKOBAN_LEVEL2_RECOGNITION_CAPACITY];
int expected_cnt2 = 0;

int push_steps2 = 0;
char push_commands2[SOKOBAN_LEVEL2_OUTPUT_CAPACITY];
int sokoban_error2 = SOKOBAN_OK2;
uint32_t explore_search_nodes2 = 0;
uint32_t push_search_nodes2 = 0;

volatile uint32_t g_l2_recog_move_bfs_calls2 = 0U;
volatile uint32_t g_l2_recog_dp_states2 = 0U;
volatile uint32_t g_l2_recog_dp_transitions2 = 0U;
volatile uint32_t g_l2_recog_pose_count2 = 0U;
volatile uint32_t g_l2_recog_compact_state_capacity2 = 0U;
volatile int32_t g_l2_recog_best_cost2 = -1;
volatile int32_t g_l2_recog_result2 = 0;

volatile uint32_t g_l2_strict_dp_states2 = 0U;
volatile uint32_t g_l2_strict_transition_searches2 = 0U;
volatile uint32_t g_l2_strict_transition_nodes2 = 0U;
volatile int32_t g_l2_strict_best_cost2 = -1;
volatile int32_t g_l2_strict_result2 = 0;

/* 0=no solution, 1=safe direct-to-target plan, 2=cooperative multibox plan. */
int push_plan_mode2 = 0;
char push_safe_order2[MAX_BOXES2 + 1] = {0};


// ??????????????????????????????????
static Pos2 explore_end_pos;

// ???????????????????
static DynObj2 dyn_boxes[MAX_BOXES2];
static DynObj2 dyn_targets[MAX_BOXES2];
static int num_dyn_boxes = 0;
static int num_dyn_targets = 0;

// ?? ???????????
static int tsp_search_cnt = 0;
static bool tsp_path_capacity_hit2 = false;
static bool push_path_capacity_hit2 = false;

// ============================================================================
// ????S??????????BFS (???????)
// ============================================================================
static bool is_obstacle(int nx, int ny, bool is_target_passable) {
    /* Targets are floor for the vehicle in every explore sub-search.  Keep the
     * parameter for interface compatibility with the existing BFS cache key. */
    (void)is_target_passable;
    if (nx < 0 || ny < 0 || nx >= 16 || ny >= 12) return true;
    char c = dynamic_map[ny][nx];
    if (c == '#') return true;
    if (c == '$' || (c >= 'A' && c <= 'Z')) return true;
    return false;
}

static bool bfs_move_only_c(Pos2 start, Pos2 end, bool is_target_passable,
                            int* out_len, int* out_turns, char* out_path) {
    g_l2_recog_move_bfs_calls2++;
    /*
     * Mecanum movement optimization:
     *   1) keep the grid-step count shortest;
     *   2) among equally short paths, choose the one with fewer U/D/L/R changes.
     *
     * The state includes the last translation direction. This does NOT insert
     * Q/E turns when the translation direction changes; U/D/L/R remain direct
     * mecanum translations. Q/E/B are only used later to face an object.
     */
    enum { MOVE_LAST_NONE = 4, MOVE_STATE_COUNT = 12 * 16 * 5 };

    if (start.x == end.x && start.y == end.y) {
        *out_len = 0;
        *out_turns = 0;
        return true;
    }

    memset(move_parent_buf, 0xFF, sizeof(move_parent_buf));
    memset(move_dist_buf, 0xFF, sizeof(move_dist_buf));
    memset(move_turns_buf, 0xFF, sizeof(move_turns_buf));

    int head = 0, tail = 0;
    int start_cell = start.y * 16 + start.x;
    uint16_t start_state = (uint16_t)(start_cell * 5 + MOVE_LAST_NONE);
    move_queue_buf[tail++] = start_state;
    move_parent_buf[start_state] = start_state;
    move_dist_buf[start_state] = 0;
    move_turns_buf[start_state] = 0;

    static const int8_t dx[4] = {0, 0, -1, 1};
    static const int8_t dy[4] = {-1, 1, 0, 0};
    static const char dirs[4] = {'U', 'D', 'L', 'R'};

    uint16_t best_target_state = 0xFFFF;
    uint16_t best_target_dist = 0xFFFF;
    uint8_t best_target_turns = 0xFF;

    while (head < tail) {
        uint16_t curr = move_queue_buf[head++];
        uint8_t curr_dist = move_dist_buf[curr];
        if (best_target_dist != 0xFFFF && curr_dist > best_target_dist) break;

        int cell = curr / 5;
        int last_dir = curr % 5;
        int px = cell % 16;
        int py = cell / 16;

        if (px == end.x && py == end.y) {
            if (best_target_state == 0xFFFF || move_turns_buf[curr] < best_target_turns) {
                best_target_state = curr;
                best_target_dist = curr_dist;
                best_target_turns = move_turns_buf[curr];
            }
            continue;
        }

        for (int i = 0; i < 4; i++) {
            int nx = px + dx[i], ny = py + dy[i];
            if (is_obstacle(nx, ny, is_target_passable)) continue;

            int next_cell = ny * 16 + nx;
            uint16_t next_state = (uint16_t)(next_cell * 5 + i);
            uint8_t next_dist = (uint8_t)(curr_dist + 1);
            uint8_t next_turns = (uint8_t)(move_turns_buf[curr] +
                                  ((last_dir != MOVE_LAST_NONE && last_dir != i) ? 1 : 0));

            if (move_dist_buf[next_state] == 0xFF) {
                move_dist_buf[next_state] = next_dist;
                move_turns_buf[next_state] = next_turns;
                move_parent_buf[next_state] = curr;
                move_dir_buf[next_state] = dirs[i];
                if (tail < MOVE_STATE_COUNT) move_queue_buf[tail++] = next_state;
            } else if (move_dist_buf[next_state] == next_dist &&
                       next_turns < move_turns_buf[next_state]) {
                /* Same shortest length, but this predecessor makes a straighter path. */
                move_turns_buf[next_state] = next_turns;
                move_parent_buf[next_state] = curr;
                move_dir_buf[next_state] = dirs[i];
            }
        }
    }

    if (best_target_state == 0xFFFF) return false;

    int steps = 0;
    uint16_t c = best_target_state;
    while (c != start_state &&
           steps < SOKOBAN_LEVEL2_BFS_PATH_CAPACITY) {
        if (c == 0xFFFFU || move_parent_buf[c] == 0xFFFFU) return false;
        out_path[steps++] = move_dir_buf[c];
        c = move_parent_buf[c];
    }
    if (c != start_state) return false;

    for (int i = 0; i < steps / 2; i++) {
        char tmp = out_path[i];
        out_path[i] = out_path[steps - 1 - i];
        out_path[steps - 1 - i] = tmp;
    }
    *out_len = steps;
    *out_turns = best_target_turns;
    return true;
}


typedef struct { int type; int orig_idx; Pos2 p; char name; } TspObj;
static TspObj all_objs[SOKOBAN_LEVEL2_TSP_MAX_OBJECTS];
static int num_objs = 0;
static int global_best_cost = 999999;
static int global_best_turns = 999999;
static char global_best_path[1000];
static int global_best_len = 0;

static void get_rotation(char c_dir, char t_dir,
                         char* rot_cmds, int* rot_len, int* rot_cost);

static uint8_t level2_recog_popcount2(uint32_t mask2)
{
    uint8_t count2 = 0U;
    while (mask2 != 0U) {
        count2 = (uint8_t)(count2 + (uint8_t)(mask2 & 1U));
        mask2 >>= 1U;
    }
    return count2;
}

static uint16_t level2_recog_rotation_cost2(char from2, char to2,
                                             char *rot2, int *rot_len2)
{
    int rot_cost2 = 0;
    get_rotation(from2, to2, rot2, rot_len2, &rot_cost2);
    return (uint16_t)rot_cost2;
}

static uint32_t level2_recog_state_index2(uint32_t mask2, uint8_t pose2)
{
    const SokobanRecognitionPose3 *p2 = &level2_recog_ws2.poses[pose2];
    uint32_t lower_mask2 = mask2 & ((1u << p2->object_index) - 1u);
    uint32_t lower_pose_count2 =
        level2_recog_ws2.mask_base[lower_mask2 + 1u] -
        level2_recog_ws2.mask_base[lower_mask2];
    return level2_recog_ws2.mask_base[mask2] + lower_pose_count2 +
           p2->local_index;
}

/* Build shortest translation lengths from one source to every cell.  Pairwise
 * recognition costs only consume the length, so one 192-cell BFS can service
 * every destination pose instead of one BFS per source/destination pair. */
static void level2_recog_build_distance_map2(Pos2 start2)
{
    enum { CELL_COUNT2 = 12 * 16 };
    static const int8_t dx2[4] = {0, 0, -1, 1};
    static const int8_t dy2[4] = {-1, 1, 0, 0};
    uint16_t head2 = 0U, tail2 = 0U;
    uint8_t start_cell2 = (uint8_t)(start2.y * 16 + start2.x);

    g_l2_recog_move_bfs_calls2++;
    memset(move_dist_buf, 0xFF, CELL_COUNT2);
    move_dist_buf[start_cell2] = 0U;
    move_queue_buf[tail2++] = start_cell2;

    while (head2 < tail2) {
        uint8_t cell2 = (uint8_t)move_queue_buf[head2++];
        int x2 = cell2 & 0x0F;
        int y2 = cell2 >> 4;
        uint8_t next_distance2 = (uint8_t)(move_dist_buf[cell2] + 1U);
        for (int dir2 = 0; dir2 < 4; dir2++) {
            int nx2 = x2 + dx2[dir2];
            int ny2 = y2 + dy2[dir2];
            if (is_obstacle(nx2, ny2, true)) continue;
            uint8_t next2 = (uint8_t)(ny2 * 16 + nx2);
            if (move_dist_buf[next2] != 0xFFU) continue;
            move_dist_buf[next2] = next_distance2;
            move_queue_buf[tail2++] = next2;
        }
    }
}

static uint16_t level2_recog_cost_from_distance2(char from_dir2,
                                                  uint8_t to_pose2)
{
    const SokobanRecognitionPose3 *pose2 = &level2_recog_ws2.poses[to_pose2];
    uint8_t path_len2 = move_dist_buf[pose2->stand_cell];
    if (path_len2 == 0xFFU) return LEVEL2_RECOG_INF2;

    char rot2[3];
    int rot_len2 = 0;
    uint16_t rotation_cost2 = level2_recog_rotation_cost2(
        from_dir2, pose2->view_dir, rot2, &rot_len2);
    if (path_len2 == 0U && rot_len2 == 0) return 1U;
    return (uint16_t)path_len2 + rotation_cost2;
}

static bool level2_recog_build_poses2(void)
{
    static const int8_t dx2[4] = {0, 0, -1, 1};
    static const int8_t dy2[4] = {-1, 1, 0, 0};
    static const char view2[4] = {'D', 'U', 'R', 'L'};
    uint8_t total2 = 0U;

    if (num_objs <= 0 || num_objs > SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY) {
        g_l2_recog_result2 = -101;
        return false;
    }
    memset(level2_recog_ws2.object_pose_start, 0,
           sizeof(level2_recog_ws2.object_pose_start));
    memset(level2_recog_ws2.object_pose_count, 0,
           sizeof(level2_recog_ws2.object_pose_count));

    for (int object2 = 0; object2 < num_objs; object2++) {
        level2_recog_ws2.object_pose_start[object2] = total2;
        for (uint8_t local2 = 0U; local2 < 4U; local2++) {
            int sx2 = all_objs[object2].p.x + dx2[local2];
            int sy2 = all_objs[object2].p.y + dy2[local2];
            bool passable2 = all_objs[object2].type == 1;
            if (is_obstacle(sx2, sy2, passable2)) continue;
            if (total2 >= SOKOBAN_LEVEL3_RECOG_MAX_POSES) {
                g_l2_recog_result2 = -102;
                return false;
            }
            level2_recog_ws2.poses[total2].object_index = (uint8_t)object2;
            level2_recog_ws2.poses[total2].local_index =
                level2_recog_ws2.object_pose_count[object2];
            level2_recog_ws2.poses[total2].stand_cell =
                (uint8_t)(sy2 * 16 + sx2);
            level2_recog_ws2.poses[total2].view_dir = view2[local2];
            level2_recog_ws2.object_pose_count[object2]++;
            total2++;
        }
        if (level2_recog_ws2.object_pose_count[object2] == 0U) {
            g_l2_recog_result2 = -103;
            return false;
        }
    }
    g_l2_recog_pose_count2 = total2;
    return true;
}

static bool level2_recog_build_layout2(uint32_t full_mask2)
{
    uint32_t next_base2 = 0U;
    level2_recog_ws2.mask_base[0] = 0U;
    for (uint32_t mask2 = 0U; mask2 <= full_mask2; mask2++) {
        uint32_t count2 = 0U;
        for (int object2 = 0; object2 < num_objs; object2++) {
            if ((mask2 & (1u << object2)) != 0U)
                count2 += level2_recog_ws2.object_pose_count[object2];
        }
        next_base2 += count2;
        level2_recog_ws2.mask_base[mask2 + 1U] = next_base2;
    }
    if (next_base2 > SOKOBAN_LEVEL3_RECOG_MAX_STATES) {
        g_l2_recog_result2 = -104;
        return false;
    }
    g_l2_recog_compact_state_capacity2 = next_base2;
    memset(level2_recog_ws2.dp_cost, 0xFF,
           next_base2 * sizeof(level2_recog_ws2.dp_cost[0]));
    memset(level2_recog_ws2.dp_parent_pose, LEVEL2_RECOG_PARENT_NONE2,
           next_base2 * sizeof(level2_recog_ws2.dp_parent_pose[0]));
    return true;
}

static bool level2_recog_precompute2(Pos2 start2, char start_dir2)
{
    uint8_t pose_count2 = (uint8_t)g_l2_recog_pose_count2;

    level2_recog_build_distance_map2(start2);
    for (uint8_t to2 = 0U; to2 < pose_count2; to2++) {
        level2_recog_ws2.start_cost[to2] =
            level2_recog_cost_from_distance2(start_dir2, to2);
    }

    for (uint8_t from2 = 0U; from2 < pose_count2; from2++) {
        const SokobanRecognitionPose3 *src2 = &level2_recog_ws2.poses[from2];
        Pos2 src_pos2 = {(int)(src2->stand_cell & 0x0Fu),
                         (int)(src2->stand_cell >> 4)};
        level2_recog_build_distance_map2(src_pos2);
        for (uint8_t to2 = 0U; to2 < pose_count2; to2++) {
            if (src2->object_index == level2_recog_ws2.poses[to2].object_index) {
                level2_recog_ws2.edge_cost[from2][to2] = LEVEL2_RECOG_INF2;
            } else {
                level2_recog_ws2.edge_cost[from2][to2] =
                    level2_recog_cost_from_distance2(src2->view_dir, to2);
            }
        }
    }
    return true;
}

static bool level2_recog_rebuild2(Pos2 start2, char start_dir2,
                                   uint8_t sequence_len2)
{
    Pos2 current2 = start2;
    char current_dir2 = start_dir2;
    int output_len2 = 0;

    for (uint8_t step2 = 0U; step2 < sequence_len2; step2++) {
        uint8_t pose_index2 = level2_recog_ws2.chosen_pose_sequence[step2];
        const SokobanRecognitionPose3 *pose2 =
            &level2_recog_ws2.poses[pose_index2];
        Pos2 stand2 = {(int)(pose2->stand_cell & 0x0Fu),
                       (int)(pose2->stand_cell >> 4)};
        int path_len2 = 0, path_turns2 = 0;
        char rot2[3];
        int rot_len2 = 0;
        bool passable2 = all_objs[pose2->object_index].type == 1;

        if (!bfs_move_only_c(current2, stand2, passable2,
                             &path_len2, &path_turns2,
                             level2_recog_ws2.path_scratch)) {
            g_l2_recog_result2 = -105;
            return false;
        }
        (void)path_turns2;
        (void)level2_recog_rotation_cost2(current_dir2, pose2->view_dir,
                                           rot2, &rot_len2);
        int command_len2 = path_len2 + rot_len2;
        int append_len2 = command_len2 == 0 ? 1 : command_len2;
        if (output_len2 + append_len2 >= SOKOBAN_LEVEL2_INTERNAL_PATH_CAPACITY) {
            g_l2_recog_result2 = -106;
            return false;
        }
        if (command_len2 == 0) {
            global_best_path[output_len2++] = 'f';
        } else {
            if (path_len2 > 0) {
                memcpy(global_best_path + output_len2,
                       level2_recog_ws2.path_scratch, (size_t)path_len2);
                output_len2 += path_len2;
            }
            if (rot_len2 > 0) {
                memcpy(global_best_path + output_len2, rot2, (size_t)rot_len2);
                output_len2 += rot_len2;
            }
            char last2 = global_best_path[output_len2 - 1];
            if (last2 >= 'A' && last2 <= 'Z')
                global_best_path[output_len2 - 1] =
                    (char)(last2 + ('a' - 'A'));
        }
        current2 = stand2;
        current_dir2 = pose2->view_dir;
    }
    global_best_len = output_len2;
    global_best_cost = g_l2_recog_best_cost2;
    global_best_turns = 0;
    return true;
}

static bool level2_plan_recognition_dp2(Pos2 start2, char start_dir2,
                                         int required2)
{
    uint16_t best_cost2 = LEVEL2_RECOG_INF2;
    uint32_t best_mask2 = 0U;
    uint8_t best_pose2 = LEVEL2_RECOG_PARENT_NONE2;

    g_l2_recog_move_bfs_calls2 = 0U;
    g_l2_recog_dp_states2 = 0U;
    g_l2_recog_dp_transitions2 = 0U;
    g_l2_recog_pose_count2 = 0U;
    g_l2_recog_compact_state_capacity2 = 0U;
    g_l2_recog_best_cost2 = -1;
    g_l2_recog_result2 = 1;

    if (required2 <= 0 || required2 > num_objs ||
        num_objs > SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS) {
        g_l2_recog_result2 = -107;
        return false;
    }
    if (!level2_recog_build_poses2()) return false;
    uint32_t full_mask2 = (1u << num_objs) - 1u;
    if (!level2_recog_build_layout2(full_mask2)) return false;
    if (!level2_recog_precompute2(start2, start_dir2)) return false;
    uint8_t pose_count2 = (uint8_t)g_l2_recog_pose_count2;

    for (uint8_t pose2 = 0U; pose2 < pose_count2; pose2++) {
        uint16_t cost2 = level2_recog_ws2.start_cost[pose2];
        if (cost2 == LEVEL2_RECOG_INF2) continue;
        uint32_t mask2 = 1u << level2_recog_ws2.poses[pose2].object_index;
        uint32_t state2 = level2_recog_state_index2(mask2, pose2);
        if (cost2 < level2_recog_ws2.dp_cost[state2]) {
            level2_recog_ws2.dp_cost[state2] = cost2;
            level2_recog_ws2.dp_parent_pose[state2] = LEVEL2_RECOG_PARENT_START2;
            g_l2_recog_dp_states2++;
        }
    }

    for (uint32_t mask2 = 1U; mask2 <= full_mask2; mask2++) {
        uint8_t depth2 = level2_recog_popcount2(mask2);
        if (depth2 >= (uint8_t)required2) continue;
        for (uint8_t last2 = 0U; last2 < pose_count2; last2++) {
            uint8_t last_object2 = level2_recog_ws2.poses[last2].object_index;
            if ((mask2 & (1u << last_object2)) == 0U) continue;
            uint32_t state2 = level2_recog_state_index2(mask2, last2);
            uint16_t current2 = level2_recog_ws2.dp_cost[state2];
            if (current2 == LEVEL2_RECOG_INF2) continue;

            for (uint8_t next_object2 = 0U;
                 next_object2 < (uint8_t)num_objs; next_object2++) {
                uint32_t bit2 = 1u << next_object2;
                if ((mask2 & bit2) != 0U) continue;
                uint8_t first2 = level2_recog_ws2.object_pose_start[next_object2];
                uint8_t count2 = level2_recog_ws2.object_pose_count[next_object2];
                uint32_t next_mask2 = mask2 | bit2;
                uint32_t lower_mask2 =
                    mask2 & ((1u << next_object2) - 1u);
                uint32_t next_state_base2 = level2_recog_ws2.mask_base[next_mask2] +
                    (level2_recog_ws2.mask_base[lower_mask2 + 1u] -
                     level2_recog_ws2.mask_base[lower_mask2]);
                for (uint8_t local2 = 0U; local2 < count2; local2++) {
                    uint8_t next_pose2 = (uint8_t)(first2 + local2);
                    uint16_t edge2 = level2_recog_ws2.edge_cost[last2][next_pose2];
                    g_l2_recog_dp_transitions2++;
                    if (edge2 == LEVEL2_RECOG_INF2) continue;
                    uint32_t sum2 = (uint32_t)current2 + edge2;
                    if (sum2 >= LEVEL2_RECOG_INF2) continue;
                    uint32_t next_state2 = next_state_base2 + local2;
                    uint16_t old2 = level2_recog_ws2.dp_cost[next_state2];
                    if (sum2 < old2 ||
                        (sum2 == old2 && last2 <
                         level2_recog_ws2.dp_parent_pose[next_state2])) {
                        if (old2 == LEVEL2_RECOG_INF2) g_l2_recog_dp_states2++;
                        level2_recog_ws2.dp_cost[next_state2] = (uint16_t)sum2;
                        level2_recog_ws2.dp_parent_pose[next_state2] = last2;
                    }
                }
            }
        }
    }

    if (required2 == num_objs) {
        for (uint8_t pose2 = 0U; pose2 < pose_count2; pose2++) {
            uint16_t cost2 = level2_recog_ws2.dp_cost[
                level2_recog_state_index2(full_mask2, pose2)];
            if (cost2 < best_cost2) {
                best_cost2 = cost2; best_mask2 = full_mask2; best_pose2 = pose2;
            }
        }
    } else {
        for (uint8_t drop2 = 0U; drop2 < (uint8_t)num_objs; drop2++) {
            uint32_t mask2 = full_mask2 & ~(1u << drop2);
            for (uint8_t pose2 = 0U; pose2 < pose_count2; pose2++) {
                uint8_t object2 = level2_recog_ws2.poses[pose2].object_index;
                if ((mask2 & (1u << object2)) == 0U) continue;
                uint16_t cost2 = level2_recog_ws2.dp_cost[
                    level2_recog_state_index2(mask2, pose2)];
                if (cost2 < best_cost2) {
                    best_cost2 = cost2; best_mask2 = mask2; best_pose2 = pose2;
                }
            }
        }
    }
    if (best_pose2 == LEVEL2_RECOG_PARENT_NONE2 ||
        best_cost2 == LEVEL2_RECOG_INF2) {
        g_l2_recog_result2 = -109;
        return false;
    }

    uint32_t trace_mask2 = best_mask2;
    uint8_t trace_pose2 = best_pose2;
    for (int step2 = required2 - 1; step2 >= 0; step2--) {
        level2_recog_ws2.chosen_pose_sequence[step2] = trace_pose2;
        uint32_t state2 = level2_recog_state_index2(trace_mask2, trace_pose2);
        uint8_t parent2 = level2_recog_ws2.dp_parent_pose[state2];
        trace_mask2 &= ~(1u << level2_recog_ws2.poses[trace_pose2].object_index);
        if (step2 == 0) {
            if (parent2 != LEVEL2_RECOG_PARENT_START2 || trace_mask2 != 0U) {
                g_l2_recog_result2 = -111;
                return false;
            }
        } else {
            if (parent2 >= pose_count2) {
                g_l2_recog_result2 = -112;
                return false;
            }
            trace_pose2 = parent2;
        }
    }

    g_l2_recog_best_cost2 = best_cost2;
    if (!level2_recog_rebuild2(start2, start_dir2, (uint8_t)required2))
        return false;
    g_l2_recog_result2 = 0;
    return true;
}

/* Fixed-memory recognition fallback for scenes larger than the exact subset-DP
 * threshold.  Runtime object count comes from the current map; the compile-time
 * constants are storage capacities only.  We greedily choose the cheapest
 * currently reachable recognition pose, then rebuild every selected leg with
 * the same exact movement BFS used by the DP path. */
static bool level2_plan_recognition_scalable2(Pos2 start2, char start_dir2,
                                               int required2)
{
    uint32_t visited2 = 0U;
    Pos2 current2 = start2;
    char current_dir2 = start_dir2;
    uint32_t total_cost2 = 0U;

    g_l2_recog_move_bfs_calls2 = 0U;
    g_l2_recog_dp_states2 = 0U;
    g_l2_recog_dp_transitions2 = 0U;
    g_l2_recog_pose_count2 = 0U;
    g_l2_recog_compact_state_capacity2 = 0U;
    g_l2_recog_best_cost2 = -1;
    g_l2_recog_result2 = 1;

    if (required2 <= 0 || required2 > num_objs ||
        num_objs > SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY || num_objs > 32) {
        g_l2_recog_result2 = -113;
        return false;
    }
    if (!level2_recog_build_poses2()) return false;

    for (int step2 = 0; step2 < required2; step2++) {
        uint16_t best_cost2 = LEVEL2_RECOG_INF2;
        uint8_t best_pose2 = LEVEL2_RECOG_PARENT_NONE2;
        uint8_t pose_count2 = (uint8_t)g_l2_recog_pose_count2;

        level2_recog_build_distance_map2(current2);
        for (uint8_t pose2 = 0U; pose2 < pose_count2; pose2++) {
            uint8_t object2 = level2_recog_ws2.poses[pose2].object_index;
            uint32_t bit2 = 1u << object2;
            if ((visited2 & bit2) != 0U) continue;
            uint16_t cost2 = level2_recog_cost_from_distance2(current_dir2,
                                                               pose2);
            g_l2_recog_dp_transitions2++;
            if (cost2 < best_cost2 ||
                (cost2 == best_cost2 && pose2 < best_pose2)) {
                best_cost2 = cost2;
                best_pose2 = pose2;
            }
        }
        if (best_pose2 == LEVEL2_RECOG_PARENT_NONE2 ||
            best_cost2 == LEVEL2_RECOG_INF2) {
            g_l2_recog_result2 = -114;
            return false;
        }

        level2_recog_ws2.chosen_pose_sequence[step2] = best_pose2;
        const SokobanRecognitionPose3 *chosen2 =
            &level2_recog_ws2.poses[best_pose2];
        visited2 |= 1u << chosen2->object_index;
        current2.x = (int)(chosen2->stand_cell & 0x0Fu);
        current2.y = (int)(chosen2->stand_cell >> 4);
        current_dir2 = chosen2->view_dir;
        total_cost2 += best_cost2;
        g_l2_recog_dp_states2++;
    }

    if (total_cost2 > 0x7FFFFFFFU) total_cost2 = 0x7FFFFFFFU;
    g_l2_recog_best_cost2 = (int32_t)total_cost2;
    if (!level2_recog_rebuild2(start2, start_dir2, (uint8_t)required2))
        return false;
    /* 2 explicitly means bounded scalable mode, not exact subset DP. */
    g_l2_recog_result2 = 2;
    return true;
}

#define MAX_CACHE 600
typedef SokobanBfsCache2 BfsCache;
#define bfs_cache (g_sokoban_workspace.level2.stage.explore.planner.legacy.bfs_cache)
static int bfs_cache_cnt = 0;
static void get_rotation(char c_dir, char t_dir, char* rot_cmds, int* rot_len, int* rot_cost) {
    // rot_len  = actual command bytes written into the output path.
    // rot_cost = logical planning cost. A 180-degree turn B is one command byte,
    //            but it should be treated as two right turns E+E while searching.
    rot_cmds[0] = 0;
    rot_cmds[1] = 0;
    rot_cmds[2] = 0;
    if (c_dir == t_dir) { *rot_len = 0; *rot_cost = 0; return; }
    static const char dirs[4] = {'U', 'R', 'D', 'L'}; int c_idx = 0, t_idx = 0;
    for(int i=0; i<4; i++) { if(dirs[i] == c_dir) c_idx = i; if(dirs[i] == t_dir) t_idx = i; }
    int diff = (t_idx - c_idx + 4) % 4;
    if (diff == 1) { rot_cmds[0] = 'E'; *rot_len = 1; *rot_cost = 1; }
    else if (diff == 2) { rot_cmds[0] = 'B'; *rot_len = 1; *rot_cost = 2; }
    else if (diff == 3) { rot_cmds[0] = 'Q'; *rot_len = 1; *rot_cost = 1; }
}

static int get_cached_bfs_idx(Pos2 s, Pos2 e, int type) {
    for (int i = 0; i < bfs_cache_cnt; i++) {
        if (bfs_cache[i].sx == s.x && bfs_cache[i].sy == s.y &&
            bfs_cache[i].ex == e.x && bfs_cache[i].ey == e.y &&
            bfs_cache[i].type == type) return i;
    }

    int len = 0, turns = 0;
    char *path = bfs_build_path;
    bool is_target_passable = (type == 1);
    bool res = bfs_move_only_c(s, e, is_target_passable, &len, &turns, path);

    if (bfs_cache_cnt < MAX_CACHE) {
        BfsCache* cache = &bfs_cache[bfs_cache_cnt];
        cache->sx = (uint8_t)s.x; cache->sy = (uint8_t)s.y;
        cache->ex = (uint8_t)e.x; cache->ey = (uint8_t)e.y;
        cache->type = (uint8_t)type;
        cache->turns = 0;
        cache->first_dir = 0;
        cache->last_dir = 0;

        if (res && len <= (int)sizeof(cache->path)) {
            cache->len = len;
            cache->turns = (uint8_t)turns;
            if (len > 0) {
                cache->first_dir = path[0];
                cache->last_dir = path[len - 1];
                memcpy(cache->path, path, (size_t)len);
            }
        } else {
            /* Never overflow the fixed MCU cache. */
            cache->len = -1;
        }
        return bfs_cache_cnt++;
    }
    return -1;
}

typedef SokobanTspCandidate2 NextStep;

static uint8_t build_tsp_candidates2(const SokobanTspFrame2 *frame,
                                     NextStep *steps)
{
    static const int8_t dx[4] = {0, 0, -1, 1};
    static const int8_t dy[4] = {-1, 1, 0, 0};
    static const char view_dirs[4] = {'D', 'U', 'R', 'L'};
    Pos2 curr_p = {(int)(frame->curr_cell % 16U),
                   (int)(frame->curr_cell / 16U)};
    int step_cnt = 0;

    for (int i = 0; i < num_objs; i++) {
        if (!(frame->unvisited_mask & (1u << i))) continue;

        for (int d = 0; d < 4; d++) {
            int nx = all_objs[i].p.x + dx[d];
            int ny = all_objs[i].p.y + dy[d];
            if (is_obstacle(nx, ny, all_objs[i].type == 1)) continue;

            int c_idx = get_cached_bfs_idx(curr_p, (Pos2){nx, ny},
                                           all_objs[i].type);
            /* Keep the original call/check order: even after the candidate
             * array is full, cache population still occurs deterministically. */
            if (c_idx == -1 || bfs_cache[c_idx].len == -1 ||
                step_cnt >= SOKOBAN_LEVEL2_TSP_MAX_CANDIDATES) {
                continue;
            }

            char t_dir = view_dirs[d];
            char r_cmds[3];
            int r_len = 0;
            int r_cost = 0;
            get_rotation(frame->curr_dir, t_dir, r_cmds, &r_len, &r_cost);

            steps[step_cnt].cache_idx = (uint16_t)c_idx;
            steps[step_cnt].next_i = (uint8_t)i;
            steps[step_cnt].next_cell = (uint8_t)(ny * 16 + nx);
            steps[step_cnt].path_len = (uint8_t)bfs_cache[c_idx].len;
            steps[step_cnt].move_turns = bfs_cache[c_idx].turns;
            steps[step_cnt].first_move_dir = bfs_cache[c_idx].first_dir;
            steps[step_cnt].last_move_dir = bfs_cache[c_idx].last_dir;
            steps[step_cnt].target_dir = t_dir;
            steps[step_cnt].rot_cmd = r_cmds[0];
            steps[step_cnt].rot_len = (uint8_t)r_len;
            steps[step_cnt].rot_cost = (uint8_t)r_cost;
            step_cnt++;
        }
    }

    /* Same stable candidate ordering as the recursive implementation. */
    for (int a = 0; a < step_cnt - 1; a++) {
        for (int b = a + 1; b < step_cnt; b++) {
            int cost_a = steps[a].path_len + steps[a].rot_cost;
            int cost_b = steps[b].path_len + steps[b].rot_cost;
            int turn_a = steps[a].move_turns;
            int turn_b = steps[b].move_turns;
            if (frame->last_move_dir && steps[a].path_len > 0 &&
                frame->last_move_dir != steps[a].first_move_dir) {
                turn_a++;
            }
            if (frame->last_move_dir && steps[b].path_len > 0 &&
                frame->last_move_dir != steps[b].first_move_dir) {
                turn_b++;
            }

            if (cost_a > cost_b ||
                (cost_a == cost_b && turn_a > turn_b)) {
                NextStep tmp = steps[a];
                steps[a] = steps[b];
                steps[b] = tmp;
            }
        }
    }

    return (uint8_t)step_cnt;
}

/* Manual DFS frame machine replacing dfs_tsp().  The parent advances its
 * candidate index before the child is pushed, exactly like a recursive call
 * returning into the for-loop.  tsp_current_path is shared because each frame
 * owns a distinct [0,current_len) prefix. */
static void SOKOBAN_UNUSED_FN2 search_tsp_iterative2(Pos2 start_p,
                                  char start_dir,
                                  char start_last_move_dir,
                                  uint32_t start_mask)
{
    int top = 0;
    memset(tsp_stack, 0, sizeof(tsp_stack));

    tsp_stack[0].curr_cell = (uint8_t)(start_p.y * 16 + start_p.x);
    tsp_stack[0].curr_dir = start_dir;
    tsp_stack[0].last_move_dir = start_last_move_dir;
    tsp_stack[0].unvisited_mask = start_mask;

    while (top >= 0 && sokoban_error2 == SOKOBAN_OK2) {
        SokobanTspFrame2 *frame = &tsp_stack[top];

        if (!frame->entered) {
            tsp_search_cnt++;
            explore_search_nodes2++;
            if (tsp_search_cnt > 5000) {
                top--;
                continue;
            }

            if (frame->current_cost > global_best_cost ||
                (frame->current_cost == global_best_cost &&
                 frame->current_turns >= global_best_turns)) {
                top--;
                continue;
            }

            if (frame->unvisited_mask == 0U) {
                if (frame->current_cost < global_best_cost ||
                    (frame->current_cost == global_best_cost &&
                     frame->current_turns < global_best_turns)) {
                    global_best_cost = frame->current_cost;
                    global_best_turns = frame->current_turns;
                    global_best_len = frame->current_len;
                    memcpy(global_best_path, tsp_current_path,
                           (size_t)frame->current_len);
                }
                top--;
                continue;
            }

            frame->candidate_count =
                build_tsp_candidates2(frame, tsp_candidates[top]);
            frame->next_candidate = 0;
            frame->entered = 1;
            continue;
        }

        if (frame->next_candidate >= frame->candidate_count) {
            top--;
            continue;
        }

        const NextStep *step =
            &tsp_candidates[top][frame->next_candidate++];
        int p_len = step->path_len;
        int r_len = step->rot_len;
        int cmd_step_len = p_len + r_len;
        int logical_step_cost = p_len + step->rot_cost;
        int append_len = (cmd_step_len == 0) ? 1 : cmd_step_len;
        int append_cost = (cmd_step_len == 0) ? 1 : logical_step_cost;
        int added_turns = step->move_turns;

        if (p_len > 0 && frame->last_move_dir &&
            frame->last_move_dir != step->first_move_dir) {
            added_turns++;
        }
        char next_last_move_dir =
            (p_len > 0) ? step->last_move_dir : frame->last_move_dir;

        if ((int)frame->current_len + append_len >=
            SOKOBAN_LEVEL2_INTERNAL_PATH_CAPACITY) {
            tsp_path_capacity_hit2 = true;
            continue;
        }

        if (cmd_step_len > 0) {
            memcpy(tsp_current_path + frame->current_len,
                   bfs_cache[step->cache_idx].path, (size_t)p_len);
            if (r_len > 0) {
                tsp_current_path[frame->current_len + p_len] = step->rot_cmd;
            }

            int last_idx = frame->current_len + cmd_step_len - 1;
            char last_c = tsp_current_path[last_idx];
            if (last_c >= 'A' && last_c <= 'Z') {
                tsp_current_path[last_idx] = (char)(last_c + 32);
            }
        } else {
            tsp_current_path[frame->current_len] = 'f';
        }

        if (top + 1 >= SOKOBAN_LEVEL2_TSP_MAX_OBJECTS) {
            sokoban_error2 = SOKOBAN_ERROR_SEARCH_STACK2;
            return;
        }

        SokobanTspFrame2 *child = &tsp_stack[++top];
        memset(child, 0, sizeof(*child));
        child->curr_cell = step->next_cell;
        child->curr_dir = step->target_dir;
        child->last_move_dir = next_last_move_dir;
        child->unvisited_mask =
            frame->unvisited_mask & ~(1u << step->next_i);
        child->current_cost = frame->current_cost + append_cost;
        child->current_turns = frame->current_turns + added_turns;
        child->current_len = (uint16_t)(frame->current_len + append_len);
    }
}

// ============================================================================
// ??????? 1???????????j?h???
// ============================================================================
void plan_explore_phase2(char external_map[12][16]) {
    memcpy(dynamic_map, external_map, sizeof(dynamic_map));
    explore_steps2 = 0;
    explore_commands2[0] = '\0';
    expected_cnt2 = 0;
    memset(expected_targets2, 0, sizeof(expected_targets2));
    sokoban_error2 = SOKOBAN_OK2;
    explore_search_nodes2 = 0;
    tsp_path_capacity_hit2 = false;
    g_l2_recog_move_bfs_calls2 = 0U;
    g_l2_recog_dp_states2 = 0U;
    g_l2_recog_dp_transitions2 = 0U;
    g_l2_recog_pose_count2 = 0U;
    g_l2_recog_compact_state_capacity2 = 0U;
    g_l2_recog_best_cost2 = -1;
    g_l2_recog_result2 = 0;

    Pos2 cur_p = {0, 0};
    char cur_dir = 'U';
    bool player_found = false;

    num_objs = 0;
    bfs_cache_cnt = 0;
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            if (dynamic_map[y][x] == '@') {
                cur_p = (Pos2){x, y};
                dynamic_map[y][x] = '-';
                player_found = true;
            } else if (dynamic_map[y][x] == '$' ||
                       dynamic_map[y][x] == '.') {
                if (num_objs >= SOKOBAN_LEVEL2_TSP_MAX_OBJECTS) {
                    sokoban_error2 = SOKOBAN_ERROR_INVALID_MAP2;
                    return;
                }
                int type = (dynamic_map[y][x] == '.') ? 1 : 0;
                all_objs[num_objs] =
                    (TspObj){type, num_objs, (Pos2){x, y}, dynamic_map[y][x]};
                num_objs++;
            }
        }
    }

    if (!player_found || num_objs <= 0 || num_objs > 32) {
        sokoban_error2 = SOKOBAN_ERROR_INVALID_MAP2;
        return;
    }

    global_best_cost = 999999;
    global_best_turns = 999999;
    global_best_len = 0;
#if LEVEL2_USE_RECOGNITION_DP2
    bool recognition_ok2;
    if (num_objs <= SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS) {
        recognition_ok2 = level2_plan_recognition_dp2(cur_p, cur_dir,
                                                       num_objs - 1);
    } else {
        recognition_ok2 = level2_plan_recognition_scalable2(cur_p, cur_dir,
                                                             num_objs - 1);
    }
    if (!recognition_ok2) {
        if (g_l2_recog_result2 == -106)
            sokoban_error2 = SOKOBAN_ERROR_EXPLORE_PATH_CAPACITY2;
        else
            sokoban_error2 = SOKOBAN_ERROR_INVALID_MAP2;
        return;
    }
    explore_search_nodes2 = g_l2_recog_dp_states2;
#else
    for (int drop = 0; drop < num_objs; drop++) {
        uint32_t init_mask = 0;
        for (int i = 0; i < num_objs; i++) {
            if (i != drop) init_mask |= (1u << i);
        }
        tsp_search_cnt = 0;
        search_tsp_iterative2(cur_p, cur_dir, 0, init_mask);
        if (sokoban_error2 != SOKOBAN_OK2) return;
    }
#endif

    if (global_best_cost == 999999) {
        if (tsp_path_capacity_hit2) {
            sokoban_error2 = SOKOBAN_ERROR_EXPLORE_PATH_CAPACITY2;
            return;
        }
        explore_end_pos = cur_p;
        return;
    }

    if (global_best_len < 0 ||
        global_best_len >= (int)sizeof(explore_commands2)) {
        sokoban_error2 = SOKOBAN_ERROR_EXPLORE_PATH_CAPACITY2;
        return;
    }

    int snap_count = 0;
    for (int k = 0; k < global_best_len; k++) {
        if (global_best_path[k] >= 'a' && global_best_path[k] <= 'z') {
            snap_count++;
        }
    }
    if (snap_count > (int)(sizeof(expected_targets2) /
                           sizeof(expected_targets2[0]))) {
        sokoban_error2 = SOKOBAN_ERROR_EXPECTED_TARGET_CAPACITY2;
        return;
    }

    memcpy(explore_commands2, global_best_path, (size_t)global_best_len);
    explore_steps2 = global_best_len;
    explore_commands2[explore_steps2] = '\0';

    for (int k = 0; k < explore_steps2; k++) {
        char cmd = explore_commands2[k];
        bool is_snap = false;
        if (cmd >= 'a' && cmd <= 'z') {
            is_snap = true;
            cmd = (char)(cmd - 32);
        }

        if (cmd == 'U') cur_p.y--;
        else if (cmd == 'D') cur_p.y++;
        else if (cmd == 'L') cur_p.x--;
        else if (cmd == 'R') cur_p.x++;
        else if (cmd == 'E') {
            if (cur_dir == 'U') cur_dir = 'R';
            else if (cur_dir == 'R') cur_dir = 'D';
            else if (cur_dir == 'D') cur_dir = 'L';
            else if (cur_dir == 'L') cur_dir = 'U';
        } else if (cmd == 'Q') {
            if (cur_dir == 'U') cur_dir = 'L';
            else if (cur_dir == 'L') cur_dir = 'D';
            else if (cur_dir == 'D') cur_dir = 'R';
            else if (cur_dir == 'R') cur_dir = 'U';
        } else if (cmd == 'B') {
            if (cur_dir == 'U') cur_dir = 'D';
            else if (cur_dir == 'R') cur_dir = 'L';
            else if (cur_dir == 'D') cur_dir = 'U';
            else if (cur_dir == 'L') cur_dir = 'R';
        }

        if (is_snap) {
            int tx = cur_p.x;
            int ty = cur_p.y;
            if (cur_dir == 'U') ty--;
            else if (cur_dir == 'D') ty++;
            else if (cur_dir == 'L') tx--;
            else if (cur_dir == 'R') tx++;
            expected_targets2[expected_cnt2++] = (Pos2){tx, ty};
        }
    }

    explore_end_pos = cur_p;
}

// ============================================================================
// Push phase: weighted A* over the player and ALL labelled boxes
// ============================================================================
/*
 * The previous implementation solved one selected box at a time and converted
 * every other box into a wall.  That misses valid solutions which temporarily
 * move another box out of the way.  The state below contains the player and all
 * boxes, so every legal push can move any box and boxes may be repositioned
 * until they reach their matching lower-case targets; on a match they disappear.
 *
 * Matching rule is unchanged: 'A' -> 'a', 'B' -> 'b', ...
 * Equal-labelled boxes are interchangeable; their positions are sorted to
 * avoid exploring duplicate permutations.
 */

#define PUSH_W                    16
#define PUSH_H                    12
#define PUSH_CELL_COUNT           (PUSH_W * PUSH_H)
#define PUSH_MAX_NODES            12000
#define PUSH_HASH_SIZE            32768
#define PUSH_NO_NODE              0xFFFFu
#define PUSH_PATH_LIMIT           SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY
#define PUSH_ACTIVE_NONE           0xFFu
#define PUSH_BOX_REMOVED           0xFFu

#define PUSH_PLAN_NONE2             0
#define PUSH_PLAN_SAFE_DIRECT2      1
#define PUSH_PLAN_COOPERATIVE2      2

#define DUP_BEAM_WIDTH2             128
#define DUP_MAX_PUSH_ACTIONS2       (MAX_BOXES2 * 16)

typedef struct {
    uint8_t player;
    uint8_t boxes[MAX_BOXES2];
    SokobanBoxMask used_target_mask;
    uint16_t cost;
    uint16_t score;
    uint8_t action_count;
    uint8_t actions[DUP_MAX_PUSH_ACTIONS2];
} DuplicateBeamState2;

/* The legacy A* node pool is much larger than the bounded duplicate beam.
 * Reuse its storage rather than allocating another MCU-sized static arena. */
#define dup_beam_current2 \
    ((DuplicateBeamState2 *)(void *)&push_nodes[0])
#define dup_beam_next2 \
    ((DuplicateBeamState2 *)(void *)((uint8_t *)(void *)&push_nodes[0] + \
        DUP_BEAM_WIDTH2 * sizeof(DuplicateBeamState2)))

typedef char duplicate_beam_fits_legacy_pool2[
    (2U * DUP_BEAM_WIDTH2 * sizeof(DuplicateBeamState2) <= sizeof(push_nodes))
        ? 1 : -1];

/* First search favours route quality.  Later attempts are greedier fallbacks
 * for hard maps which would otherwise exhaust PUSH_MAX_NODES. */
/* Ordinary A*: each fixed one-box-at-a-time order is solved for its shortest
 * executable route within the fixed MCU node pool.  We intentionally do not
 * accept a greedier approximate route, because the selected box order must be
 * compared by true path length rather than by a heuristic fallback. */
static const uint8_t push_search_weights[] = {1};
/* Duplicate-element fallback uses the cooperative A* below.  Greedier
 * weights are tried first so ten-box repeated-label scenes stay within the
 * fixed 12k-node MCU pool; weight 1 remains the completeness/quality fallback
 * when the pool is sufficient. */
static const uint8_t duplicate_push_search_weights[] = {4, 6, 2, 1};

static const int8_t push_dx[4] = {0, 0, -1, 1};
static const int8_t push_dy[4] = {-1, 1, 0, 0};
static const char push_dir_char[4] = {'U', 'D', 'L', 'R'};

typedef SokobanPushNode2 PushNode;

static bool safe_direct_mode_supported(void);

static uint16_t push_node_count;
static uint16_t push_heap_count;


static uint8_t push_cell(int x, int y)
{
    return (uint8_t)(y * PUSH_W + x);
}

static int push_cell_x(uint8_t cell)
{
    return cell % PUSH_W;
}

static int push_cell_y(uint8_t cell)
{
    return cell / PUSH_W;
}

static bool push_inside(int x, int y)
{
    return x >= 0 && x < PUSH_W && y >= 0 && y < PUSH_H;
}

static void sort_dynamic_objects(void)
{
    /* Keep box labels in a deterministic order. */
    for (int i = 0; i < num_dyn_boxes - 1; i++) {
        for (int j = i + 1; j < num_dyn_boxes; j++) {
            int pi = dyn_boxes[i].pos.y * PUSH_W + dyn_boxes[i].pos.x;
            int pj = dyn_boxes[j].pos.y * PUSH_W + dyn_boxes[j].pos.x;
            if (dyn_boxes[j].label < dyn_boxes[i].label ||
                (dyn_boxes[j].label == dyn_boxes[i].label && pj < pi)) {
                DynObj2 tmp = dyn_boxes[i];
                dyn_boxes[i] = dyn_boxes[j];
                dyn_boxes[j] = tmp;
            }
        }
    }

    for (int i = 0; i < num_dyn_targets - 1; i++) {
        for (int j = i + 1; j < num_dyn_targets; j++) {
            int pi = dyn_targets[i].pos.y * PUSH_W + dyn_targets[i].pos.x;
            int pj = dyn_targets[j].pos.y * PUSH_W + dyn_targets[j].pos.x;
            if (dyn_targets[j].label < dyn_targets[i].label ||
                (dyn_targets[j].label == dyn_targets[i].label && pj < pi)) {
                DynObj2 tmp = dyn_targets[i];
                dyn_targets[i] = dyn_targets[j];
                dyn_targets[j] = tmp;
            }
        }
    }
}

static void sort_equal_label_box_positions(uint8_t boxes[MAX_BOXES2])
{
    int begin = 0;
    while (begin < num_dyn_boxes) {
        int end = begin + 1;
        while (end < num_dyn_boxes &&
               dyn_boxes[end].label == dyn_boxes[begin].label) {
            end++;
        }

        for (int i = begin; i < end - 1; i++) {
            for (int j = i + 1; j < end; j++) {
                if (boxes[j] < boxes[i]) {
                    uint8_t tmp = boxes[i];
                    boxes[i] = boxes[j];
                    boxes[j] = tmp;
                }
            }
        }
        begin = end;
    }
}

static bool box_matches_target_cell(int box_idx, uint8_t cell)
{
    if (cell == PUSH_BOX_REMOVED) return false;

    char wanted = (char)(dyn_boxes[box_idx].label + ('a' - 'A'));
    int x = push_cell_x(cell);
    int y = push_cell_y(cell);

    for (int t = 0; t < num_dyn_targets; t++) {
        if (dyn_targets[t].label == wanted &&
            dyn_targets[t].pos.x == x &&
            dyn_targets[t].pos.y == y) {
            return true;
        }
    }
    return false;
}

static int box_target_index_at_cell(int box_idx, uint8_t cell)
{
    if (cell == PUSH_BOX_REMOVED) return -1;

    char wanted = (char)(dyn_boxes[box_idx].label + ('a' - 'A'));
    int x = push_cell_x(cell);
    int y = push_cell_y(cell);

    for (int t = 0; t < num_dyn_targets; t++) {
        if (dyn_targets[t].label == wanted &&
            dyn_targets[t].pos.x == x &&
            dyn_targets[t].pos.y == y) {
            return t;
        }
    }
    return -1;
}

static bool has_duplicate_box_labels2(void)
{
    for (int i = 1; i < num_dyn_boxes; i++) {
        if (dyn_boxes[i - 1].label == dyn_boxes[i].label) return true;
    }
    return false;
}

/* Reverse-push distances with walls only.  Besides providing a heuristic,
 * 0xFF marks cells from which that labelled box can never reach any matching
 * target even if all other boxes are removed.  Pruning such cells is safe. */
static bool build_static_box_distances(void)
{
    for (int b = 0; b < num_dyn_boxes; b++) {
        memset(push_static_dist[b], 0xFF, PUSH_CELL_COUNT);

        int head = 0;
        int tail = 0;
        char wanted = (char)(dyn_boxes[b].label + ('a' - 'A'));

        for (int t = 0; t < num_dyn_targets; t++) {
            if (dyn_targets[t].label != wanted) continue;
            uint8_t c = push_cell(dyn_targets[t].pos.x,
                                  dyn_targets[t].pos.y);
            if (push_static_dist[b][c] == 0xFF) {
                push_static_dist[b][c] = 0;
                push_walk_queue[tail++] = c;
            }
        }

        if (tail == 0) return false;

        while (head < tail) {
            uint8_t curr = push_walk_queue[head++];
            int cx = push_cell_x(curr);
            int cy = push_cell_y(curr);

            for (int d = 0; d < 4; d++) {
                /* A forward push would move prev -> curr. */
                int px = cx - push_dx[d];
                int py = cy - push_dy[d];
                int sx = px - push_dx[d];
                int sy = py - push_dy[d];

                if (!push_inside(px, py) || !push_inside(sx, sy)) continue;

                uint8_t prev = push_cell(px, py);
                uint8_t support = push_cell(sx, sy);
                if (push_wall[prev] || push_wall[support]) continue;
                if (push_static_dist[b][prev] != 0xFF) continue;

                push_static_dist[b][prev] =
                    (uint8_t)(push_static_dist[b][curr] + 1u);
                push_walk_queue[tail++] = prev;
            }
        }
    }
    return true;
}

/* Duplicate mode needs to know which *physical target* remains available.
 * Rebuild the same reverse-push table indexed by target object rather than by
 * box.  This overwrites push_static_dist only after strict mode has already
 * been rejected for a repeated-label map, so no extra 10x192 buffer is needed. */
static void build_duplicate_target_distances2(void)
{
    for (int t = 0; t < num_dyn_targets; t++) {
        memset(push_static_dist[t], 0xFF, PUSH_CELL_COUNT);
        int head = 0;
        int tail = 0;
        uint8_t target = push_cell(dyn_targets[t].pos.x,
                                   dyn_targets[t].pos.y);
        push_static_dist[t][target] = 0U;
        push_walk_queue[tail++] = target;

        while (head < tail) {
            uint8_t curr = push_walk_queue[head++];
            int cx = push_cell_x(curr);
            int cy = push_cell_y(curr);

            for (int d = 0; d < 4; d++) {
                int px = cx - push_dx[d];
                int py = cy - push_dy[d];
                int sx = px - push_dx[d];
                int sy = py - push_dy[d];
                if (!push_inside(px, py) || !push_inside(sx, sy)) continue;

                uint8_t prev = push_cell(px, py);
                uint8_t support = push_cell(sx, sy);
                if (push_wall[prev] || push_wall[support]) continue;
                if (push_static_dist[t][prev] != 0xFFU) continue;
                push_static_dist[t][prev] =
                    (uint8_t)(push_static_dist[t][curr] + 1U);
                push_walk_queue[tail++] = prev;
            }
        }
    }
}

static uint8_t duplicate_box_min_target_distance2(
    int box_index,
    uint8_t cell,
    SokobanBoxMask used_target_mask)
{
    if (cell == PUSH_BOX_REMOVED) return 0U;
    char wanted = (char)(dyn_boxes[box_index].label + ('a' - 'A'));
    uint8_t best = 0xFFU;
    for (int t = 0; t < num_dyn_targets; t++) {
        if (dyn_targets[t].label != wanted) continue;
        SokobanBoxMask bit = SOKOBAN_BOX_MASK_BIT(t);
        if ((used_target_mask & bit) != 0U) continue;
        uint8_t d = push_static_dist[t][cell];
        if (d < best) best = d;
    }
    return best;
}

static int duplicate_heuristic2(const uint8_t boxes[MAX_BOXES2],
                                SokobanBoxMask used_target_mask)
{
    int sum = 0;
    for (int i = 0; i < num_dyn_boxes; i++) {
        if (boxes[i] == PUSH_BOX_REMOVED) continue;
        uint8_t d = duplicate_box_min_target_distance2(
            i, boxes[i], used_target_mask);
        if (d == 0xFFU) return 10000;
        sum += d;
    }
    return sum;
}

static bool duplicate_deadlock2(const uint8_t boxes[MAX_BOXES2],
                                SokobanBoxMask used_target_mask)
{
    for (int i = 0; i < num_dyn_boxes; i++) {
        uint8_t cell = boxes[i];
        if (cell == PUSH_BOX_REMOVED) continue;
        if (duplicate_box_min_target_distance2(i, cell,
                                               used_target_mask) == 0xFFU) {
            return true;
        }

        int x = push_cell_x(cell);
        int y = push_cell_y(cell);
        bool up = (y <= 0) || push_wall[push_cell(x, y - 1)];
        bool down = (y >= PUSH_H - 1) || push_wall[push_cell(x, y + 1)];
        bool left = (x <= 0) || push_wall[push_cell(x - 1, y)];
        bool right = (x >= PUSH_W - 1) || push_wall[push_cell(x + 1, y)];
        if ((up || down) && (left || right)) {
            int target_index = box_target_index_at_cell(i, cell);
            if (target_index < 0 ||
                (used_target_mask & SOKOBAN_BOX_MASK_BIT(target_index)) != 0U) {
                return true;
            }
        }
    }
    return false;
}

static uint16_t push_heuristic(const uint8_t boxes[MAX_BOXES2])
{
    uint16_t h = 0;
    for (int i = 0; i < num_dyn_boxes; i++) {
        if (boxes[i] == PUSH_BOX_REMOVED) continue;
        uint8_t d = push_static_dist[i][boxes[i]];
        if (d == 0xFF) return 0xFFFFu;
        h = (uint16_t)(h + d);
    }
    return h;
}

static bool push_goal(const uint8_t boxes[MAX_BOXES2])
{
    /* A correctly matched box falls into its pit and disappears. */
    for (int i = 0; i < num_dyn_boxes; i++) {
        if (boxes[i] != PUSH_BOX_REMOVED) return false;
    }
    return true;
}


#define STRICT_DP_INF2 0xFFFFu
#define STRICT_TRANS_STATE_COUNT2 SOKOBAN_LEVEL2_STRICT_TRANSITION_STATES
#define STRICT_DP_STATE_COUNT2 SOKOBAN_LEVEL2_STRICT_DP_STATES

static uint32_t strict_dp_index2(uint8_t mask2, uint8_t player2)
{
    return (uint32_t)mask2 * PUSH_CELL_COUNT + player2;
}

static uint16_t strict_transition_state2(uint8_t box2, uint8_t player2)
{
    return (uint16_t)((uint16_t)box2 * PUSH_CELL_COUNT + player2);
}

/* Queue entries use a byte-packed key so hot-path decode is shifts/masks.
 * The distance table remains compact (192*192) and is indexed separately. */
static uint16_t strict_transition_key2(uint8_t box2, uint8_t player2)
{
    return (uint16_t)(((uint16_t)box2 << 8) | player2);
}

static uint8_t strict_transition_box2(uint16_t state2)
{
    return (uint8_t)(state2 / PUSH_CELL_COUNT);
}

static uint8_t strict_transition_player2(uint16_t state2)
{
    return (uint8_t)(state2 % PUSH_CELL_COUNT);
}

static bool strict_prepare_targets2(void)
{
    for (int box2 = 0; box2 < num_dyn_boxes; box2++) {
        char wanted2 = (char)(dyn_boxes[box2].label + ('a' - 'A'));
        int found2 = -1;
        for (int target2 = 0; target2 < num_dyn_targets; target2++) {
            if (dyn_targets[target2].label != wanted2) continue;
            if (found2 >= 0) return false;
            found2 = target2;
        }
        if (found2 < 0) return false;
        strict_target_cell2[box2] = push_cell(dyn_targets[found2].pos.x,
                                               dyn_targets[found2].pos.y);
    }
    return true;
}

static void strict_build_fixed_boxes2(SokobanBoxMask completed_mask2,
                                      uint8_t active_box2)
{
    /* Fold immutable walls into the hot occupancy map once per transition
     * group so neighbor tests need a single byte load. */
    memcpy(push_occupied, push_wall, PUSH_CELL_COUNT);
    for (int box2 = 0; box2 < num_dyn_boxes; box2++) {
        if (box2 == active_box2) continue;
        if ((completed_mask2 & SOKOBAN_BOX_MASK_BIT(box2)) != 0U) continue;
        uint8_t cell2 = push_cell(dyn_boxes[box2].pos.x,
                                  dyn_boxes[box2].pos.y);
        push_occupied[cell2] = 1U;
    }
}

static bool strict_cell_free2(int x2, int y2)
{
    if (!push_inside(x2, y2)) return false;
    return push_occupied[push_cell(x2, y2)] == 0U;
}

static uint8_t strict_neighbor_cell2(uint8_t cell2, int dir2)
{
    if (dir2 == 0) return cell2 >= 16U ? (uint8_t)(cell2 - 16U) : 0xFFU;
    if (dir2 == 1) return cell2 < 176U ? (uint8_t)(cell2 + 16U) : 0xFFU;
    if (dir2 == 2) return (cell2 & 0x0FU) != 0U ? (uint8_t)(cell2 - 1U) : 0xFFU;
    return (cell2 & 0x0FU) != 15U ? (uint8_t)(cell2 + 1U) : 0xFFU;
}

/* Expand one strict transition. Only active_box2 may move; every other
 * unfinished box is a fixed obstacle. Reaching the matching target removes
 * the active box and updates all possible terminal player cells. */
/* Solve all starts for one (completed-mask, active-box) transition in
 * reverse.  A forward strict search was previously repeated once for every
 * reachable start-player cell.  Here each possible final push direction runs
 * one reverse BFS and updates every current DP player state it reaches. */
static void strict_expand_box_transitions2(uint8_t completed_mask2,
                                           uint8_t active_box2)
{
    uint8_t start_box2 = push_cell(dyn_boxes[active_box2].pos.x,
                                   dyn_boxes[active_box2].pos.y);
    uint8_t target2 = strict_target_cell2[active_box2];
    uint8_t next_mask2 = (uint8_t)(completed_mask2 |
                                   (uint8_t)(1u << active_box2));
    uint32_t current_dp_base2 = strict_dp_index2(completed_mask2, 0U);
    int tx2 = push_cell_x(target2);
    int ty2 = push_cell_y(target2);
    uint16_t needed_start_count2 = 0U;

    strict_build_fixed_boxes2(completed_mask2, active_box2);
    for (uint16_t player2 = 0U; player2 < PUSH_CELL_COUNT; player2++) {
        if (strict_dp_cost2[current_dp_base2 + player2] == STRICT_DP_INF2)
            continue;
        if (push_occupied[player2] || player2 == start_box2) continue;
        needed_start_count2++;
    }
    if (needed_start_count2 == 0U) return;

    for (int final_dir2 = 0; final_dir2 < 4; final_dir2++) {
        int end_bx2 = tx2 - push_dx[final_dir2];
        int end_by2 = ty2 - push_dy[final_dir2];
        int end_px2 = end_bx2 - push_dx[final_dir2];
        int end_py2 = end_by2 - push_dy[final_dir2];
        if (!strict_cell_free2(end_bx2, end_by2) ||
            !strict_cell_free2(end_px2, end_py2)) {
            continue;
        }

        uint8_t end_box2 = push_cell(end_bx2, end_by2);
        uint8_t end_player_before_push2 = push_cell(end_px2, end_py2);
        uint8_t terminal_player2 = end_box2;
        uint32_t next_dp_index2 = strict_dp_index2(next_mask2,
                                                   terminal_player2);
        uint32_t head2 = 0U, tail2 = 0U;
        uint16_t source_index2 = strict_transition_state2(
            end_box2, end_player_before_push2);
        uint16_t starts_left2 = needed_start_count2;

        strict_plan_dist2[source_index2] = 1U; /* include the final push */
        strict_plan_queue2[tail2++] = strict_transition_key2(
            end_box2, end_player_before_push2);
        g_l2_strict_transition_searches2++;

        while (head2 < tail2 && starts_left2 != 0U) {
            uint16_t key2 = strict_plan_queue2[head2++];
            uint8_t box_cell2 = (uint8_t)(key2 >> 8);
            uint8_t player_cell2 = (uint8_t)key2;
            uint16_t state_index2 = strict_transition_state2(box_cell2,
                                                             player_cell2);
            uint16_t distance2 = strict_plan_dist2[state_index2];
            g_l2_strict_transition_nodes2++;
            push_search_nodes2++;

            if (box_cell2 == start_box2) {
                uint16_t base2 = strict_dp_cost2[current_dp_base2 +
                                                 player_cell2];
                if (base2 != STRICT_DP_INF2) {
                    uint32_t total2 = (uint32_t)base2 + distance2;
                    if (total2 < STRICT_DP_INF2) {
                        uint16_t old2 = strict_dp_cost2[next_dp_index2];
                        uint8_t old_box2 = strict_dp_parent_box2[next_dp_index2];
                        uint8_t old_player2 =
                            strict_dp_parent_player2[next_dp_index2];
                        if (total2 < old2 ||
                            (total2 == old2 &&
                             (active_box2 < old_box2 ||
                              (active_box2 == old_box2 &&
                               player_cell2 < old_player2)))) {
                            if (old2 == STRICT_DP_INF2)
                                g_l2_strict_dp_states2++;
                            strict_dp_cost2[next_dp_index2] = (uint16_t)total2;
                            strict_dp_parent_player2[next_dp_index2] =
                                player_cell2;
                            strict_dp_parent_box2[next_dp_index2] = active_box2;
                        }
                    }
                    starts_left2--;
                }
            }

            /* Reverse ordinary player walks.  Because current player != box,
             * the corresponding forward step does not push. */
            for (int dir2 = 0; dir2 < 4; dir2++) {
                uint8_t prev_player2 = strict_neighbor_cell2(player_cell2, dir2);
                if (prev_player2 == 0xFFU || push_occupied[prev_player2] ||
                    prev_player2 == box_cell2) continue;
                uint16_t prev_index2 = strict_transition_state2(
                    box_cell2, prev_player2);
                if (strict_plan_dist2[prev_index2] != STRICT_DP_INF2) continue;
                strict_plan_dist2[prev_index2] = (uint16_t)(distance2 + 1U);
                strict_plan_queue2[tail2++] = strict_transition_key2(
                    box_cell2, prev_player2);
            }

            /* Reverse a push using linear cell deltas; no x/y division is
             * needed in this 16-column map. */
            int delta2 = (int)box_cell2 - (int)player_cell2;
            bool adjacent2 = (delta2 == 16 || delta2 == -16);
            if (delta2 == 1 || delta2 == -1) {
                adjacent2 = ((box_cell2 >> 4) == (player_cell2 >> 4));
            }
            if (adjacent2 &&
                push_static_dist[active_box2][box_cell2] != 0xFFU) {
                int old_player_i2 = (int)player_cell2 - delta2;
                if (old_player_i2 >= 0 && old_player_i2 < PUSH_CELL_COUNT) {
                    uint8_t old_player2 = (uint8_t)old_player_i2;
                    uint8_t old_box2 = player_cell2;
                    if (!push_occupied[old_player2] &&
                        old_player2 != old_box2 &&
                        ((delta2 != 1 && delta2 != -1) ||
                         ((old_player2 >> 4) == (old_box2 >> 4)))) {
                        uint16_t prev_index2 = strict_transition_state2(
                            old_box2, old_player2);
                        if (strict_plan_dist2[prev_index2] == STRICT_DP_INF2) {
                            strict_plan_dist2[prev_index2] =
                                (uint16_t)(distance2 + 1U);
                            strict_plan_queue2[tail2++] = strict_transition_key2(
                                old_box2, old_player2);
                        }
                    }
                }
            }
        }

        for (uint32_t i2 = 0U; i2 < tail2; i2++) {
            uint16_t key2 = strict_plan_queue2[i2];
            strict_plan_dist2[strict_transition_state2(
                (uint8_t)(key2 >> 8), (uint8_t)key2)] = STRICT_DP_INF2;
        }
    }
}

static int strict_rebuild_transition2(SokobanBoxMask completed_mask2,
                                      uint8_t start_player2,
                                      uint8_t active_box2,
                                      uint8_t wanted_end_player2,
                                      char *out2,
                                      int capacity2);

/* One shortest strict transition for the scalable (> exact-DP threshold)
 * planner.  Only active_box2 may move; every other unfinished box remains a
 * fixed obstacle.  The queue doubles as the touched-state list, so the 36,864
 * distance table is restored without a full memset after every transition. */
static uint16_t strict_find_transition_cost2(SokobanBoxMask completed_mask2,
                                              uint8_t start_player2,
                                              uint8_t active_box2,
                                              uint8_t *end_player2)
{
    uint8_t start_box2 = push_cell(dyn_boxes[active_box2].pos.x,
                                   dyn_boxes[active_box2].pos.y);
    uint8_t target2 = strict_target_cell2[active_box2];
    uint32_t head2 = 0U, tail2 = 0U;
    uint16_t answer2 = STRICT_DP_INF2;

    strict_build_fixed_boxes2(completed_mask2, active_box2);
    if (push_occupied[start_player2] || start_player2 == start_box2) return answer2;

    uint16_t start_state2 = strict_transition_state2(start_box2, start_player2);
    strict_plan_dist2[start_state2] = 0U;
    strict_plan_queue2[tail2++] = strict_transition_key2(start_box2, start_player2);
    g_l2_strict_transition_searches2++;

    while (head2 < tail2) {
        uint16_t key2 = strict_plan_queue2[head2++];
        uint8_t box_cell2 = (uint8_t)(key2 >> 8);
        uint8_t player_cell2 = (uint8_t)key2;
        uint16_t state2 = strict_transition_state2(box_cell2, player_cell2);
        uint16_t distance2 = strict_plan_dist2[state2];
        g_l2_strict_transition_nodes2++;
        push_search_nodes2++;

        for (int dir2 = 0; dir2 < 4; dir2++) {
            uint8_t next_player2 = strict_neighbor_cell2(player_cell2, dir2);
            if (next_player2 == 0xFFU || push_occupied[next_player2]) continue;
            uint8_t next_box2 = box_cell2;

            if (next_player2 == box_cell2) {
                uint8_t front2 = strict_neighbor_cell2(box_cell2, dir2);
                if (front2 == 0xFFU || push_occupied[front2]) continue;
                if (front2 == target2) {
                    answer2 = (uint16_t)(distance2 + 1U);
                    *end_player2 = box_cell2;
                    goto strict_transition_done2;
                }
                if (push_static_dist[active_box2][front2] == 0xFFU) continue;
                next_box2 = front2;
                next_player2 = box_cell2;
            }

            uint16_t next_state2 = strict_transition_state2(next_box2,
                                                            next_player2);
            if (strict_plan_dist2[next_state2] != STRICT_DP_INF2) continue;
            strict_plan_dist2[next_state2] = (uint16_t)(distance2 + 1U);
            strict_plan_queue2[tail2++] = strict_transition_key2(next_box2,
                                                                 next_player2);
        }
    }

strict_transition_done2:
    for (uint32_t i2 = 0U; i2 < tail2; i2++) {
        uint16_t key2 = strict_plan_queue2[i2];
        strict_plan_dist2[strict_transition_state2((uint8_t)(key2 >> 8),
                                                   (uint8_t)key2)] = STRICT_DP_INF2;
    }
    return answer2;
}

static uint16_t strict_beam_lower_bound2(SokobanBoxMask completed_mask2)
{
    uint32_t sum2 = 0U;
    for (uint8_t box2 = 0U; box2 < (uint8_t)num_dyn_boxes; box2++) {
        if ((completed_mask2 & SOKOBAN_BOX_MASK_BIT(box2)) != 0U) continue;
        uint8_t start_box2 = push_cell(dyn_boxes[box2].pos.x,
                                       dyn_boxes[box2].pos.y);
        uint8_t d2 = push_static_dist[box2][start_box2];
        if (d2 == 0xFFU) return STRICT_DP_INF2;
        sum2 += d2;
    }
    return sum2 >= STRICT_DP_INF2 ? (uint16_t)(STRICT_DP_INF2 - 1U)
                                  : (uint16_t)sum2;
}

static bool strict_beam_better2(const SokobanStrictBeamState2 *a2,
                                const SokobanStrictBeamState2 *b2)
{
    if (a2->score != b2->score) return a2->score < b2->score;
    if (a2->cost != b2->cost) return a2->cost < b2->cost;
    if (a2->completed_mask != b2->completed_mask)
        return a2->completed_mask < b2->completed_mask;
    return a2->player < b2->player;
}

static void strict_beam_keep_candidate2(const SokobanStrictBeamState2 *candidate2,
                                         uint8_t *count2)
{
    for (uint8_t i2 = 0U; i2 < *count2; i2++) {
        if (strict_beam_next2[i2].completed_mask != candidate2->completed_mask ||
            strict_beam_next2[i2].player != candidate2->player) continue;
        if (candidate2->cost < strict_beam_next2[i2].cost ||
            (candidate2->cost == strict_beam_next2[i2].cost &&
             strict_beam_better2(candidate2, &strict_beam_next2[i2]))) {
            strict_beam_next2[i2] = *candidate2;
        }
        return;
    }

    if (*count2 < SOKOBAN_LEVEL2_STRICT_BEAM_WIDTH) {
        strict_beam_next2[*count2] = *candidate2;
        (*count2)++;
        return;
    }

    uint8_t worst2 = 0U;
    for (uint8_t i2 = 1U; i2 < *count2; i2++) {
        if (strict_beam_better2(&strict_beam_next2[worst2],
                                &strict_beam_next2[i2])) {
            worst2 = i2;
        }
    }
    if (strict_beam_better2(candidate2, &strict_beam_next2[worst2]))
        strict_beam_next2[worst2] = *candidate2;
}

static bool find_best_strict_beam_plan2(uint8_t start_player2)
{
    if (!safe_direct_mode_supported() || !strict_prepare_targets2()) return false;
    if (num_dyn_boxes <= SOKOBAN_LEVEL2_STRICT_EXACT_MAX_BOXES ||
        num_dyn_boxes > MAX_BOXES2) return false;

    memset(strict_plan_dist2, 0xFF, sizeof(strict_plan_dist2));
    memset(strict_beam_current2, 0, sizeof(strict_beam_current2));
    memset(strict_beam_next2, 0, sizeof(strict_beam_next2));
    g_l2_strict_dp_states2 = 1U;
    g_l2_strict_transition_searches2 = 0U;
    g_l2_strict_transition_nodes2 = 0U;
    g_l2_strict_best_cost2 = -1;
    g_l2_strict_result2 = 1;

    uint8_t current_count2 = 1U;
    strict_beam_current2[0].completed_mask = 0U;
    strict_beam_current2[0].cost = 0U;
    strict_beam_current2[0].player = start_player2;
    strict_beam_current2[0].score = strict_beam_lower_bound2(0U);

    for (uint8_t depth2 = 0U; depth2 < (uint8_t)num_dyn_boxes; depth2++) {
        uint8_t next_count2 = 0U;
        memset(strict_beam_next2, 0, sizeof(strict_beam_next2));

        for (uint8_t state_i2 = 0U; state_i2 < current_count2; state_i2++) {
            const SokobanStrictBeamState2 *state2 = &strict_beam_current2[state_i2];
            for (uint8_t box2 = 0U; box2 < (uint8_t)num_dyn_boxes; box2++) {
                SokobanBoxMask bit2 = SOKOBAN_BOX_MASK_BIT(box2);
                if ((state2->completed_mask & bit2) != 0U) continue;
                uint8_t end_player2 = 0xFFU;
                uint16_t transition2 = strict_find_transition_cost2(
                    state2->completed_mask, state2->player, box2, &end_player2);
                if (transition2 == STRICT_DP_INF2) continue;
                uint32_t total2 = (uint32_t)state2->cost + transition2;
                if (total2 >= STRICT_DP_INF2) continue;

                SokobanStrictBeamState2 candidate2 = *state2;
                candidate2.completed_mask = (SokobanBoxMask)(state2->completed_mask | bit2);
                candidate2.cost = (uint16_t)total2;
                candidate2.player = end_player2;
                candidate2.order[depth2] = box2;
                candidate2.start_player[depth2] = state2->player;
                candidate2.end_player[depth2] = end_player2;
                uint16_t lower2 = strict_beam_lower_bound2(candidate2.completed_mask);
                uint32_t score2 = total2 + lower2;
                candidate2.score = score2 >= STRICT_DP_INF2
                                 ? (uint16_t)(STRICT_DP_INF2 - 1U)
                                 : (uint16_t)score2;
                strict_beam_keep_candidate2(&candidate2, &next_count2);
            }
        }

        if (next_count2 == 0U) {
            g_l2_strict_result2 = -21;
            return false;
        }
        g_l2_strict_dp_states2 += next_count2;
        memcpy(strict_beam_current2, strict_beam_next2,
               (size_t)next_count2 * sizeof(strict_beam_current2[0]));
        current_count2 = next_count2;
    }

    uint8_t best_i2 = 0U;
    for (uint8_t i2 = 1U; i2 < current_count2; i2++) {
        if (strict_beam_current2[i2].cost < strict_beam_current2[best_i2].cost ||
            (strict_beam_current2[i2].cost == strict_beam_current2[best_i2].cost &&
             strict_beam_better2(&strict_beam_current2[i2],
                                 &strict_beam_current2[best_i2]))) {
            best_i2 = i2;
        }
    }
    const SokobanStrictBeamState2 *best2 = &strict_beam_current2[best_i2];
    for (uint8_t depth2 = 0U; depth2 < (uint8_t)num_dyn_boxes; depth2++) {
        strict_chosen_box2[depth2] = best2->order[depth2];
        strict_chosen_start2[depth2] = best2->start_player[depth2];
        strict_chosen_end2[depth2] = best2->end_player[depth2];
    }

    int output_len2 = 0;
    SokobanBoxMask completed2 = 0U;
    for (uint8_t depth2 = 0U; depth2 < (uint8_t)num_dyn_boxes; depth2++) {
        uint8_t box2 = strict_chosen_box2[depth2];
        int length2 = strict_rebuild_transition2(
            completed2, strict_chosen_start2[depth2], box2,
            strict_chosen_end2[depth2], strict_work_commands2 + output_len2,
            SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY - output_len2);
        if (length2 == -2) {
            sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
            g_l2_strict_result2 = -22;
            return false;
        }
        if (length2 < 0) {
            g_l2_strict_result2 = -23;
            return false;
        }
        output_len2 += length2;
        completed2 = (SokobanBoxMask)(completed2 | SOKOBAN_BOX_MASK_BIT(box2));
        push_safe_order2[depth2] = dyn_boxes[box2].label;
    }

    if (output_len2 >= SOKOBAN_LEVEL2_OUTPUT_CAPACITY) {
        sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
        g_l2_strict_result2 = -24;
        return false;
    }
    strict_work_commands2[output_len2] = '\0';
    memcpy(push_commands2, strict_work_commands2, (size_t)output_len2 + 1U);
    push_steps2 = output_len2;
    push_safe_order2[num_dyn_boxes] = '\0';
    g_l2_strict_best_cost2 = best2->cost;
    g_l2_strict_result2 = 2;
    return true;
}

static int strict_rebuild_transition2(SokobanBoxMask completed_mask2,
                                      uint8_t start_player2,
                                      uint8_t active_box2,
                                      uint8_t wanted_end_player2,
                                      char *out2,
                                      int capacity2)
{
    uint8_t start_box2 = push_cell(dyn_boxes[active_box2].pos.x,
                                   dyn_boxes[active_box2].pos.y);
    uint8_t target2 = strict_target_cell2[active_box2];
    uint16_t final_state2 = 0xFFFFU;
    char final_move2 = 0;

    strict_build_fixed_boxes2(completed_mask2, active_box2);
    memset(strict_rebuild_parent2, 0xFF, sizeof(strict_rebuild_parent2));
    uint32_t head2 = 0U;
    uint32_t tail2 = 0U;
    uint16_t start_state2 = strict_transition_state2(start_box2,
                                                     start_player2);
    strict_rebuild_parent2[start_state2] = start_state2;
    strict_rebuild_queue2[tail2++] = start_state2;

    while (head2 < tail2 && final_state2 == 0xFFFFU) {
        uint16_t state2 = strict_rebuild_queue2[head2++];
        uint8_t box_cell2 = strict_transition_box2(state2);
        uint8_t player_cell2 = strict_transition_player2(state2);
        int px2 = push_cell_x(player_cell2);
        int py2 = push_cell_y(player_cell2);
        int bx2 = push_cell_x(box_cell2);
        int by2 = push_cell_y(box_cell2);

        for (int dir2 = 0; dir2 < 4; dir2++) {
            int nx2 = px2 + push_dx[dir2];
            int ny2 = py2 + push_dy[dir2];
            if (!strict_cell_free2(nx2, ny2)) continue;
            uint8_t next_player2 = push_cell(nx2, ny2);
            uint8_t next_box2 = box_cell2;

            if (next_player2 == box_cell2) {
                int fx2 = bx2 + push_dx[dir2];
                int fy2 = by2 + push_dy[dir2];
                if (!strict_cell_free2(fx2, fy2)) continue;
                uint8_t front2 = push_cell(fx2, fy2);
                if (front2 != target2 &&
                    push_static_dist[active_box2][front2] == 0xFFU) {
                    continue;
                }
                if (front2 == target2) {
                    if (box_cell2 == wanted_end_player2) {
                        final_state2 = state2;
                        final_move2 = push_dir_char[dir2];
                        break;
                    }
                    continue;
                }
                next_box2 = front2;
                next_player2 = box_cell2;
            }

            uint16_t next_state2 = strict_transition_state2(next_box2,
                                                            next_player2);
            if (strict_rebuild_parent2[next_state2] != 0xFFFFU) continue;
            strict_rebuild_parent2[next_state2] = state2;
            strict_rebuild_move2[next_state2] = push_dir_char[dir2];
            if (tail2 < STRICT_TRANS_STATE_COUNT2) {
                strict_rebuild_queue2[tail2++] = next_state2;
            }
        }
    }

    if (final_state2 == 0xFFFFU) return -1;
    int reverse_len2 = 0;
    uint16_t trace2 = final_state2;
    while (trace2 != start_state2) {
        if (reverse_len2 >= SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY - 1) return -2;
        strict_reverse_path2[reverse_len2++] = strict_rebuild_move2[trace2];
        trace2 = strict_rebuild_parent2[trace2];
        if (trace2 == 0xFFFFU) return -1;
    }
    if (reverse_len2 + 1 > capacity2) return -2;
    for (int i2 = 0; i2 < reverse_len2; i2++) {
        out2[i2] = strict_reverse_path2[reverse_len2 - 1 - i2];
    }
    out2[reverse_len2] = final_move2;
    return reverse_len2 + 1;
}

static bool find_best_strict_subset_plan2(uint8_t start_player2)
{
    if (num_dyn_boxes > SOKOBAN_LEVEL2_STRICT_EXACT_MAX_BOXES) {
        g_l2_strict_result2 = -20;
        return false;
    }
    if (!safe_direct_mode_supported()) return false;
    if (!strict_prepare_targets2()) return false;

    memset(strict_dp_cost2, 0xFF, sizeof(strict_dp_cost2));
    memset(strict_dp_parent_player2, 0xFF,
           sizeof(strict_dp_parent_player2));
    memset(strict_dp_parent_box2, 0xFF, sizeof(strict_dp_parent_box2));
    memset(strict_plan_dist2, 0xFF, sizeof(strict_plan_dist2));
    g_l2_strict_dp_states2 = 1U;
    g_l2_strict_transition_searches2 = 0U;
    g_l2_strict_transition_nodes2 = 0U;
    g_l2_strict_best_cost2 = -1;
    g_l2_strict_result2 = 1;

    uint8_t full_mask2 = (uint8_t)((1u << num_dyn_boxes) - 1u);
    strict_dp_cost2[strict_dp_index2(0U, start_player2)] = 0U;

    for (uint8_t mask2 = 0U; mask2 < full_mask2; mask2++) {
        bool has_state2 = false;
        uint32_t base_index2 = strict_dp_index2(mask2, 0U);
        for (uint16_t player2 = 0U; player2 < PUSH_CELL_COUNT; player2++) {
            if (strict_dp_cost2[base_index2 + player2] != STRICT_DP_INF2) {
                has_state2 = true;
                break;
            }
        }
        if (!has_state2) continue;
        for (uint8_t box2 = 0U; box2 < (uint8_t)num_dyn_boxes; box2++) {
            if ((mask2 & (uint8_t)(1u << box2)) != 0U) continue;
            strict_expand_box_transitions2(mask2, box2);
        }
    }

    uint16_t best_cost2 = STRICT_DP_INF2;
    uint8_t best_player2 = 0xFFU;
    for (uint16_t player2 = 0U; player2 < PUSH_CELL_COUNT; player2++) {
        uint16_t cost2 = strict_dp_cost2[
            strict_dp_index2(full_mask2, (uint8_t)player2)];
        if (cost2 < best_cost2) {
            best_cost2 = cost2;
            best_player2 = (uint8_t)player2;
        }
    }
    if (best_player2 == 0xFFU || best_cost2 == STRICT_DP_INF2) {
        g_l2_strict_result2 = -1;
        return false;
    }

    uint8_t trace_mask2 = full_mask2;
    uint8_t trace_player2 = best_player2;
    for (int depth2 = num_dyn_boxes - 1; depth2 >= 0; depth2--) {
        uint32_t index2 = strict_dp_index2(trace_mask2, trace_player2);
        uint8_t box2 = strict_dp_parent_box2[index2];
        uint8_t previous_player2 = strict_dp_parent_player2[index2];
        if (box2 >= (uint8_t)num_dyn_boxes || previous_player2 == 0xFFU) {
            g_l2_strict_result2 = -2;
            return false;
        }
        strict_chosen_box2[depth2] = box2;
        strict_chosen_start2[depth2] = previous_player2;
        strict_chosen_end2[depth2] = trace_player2;
        trace_mask2 = (uint8_t)(trace_mask2 &
                      (uint8_t)~(uint8_t)(1u << box2));
        trace_player2 = previous_player2;
    }
    if (trace_mask2 != 0U || trace_player2 != start_player2) {
        g_l2_strict_result2 = -3;
        return false;
    }

    int output_len2 = 0;
    uint8_t completed2 = 0U;
    for (int depth2 = 0; depth2 < num_dyn_boxes; depth2++) {
        uint8_t box2 = strict_chosen_box2[depth2];
        int length2 = strict_rebuild_transition2(
            completed2,
            strict_chosen_start2[depth2],
            box2,
            strict_chosen_end2[depth2],
            strict_work_commands2 + output_len2,
            SOKOBAN_LEVEL2_PUSH_PATH_CAPACITY - output_len2);
        if (length2 == -2) {
            sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
            g_l2_strict_result2 = -4;
            return false;
        }
        if (length2 < 0) {
            g_l2_strict_result2 = -5;
            return false;
        }
        output_len2 += length2;
        completed2 = (uint8_t)(completed2 | (uint8_t)(1u << box2));
        push_safe_order2[depth2] = dyn_boxes[box2].label;
    }

    if (output_len2 >= SOKOBAN_LEVEL2_OUTPUT_CAPACITY) {
        sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
        g_l2_strict_result2 = -6;
        return false;
    }
    strict_work_commands2[output_len2] = '\0';
    memcpy(push_commands2, strict_work_commands2, (size_t)output_len2 + 1U);
    push_steps2 = output_len2;
    push_safe_order2[num_dyn_boxes] = '\0';
    g_l2_strict_best_cost2 = best_cost2;
    g_l2_strict_result2 = 0;
    return true;
}

static uint32_t push_state_hash(uint8_t player,
                                const uint8_t boxes[MAX_BOXES2],
                                SokobanBoxMask active_box)
{
    uint32_t h = 2166136261u;
    h = (h ^ player) * 16777619u;
    h = (h ^ (uint8_t)active_box) * 16777619u;
    h = (h ^ (uint8_t)(active_box >> 8)) * 16777619u;
    for (int i = 0; i < num_dyn_boxes; i++) {
        h = (h ^ boxes[i]) * 16777619u;
    }
    return h;
}

static bool push_same_state(uint16_t node_idx,
                            uint8_t player,
                            const uint8_t boxes[MAX_BOXES2],
                            SokobanBoxMask active_box)
{
    const PushNode *n = &push_nodes[node_idx];
    if (n->player != player || n->active_box != active_box) return false;
    return memcmp(n->boxes, boxes, (size_t)num_dyn_boxes) == 0;
}

/* Returns an existing node, or PUSH_NO_NODE and the empty insertion slot. */
static uint16_t push_hash_lookup(uint8_t player,
                                 const uint8_t boxes[MAX_BOXES2],
                                 SokobanBoxMask active_box,
                                 uint16_t *out_slot)
{
    uint32_t slot = push_state_hash(player, boxes, active_box) & (PUSH_HASH_SIZE - 1u);

    for (uint32_t probe = 0; probe < PUSH_HASH_SIZE; probe++) {
        uint16_t entry = push_hash[slot];
        if (entry == 0) {
            *out_slot = (uint16_t)slot;
            return PUSH_NO_NODE;
        }

        uint16_t idx = (uint16_t)(entry - 1u);
        if (push_same_state(idx, player, boxes, active_box)) {
            *out_slot = (uint16_t)slot;
            return idx;
        }
        slot = (slot + 1u) & (PUSH_HASH_SIZE - 1u);
    }

    *out_slot = PUSH_NO_NODE;
    return PUSH_NO_NODE;
}

static bool push_heap_better(uint16_t a, uint16_t b)
{
    if (push_nodes[a].f_cost != push_nodes[b].f_cost) {
        return push_nodes[a].f_cost < push_nodes[b].f_cost;
    }
    /* On equal f, prefer the state which has made more real progress. */
    return push_nodes[a].g_cost > push_nodes[b].g_cost;
}

static void push_heap_swap(uint16_t a, uint16_t b)
{
    uint16_t ia = push_heap[a];
    uint16_t ib = push_heap[b];
    push_heap[a] = ib;
    push_heap[b] = ia;
    push_nodes[ia].heap_pos = b;
    push_nodes[ib].heap_pos = a;
}

static void push_heap_sift_up(uint16_t pos)
{
    while (pos > 0) {
        uint16_t parent = (uint16_t)((pos - 1u) / 2u);
        if (!push_heap_better(push_heap[pos], push_heap[parent])) break;
        push_heap_swap(pos, parent);
        pos = parent;
    }
}

static void push_heap_sift_down(uint16_t pos)
{
    for (;;) {
        uint16_t left = (uint16_t)(pos * 2u + 1u);
        if (left >= push_heap_count) break;

        uint16_t best = left;
        uint16_t right = (uint16_t)(left + 1u);
        if (right < push_heap_count &&
            push_heap_better(push_heap[right], push_heap[left])) {
            best = right;
        }

        if (!push_heap_better(push_heap[best], push_heap[pos])) break;
        push_heap_swap(pos, best);
        pos = best;
    }
}

static void push_heap_insert(uint16_t node_idx)
{
    if (push_heap_count >= PUSH_MAX_NODES) return;
    uint16_t pos = push_heap_count++;
    push_heap[pos] = node_idx;
    push_nodes[node_idx].heap_pos = pos;
    push_heap_sift_up(pos);
}

static uint16_t push_heap_pop(void)
{
    if (push_heap_count == 0) return PUSH_NO_NODE;

    uint16_t result = push_heap[0];
    push_nodes[result].heap_pos = PUSH_NO_NODE;
    push_heap_count--;

    if (push_heap_count > 0) {
        push_heap[0] = push_heap[push_heap_count];
        push_nodes[push_heap[0]].heap_pos = 0;
        push_heap_sift_down(0);
    }
    return result;
}

static void build_player_reachability(uint8_t start,
                                      const uint8_t boxes[MAX_BOXES2])
{
    memset(push_walk_dist, 0xFF, PUSH_CELL_COUNT);
    memset(push_occupied, 0, PUSH_CELL_COUNT);
    for (int i = 0; i < num_dyn_boxes; i++) {
        if (boxes[i] != PUSH_BOX_REMOVED) {
            push_occupied[boxes[i]] = 1;
        }
    }

    int head = 0;
    int tail = 0;
    push_walk_dist[start] = 0;
    push_walk_queue[tail++] = start;

    while (head < tail) {
        uint8_t curr = push_walk_queue[head++];
        int x = push_cell_x(curr);
        int y = push_cell_y(curr);

        for (int d = 0; d < 4; d++) {
            int nx = x + push_dx[d];
            int ny = y + push_dy[d];
            if (!push_inside(nx, ny)) continue;

            uint8_t next = push_cell(nx, ny);
            if (push_wall[next] || push_occupied[next]) continue;
            if (push_walk_dist[next] != 0xFF) continue;

            push_walk_dist[next] = (uint8_t)(push_walk_dist[curr] + 1u);
            push_walk_queue[tail++] = next;
        }
    }
}

static uint16_t weighted_f_cost(uint16_t g, uint16_t h, uint8_t weight)
{
    uint32_t f = (uint32_t)g + (uint32_t)weight * h;
    return (f > 65535u) ? 65535u : (uint16_t)f;
}

/* Returns goal node, PUSH_NO_NODE if exhausted, or PUSH_NO_NODE after hitting
 * the node limit.  Caller retries with a greedier weight when needed. */
static uint16_t run_push_search(uint8_t start_player,
                                const uint8_t start_boxes[MAX_BOXES2],
                                uint8_t weight,
                                bool safe_direct_mode,
                                const uint8_t forced_order[MAX_BOXES2])
{
    memset(push_hash, 0, sizeof(push_hash));
    push_node_count = 0;
    push_heap_count = 0;

    uint16_t h0 = push_heuristic(start_boxes);
    if (h0 == 0xFFFFu) return PUSH_NO_NODE;

    PushNode *root = &push_nodes[0];
    memset(root, 0, sizeof(*root));
    root->parent = PUSH_NO_NODE;
    root->g_cost = 0;
    root->f_cost = weighted_f_cost(0, h0, weight);
    root->heap_pos = PUSH_NO_NODE;
    root->player = start_player;
    memcpy(root->boxes, start_boxes, (size_t)num_dyn_boxes);
    root->move_box = 0xFF;
    root->move_dir = 0xFF;
    /* active_box is an overloaded 16-bit auxiliary state:
     *   strict legacy search -> currently active box index / 0xFF;
     *   cooperative duplicate search -> mask of already consumed targets.
     * The mode is fixed for one run_push_search() invocation, so the two
     * interpretations never coexist in one hash table. */
    root->active_box = safe_direct_mode ? PUSH_ACTIVE_NONE : 0U;
    root->closed = 0;

    uint16_t slot;
    (void)push_hash_lookup(root->player, root->boxes,
                           root->active_box, &slot);
    if (slot == PUSH_NO_NODE) return PUSH_NO_NODE;
    push_hash[slot] = 1;
    push_node_count = 1;
    push_heap_insert(0);

    while (push_heap_count > 0) {
        uint16_t curr_idx = push_heap_pop();
        if (curr_idx == PUSH_NO_NODE) break;

        PushNode *curr = &push_nodes[curr_idx];
        if (curr->closed) continue;
        curr->closed = 1;

        if (push_goal(curr->boxes)) return curr_idx;

        build_player_reachability(curr->player, curr->boxes);

        for (int b = 0; b < num_dyn_boxes; b++) {
            uint8_t box_cell_pos = curr->boxes[b];

            if (safe_direct_mode) {
                /* Once one box starts moving, finish it before touching another. */
                if (curr->active_box != PUSH_ACTIVE_NONE) {
                    if (b != curr->active_box) continue;
                } else {
                    /* Explicit permutation mode: only the first unfinished box
                     * in forced_order may be selected. This tests A->B->C,
                     * A->C->B, ... as genuinely separate plans. */
                    int required_box = -1;
                    if (forced_order != NULL) {
                        for (int oi = 0; oi < num_dyn_boxes; oi++) {
                            int candidate = forced_order[oi];
                            if (curr->boxes[candidate] != PUSH_BOX_REMOVED) {
                                required_box = candidate;
                                break;
                            }
                        }
                    }

                    if (required_box >= 0) {
                        if (b != required_box) continue;
                    } else {
                        continue;
                    }
                }
            }

            /* A completed box has fallen into its pit and no longer occupies
             * any cell, so it cannot be selected or block later routes. */
            if (box_cell_pos == PUSH_BOX_REMOVED) continue;

            int bx = push_cell_x(box_cell_pos);
            int by = push_cell_y(box_cell_pos);

            for (int d = 0; d < 4; d++) {
                int fx = bx + push_dx[d];
                int fy = by + push_dy[d];
                int hx = bx - push_dx[d];
                int hy = by - push_dy[d];

                if (!push_inside(fx, fy) || !push_inside(hx, hy)) continue;

                uint8_t front = push_cell(fx, fy);
                uint8_t behind = push_cell(hx, hy);
                if (push_wall[front] || push_occupied[front]) continue;
                if (push_walk_dist[behind] == 0xFF) continue;

                /* Static dead-square pruning for this label. */
                if (push_static_dist[b][front] == 0xFF) continue;

                uint16_t new_g = (uint16_t)(curr->g_cost +
                                           push_walk_dist[behind] + 1u);
                if (new_g >= PUSH_PATH_LIMIT) continue;

                uint8_t next_boxes[MAX_BOXES2];
                memcpy(next_boxes, curr->boxes, (size_t)num_dyn_boxes);
                SokobanBoxMask next_active = curr->active_box;
                if (safe_direct_mode) {
                    if (box_matches_target_cell(b, front)) {
                        next_boxes[b] = PUSH_BOX_REMOVED;
                    } else {
                        next_boxes[b] = front;
                    }
                } else {
                    /* Repeated elements need target identity even though box
                     * identity is canonicalised away.  A matching target may
                     * consume exactly one physical box; once its bit is set,
                     * that cell behaves as ordinary floor for later boxes. */
                    int target_index = box_target_index_at_cell(b, front);
                    if (target_index >= 0) {
                        SokobanBoxMask target_bit =
                            SOKOBAN_BOX_MASK_BIT(target_index);
                        if ((curr->active_box & target_bit) == 0U) {
                            next_boxes[b] = PUSH_BOX_REMOVED;
                            next_active = (SokobanBoxMask)(
                                curr->active_box | target_bit);
                        } else {
                            next_boxes[b] = front;
                        }
                    } else {
                        next_boxes[b] = front;
                    }
                }
                sort_equal_label_box_positions(next_boxes);

                uint16_t h = push_heuristic(next_boxes);
                if (h == 0xFFFFu) continue;

                uint8_t next_player = box_cell_pos;
                if (safe_direct_mode) {
                    next_active = (next_boxes[b] == PUSH_BOX_REMOVED)
                                      ? PUSH_ACTIVE_NONE
                                      : (SokobanBoxMask)b;
                }

                uint16_t hash_slot;
                uint16_t next_idx =
                    push_hash_lookup(next_player, next_boxes,
                                     next_active, &hash_slot);

                if (next_idx == PUSH_NO_NODE) {
                    if (hash_slot == PUSH_NO_NODE ||
                        push_node_count >= PUSH_MAX_NODES) {
                        continue;
                    }

                    next_idx = push_node_count++;
                    PushNode *next = &push_nodes[next_idx];
                    memset(next, 0, sizeof(*next));
                    next->parent = curr_idx;
                    next->g_cost = new_g;
                    next->f_cost = weighted_f_cost(new_g, h, weight);
                    next->heap_pos = PUSH_NO_NODE;
                    next->player = next_player;
                    memcpy(next->boxes, next_boxes,
                           (size_t)num_dyn_boxes);
                    next->move_box = (uint8_t)b;
                    next->move_dir = (uint8_t)d;
                    next->active_box = next_active;
                    next->closed = 0;

                    push_hash[hash_slot] = (uint16_t)(next_idx + 1u);
                    push_heap_insert(next_idx);
                } else {
                    PushNode *next = &push_nodes[next_idx];
                    if (new_g >= next->g_cost) continue;

                    next->parent = curr_idx;
                    next->g_cost = new_g;
                    next->f_cost = weighted_f_cost(new_g, h, weight);
                    next->move_box = (uint8_t)b;
                    next->move_dir = (uint8_t)d;
                    next->active_box = next_active;

                    if (next->closed) {
                        next->closed = 0;
                        push_heap_insert(next_idx);
                    } else if (next->heap_pos != PUSH_NO_NODE) {
                        push_heap_sift_up(next->heap_pos);
                    } else {
                        push_heap_insert(next_idx);
                    }
                }
            }
        }
    }

    return PUSH_NO_NODE;
}

static int build_shortest_walk_path(uint8_t start,
                                    uint8_t goal,
                                    const uint8_t boxes[MAX_BOXES2],
                                    char *out,
                                    int out_capacity)
{
    memset(push_walk_parent, 0xFF, sizeof(push_walk_parent));
    memset(push_occupied, 0, PUSH_CELL_COUNT);
    for (int i = 0; i < num_dyn_boxes; i++) {
        if (boxes[i] != PUSH_BOX_REMOVED) {
            push_occupied[boxes[i]] = 1;
        }
    }

    int head = 0;
    int tail = 0;
    push_walk_parent[start] = start;
    push_walk_queue[tail++] = start;

    while (head < tail && push_walk_parent[goal] < 0) {
        uint8_t curr = push_walk_queue[head++];
        int x = push_cell_x(curr);
        int y = push_cell_y(curr);

        for (int d = 0; d < 4; d++) {
            int nx = x + push_dx[d];
            int ny = y + push_dy[d];
            if (!push_inside(nx, ny)) continue;

            uint8_t next = push_cell(nx, ny);
            if (push_wall[next] || push_occupied[next]) continue;
            if (push_walk_parent[next] >= 0) continue;

            push_walk_parent[next] = curr;
            push_walk_move[next] = push_dir_char[d];
            push_walk_queue[tail++] = next;
        }
    }

    if (push_walk_parent[goal] < 0) return -1;

    char *reverse_path = push_reverse_path;
    int len = 0;
    uint8_t curr = goal;
    while (curr != start) {
        if (len >= PUSH_CELL_COUNT) return -1;
        reverse_path[len++] = push_walk_move[curr];
        curr = (uint8_t)push_walk_parent[curr];
    }

    if (len > out_capacity) return -2;
    for (int i = 0; i < len; i++) {
        out[i] = reverse_path[len - 1 - i];
    }
    return len;
}

static bool duplicate_same_state2(const DuplicateBeamState2 *a,
                                  const DuplicateBeamState2 *b)
{
    return a->player == b->player &&
           a->used_target_mask == b->used_target_mask &&
           memcmp(a->boxes, b->boxes, (size_t)num_dyn_boxes) == 0;
}

static int duplicate_state_compare2(const void *left_ptr,
                                    const void *right_ptr)
{
    const DuplicateBeamState2 *left =
        (const DuplicateBeamState2 *)left_ptr;
    const DuplicateBeamState2 *right =
        (const DuplicateBeamState2 *)right_ptr;
    if (left->score != right->score)
        return (int)left->score - (int)right->score;
    return (int)left->cost - (int)right->cost;
}

static int duplicate_find_worst2(int count)
{
    int worst = 0;
    for (int i = 1; i < count; i++) {
        if (dup_beam_next2[i].score > dup_beam_next2[worst].score ||
            (dup_beam_next2[i].score == dup_beam_next2[worst].score &&
             dup_beam_next2[i].cost > dup_beam_next2[worst].cost)) {
            worst = i;
        }
    }
    return worst;
}

static void duplicate_keep_candidate2(const DuplicateBeamState2 *candidate,
                                      int *count,
                                      int *worst_index)
{
    for (int i = 0; i < *count; i++) {
        if (!duplicate_same_state2(&dup_beam_next2[i], candidate)) continue;
        if (candidate->cost < dup_beam_next2[i].cost) {
            dup_beam_next2[i] = *candidate;
            if (*count >= DUP_BEAM_WIDTH2 && i == *worst_index)
                *worst_index = duplicate_find_worst2(*count);
        }
        return;
    }

    if (*count < DUP_BEAM_WIDTH2) {
        dup_beam_next2[(*count)++] = *candidate;
        if (*count == DUP_BEAM_WIDTH2)
            *worst_index = duplicate_find_worst2(*count);
        return;
    }

    int worst = *worst_index;
    if (candidate->score > dup_beam_next2[worst].score ||
        (candidate->score == dup_beam_next2[worst].score &&
         candidate->cost >= dup_beam_next2[worst].cost)) {
        return;
    }
    dup_beam_next2[worst] = *candidate;
    *worst_index = duplicate_find_worst2(*count);
}

static bool duplicate_rebuild_beam_path2(
    uint8_t start_player,
    const uint8_t start_boxes[MAX_BOXES2],
    const DuplicateBeamState2 *best)
{
    uint8_t player = start_player;
    uint8_t boxes[MAX_BOXES2];
    memcpy(boxes, start_boxes, (size_t)num_dyn_boxes);
    SokobanBoxMask used_target_mask = 0U;
    int output_len = 0;

    for (int action_index = 0;
         action_index < best->action_count;
         action_index++) {
        uint8_t action = best->actions[action_index];
        int box_index = action / 4;
        int d = action % 4;
        if (box_index < 0 || box_index >= num_dyn_boxes) return false;
        uint8_t box_cell_pos = boxes[box_index];
        if (box_cell_pos == PUSH_BOX_REMOVED) return false;

        int bx = push_cell_x(box_cell_pos);
        int by = push_cell_y(box_cell_pos);
        int hx = bx - push_dx[d];
        int hy = by - push_dy[d];
        int fx = bx + push_dx[d];
        int fy = by + push_dy[d];
        if (!push_inside(hx, hy) || !push_inside(fx, fy)) return false;

        uint8_t behind = push_cell(hx, hy);
        int walk_len = build_shortest_walk_path(
            player, behind, boxes,
            push_work_commands + output_len,
            SOKOBAN_LEVEL2_OUTPUT_CAPACITY - 1 - output_len);
        if (walk_len < 0) return false;
        output_len += walk_len;
        if (output_len >= SOKOBAN_LEVEL2_OUTPUT_CAPACITY - 1) return false;
        push_work_commands[output_len++] = push_dir_char[d];

        uint8_t destination = push_cell(fx, fy);
        int target_index = box_target_index_at_cell(box_index, destination);
        if (target_index >= 0) {
            SokobanBoxMask bit = SOKOBAN_BOX_MASK_BIT(target_index);
            if ((used_target_mask & bit) == 0U) {
                boxes[box_index] = PUSH_BOX_REMOVED;
                used_target_mask = (SokobanBoxMask)(used_target_mask | bit);
            } else {
                boxes[box_index] = destination;
            }
        } else {
            boxes[box_index] = destination;
        }
        sort_equal_label_box_positions(boxes);
        player = box_cell_pos;
    }

    if (!push_goal(boxes) ||
        used_target_mask != sokoban_full_mask_u8((uint8_t)num_dyn_targets)) {
        return false;
    }
    if (player != best->player || used_target_mask != best->used_target_mask ||
        memcmp(boxes, best->boxes, (size_t)num_dyn_boxes) != 0) {
        return false;
    }

    push_steps2 = output_len;
    push_work_commands[push_steps2] = '\0';
    return true;
}

static bool duplicate_beam_solve2(
    uint8_t start_player,
    const uint8_t start_boxes[MAX_BOXES2])
{
    build_duplicate_target_distances2();
    memset(&dup_beam_current2[0], 0, sizeof(dup_beam_current2[0]));
    dup_beam_current2[0].player = start_player;
    memcpy(dup_beam_current2[0].boxes, start_boxes,
           (size_t)num_dyn_boxes);
    dup_beam_current2[0].used_target_mask = 0U;
    int initial_h = duplicate_heuristic2(start_boxes, 0U);
    if (initial_h >= 10000) return false;
    dup_beam_current2[0].cost = 0U;
    dup_beam_current2[0].score = (uint16_t)(3 * initial_h);

    int current_count = 1;
    DuplicateBeamState2 best_state;
    int best_cost = 999999;
    push_search_nodes2 = 1U;

    for (int depth = 0; depth <= DUP_MAX_PUSH_ACTIONS2; depth++) {
        int minimum_open_cost = 999999;
        for (int i = 0; i < current_count; i++) {
            if (push_goal(dup_beam_current2[i].boxes)) {
                if (dup_beam_current2[i].cost < best_cost) {
                    best_state = dup_beam_current2[i];
                    best_cost = dup_beam_current2[i].cost;
                }
            } else if (dup_beam_current2[i].cost < minimum_open_cost) {
                minimum_open_cost = dup_beam_current2[i].cost;
            }
        }
        if (best_cost < 999999 && minimum_open_cost >= best_cost) break;
        if (depth >= DUP_MAX_PUSH_ACTIONS2) break;

        int next_count = 0;
        int worst_index = -1;
        for (int state_index = 0;
             state_index < current_count;
             state_index++) {
            const DuplicateBeamState2 *state =
                &dup_beam_current2[state_index];
            build_player_reachability(state->player, state->boxes);

            for (int box_index = 0;
                 box_index < num_dyn_boxes;
                 box_index++) {
                uint8_t box_cell_pos = state->boxes[box_index];
                if (box_cell_pos == PUSH_BOX_REMOVED) continue;
                int bx = push_cell_x(box_cell_pos);
                int by = push_cell_y(box_cell_pos);

                for (int d = 0; d < 4; d++) {
                    int fx = bx + push_dx[d];
                    int fy = by + push_dy[d];
                    int hx = bx - push_dx[d];
                    int hy = by - push_dy[d];
                    if (!push_inside(fx, fy) || !push_inside(hx, hy)) continue;

                    uint8_t destination = push_cell(fx, fy);
                    uint8_t behind = push_cell(hx, hy);
                    if (push_wall[destination] || push_occupied[destination])
                        continue;
                    if (push_walk_dist[behind] == 0xFFU) continue;

                    DuplicateBeamState2 candidate = *state;
                    candidate.player = box_cell_pos;
                    int target_index = box_target_index_at_cell(
                        box_index, destination);
                    if (target_index >= 0) {
                        SokobanBoxMask bit = SOKOBAN_BOX_MASK_BIT(target_index);
                        if ((candidate.used_target_mask & bit) == 0U) {
                            candidate.boxes[box_index] = PUSH_BOX_REMOVED;
                            candidate.used_target_mask = (SokobanBoxMask)(
                                candidate.used_target_mask | bit);
                        } else {
                            candidate.boxes[box_index] = destination;
                        }
                    } else {
                        candidate.boxes[box_index] = destination;
                    }
                    sort_equal_label_box_positions(candidate.boxes);

                    if (duplicate_deadlock2(candidate.boxes,
                                            candidate.used_target_mask)) {
                        continue;
                    }
                    uint32_t new_cost = (uint32_t)state->cost +
                                        push_walk_dist[behind] + 1U;
                    if (new_cost >= SOKOBAN_LEVEL2_OUTPUT_CAPACITY) continue;
                    int heuristic = duplicate_heuristic2(
                        candidate.boxes, candidate.used_target_mask);
                    if (heuristic >= 10000) continue;

                    candidate.cost = (uint16_t)new_cost;
                    uint32_t score = new_cost + 3U * (uint32_t)heuristic;
                    candidate.score = score > 65535U
                                      ? 65535U : (uint16_t)score;
                    candidate.actions[depth] =
                        (uint8_t)(box_index * 4 + d);
                    candidate.action_count = (uint8_t)(depth + 1);
                    duplicate_keep_candidate2(&candidate, &next_count,
                                              &worst_index);
                    push_search_nodes2++;
                }
            }
        }

        if (next_count <= 0) break;
        qsort(dup_beam_next2, (size_t)next_count,
              sizeof(DuplicateBeamState2), duplicate_state_compare2);
        memcpy(dup_beam_current2, dup_beam_next2,
               (size_t)next_count * sizeof(DuplicateBeamState2));
        current_count = next_count;
    }

    if (best_cost >= 999999) return false;
    if (!duplicate_rebuild_beam_path2(start_player, start_boxes,
                                      &best_state)) {
        return false;
    }
    return push_steps2 == best_cost;
}

static bool reconstruct_push_commands(uint16_t goal_idx,
                                      bool safe_direct_mode)
{
    int chain_len = 0;
    uint16_t curr = goal_idx;

    while (curr != PUSH_NO_NODE) {
        if (chain_len >= PUSH_PATH_LIMIT) { push_path_capacity_hit2 = true; return false; }
        push_solution_chain[chain_len++] = curr;
        curr = push_nodes[curr].parent;
    }

    if (chain_len <= 0) return false;

    PushNode *root = &push_nodes[push_solution_chain[chain_len - 1]];
    uint8_t player = root->player;
    uint8_t boxes[MAX_BOXES2];
    memcpy(boxes, root->boxes, (size_t)num_dyn_boxes);
    SokobanBoxMask aux_state = root->active_box;

    int out_len = 0;

    for (int k = chain_len - 2; k >= 0; k--) {
        PushNode *child = &push_nodes[push_solution_chain[k]];
        int b = child->move_box;
        int d = child->move_dir;
        if (b < 0 || b >= num_dyn_boxes || d < 0 || d >= 4) return false;

        uint8_t old_box = boxes[b];
        if (old_box == PUSH_BOX_REMOVED) return false;
        int bx = push_cell_x(old_box);
        int by = push_cell_y(old_box);
        int hx = bx - push_dx[d];
        int hy = by - push_dy[d];
        int fx = bx + push_dx[d];
        int fy = by + push_dy[d];
        if (!push_inside(hx, hy) || !push_inside(fx, fy)) return false;

        uint8_t behind = push_cell(hx, hy);
        int walk_len = build_shortest_walk_path(
            player,
            behind,
            boxes,
            push_work_commands + out_len,
            PUSH_PATH_LIMIT - 1 - out_len);
        if (walk_len == -2) { push_path_capacity_hit2 = true; return false; }
        if (walk_len < 0) return false;
        out_len += walk_len;

        if (out_len >= PUSH_PATH_LIMIT - 1) { push_path_capacity_hit2 = true; return false; }
        push_work_commands[out_len++] = push_dir_char[d];

        player = old_box;
        uint8_t front = push_cell(fx, fy);
        if (safe_direct_mode) {
            boxes[b] = box_matches_target_cell(b, front)
                       ? PUSH_BOX_REMOVED
                       : front;
            aux_state = (boxes[b] == PUSH_BOX_REMOVED)
                          ? PUSH_ACTIVE_NONE
                          : (SokobanBoxMask)b;
        } else {
            int target_index = box_target_index_at_cell(b, front);
            if (target_index >= 0) {
                SokobanBoxMask target_bit =
                    SOKOBAN_BOX_MASK_BIT(target_index);
                if ((aux_state & target_bit) == 0U) {
                    boxes[b] = PUSH_BOX_REMOVED;
                    aux_state = (SokobanBoxMask)(aux_state | target_bit);
                } else {
                    boxes[b] = front;
                }
            } else {
                boxes[b] = front;
            }
        }
        sort_equal_label_box_positions(boxes);

        if (player != child->player ||
            aux_state != child->active_box ||
            memcmp(boxes, child->boxes, (size_t)num_dyn_boxes) != 0) {
            return false;
        }
    }

    push_steps2 = out_len;
    push_work_commands[push_steps2] = '\0';
    return true;
}

static bool safe_direct_mode_supported(void)
{
    /* The competition currently uses unique A/B/C elements.  Keeping this
     * restriction avoids identity ambiguity when equal-labelled boxes are
     * canonicalised by sorting. Cooperative mode still supports duplicates. */
    for (int i = 0; i < num_dyn_boxes; i++) {
        for (int j = i + 1; j < num_dyn_boxes; j++) {
            if (dyn_boxes[i].label == dyn_boxes[j].label) return false;
        }
    }
    return true;
}


static int safe_best_steps;
static uint8_t safe_best_order[MAX_BOXES2];
static uint8_t safe_work_order[MAX_BOXES2];
static uint8_t safe_order_used[MAX_BOXES2];
static uint8_t safe_start_boxes[MAX_BOXES2];
static uint8_t safe_start_player;

static void test_one_safe_permutation(void)
{
    /* Try progressively greedier searches only when the previous attempt
     * exhausts the node budget. A successful route for this fixed order is
     * compared with all other permutations and the shortest one is retained. */
    for (unsigned int attempt = 0;
         attempt < sizeof(push_search_weights) / sizeof(push_search_weights[0]);
         attempt++) {
        uint16_t goal = run_push_search(safe_start_player,
                                        safe_start_boxes,
                                        push_search_weights[attempt],
                                        true,
                                        safe_work_order);
        push_search_nodes2 += push_node_count;
        if (goal != PUSH_NO_NODE && reconstruct_push_commands(goal, true)) {
            if (safe_best_steps < 0 || push_steps2 < safe_best_steps) {
                safe_best_steps = push_steps2;
                memcpy(safe_best_commands, push_work_commands,
                       (size_t)push_steps2 + 1u);
                memcpy(safe_best_order, safe_work_order,
                       (size_t)num_dyn_boxes);
            }
            break;
        }
    }
}

static void finish_safe_permutation_depth2(int *depth)
{
    (*depth)--;
    if (*depth >= 0) {
        SokobanPermutationFrame *parent = &safe_perm_stack[*depth];
        if (parent->active_index < (uint8_t)num_dyn_boxes) {
            safe_order_used[parent->active_index] = 0;
        }
        parent->active_index = 0xFFU;
    }
}

/* Iterative equivalent of enumerate_safe_permutations(). */
static void enumerate_safe_permutations_iterative2(void)
{
    int depth = 0;
    memset(safe_perm_stack, 0xFF, sizeof(safe_perm_stack));
    safe_perm_stack[0].next_index = 0;

    while (depth >= 0) {
        if (depth >= num_dyn_boxes) {
            test_one_safe_permutation();
            finish_safe_permutation_depth2(&depth);
            continue;
        }

        SokobanPermutationFrame *frame = &safe_perm_stack[depth];
        while (frame->next_index < (uint8_t)num_dyn_boxes &&
               safe_order_used[frame->next_index]) {
            frame->next_index++;
        }

        if (frame->next_index >= (uint8_t)num_dyn_boxes) {
            finish_safe_permutation_depth2(&depth);
            continue;
        }

        uint8_t selected = frame->next_index++;
        safe_order_used[selected] = 1;
        safe_work_order[depth] = selected;
        frame->active_index = selected;

        depth++;
        if (depth < num_dyn_boxes) {
            if (depth >= MAX_BOXES2) {
                sokoban_error2 = SOKOBAN_ERROR_SEARCH_STACK2;
                return;
            }
            safe_perm_stack[depth].next_index = 0;
            safe_perm_stack[depth].active_index = 0xFFU;
        }
    }
}

static bool SOKOBAN_UNUSED_FN2 find_best_explicit_safe_plan(uint8_t start_player,
                                         const uint8_t start_boxes[MAX_BOXES2])
{
    safe_best_steps = -1;
    safe_start_player = start_player;
    memcpy(safe_start_boxes, start_boxes, sizeof(safe_start_boxes));
    memset(safe_order_used, 0, sizeof(safe_order_used));

    enumerate_safe_permutations_iterative2();
    if (sokoban_error2 != SOKOBAN_OK2) return false;
    if (safe_best_steps < 0) {
        if (push_path_capacity_hit2) {
            sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
        }
        return false;
    }

    if (safe_best_steps >= (int)sizeof(push_commands2)) {
        push_steps2 = 0;
        push_commands2[0] = '\0';
        sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
        return false;
    }

    push_steps2 = safe_best_steps;
    memcpy(push_commands2, safe_best_commands, (size_t)push_steps2 + 1u);
    for (int i = 0; i < num_dyn_boxes; i++) {
        push_safe_order2[i] = dyn_boxes[safe_best_order[i]].label;
    }
    push_safe_order2[num_dyn_boxes] = '\0';
    return true;
}

static bool find_duplicate_cooperative_plan2(
    uint8_t start_player,
    const uint8_t start_boxes[MAX_BOXES2])
{
    if (duplicate_beam_solve2(start_player, start_boxes)) {
        memcpy(push_commands2, push_work_commands,
               (size_t)push_steps2 + 1U);
        push_safe_order2[0] = '\0';
        return true;
    }

    /* Restore the box-indexed reverse-distance layout before the legacy A*
     * fallback.  The bounded beam above is the normal repeated-element path;
     * these attempts only provide another chance on unusual geometries. */
    if (!build_static_box_distances()) return false;
    for (unsigned int attempt = 0;
         attempt < sizeof(duplicate_push_search_weights) /
                   sizeof(duplicate_push_search_weights[0]);
         attempt++) {
        uint16_t goal = run_push_search(
            start_player, start_boxes,
            duplicate_push_search_weights[attempt], false, NULL);
        push_search_nodes2 += push_node_count;

        if (goal == PUSH_NO_NODE) continue;
        if (!reconstruct_push_commands(goal, false)) continue;
        if (push_steps2 >= (int)sizeof(push_commands2)) {
            sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
            return false;
        }

        memcpy(push_commands2, push_work_commands,
               (size_t)push_steps2 + 1U);
        push_safe_order2[0] = '\0';
        return true;
    }

    if (push_path_capacity_hit2)
        sokoban_error2 = SOKOBAN_ERROR_PUSH_PATH_CAPACITY2;
    return false;
}

void plan_push_phase2(char external_map[12][16])
{
    memcpy(dynamic_map, external_map, sizeof(dynamic_map));
    push_steps2 = 0;
    push_commands2[0] = '\0';
    push_plan_mode2 = PUSH_PLAN_NONE2;
    push_safe_order2[0] = '\0';
    sokoban_error2 = SOKOBAN_OK2;
    push_search_nodes2 = 0;
    push_path_capacity_hit2 = false;
    g_l2_strict_dp_states2 = 0U;
    g_l2_strict_transition_searches2 = 0U;
    g_l2_strict_transition_nodes2 = 0U;
    g_l2_strict_best_cost2 = -1;
    g_l2_strict_result2 = 0;

    /* Preserve the original unknown-element completion rule.  These counters
     * share the push-stage workspace instead of occupying the main stack. */
    int16_t *b_cnt = push_box_counts;
    int16_t *t_cnt = push_target_counts;
    memset(b_cnt, 0, sizeof(g_sokoban_workspace.level2.stage.push.label_counts[0]));
    memset(t_cnt, 0, sizeof(g_sokoban_workspace.level2.stage.push.label_counts[1]));

    for (int y = 0; y < PUSH_H; y++) {
        for (int x = 0; x < PUSH_W; x++) {
            char c = dynamic_map[y][x];
            if (c >= 'A' && c <= 'Z') b_cnt[c - 'A']++;
            if (c >= 'a' && c <= 'z') t_cnt[c - 'a']++;
        }
    }

    for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        while (b_cnt[i] < t_cnt[i]) {
            bool filled = false;
            for (int y = 0; y < PUSH_H && !filled; y++) {
                for (int x = 0; x < PUSH_W && !filled; x++) {
                    if (dynamic_map[y][x] == '$') {
                        dynamic_map[y][x] = (char)('A' + i);
                        b_cnt[i]++;
                        filled = true;
                    }
                }
            }
            if (!filled) break;
        }

        while (t_cnt[i] < b_cnt[i]) {
            bool filled = false;
            for (int y = 0; y < PUSH_H && !filled; y++) {
                for (int x = 0; x < PUSH_W && !filled; x++) {
                    if (dynamic_map[y][x] == '.') {
                        dynamic_map[y][x] = (char)('a' + i);
                        t_cnt[i]++;
                        filled = true;
                    }
                }
            }
            if (!filled) break;
        }
    }

    num_dyn_boxes = 0;
    num_dyn_targets = 0;
    memset(push_wall, 0, sizeof(push_wall));

    for (int y = 0; y < PUSH_H; y++) {
        for (int x = 0; x < PUSH_W; x++) {
            char c = dynamic_map[y][x];
            uint8_t cell = push_cell(x, y);
            push_wall[cell] = (c == '#') ? 1u : 0u;

            if (c >= 'A' && c <= 'Z') {
                if (num_dyn_boxes >= MAX_BOXES2) {
                    sokoban_error2 = SOKOBAN_ERROR_INVALID_MAP2;
                    return;
                }
                dyn_boxes[num_dyn_boxes].label = c;
                dyn_boxes[num_dyn_boxes].pos = (Pos2){x, y};
                num_dyn_boxes++;
            } else if (c >= 'a' && c <= 'z') {
                if (num_dyn_targets >= MAX_BOXES2) {
                    sokoban_error2 = SOKOBAN_ERROR_INVALID_MAP2;
                    return;
                }
                dyn_targets[num_dyn_targets].label = c;
                dyn_targets[num_dyn_targets].pos = (Pos2){x, y};
                num_dyn_targets++;
            }
        }
    }

    if (num_dyn_boxes <= 0 || num_dyn_boxes != num_dyn_targets) {
        sokoban_error2 = SOKOBAN_ERROR_INVALID_MAP2;
        return;
    }

    /* Every upper-case count must exactly match its lower-case count. */
    memset(b_cnt, 0, sizeof(g_sokoban_workspace.level2.stage.push.label_counts[0]));
    memset(t_cnt, 0, sizeof(g_sokoban_workspace.level2.stage.push.label_counts[1]));
    for (int i = 0; i < num_dyn_boxes; i++) b_cnt[dyn_boxes[i].label - 'A']++;
    for (int i = 0; i < num_dyn_targets; i++) t_cnt[dyn_targets[i].label - 'a']++;
    for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        if (b_cnt[i] != t_cnt[i]) {
            sokoban_error2 = SOKOBAN_ERROR_INVALID_MAP2;
            return;
        }
    }

    sort_dynamic_objects();

    uint8_t start_boxes[MAX_BOXES2];
    for (int i = 0; i < num_dyn_boxes; i++) {
        start_boxes[i] = push_cell(dyn_boxes[i].pos.x,
                                  dyn_boxes[i].pos.y);
    }
    sort_equal_label_box_positions(start_boxes);

    if (!build_static_box_distances()) return;
    if (push_heuristic(start_boxes) == 0xFFFFu) return;

    if (!push_inside(explore_end_pos.x, explore_end_pos.y)) return;
    uint8_t start_player = push_cell(explore_end_pos.x,
                                     explore_end_pos.y);

    /* Default six-box planner: reuse common order prefixes through a subset
     * DP over (completed mask, player cell). It preserves the competition
     * rule that one selected box must disappear before another box is moved. */
#if LEVEL2_USE_STRICT_ORDER_DP2
    bool strict_ok2;
    if (num_dyn_boxes <= SOKOBAN_LEVEL2_STRICT_EXACT_MAX_BOXES)
        strict_ok2 = find_best_strict_subset_plan2(start_player);
    else
        strict_ok2 = find_best_strict_beam_plan2(start_player);
    if (strict_ok2) {
        push_plan_mode2 = PUSH_PLAN_SAFE_DIRECT2;
        return;
    }
#else
    if (safe_direct_mode_supported() &&
        find_best_explicit_safe_plan(start_player, start_boxes)) {
        push_plan_mode2 = PUSH_PLAN_SAFE_DIRECT2;
        return;
    }
#endif
    if (sokoban_error2 != SOKOBAN_OK2) return;

    /* Repeated element labels are a different state model, not an error.
     * Equal boxes are canonicalised by position, while active_box in the A*
     * node stores which physical target cells have already been consumed.
     * This prevents two A boxes from both "falling" into the same a cell. */
    if (has_duplicate_box_labels2()) {
        if (find_duplicate_cooperative_plan2(start_player, start_boxes)) {
            push_plan_mode2 = PUSH_PLAN_COOPERATIVE2;
        }
        return;
    }

    /* Strict competition mode: do not fall back to cooperative multi-box
     * motion.  If no permutation can finish one selected box completely before
     * touching the next, leave PUSH_PLAN_NONE2 and report no path. */
    return;
}

void clear_sokoban_memory2(void)
{
    /* Use only before a fresh run or after all level-2 commands finish. */
    sokoban_workspace_clear();

    explore_steps2 = 0;
    memset(explore_commands2, 0, sizeof(explore_commands2));
    memset(expected_targets2, 0, sizeof(expected_targets2));
    expected_cnt2 = 0;

    push_steps2 = 0;
    memset(push_commands2, 0, sizeof(push_commands2));
    push_plan_mode2 = PUSH_PLAN_NONE2;
    sokoban_error2 = SOKOBAN_OK2;
    explore_search_nodes2 = 0;
    push_search_nodes2 = 0;
    g_l2_recog_move_bfs_calls2 = 0U;
    g_l2_recog_dp_states2 = 0U;
    g_l2_recog_dp_transitions2 = 0U;
    g_l2_recog_pose_count2 = 0U;
    g_l2_recog_compact_state_capacity2 = 0U;
    g_l2_recog_best_cost2 = -1;
    g_l2_recog_result2 = 0;
    g_l2_strict_dp_states2 = 0U;
    g_l2_strict_transition_searches2 = 0U;
    g_l2_strict_transition_nodes2 = 0U;
    g_l2_strict_best_cost2 = -1;
    g_l2_strict_result2 = 0;
    memset(push_safe_order2, 0, sizeof(push_safe_order2));

    explore_end_pos = (Pos2){0, 0};
    memset(dyn_boxes, 0, sizeof(dyn_boxes));
    memset(dyn_targets, 0, sizeof(dyn_targets));
    num_dyn_boxes = 0;
    num_dyn_targets = 0;

    tsp_search_cnt = 0;
    tsp_path_capacity_hit2 = false;
    push_path_capacity_hit2 = false;
    bfs_cache_cnt = 0;
    memset(all_objs, 0, sizeof(all_objs));
    num_objs = 0;
    global_best_cost = 999999;
    global_best_turns = 999999;
    memset(global_best_path, 0, sizeof(global_best_path));
    global_best_len = 0;

    push_node_count = 0;
    push_heap_count = 0;
    safe_best_steps = -1;
    memset(safe_best_order, 0, sizeof(safe_best_order));
    memset(safe_work_order, 0, sizeof(safe_work_order));
    memset(safe_order_used, 0, sizeof(safe_order_used));
    memset(safe_start_boxes, 0, sizeof(safe_start_boxes));
    safe_start_player = 0;
}
