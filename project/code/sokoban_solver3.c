#include <string.h>
#include <stdlib.h>
#include "sokoban_solver3.h"
#include "sokoban_workspace.h"

/* Alternate legacy planners are intentionally retained for API compatibility.
 * The competition entry points do not reference every static helper. */
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// ================= ????? (???) =================
#ifndef BFS_QUEUE_SIZE
#define BFS_QUEUE_SIZE 10000
#endif

/* 0: conservative online mode; 1: allow optional bomb shortcuts before N-1.
 * Enabling 1 is more aggressive but can consume an irreversible bomb before
 * unknown labels are known. */
#ifndef LEVEL3_ALLOW_OPTIONAL_BOMB_SHORTCUT3
#define LEVEL3_ALLOW_OPTIONAL_BOMB_SHORTCUT3 0
#endif

/* 1: compressed bitmask DP for recognition order; 0: legacy factorial DFS. */
#ifndef LEVEL3_USE_RECOGNITION_DP3
#define LEVEL3_USE_RECOGNITION_DP3 1
#endif

/* Compact BFS workspace.
 * Each 16-bit state stores one 4-bit predecessor code instead of a complete
 * 16-bit parent state. Codes 0..3 are ordinary moves, 4..7 are pushes,
 * 0xE marks the root and 0xF means unvisited. This reduces the predecessor
 * table from 128 KiB to 18 KiB after compact 12x16 state indexing.
 */
#define LEVEL3_CELL_COUNT3      (12u * 16u)
#define BFS_STATE_COUNT3        (LEVEL3_CELL_COUNT3 * LEVEL3_CELL_COUNT3)
#define BFS_PARENT_CODE_BYTES3 ((BFS_STATE_COUNT3 + 1u) / 2u)
#define BFS_PARENT_ROOT3       0x0Eu
#define BFS_PARENT_UNVISITED3  0x0Fu

#ifndef LEVEL3_MULTI_PUSH_BEAM_WIDTH3
#define LEVEL3_MULTI_PUSH_BEAM_WIDTH3 106
#endif
#ifndef LEVEL3_MULTI_PUSH_MAX_ACTIONS3
#define LEVEL3_MULTI_PUSH_MAX_ACTIONS3 (MAX_BOXES3 * 16)
#endif
#ifndef LEVEL3_MULTI_PUSH_MAX_BOXES3
#define LEVEL3_MULTI_PUSH_MAX_BOXES3 MAX_BOXES3
#endif
#define LEVEL3_MULTI_PUSH_REMOVED3 0xFFu

typedef struct {
    uint8_t player;
    uint8_t boxes[LEVEL3_MULTI_PUSH_MAX_BOXES3];
    SokobanBoxMask used_target_mask;
    uint16_t cost;
    uint16_t score;
    uint8_t action_count;
    uint8_t actions[LEVEL3_MULTI_PUSH_MAX_ACTIONS3];
} Level3MultiPushState3;

typedef enum { BFS_STATE_CELL3 = 0, BFS_STATE_PLAYER_BOX3 = 1 } BfsStateMode3;

/* Bomb/explore BFS and final Beam search never execute at the same time.
 * Overlay their largest arrays so only the larger BFS workspace is resident. */
typedef union {
    struct {
        uint8_t parent_code3[BFS_PARENT_CODE_BYTES3];
        uint16_t queue3[BFS_QUEUE_SIZE];
    } bfs3;
    struct {
        Level3MultiPushState3 cur3[LEVEL3_MULTI_PUSH_BEAM_WIDTH3];
        Level3MultiPushState3 next3[LEVEL3_MULTI_PUSH_BEAM_WIDTH3];
    } beam3;
    /* Shortest-path straightening workspace.  It overlays the larger BFS /
     * Beam storage, so this feature does not increase static RAM. */
    struct {
        uint8_t dist_from_start3[LEVEL3_CELL_COUNT3];
        uint8_t dist_to_end3[LEVEL3_CELL_COUNT3];
        uint8_t queue3[LEVEL3_CELL_COUNT3];
        uint8_t blocked3[LEVEL3_CELL_COUNT3];
        uint8_t best_turns3[LEVEL3_CELL_COUNT3 * 4u];
        uint8_t next_dir3[LEVEL3_CELL_COUNT3 * 4u];
    } turn3;
} Level3LargeWorkspace3;

static Level3LargeWorkspace3 level3_large_workspace3;
#define bfs_parent_code3  (level3_large_workspace3.bfs3.parent_code3)
#define bfs_queue         (level3_large_workspace3.bfs3.queue3)
#define level3_multi_cur3 (level3_large_workspace3.beam3.cur3)
#define level3_multi_next3 (level3_large_workspace3.beam3.next3)
#define turn_dist_start3 (level3_large_workspace3.turn3.dist_from_start3)
#define turn_dist_end3   (level3_large_workspace3.turn3.dist_to_end3)
#define turn_queue3      (level3_large_workspace3.turn3.queue3)
#define turn_blocked3    (level3_large_workspace3.turn3.blocked3)
#define turn_best3       (level3_large_workspace3.turn3.best_turns3)
#define turn_next3       (level3_large_workspace3.turn3.next_dir3)

static int bfs_used_count = 0;
static BfsStateMode3 bfs_state_mode3 = BFS_STATE_CELL3;
static bool bfs_parent_initialized3 = false;
static char dynamic_map[12][16];
static char current_map[12][16];

/* One physical command buffer is reused by all public/internal stages. */
int level3_steps3 = 0;
char level3_commands3[LEVEL3_PATH_CAPACITY3];

/* Persistent front-stage output.
 * plan_bomb_explore_phase3() stores bomb + recognition here so that a later
 * plan_push_phase3() call may overwrite level3_commands3[] without losing
 * the already planned front path. This costs only 256 bytes plus one int. */
int bomb_explore_steps3 = 0;
char bomb_explore_commands3[LEVEL3_PATH_CAPACITY3];

#define explore_commands3     level3_commands3
#define bomb_commands3        level3_commands3
#define push_commands3        level3_commands3
#define prepare_commands3     level3_commands3
#define level3_easy_commands3 level3_commands3

int explore_steps3 = 0;
Pos3 expected_targets3[LEVEL3_RECOGNITION_CAPACITY3];
int expected_cnt3 = 0;
int bomb_steps3 = 0;
uint8_t level3_bombs_used3 = 0U;
int push_steps3 = 0;
int prepare_steps3 = 0;

/* Unified easy-controller output. */
Level3EasyState3 level3_easy_state3 = LEVEL3_EASY_IDLE3;
int level3_easy_steps3 = 0;
Pos3 level3_easy_target3 = { -1, -1 };
int level3_easy_error3 = 0;

/* Minimal integrated output for the final vehicle controller. */
uint8_t level3_need_vision3 = 0;
uint8_t level3_finished3 = 0;
uint8_t level3_error3 = 0;
int level3_error_code3 = 0;
Pos3 level3_vision_pos3 = { -1, -1 };


// ??????????
static Pos3 explore_end_pos;
static char explore_end_dir;

static Pos3 bombs[MAX_BOMBS3];
static int num_bombs = 0;

static int level3_input_bomb_count3(const char map3[12][16])
{
    int count3 = 0;
    for (int y3 = 0; y3 < 12; y3++)
        for (int x3 = 0; x3 < 16; x3++)
            if (map3[y3][x3] == '*') count3++;
    return count3;
}

/* Fixed preliminary flow state: bomb -> recognize -> push. */
static bool level3_bomb_first_initialized3 = false;


// ?????????????????????????????????
#ifndef ENABLE_EXPLORE_BOMB_INSERT
#define ENABLE_EXPLORE_BOMB_INSERT 0
#endif
#ifndef EXPLORE_BOMB_INSERT_LIMIT
#define EXPLORE_BOMB_INSERT_LIMIT SOKOBAN_LEVEL3_BOMB_LIMIT
#endif
#ifndef EXPLORE_BOMB_PUSH_MAX_COST
#define EXPLORE_BOMB_PUSH_MAX_COST 36
#endif
#ifndef EXPLORE_BOMB_MIN_GAIN
#define EXPLORE_BOMB_MIN_GAIN 4
#endif
#ifndef ENABLE_PUSH_SHORTCUT_BOMB
#define ENABLE_PUSH_SHORTCUT_BOMB 0
#endif
#ifndef PUSH_SHORTCUT_BOMB_LIMIT
#define PUSH_SHORTCUT_BOMB_LIMIT SOKOBAN_LEVEL3_BOMB_LIMIT
#endif
#ifndef PUSH_SHORTCUT_BOMB_MAX_COST
#define PUSH_SHORTCUT_BOMB_MAX_COST 46
#endif
#ifndef PUSH_SHORTCUT_MIN_GAIN
#define PUSH_SHORTCUT_MIN_GAIN 5
#endif
int solver_explore_bomb_insert_count = 0;
int solver_push_shortcut_bomb_count = 0;


/*
 * Stable staged controller state.
 * One call plans exactly one executable segment:
 * recognition, one bomb, or the final box-pushing path.
 */
static bool level3_staged_initialized = false;
static bool level3_staged_push_emitted = false;
static Pos3 level3_staged_player = {1, 5};
static char level3_staged_view_dir = 'U';
static uint32_t level3_staged_recognized_mask = 0;
static int level3_staged_pending_object = -1;
int level3_last_error3 = 0;

/* Global joint-planner diagnostics (current-information online search). */
int level3_global_score3 = 999999;
int level3_global_nodes_expanded3 = 0;
int level3_global_candidates3 = 0;
int level3_global_bomb_prefix3 = 0;
uint8_t level3_global_truncated3 = 0;
int level3_segment_type3 = 0; /* 1=recognize, 2=bomb, 3=push */

static void level3_internal_reset3(void);

typedef struct { char label; Pos3 box_pos; Pos3 target_pos; } DynBoxTargetPair;
static DynBoxTargetPair dyn_pairs[MAX_BOXES3];
static int num_dyn_pairs = 0;

static int best_push_steps = 999999;
static char best_push_path[LEVEL3_INTERNAL_PATH_CAPACITY3];

/* Explicit removed state: a box on its target no longer exists. */
static const Pos3 PUSH_BOX_REMOVED3 = { -1, -1 };

static bool is_box_removed3(Pos3 p)
{
    return (p.x < 0 || p.y < 0);
}


static uint16_t bfs_cell_state3(int x3, int y3)
{
    return (uint16_t)(y3 * 16 + x3);
}

static uint16_t bfs_full_state3(int px3, int py3, int bx3, int by3)
{
    uint16_t player3 = bfs_cell_state3(px3, py3);
    uint16_t box3 = bfs_cell_state3(bx3, by3);
    return (uint16_t)(player3 * LEVEL3_CELL_COUNT3 + box3);
}

static void bfs_decode_full_state3(
    uint16_t state3, int *px3, int *py3, int *bx3, int *by3)
{
    uint16_t player3 = (uint16_t)(state3 / LEVEL3_CELL_COUNT3);
    uint16_t box3 = (uint16_t)(state3 % LEVEL3_CELL_COUNT3);
    *px3 = player3 & 0x0F;
    *py3 = player3 >> 4;
    *bx3 = box3 & 0x0F;
    *by3 = box3 >> 4;
}

static uint8_t bfs_parent_code_get3(uint16_t state3)
{
    uint8_t packed3 = bfs_parent_code3[state3 >> 1];
    return (state3 & 1u) ? (uint8_t)(packed3 >> 4)
                         : (uint8_t)(packed3 & 0x0Fu);
}

static void bfs_parent_code_set3(uint16_t state3, uint8_t code3)
{
    uint8_t *packed3 = &bfs_parent_code3[state3 >> 1];
    if (state3 & 1u) {
        *packed3 = (uint8_t)((*packed3 & 0x0Fu) | (uint8_t)(code3 << 4));
    } else {
        *packed3 = (uint8_t)((*packed3 & 0xF0u) | (code3 & 0x0Fu));
    }
}

static bool bfs_is_visited_c(uint16_t state3)
{
    return bfs_parent_code_get3(state3) != BFS_PARENT_UNVISITED3;
}

static void bfs_invalidate_workspace3(void)
{
    bfs_used_count = 0;
    bfs_parent_initialized3 = false;
}

/* Clear only states touched by the previous BFS. */
static void bfs_clear_states_c(void)
{
    for (int i = 0; i < bfs_used_count; i++) {
        bfs_parent_code_set3(bfs_queue[i], BFS_PARENT_UNVISITED3);
    }
    bfs_used_count = 0;
}

static void bfs_begin_c(uint16_t start_state, BfsStateMode3 mode3)
{
    if (!bfs_parent_initialized3) {
        memset(bfs_parent_code3, 0xFF, sizeof(bfs_parent_code3));
        bfs_parent_initialized3 = true;
    }
    bfs_clear_states_c();
    bfs_state_mode3 = mode3;
    bfs_queue[0] = start_state;
    bfs_used_count = 1;
    bfs_parent_code_set3(start_state, BFS_PARENT_ROOT3);
}

static uint8_t bfs_encode_parent_action3(uint16_t parent3, uint16_t child3)
{
    int px3, py3, cx3, cy3;
    bool pushed3 = false;

    if (bfs_state_mode3 == BFS_STATE_CELL3) {
        px3 = parent3 & 0x0F;
        py3 = parent3 >> 4;
        cx3 = child3 & 0x0F;
        cy3 = child3 >> 4;
    } else {
        int pbx3, pby3, cbx3, cby3;
        bfs_decode_full_state3(parent3, &px3, &py3, &pbx3, &pby3);
        bfs_decode_full_state3(child3, &cx3, &cy3, &cbx3, &cby3);
        pushed3 = (pbx3 != cbx3 || pby3 != cby3);
    }

    uint8_t direction3;
    if (cy3 < py3) direction3 = 0u;       /* U */
    else if (cy3 > py3) direction3 = 1u;  /* D */
    else if (cx3 < px3) direction3 = 2u;  /* L */
    else direction3 = 3u;                 /* R */

    return (uint8_t)(direction3 | (pushed3 ? 4u : 0u));
}

static bool bfs_enqueue_c(uint16_t state, uint16_t parent, int *tail)
{
    if (bfs_is_visited_c(state)) return false;
    if (*tail >= BFS_QUEUE_SIZE) return false;
    bfs_parent_code_set3(state, bfs_encode_parent_action3(parent, state));
    bfs_queue[(*tail)++] = state;
    bfs_used_count = *tail;
    return true;
}

static uint16_t bfs_parent_state_c(uint16_t child3)
{
    static const int dx3[4] = {0, 0, -1, 1};
    static const int dy3[4] = {-1, 1, 0, 0};
    uint8_t code3 = bfs_parent_code_get3(child3);
    int direction3 = code3 & 3u;

    if (bfs_state_mode3 == BFS_STATE_CELL3) {
        int cx3 = child3 & 0x0F;
        int cy3 = child3 >> 4;
        return bfs_cell_state3(
            cx3 - dx3[direction3], cy3 - dy3[direction3]);
    }

    int cx3, cy3, cbx3, cby3;
    bfs_decode_full_state3(child3, &cx3, &cy3, &cbx3, &cby3);
    int pbx3 = cbx3;
    int pby3 = cby3;
    if (code3 & 4u) {
        pbx3 -= dx3[direction3];
        pby3 -= dy3[direction3];
    }

    return bfs_full_state3(
        cx3 - dx3[direction3], cy3 - dy3[direction3], pbx3, pby3);
}

static char bfs_full_move_c(uint16_t parent, uint16_t child)
{
    int px, py, bx, by;
    int cx, cy, cbx, cby;
    bfs_decode_full_state3(parent, &px, &py, &bx, &by);
    bfs_decode_full_state3(child, &cx, &cy, &cbx, &cby);
    (void)bx; (void)by; (void)cbx; (void)cby;
    if (cx > px) return 'R';
    if (cx < px) return 'L';
    if (cy > py) return 'D';
    return 'U';
}

static char bfs_cell_move_c(uint16_t parent, uint16_t child)
{
    int px = parent & 0x0F;
    int py = (parent >> 4) & 0x0F;
    int cx = child & 0x0F;
    int cy = (child >> 4) & 0x0F;
    if (cx > px) return 'R';
    if (cx < px) return 'L';
    if (cy > py) return 'D';
    return 'U';
}

// ??
typedef struct { int type; int orig_idx; Pos3 p; char name; Pos3 target_wall; } TspObj;
static bool level3_dijkstra_reach_count_c(Pos3 start_p, int required_count, TspObj *out_seq, int *out_cnt, int *out_cost);
static TspObj all_objs[SOKOBAN_RECOGNITION_OBJECT_CAPACITY]; static int num_objs = 0;

// ??? TSP ?????
// C ???? TSP ?????
static int global_best_cost = 999999; 
static char global_best_path[LEVEL3_INTERNAL_PATH_CAPACITY3]; 
static int global_best_len = 0;

/* Keil Watch diagnostics for the recognition-order planner. */
volatile uint32_t g_l3_recog_legacy_dfs_calls3 = 0U;
volatile uint32_t g_l3_recog_move_bfs_calls3 = 0U;
volatile uint32_t g_l3_recog_turn_bfs_calls3 = 0U;
volatile uint32_t g_l3_recog_dp_states3 = 0U;
volatile uint32_t g_l3_recog_dp_transitions3 = 0U;
volatile uint32_t g_l3_recog_pose_count3 = 0U;
volatile uint32_t g_l3_recog_compact_state_capacity3 = 0U;
volatile int32_t g_l3_recog_best_cost3 = -1;
volatile int32_t g_l3_recog_result3 = 0;

static bool check_is_solvable_c(Pos3 p_pos, Pos3* cur_bombs, int cur_nb);
static int estimate_push_cost_with_nminus1_c(Pos3 p_pos, Pos3* cur_bombs, int cur_nb);
static int level3_count_reachable_unrecognized_state_c(Pos3 from, Pos3 *state_bombs, int state_nb);

static void level3_mirror_world_to_external_c(char external_map[12][16]);

// ============================================================================
// ????? ( Python)
// ============================================================================
static bool is_pos_in_bombs(int x, int y, Pos3* b_arr, int nb) {
    for(int i=0; i<nb; i++) if(b_arr[i].x == x && b_arr[i].y == y) return true;
    return false;
}

static int get_total_box_count() {
    int cnt = 0;
    for(int y=0; y<12; y++) {
        for(int x=0; x<16; x++) {
            char c = dynamic_map[y][x];
            if ((c >= 'A' && c <= 'Z') || c == '$') cnt++;
        }
    }
    return cnt;
}

// ???????????????
static bool bfs_push_box_terrain_only_c(Pos3 start_player, Pos3 start_box, Pos3 target, Pos3* b_arr, int nb) {
    int head = 0, tail = 1;
    uint16_t start_state = bfs_full_state3(
        start_player.x, start_player.y, start_box.x, start_box.y);
    bfs_begin_c(start_state, BFS_STATE_PLAYER_BOX3);
    const int dx[4] = {0, 0, -1, 1};
    const int dy[4] = {-1, 1, 0, 0};

    while (head < tail) {
        uint16_t curr = bfs_queue[head++];
        int px, py, bx, by;
        bfs_decode_full_state3(curr, &px, &py, &bx, &by);

        if (bx == target.x && by == target.y) {
            bfs_clear_states_c();
            return true;
        }

        for (int i = 0; i < 4; i++) {
            int npx = px + dx[i], npy = py + dy[i];
            if (npx < 0 || npx >= 16 || npy < 0 || npy >= 12) continue;
            if (dynamic_map[npy][npx] == '#') continue;
            if (is_pos_in_bombs(npx, npy, b_arr, nb)) continue;

            int nbx = bx, nby = by;
            if (npx == bx && npy == by) {
                nbx += dx[i]; nby += dy[i];
                if (nbx < 0 || nbx >= 16 || nby < 0 || nby >= 12) continue;
                if (dynamic_map[nby][nbx] == '#') continue;
                if (is_pos_in_bombs(nbx, nby, b_arr, nb)) continue;
            }

            uint16_t ns = bfs_full_state3(npx, npy, nbx, nby);
            if (!bfs_is_visited_c(ns) && !bfs_enqueue_c(ns, curr, &tail)) {
                bfs_clear_states_c();
                return false;
            }
        }
    }

    bfs_clear_states_c();
    return false;
}

// ???????
static int get_solvable_boxes_c(Pos3 p_pos, Pos3* b_arr, int nb) {
    int solvable = 0;
    for(int y=0; y<12; y++){
        for(int x=0; x<16; x++){
            char c = dynamic_map[y][x];
            if((c >= 'A' && c <= 'Z') || c == '$') {
                bool found = false;
                for(int ty=0; ty<12; ty++){
                    for(int tx=0; tx<16; tx++){
                        char tc = dynamic_map[ty][tx];
                        bool target_ok = false;
                        if (c == '$') {
                            target_ok = (tc == '.' || (tc >= 'a' && tc <= 'z'));
                        } else if (c >= 'A' && c <= 'Z') {
                            target_ok = (tc == (char)(c + 32) || tc == '.');
                        }
                        if (target_ok) {
                            if (bfs_push_box_terrain_only_c(p_pos, (Pos3){x, y}, (Pos3){tx, ty}, b_arr, nb)) {
                                found = true; break;
                            }
                        }
                    }
                    if (found) break;
                }
                if (found) solvable++;
            }
        }
    }
    return solvable;
}

// ????????????????
static Pos3 blocking_walls[200];
static int num_blocking_walls = 0;

static void get_blocking_walls_c(Pos3* b_arr, int nb) {
    num_blocking_walls = 0;
    int dx[] = {-1, 1, 0, 0}; int dy[] = {0, 0, -1, 1};

    for(int y=0; y<12; y++){
        for(int x=0; x<16; x++){
            char c = dynamic_map[y][x];
            if((c >= 'A' && c <= 'Z') || c == '$' || (c >= 'a' && c <= 'z') || c == '.') {
                for(int i=0; i<4; i++){
                    int nx = x + dx[i], ny = y + dy[i];
                    if(nx>=0 && nx<16 && ny>=0 && ny<12 && dynamic_map[ny][nx] == '#') {
                        bool dup = false;
                        for(int k=0; k<num_blocking_walls; k++) if(blocking_walls[k].x == nx && blocking_walls[k].y == ny) { dup=true; break; }
                        if(!dup && num_blocking_walls < 200) blocking_walls[num_blocking_walls++] = (Pos3){nx, ny};
                    }
                }
            }
        }
    }
    for(int i=0; i<nb; i++){
        int x = b_arr[i].x, y = b_arr[i].y;
        for(int j=0; j<4; j++){
            int nx = x + dx[j], ny = y + dy[j];
            if(nx>=0 && nx<16 && ny>=0 && ny<12 && dynamic_map[ny][nx] == '#') {
                bool dup = false;
                for(int k=0; k<num_blocking_walls; k++) if(blocking_walls[k].x == nx && blocking_walls[k].y == ny) { dup=true; break; }
                if(!dup && num_blocking_walls < 200) blocking_walls[num_blocking_walls++] = (Pos3){nx, ny};
            }
        }
    }
}

// ???????
typedef struct { Pos3 wall; int path_len; Pos3 end_p; } ReachableWall;
static ReachableWall reach_walls[60];
static int num_reach_walls = 0;

static void get_all_reachable_walls_c(Pos3 player, Pos3 bomb, Pos3* cur_bombs, int cur_nb) {
    num_reach_walls = 0;
    int head = 0, tail = 1;
    int depth = 0, level_left = 1, next_level = 0;
    uint16_t start_state = bfs_full_state3(
        player.x, player.y, bomb.x, bomb.y);
    bfs_begin_c(start_state, BFS_STATE_PLAYER_BOX3);
    const int dx[4] = {0, 0, -1, 1};
    const int dy[4] = {-1, 1, 0, 0};

    while (head < tail) {
        uint16_t curr = bfs_queue[head++];
        int px, py, bx, by;
        bfs_decode_full_state3(curr, &px, &py, &bx, &by);

        if (depth <= 200) {
            for (int i = 0; i < 4; i++) {
                int npx = px + dx[i], npy = py + dy[i];
                if (npx < 0 || npx >= 16 || npy < 0 || npy >= 12) continue;
                char p_tc = dynamic_map[npy][npx];
                if (p_tc == '#') continue;
                if ((p_tc >= 'A' && p_tc <= 'Z') || p_tc == '$') continue;

                bool hit_other_bomb = false;
                for (int k = 0; k < cur_nb; k++) {
                    if (cur_bombs[k].x == npx && cur_bombs[k].y == npy &&
                        !(cur_bombs[k].x == bomb.x && cur_bombs[k].y == bomb.y)) {
                        hit_other_bomb = true; break;
                    }
                }
                if (hit_other_bomb) continue;

                int nbx = bx, nby = by;
                if (npx == bx && npy == by) {
                    nbx += dx[i]; nby += dy[i];
                    if (nbx < 0 || nbx >= 16 || nby < 0 || nby >= 12) continue;

                    if (dynamic_map[nby][nbx] == '#') {
                        if (nbx > 0 && nbx < 15 && nby > 0 && nby < 11) {
                            bool dup = false;
                            for (int k = 0; k < num_reach_walls; k++) {
                                if (reach_walls[k].wall.x == nbx && reach_walls[k].wall.y == nby) {
                                    dup = true; break;
                                }
                            }
                            if (!dup && num_reach_walls < 60) {
                                reach_walls[num_reach_walls++] =
                                    (ReachableWall){(Pos3){nbx, nby}, depth + 1, (Pos3){npx, npy}};
                            }
                        }
                        continue;
                    }

                    char bc = dynamic_map[nby][nbx];
                    if ((bc >= 'A' && bc <= 'Z') || bc == '$') continue;
                    bool b_hit_other = false;
                    for (int k = 0; k < cur_nb; k++) {
                        if (cur_bombs[k].x == nbx && cur_bombs[k].y == nby &&
                            !(cur_bombs[k].x == bomb.x && cur_bombs[k].y == bomb.y)) {
                            b_hit_other = true; break;
                        }
                    }
                    if (b_hit_other) continue;
                }

                uint16_t ns = bfs_full_state3(npx, npy, nbx, nby);
                if (!bfs_is_visited_c(ns)) {
                    if (!bfs_enqueue_c(ns, curr, &tail)) {
                        bfs_clear_states_c();
                        return;
                    }
                    next_level++;
                }
            }
        }

        level_left--;
        if (level_left == 0) {
            depth++;
            level_left = next_level;
            next_level = 0;
        }
    }

    bfs_clear_states_c();
}

// 2. ?? (???????)
static int best_bomb_plan_cost = 999999;
static TspObj best_bomb_plan_seq[MAX_BOMBS3];
static int best_bomb_plan_cnt = 0;

static void dfs_bomb_plan(int max_depth, int current_depth, int current_cost, Pos3 p_pos, Pos3* cur_bombs, int cur_nb, TspObj* task_seq) {
    if (current_cost >= best_bomb_plan_cost) return;

    // ??
    int solvable_cnt = get_solvable_boxes_c(p_pos, cur_bombs, cur_nb);
    int total_boxes = get_total_box_count(); 
    if (solvable_cnt == total_boxes) {
        if (check_is_solvable_c(p_pos, cur_bombs, cur_nb)) { 
            if (current_cost < best_bomb_plan_cost) {
                best_bomb_plan_cost = current_cost;
                best_bomb_plan_cnt = current_depth;
                memcpy(best_bomb_plan_seq, task_seq, sizeof(TspObj) * current_depth);
            }
            return; 
        }
    }
    
    if (current_depth == max_depth || cur_nb == 0) return;

    // ?????
    get_blocking_walls_c(cur_bombs, cur_nb); 

    for (int i = 0; i < cur_nb; i++) {
        Pos3 used_bomb = cur_bombs[i];
        get_all_reachable_walls_c(p_pos, used_bomb, cur_bombs, cur_nb);
        
        int c_cnt = num_reach_walls;
        ReachableWall cands[60];
        for(int k=0; k<c_cnt; k++) cands[k] = reach_walls[k];
        
        for (int c = 0; c < c_cnt; c++) {
            Pos3 wall = cands[c].wall;
            
            // ?? ???????<1ms
            bool is_useful = false;
            for (int ey = -1; ey <= 1; ey++) {
                for (int ex = -1; ex <= 1; ex++) {
                    int tx = wall.x+ex, ty = wall.y+ey;
                    for (int bw = 0; bw < num_blocking_walls; bw++) {
                        if (blocking_walls[bw].x == tx && blocking_walls[bw].y == ty) {
                            is_useful = true; break;
                        }
                    }
                    if (is_useful) break;
                }
                if (is_useful) break;
            }
            if (!is_useful) continue;

            // ?
            char backup_map[12][16]; memcpy(backup_map, dynamic_map, sizeof(dynamic_map));
            Pos3 backup_bombs[MAX_BOMBS3]; memcpy(backup_bombs, cur_bombs, sizeof(backup_bombs));
            
            for (int ey = -1; ey <= 1; ey++) {
                for (int ex = -1; ex <= 1; ex++) {
                    int tx = wall.x+ex, ty = wall.y+ey;
                    if (tx>0 && tx<15 && ty>0 && ty<11 && dynamic_map[ty][tx] == '#') dynamic_map[ty][tx] = '-';
                }
            }
            
            Pos3 next_p = cands[c].end_p;
            for(int k=i; k<cur_nb-1; k++) cur_bombs[k] = cur_bombs[k+1];
            
            task_seq[current_depth] = (TspObj){2, -1, used_bomb, '*', wall};
            
            dfs_bomb_plan(max_depth, current_depth + 1, current_cost + cands[c].path_len, next_p, cur_bombs, cur_nb - 1, task_seq);
            
            // ??
            memcpy(dynamic_map, backup_map, sizeof(dynamic_map));
            memcpy(cur_bombs, backup_bombs, sizeof(backup_bombs));
        }
    }
}

// ============================================================================
// Equal-length minimum-turn path helper
// ============================================================================
static const int level3_turn_dx3[4] = {0, 0, -1, 1};
static const int level3_turn_dy3[4] = {-1, 1, 0, 0};
static const char level3_turn_cmd3[4] = {'U', 'D', 'L', 'R'};

static int level3_turn_dir_from_cmd3(char cmd3)
{
    if (cmd3 == 'U' || cmd3 == 'u') return 0;
    if (cmd3 == 'D' || cmd3 == 'd') return 1;
    if (cmd3 == 'L' || cmd3 == 'l') return 2;
    if (cmd3 == 'R' || cmd3 == 'r') return 3;
    return -1;
}

static uint8_t level3_turn_cell3(int x3, int y3)
{
    return (uint8_t)((y3 << 4) | x3);
}

static void level3_turn_distance_bfs3(
    uint8_t start3, const uint8_t *blocked3, uint8_t *distance3)
{
    g_l3_recog_turn_bfs_calls3++;
    memset(distance3, 0xFF, LEVEL3_CELL_COUNT3);
    int head3 = 0;
    int tail3 = 0;
    turn_queue3[tail3++] = start3;
    distance3[start3] = 0;

    while (head3 < tail3) {
        uint8_t cell3 = turn_queue3[head3++];
        int x3 = cell3 & 0x0F;
        int y3 = cell3 >> 4;
        for (int d3 = 0; d3 < 4; d3++) {
            int nx3 = x3 + level3_turn_dx3[d3];
            int ny3 = y3 + level3_turn_dy3[d3];
            if (nx3 < 0 || nx3 >= 16 || ny3 < 0 || ny3 >= 12) continue;
            uint8_t next3 = level3_turn_cell3(nx3, ny3);
            if (blocked3[next3] || distance3[next3] != 0xFFu) continue;
            distance3[next3] = (uint8_t)(distance3[cell3] + 1u);
            turn_queue3[tail3++] = next3;
        }
    }
}

/* Find a shortest path and, among all paths with that same length, minimize
 * direction changes. prev_dir3 / next_dir3 may be -1. Boundary directions are
 * included in the turn count so adjacent walking and pushing segments join as
 * straight as possible. required_len3 >= 0 forbids any length change. */
static bool level3_min_turn_shortest_path3(
    Pos3 start3, Pos3 end3, const uint8_t *blocked_input3,
    int prev_dir3, int next_dir3, int required_len3,
    char *out_path3, int out_capacity3, int *out_len3)
{
    /* turn3 overlays the compact predecessor table.  Mark that table invalid
     * before using the shared bytes so the next full-state BFS reinitializes
     * them instead of reading stale turn-planner data. */
    bfs_invalidate_workspace3();
    if (blocked_input3 != turn_blocked3) {
        memcpy(turn_blocked3, blocked_input3, LEVEL3_CELL_COUNT3);
    }

    if (start3.x < 0 || start3.x >= 16 || start3.y < 0 || start3.y >= 12 ||
        end3.x < 0 || end3.x >= 16 || end3.y < 0 || end3.y >= 12) {
        return false;
    }

    uint8_t start_cell3 = level3_turn_cell3(start3.x, start3.y);
    uint8_t end_cell3 = level3_turn_cell3(end3.x, end3.y);
    turn_blocked3[start_cell3] = 0;
    turn_blocked3[end_cell3] = 0;

    if (start_cell3 == end_cell3) {
        if (required_len3 > 0 || out_capacity3 <= 0) return false;
        *out_len3 = 0;
        return true;
    }

    level3_turn_distance_bfs3(start_cell3, turn_blocked3, turn_dist_start3);
    if (turn_dist_start3[end_cell3] == 0xFFu) return false;
    int shortest3 = turn_dist_start3[end_cell3];
    if (required_len3 >= 0 && shortest3 != required_len3) return false;
    if (shortest3 >= out_capacity3) return false;

    level3_turn_distance_bfs3(end_cell3, turn_blocked3, turn_dist_end3);
    memset(turn_best3, 0xFF, LEVEL3_CELL_COUNT3 * 4u);
    memset(turn_next3, 0xFF, LEVEL3_CELL_COUNT3 * 4u);

    for (int previous3 = 0; previous3 < 4; previous3++) {
        turn_best3[(int)end_cell3 * 4 + previous3] =
            (uint8_t)((next_dir3 >= 0 && previous3 != next_dir3) ? 1 : 0);
    }

    for (int remaining3 = 1; remaining3 <= shortest3; remaining3++) {
        for (int cell_index3 = 0; cell_index3 < (int)LEVEL3_CELL_COUNT3; cell_index3++) {
            if (turn_dist_end3[cell_index3] != (uint8_t)remaining3) continue;
            if (turn_dist_start3[cell_index3] == 0xFFu ||
                (int)turn_dist_start3[cell_index3] + remaining3 != shortest3) {
                continue;
            }

            int x3 = cell_index3 & 0x0F;
            int y3 = cell_index3 >> 4;
            for (int previous3 = 0; previous3 < 4; previous3++) {
                uint8_t best3 = 0xFFu;
                uint8_t selected3 = 0xFFu;

                /* Trying the current direction first makes equal-cost ties
                 * favor a longer straight run. */
                for (int order3 = 0; order3 < 5; order3++) {
                    int d3 = (order3 == 0) ? previous3 : order3 - 1;
                    if (order3 > 0 && d3 == previous3) continue;
                    int nx3 = x3 + level3_turn_dx3[d3];
                    int ny3 = y3 + level3_turn_dy3[d3];
                    if (nx3 < 0 || nx3 >= 16 || ny3 < 0 || ny3 >= 12) continue;
                    uint8_t next3 = level3_turn_cell3(nx3, ny3);
                    if (turn_blocked3[next3]) continue;
                    if (turn_dist_end3[next3] != (uint8_t)(remaining3 - 1)) continue;
                    if (turn_dist_start3[next3] !=
                        (uint8_t)(turn_dist_start3[cell_index3] + 1u)) continue;

                    uint8_t suffix3 = turn_best3[(int)next3 * 4 + d3];
                    if (suffix3 == 0xFFu) continue;
                    uint8_t value3 = (uint8_t)(suffix3 + (previous3 != d3));
                    if (value3 < best3) {
                        best3 = value3;
                        selected3 = (uint8_t)d3;
                    }
                }

                turn_best3[cell_index3 * 4 + previous3] = best3;
                turn_next3[cell_index3 * 4 + previous3] = selected3;
            }
        }
    }

    uint8_t current3 = start_cell3;
    int current_prev3 = prev_dir3;
    int produced3 = 0;

    while (current3 != end_cell3) {
        int x3 = current3 & 0x0F;
        int y3 = current3 >> 4;
        int selected3 = -1;
        uint8_t selected_cost3 = 0xFFu;

        if (current_prev3 >= 0) {
            selected3 = turn_next3[(int)current3 * 4 + current_prev3];
        } else {
            for (int d3 = 0; d3 < 4; d3++) {
                int nx3 = x3 + level3_turn_dx3[d3];
                int ny3 = y3 + level3_turn_dy3[d3];
                if (nx3 < 0 || nx3 >= 16 || ny3 < 0 || ny3 >= 12) continue;
                uint8_t next3 = level3_turn_cell3(nx3, ny3);
                if (turn_blocked3[next3]) continue;
                if (turn_dist_start3[next3] != 1u ||
                    turn_dist_end3[next3] != (uint8_t)(shortest3 - 1)) continue;
                uint8_t value3 = turn_best3[(int)next3 * 4 + d3];
                if (value3 < selected_cost3) {
                    selected_cost3 = value3;
                    selected3 = d3;
                }
            }
        }

        if (selected3 < 0 || selected3 >= 4 || produced3 >= out_capacity3) {
            return false;
        }

        out_path3[produced3++] = level3_turn_cmd3[selected3];
        x3 += level3_turn_dx3[selected3];
        y3 += level3_turn_dy3[selected3];
        current3 = level3_turn_cell3(x3, y3);
        current_prev3 = selected3;
    }

    *out_len3 = produced3;
    return produced3 == shortest3;
}

// ============================================================================
// ?????? BFS ????? (?????)
// ============================================================================
static bool is_obstacle_c(int nx, int ny, bool is_target_passable) {
    if (nx < 0 || ny < 0 || nx >= 16 || ny >= 12) return true; // 
    char c = dynamic_map[ny][nx]; 
    if (c == '#') return true;
    if (c == '$' || (c >= 'A' && c <= 'Z')) return true; 
    if (is_pos_in_bombs(nx, ny, bombs, num_bombs)) return true;
    if (!is_target_passable) {
        if (c == '.' || (c >= 'a' && c <= 'z')) return true; 
    }
    return false;
}

static bool bfs_move_only_c(Pos3 start, Pos3 end, bool is_target_passable, int* out_len, char* out_path) {
    g_l3_recog_move_bfs_calls3++;
    for (int y3 = 0; y3 < 12; y3++) {
        for (int x3 = 0; x3 < 16; x3++) {
            turn_blocked3[level3_turn_cell3(x3, y3)] =
                is_obstacle_c(x3, y3, is_target_passable) ? 1u : 0u;
        }
    }

    return level3_min_turn_shortest_path3(
        start, end, turn_blocked3, -1, -1, -1,
        out_path, LEVEL3_INTERNAL_PATH_CAPACITY3, out_len);
}

// ============================================================================
// ???? TSP
// ============================================================================
static void get_rotation(char c_dir, char t_dir, char* rot_cmds, int* rot_len) {
    if (c_dir == t_dir) { *rot_len = 0; return; }
    char dirs[] = {'U', 'R', 'D', 'L'}; int c_idx = 0, t_idx = 0;
    for(int i=0; i<4; i++) { if(dirs[i] == c_dir) c_idx = i; if(dirs[i] == t_dir) t_idx = i; }
    int diff = (t_idx - c_idx + 4) % 4;
    if (diff == 1) { rot_cmds[0] = 'E'; *rot_len = 1; } 
    else if (diff == 2) { rot_cmds[0] = 'B'; *rot_len = 1; }
    else if (diff == 3) { rot_cmds[0] = 'Q'; *rot_len = 1; }
}

static bool level3_straighten_bomb_route3(
    Pos3 start_player3, Pos3 start_bomb3,
    const char *input3, int input_len3, char final_push3,
    char *output3, int output_capacity3)
{
    Pos3 player3 = start_player3;
    Pos3 bomb3 = start_bomb3;
    Pos3 segment_start3 = player3;
    int segment_start_index3 = 0;
    int output_len3 = 0;
    int previous_dir3 = -1;

    for (int i3 = 0; i3 <= input_len3; i3++) {
        bool at_end3 = (i3 == input_len3);
        char cmd3 = at_end3 ? final_push3 : input3[i3];
        int dir3 = level3_turn_dir_from_cmd3(cmd3);
        if (dir3 < 0) return false;

        bool is_push3 = false;
        if (!at_end3) {
            int nx3 = player3.x + level3_turn_dx3[dir3];
            int ny3 = player3.y + level3_turn_dy3[dir3];
            is_push3 = (nx3 == bomb3.x && ny3 == bomb3.y);
        }

        if (at_end3 || is_push3) {
            int raw_walk_len3 = i3 - segment_start_index3;
            int straight_len3 = 0;

            for (int y3 = 0; y3 < 12; y3++) {
                for (int x3 = 0; x3 < 16; x3++) {
                    char cell3 = dynamic_map[y3][x3];
                    turn_blocked3[level3_turn_cell3(x3, y3)] =
                        (cell3 == '#' || cell3 == '$' ||
                         (cell3 >= 'A' && cell3 <= 'Z')) ? 1u : 0u;
                }
            }
            for (int b3 = 0; b3 < num_bombs; b3++) {
                if (bombs[b3].x == start_bomb3.x &&
                    bombs[b3].y == start_bomb3.y) continue;
                turn_blocked3[level3_turn_cell3(bombs[b3].x, bombs[b3].y)] = 1u;
            }
            turn_blocked3[level3_turn_cell3(bomb3.x, bomb3.y)] = 1u;

            if (!level3_min_turn_shortest_path3(
                    segment_start3, player3, turn_blocked3,
                    previous_dir3, dir3, raw_walk_len3,
                    output3 + output_len3,
                    output_capacity3 - output_len3, &straight_len3)) {
                return false;
            }
            output_len3 += straight_len3;

            if (at_end3) break;
            if (output_len3 >= output_capacity3) return false;
            output3[output_len3++] = cmd3;

            player3.x += level3_turn_dx3[dir3];
            player3.y += level3_turn_dy3[dir3];
            bomb3.x += level3_turn_dx3[dir3];
            bomb3.y += level3_turn_dy3[dir3];
            previous_dir3 = dir3;
            segment_start3 = player3;
            segment_start_index3 = i3 + 1;
        } else {
            player3.x += level3_turn_dx3[dir3];
            player3.y += level3_turn_dy3[dir3];
        }
    }

    return output_len3 == input_len3;
}

static bool bfs_push_specific_bomb(Pos3 player, Pos3 bomb, Pos3 target_wall, int* out_len, char* out_path, char* out_last_dir, Pos3* out_end_p) {
    int head = 0, tail = 1;
    uint16_t start_state = bfs_full_state3(
        player.x, player.y, bomb.x, bomb.y);
    bfs_begin_c(start_state, BFS_STATE_PLAYER_BOX3);
    const int dx[4] = {0, 0, -1, 1};
    const int dy[4] = {-1, 1, 0, 0};
    const char dirs[4] = {'U', 'D', 'L', 'R'};

    while (head < tail) {
        uint16_t curr = bfs_queue[head++];
        int px, py, bx, by;
        bfs_decode_full_state3(curr, &px, &py, &bx, &by);

        for (int i = 0; i < 4; i++) {
            int npx = px + dx[i], npy = py + dy[i];
            if (npx < 0 || npy < 0 || npx >= 16 || npy >= 12) continue;
            char pc = dynamic_map[npy][npx];
            if (pc == '#' || pc == '$' || (pc >= 'A' && pc <= 'Z')) continue;

            bool p_hit_bomb = false;
            for (int k = 0; k < num_bombs; k++) {
                if (bombs[k].x == npx && bombs[k].y == npy &&
                    !(bombs[k].x == bomb.x && bombs[k].y == bomb.y)) {
                    p_hit_bomb = true; break;
                }
            }
            if (p_hit_bomb) continue;

            int nbx = bx, nby = by;
            if (npx == bx && npy == by) {
                nbx += dx[i]; nby += dy[i];
                if (nbx < 0 || nby < 0 || nbx >= 16 || nby >= 12) continue;
                char bc = dynamic_map[nby][nbx];
                if (bc == '#') {
                    if (nbx == target_wall.x && nby == target_wall.y) {
                        int sc = 0;
                        char raw3[LEVEL3_INTERNAL_PATH_CAPACITY3];
                        uint16_t c = curr;
                        while (c != start_state && sc < (int)sizeof(raw3)) {
                            uint16_t ps = bfs_parent_state_c(c);
                            raw3[sc++] = bfs_full_move_c(ps, c);
                            c = ps;
                        }
                        for (int k = 0; k < sc / 2; k++) {
                            char swap3 = raw3[k];
                            raw3[k] = raw3[sc - 1 - k];
                            raw3[sc - 1 - k] = swap3;
                        }
                        bfs_clear_states_c();

                        if (!level3_straighten_bomb_route3(
                                player, bomb, raw3, sc, dirs[i],
                                out_path, LEVEL3_INTERNAL_PATH_CAPACITY3)) {
                            memcpy(out_path, raw3, (size_t)sc);
                        }
                        *out_len = sc;
                        *out_last_dir = dirs[i];
                        *out_end_p = (Pos3){npx, npy};
                        return true;
                    }
                    continue;
                }
                if (bc == '$' || (bc >= 'A' && bc <= 'Z')) continue;
                bool b_hit_bomb = false;
                for (int k = 0; k < num_bombs; k++) {
                    if (bombs[k].x == nbx && bombs[k].y == nby &&
                        !(bombs[k].x == bomb.x && bombs[k].y == bomb.y)) {
                        b_hit_bomb = true; break;
                    }
                }
                if (b_hit_bomb) continue;
            }

            uint16_t next_state = bfs_full_state3(npx, npy, nbx, nby);
            if (!bfs_is_visited_c(next_state) &&
                !bfs_enqueue_c(next_state, curr, &tail)) {
                bfs_clear_states_c();
                return false;
            }
        }
    }

    bfs_clear_states_c();
    return false;
}

static void dfs_tsp(int curr_idx, Pos3 curr_p, char curr_dir, uint32_t unvisited_mask, int current_cost, char* current_path, int current_len) {
    g_l3_recog_legacy_dfs_calls3++;
    (void)curr_idx;
    if (current_cost >= global_best_cost) return; 
    if (unvisited_mask == 0) { global_best_cost = current_cost; global_best_len = current_len; memcpy(global_best_path, current_path, current_len); return; }
    
    int dx[] = {0, 0, -1, 1}; int dy[] = {-1, 1, 0, 0}; char view_dirs[] = {'D', 'U', 'R', 'L'}; 

    for (int i = 0; i < num_objs; i++) {
        if (unvisited_mask & (1 << i)) {
            if (all_objs[i].type == 2) { 
                bool prev_unvisited = false; 
                for (int j = 0; j < num_objs; j++) if (all_objs[j].type == 2 && all_objs[j].orig_idx < all_objs[i].orig_idx && (unvisited_mask & (1 << j))) prev_unvisited = true;
                if (prev_unvisited) continue;

                int p_len; char p_path[LEVEL3_INTERNAL_PATH_CAPACITY3]; char last_dir; Pos3 end_p; 
                if (bfs_push_specific_bomb(curr_p, all_objs[i].p, all_objs[i].target_wall, &p_len, p_path, &last_dir, &end_p)) {
                    Pos3 dest[9]; int nd = 0; int bx = all_objs[i].target_wall.x, by = all_objs[i].target_wall.y;
                    for (int ey = by - 1; ey <= by + 1; ey++) for (int ex = bx - 1; ex <= bx + 1; ex++) 
                        if (ex>0 && ex<15 && ey>0 && ey<11 && dynamic_map[ey][ex] == '#') { dynamic_map[ey][ex] = '-'; dest[nd++] = (Pos3){ex, ey}; }
                    
                    Pos3 orig_b = bombs[all_objs[i].orig_idx]; bombs[all_objs[i].orig_idx] = (Pos3){-1, -1};
                    if (current_len + p_len + 1 < LEVEL3_INTERNAL_PATH_CAPACITY3) {
                        memcpy(current_path + current_len, p_path, p_len); current_path[current_len + p_len] = last_dir;
                        dfs_tsp(i, end_p, curr_dir, unvisited_mask & ~(1 << i), current_cost + p_len + 1, current_path, current_len + p_len + 1);
                    }
                    for(int k=0; k<nd; k++) dynamic_map[dest[k].y][dest[k].x] = '#';
                    bombs[all_objs[i].orig_idx] = orig_b;
                }
            }  else { 
                int best_d = -1; int min_cost = 99999; int best_p_len = 0; char best_p_path[LEVEL3_INTERNAL_PATH_CAPACITY3]; int best_r_len = 0; char best_r_cmds[3];

                for (int d = 0; d < 4; d++) { 
                    int nx = all_objs[i].p.x + dx[d], ny = all_objs[i].p.y + dy[d];
                    if (is_obstacle_c(nx, ny, all_objs[i].type == 1)) continue;
                    
                    int tmp_len = 0;
                    char tmp_path[LEVEL3_INTERNAL_PATH_CAPACITY3]; 
                    bool is_reachable = bfs_move_only_c(curr_p, (Pos3){nx, ny}, all_objs[i].type == 1, &tmp_len, tmp_path);

                    if (is_reachable) {
                        char t_dir = view_dirs[d]; char r_cmds[3]; int r_len = 0; get_rotation(curr_dir, t_dir, r_cmds, &r_len);
                        int r_cost = (r_len > 0 && r_cmds[0] == 'B') ? 2 : r_len;
                        int total_cost = tmp_len + r_cost;
                        
                        if (total_cost < min_cost) {
                            min_cost = total_cost; best_d = d; best_p_len = tmp_len;
                            memcpy(best_p_path, tmp_path, tmp_len); best_r_len = r_len; memcpy(best_r_cmds, r_cmds, r_len);
                        }
                    }
                }
                
                if (best_d != -1 && current_len + best_p_len + best_r_len < LEVEL3_INTERNAL_PATH_CAPACITY3) {
                    char t_dir = view_dirs[best_d]; int nx = all_objs[i].p.x + dx[best_d], ny = all_objs[i].p.y + dy[best_d];
                    memcpy(current_path + current_len, best_p_path, best_p_len);
                    memcpy(current_path + current_len + best_p_len, best_r_cmds, best_r_len);
                    
                    int added_len = best_p_len + best_r_len;
                    if (added_len > 0) {
                        int last_idx = current_len + added_len - 1; char last_c = current_path[last_idx];
                        if (last_c >= 'A' && last_c <= 'Z') current_path[last_idx] = last_c + 32; 
                    } else {
                        current_path[current_len] = 'p'; added_len = 1; min_cost = 1; 
                    }
                    dfs_tsp(i, (Pos3){nx, ny}, t_dir, unvisited_mask & ~(1 << i), current_cost + min_cost, current_path, current_len + added_len);
                }
            }
        }
    }

}

/* -------------------------------------------------------------------------
 * Recognition-order optimizer
 * -------------------------------------------------------------------------
 * The legacy dfs_tsp() enumerates object permutations and runs a movement BFS
 * at every branch.  For 12 objects that approaches factorial work.  The DP
 * below precomputes all legal recognition poses and pairwise movement costs,
 * then solves states (recognized-object mask, last recognition pose).
 *
 * Storage is compact: for a mask, states exist only for poses belonging to an
 * object present in that mask.  With 12 objects and four poses each this is
 * 98,304 states instead of a dense 196,608-state table.  The workspace shares
 * the existing OCRAM union with level 1/2 and therefore does not add a second
 * large permanent buffer.
 */
#define LEVEL3_RECOG_INF3          0xFFFFu
#define LEVEL3_RECOG_PARENT_START3 0xFEu
#define LEVEL3_RECOG_PARENT_NONE3  0xFFu

#define level3_recog_ws3 (g_sokoban_workspace.level3.recognition)

static uint8_t level3_recog_popcount3(uint32_t value3)
{
    uint8_t count3 = 0U;
    while (value3 != 0U) {
        value3 &= value3 - 1U;
        count3++;
    }
    return count3;
}

static int level3_recog_dir_index3(char dir3)
{
    if (dir3 == 'U') return 0;
    if (dir3 == 'R') return 1;
    if (dir3 == 'D') return 2;
    if (dir3 == 'L') return 3;
    return -1;
}

static uint16_t level3_recog_rotation_cost3(char from3, char to3,
                                             char *rot3, int *rot_len3)
{
    get_rotation(from3, to3, rot3, rot_len3);
    if (*rot_len3 > 0 && rot3[0] == 'B') return 2U;
    return (uint16_t)(*rot_len3);
}

static uint32_t level3_recog_state_index3(uint32_t mask3, uint8_t pose3)
{
    const SokobanRecognitionPose3 *p3 = &level3_recog_ws3.poses[pose3];
    uint32_t lower_mask3 = mask3 & ((1u << p3->object_index) - 1u);
    uint32_t lower_pose_count3 =
        level3_recog_ws3.mask_base[lower_mask3 + 1u] -
        level3_recog_ws3.mask_base[lower_mask3];
    return level3_recog_ws3.mask_base[mask3] + lower_pose_count3 +
           p3->local_index;
}

/* Build shortest translation distances from one source to every cell.  The
 * recognition DP uses only path length in its edge costs; minimum-turn path
 * reconstruction is deferred until the final 11-pose sequence is known. */
static void level3_recog_build_distance_map3(Pos3 start3,
                                              bool target_passable3)
{
    g_l3_recog_move_bfs_calls3++;
    bfs_invalidate_workspace3();
    for (int y3 = 0; y3 < 12; y3++) {
        for (int x3 = 0; x3 < 16; x3++) {
            turn_blocked3[level3_turn_cell3(x3, y3)] =
                is_obstacle_c(x3, y3, target_passable3) ? 1u : 0u;
        }
    }
    uint8_t start_cell3 = level3_turn_cell3(start3.x, start3.y);
    turn_blocked3[start_cell3] = 0U;
    level3_turn_distance_bfs3(start_cell3, turn_blocked3, turn_dist_start3);
}

static uint16_t level3_recog_cost_from_distance3(char from_dir3,
                                                  uint8_t to_pose3)
{
    const SokobanRecognitionPose3 *pose3 = &level3_recog_ws3.poses[to_pose3];
    uint8_t path_len3 = turn_dist_start3[pose3->stand_cell];
    if (path_len3 == 0xFFU) return LEVEL3_RECOG_INF3;

    char rot3[3];
    int rot_len3 = 0;
    uint16_t rotation_cost3 = level3_recog_rotation_cost3(
        from_dir3, pose3->view_dir, rot3, &rot_len3);
    if (path_len3 == 0U && rot_len3 == 0) return 1U;
    return (uint16_t)path_len3 + rotation_cost3;
}

static bool level3_recog_build_poses3(void)
{
    static const int dx3[4] = {0, 0, -1, 1};
    static const int dy3[4] = {-1, 1, 0, 0};
    static const char view3[4] = {'D', 'U', 'R', 'L'};
    uint8_t total3 = 0U;

    if (num_objs <= 0 || num_objs > SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY) {
        g_l3_recog_result3 = -101;
        return false;
    }

    memset(level3_recog_ws3.object_pose_start, 0,
           sizeof(level3_recog_ws3.object_pose_start));
    memset(level3_recog_ws3.object_pose_count, 0,
           sizeof(level3_recog_ws3.object_pose_count));

    for (int object3 = 0; object3 < num_objs; object3++) {
        level3_recog_ws3.object_pose_start[object3] = total3;
        for (uint8_t local3 = 0U; local3 < 4U; local3++) {
            int sx3 = all_objs[object3].p.x + dx3[local3];
            int sy3 = all_objs[object3].p.y + dy3[local3];
            bool target_passable3 = all_objs[object3].type == 1;
            if (is_obstacle_c(sx3, sy3, target_passable3)) continue;
            if (total3 >= SOKOBAN_LEVEL3_RECOG_MAX_POSES) {
                g_l3_recog_result3 = -102;
                return false;
            }

            level3_recog_ws3.poses[total3].object_index = (uint8_t)object3;
            level3_recog_ws3.poses[total3].local_index =
                level3_recog_ws3.object_pose_count[object3];
            level3_recog_ws3.poses[total3].stand_cell =
                (uint8_t)(sy3 * 16 + sx3);
            level3_recog_ws3.poses[total3].view_dir = view3[local3];
            level3_recog_ws3.object_pose_count[object3]++;
            total3++;
        }

        if (level3_recog_ws3.object_pose_count[object3] == 0U) {
            g_l3_recog_result3 = -103;
            return false;
        }
    }

    g_l3_recog_pose_count3 = total3;
    return true;
}

static bool level3_recog_build_compact_layout3(uint32_t full_mask3)
{
    uint32_t next_base3 = 0U;
    level3_recog_ws3.mask_base[0] = 0U;

    for (uint32_t mask3 = 0U; mask3 <= full_mask3; mask3++) {
        uint32_t states_in_mask3 = 0U;
        for (int object3 = 0; object3 < num_objs; object3++) {
            if ((mask3 & (1u << object3)) != 0U) {
                states_in_mask3 +=
                    level3_recog_ws3.object_pose_count[object3];
            }
        }
        next_base3 += states_in_mask3;
        level3_recog_ws3.mask_base[mask3 + 1U] = next_base3;
    }

    if (next_base3 > SOKOBAN_LEVEL3_RECOG_MAX_STATES) {
        g_l3_recog_result3 = -104;
        return false;
    }
    g_l3_recog_compact_state_capacity3 = next_base3;
    memset(level3_recog_ws3.dp_cost, 0xFF,
           next_base3 * sizeof(level3_recog_ws3.dp_cost[0]));
    memset(level3_recog_ws3.dp_parent_pose, LEVEL3_RECOG_PARENT_NONE3,
           next_base3 * sizeof(level3_recog_ws3.dp_parent_pose[0]));
    return true;
}

static bool level3_recog_precompute_costs3(Pos3 start3, char start_dir3)
{
    uint8_t pose_count3 = (uint8_t)g_l3_recog_pose_count3;

    for (int pass3 = 0; pass3 < 2; pass3++) {
        level3_recog_build_distance_map3(start3, pass3 != 0);
        for (uint8_t to3 = 0U; to3 < pose_count3; to3++) {
            bool target_passable3 =
                all_objs[level3_recog_ws3.poses[to3].object_index].type == 1;
            if (target_passable3 == (pass3 != 0)) {
                level3_recog_ws3.start_cost[to3] =
                    level3_recog_cost_from_distance3(start_dir3, to3);
            }
        }
    }

    for (uint8_t from3 = 0U; from3 < pose_count3; from3++) {
        const SokobanRecognitionPose3 *source3 =
            &level3_recog_ws3.poses[from3];
        Pos3 source_pos3 = {(int)(source3->stand_cell & 0x0Fu),
                            (int)(source3->stand_cell >> 4)};

        for (int pass3 = 0; pass3 < 2; pass3++) {
            level3_recog_build_distance_map3(source_pos3, pass3 != 0);
            for (uint8_t to3 = 0U; to3 < pose_count3; to3++) {
                const SokobanRecognitionPose3 *dest3 =
                    &level3_recog_ws3.poses[to3];
                bool target_passable3 =
                    all_objs[dest3->object_index].type == 1;
                if (target_passable3 != (pass3 != 0)) continue;
                if (source3->object_index == dest3->object_index) {
                    level3_recog_ws3.edge_cost[from3][to3] =
                        LEVEL3_RECOG_INF3;
                } else {
                    level3_recog_ws3.edge_cost[from3][to3] =
                        level3_recog_cost_from_distance3(
                            source3->view_dir, to3);
                }
            }
        }
    }
    return true;
}

static bool level3_recog_rebuild_path3(Pos3 start3, char start_dir3,
                                        uint8_t sequence_len3)
{
    Pos3 current3 = start3;
    char current_dir3 = start_dir3;
    int output_len3 = 0;

    for (uint8_t step3 = 0U; step3 < sequence_len3; step3++) {
        uint8_t pose_index3 = level3_recog_ws3.chosen_pose_sequence[step3];
        const SokobanRecognitionPose3 *pose3 =
            &level3_recog_ws3.poses[pose_index3];
        Pos3 stand3 = {(int)(pose3->stand_cell & 0x0Fu),
                       (int)(pose3->stand_cell >> 4)};
        int path_len3 = 0;
        char rotation3[3];
        int rotation_len3 = 0;
        bool target_passable3 = all_objs[pose3->object_index].type == 1;

        if (!bfs_move_only_c(current3, stand3, target_passable3,
                             &path_len3,
                             level3_recog_ws3.path_scratch)) {
            g_l3_recog_result3 = -105;
            return false;
        }
        (void)level3_recog_rotation_cost3(
            current_dir3, pose3->view_dir, rotation3, &rotation_len3);

        int command_len3 = path_len3 + rotation_len3;
        int append_len3 = command_len3 == 0 ? 1 : command_len3;
        if (output_len3 + append_len3 >= LEVEL3_INTERNAL_PATH_CAPACITY3) {
            g_l3_recog_result3 = -106;
            return false;
        }

        if (command_len3 == 0) {
            global_best_path[output_len3++] = 'p';
        } else {
            if (path_len3 > 0) {
                memcpy(global_best_path + output_len3,
                       level3_recog_ws3.path_scratch, (size_t)path_len3);
                output_len3 += path_len3;
            }
            if (rotation_len3 > 0) {
                memcpy(global_best_path + output_len3,
                       rotation3, (size_t)rotation_len3);
                output_len3 += rotation_len3;
            }
            char final3 = global_best_path[output_len3 - 1];
            if (final3 >= 'A' && final3 <= 'Z') {
                global_best_path[output_len3 - 1] =
                    (char)(final3 + ('a' - 'A'));
            }
        }

        current3 = stand3;
        current_dir3 = pose3->view_dir;
    }

    global_best_len = output_len3;
    return true;
}

static bool level3_plan_recognition_dp3(Pos3 start3, char start_dir3,
                                         int required_objects3)
{
    uint32_t full_mask3;
    uint8_t pose_count3;
    uint16_t best_cost3 = LEVEL3_RECOG_INF3;
    uint32_t best_mask3 = 0U;
    uint8_t best_pose3 = LEVEL3_RECOG_PARENT_NONE3;

    g_l3_recog_legacy_dfs_calls3 = 0U;
    g_l3_recog_move_bfs_calls3 = 0U;
    g_l3_recog_turn_bfs_calls3 = 0U;
    g_l3_recog_dp_states3 = 0U;
    g_l3_recog_dp_transitions3 = 0U;
    g_l3_recog_pose_count3 = 0U;
    g_l3_recog_compact_state_capacity3 = 0U;
    g_l3_recog_best_cost3 = -1;
    g_l3_recog_result3 = 1;

    global_best_cost = 999999;
    global_best_len = 0;

    if (required_objects3 <= 0 || required_objects3 > num_objs ||
        num_objs > SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS) {
        g_l3_recog_result3 = -107;
        return false;
    }
    if (level3_recog_dir_index3(start_dir3) < 0) {
        g_l3_recog_result3 = -108;
        return false;
    }
    if (!level3_recog_build_poses3()) return false;

    full_mask3 = (1u << num_objs) - 1u;
    if (!level3_recog_build_compact_layout3(full_mask3)) return false;
    if (!level3_recog_precompute_costs3(start3, start_dir3)) return false;
    pose_count3 = (uint8_t)g_l3_recog_pose_count3;

    for (uint8_t pose3 = 0U; pose3 < pose_count3; pose3++) {
        uint16_t cost3 = level3_recog_ws3.start_cost[pose3];
        if (cost3 == LEVEL3_RECOG_INF3) continue;
        uint32_t mask3 = 1u << level3_recog_ws3.poses[pose3].object_index;
        uint32_t index3 = level3_recog_state_index3(mask3, pose3);
        if (cost3 < level3_recog_ws3.dp_cost[index3]) {
            level3_recog_ws3.dp_cost[index3] = cost3;
            level3_recog_ws3.dp_parent_pose[index3] =
                LEVEL3_RECOG_PARENT_START3;
            g_l3_recog_dp_states3++;
        }
    }

    for (uint32_t mask3 = 1U; mask3 <= full_mask3; mask3++) {
        uint8_t depth3 = level3_recog_popcount3(mask3);
        if (depth3 >= (uint8_t)required_objects3) continue;

        for (uint8_t last3 = 0U; last3 < pose_count3; last3++) {
            uint8_t last_object3 =
                level3_recog_ws3.poses[last3].object_index;
            if ((mask3 & (1u << last_object3)) == 0U) continue;
            uint32_t state3 = level3_recog_state_index3(mask3, last3);
            uint16_t current_cost3 = level3_recog_ws3.dp_cost[state3];
            if (current_cost3 == LEVEL3_RECOG_INF3) continue;

            for (uint8_t next_object3 = 0U;
                 next_object3 < (uint8_t)num_objs; next_object3++) {
                uint32_t next_bit3 = 1u << next_object3;
                if ((mask3 & next_bit3) != 0U) continue;

                uint8_t first_pose3 =
                    level3_recog_ws3.object_pose_start[next_object3];
                uint8_t count3 =
                    level3_recog_ws3.object_pose_count[next_object3];
                uint32_t next_mask3 = mask3 | next_bit3;
                uint32_t lower_mask3 =
                    mask3 & ((1u << next_object3) - 1u);
                uint32_t next_state_base3 = level3_recog_ws3.mask_base[next_mask3] +
                    (level3_recog_ws3.mask_base[lower_mask3 + 1u] -
                     level3_recog_ws3.mask_base[lower_mask3]);
                for (uint8_t local3 = 0U; local3 < count3; local3++) {
                    uint8_t next_pose3 = (uint8_t)(first_pose3 + local3);
                    uint16_t edge3 =
                        level3_recog_ws3.edge_cost[last3][next_pose3];
                    g_l3_recog_dp_transitions3++;
                    if (edge3 == LEVEL3_RECOG_INF3) continue;
                    uint32_t sum3 = (uint32_t)current_cost3 + edge3;
                    if (sum3 >= LEVEL3_RECOG_INF3) continue;

                    uint32_t next_state3 = next_state_base3 + local3;
                    uint16_t old3 = level3_recog_ws3.dp_cost[next_state3];
                    if (sum3 < old3 ||
                        (sum3 == old3 && last3 <
                         level3_recog_ws3.dp_parent_pose[next_state3])) {
                        if (old3 == LEVEL3_RECOG_INF3) {
                            g_l3_recog_dp_states3++;
                        }
                        level3_recog_ws3.dp_cost[next_state3] =
                            (uint16_t)sum3;
                        level3_recog_ws3.dp_parent_pose[next_state3] = last3;
                    }
                }
            }
        }
    }

    /* Preserve legacy drop priority: drop object 0 first, then 1, ... . */
    if (required_objects3 == num_objs) {
        for (uint8_t pose3 = 0U; pose3 < pose_count3; pose3++) {
            uint32_t state3 = level3_recog_state_index3(full_mask3, pose3);
            uint16_t cost3 = level3_recog_ws3.dp_cost[state3];
            if (cost3 < best_cost3) {
                best_cost3 = cost3;
                best_mask3 = full_mask3;
                best_pose3 = pose3;
            }
        }
    } else {
        for (uint8_t drop3 = 0U; drop3 < (uint8_t)num_objs; drop3++) {
            uint32_t mask3 = full_mask3 & ~(1u << drop3);
            for (uint8_t pose3 = 0U; pose3 < pose_count3; pose3++) {
                uint8_t object3 =
                    level3_recog_ws3.poses[pose3].object_index;
                if ((mask3 & (1u << object3)) == 0U) continue;
                uint32_t state3 = level3_recog_state_index3(mask3, pose3);
                uint16_t cost3 = level3_recog_ws3.dp_cost[state3];
                if (cost3 < best_cost3) {
                    best_cost3 = cost3;
                    best_mask3 = mask3;
                    best_pose3 = pose3;
                }
            }
        }
    }

    if (best_pose3 == LEVEL3_RECOG_PARENT_NONE3 ||
        best_cost3 == LEVEL3_RECOG_INF3) {
        g_l3_recog_result3 = -109;
        return false;
    }

    uint32_t trace_mask3 = best_mask3;
    uint8_t trace_pose3 = best_pose3;
    for (int step3 = required_objects3 - 1; step3 >= 0; step3--) {
        if (trace_pose3 >= pose_count3) {
            g_l3_recog_result3 = -110;
            return false;
        }
        level3_recog_ws3.chosen_pose_sequence[step3] = trace_pose3;
        uint32_t state3 =
            level3_recog_state_index3(trace_mask3, trace_pose3);
        uint8_t parent3 = level3_recog_ws3.dp_parent_pose[state3];
        trace_mask3 &=
            ~(1u << level3_recog_ws3.poses[trace_pose3].object_index);

        if (step3 == 0) {
            if (parent3 != LEVEL3_RECOG_PARENT_START3 || trace_mask3 != 0U) {
                g_l3_recog_result3 = -111;
                return false;
            }
        } else {
            if (parent3 >= pose_count3) {
                g_l3_recog_result3 = -112;
                return false;
            }
            trace_pose3 = parent3;
        }
    }

    if (!level3_recog_rebuild_path3(
            start3, start_dir3, (uint8_t)required_objects3)) {
        return false;
    }

    global_best_cost = best_cost3;
    g_l3_recog_best_cost3 = best_cost3;
    g_l3_recog_result3 = 0;
    return true;
}

/* Fixed-memory recognition mode for scenes beyond the exact 12-object
 * subset DP.  The current map still decides num_objs; the compile-time value
 * is only an upper storage bound.  Each step evaluates both obstacle modes
 * (box-view / target-view), then the selected route is rebuilt by the original
 * minimum-turn BFS so command semantics remain unchanged. */
static bool level3_plan_recognition_scalable3(Pos3 start3, char start_dir3,
                                               int required_objects3)
{
    uint32_t visited3 = 0U;
    Pos3 current3 = start3;
    char current_dir3 = start_dir3;
    uint32_t total_cost3 = 0U;

    g_l3_recog_legacy_dfs_calls3 = 0U;
    g_l3_recog_move_bfs_calls3 = 0U;
    g_l3_recog_turn_bfs_calls3 = 0U;
    g_l3_recog_dp_states3 = 0U;
    g_l3_recog_dp_transitions3 = 0U;
    g_l3_recog_pose_count3 = 0U;
    g_l3_recog_compact_state_capacity3 = 0U;
    g_l3_recog_best_cost3 = -1;
    g_l3_recog_result3 = 1;
    global_best_cost = 999999;
    global_best_len = 0;

    if (required_objects3 <= 0 || required_objects3 > num_objs ||
        num_objs > SOKOBAN_LEVEL3_RECOG_OBJECT_CAPACITY || num_objs > 32 ||
        level3_recog_dir_index3(start_dir3) < 0) {
        g_l3_recog_result3 = -113;
        return false;
    }
    if (!level3_recog_build_poses3()) return false;

    for (int step3 = 0; step3 < required_objects3; step3++) {
        uint16_t best_cost3 = LEVEL3_RECOG_INF3;
        uint8_t best_pose3 = LEVEL3_RECOG_PARENT_NONE3;
        uint8_t pose_count3 = (uint8_t)g_l3_recog_pose_count3;

        for (int pass3 = 0; pass3 < 2; pass3++) {
            level3_recog_build_distance_map3(current3, pass3 != 0);
            for (uint8_t pose3 = 0U; pose3 < pose_count3; pose3++) {
                const SokobanRecognitionPose3 *candidate3 =
                    &level3_recog_ws3.poses[pose3];
                uint8_t object3 = candidate3->object_index;
                uint32_t bit3 = 1u << object3;
                if ((visited3 & bit3) != 0U) continue;
                bool target_passable3 = all_objs[object3].type == 1;
                if (target_passable3 != (pass3 != 0)) continue;
                uint16_t cost3 = level3_recog_cost_from_distance3(
                    current_dir3, pose3);
                g_l3_recog_dp_transitions3++;
                if (cost3 < best_cost3 ||
                    (cost3 == best_cost3 && pose3 < best_pose3)) {
                    best_cost3 = cost3;
                    best_pose3 = pose3;
                }
            }
        }

        if (best_pose3 == LEVEL3_RECOG_PARENT_NONE3 ||
            best_cost3 == LEVEL3_RECOG_INF3) {
            g_l3_recog_result3 = -114;
            return false;
        }
        level3_recog_ws3.chosen_pose_sequence[step3] = best_pose3;
        const SokobanRecognitionPose3 *chosen3 =
            &level3_recog_ws3.poses[best_pose3];
        visited3 |= 1u << chosen3->object_index;
        current3.x = (int)(chosen3->stand_cell & 0x0Fu);
        current3.y = (int)(chosen3->stand_cell >> 4);
        current_dir3 = chosen3->view_dir;
        total_cost3 += best_cost3;
        g_l3_recog_dp_states3++;
    }

    if (!level3_recog_rebuild_path3(
            start3, start_dir3, (uint8_t)required_objects3)) {
        return false;
    }
    if (total_cost3 > 0x7FFFFFFFU) total_cost3 = 0x7FFFFFFFU;
    global_best_cost = (int)total_cost3;
    g_l3_recog_best_cost3 = (int32_t)total_cost3;
    g_l3_recog_result3 = 2;
    return true;
}

// === ????????W ===
static int bfs_push_single_box_fixed_c(Pos3 player, Pos3 box, Pos3 target, Pos3* other_boxes, int num_other_boxes, Pos3* cur_bombs, int num_b) {
    int head = 0, tail = 1;
    uint16_t start_state = bfs_full_state3(
        player.x, player.y, box.x, box.y);
    bfs_begin_c(start_state, BFS_STATE_PLAYER_BOX3);
    const int dx[4] = {0, 0, -1, 1};
    const int dy[4] = {-1, 1, 0, 0};
    int search_cnt = 0;

    while (head < tail) {
        if (++search_cnt > 20000) {
            bfs_clear_states_c();
            return -1;
        }

        uint16_t curr = bfs_queue[head++];
        int px, py, bx, by;
        bfs_decode_full_state3(curr, &px, &py, &bx, &by);
        if (bx == target.x && by == target.y) return (int)curr;

        for (int i = 0; i < 4; i++) {
            int npx = px + dx[i], npy = py + dy[i];
            if (npx < 0 || npy < 0 || npx >= 16 || npy >= 12) continue;
            if (current_map[npy][npx] == '#') continue;

            bool p_hit_obs = false;
            for (int k = 0; k < num_other_boxes; k++)
                if (other_boxes[k].x == npx && other_boxes[k].y == npy) p_hit_obs = true;
            for (int k = 0; k < num_b; k++)
                if (cur_bombs[k].x == npx && cur_bombs[k].y == npy) p_hit_obs = true;
            if (p_hit_obs) continue;

            int nbx = bx, nby = by;
            if (npx == bx && npy == by) {
                nbx += dx[i]; nby += dy[i];
                if (nbx < 0 || nby < 0 || nbx >= 16 || nby >= 12) continue;
                if (current_map[nby][nbx] == '#') continue;

                bool b_hit_obs = false;
                for (int k = 0; k < num_other_boxes; k++)
                    if (other_boxes[k].x == nbx && other_boxes[k].y == nby) b_hit_obs = true;
                for (int k = 0; k < num_b; k++)
                    if (cur_bombs[k].x == nbx && cur_bombs[k].y == nby) b_hit_obs = true;
                if (b_hit_obs) continue;
            }

            uint16_t next_state = bfs_full_state3(npx, npy, nbx, nby);
            if (!bfs_is_visited_c(next_state) &&
                !bfs_enqueue_c(next_state, curr, &tail)) {
                bfs_clear_states_c();
                return -1;
            }
        }
    }

    bfs_clear_states_c();
    return -1;
}

static void evaluate_push_order(int *order, Pos3 start_p, Pos3* cur_bombs, int cur_nb) {
    int current_steps = 0; static char temp_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    Pos3 p_pos = start_p; Pos3 b_pos[MAX_BOXES3];
    for(int i = 0; i < num_dyn_pairs; i++) b_pos[i] = dyn_pairs[i].box_pos;

    for (int i = 0; i < num_dyn_pairs; i++) {
        int pair_idx = order[i]; 
        memcpy(current_map, dynamic_map, sizeof(current_map));
        
        for(int r = 0; r < 12; r++) {
            for(int c = 0; c < 16; c++) { 
                char ch = current_map[r][c]; 
                if((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || ch == '$' || ch == '.' || ch == '@') current_map[r][c] = '-'; 
            }
        }
        
        Pos3 other_boxes[MAX_BOXES3];
        int num_ob = 0;
        for (int j = 0; j < num_dyn_pairs; j++) {
            if (j == pair_idx) continue;

            /* Only boxes that still exist are obstacles. */
            if (!is_box_removed3(b_pos[j])) {
                other_boxes[num_ob++] = b_pos[j];
            }
        }

        int state = bfs_push_single_box_fixed_c(p_pos, b_pos[pair_idx], dyn_pairs[pair_idx].target_pos, other_boxes, num_ob, cur_bombs, cur_nb);
        if (state == -1) return; 

        uint16_t curr = state; int sc = 0; static char local_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
        uint16_t s_st = bfs_full_state3(
            p_pos.x, p_pos.y, b_pos[pair_idx].x, b_pos[pair_idx].y);
        
        while (curr != s_st) {
            if (sc >= LEVEL3_INTERNAL_PATH_CAPACITY3) {
                bfs_clear_states_c();
                return;
            }
            uint16_t parent_state = bfs_parent_state_c(curr);
            local_path[sc++] = bfs_full_move_c(parent_state, curr);
            curr = parent_state;
        }
        if (current_steps + sc >= LEVEL3_INTERNAL_PATH_CAPACITY3) {
            bfs_clear_states_c();
            return;
        }
        for (int k = 0; k < sc; k++) {
            temp_path[current_steps + k] = local_path[sc - 1 - k];
        }
        bfs_clear_states_c();
        current_steps += sc;

        if (current_steps >= best_push_steps) return; 
        
        {
            int bx3, by3;
            bfs_decode_full_state3((uint16_t)state,
                &p_pos.x, &p_pos.y, &bx3, &by3);
        } 
        /* Remove the box explicitly after it reaches the target. */
        b_pos[pair_idx] = PUSH_BOX_REMOVED3;
    }
    
    if (current_steps < best_push_steps) { 
        best_push_steps = current_steps; 
        memcpy(best_push_path, temp_path, current_steps); 
    }
}

static void swap_int(int *a, int *b) { int temp = *a; *a = *b; *b = temp; }

static void permute_push_order(int *order, int start, int end, Pos3 start_p, Pos3* cur_bombs, int cur_nb) {
    if (start == end) { evaluate_push_order(order, start_p, cur_bombs, cur_nb); return; }
    for (int i = start; i <= end; i++) {
        swap_int(&order[start], &order[i]);
        permute_push_order(order, start + 1, end, start_p, cur_bombs, cur_nb);
        swap_int(&order[start], &order[i]);
    }
}



// ============================================================================
// ?????????????? A ????????????? a ???
// ?????ß’??????????? A ???????? a?????????????????????????
// ?????????????????????????????????????????????????????????
// ???????????????????????????? A / ????? a??
// ============================================================================
#ifndef MAX_DUP_ASSIGNMENTS
#define MAX_DUP_ASSIGNMENTS 120
#endif

static int dup_group_start[SOKOBAN_ELEMENT_TYPE_CAPACITY];
static int dup_group_count[SOKOBAN_ELEMENT_TYPE_CAPACITY];
static int duplicate_assignment_eval_count = 0;
int solver_duplicate_element_group_count = 0;

static void swap_pos_c(Pos3 *a, Pos3 *b) {
    Pos3 t = *a;
    *a = *b;
    *b = t;
}

static bool build_dyn_pairs_duplicate_aware_c(void) {
    // ?? static ???? RTOS ?????????? Pos3 ????????
    static Pos3 boxes_by_label[SOKOBAN_ELEMENT_TYPE_CAPACITY][MAX_BOXES3];
    static Pos3 targets_by_label[SOKOBAN_ELEMENT_TYPE_CAPACITY][MAX_BOXES3];
    int bc[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};
    int tc[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};

    for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        dup_group_start[i] = -1;
        dup_group_count[i] = 0;
    }
    solver_duplicate_element_group_count = 0;

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (c >= 'A' && c <= 'Z') {
                int idx = c - 'A';
                if (bc[idx] >= MAX_BOXES3) return false;
                boxes_by_label[idx][bc[idx]++] = (Pos3){x, y};
            } else if (c >= 'a' && c <= 'z') {
                int idx = c - 'a';
                if (tc[idx] >= MAX_BOXES3) return false;
                targets_by_label[idx][tc[idx]++] = (Pos3){x, y};
            }
        }
    }

    num_dyn_pairs = 0;
    for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        if (bc[i] != tc[i]) {
            return false;
        }
        if (bc[i] <= 0) continue;
        if (bc[i] > 1) solver_duplicate_element_group_count++;

        dup_group_start[i] = num_dyn_pairs;
        dup_group_count[i] = bc[i];

        for (int k = 0; k < bc[i]; k++) {
            if (num_dyn_pairs >= MAX_BOXES3) return false;
            dyn_pairs[num_dyn_pairs].label = (char)('A' + i);
            dyn_pairs[num_dyn_pairs].box_pos = boxes_by_label[i][k];
            // ???????????????????????????????? target_pos????????????
            dyn_pairs[num_dyn_pairs].target_pos = targets_by_label[i][k];
            num_dyn_pairs++;
        }
    }
    return true;
}

static void evaluate_target_assignment_rec_c(int label_idx, Pos3 start_p, Pos3* cur_bombs, int cur_nb);

static void permute_targets_in_group_c(int label_idx, int pos, int end, Pos3 start_p, Pos3* cur_bombs, int cur_nb) {
    if (duplicate_assignment_eval_count >= MAX_DUP_ASSIGNMENTS) return;

    if (pos >= end) {
        evaluate_target_assignment_rec_c(label_idx + 1, start_p, cur_bombs, cur_nb);
        return;
    }

    for (int i = pos; i <= end; i++) {
        swap_pos_c(&dyn_pairs[pos].target_pos, &dyn_pairs[i].target_pos);
        permute_targets_in_group_c(label_idx, pos + 1, end, start_p, cur_bombs, cur_nb);
        swap_pos_c(&dyn_pairs[pos].target_pos, &dyn_pairs[i].target_pos);
        if (duplicate_assignment_eval_count >= MAX_DUP_ASSIGNMENTS) return;
    }
}

static void evaluate_target_assignment_rec_c(int label_idx, Pos3 start_p, Pos3* cur_bombs, int cur_nb) {
    if (duplicate_assignment_eval_count >= MAX_DUP_ASSIGNMENTS) return;

    if (label_idx >= SOKOBAN_ELEMENT_TYPE_CAPACITY) {
        int order[MAX_BOXES3];
        for (int i = 0; i < num_dyn_pairs; i++) order[i] = i;
        duplicate_assignment_eval_count++;
        permute_push_order(order, 0, num_dyn_pairs - 1, start_p, cur_bombs, cur_nb);
        return;
    }

    int st = dup_group_start[label_idx];
    int cnt = dup_group_count[label_idx];
    if (st < 0 || cnt <= 1) {
        evaluate_target_assignment_rec_c(label_idx + 1, start_p, cur_bombs, cur_nb);
    } else {
        permute_targets_in_group_c(label_idx, st, st + cnt - 1, start_p, cur_bombs, cur_nb);
    }
}

static void evaluate_all_duplicate_target_assignments_c(Pos3 start_p, Pos3* cur_bombs, int cur_nb) {
    duplicate_assignment_eval_count = 0;
    if (num_dyn_pairs <= 0) return;
    evaluate_target_assignment_rec_c(0, start_p, cur_bombs, cur_nb);
}

static bool check_is_solvable_c(Pos3 p_pos, Pos3* cur_bombs, int cur_nb) {
    char backup_map[12][16];
    memcpy(backup_map, dynamic_map, sizeof(dynamic_map));
    int saved_num_dyn_pairs = num_dyn_pairs;
    DynBoxTargetPair saved_pairs[MAX_BOXES3];
    memcpy(saved_pairs, dyn_pairs, sizeof(dyn_pairs));
    int saved_best = best_push_steps;

    int b_cnt[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0}, t_cnt[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};
    for(int y=0; y<12; y++) {
        for(int x=0; x<16; x++) {
            char c = dynamic_map[y][x];
            if(c >= 'A' && c <= 'Z') b_cnt[c - 'A']++;
            if(c >= 'a' && c <= 'z') t_cnt[c - 'a']++;
        }
    }
    for(int i=0; i<SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        while(b_cnt[i] > t_cnt[i]) {
            bool filled = false;
            for(int y=0; y<12 && !filled; y++) {
                for(int x=0; x<16 && !filled; x++) {
                    if(dynamic_map[y][x] == '.') { dynamic_map[y][x] = (char)('a' + i); t_cnt[i]++; filled = true; }
                }
            }
            if(!filled) break; 
        }
        while(t_cnt[i] > b_cnt[i]) {
            bool filled = false;
            for(int y=0; y<12 && !filled; y++) {
                for(int x=0; x<16 && !filled; x++) {
                    if(dynamic_map[y][x] == '$') { dynamic_map[y][x] = (char)('A' + i); b_cnt[i]++; filled = true; }
                }
            }
            if(!filled) break; 
        }
    }
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            if (dynamic_map[y][x] == '$') {
                for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
                    if (b_cnt[i] == 0 && t_cnt[i] == 0) {
                        dynamic_map[y][x] = (char)('A' + i); b_cnt[i]++;
                        bool found_dot = false;
                        for (int ty = 0; ty < 12 && !found_dot; ty++) {
                            for (int tx = 0; tx < 16 && !found_dot; tx++) {
                                if (dynamic_map[ty][tx] == '.') {
                                    dynamic_map[ty][tx] = (char)('a' + i); t_cnt[i]++; found_dot = true;
                                }
                            }
                        }
                        break; 
                    }
                }
            }
        }
    }

    if (!build_dyn_pairs_duplicate_aware_c()) {
        memcpy(dynamic_map, backup_map, sizeof(dynamic_map));
        num_dyn_pairs = saved_num_dyn_pairs;
        memcpy(dyn_pairs, saved_pairs, sizeof(dyn_pairs));
        best_push_steps = saved_best;
        return false;
    }

    if (num_dyn_pairs == 0) {
        memcpy(dynamic_map, backup_map, sizeof(dynamic_map));
        num_dyn_pairs = saved_num_dyn_pairs;
        memcpy(dyn_pairs, saved_pairs, sizeof(dyn_pairs));
        return true; 
    }

    best_push_steps = 999999;
    evaluate_all_duplicate_target_assignments_c(p_pos, cur_bombs, cur_nb);

    bool solvable = (best_push_steps != 999999);

    best_push_steps = saved_best;
    memcpy(dynamic_map, backup_map, sizeof(dynamic_map));
    num_dyn_pairs = saved_num_dyn_pairs;
    memcpy(dyn_pairs, saved_pairs, sizeof(dyn_pairs));

    return solvable;
}


// ============================================================================
// ?????????π◊??ß≥?? Dijkstra ????????? DFS ??????
// ???????????????????/????????????????????????????? Python??
// ???????????? BOMB_DIJKSTRA_MAX_NODES ????????? 12x16 ß≥?????
// ============================================================================
#ifndef BOMB_DIJKSTRA_MAX_NODES
#define BOMB_DIJKSTRA_MAX_NODES 1
#endif

#define BOMB_TASK_LIMIT SOKOBAN_LEVEL3_BOMB_LIMIT

typedef struct {
    bool used;
    bool closed;
    int cost;
    Pos3 player;
    Pos3 bombs_state[MAX_BOMBS3];
    int nb;
    int task_cnt;
    TspObj tasks[BOMB_TASK_LIMIT];
    char map_state[12][16];
    uint32_t hash;
} BombDijkstraNode;

static BombDijkstraNode bomb_dij_nodes[BOMB_DIJKSTRA_MAX_NODES];

static uint32_t hash_map_state_c(char mp[12][16]) {
    uint32_t h = 2166136261u;
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            h ^= (uint8_t)mp[y][x];
            h *= 16777619u;
        }
    }
    return h;
}

static bool same_bombs_state_c(Pos3* a, int na, Pos3* b, int nb) {
    if (na != nb) return false;
    for (int i = 0; i < na; i++) {
        if (a[i].x != b[i].x || a[i].y != b[i].y) return false;
    }
    return true;
}

static bool same_map_state_c(char a[12][16], char b[12][16]) {
    return memcmp(a, b, 12 * 16) == 0;
}

static bool map_ready_for_push_state_c(Pos3 p_pos, Pos3* b_arr, int nb) {
    int total_boxes = get_total_box_count();
    if (total_boxes <= 0) return false;
    int solvable_cnt = get_solvable_boxes_c(p_pos, b_arr, nb);
    if (solvable_cnt != total_boxes) return false;
    return check_is_solvable_c(p_pos, b_arr, nb);
}

static int find_same_bomb_node_c(char mp[12][16], uint32_t hash, Pos3 player, Pos3* b_arr, int nb) {
    for (int i = 0; i < BOMB_DIJKSTRA_MAX_NODES; i++) {
        if (!bomb_dij_nodes[i].used) continue;
        if (bomb_dij_nodes[i].hash != hash) continue;
        if (bomb_dij_nodes[i].player.x != player.x || bomb_dij_nodes[i].player.y != player.y) continue;
        if (!same_bombs_state_c(bomb_dij_nodes[i].bombs_state, bomb_dij_nodes[i].nb, b_arr, nb)) continue;
        if (!same_map_state_c(bomb_dij_nodes[i].map_state, mp)) continue;
        return i;
    }
    return -1;
}

static int alloc_bomb_node_c(int new_cost) {
    for (int i = 0; i < BOMB_DIJKSTRA_MAX_NODES; i++) {
        if (!bomb_dij_nodes[i].used) return i;
    }

    // ???????????ùI???????¶ƒ????????????????????????????????
    int worst = -1;
    int worst_cost = -1;
    for (int i = 0; i < BOMB_DIJKSTRA_MAX_NODES; i++) {
        if (!bomb_dij_nodes[i].closed && bomb_dij_nodes[i].cost > worst_cost) {
            worst_cost = bomb_dij_nodes[i].cost;
            worst = i;
        }
    }
    if (worst >= 0 && new_cost < worst_cost) return worst;
    return -1;
}

static int select_min_open_bomb_node_c(void) {
    int best = -1;
    int best_cost = 999999;
    for (int i = 0; i < BOMB_DIJKSTRA_MAX_NODES; i++) {
        if (bomb_dij_nodes[i].used && !bomb_dij_nodes[i].closed && bomb_dij_nodes[i].cost < best_cost) {
            best_cost = bomb_dij_nodes[i].cost;
            best = i;
        }
    }
    return best;
}

static void remove_bomb_from_state_c(Pos3* src, int nb, int remove_idx, Pos3* dst, int* out_nb) {
    int w = 0;
    for (int i = 0; i < nb; i++) {
        if (i == remove_idx) continue;
        dst[w++] = src[i];
    }
    *out_nb = w;
}

static void blast_on_map_c(char mp[12][16], Pos3 wall) {
    for (int ey = -1; ey <= 1; ey++) {
        for (int ex = -1; ex <= 1; ex++) {
            int tx = wall.x + ex;
            int ty = wall.y + ey;
            if (tx > 0 && tx < 15 && ty > 0 && ty < 11 && mp[ty][tx] == '#') {
                mp[ty][tx] = '-';
            }
        }
    }
}

static bool dijkstra_bomb_plan_c(Pos3 start_p, TspObj* out_seq, int* out_cnt, int* out_cost) {
    memset(bomb_dij_nodes, 0, sizeof(bomb_dij_nodes));

    int root = 0;
    bomb_dij_nodes[root].used = true;
    bomb_dij_nodes[root].closed = false;
    bomb_dij_nodes[root].cost = 0;
    bomb_dij_nodes[root].player = start_p;
    bomb_dij_nodes[root].nb = num_bombs;
    bomb_dij_nodes[root].task_cnt = 0;
    memcpy(bomb_dij_nodes[root].bombs_state, bombs, sizeof(bombs));
    memcpy(bomb_dij_nodes[root].map_state, dynamic_map, sizeof(dynamic_map));
    bomb_dij_nodes[root].hash = hash_map_state_c(bomb_dij_nodes[root].map_state);

    int best_total_cost = 999999;
    int best_bomb_cost = 999999;
    int best_task_cnt = 0;
    TspObj best_tasks[BOMB_TASK_LIMIT];

    while (1) {
        int idx = select_min_open_bomb_node_c();
        if (idx < 0) break;

        BombDijkstraNode* node = &bomb_dij_nodes[idx];
        node->closed = true;

        // ???????????????????????????????????????
        if (node->cost >= best_total_cost) continue;

        memcpy(dynamic_map, node->map_state, sizeof(dynamic_map));

        // ????????????????????????????????? + ?????????????????????
        // ???????????????????????????????????????ß≥?
        int push_est = estimate_push_cost_with_nminus1_c(node->player, node->bombs_state, node->nb);
        if (push_est < 999999) {
            int total = node->cost + push_est;
            if (total < best_total_cost) {
                best_total_cost = total;
                best_bomb_cost = node->cost;
                best_task_cnt = node->task_cnt;
                if (node->task_cnt > 0) {
                    memcpy(best_tasks, node->tasks, sizeof(TspObj) * node->task_cnt);
                }
            }
            // ?????? return???????????????????????????????????????????????????
        }

        if (node->nb <= 0 || node->task_cnt >= BOMB_TASK_LIMIT) continue;

        get_blocking_walls_c(node->bombs_state, node->nb);

        for (int bi = 0; bi < node->nb; bi++) {
            Pos3 used_bomb = node->bombs_state[bi];
            get_all_reachable_walls_c(node->player, used_bomb, node->bombs_state, node->nb);

            int c_cnt = num_reach_walls;
            ReachableWall cands[60];
            for (int k = 0; k < c_cnt; k++) cands[k] = reach_walls[k];

            for (int c = 0; c < c_cnt; c++) {
                Pos3 wall = cands[c].wall;
                int useful_score = 0;
                for (int ey = -1; ey <= 1; ey++) {
                    for (int ex = -1; ex <= 1; ex++) {
                        int tx = wall.x + ex;
                        int ty = wall.y + ey;
                        for (int bw = 0; bw < num_blocking_walls; bw++) {
                            if (blocking_walls[bw].x == tx && blocking_walls[bw].y == ty) {
                                useful_score++;
                                break;
                            }
                        }
                    }
                }
                if (useful_score <= 0) continue;

                char child_map[12][16];
                Pos3 child_bombs[MAX_BOMBS3];
                int child_nb = 0;
                memcpy(child_map, node->map_state, sizeof(child_map));
                blast_on_map_c(child_map, wall);
                remove_bomb_from_state_c(node->bombs_state, node->nb, bi, child_bombs, &child_nb);

                int new_cost = node->cost + cands[c].path_len;
                if (new_cost >= best_total_cost) continue;

                uint32_t child_hash = hash_map_state_c(child_map);
                int same_idx = find_same_bomb_node_c(child_map, child_hash, cands[c].end_p, child_bombs, child_nb);

                int dst = same_idx;
                if (dst >= 0) {
                    if (bomb_dij_nodes[dst].closed || new_cost >= bomb_dij_nodes[dst].cost) continue;
                } else {
                    dst = alloc_bomb_node_c(new_cost);
                    if (dst < 0) continue;
                }

                BombDijkstraNode* child = &bomb_dij_nodes[dst];
                child->used = true;
                child->closed = false;
                child->cost = new_cost;
                child->player = cands[c].end_p;
                child->nb = child_nb;
                child->task_cnt = node->task_cnt + 1;
                memcpy(child->bombs_state, child_bombs, sizeof(child->bombs_state));
                memcpy(child->map_state, child_map, sizeof(child->map_state));
                child->hash = child_hash;
                memcpy(child->tasks, node->tasks, sizeof(TspObj) * node->task_cnt);
                child->tasks[node->task_cnt] = (TspObj){2, -1, used_bomb, '*', wall};
            }
        }
    }

    if (best_total_cost < 999999 && best_task_cnt > 0) {
        memcpy(out_seq, best_tasks, sizeof(TspObj) * best_task_cnt);
        *out_cnt = best_task_cnt;
        *out_cost = best_bomb_cost;
        return true;
    }

    *out_cnt = 0;
    *out_cost = 999999;
    return false;
}

// ============================================================================
// ?? Dijkstra ?? + TSP ???? (????)
// ============================================================================

// ============================================================================
// ?????????????????????? TSP
// ????????? n-1 ???plan_explore_phase3 ????????????/?????°§???
//       ?? main ???????????????? plan_secondary_bomb_phase3 ??????????
// ============================================================================
static void append_explore_cmd_c(char c) {
    if (explore_steps3 < LEVEL3_PATH_CAPACITY3 - 1) {
        explore_commands3[explore_steps3++] = c;
        explore_commands3[explore_steps3] = '\0';
    }
}

/* Keep the bomb phase independent from explore_commands3. */
static void append_bomb_cmd_c(char c) {
    if (bomb_steps3 < LEVEL3_PATH_CAPACITY3 - 1) {
        bomb_commands3[bomb_steps3++] = c;
        bomb_commands3[bomb_steps3] = '\0';
    }
}

static void apply_blast_c(Pos3 wall) {
    for (int ey = -1; ey <= 1; ey++) {
        for (int ex = -1; ex <= 1; ex++) {
            int tx = wall.x + ex;
            int ty = wall.y + ey;
            if (tx > 0 && tx < 15 && ty > 0 && ty < 11 && dynamic_map[ty][tx] == '#') {
                dynamic_map[ty][tx] = '-';
            }
        }
    }
}

static void remove_bomb_at_c(Pos3 p) {
    for (int i = 0; i < num_bombs; i++) {
        if (bombs[i].x == p.x && bombs[i].y == p.y) {
            for (int k = i; k < num_bombs - 1; k++) bombs[k] = bombs[k + 1];
            num_bombs--;
            return;
        }
    }
}

static int count_reachable_recognition_objects_c(Pos3 from) {
    int cnt = 0;
    int dx[4] = {0, 0, -1, 1};
    int dy[4] = {-1, 1, 0, 0};
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (!(c == '$' || c == '.' || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))) continue;
            bool ok = false;
            for (int d = 0; d < 4; d++) {
                int sx = x + dx[d];
                int sy = y + dy[d];
                if (sx < 0 || sx >= 16 || sy < 0 || sy >= 12) continue;
                if (is_obstacle_c(sx, sy, c == '.' || (c >= 'a' && c <= 'z'))) continue;
                int len = 0; char tmp[LEVEL3_INTERNAL_PATH_CAPACITY3];
                if (bfs_move_only_c(from, (Pos3){sx, sy}, c == '.' || (c >= 'a' && c <= 'z'), &len, tmp)) { ok = true; break; }
            }
            if (ok) cnt++;
        }
    }
    return cnt;
}

static int count_reachable_bombs_c(Pos3 from) {
    int cnt = 0;
    int dx[4] = {0, 0, -1, 1};
    int dy[4] = {-1, 1, 0, 0};
    for (int i = 0; i < num_bombs; i++) {
        bool ok = false;
        for (int d = 0; d < 4; d++) {
            int sx = bombs[i].x + dx[d];
            int sy = bombs[i].y + dy[d];
            if (sx < 0 || sx >= 16 || sy < 0 || sy >= 12) continue;
            if (is_obstacle_c(sx, sy, true)) continue;
            int len = 0; char tmp[LEVEL3_INTERNAL_PATH_CAPACITY3];
            if (bfs_move_only_c(from, (Pos3){sx, sy}, true, &len, tmp)) { ok = true; break; }
        }
        if (ok) cnt++;
    }
    return cnt;
}

static int score_wall_usefulness_c(Pos3 wall) {
    int useful_score = 0;
    for (int ey = -1; ey <= 1; ey++) {
        for (int ex = -1; ex <= 1; ex++) {
            int tx = wall.x + ex;
            int ty = wall.y + ey;
            for (int bw = 0; bw < num_blocking_walls; bw++) {
                if (blocking_walls[bw].x == tx && blocking_walls[bw].y == ty) { useful_score++; break; }
            }
        }
    }
    return useful_score;
}

static int estimate_remaining_recognition_cost_c(Pos3 from, char dir, uint32_t recognized_mask, int need_total, int got_count) {
    int need_now = need_total - got_count;
    if (need_now <= 0) return 0;

    uint32_t remaining_mask = 0;
    int remaining_cnt = 0;
    for (int i = 0; i < num_objs; i++) {
        if ((recognized_mask & (1u << i)) == 0) {
            remaining_mask |= (1u << i);
            remaining_cnt++;
        }
    }
    if (remaining_cnt <= 0) return 0;

    int saved_cost = global_best_cost;
    int saved_len = global_best_len;
    char saved_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    memcpy(saved_path, global_best_path, sizeof(saved_path));

    global_best_cost = 999999;
    global_best_len = 0;

    // n-1 ?????°Í?need_now ??????? remaining_cnt ?? 1??
    // ?????????????????????????????????????????
    if (need_now >= remaining_cnt) {
        char tmp_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
        dfs_tsp(13, from, dir, remaining_mask, 0, tmp_path, 0);
    } else {
        for (int drop = 0; drop < num_objs; drop++) {
            if ((remaining_mask & (1u << drop)) == 0) continue;
            char tmp_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
            dfs_tsp(13, from, dir, remaining_mask & ~(1u << drop), 0, tmp_path, 0);
        }
    }

    int ans = global_best_cost;
    global_best_cost = saved_cost;
    global_best_len = saved_len;
    memcpy(global_best_path, saved_path, sizeof(saved_path));
    return ans;
}

static int estimate_push_cost_with_nminus1_c(Pos3 p_pos, Pos3* cur_bombs, int cur_nb) {
    char backup_map[12][16];
    memcpy(backup_map, dynamic_map, sizeof(dynamic_map));
    int saved_num_dyn_pairs = num_dyn_pairs;
    DynBoxTargetPair saved_pairs[MAX_BOXES3];
    memcpy(saved_pairs, dyn_pairs, sizeof(dyn_pairs));
    int saved_best = best_push_steps;

    int b_cnt[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};
    int t_cnt[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (c >= 'A' && c <= 'Z') b_cnt[c - 'A']++;
            if (c >= 'a' && c <= 'z') t_cnt[c - 'a']++;
        }
    }

    // ????????? n-1 ????????????????????????¶ƒ? $/. ????
    for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        while (b_cnt[i] > t_cnt[i]) {
            bool filled = false;
            for (int y = 0; y < 12 && !filled; y++) {
                for (int x = 0; x < 16 && !filled; x++) {
                    if (dynamic_map[y][x] == '.') {
                        dynamic_map[y][x] = (char)('a' + i);
                        t_cnt[i]++;
                        filled = true;
                    }
                }
            }
            if (!filled) break;
        }
        while (t_cnt[i] > b_cnt[i]) {
            bool filled = false;
            for (int y = 0; y < 12 && !filled; y++) {
                for (int x = 0; x < 16 && !filled; x++) {
                    if (dynamic_map[y][x] == '$') {
                        dynamic_map[y][x] = (char)('A' + i);
                        b_cnt[i]++;
                        filled = true;
                    }
                }
            }
            if (!filled) break;
        }
    }

    // ????????? $ ????? .????????ßÿ????????????t????????????????
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            if (dynamic_map[y][x] == '$') {
                for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
                    if (b_cnt[i] == 0 && t_cnt[i] == 0) {
                        dynamic_map[y][x] = (char)('A' + i);
                        b_cnt[i]++;
                        bool found_dot = false;
                        for (int ty = 0; ty < 12 && !found_dot; ty++) {
                            for (int tx = 0; tx < 16 && !found_dot; tx++) {
                                if (dynamic_map[ty][tx] == '.') {
                                    dynamic_map[ty][tx] = (char)('a' + i);
                                    t_cnt[i]++;
                                    found_dot = true;
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    int ans = 999999;
    if (build_dyn_pairs_duplicate_aware_c()) {
        if (num_dyn_pairs == 0) {
            ans = 0;
        } else {
            best_push_steps = 999999;
            evaluate_all_duplicate_target_assignments_c(p_pos, cur_bombs, cur_nb);
            ans = best_push_steps;
        }
    }

    best_push_steps = saved_best;
    memcpy(dynamic_map, backup_map, sizeof(dynamic_map));
    num_dyn_pairs = saved_num_dyn_pairs;
    memcpy(dyn_pairs, saved_pairs, sizeof(dyn_pairs));
    return ans;
}

// ????????????????¶¬??????????????????????????????????/??????? n-1 ???????
static bool try_insert_explore_bomb_c(Pos3* cur_p, char* cur_dir, uint32_t recognized_mask, int got_count, int need_total) {
#if ENABLE_EXPLORE_BOMB_INSERT
    if (solver_explore_bomb_insert_count >= EXPLORE_BOMB_INSERT_LIMIT) return false;
    if (num_bombs <= 0) return false;

    char base_map[12][16];
    Pos3 base_bombs[MAX_BOMBS3];
    int base_nb = num_bombs;
    memcpy(base_map, dynamic_map, sizeof(base_map));
    memcpy(base_bombs, bombs, sizeof(base_bombs));

    int before_remain = estimate_remaining_recognition_cost_c(*cur_p, *cur_dir, recognized_mask, need_total, got_count);
    if (before_remain >= 999999) return false;

    int before_objs = count_reachable_recognition_objects_c(*cur_p);
    int before_bombs = count_reachable_bombs_c(*cur_p);
    get_blocking_walls_c(bombs, num_bombs);

    bool found = false;
    int best_score = 999999;
    int best_gain = 0;
    Pos3 best_bomb = {0, 0};
    Pos3 best_wall = {0, 0};
    int best_len = 0;
    char best_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    char best_last_dir = 0;
    Pos3 best_end_p = {0, 0};

    for (int bi = 0; bi < base_nb; bi++) {
        Pos3 used_bomb = base_bombs[bi];
        get_all_reachable_walls_c(*cur_p, used_bomb, base_bombs, base_nb);
        int c_cnt = num_reach_walls;
        ReachableWall cands[60];
        for (int k = 0; k < c_cnt; k++) cands[k] = reach_walls[k];

        for (int ci = 0; ci < c_cnt; ci++) {
            if (cands[ci].path_len > EXPLORE_BOMB_PUSH_MAX_COST) continue;
            Pos3 wall = cands[ci].wall;
            int useful_score = score_wall_usefulness_c(wall);
            if (useful_score <= 0) continue;

            int p_len = 0;
            char p_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
            char last_dir = 0;
            Pos3 end_p = *cur_p;

            memcpy(dynamic_map, base_map, sizeof(dynamic_map));
            memcpy(bombs, base_bombs, sizeof(base_bombs));
            num_bombs = base_nb;
            if (!bfs_push_specific_bomb(*cur_p, used_bomb, wall, &p_len, p_path, &last_dir, &end_p)) continue;

            apply_blast_c(wall);
            remove_bomb_at_c(used_bomb);

            int after_remain = estimate_remaining_recognition_cost_c(end_p, last_dir, recognized_mask, need_total, got_count);
            if (after_remain >= 999999) continue;

            int after_objs = count_reachable_recognition_objects_c(end_p);
            int after_bombs = count_reachable_bombs_c(end_p);
            int gain_obj = after_objs - before_objs;
            int gain_bomb = after_bombs - before_bombs;

            int bomb_cost = p_len + 1;
            int gain = before_remain - (bomb_cost + after_remain);

            // ???????????????????????????????????????
            // ?????????°§??????????????????????????????????¶±?
            if (gain < EXPLORE_BOMB_MIN_GAIN && gain_obj <= 0 && gain_bomb <= 0) continue;

            int score = bomb_cost + after_remain - gain_obj * 20 - gain_bomb * 18 - useful_score;
            if (!found || score < best_score || (score == best_score && gain > best_gain)) {
                found = true;
                best_score = score;
                best_gain = gain;
                best_bomb = used_bomb;
                best_wall = wall;
                best_len = p_len;
                memcpy(best_path, p_path, p_len);
                best_last_dir = last_dir;
                best_end_p = end_p;
            }
        }
    }

    memcpy(dynamic_map, base_map, sizeof(dynamic_map));
    memcpy(bombs, base_bombs, sizeof(base_bombs));
    num_bombs = base_nb;

    if (!found) return false;

    for (int k = 0; k < best_len; k++) append_explore_cmd_c(best_path[k]);
    if (best_last_dir == 'U' || best_last_dir == 'D' || best_last_dir == 'L' || best_last_dir == 'R') {
        append_explore_cmd_c(best_last_dir);
        *cur_dir = best_last_dir;
    }
    apply_blast_c(best_wall);
    remove_bomb_at_c(best_bomb);
    *cur_p = best_end_p;
    solver_explore_bomb_insert_count++;
    return true;
#else
    (void)cur_p;
    (void)cur_dir;
    (void)recognized_mask;
    (void)got_count;
    (void)need_total;
    return false;
#endif
}

static bool map_ready_for_push_c(Pos3 p_pos) {
    int total_boxes = get_total_box_count();
    if (total_boxes <= 0) return false;
    int solvable_cnt = get_solvable_boxes_c(p_pos, bombs, num_bombs);
    if (solvable_cnt != total_boxes) return false;
    return check_is_solvable_c(p_pos, bombs, num_bombs);
}

// ????????????????????????????????????????°§??????????¶±?
// ????????????????????? vs ???????????????????
static bool try_push_shortcut_bomb_c(Pos3* cur_p) {
#if ENABLE_PUSH_SHORTCUT_BOMB
    if (solver_push_shortcut_bomb_count >= PUSH_SHORTCUT_BOMB_LIMIT) return false;
    if (num_bombs <= 0) return false;

    int base_push = estimate_push_cost_with_nminus1_c(*cur_p, bombs, num_bombs);
    if (base_push >= 999999) return false;  // ?????????????????? Dijkstra ??°§??

    char base_map[12][16];
    Pos3 base_bombs[MAX_BOMBS3];
    int base_nb = num_bombs;
    memcpy(base_map, dynamic_map, sizeof(base_map));
    memcpy(base_bombs, bombs, sizeof(base_bombs));

    get_blocking_walls_c(bombs, num_bombs);

    bool found = false;
    int best_total = 999999;
    int best_gain = 0;
    Pos3 best_bomb = {0, 0};
    Pos3 best_wall = {0, 0};
    int best_len = 0;
    char best_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    char best_last_dir = 0;
    Pos3 best_end_p = *cur_p;

    for (int bi = 0; bi < base_nb; bi++) {
        Pos3 used_bomb = base_bombs[bi];
        get_all_reachable_walls_c(*cur_p, used_bomb, base_bombs, base_nb);
        int c_cnt = num_reach_walls;
        ReachableWall cands[60];
        for (int k = 0; k < c_cnt; k++) cands[k] = reach_walls[k];

        for (int ci = 0; ci < c_cnt; ci++) {
            if (cands[ci].path_len > PUSH_SHORTCUT_BOMB_MAX_COST) continue;
            Pos3 wall = cands[ci].wall;
            int useful_score = score_wall_usefulness_c(wall);
            if (useful_score <= 0) continue;

            int p_len = 0;
            char p_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
            char last_dir = 0;
            Pos3 end_p = *cur_p;

            memcpy(dynamic_map, base_map, sizeof(dynamic_map));
            memcpy(bombs, base_bombs, sizeof(base_bombs));
            num_bombs = base_nb;
            if (!bfs_push_specific_bomb(*cur_p, used_bomb, wall, &p_len, p_path, &last_dir, &end_p)) continue;

            apply_blast_c(wall);
            remove_bomb_at_c(used_bomb);

            int after_push = estimate_push_cost_with_nminus1_c(end_p, bombs, num_bombs);
            if (after_push >= 999999) continue;

            int bomb_cost = p_len + 1;
            int total = bomb_cost + after_push;
            int gain = base_push - total;
            if (gain < PUSH_SHORTCUT_MIN_GAIN) continue;

            if (!found || total < best_total || (total == best_total && gain > best_gain)) {
                found = true;
                best_total = total;
                best_gain = gain;
                best_bomb = used_bomb;
                best_wall = wall;
                best_len = p_len;
                memcpy(best_path, p_path, p_len);
                best_last_dir = last_dir;
                best_end_p = end_p;
            }
        }
    }

    memcpy(dynamic_map, base_map, sizeof(dynamic_map));
    memcpy(bombs, base_bombs, sizeof(base_bombs));
    num_bombs = base_nb;

    if (!found) return false;

    for (int k = 0; k < best_len; k++) append_bomb_cmd_c(best_path[k]);
    if (best_last_dir == 'U' || best_last_dir == 'D' || best_last_dir == 'L' || best_last_dir == 'R') {
        append_bomb_cmd_c(best_last_dir);
    }
    apply_blast_c(best_wall);
    remove_bomb_at_c(best_bomb);
    *cur_p = best_end_p;
    solver_push_shortcut_bomb_count++;
    return true;
#else
    (void)cur_p;
    return false;
#endif
}

void plan_explore_phase3(char external_map[12][16])
{
    /* This phase must follow plan_bomb_phase3(). */
    if (!level3_bomb_first_initialized3) {
        level3_error3 = 1;
        level3_error_code3 = 81;
        level3_steps3 = 0;
        level3_commands3[0] = '\0';
        return;
    }

    memcpy(dynamic_map, external_map, sizeof(dynamic_map));
    explore_steps3 = 0;
    expected_cnt3 = 0;
    level3_steps3 = 0;
    level3_commands3[0] = '\0';
    level3_segment_type3 = 1;
    level3_error3 = 0;
    level3_error_code3 = 0;

    Pos3 cur_p = explore_end_pos;
    char cur_dir = explore_end_dir;

    num_objs = 0;
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (c == '$' || (c >= 'A' && c <= 'Z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int object_index3 = num_objs;
                    all_objs[object_index3] =
                        (TspObj){0, object_index3, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
            } else if (c == '.' || (c >= 'a' && c <= 'z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int object_index3 = num_objs;
                    all_objs[object_index3] =
                        (TspObj){1, object_index3, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
            }
        }
    }

    const int need_total = (num_objs > 1) ? (num_objs - 1) : num_objs;
    if (need_total <= 0) {
        explore_end_pos = cur_p;
        explore_end_dir = cur_dir;
        level3_steps3 = 0;
        level3_commands3[0] = '\0';
        return;
    }

    uint32_t all_mask3 = 0;
    for (int i = 0; i < num_objs; i++) all_mask3 |= (1u << i);

    /* Solve the N-1 recognition tour.  The optimized mode precomputes
     * movement costs and uses compact subset DP; legacy mode is retained for
     * small-map comparison and emergency rollback. */
#if LEVEL3_USE_RECOGNITION_DP3
    (void)all_mask3;
    bool recognition_ok3;
    if (num_objs <= SOKOBAN_LEVEL3_RECOG_MAX_OBJECTS)
        recognition_ok3 = level3_plan_recognition_dp3(cur_p, cur_dir, need_total);
    else
        recognition_ok3 = level3_plan_recognition_scalable3(cur_p, cur_dir, need_total);
    if (!recognition_ok3) {
        level3_error3 = 1;
        level3_error_code3 = 82;
        return;
    }
#else
    global_best_cost = 999999;
    global_best_len = 0;
    if (need_total >= num_objs) {
        char tmp_path3[LEVEL3_INTERNAL_PATH_CAPACITY3];
        dfs_tsp(13, cur_p, cur_dir, all_mask3, 0, tmp_path3, 0);
    } else {
        for (int drop3 = 0; drop3 < num_objs; drop3++) {
            char tmp_path3[LEVEL3_INTERNAL_PATH_CAPACITY3];
            dfs_tsp(13, cur_p, cur_dir,
                    all_mask3 & ~(1u << drop3), 0, tmp_path3, 0);
        }
    }
#endif

    if (global_best_cost == 999999 || global_best_len <= 0) {
        level3_error3 = 1;
        level3_error_code3 = 82;
        return;
    }

    for (int k = 0; k < global_best_len; k++) {
        char raw3 = global_best_path[k];
        char cmd3 = raw3;
        bool is_snap3 = (raw3 >= 'a' && raw3 <= 'z');
        append_explore_cmd_c(raw3);
        if (is_snap3) cmd3 = (char)(raw3 - ('a' - 'A'));

        if (cmd3 == 'U') cur_p.y--;
        else if (cmd3 == 'D') cur_p.y++;
        else if (cmd3 == 'L') cur_p.x--;
        else if (cmd3 == 'R') cur_p.x++;
        else if (cmd3 == 'E') {
            if (cur_dir == 'U') cur_dir = 'R';
            else if (cur_dir == 'R') cur_dir = 'D';
            else if (cur_dir == 'D') cur_dir = 'L';
            else cur_dir = 'U';
        } else if (cmd3 == 'Q') {
            if (cur_dir == 'U') cur_dir = 'L';
            else if (cur_dir == 'L') cur_dir = 'D';
            else if (cur_dir == 'D') cur_dir = 'R';
            else cur_dir = 'U';
        } else if (cmd3 == 'B') {
            if (cur_dir == 'U') cur_dir = 'D';
            else if (cur_dir == 'D') cur_dir = 'U';
            else if (cur_dir == 'L') cur_dir = 'R';
            else cur_dir = 'L';
        }

        if (is_snap3 && expected_cnt3 < need_total && expected_cnt3 < LEVEL3_RECOGNITION_CAPACITY3) {
            int tx3 = cur_p.x, ty3 = cur_p.y;
            if (cur_dir == 'U') ty3--;
            else if (cur_dir == 'D') ty3++;
            else if (cur_dir == 'L') tx3--;
            else if (cur_dir == 'R') tx3++;
            expected_targets3[expected_cnt3++] = (Pos3){tx3, ty3};
        }
    }

    explore_end_pos = cur_p;
    explore_end_dir = cur_dir;
    level3_steps3 = explore_steps3;
    level3_commands3[level3_steps3] = '\0';

    if (expected_cnt3 != need_total) {
        level3_error3 = 1;
        level3_error_code3 = 82;
    }
}


/* Sequential low-memory public interface:
 * 1. Plan and apply every bomb.
 * 2. Plan recognition from the final bomb pose.
 * 3. Publish bomb path + recognition path in level3_commands3[].
 *
 * bomb_steps3 and explore_steps3 keep their phase-specific lengths. The
 * temporary bomb prefix lives on the caller stack, so static RAM does not
 * increase. Recognition commands remain lowercase and expected_targets3[]
 * keeps the same order as those lowercase commands.
 */
void plan_bomb_explore_phase3(char external_map[12][16])
{
    char bomb_prefix3[LEVEL3_PATH_CAPACITY3];
    int bomb_len3;
    int explore_len3;

    bomb_explore_steps3 = 0;
    bomb_explore_commands3[0] = '\0';

    plan_bomb_phase3(external_map);
    if (level3_error3) return;

    bomb_len3 = bomb_steps3;
    if (bomb_len3 < 0 || bomb_len3 >= LEVEL3_PATH_CAPACITY3) {
        level3_error3 = 1;
        level3_error_code3 = 83;
        level3_steps3 = 0;
        level3_commands3[0] = '\0';
        return;
    }
    if (bomb_len3 > 0) {
        memcpy(bomb_prefix3, level3_commands3, (size_t)bomb_len3);
    }

    plan_explore_phase3(external_map);
    if (level3_error3) return;

    explore_len3 = explore_steps3;
    if (explore_len3 < 0 ||
        bomb_len3 + explore_len3 >= LEVEL3_PATH_CAPACITY3) {
        level3_error3 = 1;
        level3_error_code3 = 84;
        level3_steps3 = 0;
        level3_commands3[0] = '\0';
        return;
    }

    if (explore_len3 > 0) {
        memmove(level3_commands3 + bomb_len3,
                level3_commands3,
                (size_t)explore_len3);
    }
    if (bomb_len3 > 0) {
        memcpy(level3_commands3, bomb_prefix3, (size_t)bomb_len3);
    }

    level3_steps3 = bomb_len3 + explore_len3;
    level3_commands3[level3_steps3] = '\0';

    memcpy(bomb_explore_commands3,
           level3_commands3,
           (size_t)level3_steps3 + 1u);
    bomb_explore_steps3 = level3_steps3;

    level3_segment_type3 = 4;
}


void plan_secondary_bomb_phase3(char external_map[12][16]) {
    memcpy(dynamic_map, external_map, sizeof(dynamic_map));
    bomb_steps3 = 0;
    bomb_commands3[0] = '\0';
    Pos3 cur_p = explore_end_pos;

    // ????????ß’?? external_map ??????????????
    // ?????®¿?ß≥?? Dijkstra + ????????1ú˝????
    // 1. ????????????????????????? DFS/??????????????
    // 2. ?????????????????¶Ã??????????????????????????Å£???????
    for (int round = 0; round < SOKOBAN_LEVEL3_BOMB_LIMIT && num_bombs > 0; round++) {
        if (map_ready_for_push_c(cur_p)) {
            if (try_push_shortcut_bomb_c(&cur_p)) continue;
            break;
        }

        TspObj task_seq[BOMB_TASK_LIMIT];
        int task_cnt = 0;
        int plan_cost = 999999;
        char real_map_backup[12][16];
        memcpy(real_map_backup, dynamic_map, sizeof(real_map_backup));

        bool ok = dijkstra_bomb_plan_c(cur_p, task_seq, &task_cnt, &plan_cost);

        // dijkstra_bomb_plan_c ??????????ß›? dynamic_map??
        // ????????????????????????????????????? BFS ????????????????????°§????
        memcpy(dynamic_map, real_map_backup, sizeof(real_map_backup));

        if (!ok || task_cnt <= 0) {
            break;
        }

        // ???ß÷??????????????????????1ú˝??
        // ????????????????????????????????????????????/?????????????????
        Pos3 used_bomb = task_seq[0].p;
        Pos3 wall = task_seq[0].target_wall;
        int p_len = 0;
        char p_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
        char last_dir = 0;
        Pos3 end_p = cur_p;

        if (!bfs_push_specific_bomb(cur_p, used_bomb, wall, &p_len, p_path, &last_dir, &end_p)) {
            break;
        }

        for (int k = 0; k < p_len; k++) append_bomb_cmd_c(p_path[k]);
        if (last_dir == 'U' || last_dir == 'D' || last_dir == 'L' || last_dir == 'R') {
            append_bomb_cmd_c(last_dir);
        }

        cur_p = end_p;
        apply_blast_c(wall);
        remove_bomb_at_c(used_bomb);

        if (map_ready_for_push_c(cur_p)) {
            if (try_push_shortcut_bomb_c(&cur_p)) continue;
            break;
        }
    }

    explore_end_pos = cur_p;
    bomb_commands3[bomb_steps3] = '\0';
    memcpy(external_map, dynamic_map, sizeof(dynamic_map));
}

// ============================================================================
// ??¶∆??????????????????? n-1 ????????? BFS
// ============================================================================

/* ========================================================================== */
/* Low-memory interleaved multi-box beam search.                               */
/*                                                                            */
/* The old final solver finishes one box before touching the next one.  Some   */
/* third-level maps are shorter only when another box is moved temporarily.    */
/* This beam search treats one legal box push as a macro action.  Walking to    */
/* the required pushing side is found by a 192-cell BFS, and the action history */
/* is packed into four bits per push. Width 106 and bounded macro actions preserve    */
/* the 122-step result; its two layers overlay the earlier BFS workspace.        */
/* ========================================================================== */
static uint8_t level3_multi_wall3[12 * 16];
static uint8_t level3_multi_push_dist3[LEVEL3_MULTI_PUSH_MAX_BOXES3][12 * 16];
static uint8_t level3_multi_walk_dist3[12 * 16];
static uint8_t level3_multi_walk_parent3[12 * 16];
static uint8_t level3_multi_walk_move3[12 * 16];
static uint8_t level3_multi_walk_queue3[12 * 16];

uint8_t level3_multi_push_used3 = 0;
int level3_multi_push_nodes3 = 0;
int level3_multi_push_best_cost3 = 999999;

static const int level3_multi_dx3[4] = {0, 0, -1, 1};
static const int level3_multi_dy3[4] = {-1, 1, 0, 0};
static const char level3_multi_cmd3[4] = {'U', 'D', 'L', 'R'};

static uint8_t level3_multi_cell3(int x, int y)
{
    return (uint8_t)(y * 16 + x);
}

static void level3_multi_set_action3(
    Level3MultiPushState3 *state3, int index3, uint8_t action3)
{
    state3->actions[index3] = action3;
}

static uint8_t level3_multi_get_action3(
    const Level3MultiPushState3 *state3, int index3)
{
    return state3->actions[index3];
}

static bool level3_multi_box_at3(
    const uint8_t *boxes3, int box_count3, uint8_t cell3)
{
    for (int i = 0; i < box_count3; i++) {
        if (boxes3[i] == cell3) return true;
    }
    return false;
}

static void level3_multi_sort_equal_boxes3(
    uint8_t *boxes3, int box_count3)
{
    for (int label3 = 0; label3 < SOKOBAN_ELEMENT_TYPE_CAPACITY; label3++) {
        int start3 = dup_group_start[label3];
        int count3 = dup_group_count[label3];
        if (start3 < 0 || count3 <= 1) continue;
        if (start3 + count3 > box_count3) continue;

        for (int i3 = start3 + 1; i3 < start3 + count3; i3++) {
            uint8_t value3 = boxes3[i3];
            int j3 = i3 - 1;
            while (j3 >= start3 && boxes3[j3] > value3) {
                boxes3[j3 + 1] = boxes3[j3];
                j3--;
            }
            boxes3[j3 + 1] = value3;
        }
    }
}

static bool level3_multi_same_state3(
    const Level3MultiPushState3 *a3,
    const Level3MultiPushState3 *b3,
    int box_count3)
{
    if (a3->player != b3->player) return false;
    if (a3->used_target_mask != b3->used_target_mask) return false;
    for (int i = 0; i < box_count3; i++) {
        if (a3->boxes[i] != b3->boxes[i]) return false;
    }
    return true;
}

static int level3_multi_matching_target_at3(
    int box_index3, uint8_t cell3, SokobanBoxMask used_target_mask3,
    int box_count3)
{
    char label3 = dyn_pairs[box_index3].label;
    for (int target_index3 = 0; target_index3 < box_count3; target_index3++) {
        SokobanBoxMask bit3 = (SokobanBoxMask)1u << target_index3;
        if (used_target_mask3 & bit3) continue;
        if (dyn_pairs[target_index3].label != label3) continue;
        uint8_t target3 = level3_multi_cell3(
            dyn_pairs[target_index3].target_pos.x,
            dyn_pairs[target_index3].target_pos.y);
        if (target3 == cell3) return target_index3;
    }
    return -1;
}

static void level3_multi_walk_bfs3(
    uint8_t start3, const uint8_t *boxes3, int box_count3)
{
    /* 192 cells fit in uint8_t distance. A 192-bit box mask replaces the
     * repeated linear scan over all boxes for every BFS neighbour. */
    uint32_t box_bits3[(12U * 16U + 31U) / 32U] = {0};
    for (int i = 0; i < box_count3; i++) {
        uint8_t b3 = boxes3[i];
        if (b3 != LEVEL3_MULTI_PUSH_REMOVED3)
            box_bits3[b3 >> 5] |= (uint32_t)1u << (b3 & 31u);
    }
    memset(level3_multi_walk_dist3, 0xFF, sizeof(level3_multi_walk_dist3));

    int head3 = 0;
    int tail3 = 0;
    level3_multi_walk_queue3[tail3++] = start3;
    level3_multi_walk_dist3[start3] = 0;
    level3_multi_walk_parent3[start3] = start3;

    while (head3 < tail3) {
        uint8_t cell3 = level3_multi_walk_queue3[head3++];
        int x3 = cell3 & 0x0F;
        int y3 = cell3 >> 4;

        for (int d3 = 0; d3 < 4; d3++) {
            int nx3 = x3 + level3_multi_dx3[d3];
            int ny3 = y3 + level3_multi_dy3[d3];
            if (nx3 < 0 || nx3 >= 16 || ny3 < 0 || ny3 >= 12) continue;

            uint8_t next3 = level3_multi_cell3(nx3, ny3);
            if (level3_multi_wall3[next3]) continue;
            if ((box_bits3[next3 >> 5] >> (next3 & 31u)) & 1u) continue;
            if (level3_multi_walk_dist3[next3] != 0xFFu) continue;

            level3_multi_walk_dist3[next3] =
                (uint8_t)(level3_multi_walk_dist3[cell3] + 1u);
            level3_multi_walk_parent3[next3] = cell3;
            level3_multi_walk_move3[next3] = (uint8_t)d3;
            level3_multi_walk_queue3[tail3++] = next3;
        }
    }
}

static int level3_multi_emit_walk3(
    uint8_t start3, uint8_t end3,
    const uint8_t *boxes3, int box_count3,
    int previous_dir3, int next_push_dir3,
    char *out_path3, int out_capacity3)
{
    int required_len3 = level3_multi_walk_dist3[end3];
    if (required_len3 == 0xFF) return -1;

    memcpy(turn_blocked3, level3_multi_wall3, LEVEL3_CELL_COUNT3);
    for (int i3 = 0; i3 < box_count3; i3++) {
        if (boxes3[i3] != LEVEL3_MULTI_PUSH_REMOVED3)
            turn_blocked3[boxes3[i3]] = 1u;
    }

    Pos3 start_pos3 = {(int)(start3 & 0x0F), (int)(start3 >> 4)};
    Pos3 end_pos3 = {(int)(end3 & 0x0F), (int)(end3 >> 4)};
    int produced3 = 0;
    if (level3_min_turn_shortest_path3(
            start_pos3, end_pos3, turn_blocked3,
            previous_dir3, next_push_dir3, required_len3,
            out_path3, out_capacity3, &produced3)) {
        return produced3;
    }

    /* Conservative fallback to the original BFS parent chain. */
    int count3 = 0;
    uint8_t cell3 = end3;
    while (cell3 != start3) {
        if (count3 >= out_capacity3) return -1;
        out_path3[count3++] =
            level3_multi_cmd3[level3_multi_walk_move3[cell3]];
        cell3 = level3_multi_walk_parent3[cell3];
    }
    for (int i3 = 0; i3 < count3 / 2; i3++) {
        char temp3 = out_path3[i3];
        out_path3[i3] = out_path3[count3 - 1 - i3];
        out_path3[count3 - 1 - i3] = temp3;
    }
    return count3;
}

static void level3_multi_prepare_push_dist3(int box_count3)
{
    /* Distance row t belongs to physical target t, not to a box identity. */
    for (int target_index3 = 0; target_index3 < box_count3; target_index3++) {
        for (int cell3 = 0; cell3 < 12 * 16; cell3++) {
            level3_multi_push_dist3[target_index3][cell3] = 0xFFu;
        }

        uint8_t queue3[12 * 16];
        int head3 = 0;
        int tail3 = 0;
        uint8_t target3 = level3_multi_cell3(
            dyn_pairs[target_index3].target_pos.x,
            dyn_pairs[target_index3].target_pos.y);

        level3_multi_push_dist3[target_index3][target3] = 0;
        queue3[tail3++] = target3;

        while (head3 < tail3) {
            uint8_t box_cell3 = queue3[head3++];
            int box_x3 = box_cell3 & 0x0F;
            int box_y3 = box_cell3 >> 4;

            for (int d3 = 0; d3 < 4; d3++) {
                int previous_x3 = box_x3 - level3_multi_dx3[d3];
                int previous_y3 = box_y3 - level3_multi_dy3[d3];
                int stand_x3 = previous_x3 - level3_multi_dx3[d3];
                int stand_y3 = previous_y3 - level3_multi_dy3[d3];

                if (previous_x3 < 0 || previous_x3 >= 16 ||
                    previous_y3 < 0 || previous_y3 >= 12 ||
                    stand_x3 < 0 || stand_x3 >= 16 ||
                    stand_y3 < 0 || stand_y3 >= 12) {
                    continue;
                }

                uint8_t previous3 =
                    level3_multi_cell3(previous_x3, previous_y3);
                uint8_t stand3 = level3_multi_cell3(stand_x3, stand_y3);
                if (level3_multi_wall3[previous3] ||
                    level3_multi_wall3[stand3]) {
                    continue;
                }
                if (level3_multi_push_dist3[target_index3][previous3] != 0xFFu) {
                    continue;
                }

                level3_multi_push_dist3[target_index3][previous3] =
                    (uint8_t)(
                        level3_multi_push_dist3[target_index3][box_cell3] + 1);
                queue3[tail3++] = previous3;
            }
        }
    }
}

/* Exact minimum one-to-one matching cost inside one repeated-label group.
 * Hungarian is O(n^3), n <= MAX_BOXES3, and avoids factorial target
 * assignment while still respecting that each physical target is used once. */
static int level3_multi_group_matching_cost3(
    const uint8_t *boxes3, int group_start3, int group_count3,
    SokobanBoxMask used_target_mask3)
{
    int box_slots3[MAX_BOXES3];
    int target_slots3[MAX_BOXES3];
    int nbox3 = 0;
    int ntarget3 = 0;

    for (int k3 = 0; k3 < group_count3; k3++) {
        int slot3 = group_start3 + k3;
        if (boxes3[slot3] != LEVEL3_MULTI_PUSH_REMOVED3)
            box_slots3[nbox3++] = slot3;
        if ((used_target_mask3 & ((SokobanBoxMask)1u << slot3)) == 0)
            target_slots3[ntarget3++] = slot3;
    }
    if (nbox3 != ntarget3) return 10000;
    if (nbox3 == 0) return 0;

    int u3[MAX_BOXES3 + 1] = {0};
    int v3[MAX_BOXES3 + 1] = {0};
    int p3[MAX_BOXES3 + 1] = {0};
    int way3[MAX_BOXES3 + 1] = {0};

    for (int i3 = 1; i3 <= nbox3; i3++) {
        p3[0] = i3;
        int j03 = 0;
        int minv3[MAX_BOXES3 + 1];
        uint8_t used3[MAX_BOXES3 + 1] = {0};
        for (int j3 = 1; j3 <= nbox3; j3++) minv3[j3] = 10000;

        do {
            used3[j03] = 1;
            int i03 = p3[j03];
            int delta3 = 10000;
            int j13 = 0;
            for (int j3 = 1; j3 <= nbox3; j3++) {
                if (used3[j3]) continue;
                int target_slot3 = target_slots3[j3 - 1];
                uint8_t dist3 = level3_multi_push_dist3[target_slot3]
                    [boxes3[box_slots3[i03 - 1]]];
                int edge3 = (dist3 == 0xFFu) ? 10000 : (int)dist3;
                int cur3 = edge3 - u3[i03] - v3[j3];
                if (cur3 < minv3[j3]) {
                    minv3[j3] = cur3;
                    way3[j3] = j03;
                }
                if (minv3[j3] < delta3) {
                    delta3 = minv3[j3];
                    j13 = j3;
                }
            }
            if (delta3 >= 10000) return 10000;
            for (int j3 = 0; j3 <= nbox3; j3++) {
                if (used3[j3]) {
                    u3[p3[j3]] += delta3;
                    v3[j3] -= delta3;
                } else if (j3 != 0) {
                    minv3[j3] -= delta3;
                }
            }
            j03 = j13;
        } while (p3[j03] != 0);

        do {
            int j13 = way3[j03];
            p3[j03] = p3[j13];
            j03 = j13;
        } while (j03 != 0);
    }

    int result3 = -v3[0];
    return (result3 >= 10000) ? 10000 : result3;
}

static int level3_multi_heuristic3(
    const uint8_t *boxes3, int box_count3,
    SokobanBoxMask used_target_mask3)
{
    int result3 = 0;
    for (int label3 = 0; label3 < SOKOBAN_ELEMENT_TYPE_CAPACITY; label3++) {
        int start3 = dup_group_start[label3];
        int count3 = dup_group_count[label3];
        if (start3 < 0 || count3 <= 0) continue;
        if (start3 + count3 > box_count3) return 10000;
        int group_cost3 = level3_multi_group_matching_cost3(
            boxes3, start3, count3, used_target_mask3);
        if (group_cost3 >= 10000) return 10000;
        result3 += group_cost3;
    }
    return result3;
}

static bool level3_multi_deadlock3(
    const uint8_t *boxes3, int box_count3,
    SokobanBoxMask used_target_mask3)
{
    if (level3_multi_heuristic3(
            boxes3, box_count3, used_target_mask3) >= 10000) {
        return true;
    }

    for (int i = 0; i < box_count3; i++) {
        uint8_t cell3 = boxes3[i];
        if (cell3 == LEVEL3_MULTI_PUSH_REMOVED3) continue;

        int x3 = cell3 & 0x0F;
        int y3 = cell3 >> 4;
        bool up3 = (y3 <= 0) ||
            level3_multi_wall3[level3_multi_cell3(x3, y3 - 1)];
        bool down3 = (y3 >= 11) ||
            level3_multi_wall3[level3_multi_cell3(x3, y3 + 1)];
        bool left3 = (x3 <= 0) ||
            level3_multi_wall3[level3_multi_cell3(x3 - 1, y3)];
        bool right3 = (x3 >= 15) ||
            level3_multi_wall3[level3_multi_cell3(x3 + 1, y3)];

        if ((up3 || down3) && (left3 || right3)) return true;
    }
    return false;
}

static int level3_multi_state_compare3(const void *a3, const void *b3)
{
    const Level3MultiPushState3 *left3 =
        (const Level3MultiPushState3 *)a3;
    const Level3MultiPushState3 *right3 =
        (const Level3MultiPushState3 *)b3;

    if (left3->score != right3->score) {
        return (int)left3->score - (int)right3->score;
    }
    return (int)left3->cost - (int)right3->cost;
}

static int level3_multi_find_worst3(int count3)
{
    int worst3 = 0;
    for (int i = 1; i < count3; i++) {
        if (level3_multi_next3[i].score > level3_multi_next3[worst3].score ||
            (level3_multi_next3[i].score == level3_multi_next3[worst3].score &&
             level3_multi_next3[i].cost > level3_multi_next3[worst3].cost))
            worst3 = i;
    }
    return worst3;
}

static void level3_multi_keep_candidate3(
    const Level3MultiPushState3 *candidate3,
    int *candidate_count3,
    int box_count3,
    int *worst_index3)
{
    for (int i = 0; i < *candidate_count3; i++) {
        if (level3_multi_same_state3(
                &level3_multi_next3[i], candidate3, box_count3)) {
            if (candidate3->cost < level3_multi_next3[i].cost) {
                level3_multi_next3[i] = *candidate3;
                if (*candidate_count3 >= LEVEL3_MULTI_PUSH_BEAM_WIDTH3 &&
                    i == *worst_index3)
                    *worst_index3 = level3_multi_find_worst3(*candidate_count3);
            }
            return;
        }
    }

    if (*candidate_count3 < LEVEL3_MULTI_PUSH_BEAM_WIDTH3) {
        level3_multi_next3[(*candidate_count3)++] = *candidate3;
        if (*candidate_count3 == LEVEL3_MULTI_PUSH_BEAM_WIDTH3)
            *worst_index3 = level3_multi_find_worst3(*candidate_count3);
        return;
    }

    int worst3 = *worst_index3;
    if (candidate3->score > level3_multi_next3[worst3].score ||
        (candidate3->score == level3_multi_next3[worst3].score &&
         candidate3->cost >= level3_multi_next3[worst3].cost))
        return;

    level3_multi_next3[worst3] = *candidate3;
    *worst_index3 = level3_multi_find_worst3(*candidate_count3);
}


static bool level3_multi_push_solve3(
    Pos3 start_player3, char *out_path3, int *out_len3)
{
    bfs_invalidate_workspace3();
    int box_count3 = num_dyn_pairs;
    if (box_count3 <= 0 ||
        box_count3 > LEVEL3_MULTI_PUSH_MAX_BOXES3) {
        return false;
    }

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            level3_multi_wall3[level3_multi_cell3(x, y)] =
                (dynamic_map[y][x] == '#') ? 1u : 0u;
        }
    }
    level3_multi_prepare_push_dist3(box_count3);

    memset(&level3_multi_cur3[0], 0, sizeof(level3_multi_cur3[0]));
    level3_multi_cur3[0].player =
        level3_multi_cell3(start_player3.x, start_player3.y);
    for (int i = 0; i < box_count3; i++) {
        level3_multi_cur3[0].boxes[i] = level3_multi_cell3(
            dyn_pairs[i].box_pos.x, dyn_pairs[i].box_pos.y);
    }
    level3_multi_cur3[0].used_target_mask = 0;
    level3_multi_sort_equal_boxes3(level3_multi_cur3[0].boxes, box_count3);
    int initial_h3 = level3_multi_heuristic3(
        level3_multi_cur3[0].boxes, box_count3,
        level3_multi_cur3[0].used_target_mask);
    if (initial_h3 >= 10000) return false;

    level3_multi_cur3[0].cost = 0;
    level3_multi_cur3[0].score = (uint16_t)(3 * initial_h3);
    int current_count3 = 1;

    Level3MultiPushState3 best_state3;
    int best_cost3 = 999999;
    level3_multi_push_nodes3 = 1;

    for (int depth3 = 0;
         depth3 <= LEVEL3_MULTI_PUSH_MAX_ACTIONS3;
         depth3++) {
        int minimum_open_cost3 = 999999;
        for (int i = 0; i < current_count3; i++) {
            bool finished3 = true;
            for (int b = 0; b < box_count3; b++) {
                if (level3_multi_cur3[i].boxes[b] !=
                    LEVEL3_MULTI_PUSH_REMOVED3) {
                    finished3 = false;
                    break;
                }
            }
            if (finished3) {
                if (level3_multi_cur3[i].cost < best_cost3) {
                    best_state3 = level3_multi_cur3[i];
                    best_cost3 = level3_multi_cur3[i].cost;
                }
            } else if (level3_multi_cur3[i].cost < minimum_open_cost3) {
                minimum_open_cost3 = level3_multi_cur3[i].cost;
            }
        }

        /* Every future push adds at least one command.  Once all surviving
         * beam states already cost no less than the best completed solution,
         * further layers cannot improve it. */
        if (best_cost3 < 999999 && minimum_open_cost3 >= best_cost3) break;
        if (depth3 >= LEVEL3_MULTI_PUSH_MAX_ACTIONS3) break;

        int next_count3 = 0;
        int worst_index3 = -1;
        for (int state_index3 = 0;
             state_index3 < current_count3;
             state_index3++) {
            const Level3MultiPushState3 *state3 =
                &level3_multi_cur3[state_index3];

            level3_multi_walk_bfs3(
                state3->player, state3->boxes, box_count3);

            for (int box_index3 = 0;
                 box_index3 < box_count3;
                 box_index3++) {
                uint8_t box_cell3 = state3->boxes[box_index3];
                if (box_cell3 == LEVEL3_MULTI_PUSH_REMOVED3) continue;

                int box_x3 = box_cell3 & 0x0F;
                int box_y3 = box_cell3 >> 4;

                for (int d3 = 0; d3 < 4; d3++) {
                    int destination_x3 = box_x3 + level3_multi_dx3[d3];
                    int destination_y3 = box_y3 + level3_multi_dy3[d3];
                    int stand_x3 = box_x3 - level3_multi_dx3[d3];
                    int stand_y3 = box_y3 - level3_multi_dy3[d3];

                    if (destination_x3 < 0 || destination_x3 >= 16 ||
                        destination_y3 < 0 || destination_y3 >= 12 ||
                        stand_x3 < 0 || stand_x3 >= 16 ||
                        stand_y3 < 0 || stand_y3 >= 12) {
                        continue;
                    }

                    uint8_t destination3 = level3_multi_cell3(
                        destination_x3, destination_y3);
                    uint8_t stand3 = level3_multi_cell3(
                        stand_x3, stand_y3);

                    if (level3_multi_wall3[destination3]) continue;
                    if (level3_multi_box_at3(
                            state3->boxes, box_count3, destination3)) {
                        continue;
                    }
                    if (level3_multi_walk_dist3[stand3] == 0xFFu) continue;

                    Level3MultiPushState3 candidate3 = *state3;
                    candidate3.player = box_cell3;

                    int matched_target3 = level3_multi_matching_target_at3(
                        box_index3, destination3,
                        candidate3.used_target_mask, box_count3);
                    if (matched_target3 >= 0) {
                        candidate3.used_target_mask |=
                            (SokobanBoxMask)1u << matched_target3;
                        candidate3.boxes[box_index3] =
                            LEVEL3_MULTI_PUSH_REMOVED3;
                    } else {
                        candidate3.boxes[box_index3] = destination3;
                    }
                    level3_multi_sort_equal_boxes3(
                        candidate3.boxes, box_count3);

                    if (level3_multi_deadlock3(
                            candidate3.boxes, box_count3,
                            candidate3.used_target_mask)) {
                        continue;
                    }

                    candidate3.cost = (uint16_t)(
                        state3->cost +
                        level3_multi_walk_dist3[stand3] + 1);
                    int heuristic3 = level3_multi_heuristic3(
                        candidate3.boxes, box_count3,
                        candidate3.used_target_mask);
                    if (heuristic3 >= 10000) continue;

                    candidate3.score = (uint16_t)(
                        candidate3.cost + 3 * heuristic3);
                    level3_multi_set_action3(
                        &candidate3, depth3,
                        (uint8_t)(box_index3 * 4 + d3));
                    candidate3.action_count = (uint8_t)(depth3 + 1);

                    level3_multi_keep_candidate3(
                        &candidate3, &next_count3, box_count3, &worst_index3);
                    level3_multi_push_nodes3++;
                }
            }
        }

        if (next_count3 <= 0) break;

        qsort(level3_multi_next3, (size_t)next_count3,
              sizeof(Level3MultiPushState3),
              level3_multi_state_compare3);
        current_count3 = next_count3;
        memcpy(level3_multi_cur3, level3_multi_next3,
               (size_t)current_count3 * sizeof(Level3MultiPushState3));
    }

    if (best_cost3 >= 999999) return false;

    int path_len3 = 0;
    uint8_t player3 = level3_multi_cell3(
        start_player3.x, start_player3.y);
    uint8_t boxes3[LEVEL3_MULTI_PUSH_MAX_BOXES3];
    for (int i = 0; i < box_count3; i++) {
        boxes3[i] = level3_multi_cell3(
            dyn_pairs[i].box_pos.x, dyn_pairs[i].box_pos.y);
    }
    level3_multi_sort_equal_boxes3(boxes3, box_count3);
    SokobanBoxMask used_target_mask3 = 0;
    int previous_move_dir3 = -1;

    for (int action_index3 = 0;
         action_index3 < best_state3.action_count;
         action_index3++) {
        uint8_t action3 = level3_multi_get_action3(
            &best_state3, action_index3);
        int box_index3 = action3 / 4;
        int d3 = action3 % 4;
        uint8_t box_cell3 = boxes3[box_index3];
        int box_x3 = box_cell3 & 0x0F;
        int box_y3 = box_cell3 >> 4;
        uint8_t stand3 = level3_multi_cell3(
            box_x3 - level3_multi_dx3[d3],
            box_y3 - level3_multi_dy3[d3]);

        level3_multi_walk_bfs3(player3, boxes3, box_count3);
        int walk_len3 = level3_multi_emit_walk3(
            player3, stand3, boxes3, box_count3,
            previous_move_dir3, d3,
            out_path3 + path_len3,
            LEVEL3_INTERNAL_PATH_CAPACITY3 - path_len3 - 1);
        if (walk_len3 < 0) return false;
        path_len3 += walk_len3;
        if (path_len3 >= LEVEL3_INTERNAL_PATH_CAPACITY3 - 1) return false;

        out_path3[path_len3++] = level3_multi_cmd3[d3];
        previous_move_dir3 = d3;

        uint8_t destination3 = level3_multi_cell3(
            box_x3 + level3_multi_dx3[d3],
            box_y3 + level3_multi_dy3[d3]);
        int matched_target3 = level3_multi_matching_target_at3(
            box_index3, destination3, used_target_mask3, box_count3);
        if (matched_target3 >= 0) {
            used_target_mask3 |= (SokobanBoxMask)1u << matched_target3;
            boxes3[box_index3] = LEVEL3_MULTI_PUSH_REMOVED3;
        } else {
            boxes3[box_index3] = destination3;
        }
        level3_multi_sort_equal_boxes3(boxes3, box_count3);
        player3 = box_cell3;
    }

    out_path3[path_len3] = '\0';
    *out_len3 = path_len3;
    level3_multi_push_best_cost3 = best_cost3;
    return (path_len3 == best_cost3);
}

void plan_push_phase3(char external_map[12][16]) {
    memcpy(dynamic_map, external_map, sizeof(dynamic_map));
    push_steps3 = 0;
    level3_steps3 = 0;
    level3_commands3[0] = '\0';
    level3_segment_type3 = 3;
    level3_error3 = 0;
    level3_error_code3 = 0;

    int b_cnt[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0}, t_cnt[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};

    for(int y=0; y<12; y++) {
        for(int x=0; x<16; x++) {
            char c = dynamic_map[y][x];
            if(c >= 'A' && c <= 'Z') b_cnt[c - 'A']++;
            if(c >= 'a' && c <= 'z') t_cnt[c - 'a']++;
        }
    }

    for(int i=0; i<SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        while(b_cnt[i] > t_cnt[i]) {
            bool filled = false;
            for(int y=0; y<12 && !filled; y++) {
                for(int x=0; x<16 && !filled; x++) {
                    if(dynamic_map[y][x] == '.') { dynamic_map[y][x] = (char)('a' + i); t_cnt[i]++; filled = true; }
                }
            }
            if(!filled) break; 
        }
        while(t_cnt[i] > b_cnt[i]) {
            bool filled = false;
            for(int y=0; y<12 && !filled; y++) {
                for(int x=0; x<16 && !filled; x++) {
                    if(dynamic_map[y][x] == '$') { dynamic_map[y][x] = (char)('A' + i); b_cnt[i]++; filled = true; }
                }
            }
            if(!filled) break; 
        }
    }

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            if (dynamic_map[y][x] == '$') {
                for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
                    if (b_cnt[i] == 0 && t_cnt[i] == 0) {
                        dynamic_map[y][x] = (char)('A' + i); b_cnt[i]++;
                        bool found_dot = false;
                        for (int ty = 0; ty < 12 && !found_dot; ty++) {
                            for (int tx = 0; tx < 16 && !found_dot; tx++) {
                                if (dynamic_map[ty][tx] == '.') {
                                    dynamic_map[ty][tx] = (char)('a' + i); t_cnt[i]++; found_dot = true;
                                }
                            }
                        }
                        break; 
                    }
                }
            }
        }
    }

    if (!build_dyn_pairs_duplicate_aware_c()) {
        push_steps3 = 0;
        push_commands3[0] = '\0';
        level3_steps3 = 0;
        level3_error3 = 1;
        level3_error_code3 = 83;
        return;
    }

    level3_multi_push_used3 = 0;
    level3_multi_push_best_cost3 = 999999;
    best_push_steps = 999999;

    bool multi_push_ok3 = false;
    if (num_dyn_pairs > 0 &&
        num_dyn_pairs <= LEVEL3_MULTI_PUSH_MAX_BOXES3 &&
        num_bombs == 0) {
        multi_push_ok3 = level3_multi_push_solve3(
            explore_end_pos, best_push_path, &best_push_steps);
    }

    if (multi_push_ok3) {
        level3_multi_push_used3 = 1;
    } else {
        best_push_steps = 999999;
        /* Legacy permutation fallback is retained only for small scenes.
         * Repeated 10-box layouts must never fall back to factorial search. */
        if (num_dyn_pairs <= 6 || solver_duplicate_element_group_count == 0) {
            evaluate_all_duplicate_target_assignments_c(
                explore_end_pos, bombs, num_bombs);
        }
    }

    if (best_push_steps != 999999) {
        for (int i = 0; i < best_push_steps; i++) push_commands3[push_steps3++] = best_push_path[i];
    }
    push_commands3[push_steps3] = '\0';

    level3_steps3 = push_steps3;
    level3_commands3[level3_steps3] = '\0';
    level3_segment_type3 = 3;
    if (push_steps3 <= 0 && get_total_box_count() > 0) {
        level3_error3 = 1;
        level3_error_code3 = 84;
    }
}

/*
 * Recommended level-3 API after recognition:
 *   plan_bomb_and_push_phase3(map);
 * It plans the bomb phase first, updates the simulated map, and then plans
 * the box-pushing phase from the bomb phase end position.
 * Execute bomb_commands3 first, then push_commands3.
 */
void plan_bomb_and_push_phase3(char external_map[12][16])
{
    plan_secondary_bomb_phase3(external_map);
    plan_push_phase3(external_map);
}


// ============================================================================
// Stable staged controller: explore -> bomb -> re-explore -> final push
// ============================================================================

static bool level3_is_box_label_c(char c)
{
    return (c >= 'A' && c <= 'Z');
}

static bool level3_is_target_label_c(char c)
{
    return (c >= 'a' && c <= 'z');
}

static bool level3_pos_is_remaining_bomb_c(int x, int y)
{
    for (int i = 0; i < num_bombs; i++) {
        if (bombs[i].x == x && bombs[i].y == y) return true;
    }
    return false;
}

/*
 * Mirror terrain and bomb changes to the caller's map without destroying
 * unrecognised ground-truth letters that may be used by a PC test harness.
 */
static void level3_mirror_world_to_external_c(char external_map[12][16])
{
    bool object_cell[12][16] = {{false}};

    for (int i = 0; i < num_objs; i++) {
        Pos3 p = all_objs[i].p;
        if (p.x >= 0 && p.x < 16 && p.y >= 0 && p.y < 12) {
            object_cell[p.y][p.x] = true;
        }
    }

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            if (object_cell[y][x]) continue;
            external_map[y][x] = dynamic_map[y][x];
        }
    }

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            if (level3_pos_is_remaining_bomb_c(x, y)) {
                external_map[y][x] = '*';
            }
        }
    }

    for (int i = 0; i < num_objs; i++) {
        if ((level3_staged_recognized_mask & (1u << i)) == 0) continue;
        Pos3 p = all_objs[i].p;
        char c = dynamic_map[p.y][p.x];
        if ((all_objs[i].type == 0 && level3_is_box_label_c(c)) ||
            (all_objs[i].type == 1 && level3_is_target_label_c(c))) {
            external_map[p.y][p.x] = c;
        }
    }
}

static void level3_reset_public_outputs_c(void)
{
    explore_steps3 = 0;
    explore_commands3[0] = '\0';
    expected_cnt3 = 0;
    memset(expected_targets3, 0, sizeof(expected_targets3));

    bomb_steps3 = 0;
    bomb_commands3[0] = '\0';

    push_steps3 = 0;
    push_commands3[0] = '\0';

    prepare_steps3 = 0;
    prepare_commands3[0] = '\0';
}

/*
 * A recognition segment is considered committed only after the caller has
 * written a valid camera result to external_map. This prevents the planner
 * from silently applying n-1 before the latest recognition result exists.
 */
static bool level3_commit_pending_recognition_c(char external_map[12][16])
{
    if (level3_staged_pending_object < 0) return true;

    int idx = level3_staged_pending_object;
    if (idx >= num_objs) {
        level3_last_error3 = 11;
        return false;
    }

    Pos3 p = all_objs[idx].p;
    char c = external_map[p.y][p.x];
    bool valid = false;

    if (all_objs[idx].type == 0) valid = level3_is_box_label_c(c);
    else valid = level3_is_target_label_c(c);

    if (!valid) {
        level3_last_error3 = 12; /* waiting for camera result */
        return false;
    }

    dynamic_map[p.y][p.x] = c;
    level3_staged_recognized_mask |= (1u << idx);
    level3_staged_pending_object = -1;
    level3_last_error3 = 0;
    return true;
}

/*
 * Safe n-1 completion.
 * It never invents a fresh box-target pair. Multiple unrecognised objects are
 * completed only when every one of them is forced to the same repeated label.
 */
static bool level3_try_complete_labels_c(char external_map[12][16])
{
    int box_count[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};
    int target_count[SOKOBAN_ELEMENT_TYPE_CAPACITY] = {0};
    int unknown_boxes[20];
    int unknown_targets[20];
    int ub = 0, ut = 0;

    for (int i = 0; i < num_objs; i++) {
        if ((level3_staged_recognized_mask & (1u << i)) == 0) {
            if (all_objs[i].type == 0) unknown_boxes[ub++] = i;
            else unknown_targets[ut++] = i;
            continue;
        }

        Pos3 p = all_objs[i].p;
        char c = dynamic_map[p.y][p.x];
        if (level3_is_box_label_c(c)) box_count[c - 'A']++;
        if (level3_is_target_label_c(c)) target_count[c - 'a']++;
    }

    if (ub == 0 && ut == 0) return true;

    int box_need_total = 0;
    int target_need_total = 0;
    int box_need_label = -1;
    int target_need_label = -1;
    bool box_need_mixed = false;
    bool target_need_mixed = false;

    for (int i = 0; i < SOKOBAN_ELEMENT_TYPE_CAPACITY; i++) {
        int need_box = target_count[i] - box_count[i];
        int need_target = box_count[i] - target_count[i];

        if (need_box > 0) {
            box_need_total += need_box;
            if (box_need_label < 0) box_need_label = i;
            else if (box_need_label != i) box_need_mixed = true;
        }

        if (need_target > 0) {
            target_need_total += need_target;
            if (target_need_label < 0) target_need_label = i;
            else if (target_need_label != i) target_need_mixed = true;
        }
    }

    if (ub > 0) {
        if (box_need_total != ub) return false;
        if (box_need_label < 0) return false;
        if (ub > 1 && box_need_mixed) return false;
    } else if (box_need_total != 0) {
        return false;
    }

    if (ut > 0) {
        if (target_need_total != ut) return false;
        if (target_need_label < 0) return false;
        if (ut > 1 && target_need_mixed) return false;
    } else if (target_need_total != 0) {
        return false;
    }

    /* For one unknown object, mixed deficits cannot happen in a valid n-1 map. */
    if (ub == 1 && box_need_mixed) return false;
    if (ut == 1 && target_need_mixed) return false;

    for (int k = 0; k < ub; k++) {
        int idx = unknown_boxes[k];
        Pos3 p = all_objs[idx].p;
        dynamic_map[p.y][p.x] = (char)('A' + box_need_label);
        level3_staged_recognized_mask |= (1u << idx);
    }

    for (int k = 0; k < ut; k++) {
        int idx = unknown_targets[k];
        Pos3 p = all_objs[idx].p;
        dynamic_map[p.y][p.x] = (char)('a' + target_need_label);
        level3_staged_recognized_mask |= (1u << idx);
    }

    level3_mirror_world_to_external_c(external_map);
    return true;
}

static void level3_apply_pose_commands_c(const char *path, int len)
{
    for (int i = 0; i < len; i++) {
        char c = path[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));

        if (c == 'U') level3_staged_player.y--;
        else if (c == 'D') level3_staged_player.y++;
        else if (c == 'L') level3_staged_player.x--;
        else if (c == 'R') level3_staged_player.x++;
        else if (c == 'E') {
            if (level3_staged_view_dir == 'U') level3_staged_view_dir = 'R';
            else if (level3_staged_view_dir == 'R') level3_staged_view_dir = 'D';
            else if (level3_staged_view_dir == 'D') level3_staged_view_dir = 'L';
            else level3_staged_view_dir = 'U';
        } else if (c == 'Q') {
            if (level3_staged_view_dir == 'U') level3_staged_view_dir = 'L';
            else if (level3_staged_view_dir == 'L') level3_staged_view_dir = 'D';
            else if (level3_staged_view_dir == 'D') level3_staged_view_dir = 'R';
            else level3_staged_view_dir = 'U';
        } else if (c == 'B') {
            if (level3_staged_view_dir == 'U') level3_staged_view_dir = 'D';
            else if (level3_staged_view_dir == 'D') level3_staged_view_dir = 'U';
            else if (level3_staged_view_dir == 'L') level3_staged_view_dir = 'R';
            else level3_staged_view_dir = 'L';
        }
    }

    explore_end_pos = level3_staged_player;
    explore_end_dir = level3_staged_view_dir;
}

/* Plan one camera target. One target per call is deliberate: the next call
 * cannot continue until that target's real camera result has been committed. */
static bool level3_plan_one_recognition_c(void)
{
    int best_idx = -1;
    int best_cost = 999999;
    int best_len = 0;
    char best_path[LEVEL3_INTERNAL_PATH_CAPACITY3];

    for (int i = 0; i < num_objs; i++) {
        if (level3_staged_recognized_mask & (1u << i)) continue;

        global_best_cost = 999999;
        global_best_len = 0;
        char tmp_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
        dfs_tsp(13, level3_staged_player, level3_staged_view_dir,
                (1u << i), 0, tmp_path, 0);

        if (global_best_cost < best_cost && global_best_len > 0) {
            best_idx = i;
            best_cost = global_best_cost;
            best_len = global_best_len;
            memcpy(best_path, global_best_path, global_best_len);
        }
    }

    if (best_idx < 0) return false;
    if (best_len >= (int)sizeof(explore_commands3)) {
        level3_last_error3 = 21;
        return false;
    }

    memcpy(explore_commands3, best_path, best_len);
    explore_steps3 = best_len;
    explore_commands3[explore_steps3] = '\0';
    expected_targets3[0] = all_objs[best_idx].p;
    expected_cnt3 = 1;
    level3_staged_pending_object = best_idx;

    level3_apply_pose_commands_c(best_path, best_len);
    return true;
}

static int level3_count_reachable_unrecognized_state_c(
    Pos3 from, Pos3 *state_bombs, int state_nb)
{
    Pos3 saved_bombs[MAX_BOMBS3];
    int saved_nb = num_bombs;
    memcpy(saved_bombs, bombs, sizeof(saved_bombs));
    memcpy(bombs, state_bombs, sizeof(bombs));
    num_bombs = state_nb;

    int count = 0;
    const int dx[4] = {0, 0, -1, 1};
    const int dy[4] = {-1, 1, 0, 0};

    for (int i = 0; i < num_objs; i++) {
        if (level3_staged_recognized_mask & (1u << i)) continue;

        bool reachable = false;
        for (int d = 0; d < 4; d++) {
            Pos3 stand = {all_objs[i].p.x + dx[d], all_objs[i].p.y + dy[d]};
            bool target_passable = (all_objs[i].type == 1);
            if (is_obstacle_c(stand.x, stand.y, target_passable)) continue;

            int len = 0;
            char path[LEVEL3_INTERNAL_PATH_CAPACITY3];
            if (bfs_move_only_c(from, stand, target_passable, &len, path)) {
                reachable = true;
                break;
            }
        }

        if (reachable) count++;
    }

    memcpy(bombs, saved_bombs, sizeof(bombs));
    num_bombs = saved_nb;
    return count;
}

/*
 * Dijkstra over bomb/world states. The goal is not box solvability yet; it is
 * simply to make at least one still-unrecognised object reachable. This makes
 * bomb chains work even when the first explosion only unlocks the second bomb.
 */
static bool level3_dijkstra_unlock_recognition_c(
    Pos3 start_p, TspObj *out_seq, int *out_cnt, int *out_cost)
{
    char base_map[12][16];
    Pos3 base_bombs[MAX_BOMBS3];
    int base_nb = num_bombs;
    memcpy(base_map, dynamic_map, sizeof(base_map));
    memcpy(base_bombs, bombs, sizeof(base_bombs));

    memset(bomb_dij_nodes, 0, sizeof(bomb_dij_nodes));

    BombDijkstraNode *root = &bomb_dij_nodes[0];
    root->used = true;
    root->closed = false;
    root->cost = 0;
    root->player = start_p;
    root->nb = num_bombs;
    root->task_cnt = 0;
    memcpy(root->bombs_state, bombs, sizeof(bombs));
    memcpy(root->map_state, dynamic_map, sizeof(dynamic_map));
    root->hash = hash_map_state_c(root->map_state);

    while (1) {
        int idx = select_min_open_bomb_node_c();
        if (idx < 0) break;

        BombDijkstraNode *node = &bomb_dij_nodes[idx];
        node->closed = true;
        memcpy(dynamic_map, node->map_state, sizeof(dynamic_map));

        if (node->task_cnt > 0 &&
            level3_count_reachable_unrecognized_state_c(
                node->player, node->bombs_state, node->nb) > 0) {
            memcpy(out_seq, node->tasks, sizeof(TspObj) * node->task_cnt);
            *out_cnt = node->task_cnt;
            *out_cost = node->cost;
            memcpy(dynamic_map, base_map, sizeof(base_map));
            memcpy(bombs, base_bombs, sizeof(base_bombs));
            num_bombs = base_nb;
            return true;
        }

        if (node->nb <= 0 || node->task_cnt >= BOMB_TASK_LIMIT) continue;

        for (int bi = 0; bi < node->nb; bi++) {
            Pos3 used_bomb = node->bombs_state[bi];
            get_all_reachable_walls_c(
                node->player, used_bomb, node->bombs_state, node->nb);

            int c_cnt = num_reach_walls;
            ReachableWall candidates[60];
            for (int k = 0; k < c_cnt; k++) candidates[k] = reach_walls[k];

            for (int ci = 0; ci < c_cnt; ci++) {
                Pos3 wall = candidates[ci].wall;
                char child_map[12][16];
                Pos3 child_bombs[MAX_BOMBS3];
                int child_nb = 0;

                memcpy(child_map, node->map_state, sizeof(child_map));
                blast_on_map_c(child_map, wall);
                remove_bomb_from_state_c(
                    node->bombs_state, node->nb, bi, child_bombs, &child_nb);

                int new_cost = node->cost + candidates[ci].path_len;
                uint32_t child_hash = hash_map_state_c(child_map);
                int same_idx = find_same_bomb_node_c(
                    child_map, child_hash, candidates[ci].end_p,
                    child_bombs, child_nb);

                int dst = same_idx;
                if (dst >= 0) {
                    if (bomb_dij_nodes[dst].closed ||
                        new_cost >= bomb_dij_nodes[dst].cost) continue;
                } else {
                    dst = alloc_bomb_node_c(new_cost);
                    if (dst < 0) continue;
                }

                BombDijkstraNode *child = &bomb_dij_nodes[dst];
                child->used = true;
                child->closed = false;
                child->cost = new_cost;
                child->player = candidates[ci].end_p;
                child->nb = child_nb;
                child->task_cnt = node->task_cnt + 1;
                memcpy(child->bombs_state, child_bombs, sizeof(child->bombs_state));
                memcpy(child->map_state, child_map, sizeof(child->map_state));
                child->hash = child_hash;
                memcpy(child->tasks, node->tasks,
                       sizeof(TspObj) * node->task_cnt);
                child->tasks[node->task_cnt] =
                    (TspObj){2, -1, used_bomb, '*', wall};
            }
        }
    }

    memcpy(dynamic_map, base_map, sizeof(base_map));
    memcpy(bombs, base_bombs, sizeof(base_bombs));
    num_bombs = base_nb;
    *out_cnt = 0;
    *out_cost = 999999;
    return false;
}


/* Find the cheapest bomb chain that makes at least required_count unknown
 * objects reachable. This is used by the fixed bomb-first flow so that the
 * later recognition phase can finish without another explosion. */
static bool level3_dijkstra_reach_count_c(
    Pos3 start_p, int required_count,
    TspObj *out_seq, int *out_cnt, int *out_cost)
{
    char base_map[12][16];
    Pos3 base_bombs[MAX_BOMBS3];
    int base_nb = num_bombs;
    memcpy(base_map, dynamic_map, sizeof(base_map));
    memcpy(base_bombs, bombs, sizeof(base_bombs));

    memset(bomb_dij_nodes, 0, sizeof(bomb_dij_nodes));
    BombDijkstraNode *root = &bomb_dij_nodes[0];
    root->used = true;
    root->closed = false;
    root->cost = 0;
    root->player = start_p;
    root->nb = num_bombs;
    root->task_cnt = 0;
    memcpy(root->bombs_state, bombs, sizeof(bombs));
    memcpy(root->map_state, dynamic_map, sizeof(dynamic_map));
    root->hash = hash_map_state_c(root->map_state);

    while (1) {
        int idx3 = select_min_open_bomb_node_c();
        if (idx3 < 0) break;

        BombDijkstraNode *node = &bomb_dij_nodes[idx3];
        node->closed = true;
        memcpy(dynamic_map, node->map_state, sizeof(dynamic_map));

        int reachable = level3_count_reachable_unrecognized_state_c(
            node->player, node->bombs_state, node->nb);
        if (node->task_cnt > 0 && reachable >= required_count) {
            memcpy(out_seq, node->tasks, sizeof(TspObj) * node->task_cnt);
            *out_cnt = node->task_cnt;
            *out_cost = node->cost;
            memcpy(dynamic_map, base_map, sizeof(base_map));
            memcpy(bombs, base_bombs, sizeof(base_bombs));
            num_bombs = base_nb;
            return true;
        }

        if (node->nb <= 0 || node->task_cnt >= BOMB_TASK_LIMIT) continue;

        for (int bi = 0; bi < node->nb; bi++) {
            Pos3 used_bomb = node->bombs_state[bi];
            get_all_reachable_walls_c(
                node->player, used_bomb, node->bombs_state, node->nb);

            int c_cnt = num_reach_walls;
            ReachableWall candidates[60];
            for (int k = 0; k < c_cnt; k++) candidates[k] = reach_walls[k];

            for (int ci = 0; ci < c_cnt; ci++) {
                Pos3 wall = candidates[ci].wall;
                char child_map[12][16];
                Pos3 child_bombs[MAX_BOMBS3];
                int child_nb = 0;

                memcpy(child_map, node->map_state, sizeof(child_map));
                blast_on_map_c(child_map, wall);
                remove_bomb_from_state_c(
                    node->bombs_state, node->nb, bi, child_bombs, &child_nb);

                int new_cost = node->cost + candidates[ci].path_len;
                uint32_t child_hash = hash_map_state_c(child_map);
                int same_idx = find_same_bomb_node_c(
                    child_map, child_hash, candidates[ci].end_p,
                    child_bombs, child_nb);

                int dst = same_idx;
                if (dst >= 0) {
                    if (bomb_dij_nodes[dst].closed ||
                        new_cost >= bomb_dij_nodes[dst].cost) continue;
                } else {
                    dst = alloc_bomb_node_c(new_cost);
                    if (dst < 0) continue;
                }

                BombDijkstraNode *child = &bomb_dij_nodes[dst];
                child->used = true;
                child->closed = false;
                child->cost = new_cost;
                child->player = candidates[ci].end_p;
                child->nb = child_nb;
                child->task_cnt = node->task_cnt + 1;
                memcpy(child->bombs_state, child_bombs, sizeof(child->bombs_state));
                memcpy(child->map_state, child_map, sizeof(child->map_state));
                child->hash = child_hash;
                memcpy(child->tasks, node->tasks,
                       sizeof(TspObj) * node->task_cnt);
                child->tasks[node->task_cnt] =
                    (TspObj){2, -1, used_bomb, '*', wall};
            }
        }
    }

    memcpy(dynamic_map, base_map, sizeof(base_map));
    memcpy(bombs, base_bombs, sizeof(base_bombs));
    num_bombs = base_nb;
    *out_cnt = 0;
    *out_cost = 999999;
    return false;
}


/* Plan a minimum-cost order that detonates every remaining bomb. Since a blast
 * only removes walls, this conservative preliminary mode opens the terrain
 * before any label is observed and never needs another bomb after recognition. */
static bool level3_dijkstra_all_bombs_c(
    Pos3 start_p, TspObj *out_seq, int *out_cnt, int *out_cost)
{
    char base_map[12][16];
    Pos3 base_bombs[MAX_BOMBS3];
    int base_nb = num_bombs;
    memcpy(base_map, dynamic_map, sizeof(base_map));
    memcpy(base_bombs, bombs, sizeof(base_bombs));

    memset(bomb_dij_nodes, 0, sizeof(bomb_dij_nodes));
    BombDijkstraNode *root = &bomb_dij_nodes[0];
    root->used = true;
    root->cost = 0;
    root->player = start_p;
    root->nb = num_bombs;
    memcpy(root->bombs_state, bombs, sizeof(bombs));
    memcpy(root->map_state, dynamic_map, sizeof(dynamic_map));
    root->hash = hash_map_state_c(root->map_state);

    while (1) {
        int idx3 = select_min_open_bomb_node_c();
        if (idx3 < 0) break;
        BombDijkstraNode *node = &bomb_dij_nodes[idx3];
        node->closed = true;
        memcpy(dynamic_map, node->map_state, sizeof(dynamic_map));

        if (node->nb == 0) {
            memcpy(out_seq, node->tasks, sizeof(TspObj) * node->task_cnt);
            *out_cnt = node->task_cnt;
            *out_cost = node->cost;
            memcpy(dynamic_map, base_map, sizeof(base_map));
            memcpy(bombs, base_bombs, sizeof(base_bombs));
            num_bombs = base_nb;
            return true;
        }

        if (node->task_cnt >= BOMB_TASK_LIMIT) continue;

        for (int bi = 0; bi < node->nb; bi++) {
            Pos3 used_bomb = node->bombs_state[bi];
            get_all_reachable_walls_c(
                node->player, used_bomb, node->bombs_state, node->nb);

            int c_cnt = num_reach_walls;
            ReachableWall candidates[60];
            for (int k = 0; k < c_cnt; k++) candidates[k] = reach_walls[k];

            for (int ci = 0; ci < c_cnt; ci++) {
                Pos3 wall = candidates[ci].wall;
                int destroyed = 0;
                for (int ey = -1; ey <= 1; ey++) {
                    for (int ex = -1; ex <= 1; ex++) {
                        int tx = wall.x + ex, ty = wall.y + ey;
                        if (tx > 0 && tx < 15 && ty > 0 && ty < 11 &&
                            node->map_state[ty][tx] == '#') destroyed++;
                    }
                }
                if (destroyed <= 0) continue;

                char child_map[12][16];
                Pos3 child_bombs[MAX_BOMBS3];
                int child_nb = 0;
                memcpy(child_map, node->map_state, sizeof(child_map));
                blast_on_map_c(child_map, wall);
                remove_bomb_from_state_c(
                    node->bombs_state, node->nb, bi, child_bombs, &child_nb);

                /* Path length dominates. The small penalty prefers a blast that
                 * removes more walls when movement costs are close. */
                int new_cost = node->cost + candidates[ci].path_len +
                               (9 - destroyed) * 2;
                uint32_t child_hash = hash_map_state_c(child_map);
                int same_idx = find_same_bomb_node_c(
                    child_map, child_hash, candidates[ci].end_p,
                    child_bombs, child_nb);

                int dst = same_idx;
                if (dst >= 0) {
                    if (bomb_dij_nodes[dst].closed ||
                        new_cost >= bomb_dij_nodes[dst].cost) continue;
                } else {
                    dst = alloc_bomb_node_c(new_cost);
                    if (dst < 0) continue;
                }

                BombDijkstraNode *child = &bomb_dij_nodes[dst];
                child->used = true;
                child->closed = false;
                child->cost = new_cost;
                child->player = candidates[ci].end_p;
                child->nb = child_nb;
                child->task_cnt = node->task_cnt + 1;
                memcpy(child->bombs_state, child_bombs, sizeof(child->bombs_state));
                memcpy(child->map_state, child_map, sizeof(child->map_state));
                child->hash = child_hash;
                memcpy(child->tasks, node->tasks,
                       sizeof(TspObj) * node->task_cnt);
                child->tasks[node->task_cnt] =
                    (TspObj){2, -1, used_bomb, '*', wall};
            }
        }
    }

    memcpy(dynamic_map, base_map, sizeof(base_map));
    memcpy(bombs, base_bombs, sizeof(base_bombs));
    num_bombs = base_nb;
    *out_cnt = 0;
    *out_cost = 999999;
    return false;
}

/* Phase 1: initialize the world and finish all required explosions before
 * recognition. The output path is level3_commands3[0..level3_steps3-1]. */

typedef struct {
    Pos3 bomb;
    Pos3 wall;
    Pos3 end_p;
    int path_len;
    int rank_score;
} BombFirstCandidate3;

#ifndef BOMB_FIRST_BRANCH3
/* Keep only the six strongest structural bomb candidates at each depth.
 * On the competition chain-bomb map this preserves the 18-step bomb route
 * while cutting bomb-planning time by roughly an order of magnitude. */
#define BOMB_FIRST_BRANCH3 6
#endif

static int bomb_first_best_cost3 = 999999;
static int bomb_first_best_pair_count3 = -1;
static int bomb_first_best_cnt3 = 0;
static TspObj bomb_first_best_seq3[BOMB_TASK_LIMIT];

/* Fast structural box-target score used only to rank completed bomb plans.
 * Reverse-pull reachability is exact for static terrain and avoids the much
 * larger player+box BFS for every box/target pair. Other boxes are ignored,
 * matching the old ranking function's terrain-only intent. */
static int level3_box_target_pair_count_c(
    char state_map[12][16], Pos3 player)
{
    (void)player;
    Pos3 boxes3[MAX_BOXES3];
    Pos3 targets3[MAX_BOXES3];
    int nb3 = 0, nt3 = 0;

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = state_map[y][x];
            if ((c == '$' || (c >= 'A' && c <= 'Z')) && nb3 < MAX_BOXES3)
                boxes3[nb3++] = (Pos3){x, y};
            else if ((c == '.' || (c >= 'a' && c <= 'z')) && nt3 < MAX_BOXES3)
                targets3[nt3++] = (Pos3){x, y};
        }
    }

    int count3 = 0;
    static const int dx3[4] = {0, 0, -1, 1};
    static const int dy3[4] = {-1, 1, 0, 0};

    for (int t = 0; t < nt3; t++) {
        uint8_t seen3[12][16] = {{0}};
        uint8_t queue3[192];
        int head3 = 0, tail3 = 0;
        int start3 = targets3[t].y * 16 + targets3[t].x;
        queue3[tail3++] = (uint8_t)start3;
        seen3[targets3[t].y][targets3[t].x] = 1;

        while (head3 < tail3) {
            int cell3 = queue3[head3++];
            int cx3 = cell3 & 15;
            int cy3 = cell3 >> 4;

            for (int d3 = 0; d3 < 4; d3++) {
                int px3 = cx3 - dx3[d3];
                int py3 = cy3 - dy3[d3];
                int sx3 = px3 - dx3[d3];
                int sy3 = py3 - dy3[d3];

                if (px3 <= 0 || px3 >= 15 || py3 <= 0 || py3 >= 11 ||
                    sx3 <= 0 || sx3 >= 15 || sy3 <= 0 || sy3 >= 11)
                    continue;
                if (state_map[py3][px3] == '#' || state_map[sy3][sx3] == '#')
                    continue;
                if (seen3[py3][px3]) continue;

                seen3[py3][px3] = 1;
                queue3[tail3++] = (uint8_t)(py3 * 16 + px3);
            }
        }

        for (int b = 0; b < nb3; b++) {
            if (seen3[boxes3[b].y][boxes3[b].x]) count3++;
        }
    }
    return count3;
}

static void level3_dfs_all_bombs_c(
    char state_map[12][16], Pos3 player,
    Pos3 *state_bombs, int state_nb,
    int depth, int cost, TspObj *seq)
{
    if (state_nb <= 0) {
        int pair_count3 = level3_box_target_pair_count_c(state_map, player);
        if (pair_count3 > bomb_first_best_pair_count3 ||
            (pair_count3 == bomb_first_best_pair_count3 &&
             cost < bomb_first_best_cost3)) {
            bomb_first_best_pair_count3 = pair_count3;
            bomb_first_best_cost3 = cost;
            bomb_first_best_cnt3 = depth;
            memcpy(bomb_first_best_seq3, seq, sizeof(TspObj) * depth);
        }
        return;
    }
    if (depth >= BOMB_TASK_LIMIT) return;

    memcpy(dynamic_map, state_map, sizeof(dynamic_map));
    get_blocking_walls_c(state_bombs, state_nb);

    /* Only the strongest branch candidates are ever expanded. Keep a sorted
     * top-K list while enumerating instead of storing up to 120 candidates and
     * performing an O(N^2) full sort at every DFS node. Equal scores preserve
     * their original encounter order, so selected paths remain deterministic. */
    BombFirstCandidate3 candidates[BOMB_FIRST_BRANCH3];
    int candidate_cnt = 0;

    for (int bi = 0; bi < state_nb; bi++) {
        Pos3 used_bomb = state_bombs[bi];
        get_all_reachable_walls_c(player, used_bomb, state_bombs, state_nb);

        int c_cnt = num_reach_walls;
        ReachableWall walls[60];
        for (int k = 0; k < c_cnt; k++) walls[k] = reach_walls[k];

        for (int ci = 0; ci < c_cnt; ci++) {
            Pos3 wall = walls[ci].wall;
            int destroyed = 0;
            int blocking = 0;

            for (int ey = -1; ey <= 1; ey++) {
                for (int ex = -1; ex <= 1; ex++) {
                    int tx = wall.x + ex, ty = wall.y + ey;
                    if (tx > 0 && tx < 15 && ty > 0 && ty < 11 &&
                        state_map[ty][tx] == '#') {
                        destroyed++;
                        for (int bw = 0; bw < num_blocking_walls; bw++) {
                            if (blocking_walls[bw].x == tx &&
                                blocking_walls[bw].y == ty) {
                                blocking++;
                                break;
                            }
                        }
                    }
                }
            }
            if (destroyed <= 0) continue;

            BombFirstCandidate3 cand3 = {
                used_bomb, wall, walls[ci].end_p, walls[ci].path_len,
                blocking * 1000 + destroyed * 100 - walls[ci].path_len
            };
            int pos3 = 0;
            while (pos3 < candidate_cnt &&
                   candidates[pos3].rank_score >= cand3.rank_score)
                pos3++;
            if (pos3 >= BOMB_FIRST_BRANCH3) continue;

            int new_count3 = candidate_cnt;
            if (new_count3 < BOMB_FIRST_BRANCH3) new_count3++;
            for (int move3 = new_count3 - 1; move3 > pos3; move3--)
                candidates[move3] = candidates[move3 - 1];
            candidates[pos3] = cand3;
            candidate_cnt = new_count3;
        }
    }

    for (int ci = 0; ci < candidate_cnt; ci++) {
        BombFirstCandidate3 *cand = &candidates[ci];
        int bi = -1;
        for (int k = 0; k < state_nb; k++) {
            if (state_bombs[k].x == cand->bomb.x &&
                state_bombs[k].y == cand->bomb.y) {
                bi = k;
                break;
            }
        }
        if (bi < 0) continue;

        char child_map[12][16];
        Pos3 child_bombs[MAX_BOMBS3];
        int child_nb = 0;
        memcpy(child_map, state_map, sizeof(child_map));
        blast_on_map_c(child_map, cand->wall);
        remove_bomb_from_state_c(
            state_bombs, state_nb, bi, child_bombs, &child_nb);

        seq[depth] = (TspObj){2, -1, cand->bomb, '*', cand->wall};
        level3_dfs_all_bombs_c(
            child_map, cand->end_p, child_bombs, child_nb,
            depth + 1, cost + cand->path_len + 1, seq);
    }

    memcpy(dynamic_map, state_map, sizeof(dynamic_map));
}

static bool level3_find_all_bombs_plan_c(
    Pos3 start_p, TspObj *out_seq, int *out_cnt, int *out_cost)
{
    char start_map[12][16];
    Pos3 start_bombs[MAX_BOMBS3];
    TspObj seq[BOMB_TASK_LIMIT];
    memcpy(start_map, dynamic_map, sizeof(start_map));
    memcpy(start_bombs, bombs, sizeof(start_bombs));

    bomb_first_best_cost3 = 999999;
    bomb_first_best_pair_count3 = -1;
    bomb_first_best_cnt3 = 0;
    level3_dfs_all_bombs_c(
        start_map, start_p, start_bombs, num_bombs, 0, 0, seq);

    memcpy(dynamic_map, start_map, sizeof(dynamic_map));
    if (bomb_first_best_cnt3 <= 0) {
        *out_cnt = 0;
        *out_cost = 999999;
        return false;
    }

    memcpy(out_seq, bomb_first_best_seq3,
           sizeof(TspObj) * bomb_first_best_cnt3);
    *out_cnt = bomb_first_best_cnt3;
    *out_cost = bomb_first_best_cost3;
    return true;
}

void plan_bomb_phase3(char external_map[12][16])
{
    level3_internal_reset3();
    level3_bomb_first_initialized3 = true;
    memcpy(dynamic_map, external_map, sizeof(dynamic_map));
    if (level3_input_bomb_count3(external_map) > SOKOBAN_LEVEL3_BOMB_LIMIT) {
        level3_bomb_first_initialized3 = false;
        level3_error3 = 1;
        level3_error_code3 = 70;
        level3_bombs_used3 = 0U;
        return;
    }

    Pos3 cur_p = {1, 5};
    num_bombs = 0;
    num_objs = 0;

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (c == '@') {
                cur_p = (Pos3){x, y};
                dynamic_map[y][x] = '-';
            } else if (c == '*') {
                if (num_bombs < MAX_BOMBS3) bombs[num_bombs++] = (Pos3){x, y};
                dynamic_map[y][x] = '-';
            } else if (c == '$' || (c >= 'A' && c <= 'Z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int object_index3 = num_objs;
                    all_objs[object_index3] =
                        (TspObj){0, object_index3, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
            } else if (c == '.' || (c >= 'a' && c <= 'z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int object_index3 = num_objs;
                    all_objs[object_index3] =
                        (TspObj){1, object_index3, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
            }
        }
    }

    explore_end_pos = cur_p;
    explore_end_dir = 'U';
    bomb_steps3 = 0;
    level3_steps3 = 0;
    level3_commands3[0] = '\0';
    level3_error3 = 0;
    level3_error_code3 = 0;
    level3_segment_type3 = 2;

    /* Compute the complete bomb chain once, then execute the selected tasks.
     * The previous version recomputed the same remaining chain after every
     * blast. Reusing the selected sequence removes most bomb-phase CPU time
     * without adding a persistent node pool or changing path semantics. */
    if (num_bombs > 0) {
        TspObj seq[BOMB_TASK_LIMIT];
        int seq_cnt = 0;
        int seq_cost = 999999;
        if (!level3_find_all_bombs_plan_c(
                cur_p, seq, &seq_cnt, &seq_cost) || seq_cnt <= 0) {
            level3_error3 = 1;
            level3_error_code3 = 71;
        } else {
            for (int round = 0; round < seq_cnt; round++) {
                Pos3 best_bomb = seq[round].p;
                Pos3 best_wall = seq[round].target_wall;
                int p_len = 0;
                char p_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
                char last_dir = 0;
                Pos3 end_p = cur_p;

                if (!bfs_push_specific_bomb(
                        cur_p, best_bomb, best_wall,
                        &p_len, p_path, &last_dir, &end_p)) {
                    level3_error3 = 1;
                    level3_error_code3 = 72;
                    break;
                }
                if (bomb_steps3 + p_len + 1 >= LEVEL3_PATH_CAPACITY3) {
                    level3_error3 = 1;
                    level3_error_code3 = 73;
                    break;
                }

                for (int k = 0; k < p_len; k++) append_bomb_cmd_c(p_path[k]);
                if (last_dir == 'U' || last_dir == 'D' ||
                    last_dir == 'L' || last_dir == 'R')
                    append_bomb_cmd_c(last_dir);

                cur_p = end_p;
                apply_blast_c(best_wall);
                remove_bomb_at_c(best_bomb);
                if (level3_bombs_used3 < SOKOBAN_LEVEL3_BOMB_LIMIT)
                    level3_bombs_used3++;
            }
        }
    }

    explore_end_pos = cur_p;
    explore_end_dir = 'U';
    memcpy(external_map, dynamic_map, sizeof(dynamic_map));
    level3_steps3 = bomb_steps3;
    level3_commands3[level3_steps3] = '\0';
}


/* ========================================================================== */
/* First progressive fusion version:                                          */
/* 1. Keep the bomb sequence selected by plan_bomb_phase3().                  */
/* 2. Insert recognition commands at positions already passed by that route.  */
/* 3. Optimize the remaining N-1 recognition route from the final bomb pose.  */
/*                                                                            */
/* To keep this version small and deterministic, each opportunistic            */
/* recognition restores the camera to U before the bomb route continues.      */
/* Therefore the original bomb path remains physically unchanged.             */
/* ========================================================================== */

/* Cache budget only.  This is NOT the current recognition-object count:
 * masks outside this cache are still solved normally, just without reuse. */
#ifndef LEVEL3_JOINT_CACHE_MASK_BITS3
#define LEVEL3_JOINT_CACHE_MASK_BITS3 10U
#endif
#define LEVEL3_JOINT_CACHE_MASK_COUNT3 (1U << LEVEL3_JOINT_CACHE_MASK_BITS3)

int bomb_base_steps3 = 0;
int route_recognition_cnt3 = 0;
int remaining_recognition_cnt3 = 0;
uint8_t level3_joint_fallback3 = 0;

/* Push-aware terminal diagnostics. The labels are still unknown while the
 * joint route is selected, so terminal cost is evaluated over the possible
 * box-target pairings (small maps) instead of using one arbitrary pairing. */
int level3_joint_candidate_count3 = 0;
int level3_joint_terminal_push_estimate3 = 999999;
int level3_joint_front_cost3 = 999999;

#ifndef LEVEL3_JOINT_TERMINAL_CANDIDATES3
#define LEVEL3_JOINT_TERMINAL_CANDIDATES3 8
#endif
#ifndef LEVEL3_JOINT_UNKNOWN_PUSH_MAX3
#define LEVEL3_JOINT_UNKNOWN_PUSH_MAX3 4
#endif
#ifndef LEVEL3_JOINT_UNSOLVABLE_PENALTY3
#define LEVEL3_JOINT_UNSOLVABLE_PENALTY3 1000
#endif

static char joint_bomb_base_path3[2000];
static int joint_tsp_cost_cache3[LEVEL3_JOINT_CACHE_MASK_COUNT3];
static uint8_t joint_tsp_valid3[LEVEL3_JOINT_CACHE_MASK_COUNT3];
static uint8_t joint_tsp_end_cell_cache3[LEVEL3_JOINT_CACHE_MASK_COUNT3];
static char joint_tsp_dir_cache3[LEVEL3_JOINT_CACHE_MASK_COUNT3];

static char joint_view_dir3(Pos3 from, Pos3 object)
{
    if (object.x == from.x && object.y == from.y - 1) return 'U';
    if (object.x == from.x && object.y == from.y + 1) return 'D';
    if (object.x == from.x - 1 && object.y == from.y) return 'L';
    if (object.x == from.x + 1 && object.y == from.y) return 'R';
    return 0;
}

static void joint_apply_pose_cmd3(Pos3 *p, char *dir, char cmd)
{
    char c = cmd;
    if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));

    if (c == 'U') p->y--;
    else if (c == 'D') p->y++;
    else if (c == 'L') p->x--;
    else if (c == 'R') p->x++;
    else if (c == 'E') {
        if (*dir == 'U') *dir = 'R';
        else if (*dir == 'R') *dir = 'D';
        else if (*dir == 'D') *dir = 'L';
        else *dir = 'U';
    } else if (c == 'Q') {
        if (*dir == 'U') *dir = 'L';
        else if (*dir == 'L') *dir = 'D';
        else if (*dir == 'D') *dir = 'R';
        else *dir = 'U';
    } else if (c == 'B') {
        if (*dir == 'U') *dir = 'D';
        else if (*dir == 'D') *dir = 'U';
        else if (*dir == 'L') *dir = 'R';
        else *dir = 'L';
    }
}

static int joint_route_insert_cost3(char view_dir)
{
    if (view_dir == 'U') return 1;       /* p */
    if (view_dir == 'D') return 4;       /* b + B, each 180 deg costs 2 */
    return 2;                            /* e/Q or q/E */
}

static bool joint_append_cmd3(char c)
{
    if (level3_steps3 >= (int)sizeof(level3_commands3) - 1) return false;
    level3_commands3[level3_steps3++] = c;
    level3_commands3[level3_steps3] = '\0';
    return true;
}

static bool joint_append_route_snapshot3(char view_dir)
{
    if (view_dir == 'U') return joint_append_cmd3('p');
    if (view_dir == 'R') return joint_append_cmd3('e') && joint_append_cmd3('Q');
    if (view_dir == 'L') return joint_append_cmd3('q') && joint_append_cmd3('E');
    if (view_dir == 'D') return joint_append_cmd3('b') && joint_append_cmd3('B');
    return false;
}

static void joint_rebuild_objects3(void)
{
    num_objs = 0;
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (c == '$' || (c >= 'A' && c <= 'Z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int idx = num_objs;
                    all_objs[idx] = (TspObj){0, idx, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
            } else if (c == '.' || (c >= 'a' && c <= 'z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int idx = num_objs;
                    all_objs[idx] = (TspObj){1, idx, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
            }
        }
    }
}

static int joint_tsp_solution3(
    uint32_t mask, Pos3 start_p, Pos3 *out_end, char *out_dir)
{
    if (mask == 0) {
        if (out_end) *out_end = start_p;
        if (out_dir) *out_dir = 'U';
        return 0;
    }

    if (mask < LEVEL3_JOINT_CACHE_MASK_COUNT3 &&
        joint_tsp_valid3[mask]) {
        if (out_end) {
            uint8_t cell = joint_tsp_end_cell_cache3[mask];
            *out_end = (Pos3){cell & 0x0F, cell >> 4};
        }
        if (out_dir) *out_dir = joint_tsp_dir_cache3[mask];
        return joint_tsp_cost_cache3[mask];
    }

    global_best_cost = 999999;
    global_best_len = 0;
    char tmp_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    dfs_tsp(13, start_p, 'U', mask, 0, tmp_path, 0);

    Pos3 end_p = start_p;
    char end_dir = 'U';
    if (global_best_cost < 999999) {
        for (int i = 0; i < global_best_len; i++) {
            joint_apply_pose_cmd3(&end_p, &end_dir, global_best_path[i]);
        }
    }

    if (mask < LEVEL3_JOINT_CACHE_MASK_COUNT3) {
        joint_tsp_valid3[mask] = 1;
        joint_tsp_cost_cache3[mask] = global_best_cost;
        joint_tsp_end_cell_cache3[mask] =
            (uint8_t)(((end_p.y & 0x0F) << 4) | (end_p.x & 0x0F));
        joint_tsp_dir_cache3[mask] = end_dir;
    }

    if (out_end) *out_end = end_p;
    if (out_dir) *out_dir = end_dir;
    return global_best_cost;
}

/* Evaluate one completely labelled map without touching the public path. */
static int joint_eval_labeled_push_map3(
    char labelled_map[12][16], Pos3 start_p)
{
    memcpy(dynamic_map, labelled_map, sizeof(dynamic_map));
    if (!build_dyn_pairs_duplicate_aware_c()) return 999999;
    if (num_dyn_pairs == 0) return 0;

    best_push_steps = 999999;
    evaluate_all_duplicate_target_assignments_c(start_p, bombs, num_bombs);
    return best_push_steps;
}

typedef struct {
    Pos3 boxes[MAX_BOXES3];
    Pos3 targets[MAX_BOXES3];
    int n;
    int perm[MAX_BOXES3];
    uint8_t used[MAX_BOXES3];
    char base_map[12][16];
    Pos3 start_p;
    int sum_cost;
    int solved_count;
    int unsolved_count;
} JointUnknownPushCtx3;

static void joint_unknown_push_perm_rec3(JointUnknownPushCtx3 *ctx, int depth)
{
    if (depth >= ctx->n) {
        char labelled[12][16];
        memcpy(labelled, ctx->base_map, sizeof(labelled));
        for (int i = 0; i < ctx->n; i++) {
            labelled[ctx->boxes[i].y][ctx->boxes[i].x] = (char)('A' + i);
            int ti = ctx->perm[i];
            labelled[ctx->targets[ti].y][ctx->targets[ti].x] = (char)('a' + i);
        }

        int cost = joint_eval_labeled_push_map3(labelled, ctx->start_p);
        if (cost >= 999999) ctx->unsolved_count++;
        else {
            ctx->sum_cost += cost;
            ctx->solved_count++;
        }
        return;
    }

    for (int i = 0; i < ctx->n; i++) {
        if (ctx->used[i]) continue;
        ctx->used[i] = 1;
        ctx->perm[depth] = i;
        joint_unknown_push_perm_rec3(ctx, depth + 1);
        ctx->used[i] = 0;
    }
}

/* Label-independent terminal score. For the normal three-box competition map
 * this evaluates all 3! possible pairings and uses their average, with a large
 * penalty for pairings that cannot be solved. This makes the route end at a
 * position that is generally good for the later real push solution instead of
 * optimizing only the recognition prefix. */
static int joint_estimate_terminal_push3(Pos3 start_p)
{
    char saved_map[12][16];
    char saved_current[12][16];
    DynBoxTargetPair saved_pairs[MAX_BOXES3];
    char saved_best_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    int saved_num_pairs = num_dyn_pairs;
    int saved_best_steps = best_push_steps;
    memcpy(saved_map, dynamic_map, sizeof(saved_map));
    memcpy(saved_current, current_map, sizeof(saved_current));
    memcpy(saved_pairs, dyn_pairs, sizeof(saved_pairs));
    memcpy(saved_best_path, best_push_path, sizeof(saved_best_path));

    JointUnknownPushCtx3 ctx;
    memset(&ctx, 0, sizeof(ctx));
    memcpy(ctx.base_map, dynamic_map, sizeof(ctx.base_map));
    ctx.start_p = start_p;

    bool has_known_label = false;
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (c == '$' && ctx.n < MAX_BOXES3) {
                ctx.boxes[ctx.n++] = (Pos3){x, y};
            } else if ((c >= 'A' && c <= 'Z') ||
                       (c >= 'a' && c <= 'z')) {
                has_known_label = true;
            }
        }
    }

    int target_count = 0;
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            if (dynamic_map[y][x] == '.' && target_count < MAX_BOXES3) {
                ctx.targets[target_count++] = (Pos3){x, y};
            }
        }
    }

    int answer = 999999;
    if (!has_known_label && ctx.n == target_count && ctx.n > 0 &&
        ctx.n <= LEVEL3_JOINT_UNKNOWN_PUSH_MAX3) {
        joint_unknown_push_perm_rec3(&ctx, 0);
        if (ctx.solved_count > 0) {
            answer = (ctx.sum_cost + ctx.solved_count / 2) / ctx.solved_count;
            answer += ctx.unsolved_count * LEVEL3_JOINT_UNSOLVABLE_PENALTY3;
        }
    } else {
        /* Partial labels or larger sets use the existing N-1 completion
         * estimator to keep runtime bounded. */
        answer = estimate_push_cost_with_nminus1_c(start_p, bombs, num_bombs);
    }

    memcpy(dynamic_map, saved_map, sizeof(dynamic_map));
    memcpy(current_map, saved_current, sizeof(current_map));
    num_dyn_pairs = saved_num_pairs;
    memcpy(dyn_pairs, saved_pairs, sizeof(dyn_pairs));
    best_push_steps = saved_best_steps;
    memcpy(best_push_path, saved_best_path, sizeof(best_push_path));
    return answer;
}

typedef struct {
    int front_cost;
    int push_cost;
    int total_cost;
    uint32_t route_mask;
    uint32_t remaining_mask;
    Pos3 end_p;
    char end_dir;
} JointTerminalCandidate3;

static void joint_keep_terminal_candidate3(
    JointTerminalCandidate3 *list, int *count,
    const JointTerminalCandidate3 *cand)
{
    /* For the same remaining set only the cheaper recognition prefix matters. */
    for (int i = 0; i < *count; i++) {
        if (list[i].remaining_mask == cand->remaining_mask) {
            if (cand->front_cost < list[i].front_cost) list[i] = *cand;
            return;
        }
    }

    if (*count < LEVEL3_JOINT_TERMINAL_CANDIDATES3) {
        list[(*count)++] = *cand;
    } else {
        int worst = 0;
        for (int i = 1; i < *count; i++) {
            if (list[i].front_cost > list[worst].front_cost) worst = i;
        }
        if (cand->front_cost < list[worst].front_cost) list[worst] = *cand;
    }
}

static bool joint_append_tail_and_targets3(
    uint32_t remaining_mask, Pos3 start_p, char start_dir)
{
    if (remaining_mask == 0) {
        explore_end_pos = start_p;
        explore_end_dir = start_dir;
        return true;
    }

    global_best_cost = 999999;
    global_best_len = 0;
    char tmp_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    dfs_tsp(13, start_p, start_dir, remaining_mask, 0, tmp_path, 0);
    if (global_best_cost >= 999999 || global_best_len <= 0) return false;

    Pos3 p = start_p;
    char dir = start_dir;
    for (int i = 0; i < global_best_len; i++) {
        char cmd = global_best_path[i];
        bool snap = (cmd >= 'a' && cmd <= 'z');
        if (!joint_append_cmd3(cmd)) return false;
        joint_apply_pose_cmd3(&p, &dir, cmd);

        if (snap) {
            int tx = p.x, ty = p.y;
            if (dir == 'U') ty--;
            else if (dir == 'D') ty++;
            else if (dir == 'L') tx--;
            else if (dir == 'R') tx++;

            if (expected_cnt3 >= LEVEL3_RECOGNITION_CAPACITY3) return false;
            expected_targets3[expected_cnt3++] = (Pos3){tx, ty};
            remaining_recognition_cnt3++;
        }
    }

    explore_end_pos = p;
    explore_end_dir = dir;
    return true;
}

void plan_joint_bomb_explore_phase3(char external_map[12][16])
{
    /* Legacy competition version: use the already selected bomb sequence, but allow
     * recognition tasks to be inserted anywhere before, between, or after the
     * bomb tasks.  This is still a small macro-action search rather than a
     * per-grid-step global search, so RAM stays bounded. */
    level3_internal_reset3();
    if (level3_input_bomb_count3(external_map) > SOKOBAN_LEVEL3_BOMB_LIMIT) {
        level3_error3 = 1U;
        level3_error_code3 = 70;
        level3_last_error3 = 70;
        level3_bombs_used3 = 0U;
        return;
    }

    Pos3 start_player = {1, 5};
    Pos3 initial_bombs3[MAX_BOMBS3];
    int initial_bomb_count3 = 0;
    TspObj recognize_objects3[SOKOBAN_RECOGNITION_OBJECT_CAPACITY];
    int recognize_count3 = 0;

    memcpy(dynamic_map, external_map, sizeof(dynamic_map));
    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if (c == '@') {
                start_player = (Pos3){x, y};
                dynamic_map[y][x] = '-';
            } else if (c == '*') {
                if (initial_bomb_count3 < MAX_BOMBS3) {
                    initial_bombs3[initial_bomb_count3++] = (Pos3){x, y};
                }
                dynamic_map[y][x] = '-';
            } else if (c == '$' || (c >= 'A' && c <= 'Z')) {
                if (recognize_count3 < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int idx = recognize_count3;
                    recognize_objects3[idx] =
                        (TspObj){0, idx, (Pos3){x, y}, c, {0, 0}};
                    recognize_count3++;
                }
            } else if (c == '.' || (c >= 'a' && c <= 'z')) {
                if (recognize_count3 < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    int idx = recognize_count3;
                    recognize_objects3[idx] =
                        (TspObj){1, idx, (Pos3){x, y}, c, {0, 0}};
                    recognize_count3++;
                }
            }
        }
    }

    char initial_map3[12][16];
    memcpy(initial_map3, dynamic_map, sizeof(initial_map3));
    memcpy(bombs, initial_bombs3, sizeof(initial_bombs3));
    num_bombs = initial_bomb_count3;

    TspObj bomb_sequence3[BOMB_TASK_LIMIT];
    int bomb_sequence_count3 = 0;
    int bomb_sequence_cost3 = 0;

    if (initial_bomb_count3 > 0) {
        if (!level3_find_all_bombs_plan_c(
                start_player, bomb_sequence3,
                &bomb_sequence_count3, &bomb_sequence_cost3) ||
            bomb_sequence_count3 <= 0) {
            level3_error3 = 1;
            level3_error_code3 = 96;
            level3_steps3 = 0;
            level3_commands3[0] = '\0';
            return;
        }
    }

    int need_recognition3 =
        (recognize_count3 > 1) ? recognize_count3 - 1 : recognize_count3;
    int best_cost3 = 999999;
    int best_len3 = 0;
    char best_path3[LEVEL3_INTERNAL_PATH_CAPACITY3];

    /* Try every possible N-1 omitted object.  Bomb tasks keep their selected
     * order, while recognition tasks may be interleaved freely. */
    int drop_begin3 = (recognize_count3 > 1) ? 0 : -1;
    int drop_end3 = (recognize_count3 > 1) ? recognize_count3 : 0;

    for (int drop3 = drop_begin3; drop3 < drop_end3; drop3++) {
        memcpy(dynamic_map, initial_map3, sizeof(dynamic_map));

        /* Reorder the global bomb array to match the selected sequence.
         * dfs_tsp uses orig_idx both as sequence order and bomb-array index. */
        num_bombs = bomb_sequence_count3;
        for (int i = 0; i < bomb_sequence_count3; i++) {
            bombs[i] = bomb_sequence3[i].p;
        }

        num_objs = 0;
        for (int i = 0; i < recognize_count3; i++) {
            if (i == drop3) continue;
            if (num_objs >= 20) break;
            all_objs[num_objs] = recognize_objects3[i];
            all_objs[num_objs].orig_idx = num_objs;
            num_objs++;
        }
        for (int i = 0; i < bomb_sequence_count3; i++) {
            if (num_objs >= 20) break;
            all_objs[num_objs++] = (TspObj){
                2, i, bomb_sequence3[i].p, '*',
                bomb_sequence3[i].target_wall
            };
        }

        if (num_objs <= 0 || num_objs >= 31) continue;

        global_best_cost = 999999;
        global_best_len = 0;
        char current_path3[LEVEL3_INTERNAL_PATH_CAPACITY3];
        dfs_tsp(13, start_player, 'U',
                (1u << num_objs) - 1u, 0, current_path3, 0);

        if (global_best_cost < best_cost3 ||
            (global_best_cost == best_cost3 &&
             global_best_len < best_len3)) {
            best_cost3 = global_best_cost;
            best_len3 = global_best_len;
            if (best_len3 > 0) {
                memcpy(best_path3, global_best_path, (size_t)best_len3);
            }
        }
    }

    if (best_cost3 >= 999999 || best_len3 <= 0 ||
        best_len3 >= (int)sizeof(level3_commands3)) {
        level3_error3 = 1;
        level3_error_code3 = 97;
        level3_steps3 = 0;
        level3_commands3[0] = '\0';
        return;
    }

    int bomb_base_actual_steps3 = 0;
    if (bomb_sequence_count3 > 0) {
        memcpy(dynamic_map, initial_map3, sizeof(dynamic_map));
        memcpy(bombs, initial_bombs3, sizeof(initial_bombs3));
        num_bombs = initial_bomb_count3;
        Pos3 bomb_player3 = start_player;

        for (int i = 0; i < bomb_sequence_count3; i++) {
            int path_len3 = 0;
            char path3[LEVEL3_INTERNAL_PATH_CAPACITY3];
            char final_dir3 = 0;
            Pos3 end_player3 = bomb_player3;
            if (!bfs_push_specific_bomb(
                    bomb_player3, bomb_sequence3[i].p,
                    bomb_sequence3[i].target_wall,
                    &path_len3, path3, &final_dir3, &end_player3)) {
                bomb_base_actual_steps3 = bomb_sequence_cost3;
                break;
            }
            bomb_base_actual_steps3 += path_len3 + 1;
            bomb_player3 = end_player3;
            apply_blast_c(bomb_sequence3[i].target_wall);
            remove_bomb_at_c(bomb_sequence3[i].p);
        }
    }

    /* Build the final post-blast map.  The command path itself still contains
     * the real interleaved bomb operations and recognition snapshots. */
    memcpy(dynamic_map, initial_map3, sizeof(dynamic_map));
    for (int i = 0; i < bomb_sequence_count3; i++) {
        blast_on_map_c(dynamic_map, bomb_sequence3[i].target_wall);
    }
    num_bombs = 0;
    memcpy(external_map, dynamic_map, sizeof(dynamic_map));

    level3_steps3 = best_len3;
    memcpy(level3_commands3, best_path3, (size_t)best_len3);
    level3_commands3[best_len3] = '\0';

    expected_cnt3 = 0;
    Pos3 end_player3 = start_player;
    char end_dir3 = 'U';
    for (int i = 0; i < best_len3; i++) {
        char cmd3 = best_path3[i];
        bool snapshot3 =
            ((cmd3 >= 'a' && cmd3 <= 'z') || cmd3 == 'p');

        joint_apply_pose_cmd3(&end_player3, &end_dir3, cmd3);

        if (snapshot3) {
            int tx3 = end_player3.x;
            int ty3 = end_player3.y;
            if (end_dir3 == 'U') ty3--;
            else if (end_dir3 == 'D') ty3++;
            else if (end_dir3 == 'L') tx3--;
            else if (end_dir3 == 'R') tx3++;

            if (expected_cnt3 >= LEVEL3_RECOGNITION_CAPACITY3 ||
                tx3 < 0 || tx3 >= 16 || ty3 < 0 || ty3 >= 12) {
                level3_error3 = 1;
                level3_error_code3 = 98;
                level3_steps3 = 0;
                level3_commands3[0] = '\0';
                return;
            }
            expected_targets3[expected_cnt3++] = (Pos3){tx3, ty3};
        }
    }

    if (expected_cnt3 != need_recognition3) {
        level3_error3 = 1;
        level3_error_code3 = 99;
        level3_steps3 = 0;
        level3_commands3[0] = '\0';
        return;
    }

    explore_end_pos = end_player3;
    explore_end_dir = end_dir3;
    explore_steps3 = best_len3;
    bomb_steps3 = 0;
    bomb_base_steps3 = bomb_base_actual_steps3;
    route_recognition_cnt3 = expected_cnt3;
    remaining_recognition_cnt3 = 0;
    level3_joint_fallback3 = 0;
    level3_joint_candidate_count3 = recognize_count3;
    level3_joint_front_cost3 = best_cost3;
    level3_joint_terminal_push_estimate3 = 0;
    level3_segment_type3 = 4;
    level3_error3 = 0;
    level3_error_code3 = 0;
}



static bool level3_emit_one_bomb_c(TspObj task, char external_map[12][16])
{
    int path_len = 0;
    char path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    char final_push_dir = 0;
    Pos3 end_player = level3_staged_player;

    if (!bfs_push_specific_bomb(
            level3_staged_player, task.p, task.target_wall,
            &path_len, path, &final_push_dir, &end_player)) {
        level3_last_error3 = 31;
        return false;
    }

    if (path_len + 1 >= (int)sizeof(bomb_commands3)) {
        level3_last_error3 = 32;
        return false;
    }

    memcpy(bomb_commands3, path, path_len);
    bomb_steps3 = path_len;
    bomb_commands3[bomb_steps3++] = final_push_dir;
    bomb_commands3[bomb_steps3] = '\0';

    level3_staged_player = end_player;
    /* U/D/L/R are map-frame translations; they do not rotate the camera. */
    explore_end_pos = level3_staged_player;
    explore_end_dir = level3_staged_view_dir;

    apply_blast_c(task.target_wall);
    remove_bomb_at_c(task.p);
    level3_mirror_world_to_external_c(external_map);
    level3_last_error3 = 0;
    return true;
}

void level3_begin_staged3(char external_map[12][16])
{
    level3_internal_reset3();
    if (level3_input_bomb_count3(external_map) > SOKOBAN_LEVEL3_BOMB_LIMIT) {
        level3_staged_initialized = false;
        level3_error3 = 1U;
        level3_error_code3 = 70;
        level3_last_error3 = 70;
        level3_bombs_used3 = 0U;
        return;
    }
    memcpy(dynamic_map, external_map, sizeof(dynamic_map));

    level3_staged_initialized = true;
    level3_staged_push_emitted = false;
    level3_staged_player = (Pos3){1, 5};
    level3_staged_view_dir = 'U';
    level3_staged_recognized_mask = 0;
    level3_staged_pending_object = -1;
    level3_last_error3 = 0;
    num_bombs = 0;
    num_objs = 0;

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];

            if (c == '@') {
                level3_staged_player = (Pos3){x, y};
                dynamic_map[y][x] = '-';
            } else if (c == '*') {
                if (num_bombs < MAX_BOMBS3) bombs[num_bombs++] = (Pos3){x, y};
                dynamic_map[y][x] = '-';
            } else if (c == '$' || (c >= 'A' && c <= 'Z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    all_objs[num_objs] =
                        (TspObj){0, num_objs, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
                /* Never leak an unrecognised ground-truth letter internally. */
                dynamic_map[y][x] = '$';
            } else if (c == '.' || (c >= 'a' && c <= 'z')) {
                if (num_objs < SOKOBAN_RECOGNITION_OBJECT_CAPACITY) {
                    all_objs[num_objs] =
                        (TspObj){1, num_objs, (Pos3){x, y}, c, {0, 0}};
                    num_objs++;
                }
                dynamic_map[y][x] = '.';
            }
        }
    }

    explore_end_pos = level3_staged_player;
    explore_end_dir = level3_staged_view_dir;
    level3_reset_public_outputs_c();
    level3_mirror_world_to_external_c(external_map);
}

/*
 * Internal pre-push planner.
 * It generates exactly one executable segment, but does not generate the final
 * box-pushing path. This is the common core used by both public APIs.
 */

/* ========================================================================== */
/* First-tier joint planner: recognition and bomb actions are evaluated in a  */
/* single bounded Dijkstra search. The real label is still unknown, so the    */
/* search stops at the next observation and replans after the camera result.  */
/* Exact box-pushing cost is used once labels are sufficient; before that an  */
/* admissible label-compatible Manhattan assignment lower bound is used.      */
/* ========================================================================== */

static int level3_popcount32_c(uint32_t v)
{
    int n = 0;
    while (v) { n += (int)(v & 1u); v >>= 1; }
    return n;
}

static int level3_push_assignment_lb_c(void)
{
    Pos3 box_pos[MAX_BOXES3];
    Pos3 target_pos[MAX_BOXES3];
    char box_label[MAX_BOXES3];
    char target_label[MAX_BOXES3];
    int nb = 0, nt = 0;

    for (int y = 0; y < 12; y++) {
        for (int x = 0; x < 16; x++) {
            char c = dynamic_map[y][x];
            if ((c == '$' || (c >= 'A' && c <= 'Z')) && nb < MAX_BOXES3) {
                box_pos[nb] = (Pos3){x, y};
                box_label[nb++] = c;
            } else if ((c == '.' || (c >= 'a' && c <= 'z')) && nt < MAX_BOXES3) {
                target_pos[nt] = (Pos3){x, y};
                target_label[nt++] = c;
            }
        }
    }

    if (nb <= 0 || nb != nt || nb > MAX_BOXES3) return 0;

    int full = 1 << nt;
    int dp[1U << MAX_BOXES3];
    for (int i = 0; i < full; i++) dp[i] = 999999;
    dp[0] = 0;

    for (int mask = 0; mask < full; mask++) {
        if (dp[mask] >= 999999) continue;
        int bi = level3_popcount32_c((uint32_t)mask);
        if (bi >= nb) continue;

        for (int ti = 0; ti < nt; ti++) {
            if (mask & (1 << ti)) continue;

            char b = box_label[bi];
            char t = target_label[ti];
            if (b >= 'A' && b <= 'Z' && t >= 'a' && t <= 'z' &&
                t != (char)(b + ('a' - 'A'))) {
                continue;
            }

            int d = abs(box_pos[bi].x - target_pos[ti].x) +
                    abs(box_pos[bi].y - target_pos[ti].y);
            int nm = mask | (1 << ti);
            if (dp[mask] + d < dp[nm]) dp[nm] = dp[mask] + d;
        }
    }

    return (dp[full - 1] < 999999) ? dp[full - 1] : 0;
}

static int level3_future_observation_lb_c(
    Pos3 from, uint32_t recognized_after)
{
    int recognized = level3_popcount32_c(recognized_after);
    int still_needed = (num_objs > 0 ? num_objs - 1 : 0) - recognized;
    if (still_needed <= 0) return 0;

    int nearest = 999999;
    for (int i = 0; i < num_objs; i++) {
        if (recognized_after & (1u << i)) continue;
        int d = abs(from.x - all_objs[i].p.x) +
                abs(from.y - all_objs[i].p.y) - 1;
        if (d < 0) d = 0;
        if (d < nearest) nearest = d;
    }
    if (nearest >= 999999) nearest = 0;

    /* Every future observation needs at least one snapshot command. */
    return nearest + still_needed;
}

static void level3_simulate_pose_c(
    Pos3 start_p, char start_dir, const char *path, int len,
    Pos3 *out_p, char *out_dir)
{
    Pos3 p = start_p;
    char d = start_dir;

    for (int i = 0; i < len; i++) {
        char c = path[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));
        if (c == 'U') p.y--;
        else if (c == 'D') p.y++;
        else if (c == 'L') p.x--;
        else if (c == 'R') p.x++;
        else if (c == 'E') {
            if (d == 'U') d = 'R'; else if (d == 'R') d = 'D';
            else if (d == 'D') d = 'L'; else d = 'U';
        } else if (c == 'Q') {
            if (d == 'U') d = 'L'; else if (d == 'L') d = 'D';
            else if (d == 'D') d = 'R'; else d = 'U';
        } else if (c == 'B') {
            if (d == 'U') d = 'D'; else if (d == 'D') d = 'U';
            else if (d == 'L') d = 'R'; else d = 'L';
        }
    }

    *out_p = p;
    *out_dir = d;
}

static bool level3_one_observation_path_c(
    Pos3 player, char dir, int obj_idx,
    char *out_path, int *out_len, int *out_cost,
    Pos3 *out_end_p, char *out_end_dir)
{
    int saved_cost = global_best_cost;
    int saved_len = global_best_len;
    char saved_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    memcpy(saved_path, global_best_path, sizeof(saved_path));

    global_best_cost = 999999;
    global_best_len = 0;
    char tmp[LEVEL3_INTERNAL_PATH_CAPACITY3];
    dfs_tsp(13, player, dir, (1u << obj_idx), 0, tmp, 0);

    bool ok = (global_best_cost < 999999 && global_best_len > 0);
    if (ok) {
        memcpy(out_path, global_best_path, (size_t)global_best_len);
        *out_len = global_best_len;
        *out_cost = global_best_cost;
        level3_simulate_pose_c(
            player, dir, out_path, *out_len, out_end_p, out_end_dir);
    }

    global_best_cost = saved_cost;
    global_best_len = saved_len;
    memcpy(global_best_path, saved_path, sizeof(saved_path));
    return ok;
}

typedef struct {
    int score;
    int obj_idx;
    int obs_len;
    int obs_cost;
    char obs_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
    int task_cnt;
    TspObj tasks[BOMB_TASK_LIMIT];
} Level3GlobalChoiceC;

static int level3_alloc_global_node_c(void)
{
    for (int i = 0; i < BOMB_DIJKSTRA_MAX_NODES; i++) {
        if (!bomb_dij_nodes[i].used) return i;
    }
    level3_global_truncated3 = 1;
    return -1;
}

static bool level3_global_observe_or_bomb_c(
    char external_map[12][16], Level3PrepareResult3 *out_result)
{
    char base_map[12][16];
    Pos3 base_bombs[MAX_BOMBS3];
    int base_nb = num_bombs;
    memcpy(base_map, dynamic_map, sizeof(base_map));
    memcpy(base_bombs, bombs, sizeof(base_bombs));

    memset(bomb_dij_nodes, 0, sizeof(bomb_dij_nodes));
    level3_global_score3 = 999999;
    level3_global_nodes_expanded3 = 0;
    level3_global_candidates3 = 0;
    level3_global_bomb_prefix3 = 0;
    level3_global_truncated3 = 0;

    BombDijkstraNode *root = &bomb_dij_nodes[0];
    root->used = true;
    root->closed = false;
    root->cost = 0;
    root->player = level3_staged_player;
    root->nb = num_bombs;
    root->task_cnt = 0;
    memcpy(root->bombs_state, bombs, sizeof(bombs));
    memcpy(root->map_state, dynamic_map, sizeof(dynamic_map));
    root->hash = hash_map_state_c(root->map_state);

    Level3GlobalChoiceC best;
    memset(&best, 0, sizeof(best));
    best.score = 999999;
    best.obj_idx = -1;
    bool root_has_observation = false;

    while (1) {
        int idx = select_min_open_bomb_node_c();
        if (idx < 0) break;
        BombDijkstraNode *node = &bomb_dij_nodes[idx];
        node->closed = true;
        level3_global_nodes_expanded3++;

        if (node->cost >= best.score) continue;

        memcpy(dynamic_map, node->map_state, sizeof(dynamic_map));
        memcpy(bombs, node->bombs_state, sizeof(bombs));
        num_bombs = node->nb;

        /* Evaluate every next observation from this bomb-modified state. */
        for (int oi = 0; oi < num_objs; oi++) {
            if (level3_staged_recognized_mask & (1u << oi)) continue;

            char obs_path[LEVEL3_INTERNAL_PATH_CAPACITY3];
            int obs_len = 0, obs_cost = 0;
            Pos3 obs_end;
            char obs_dir;
            if (!level3_one_observation_path_c(
                    node->player, level3_staged_view_dir, oi,
                    obs_path, &obs_len, &obs_cost, &obs_end, &obs_dir)) {
                continue;
            }

            uint32_t after_mask =
                level3_staged_recognized_mask | (1u << oi);
            int remain_lb =
                level3_future_observation_lb_c(obs_end, after_mask);
            int push_lb = level3_push_assignment_lb_c();
            int total = node->cost + obs_cost + remain_lb + push_lb;
            level3_global_candidates3++;
            if (node->task_cnt == 0) root_has_observation = true;

            bool better = false;
            if (total < best.score) better = true;
            else if (total == best.score && node->task_cnt < best.task_cnt) better = true;
            else if (total == best.score && node->task_cnt == best.task_cnt &&
                     obs_cost < best.obs_cost) better = true;

            if (better) {
                best.score = total;
                best.obj_idx = oi;
                best.obs_len = obs_len;
                best.obs_cost = obs_cost;
                memcpy(best.obs_path, obs_path, (size_t)obs_len);
                best.task_cnt = node->task_cnt;
                if (node->task_cnt > 0) {
                    memcpy(best.tasks, node->tasks,
                           sizeof(TspObj) * (size_t)node->task_cnt);
                }
            }
        }

        /* Conservative first-tier rule: while at least one object can be
         * observed on the current map, do not consume an irreversible bomb
         * merely as a shortcut. Bomb Dijkstra is activated only when current
         * recognition is blocked. This keeps online replanning safe before
         * the unknown labels are known. */
        if (!LEVEL3_ALLOW_OPTIONAL_BOMB_SHORTCUT3 &&
            node->task_cnt == 0 && root_has_observation) continue;

        if (node->nb <= 0 || node->task_cnt >= BOMB_TASK_LIMIT) continue;

        /* Expand every physically reachable bomb-to-wall detonation. */
        for (int bi = 0; bi < node->nb; bi++) {
            Pos3 used_bomb = node->bombs_state[bi];
            get_all_reachable_walls_c(
                node->player, used_bomb, node->bombs_state, node->nb);

            int c_cnt = num_reach_walls;
            ReachableWall candidates[60];
            for (int k = 0; k < c_cnt; k++) candidates[k] = reach_walls[k];

            for (int ci = 0; ci < c_cnt; ci++) {
                int new_cost = node->cost + candidates[ci].path_len;
                if (new_cost >= best.score) continue;

                char child_map[12][16];
                Pos3 child_bombs[MAX_BOMBS3];
                int child_nb = 0;
                memcpy(child_map, node->map_state, sizeof(child_map));
                blast_on_map_c(child_map, candidates[ci].wall);
                remove_bomb_from_state_c(
                    node->bombs_state, node->nb, bi,
                    child_bombs, &child_nb);

                uint32_t child_hash = hash_map_state_c(child_map);
                int same = find_same_bomb_node_c(
                    child_map, child_hash, candidates[ci].end_p,
                    child_bombs, child_nb);

                int dst = same;
                if (dst >= 0) {
                    if (bomb_dij_nodes[dst].closed ||
                        new_cost >= bomb_dij_nodes[dst].cost) continue;
                } else {
                    dst = level3_alloc_global_node_c();
                    if (dst < 0) continue;
                }

                BombDijkstraNode *child = &bomb_dij_nodes[dst];
                child->used = true;
                child->closed = false;
                child->cost = new_cost;
                child->player = candidates[ci].end_p;
                child->nb = child_nb;
                child->task_cnt = node->task_cnt + 1;
                memcpy(child->bombs_state, child_bombs,
                       sizeof(child->bombs_state));
                memcpy(child->map_state, child_map,
                       sizeof(child->map_state));
                child->hash = child_hash;
                memcpy(child->tasks, node->tasks,
                       sizeof(TspObj) * (size_t)node->task_cnt);
                child->tasks[node->task_cnt] =
                    (TspObj){2, -1, used_bomb, '*', candidates[ci].wall};
            }
        }
    }

    memcpy(dynamic_map, base_map, sizeof(base_map));
    memcpy(bombs, base_bombs, sizeof(base_bombs));
    num_bombs = base_nb;

    if (best.obj_idx < 0) {
        level3_last_error3 = 142;
        *out_result = LEVEL3_PREPARE_ERROR3;
        return false;
    }

    level3_global_score3 = best.score;
    level3_global_bomb_prefix3 = best.task_cnt;

    if (best.task_cnt > 0) {
        if (!level3_emit_one_bomb_c(best.tasks[0], external_map)) {
            *out_result = LEVEL3_PREPARE_ERROR3;
            return false;
        }
        level3_segment_type3 = 2;
        *out_result = LEVEL3_PREPARE_BOMB3;
        return true;
    }

    if (best.obs_len >= (int)sizeof(explore_commands3)) {
        level3_last_error3 = 143;
        *out_result = LEVEL3_PREPARE_ERROR3;
        return false;
    }

    memcpy(explore_commands3, best.obs_path, (size_t)best.obs_len);
    explore_steps3 = best.obs_len;
    explore_commands3[explore_steps3] = '\0';
    expected_targets3[0] = all_objs[best.obj_idx].p;
    expected_cnt3 = 1;
    level3_staged_pending_object = best.obj_idx;
    level3_apply_pose_commands_c(best.obs_path, best.obs_len);
    level3_segment_type3 = 1;
    *out_result = LEVEL3_PREPARE_RECOGNIZE3;
    return true;
}

static Level3PrepareResult3 level3_prepare_next_core3(
    char external_map[12][16])
{
    level3_reset_public_outputs_c();

    if (!level3_staged_initialized) {
        level3_last_error3 = 1;
        return LEVEL3_PREPARE_ERROR3;
    }

    if (!level3_commit_pending_recognition_c(external_map)) {
        if (level3_last_error3 == 12) {
            if (level3_staged_pending_object >= 0 &&
                level3_staged_pending_object < num_objs) {
                expected_targets3[0] =
                    all_objs[level3_staged_pending_object].p;
                expected_cnt3 = 1;
            }
            return LEVEL3_PREPARE_WAIT_RECOGNITION3;
        }
        return LEVEL3_PREPARE_ERROR3;
    }

    if (!level3_try_complete_labels_c(external_map)) {
        Level3PrepareResult3 global_result3 = LEVEL3_PREPARE_ERROR3;
        if (!level3_global_observe_or_bomb_c(
                external_map, &global_result3)) {
            return LEVEL3_PREPARE_ERROR3;
        }
        return global_result3;
    }

    /* All labels are known or safely inferred. Bomb only when required. */
    explore_end_pos = level3_staged_player;
    explore_end_dir = level3_staged_view_dir;

    if (!map_ready_for_push_c(level3_staged_player)) {
        if (num_bombs <= 0) {
            level3_last_error3 = 51;
            return LEVEL3_PREPARE_ERROR3;
        }

        TspObj bomb_seq[BOMB_TASK_LIMIT];
        int bomb_cnt = 0;
        int bomb_cost = 999999;
        char real_map_backup[12][16];
        memcpy(real_map_backup, dynamic_map, sizeof(real_map_backup));

        bool ok = dijkstra_bomb_plan_c(
            level3_staged_player, bomb_seq, &bomb_cnt, &bomb_cost);
        memcpy(dynamic_map, real_map_backup, sizeof(real_map_backup));

        if (!ok || bomb_cnt <= 0) {
            level3_last_error3 = 52;
            return LEVEL3_PREPARE_ERROR3;
        }

        if (!level3_emit_one_bomb_c(bomb_seq[0], external_map)) {
            return LEVEL3_PREPARE_ERROR3;
        }
        level3_segment_type3 = 2;
        return LEVEL3_PREPARE_BOMB3;
    }

    level3_mirror_world_to_external_c(external_map);
    explore_end_pos = level3_staged_player;
    explore_end_dir = level3_staged_view_dir;
    level3_last_error3 = 0;
    level3_segment_type3 = 3;
    return LEVEL3_PREPARE_READY_PUSH3;
}

/* Copy the current recognition/bomb segment to the unified prepare output. */
static bool level3_copy_prepare_output3(Level3PrepareResult3 result)
{
    const char *src = NULL;
    int len = 0;

    if (result == LEVEL3_PREPARE_RECOGNIZE3) {
        src = explore_commands3;
        len = explore_steps3;
    } else if (result == LEVEL3_PREPARE_BOMB3) {
        src = bomb_commands3;
        len = bomb_steps3;
    } else {
        prepare_steps3 = 0;
        prepare_commands3[0] = '\0';
        return true;
    }

    if (len < 0 || len >= (int)sizeof(prepare_commands3)) {
        level3_last_error3 = 71;
        return false;
    }

    /* All stage buffers alias level3_commands3 in the low-memory build. */
    if (src != prepare_commands3 && len > 0) {
        memmove(prepare_commands3, src, (size_t)len);
    }
    prepare_steps3 = len;
    prepare_commands3[prepare_steps3] = '\0';
    return true;
}

/*
 * Simplified API, close to level 2 usage.
 *
 * The first call initializes the staged planner automatically. Then call this
 * function again only after the previously returned segment has physically
 * finished. For RECOGNIZE, write the camera result into external_map first.
 *
 * When READY_PUSH is returned, call plan_push_phase3() once.
 */
Level3PrepareResult3 plan_prepare_phase3(char external_map[12][16])
{
    if (!level3_staged_initialized) {
        level3_begin_staged3(external_map);
    }

    Level3PrepareResult3 result =
        level3_prepare_next_core3(external_map);

    if (!level3_copy_prepare_output3(result)) {
        return LEVEL3_PREPARE_ERROR3;
    }

    return result;
}

/*
 * Backward-compatible staged API.
 * Existing state machines may keep using level3_begin_staged3() and
 * level3_plan_next3(). New code should prefer plan_prepare_phase3().
 */
Level3Action3 level3_plan_next3(char external_map[12][16])
{
    if (!level3_staged_initialized) {
        level3_last_error3 = 1;
        return LEVEL3_ACTION_ERROR3;
    }

    if (level3_staged_push_emitted) {
        return LEVEL3_ACTION_DONE3;
    }

    Level3PrepareResult3 prepare_result =
        level3_prepare_next_core3(external_map);

    switch (prepare_result) {
        case LEVEL3_PREPARE_RECOGNIZE3:
            return LEVEL3_ACTION_EXPLORE3;

        case LEVEL3_PREPARE_BOMB3:
            return LEVEL3_ACTION_BOMB3;

        case LEVEL3_PREPARE_WAIT_RECOGNITION3:
            return LEVEL3_ACTION_WAIT_RECOGNITION3;

        case LEVEL3_PREPARE_READY_PUSH3:
            plan_push_phase3(external_map);

            if (push_steps3 <= 0 && get_total_box_count() > 0) {
                level3_last_error3 = 61;
                return LEVEL3_ACTION_ERROR3;
            }

            level3_staged_push_emitted = true;
            level3_last_error3 = 0;
            return LEVEL3_ACTION_PUSH3;

        case LEVEL3_PREPARE_ERROR3:
        default:
            return LEVEL3_ACTION_ERROR3;
    }
}

static void level3_internal_reset3(void)
{
    level3_bomb_first_initialized3 = false;
    memset(bfs_parent_code3, 0xFF, sizeof(bfs_parent_code3));
    bfs_parent_initialized3 = true;
    bfs_used_count = 0;
    memset(dynamic_map, 0, sizeof(dynamic_map));
    memset(current_map, 0, sizeof(current_map));

    level3_staged_initialized = false;
    level3_staged_push_emitted = false;
    level3_staged_player = (Pos3){1, 5};
    level3_staged_view_dir = 'U';
    level3_staged_recognized_mask = 0;
    level3_staged_pending_object = -1;
    level3_last_error3 = 0;
    level3_global_score3 = 999999;
    level3_global_nodes_expanded3 = 0;
    level3_global_candidates3 = 0;
    level3_global_bomb_prefix3 = 0;
    level3_global_truncated3 = 0;
    level3_segment_type3 = 0;

    explore_steps3 = 0;
    memset(explore_commands3, 0, sizeof(explore_commands3));
    memset(expected_targets3, 0, sizeof(expected_targets3));
    expected_cnt3 = 0;

    bomb_steps3 = 0;
    level3_bombs_used3 = 0U;
    memset(bomb_commands3, 0, sizeof(bomb_commands3));

    push_steps3 = 0;
    memset(push_commands3, 0, sizeof(push_commands3));

    prepare_steps3 = 0;
    memset(prepare_commands3, 0, sizeof(prepare_commands3));

    level3_easy_state3 = LEVEL3_EASY_IDLE3;
    level3_easy_steps3 = 0;
    memset(level3_easy_commands3, 0, sizeof(level3_easy_commands3));
    level3_easy_target3 = (Pos3){-1, -1};
    level3_easy_error3 = 0;

    level3_steps3 = 0;
    memset(level3_commands3, 0, sizeof(level3_commands3));
    level3_need_vision3 = 0;
    level3_finished3 = 0;
    level3_error3 = 0;
    level3_error_code3 = 0;
    level3_vision_pos3 = (Pos3){-1, -1};


    explore_end_pos = (Pos3){0, 0};
    explore_end_dir = 'U';

    memset(bombs, 0, sizeof(bombs));
    num_bombs = 0;
    solver_explore_bomb_insert_count = 0;
    solver_push_shortcut_bomb_count = 0;

    memset(dyn_pairs, 0, sizeof(dyn_pairs));
    num_dyn_pairs = 0;
    best_push_steps = 999999;
    memset(best_push_path, 0, sizeof(best_push_path));

    memset(all_objs, 0, sizeof(all_objs));
    num_objs = 0;
    global_best_cost = 999999;
    memset(global_best_path, 0, sizeof(global_best_path));
    global_best_len = 0;

    memset(blocking_walls, 0, sizeof(blocking_walls));
    num_blocking_walls = 0;
    memset(reach_walls, 0, sizeof(reach_walls));
    num_reach_walls = 0;

    best_bomb_plan_cost = 999999;
    memset(best_bomb_plan_seq, 0, sizeof(best_bomb_plan_seq));
    best_bomb_plan_cnt = 0;

    memset(dup_group_start, 0xFF, sizeof(dup_group_start));
    memset(dup_group_count, 0, sizeof(dup_group_count));
    duplicate_assignment_eval_count = 0;
    solver_duplicate_element_group_count = 0;

    memset(bomb_dij_nodes, 0, sizeof(bomb_dij_nodes));
}


// ============================================================================
// Highly integrated public wrapper
// ============================================================================

static void level3_easy_clear_command3(void)
{
    level3_easy_steps3 = 0;
    level3_easy_commands3[0] = '\0';
}

static Level3EasyState3 level3_easy_fail3(int error3)
{
    level3_easy_clear_command3();
    level3_easy_target3 = (Pos3){-1, -1};
    level3_easy_error3 = (error3 != 0) ? error3 : level3_last_error3;
    level3_easy_state3 = LEVEL3_EASY_ERROR3;
    return level3_easy_state3;
}

static bool level3_easy_copy_commands3(const char *src3, int len3)
{
    if (src3 == NULL || len3 < 0 ||
        len3 >= (int)sizeof(level3_easy_commands3)) {
        return false;
    }

    if (src3 != level3_easy_commands3 && len3 > 0) {
        memmove(level3_easy_commands3, src3, (size_t)len3);
    }
    level3_easy_steps3 = len3;
    level3_easy_commands3[len3] = '\0';
    return true;
}

static Level3EasyState3 level3_easy_plan_next_internal3(
    char map3[12][16])
{
    Level3PrepareResult3 result3 = plan_prepare_phase3(map3);

    switch (result3) {
        case LEVEL3_PREPARE_RECOGNIZE3:
            if (expected_cnt3 <= 0 ||
                !level3_easy_copy_commands3(
                    prepare_commands3, prepare_steps3)) {
                return level3_easy_fail3(81);
            }
            level3_easy_target3 = expected_targets3[0];
            level3_easy_error3 = 0;
            level3_easy_state3 = LEVEL3_EASY_RECOGNIZE_PATH3;
            return level3_easy_state3;

        case LEVEL3_PREPARE_WAIT_RECOGNITION3:
            level3_easy_clear_command3();
            if (expected_cnt3 > 0) {
                level3_easy_target3 = expected_targets3[0];
            }
            level3_easy_error3 = 0;
            level3_easy_state3 = LEVEL3_EASY_WAIT_VISION3;
            return level3_easy_state3;

        case LEVEL3_PREPARE_BOMB3:
            if (!level3_easy_copy_commands3(
                    prepare_commands3, prepare_steps3)) {
                return level3_easy_fail3(82);
            }
            level3_easy_target3 = (Pos3){-1, -1};
            level3_easy_error3 = 0;
            level3_easy_state3 = LEVEL3_EASY_BOMB_PATH3;
            return level3_easy_state3;

        case LEVEL3_PREPARE_READY_PUSH3:
            plan_push_phase3(map3);

            if (push_steps3 <= 0) {
                if (get_total_box_count() > 0) {
                    return level3_easy_fail3(
                        level3_last_error3 ? level3_last_error3 : 83);
                }

                level3_easy_state3 = LEVEL3_EASY_DONE3;
                return level3_easy_state3;
            }

            if (!level3_easy_copy_commands3(
                    push_commands3, push_steps3)) {
                return level3_easy_fail3(84);
            }

            level3_easy_target3 = (Pos3){-1, -1};
            level3_easy_error3 = 0;
            level3_easy_state3 = LEVEL3_EASY_PUSH_PATH3;
            return level3_easy_state3;

        case LEVEL3_PREPARE_ERROR3:
        default:
            return level3_easy_fail3(level3_last_error3);
    }
}

Level3EasyState3 level3_easy_start3(char map3[12][16])
{
    if (map3 == NULL) {
        return level3_easy_fail3(85);
    }

    level3_internal_reset3();
    level3_easy_state3 = LEVEL3_EASY_IDLE3;
    level3_easy_error3 = 0;
    level3_easy_target3 = (Pos3){-1, -1};
    return level3_easy_plan_next_internal3(map3);
}

Level3EasyState3 level3_easy_path_finished3(char map3[12][16])
{
    if (map3 == NULL) {
        return level3_easy_fail3(86);
    }

    switch (level3_easy_state3) {
        case LEVEL3_EASY_RECOGNIZE_PATH3:
            /* The car is now at the recognition pose. Wait for the camera. */
            level3_easy_clear_command3();
            level3_easy_state3 = LEVEL3_EASY_WAIT_VISION3;
            return level3_easy_state3;

        case LEVEL3_EASY_BOMB_PATH3:
            /* The bomb segment physically finished; plan the next segment. */
            level3_easy_clear_command3();
            return level3_easy_plan_next_internal3(map3);

        case LEVEL3_EASY_PUSH_PATH3:
            /* Standalone test: keep diagnostics/state after completion. */
            level3_easy_clear_command3();
            level3_easy_state3 = LEVEL3_EASY_DONE3;
            level3_easy_error3 = 0;
            return level3_easy_state3;

        case LEVEL3_EASY_WAIT_VISION3:
            return level3_easy_state3;

        case LEVEL3_EASY_DONE3:
        case LEVEL3_EASY_ERROR3:
            return level3_easy_state3;

        case LEVEL3_EASY_IDLE3:
        default:
            return level3_easy_fail3(87);
    }
}

Level3EasyState3 level3_easy_vision_finished3(
    char map3[12][16], char variety3)
{
    int object_type3;

    if (map3 == NULL) {
        return level3_easy_fail3(88);
    }

    /* Calling directly after the recognition path is also supported. */
    if (level3_easy_state3 != LEVEL3_EASY_WAIT_VISION3 &&
        level3_easy_state3 != LEVEL3_EASY_RECOGNIZE_PATH3) {
        return level3_easy_fail3(89);
    }

    if (level3_staged_pending_object < 0 ||
        level3_staged_pending_object >= num_objs) {
        return level3_easy_fail3(90);
    }

    level3_easy_target3 =
        all_objs[level3_staged_pending_object].p;
    object_type3 = all_objs[level3_staged_pending_object].type;

    if (level3_easy_target3.x < 0 || level3_easy_target3.x >= 16 ||
        level3_easy_target3.y < 0 || level3_easy_target3.y >= 12) {
        return level3_easy_fail3(91);
    }

    /* Box labels must be uppercase; target labels must be lowercase. */
    if ((object_type3 == 0 &&
         !(variety3 >= 'A' && variety3 <= 'Z')) ||
        (object_type3 == 1 &&
         !(variety3 >= 'a' && variety3 <= 'z'))) {
        return level3_easy_fail3(92);
    }

    map3[level3_easy_target3.y][level3_easy_target3.x] = variety3;
    level3_easy_clear_command3();
    return level3_easy_plan_next_internal3(map3);
}

void level3_easy_reset3(void)
{
    level3_internal_reset3();
}

// ============================================================================
// Minimal integrated public API
// ============================================================================

static void level3_clear_public_output3(void)
{
    level3_steps3 = 0;
    level3_commands3[0] = '\0';
    level3_need_vision3 = 0;
    level3_finished3 = 0;
    level3_error3 = 0;
    level3_error_code3 = 0;
    level3_vision_pos3 = (Pos3){-1, -1};
}

static bool level3_copy_public_commands3(const char *src3, int len3)
{
    if (src3 == NULL || len3 < 0 ||
        len3 >= (int)sizeof(level3_commands3)) {
        return false;
    }

    if (src3 != level3_commands3 && len3 > 0) {
        memmove(level3_commands3, src3, (size_t)len3);
    }
    level3_steps3 = len3;
    level3_commands3[len3] = '\0';
    return true;
}

static void level3_publish_easy_state3(void)
{
    level3_steps3 = 0;
    level3_need_vision3 = 0;
    level3_finished3 = 0;
    level3_error3 = 0;
    level3_error_code3 = 0;
    level3_vision_pos3 = (Pos3){-1, -1};

    switch (level3_easy_state3) {
        case LEVEL3_EASY_RECOGNIZE_PATH3:
            if (!level3_copy_public_commands3(
                    level3_easy_commands3,
                    level3_easy_steps3)) {
                level3_error3 = 1;
                level3_error_code3 = 101;
                return;
            }
            level3_need_vision3 = 1;
            level3_vision_pos3 = level3_easy_target3;
            break;

        case LEVEL3_EASY_WAIT_VISION3:
            level3_need_vision3 = 1;
            level3_vision_pos3 = level3_easy_target3;
            break;

        case LEVEL3_EASY_BOMB_PATH3:
        case LEVEL3_EASY_PUSH_PATH3:
            if (!level3_copy_public_commands3(
                    level3_easy_commands3,
                    level3_easy_steps3)) {
                level3_error3 = 1;
                level3_error_code3 = 102;
            }
            break;

        case LEVEL3_EASY_DONE3:
            level3_segment_type3 = 0;
            level3_finished3 = 1;
            break;

        case LEVEL3_EASY_ERROR3:
            level3_segment_type3 = 0;
            level3_error3 = 1;
            level3_error_code3 = level3_easy_error3;
            break;

        case LEVEL3_EASY_IDLE3:
        default:
            break;
    }
}

void level3_start3(char map3[12][16])
{
    level3_clear_public_output3();
    (void)level3_easy_start3(map3);
    level3_publish_easy_state3();
}

void level3_path_finished3(char map3[12][16])
{
    if (level3_finished3 || level3_error3) {
        return;
    }

    (void)level3_easy_path_finished3(map3);
    level3_publish_easy_state3();
}

void level3_vision_finished3(char map3[12][16], char variety3)
{
    if (level3_finished3 || level3_error3) {
        return;
    }

    (void)level3_easy_vision_finished3(map3, variety3);
    level3_publish_easy_state3();
}

void level3_reset3(void)
{
    level3_easy_reset3();
    level3_clear_public_output3();
}

