#include "sokoban_map1.h"
#include "sokoban_solver1.h"
#include "sokoban_workspace.h"
#include "zf_common_headfile.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define L1_W 16
#define L1_H 12
#define L1_CELLS (L1_W * L1_H)
#define L1_REMOVED 0xFFu
#define L1_NO_NODE 0xFFFFu
#define L1_HASH_SIZE SOKOBAN_LEVEL1_PUSH_HASH_SIZE
#define L1_MAX_NODES SOKOBAN_LEVEL1_PUSH_MAX_NODES
#define L1_PATH_LIMIT SOKOBAN_LEVEL1_PUSH_PATH_CAPACITY

#define l1_nodes        (g_sokoban_workspace.level1.push_nodes)
#define l1_hash         (g_sokoban_workspace.level1.push_hash)
#define l1_heap         (g_sokoban_workspace.level1.push_heap)
#define l1_chain        (g_sokoban_workspace.level1.solution_chain)
#define l1_wall         (g_sokoban_workspace.level1.wall)
#define l1_goal_cells   (g_sokoban_workspace.level1.goal_cells)
#define l1_goal_index   (g_sokoban_workspace.level1.goal_index_by_cell)
#define l1_goal_dist    (g_sokoban_workspace.level1.goal_push_dist)
#define l1_walk_dist    (g_sokoban_workspace.level1.walk_dist)
#define l1_walk_parent  (g_sokoban_workspace.level1.walk_parent)
#define l1_walk_move    (g_sokoban_workspace.level1.walk_move)
#define l1_goal_reach   (g_sokoban_workspace.level1.goal_reach_mask)
#define l1_walk_queue   (g_sokoban_workspace.level1.walk_queue)
#define l1_occupied     (g_sokoban_workspace.level1.occupied)
#define l1_work_path    (g_sokoban_workspace.level1.work_commands)
#define l1_reverse_path (g_sokoban_workspace.level1.reverse_path)

typedef SokobanPushNode2 L1Node;

static const int8_t l1_dx[4] = {0, 0, -1, 1};
static const int8_t l1_dy[4] = {-1, 1, 0, 0};
static const char l1_cmd[4] = {'U', 'D', 'L', 'R'};
/* Weight 4 solved the shipped stress map only after weights 1 and 2 had each
 * exhausted the node pool.  Try the proven fast ordering first; the remaining
 * entries are completeness fallbacks for maps with a different topology. */
static const uint8_t l1_weights[] = {4, 6, 2, 1};

static uint16_t l1_node_count;
static uint16_t l1_heap_count;
static uint8_t l1_box_count;
static uint8_t l1_goal_count;
static SokobanBoxMask l1_full_goal_mask;
static bool l1_path_capacity_hit;
static bool l1_search_capacity_hit;
static uint16_t l1_floor_rows[L1_H];

int final_steps1 = 0;
char final_commands1[SOKOBAN_LEVEL1_OUTPUT_CAPACITY];
int sokoban_error1 = SOKOBAN_OK1;
uint32_t search_nodes1 = 0;

/* Keil Watch diagnostics. */
volatile uint32_t g_l1_astar_attempts1 = 0U;
volatile uint32_t g_l1_astar_nodes1 = 0U;
volatile uint32_t g_l1_astar_pushes1 = 0U;
volatile uint32_t g_l1_astar_reopens1 = 0U;
volatile int32_t g_l1_astar_weight1 = -1;
volatile int32_t g_l1_astar_result1 = 0;
volatile uint32_t expanded_nodes = 0U;
volatile uint32_t generated_nodes = 0U;
volatile uint32_t duplicate_nodes = 0U;
volatile uint32_t deadlock_pruned = 0U;
volatile uint32_t max_open = 0U;
volatile uint32_t final_path_len = 0U;
volatile uint32_t g_l1_hash_lookups1 = 0U;
volatile uint32_t g_l1_hash_probes1 = 0U;
volatile uint32_t g_l1_hash_collisions1 = 0U;
volatile uint32_t g_l1_hash_max_probe1 = 0U;
volatile uint32_t g_l1_node_pool_hits1 = 0U;
volatile uint32_t g_l1_hash_full_hits1 = 0U;

static uint8_t l1_cell(int x, int y)
{
    return (uint8_t)(y * L1_W + x);
}

static int l1_x(uint8_t c) { return c % L1_W; }
static int l1_y(uint8_t c) { return c / L1_W; }

static bool l1_inside(int x, int y)
{
    return x >= 0 && x < L1_W && y >= 0 && y < L1_H;
}

static void l1_sort_boxes(uint8_t boxes[MAX_BOXES1])
{
    for (int i = 0; i < l1_box_count - 1; i++) {
        for (int j = i + 1; j < l1_box_count; j++) {
            if (boxes[j] < boxes[i]) {
                uint8_t t = boxes[i];
                boxes[i] = boxes[j];
                boxes[j] = t;
            }
        }
    }
}

static uint32_t l1_state_hash(uint8_t region,
                              const uint8_t boxes[MAX_BOXES1],
                              SokobanBoxMask used_goal_mask)
{
    uint32_t h = 2166136261u;
    h = (h ^ region) * 16777619u;
    h = (h ^ (uint8_t)used_goal_mask) * 16777619u;
    h = (h ^ (uint8_t)(used_goal_mask >> 8)) * 16777619u;
    for (int i = 0; i < l1_box_count; i++) {
        h = (h ^ boxes[i]) * 16777619u;
    }
    return h;
}

static bool l1_same_state(uint16_t idx, uint8_t region,
                          const uint8_t boxes[MAX_BOXES1],
                          SokobanBoxMask used_goal_mask)
{
    const L1Node *n = &l1_nodes[idx];
    return n->region == region && n->active_box == used_goal_mask &&
           memcmp(n->boxes, boxes, l1_box_count) == 0;
}

static uint16_t l1_hash_lookup(uint8_t region,
                               const uint8_t boxes[MAX_BOXES1],
                               SokobanBoxMask used_goal_mask,
                               uint16_t *out_slot)
{
    uint32_t slot = l1_state_hash(region, boxes, used_goal_mask) &
                    (L1_HASH_SIZE - 1u);
    g_l1_hash_lookups1++;
    for (uint32_t probe = 0; probe < L1_HASH_SIZE; probe++) {
        uint32_t probes_used = probe + 1U;
        g_l1_hash_probes1++;
        if (probes_used > g_l1_hash_max_probe1)
            g_l1_hash_max_probe1 = probes_used;
        uint16_t entry = l1_hash[slot];
        if (entry == 0U) {
            *out_slot = (uint16_t)slot;
            return L1_NO_NODE;
        }
        uint16_t idx = (uint16_t)(entry - 1u);
        if (l1_same_state(idx, region, boxes, used_goal_mask)) {
            *out_slot = (uint16_t)slot;
            return idx;
        }
        g_l1_hash_collisions1++;
        slot = (slot + 1u) & (L1_HASH_SIZE - 1u);
    }
    g_l1_hash_full_hits1++;
    l1_search_capacity_hit = true;
    *out_slot = L1_NO_NODE;
    return L1_NO_NODE;
}

static bool l1_heap_better(uint16_t a, uint16_t b)
{
    if (l1_nodes[a].f_cost != l1_nodes[b].f_cost)
        return l1_nodes[a].f_cost < l1_nodes[b].f_cost;
    return l1_nodes[a].g_cost > l1_nodes[b].g_cost;
}

static void l1_heap_swap(uint16_t a, uint16_t b)
{
    uint16_t ia = l1_heap[a], ib = l1_heap[b];
    l1_heap[a] = ib;
    l1_heap[b] = ia;
    l1_nodes[ia].heap_pos = b;
    l1_nodes[ib].heap_pos = a;
}

static void l1_heap_up(uint16_t pos)
{
    while (pos > 0U) {
        uint16_t parent = (uint16_t)((pos - 1U) / 2U);
        if (!l1_heap_better(l1_heap[pos], l1_heap[parent])) break;
        l1_heap_swap(pos, parent);
        pos = parent;
    }
}

static void l1_heap_down(uint16_t pos)
{
    for (;;) {
        uint16_t left = (uint16_t)(pos * 2U + 1U);
        if (left >= l1_heap_count) break;
        uint16_t best = left;
        uint16_t right = (uint16_t)(left + 1U);
        if (right < l1_heap_count &&
            l1_heap_better(l1_heap[right], l1_heap[left])) best = right;
        if (!l1_heap_better(l1_heap[best], l1_heap[pos])) break;
        l1_heap_swap(pos, best);
        pos = best;
    }
}

static void l1_heap_insert(uint16_t idx)
{
    if (l1_heap_count >= L1_MAX_NODES) return;
    l1_heap[l1_heap_count] = idx;
    l1_nodes[idx].heap_pos = l1_heap_count;
    l1_heap_up(l1_heap_count++);
    if ((uint32_t)l1_heap_count > max_open) max_open = l1_heap_count;
}

static uint16_t l1_heap_pop(void)
{
    if (l1_heap_count == 0U) return L1_NO_NODE;
    uint16_t result = l1_heap[0];
    l1_nodes[result].heap_pos = L1_NO_NODE;
    l1_heap_count--;
    if (l1_heap_count > 0U) {
        l1_heap[0] = l1_heap[l1_heap_count];
        l1_nodes[l1_heap[0]].heap_pos = 0U;
        l1_heap_down(0U);
    }
    return result;
}

static void l1_build_reachability(uint8_t start,
                                  const uint8_t boxes[MAX_BOXES1])
{
    memset(l1_walk_dist, 0xFF, L1_CELLS);
    memset(l1_occupied, 0, L1_CELLS);
    for (int i = 0; i < l1_box_count; i++) {
        if (boxes[i] != L1_REMOVED) l1_occupied[boxes[i]] = 1U;
    }

    int head = 0, tail = 0;
    l1_walk_dist[start] = 0U;
    l1_walk_queue[tail++] = start;
    while (head < tail) {
        uint8_t curr = l1_walk_queue[head++];
        int x = l1_x(curr), y = l1_y(curr);
        for (int d = 0; d < 4; d++) {
            int nx = x + l1_dx[d], ny = y + l1_dy[d];
            if (!l1_inside(nx, ny)) continue;
            uint8_t next = l1_cell(nx, ny);
            if (l1_wall[next] || l1_occupied[next]) continue;
            if (l1_walk_dist[next] != 0xFFU) continue;
            l1_walk_dist[next] = (uint8_t)(l1_walk_dist[curr] + 1U);
            l1_walk_queue[tail++] = next;
        }
    }
}

/* Canonicalize every exact player cell in the same reachable component to
 * the component's smallest row-major cell.  A compact row flood is cheaper
 * than another 192-entry queue and needs only 48 bytes of bounded stack. */
static uint8_t l1_region_anchor(uint8_t start,
                                const uint8_t boxes[MAX_BOXES1])
{
    uint16_t free_rows[L1_H];
    uint16_t reach[L1_H];
    bool changed;

    for (int y = 0; y < L1_H; y++) {
        free_rows[y] = l1_floor_rows[y];
        reach[y] = 0U;
    }
    for (int i = 0; i < l1_box_count; i++) {
        uint8_t box = boxes[i];
        if (box != L1_REMOVED)
            free_rows[l1_y(box)] &= (uint16_t)~(1U << l1_x(box));
    }
    reach[l1_y(start)] = (uint16_t)(1U << l1_x(start));

    do {
        changed = false;
        for (int y = 0; y < L1_H; y++) {
            uint16_t old = reach[y];
            uint16_t next = old;
            next |= (uint16_t)(old << 1);
            next |= (uint16_t)(old >> 1);
            if (y > 0) next |= reach[y - 1];
            if (y + 1 < L1_H) next |= reach[y + 1];
            next &= free_rows[y];
            if (next != old) {
                reach[y] = next;
                changed = true;
            }
        }
    } while (changed);

    for (int y = 0; y < L1_H; y++) {
        if (reach[y] != 0U) {
            for (int x = 0; x < L1_W; x++) {
                if ((reach[y] & (1U << x)) != 0U) return l1_cell(x, y);
            }
        }
    }
    return start;
}

static bool l1_box_at(uint8_t cell, const uint8_t boxes[MAX_BOXES1])
{
    for (int i = 0; i < l1_box_count; i++)
        if (boxes[i] == cell) return true;
    return false;
}

/* A wall/box-filled 2x2 block containing the newly moved box can never be
 * separated: no box in the block has a free support square.  Goals consume
 * boxes immediately in level 1, so every retained box in such a block is
 * necessarily unfinished. */
static bool l1_has_2x2_deadlock(uint8_t moved_box,
                                const uint8_t boxes[MAX_BOXES1])
{
    int bx = l1_x(moved_box), by = l1_y(moved_box);
    for (int oy = -1; oy <= 0; oy++) {
        for (int ox = -1; ox <= 0; ox++) {
            int x0 = bx + ox, y0 = by + oy;
            bool solid = true;
            if (x0 < 0 || x0 + 1 >= L1_W ||
                y0 < 0 || y0 + 1 >= L1_H) continue;
            for (int y = y0; y <= y0 + 1 && solid; y++) {
                for (int x = x0; x <= x0 + 1; x++) {
                    uint8_t cell = l1_cell(x, y);
                    if (!l1_wall[cell] && !l1_box_at(cell, boxes)) {
                        solid = false;
                        break;
                    }
                }
            }
            if (solid) return true;
        }
    }
    return false;
}

static bool l1_build_goal_distances(void)
{
    for (int g = 0; g < l1_goal_count; g++) {
        memset(l1_goal_dist[g], 0xFF, L1_CELLS);
        int head = 0, tail = 0;
        uint8_t goal = l1_goal_cells[g];
        l1_goal_dist[g][goal] = 0U;
        l1_walk_queue[tail++] = goal;

        while (head < tail) {
            uint8_t curr = l1_walk_queue[head++];
            int cx = l1_x(curr), cy = l1_y(curr);
            for (int d = 0; d < 4; d++) {
                int px = cx - l1_dx[d];
                int py = cy - l1_dy[d];
                int sx = px - l1_dx[d];
                int sy = py - l1_dy[d];
                if (!l1_inside(px, py) || !l1_inside(sx, sy)) continue;
                uint8_t prev = l1_cell(px, py);
                uint8_t support = l1_cell(sx, sy);
                if (l1_wall[prev] || l1_wall[support]) continue;
                if (l1_goal_dist[g][prev] != 0xFFU) continue;
                l1_goal_dist[g][prev] =
                    (uint8_t)(l1_goal_dist[g][curr] + 1U);
                l1_walk_queue[tail++] = prev;
            }
        }
    }
    memset(l1_goal_reach, 0, sizeof(g_sokoban_workspace.level1.goal_reach_mask));
    for (int cell = 0; cell < L1_CELLS; cell++) {
        SokobanBoxMask mask = 0U;
        for (int g = 0; g < l1_goal_count; g++) {
            if (l1_goal_dist[g][cell] != 0xFFU)
                mask |= SOKOBAN_BOX_MASK_BIT(g);
        }
        l1_goal_reach[cell] = mask;
    }
    return true;
}

/* Minimum bipartite assignment of remaining generic boxes to unused goals. */
static uint16_t l1_heuristic(const uint8_t boxes[MAX_BOXES1],
                             SokobanBoxMask used_goal_mask)
{
    uint16_t dp[1U << MAX_BOXES1];
    uint8_t goals[MAX_BOXES1];
    uint8_t active_boxes[MAX_BOXES1];
    const uint16_t INF = 0xFFFFU;
    int remaining_goals = 0;
    int remaining_boxes = 0;

    for (int g = 0; g < l1_goal_count; g++) {
        if ((used_goal_mask & SOKOBAN_BOX_MASK_BIT(g)) == 0U)
            goals[remaining_goals++] = (uint8_t)g;
    }
    for (int b = 0; b < l1_box_count; b++) {
        if (boxes[b] != L1_REMOVED)
            active_boxes[remaining_boxes++] = boxes[b];
    }
    if (remaining_boxes != remaining_goals) return INF;
    if (remaining_boxes == 0) return 0U;

    /* One forward subset DP is enough: popcount(mask) identifies which box is
     * assigned next.  This removes the old second 64-entry array plus a full
     * clear/copy for every box layer. */
    unsigned limit = 1U << remaining_goals;
    for (unsigned i = 0; i < limit; i++) dp[i] = INF;
    dp[0] = 0U;

    for (unsigned mask = 0U; mask < limit; mask++) {
        uint16_t base = dp[mask];
        if (base == INF) continue;
        unsigned bits = mask;
        int box_index = 0;
        while (bits != 0U) {
            bits &= bits - 1U;
            box_index++;
        }
        if (box_index >= remaining_boxes) continue;
        uint8_t box_cell = active_boxes[box_index];

        for (int gi = 0; gi < remaining_goals; gi++) {
            unsigned bit = 1U << gi;
            if ((mask & bit) != 0U) continue;
            uint8_t dist = l1_goal_dist[goals[gi]][box_cell];
            if (dist == 0xFFU) continue;
            unsigned next_mask = mask | bit;
            uint16_t sum = (uint16_t)(base + dist);
            if (sum < dp[next_mask]) dp[next_mask] = sum;
        }
    }
    return dp[limit - 1U];
}

static bool l1_cell_can_reach_unused_goal(uint8_t cell, SokobanBoxMask used_mask)
{
    SokobanBoxMask unused = (SokobanBoxMask)(l1_full_goal_mask & (SokobanBoxMask)~used_mask);
    return (l1_goal_reach[cell] & unused) != 0U;
}

static uint16_t l1_weighted_f(uint16_t g, uint16_t h, uint8_t weight)
{
    uint32_t f = (uint32_t)g + (uint32_t)weight * h;
    return f > 65535U ? 65535U : (uint16_t)f;
}

static uint16_t l1_run_search(uint8_t start_player,
                              const uint8_t start_boxes[MAX_BOXES1],
                              uint8_t weight)
{
    memset(l1_hash, 0, sizeof(g_sokoban_workspace.level1.push_hash));
    l1_node_count = 0U;
    l1_heap_count = 0U;

    for (int b = 0; b < l1_box_count; b++) {
        if (start_boxes[b] != L1_REMOVED &&
            l1_has_2x2_deadlock(start_boxes[b], start_boxes)) {
            deadlock_pruned++;
            return L1_NO_NODE;
        }
    }
    uint16_t h0 = l1_heuristic(start_boxes, 0U);
    if (h0 == 0xFFFFU) {
        deadlock_pruned++;
        return L1_NO_NODE;
    }

    L1Node *root = &l1_nodes[0];
    memset(root, 0, sizeof(*root));
    root->parent = L1_NO_NODE;
    root->g_cost = 0U;
    root->f_cost = l1_weighted_f(0U, h0, weight);
    root->heap_pos = L1_NO_NODE;
    root->player = start_player;
    memcpy(root->boxes, start_boxes, l1_box_count);
    root->region = l1_region_anchor(start_player, start_boxes);
    root->move_box = L1_REMOVED;
    root->move_dir = 0xFFU;
    root->active_box = 0U;

    uint16_t slot;
    (void)l1_hash_lookup(root->region, root->boxes, 0U, &slot);
    if (slot == L1_NO_NODE) return L1_NO_NODE;
    l1_hash[slot] = 1U;
    l1_node_count = 1U;
    l1_heap_insert(0U);

    while (l1_heap_count > 0U) {
        uint16_t curr_idx = l1_heap_pop();
        if (curr_idx == L1_NO_NODE) break;
        L1Node *curr = &l1_nodes[curr_idx];
        if (curr->closed) continue;
        curr->closed = 1U;
        expanded_nodes++;

        if (curr->active_box == l1_full_goal_mask) return curr_idx;
        l1_build_reachability(curr->player, curr->boxes);

        for (int b = 0; b < l1_box_count; b++) {
            uint8_t box_cell = curr->boxes[b];
            if (box_cell == L1_REMOVED) continue;
            int bx = l1_x(box_cell), by = l1_y(box_cell);

            for (int d = 0; d < 4; d++) {
                int fx = bx + l1_dx[d], fy = by + l1_dy[d];
                int hx = bx - l1_dx[d], hy = by - l1_dy[d];
                if (!l1_inside(fx, fy) || !l1_inside(hx, hy)) continue;
                uint8_t front = l1_cell(fx, fy);
                uint8_t behind = l1_cell(hx, hy);
                if (l1_wall[front] || l1_occupied[front]) continue;
                if (l1_walk_dist[behind] == 0xFFU) continue;
                generated_nodes++;

                SokobanBoxMask next_goal_mask = curr->active_box;
                int goal_idx = l1_goal_index[front];
                bool consumed = goal_idx >= 0 &&
                    ((next_goal_mask & SOKOBAN_BOX_MASK_BIT(goal_idx)) == 0U);
                if (consumed) next_goal_mask |= SOKOBAN_BOX_MASK_BIT(goal_idx);
                else if (!l1_cell_can_reach_unused_goal(front, next_goal_mask)) {
                    deadlock_pruned++;
                    continue;
                }

                uint16_t new_g = (uint16_t)(curr->g_cost +
                                            l1_walk_dist[behind] + 1U);
                if (new_g >= L1_PATH_LIMIT) continue;

                uint8_t next_boxes[MAX_BOXES1];
                memcpy(next_boxes, curr->boxes, l1_box_count);
                next_boxes[b] = consumed ? L1_REMOVED : front;
                l1_sort_boxes(next_boxes);
                uint16_t h;
                if (!consumed && l1_has_2x2_deadlock(front, next_boxes)) {
                    deadlock_pruned++;
                    continue;
                }

                uint8_t next_player = box_cell;
                uint8_t next_region = l1_region_anchor(next_player, next_boxes);
                uint16_t hash_slot;
                uint16_t next_idx = l1_hash_lookup(next_region, next_boxes,
                                                    next_goal_mask, &hash_slot);
                g_l1_astar_pushes1++;

                if (next_idx == L1_NO_NODE) {
                    if (hash_slot == L1_NO_NODE) {
                        l1_search_capacity_hit = true;
                        continue;
                    }
                    if (l1_node_count >= L1_MAX_NODES) {
                        l1_search_capacity_hit = true;
                        g_l1_node_pool_hits1++;
                        continue;
                    }
                    h = l1_heuristic(next_boxes, next_goal_mask);
                    if (h == 0xFFFFU) {
                        deadlock_pruned++;
                        continue;
                    }
                    next_idx = l1_node_count++;
                    L1Node *next_node = &l1_nodes[next_idx];
                    memset(next_node, 0, sizeof(*next_node));
                    next_node->parent = curr_idx;
                    next_node->g_cost = new_g;
                    next_node->f_cost = l1_weighted_f(new_g, h, weight);
                    next_node->heap_pos = L1_NO_NODE;
                    next_node->player = next_player;
                    next_node->region = next_region;
                    memcpy(next_node->boxes, next_boxes, l1_box_count);
                    next_node->move_box = box_cell; /* old cell, stable after sort */
                    next_node->move_dir = (uint8_t)d;
                    next_node->active_box = next_goal_mask;
                    l1_hash[hash_slot] = (uint16_t)(next_idx + 1U);
                    l1_heap_insert(next_idx);
                } else {
                    L1Node *next_node = &l1_nodes[next_idx];
                    duplicate_nodes++;
                    if (new_g >= next_node->g_cost) continue;
                    /* h belongs to the canonical box/goal state and is
                     * unchanged by a cheaper representative player cell. */
                    h = (uint16_t)((next_node->f_cost - next_node->g_cost) /
                                   weight);
                    next_node->parent = curr_idx;
                    next_node->g_cost = new_g;
                    next_node->f_cost = l1_weighted_f(new_g, h, weight);
                    next_node->player = next_player;
                    next_node->move_box = box_cell;
                    next_node->move_dir = (uint8_t)d;
                    if (next_node->closed) {
                        next_node->closed = 0U;
                        g_l1_astar_reopens1++;
                    }
                    if (next_node->heap_pos == L1_NO_NODE)
                        l1_heap_insert(next_idx);
                    else {
                        l1_heap_up(next_node->heap_pos);
                        l1_heap_down(next_node->heap_pos);
                    }
                }
            }
        }
    }
    return L1_NO_NODE;
}

static int l1_build_walk_path(uint8_t start, uint8_t goal,
                              const uint8_t boxes[MAX_BOXES1],
                              char *out, int capacity)
{
    memset(l1_walk_parent, 0xFF,
           sizeof(g_sokoban_workspace.level1.walk_parent));
    memset(l1_occupied, 0, L1_CELLS);
    for (int i = 0; i < l1_box_count; i++)
        if (boxes[i] != L1_REMOVED) l1_occupied[boxes[i]] = 1U;

    int head = 0, tail = 0;
    l1_walk_parent[start] = start;
    l1_walk_queue[tail++] = start;
    while (head < tail && l1_walk_parent[goal] < 0) {
        uint8_t curr = l1_walk_queue[head++];
        int x = l1_x(curr), y = l1_y(curr);
        for (int d = 0; d < 4; d++) {
            int nx = x + l1_dx[d], ny = y + l1_dy[d];
            if (!l1_inside(nx, ny)) continue;
            uint8_t next = l1_cell(nx, ny);
            if (l1_wall[next] || l1_occupied[next]) continue;
            if (l1_walk_parent[next] >= 0) continue;
            l1_walk_parent[next] = curr;
            l1_walk_move[next] = l1_cmd[d];
            l1_walk_queue[tail++] = next;
        }
    }
    if (l1_walk_parent[goal] < 0) return -1;

    int len = 0;
    uint8_t curr = goal;
    while (curr != start) {
        if (len >= L1_CELLS) return -1;
        l1_reverse_path[len++] = l1_walk_move[curr];
        curr = (uint8_t)l1_walk_parent[curr];
    }
    if (len > capacity) return -2;
    for (int i = 0; i < len; i++) out[i] = l1_reverse_path[len - 1 - i];
    return len;
}

static bool l1_reconstruct(uint16_t goal_idx)
{
    int chain_len = 0;
    uint16_t curr = goal_idx;
    while (curr != L1_NO_NODE) {
        if (chain_len >= L1_PATH_LIMIT) {
            l1_path_capacity_hit = true;
            return false;
        }
        l1_chain[chain_len++] = curr;
        curr = l1_nodes[curr].parent;
    }
    if (chain_len <= 0) return false;

    const L1Node *root = &l1_nodes[l1_chain[chain_len - 1]];
    uint8_t player = root->player;
    uint8_t boxes[MAX_BOXES1];
    memcpy(boxes, root->boxes, l1_box_count);
    SokobanBoxMask used_goal_mask = root->active_box;
    int out_len = 0;

    for (int k = chain_len - 2; k >= 0; k--) {
        const L1Node *child = &l1_nodes[l1_chain[k]];
        uint8_t old_box = child->move_box;
        int d = child->move_dir;
        int box_index = -1;
        for (int i = 0; i < l1_box_count; i++) {
            if (boxes[i] == old_box) { box_index = i; break; }
        }
        if (box_index < 0 || d < 0 || d >= 4) return false;

        int bx = l1_x(old_box), by = l1_y(old_box);
        int hx = bx - l1_dx[d], hy = by - l1_dy[d];
        int fx = bx + l1_dx[d], fy = by + l1_dy[d];
        if (!l1_inside(hx, hy) || !l1_inside(fx, fy)) return false;
        uint8_t behind = l1_cell(hx, hy);
        int walk_len = l1_build_walk_path(player, behind, boxes,
                                          l1_work_path + out_len,
                                          L1_PATH_LIMIT - 1 - out_len);
        if (walk_len == -2) l1_path_capacity_hit = true;
        if (walk_len < 0) return false;
        out_len += walk_len;
        if (out_len >= L1_PATH_LIMIT - 1) {
            l1_path_capacity_hit = true;
            return false;
        }
        l1_work_path[out_len++] = l1_cmd[d];

        uint8_t front = l1_cell(fx, fy);
        int goal_index = l1_goal_index[front];
        bool consumed = goal_index >= 0 &&
            ((used_goal_mask & SOKOBAN_BOX_MASK_BIT(goal_index)) == 0U);
        if (consumed) {
            used_goal_mask |= SOKOBAN_BOX_MASK_BIT(goal_index);
            boxes[box_index] = L1_REMOVED;
        } else {
            boxes[box_index] = front;
        }
        l1_sort_boxes(boxes);
        player = old_box;

        if (player != child->player || used_goal_mask != child->active_box ||
            memcmp(boxes, child->boxes, l1_box_count) != 0) return false;
    }

    if (out_len >= (int)sizeof(final_commands1)) {
        l1_path_capacity_hit = true;
        return false;
    }
    final_steps1 = out_len;
    final_path_len = (uint32_t)out_len;
    memcpy(final_commands1, l1_work_path, (size_t)out_len);
    final_commands1[out_len] = '\0';
    return true;
}

void solve_sokoban1(void)
{
    final_steps1 = 0;
    final_commands1[0] = '\0';
    sokoban_error1 = SOKOBAN_OK1;
    search_nodes1 = 0U;
    l1_path_capacity_hit = false;
    l1_search_capacity_hit = false;
    g_l1_astar_attempts1 = 0U;
    g_l1_astar_nodes1 = 0U;
    g_l1_astar_pushes1 = 0U;
    g_l1_astar_reopens1 = 0U;
    g_l1_astar_weight1 = -1;
    g_l1_astar_result1 = 1;
    expanded_nodes = 0U;
    generated_nodes = 0U;
    duplicate_nodes = 0U;
    deadlock_pruned = 0U;
    max_open = 0U;
    final_path_len = 0U;
    g_l1_hash_lookups1 = 0U;
    g_l1_hash_probes1 = 0U;
    g_l1_hash_collisions1 = 0U;
    g_l1_hash_max_probe1 = 0U;
    g_l1_node_pool_hits1 = 0U;
    g_l1_hash_full_hits1 = 0U;

    if (map_parse_error1 || num_initial_boxes1 <= 0 ||
        num_initial_boxes1 > MAX_BOXES1 ||
        num_initial_targets1 != num_initial_boxes1) {
        if (!map_parse_error1 && num_initial_boxes1 == 0 &&
            num_initial_targets1 == 0) {
            g_l1_astar_result1 = 0;
            return;
        }
        sokoban_error1 = SOKOBAN_ERROR_INVALID_MAP1;
        g_l1_astar_result1 = -1;
        return;
    }

    l1_box_count = (uint8_t)num_initial_boxes1;
    l1_goal_count = (uint8_t)num_initial_targets1;
    l1_full_goal_mask = sokoban_full_mask_u8(l1_goal_count);
    memset(l1_wall, 0, L1_CELLS);
    memset(l1_goal_index, 0xFF, L1_CELLS);
    memset(l1_floor_rows, 0, sizeof(l1_floor_rows));

    for (int y = 0; y < L1_H; y++) {
        for (int x = 0; x < L1_W; x++) {
            l1_wall[l1_cell(x, y)] = raw_map_safe1[y][x] == '#' ? 1U : 0U;
            if (!l1_wall[l1_cell(x, y)])
                l1_floor_rows[y] |= (uint16_t)(1U << x);
        }
    }
    for (int g = 0; g < l1_goal_count; g++) {
        uint8_t c = l1_cell(initial_targets1[g].x, initial_targets1[g].y);
        l1_goal_cells[g] = c;
        l1_goal_index[c] = (int8_t)g;
    }
    (void)l1_build_goal_distances();

    uint8_t start_boxes[MAX_BOXES1];
    for (int i = 0; i < l1_box_count; i++)
        start_boxes[i] = l1_cell(initial_boxes1[i].x, initial_boxes1[i].y);
    l1_sort_boxes(start_boxes);
    uint8_t start_player = l1_cell(start_player1.x, start_player1.y);

    for (unsigned attempt = 0;
         attempt < sizeof(l1_weights) / sizeof(l1_weights[0]); attempt++) {
        g_l1_astar_attempts1++;
        uint16_t goal = l1_run_search(start_player, start_boxes,
                                      l1_weights[attempt]);
        search_nodes1 += l1_node_count;
        g_l1_astar_nodes1 += l1_node_count;
        if (goal != L1_NO_NODE && l1_reconstruct(goal)) {
            g_l1_astar_weight1 = l1_weights[attempt];
            g_l1_astar_result1 = 0;
            return;
        }
    }

    if (l1_path_capacity_hit)
        sokoban_error1 = SOKOBAN_ERROR_PATH_CAPACITY1;
    else if (l1_search_capacity_hit)
        sokoban_error1 = SOKOBAN_ERROR_SEARCH_CAPACITY1;
    else
        sokoban_error1 = SOKOBAN_ERROR_NO_SOLUTION1;
    g_l1_astar_result1 = -2;
}

void clear_sokoban_memory1(void)
{
    sokoban_workspace_clear();
    final_steps1 = 0;
    memset(final_commands1, 0, sizeof(final_commands1));
    sokoban_error1 = SOKOBAN_OK1;
    search_nodes1 = 0U;
    l1_node_count = 0U;
    l1_heap_count = 0U;
    l1_box_count = 0U;
    l1_goal_count = 0U;
    l1_full_goal_mask = 0U;
    l1_path_capacity_hit = false;
    l1_search_capacity_hit = false;
    g_l1_astar_attempts1 = 0U;
    g_l1_astar_nodes1 = 0U;
    g_l1_astar_pushes1 = 0U;
    g_l1_astar_reopens1 = 0U;
    g_l1_astar_weight1 = -1;
    g_l1_astar_result1 = 0;
    expanded_nodes = 0U;
    generated_nodes = 0U;
    duplicate_nodes = 0U;
    deadlock_pruned = 0U;
    max_open = 0U;
    final_path_len = 0U;
    g_l1_hash_lookups1 = 0U;
    g_l1_hash_probes1 = 0U;
    g_l1_hash_collisions1 = 0U;
    g_l1_hash_max_probe1 = 0U;
    g_l1_node_pool_hits1 = 0U;
    g_l1_hash_full_hits1 = 0U;
    memset(l1_floor_rows, 0, sizeof(l1_floor_rows));
    clear_sokoban_map1_state();
}
