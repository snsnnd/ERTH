#include "na.h"
#include "float.h"
#include <limits.h> // 用于UINT_MAX
#include <stdbool.h>
#include <string.h>   // 用于memset
#include <math.h>     // 用于isnan或比较
#include "arm_math.h" // CMSIS-DSP for optimized math
#include "BT.h"

// 外部全局地图
extern Map global_map;
extern int16_t g_node_id_to_idx[MAX_NODE_ID];
extern bool g_node_map_inited; // 映射表是否已初始化

// 1. 定义“无效节点ID”常量（建议放在全局头文件或函数上方，确保唯一）
#define INVALID_NODE_ID UINT8_MAX // uint8_t最大值255，用于标记“无前驱节点”

// 优化：根据两个节点ID获取边的代价（使用边索引表）
static float get_edge_cost(uint8_t node1_id, uint8_t node2_id)
{
    // 检查边索引表是否初始化
    if (!g_edge_map_inited)
    {
#if DEBUG_PRINT
        log_print("na->Error: The edge index table is not initialized\r\n");
#endif
        return -1.0f;
    }

    // 检查节点ID是否超出范围
    if (node1_id >= MAX_NODE_ID || node2_id >= MAX_NODE_ID)
    {
#if DEBUG_PRINT
        log_print("Node ID out of range:\n");
        log_print("%d -> %d\r\n", node1_id, node2_id);
#endif
        return -1.0f;
    }

    // 直接从索引表获取代价
    float cost = g_edge_cost_map[node1_id][node2_id];
    if (cost < 0.0f)
    { // 未找到对应边（初始化为-1.0f）
#if DEBUG_PRINT
        log_print("Edge not found\n");
        log_print("%d -> %d\r\n", node1_id, node2_id);
#endif
        return -1.0f;
    }

    return cost;
}

// 二进制堆辅助函数
static void heap_swap(PathNode *heap, uint16_t i, uint16_t j, uint16_t *heap_indices)
{
    uint8_t id_i = heap[i].node_id;
    uint8_t id_j = heap[j].node_id;
    PathNode temp = heap[i];
    heap[i] = heap[j];
    heap[j] = temp;
    heap_indices[id_i] = j;
    heap_indices[id_j] = i;
}

static void heap_sift_up(PathNode *heap, uint16_t index, uint16_t *heap_indices)
{
    while (index > 0)
    {
        uint16_t parent = (index - 1) >> 1;
        if (heap[index].f_cost >= heap[parent].f_cost)
            break;
        heap_swap(heap, index, parent, heap_indices);
        index = parent;
    }
}

static void heap_sift_down(PathNode *heap, uint16_t size, uint16_t index, uint16_t *heap_indices)
{
    while (index < size)
    {
        uint16_t min_index = index;
        uint16_t left = (index << 1) + 1;
        uint16_t right = (index << 1) + 2;

        if (left < size && heap[left].f_cost < heap[min_index].f_cost)
        {
            min_index = left;
        }
        if (right < size && heap[right].f_cost < heap[min_index].f_cost)
        {
            min_index = right;
        }
        if (min_index == index)
            break;

        heap_swap(heap, index, min_index, heap_indices);
        index = min_index;
    }
}

// 核心逻辑：计算给定坐标（地图坐标）的最近节点
static bool find_closest_node_core(float x_map, float y_map, uint8_t *node_id)
{
#if DEBUG_PRINT
    log_print("na->find_closest_node_core: Input map coordinates (%.2f, %.2f)\r\n", x_map, y_map);
#endif
    float min_dist = FLT_MAX;
    uint8_t closest_id = global_map.nodes[0].node_id;

    for (uint8_t i = 0; i < global_map.node_count; i++)
    {
        Node *node = &global_map.nodes[i];
        float dx = x_map - node->x;
        float dy = y_map - node->y;
        float dist;
        arm_sqrt_f32(dx * dx + dy * dy, &dist);
#if DEBUG_PRINT
        log_print("na->  Node ID: %d, Node coordinates (%.2f, %.2f), Distance: %.2f\r\n",
                  node->node_id, node->x, node->y, dist);
#endif
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_id = node->node_id;
#if DEBUG_PRINT
            log_print("na->  Updating closest node: ID=%d, New distance=%.2f\r\n", closest_id, min_dist);
#endif
        }
    }

    if (min_dist == FLT_MAX)
    {
#if DEBUG_PRINT
        log_print("na->No closest node found\r\n");
#endif
        return false;
    }

#if DEBUG_PRINT
    log_print("na->Found closest node: ID=%d, Minimum distance=%.2f\r\n", closest_id, min_dist);
#endif
    *node_id = closest_id;
    return true;
}

// 1. 处理实际坐标（需要转换为地图坐标）
static bool find_closest_node_enemy(float real_x, float real_y, uint8_t *node_id)
{
    if (global_map.node_count == 0 || node_id == NULL || !map_validate(&global_map))
    {
#if DEBUG_PRINT
        log_print("na->The map is invalid or nodes are empty\r\n");
#endif
        return false;
    }

    // 实际坐标 -> 地图坐标
    float x_map = real_to_map(real_x);
    float y_map = real_to_map(real_y);
    if (isnan(x_map) || isnan(y_map))
    {
#if DEBUG_PRINT
        log_print("na->Real to map conversion failed:(%.2f, %.2f)\r\n", real_x, real_y);
#endif
        return false;
    }

    // 复用核心逻辑：计算最近节点
    return find_closest_node_core(x_map, y_map, node_id);
}

// 2. 处理地图坐标（无需转换）
static bool find_closest_node_self(float map_x, float map_y, uint8_t *node_id)
{
    if (global_map.node_count == 0 || node_id == NULL || !map_validate(&global_map))
    {
#if DEBUG_PRINT
        log_print("na->The map is invalid or nodes are empty\r\n");
#endif
        return false;
    }

    // 直接使用地图坐标计算
    return find_closest_node_core(map_x, map_y, node_id);
}

static float heuristic_distance(uint8_t node1_id, uint8_t node2_id)
{
    Node *node1 = get_node_by_id(node1_id);
    Node *node2 = get_node_by_id(node2_id);
    if (!node1 || !node2)
    {
#if DEBUG_PRINT
        log_print("na->heuristic_node is not found");
#endif
        return -1.0f;
    }
    return fabsf(node1->x - node2->x) + fabsf(node1->y - node2->y);
}

static bool reconstruct_path(uint8_t came_from[], uint8_t current_node_id, uint8_t target_node_id, Direction *direction)
{
    if (came_from == NULL || direction == NULL)
    {
#if DEBUG_PRINT
        log_print("na->The path backtracking parameter is invalid\r\n");
#endif
        return false;
    }

    if (current_node_id == target_node_id)
    {
        *direction = DIRECTION_STOP;
        return true;
    }

    // 回溯修复：从目标节点沿 came_from（父指针）回溯到起点，得到
    // goal→…→start 的逆序路径，再反转取路径第一步（起点 → 起点在最优
    // 路径上的后继）计算行进方向；旧实现从起点向后找“前驱是起点”的
    // 节点，既可能选中不在最终路径上的分支，方向语义也是反的。
    uint8_t path[MAX_NODES];
    uint16_t path_len = 0;
    uint8_t walk_id = target_node_id;
    while (walk_id != INVALID_NODE_ID && walk_id < MAX_NODES && path_len < MAX_NODES)
    {
        path[path_len++] = walk_id;
        if (walk_id == current_node_id)
        {
            break; // 已回溯到起点
        }
        walk_id = came_from[walk_id];
    }

    // 必须完整回溯到起点，且路径至少包含起点与其后继两个节点
    if (path_len < 2 || path[path_len - 1] != current_node_id)
    {
#if DEBUG_PRINT
        log_print("na->Path backtracking failed: %d -> %d\r\n", current_node_id, target_node_id);
#endif
        return false;
    }

    // 反转后取第一步的后继节点（path[path_len-1] 即起点 current_node_id）
    uint8_t next_node_id = path[path_len - 2];

    // 检查后继节点有效性
    if (next_node_id == INVALID_NODE_ID)
    {
#if DEBUG_PRINT
        log_print("na->Invalid next node ID: no predecessor (current node ID=%d)\r\n", current_node_id);
#endif
        return false;
    }

    if (next_node_id >= MAX_NODES)
    {
#if DEBUG_PRINT
        log_print("na->Next node ID exceeds MAX_NODES: ID=%d, MAX=%d\r\n", next_node_id, MAX_NODES);
#endif
        return false;
    }

    // 后续坐标计算逻辑不变...
    Node *current_node = get_node_by_id(current_node_id);
    Node *next_node = get_node_by_id(next_node_id);
    if (current_node == NULL || next_node == NULL)
    {
#if DEBUG_PRINT
        log_print("na->Failed to get current/next node (current ID=%d, next ID=%d)\r\n", current_node_id, next_node_id);
#endif
        return false;
    }

    float dx = next_node->x - current_node->x;
    float dy = next_node->y - current_node->y;
    float abs_dx = fabsf(dx);
    float abs_dy = fabsf(dy);

    if (abs_dx > abs_dy)
    {
        *direction = (dx > 0) ? DIRECTION_RIGHT : DIRECTION_LEFT;
    }
    else
    {
        *direction = (dy > 0) ? DIRECTION_UP : DIRECTION_DOWN;
    }
#if DEBUG_PRINT
    log_print("na->the direction need to do: %s\r\n",
              (*direction == DIRECTION_LEFT) ? "left" : (*direction == DIRECTION_RIGHT) ? "right"
                                                    : (*direction == DIRECTION_UP)      ? "up"
                                                                                        : "down");
#endif
    return true;
}

static bool a_star_pathfinding(uint8_t start_node_id, uint8_t goal_node_id, Direction *direction, bool allow_danger, uint8_t recursion_depth)
{
    if (recursion_depth > MAX_ASTAR_RECURSION)
    {
#if DEBUG_PRINT
        log_print("na->A* Recursion depth exceeds limit:%d\r\n", recursion_depth);
#endif
        return false;
    }
#if DEBUG_PRINT
    log_print("na->A* Recursion depth is enough\n");
#endif
    if (direction == NULL || !map_validate(&global_map))
    {
#if DEBUG_PRINT
        log_print("na->The A* direction or map is invalid\r\n");
#endif
        return false;
    }
#if DEBUG_PRINT
    log_print("na->A* direction or map is right\n");
#endif
    Node *start_node = get_node_by_id(start_node_id);
    Node *goal_node = get_node_by_id(goal_node_id);
    if (start_node == NULL || goal_node == NULL)
    {
#if DEBUG_PRINT
        log_print("na->not find start_node or goal_node\n");
#endif
        return false;
    }
    else
    {
#if DEBUG_PRINT
        log_print("na->start_node:%d goal_node:%d\n", start_node->node_id, goal_node->node_id);
#endif
    }

    float g_score[MAX_NODES];
    uint8_t came_from[MAX_NODES];
    PathNode heap[MAX_NODES];
    uint16_t heap_indices[MAX_NODES];
    uint16_t heap_size = 0;
    uint16_t max_heap_size = global_map.node_count;

    for (uint16_t i = 0; i < MAX_NODES; i++)
    {
        g_score[i] = FLT_MAX;
        came_from[i] = UINT8_MAX;
        heap_indices[i] = 0xFFFF;
    }

    g_score[start_node_id] = 0.0f;
    float h = heuristic_distance(start_node_id, goal_node_id);
    if (h < 0.0f)
    {
#if DEBUG_PRINT
        log_print("na->Heuristic distance is invalid:%d -> %d\r\n", start_node_id, goal_node_id);
#endif
        return false;
    }

    heap[0] = (PathNode){start_node_id, 0.0f, h, h, UINT8_MAX, 0, false};
    heap_indices[start_node_id] = 0;
    heap_size = 1;
    bool goal_in_heap = false;

    while (heap_size > 0)
    {
        PathNode current = heap[0];
        heap_swap(heap, 0, heap_size - 1, heap_indices);
        heap_size--;
        heap_sift_down(heap, heap_size, 0, heap_indices);
        heap_indices[current.node_id] = 0xFFFF;

        if (current.is_goal)
        {
#if DEBUG_PRINT
            // 回溯路径并打印
            uint8_t path[MAX_NODES];
            uint16_t path_len = 0;
            uint8_t curr_id = goal_node_id;
            while (curr_id != UINT8_MAX && path_len < MAX_NODES)
            {
                path[path_len++] = curr_id;
                if (curr_id == start_node_id)
                    break;
                curr_id = came_from[curr_id];
            }
            log_print("A* Path: ");

            for (int i = path_len - 1; i >= 0; i--)
            {

                log_print("%d%s", path[i], i > 0 ? " -> " : "\r\n");
            }

            // 新增：反转路径，得到从起点到目标的顺序（start -> ... -> goal）
            uint8_t forward_path[MAX_NODES];
            for (int i = 0; i < path_len; i++)
            {
                forward_path[i] = path[path_len - 1 - i]; // 反转：目标->起点 转为 起点->目标
            }

            // 新增：遍历每个节点，计算到下一个节点的方向并打印
            log_print("the direction of each node\r\n");
            for (int i = 0; i < path_len - 1; i++) // 最后一个节点（目标）无下一个节点，不遍历
            {
                uint8_t current_node_id = forward_path[i];
                uint8_t next_node_id = forward_path[i + 1];

                Node *current_node = get_node_by_id(current_node_id);
                Node *next_node = get_node_by_id(next_node_id);
                if (current_node == NULL || next_node == NULL)
                {
                    log_print("  node%d->%d :node is useless\r\n", current_node_id, next_node_id);
                    continue;
                }

                // 计算方向
                float dx = next_node->x - current_node->x;
                float dy = next_node->y - current_node->y;
                float abs_dx = fabsf(dx);
                float abs_dy = fabsf(dy);
                const char *dir_str;

                if (abs_dx > abs_dy)
                {
                    dir_str = (dx > 0) ? "right" : "left";
                }
                else
                {
                    dir_str = (dy > 0) ? "up" : "dowm";
                }

                // 打印当前节点ID、下一个节点ID及方向（起始节点的朝向就是第一个节点的方向）
                log_print("  node_1: %d -> node_2: %d  direction: %s\r\n", current_node_id, next_node_id, dir_str);
            }
#endif
            return reconstruct_path(came_from, start_node_id, goal_node_id, direction);
        } // 到达目标

        Node *current_node = get_node_by_id(current.node_id);
        if (current_node == NULL)
        {
#if DEBUG_PRINT
            log_print("na->current.node_id:%d is not found\n", current.node_id);
#endif
            continue;
        }
#if DEBUG_PRINT
        log_print("na->current_node:%d\n", current_node->node_id);
#endif
        for (uint8_t i = 0; i < current_node->neighbor_count; i++)
        {
            uint8_t neighbor_id = current_node->neighbors[i];
            // 新增：过滤“无效邻居”标记
            if (neighbor_id == INVALID_NODE_ID)
            {
                continue; // 跳过无邻居的位置
            }
            Node *neighbor = get_node_by_id(neighbor_id);
            if (neighbor == NULL || (!allow_danger && neighbor->is_dangerous))
                continue;

            float edge_cost = get_edge_cost(current.node_id, neighbor_id);
            if (edge_cost < 0.0f)
                continue;

            float neighbor_cost = neighbor->is_dangerous ? neighbor->cost * DANGER_COST_MULTIPLIER : neighbor->cost;
            float tentative_g = g_score[current.node_id] + edge_cost + neighbor_cost;

            if (tentative_g < g_score[neighbor_id])
            {
                came_from[neighbor_id] = current.node_id;
                g_score[neighbor_id] = tentative_g;
                float h = heuristic_distance(neighbor_id, goal_node_id);
                if (h < 0.0f)
                    continue;
                float f = tentative_g + h;

                bool is_goal_node = (neighbor_id == goal_node_id);
                if (is_goal_node)
                {
                    goal_in_heap = true;
                }

                if (heap_indices[neighbor_id] != 0xFFFF)
                {
                    uint16_t idx = heap_indices[neighbor_id];
                    heap[idx].g_cost = tentative_g;
                    heap[idx].h_cost = h;
                    heap[idx].f_cost = f;
                    heap[idx].parent_id = current.node_id;
                    heap[idx].is_goal = is_goal_node;
                    heap_sift_up(heap, idx, heap_indices);
                }
                else
                {
                    if (heap_size >= max_heap_size)
                    {
                        if (goal_in_heap)
                        {
#if DEBUG_PRINT
                            // 回溯路径并打印
                            uint8_t path[MAX_NODES];
                            uint16_t path_len = 0;
                            uint8_t curr_id = goal_node_id;
                            while (curr_id != UINT8_MAX && path_len < MAX_NODES)
                            {
                                path[path_len++] = curr_id;
                                if (curr_id == start_node_id)
                                    break;
                                curr_id = came_from[curr_id];
                            }
                            log_print("A* Path: ");
                            for (int i = path_len - 1; i >= 0; i--)
                            {
                                log_print("%d%s", path[i], i > 0 ? " -> " : "\r\n");
                            }
#endif
                            return reconstruct_path(came_from, start_node_id, goal_node_id, direction);
                        }
                        continue;
                    }
                    heap[heap_size] = (PathNode){neighbor_id, tentative_g, h, f, current.node_id, heap_size, is_goal_node};
                    heap_indices[neighbor_id] = heap_size;
                    heap_sift_up(heap, heap_size, heap_indices);
                    heap_size++;
                }
            }
        }
    }

    if (!allow_danger)
    {
        return a_star_pathfinding(start_node_id, goal_node_id, direction, true, recursion_depth + 1);
    }
#if DEBUG_PRINT
    log_print("na->A* No path:%d -> %d\r\n", start_node_id, goal_node_id);
#endif
    return false;
}

void map_clear_danger_zone(Map *map)
{
    if (map == NULL)
    {
#if DEBUG_PRINT
        log_print("na->The map pointer is empty\r\n");
#endif
        return;
    }
    for (uint8_t i = 0; i < map->node_count; i++)
    {
        map->nodes[i].is_dangerous = 0;
        map->nodes[i].cost = map->nodes[i].base_cost;
    }
    for (uint8_t i = 0; i < map->edge_count; i++)
    {
        map->edges[i].cost = map->edges[i].distance;
    }
}

bool select_safe_target(uint8_t enemy_node_id, uint8_t *safe_node_id)
{
    if (global_map.node_count == 0 || safe_node_id == NULL || !map_validate(&global_map))
    {
#if DEBUG_PRINT
        log_print("na->The map is invalid or the pointer of the security node is emptys\r\n");
#endif
        return false;
    }

    float max_dist = -1.0f;
    uint8_t selected_id = UINT8_MAX;
    bool found = false;

    for (uint8_t i = 0; i < global_map.node_count; i++)
    {
        Node *node = &global_map.nodes[i];
        if (node->is_dangerous == 0 && node->node_id != enemy_node_id)
        {
            float dist = heuristic_distance(node->node_id, enemy_node_id);
            if (dist >= 0.0f && (dist > max_dist || (dist == max_dist && node->node_id < selected_id)))
            {
                max_dist = dist;
                selected_id = node->node_id;
                found = true;
            }
        }
    }

    if (!found)
    {
#if DEBUG_PRINT
        log_print("na->No safe node was found\r\n");
#endif
        return false;
    }
    *safe_node_id = selected_id;
    return true;
}

/* ==================== 对战协议 → 策略层（ORP 转发） ====================
 * 由 FSM 主循环 robot_protocol_update() 每次更新（含 5s 超时清零）
 */
static float s_oppo_report_x = -1.0f;
static float s_oppo_report_y = -1.0f;
static uint8_t s_oppo_report_valid = 0;

void na_set_opponent_report(float map_x, float map_y, uint8_t valid)
{
    s_oppo_report_x = map_x;
    s_oppo_report_y = map_y;
    s_oppo_report_valid = (valid != 0) ? 1u : 0u;
}

/* ====== 选手策略区 STRATEGY：比赛时可整体替换本区实现 ======
 * 依赖：仅 map.c 图数据 API + 标准库（零 HAL 依赖，可 host gcc 单测）
 * 距离度量：图 BFS 跳数（每条边权重视为 1，不使用边 distance）
 */
#define STRATEGY_HOPS 2              // 最大图跳数（谎报偏置范围 & 对手候选集合）
#define STRATEGY_MAX_CAND 20         // 2 跳候选集合上限（图度≤4，实际上限 1+4+12=17）
#define STRATEGY_DIST_INF ((int16_t)0x7FFF) // 不可达标记

// 距离表：sb_dist_from_cand[k] = 候选 S[k] 为源的全图 BFS 距离
static int16_t sb_dist_from_cand[STRATEGY_MAX_CAND][MAX_NODE_ID];
static int16_t sb_dist_scratch[MAX_NODE_ID]; // 单次 BFS 暂存（非重入，主循环单线程调用）

/**
 * @brief  校验节点 id 并返回节点指针（NULL=非法）
 * 地图 id 从 2 开始（0/1 被跳过，见 map.c SKIP_1/SKIP_2）；
 * 幻影/映射错位 id 通过 node_id 回读比对剔除。
 * 邻居槽中可能出现 -1（→(uint8_t)255）、0、不存在的 id，均在此被拒绝 → 防越界
 */
static Node *sb_valid_node(uint8_t id)
{
    if (id < 2 || id >= MAX_NODE_ID)
    {
        return NULL;
    }
    Node *n = get_node_by_id(id);
    if (n == NULL || n->node_id != id)
    {
        return NULL;
    }
    return n;
}

/**
 * @brief  距坐标最近的有效节点（跳过 node_count 中零初始化的幻影槽位）
 * @retval node_id，0=失败
 */
static uint8_t sb_closest_node(float x_map, float y_map)
{
    float best = FLT_MAX;
    uint8_t best_id = 0;
    for (uint8_t i = 0; i < global_map.node_count; i++)
    {
        Node *n = &global_map.nodes[i];
        if (n->node_id < 2 || n->node_id >= MAX_NODE_ID)
        {
            continue; // 幻影节点（node_count 计数含零初始化槽位）
        }
        float dx = x_map - n->x;
        float dy = y_map - n->y;
        float d = dx * dx + dy * dy;
        if (d < best)
        {
            best = d;
            best_id = n->node_id;
        }
    }
    return best_id;
}

/**
 * @brief  单源 BFS（跳数距离，边权=1）
 * @param  dist: 输出数组（按 node_id 索引，长度 MAX_NODE_ID），不可达=STRATEGY_DIST_INF
 * @param  max_hops: >=0 限制展开跳数；<0 不限制
 */
static void sb_bfs(uint8_t src, int16_t *dist, int max_hops)
{
    for (uint8_t i = 0; i < MAX_NODE_ID; i++)
    {
        dist[i] = STRATEGY_DIST_INF;
    }
    if (sb_valid_node(src) == NULL)
    {
        return;
    }

    uint8_t queue[MAX_NODE_ID];
    uint8_t head = 0, tail = 0;
    dist[src] = 0;
    queue[tail++] = src;

    while (head < tail)
    {
        uint8_t id = queue[head++];
        Node *n = sb_valid_node(id);
        if (n == NULL)
        {
            continue; // 队列内均为校验过的 id（双保险）
        }
        if (max_hops >= 0 && dist[id] >= max_hops)
        {
            continue;
        }
        uint8_t cnt = n->neighbor_count;
        if (cnt > 4)
        {
            cnt = 4; // 邻居槽位固定 4，防 neighbor_count 异常
        }
        for (uint8_t i = 0; i < cnt; i++)
        {
            uint8_t nb = (uint8_t)n->neighbors[i]; // -1 → 255
            if (sb_valid_node(nb) == NULL)
            {
                continue; // 过滤 -1/0/不存在的 id：防越界与幻影节点
            }
            if (dist[nb] != STRATEGY_DIST_INF)
            {
                continue; // 已访问（也保证每个 id 只入队一次）
            }
            dist[nb] = (int16_t)(dist[id] + 1);
            queue[tail++] = nb;
        }
    }
}

/**
 * @brief  收集 src ±STRATEGY_HOPS 跳候选集合（含 src 自身），id 升序
 * @retval 集合大小（0=失败）
 */
static int sb_collect_hops(uint8_t src, uint8_t *out, int max_out)
{
    sb_bfs(src, sb_dist_scratch, STRATEGY_HOPS);
    if (sb_dist_scratch[src] == STRATEGY_DIST_INF)
    {
        return 0;
    }
    int n = 0;
    for (uint8_t id = 2; id < MAX_NODE_ID && n < max_out; id++)
    {
        if (sb_dist_scratch[id] != STRATEGY_DIST_INF)
        {
            out[n++] = id;
        }
    }
    return n;
}

/**
 * @brief  两节点坐标的欧氏距离（-1=任一节点非法）
 */
static float sb_euclid(uint8_t a, uint8_t b)
{
    Node *na = sb_valid_node(a);
    Node *nb = sb_valid_node(b);
    if (na == NULL || nb == NULL)
    {
        return -1.0f;
    }
    float dx = na->x - nb->x;
    float dy = na->y - nb->y;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief  1) 谎报位置生成：在真实位置 ±STRATEGY_HOPS 图跳内选一个偏置节点
 *         （朝远离对手的图跳方向偏移，最远者优先），输出其坐标。
 *  结构上保证：偏置节点 ∈ BFS≤2(真实最近节点) → 谎报位置与真实位置
 *  的图距离恒 ≤2 节点，平台按图距离校验 RPOS 必过。
 *  对手无效 / 最近节点失败 / 无需偏置时输出真实位置（图距离 0）。
 */
void strategy_spoof_pos(float true_x, float true_y,
                        float oppo_x, float oppo_y, uint8_t oppo_valid,
                        float *spoof_x, float *spoof_y)
{
    if (spoof_x == NULL || spoof_y == NULL)
    {
        return;
    }
    *spoof_x = true_x;
    *spoof_y = true_y; // 默认：不谎报

    if (!oppo_valid)
    {
        return;
    }

    uint8_t true_node = sb_closest_node(true_x, true_y);
    uint8_t oppo_node = sb_closest_node(oppo_x, oppo_y);
    if (true_node == 0 || oppo_node == 0)
    {
        return;
    }

    // 候选 = 真实位置 ±2 跳内的节点（含自身）
    uint8_t cand[STRATEGY_MAX_CAND];
    int cand_n = sb_collect_hops(true_node, cand, STRATEGY_MAX_CAND);
    if (cand_n <= 0)
    {
        return;
    }

    // 对手节点全图 BFS 距离（一次），用于"朝远离对手方向偏移"
    int16_t dist_to_oppo[MAX_NODE_ID];
    sb_bfs(oppo_node, dist_to_oppo, -1);

    uint8_t best = true_node;
    int best_hop = -1;
    float best_eucl = -1.0f;
    for (int i = 0; i < cand_n; i++)
    {
        uint8_t c = cand[i];
        int hop = dist_to_oppo[c];            // 越大越远离对手（不可达=INF 最远）
        float eucl = sb_euclid(c, oppo_node); // 同跳数时用欧氏距离决胜
        if (hop > best_hop || (hop == best_hop && eucl > best_eucl))
        {
            best = c;
            best_hop = hop;
            best_eucl = eucl;
        }
    }

    if (best == true_node)
    {
        return; // 最偏节点就是真实节点：报真实坐标
    }
    Node *bn = sb_valid_node(best);
    if (bn == NULL)
    {
        return;
    }
    *spoof_x = bn->x;
    *spoof_y = bn->y;
}

/**
 * @brief  2) 鲁棒最坏情况目标选择（图 BFS 跳数距离）
 *  PURSUIT(追击): S = 对手上报节点 BFS≤2 候选集（含自身）
 *                  t* = argmin_{t∈S} [ dist(我→t) + max_{s∈S} dist(t,s) ]  (minimax)
 *  EVADE(躲避)  : t* = argmax_{t∈全图有效节点} min_{s∈S} dist(t,s)          (maximin)
 *  数据无效 / 全部不可达 → 返回 0，由调用方回落原 navigate 逻辑
 */
int strategy_select_target(NavigationMode mode,
                           float self_x, float self_y,
                           float oppo_x, float oppo_y, uint8_t oppo_valid,
                           uint8_t *out_target_node_id)
{
    if (out_target_node_id == NULL)
    {
        return 0;
    }
    *out_target_node_id = 0;

    if (!oppo_valid || (mode != MODE_PURSUIT && mode != MODE_EVADE))
    {
        return 0;
    }

    uint8_t self_node = sb_closest_node(self_x, self_y);
    uint8_t oppo_node = sb_closest_node(oppo_x, oppo_y);
    if (self_node == 0 || oppo_node == 0)
    {
        return 0;
    }

    // S = 对手 ±2 跳候选集合（含对手节点自身）
    uint8_t S[STRATEGY_MAX_CAND];
    int s_n = sb_collect_hops(oppo_node, S, STRATEGY_MAX_CAND);
    if (s_n <= 0)
    {
        return 0;
    }

    // 预计算 S 内每个节点为源的全图 BFS 距离表
    for (int k = 0; k < s_n; k++)
    {
        sb_bfs(S[k], sb_dist_from_cand[k], -1);
    }

    if (mode == MODE_PURSUIT)
    {
        // minimax: t* = argmin_{t∈S} [ dist(self→t) + max_{s∈S} dist(t,s) ]
        sb_bfs(self_node, sb_dist_scratch, -1); // scratch: dist(self→·)

        int best_score = INT32_MAX;
        int best_self = INT32_MAX;
        uint8_t best_t = 0;
        for (int i = 0; i < s_n; i++)
        {
            uint8_t t = S[i];
            int d_self = sb_dist_scratch[t];
            if (d_self == STRATEGY_DIST_INF)
            {
                continue; // 我到不了 t：跳过
            }
            int worst = 0;
            for (int j = 0; j < s_n; j++)
            {
                int d = sb_dist_from_cand[i][S[j]];
                if (d == STRATEGY_DIST_INF)
                {
                    worst = STRATEGY_DIST_INF;
                    break; // t 覆盖不了某个最坏候选：跳过
                }
                if (d > worst)
                {
                    worst = d;
                }
            }
            if (worst == STRATEGY_DIST_INF)
            {
                continue;
            }
            int score = d_self + worst;
            if (score < best_score || (score == best_score && d_self < best_self))
            {
                best_score = score;
                best_self = d_self;
                best_t = t;
            }
        }
        if (best_t == 0)
        {
            return 0; // 无可达候选 → 回落原逻辑
        }
        *out_target_node_id = best_t;
        return 1;
    }

    // MODE_EVADE: maximin t* = argmax_t min_{s∈S} dist(t,s)（候选=全图有效节点）
    uint8_t best_t = 0;
    int best_min = -1;
    float best_eucl = -1.0f;
    for (uint8_t i = 0; i < global_map.node_count; i++)
    {
        Node *n = &global_map.nodes[i];
        if (n->node_id < 2 || n->node_id >= MAX_NODE_ID)
        {
            continue; // 跳过幻影节点
        }
        uint8_t t = n->node_id;

        int m = STRATEGY_DIST_INF;
        for (int j = 0; j < s_n; j++)
        {
            int d = sb_dist_from_cand[j][t];
            if (d < m)
            {
                m = d;
            }
        }
        if (m == STRATEGY_DIST_INF)
        {
            continue; // 与对手候选集完全不连通：不可用
        }
        if (m > best_min)
        {
            best_min = m;
            best_t = t;
            best_eucl = sb_euclid(t, oppo_node);
        }
        else if (m == best_min)
        {
            float e = sb_euclid(t, oppo_node); // 同分：欧氏离对手更远者优先
            if (e > best_eucl)
            {
                best_eucl = e;
                best_t = t;
            }
        }
    }
    if (best_t == 0)
    {
        return 0;
    }
    *out_target_node_id = best_t;
    return 1;
}
/* ====== 策略区结束 ====== */

bool navigate(float self_x, float self_y, float enemy_x, float enemy_y, float target_x, float target_y, NavigationMode mode, Direction *direction)
{
    if (direction == NULL || !map_validate(&global_map))
    {
#if DEBUG_PRINT
        log_print("na->The direction or map are invalid\r\n");
#endif
        return false;
    }

    uint8_t self_node_id;
    if (!find_closest_node_self(self_x, self_y, &self_node_id))
    {
#if DEBUG_PRINT
        log_print("na->Unable to find its own nearest node:(%.2f, %.2f)\r\n", self_x, self_y);
#endif
        return false;
    }

    bool has_enemy = (enemy_x >= 0.0f && enemy_y >= 0.0f);
    uint8_t enemy_node_id = 0;
    if (has_enemy)
    {
        if (!find_closest_node_enemy(enemy_x, enemy_y, &enemy_node_id))
        {
#if DEBUG_PRINT
            log_print("na->The nearest node of the enemy cannot be found:(%.2f, %.2f)\r\n", enemy_x, enemy_y);
#endif
            return false;
        }
    }

    uint8_t target_node_id;
    map_clear_danger_zone(&global_map);

#if STRATEGY_ROBUST_WORST
    // 鲁棒最坏情况决策：ORP 对手上报有效时，PURSUIT/EVADE 优先用策略区目标
    // （失败则回落各模式原有逻辑；TREASURE 不受影响）
    uint8_t robust_target_id = 0;
    uint8_t robust_ok = 0;
    if ((mode == MODE_PURSUIT || mode == MODE_EVADE) &&
        strategy_select_target(mode, self_x, self_y,
                               s_oppo_report_x, s_oppo_report_y,
                               s_oppo_report_valid, &robust_target_id))
    {
        robust_ok = 1;
    }
#endif

    switch (mode)
    {
    case MODE_PURSUIT:
#if STRATEGY_ROBUST_WORST
        if (robust_ok)
        {
            target_node_id = robust_target_id;
            break;
        }
#endif
        if (!has_enemy)
        {
#if DEBUG_PRINT
            log_print("na->The pursuit mode has no enemy coordinates\r\n");
#endif
            return false;
        }
        target_node_id = enemy_node_id;
        break;
    case MODE_EVADE:
#if STRATEGY_ROBUST_WORST
        if (robust_ok)
        {
            target_node_id = robust_target_id;
            break;
        }
#endif
        if (!has_enemy)
        {
#if DEBUG_PRINT
            log_print("na->The dodge mode has no enemy coordinates\r\n");
#endif
            return false;
        }
        map_mark_danger_zone(&global_map, enemy_node_id);
        if (!select_safe_target(enemy_node_id, &target_node_id))
        {
#if DEBUG_PRINT
            log_print("na->The security target node cannot be selected\r\n");
#endif
            return false;
        }
        break;
    case MODE_TREASURE:
        if (!find_closest_node_self(target_x, target_y, &target_node_id))
        {
#if DEBUG_PRINT
            log_print("na->The nearest node to the target cannot be found:(%.2f, %.2f)\r\n", target_x, target_y);
#endif
            return false;
        }
        if (has_enemy)
        {
            map_mark_danger_zone(&global_map, enemy_node_id);
        }
        break;
    default:
#if DEBUG_PRINT
        log_print("na->Invalid navigation mode:%d\r\n", mode);
#endif
        return false;
    }

    if (self_node_id == target_node_id)
    {
#if DEBUG_PRINT
        log_print("na->has been target\n");
#endif
        *direction = DIRECTION_STOP;
        return true;
    }

    return a_star_pathfinding(self_node_id, target_node_id, direction, false, 0);
}
