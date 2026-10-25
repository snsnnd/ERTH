#include "map.h"
#include "utils.h"
#include <string.h>
#include <float.h>
#include "BT.h"

// 危险惩罚值
#define DANGER_COST_MULTIPLIER 2.0f // 定义危险代价乘数

// 在 map.h 开头
#define INVALID_NODE_ID -1 // 定义无效节点ID

// 定义全局变量（分配内存，仅在此处定义一次）
// 注意：C 中 `= {-1}` 仅把 [0] 置 -1、其余为 0（0 会被误当成合法索引 0），
// 完整的全表 -1 初始化在 map_globalmap_init_id_map() 中用 memset 完成
int16_t g_node_id_to_idx[MAX_NODE_ID] = {-1}; // 仅 [0]=-1；全表 -1 见上方说明（memset）
bool g_node_map_inited = false;               // 初始化为未初始化
// 新增：边代价索引表（假设节点ID范围不超过MAX_NODE_ID）
float g_edge_cost_map[MAX_NODE_ID][MAX_NODE_ID];
bool g_edge_map_inited = false; // 边索引表是否初始化

/***************************  全局地图初始化  ********************************/
Map global_map = {
    .nodes = {

        {2, 50.0f, 800.0f, 0, 1.0f, 1.0f, 2, {10, 3, -1, -1}},
        {3, 210.0f, 800.0f, 0, 1.0f, 1.0f, 3, {4, 11, 2, -1}},
        {4, 370.0f, 800.0f, 0, 1.0f, 1.0f, 3, {5, 3, 13, -1}},
        {5, 450.0f, 800.0f, 0, 1.0f, 1.0f, 3, {14, 4, 6, -1}},
        {6, 530.0f, 800.0f, 0, 1.0f, 1.0f, 3, {15, 5, 7, -1}},
        {7, 690.0f, 800.0f, 0, 1.0f, 1.0f, 2, {17, 6, -1, -1}},
        {8, 770.0f, 800.0f, 0, 1.0f, 1.0f, 2, {27, 9, -1, -1}},
        {9, 930.0f, 800.0f, 0, 1.0f, 1.0f, 2, {8, 19, -1, -1}},
        {10, 50.0f, 710.0f, 0, 1.0f, 1.0f, 3, {11, 2, 20, -1}},
        {11, 210.0f, 710.0f, 0, 1.0f, 1.0f, 4, {10, 12, 3, 22}},
        {12, 290.0f, 710.0f, 0, 1.0f, 1.0f, 3, {23, 11, 13, -1}},
        {13, 370.0f, 710.0f, 0, 1.0f, 1.0f, 3, {24, 4, 12, -1}},
        {14, 450.0f, 710.0f, 0, 1.0f, 1.0f, 3, {15, 32, 5, -1}},
        {15, 530.0f, 710.0f, 0, 1.0f, 1.0f, 3, {25, 14, 6, -1}},
        {16, 610.0f, 710.0f, 0, 1.0f, 1.0f, 1, {17, -1, -1, -1}},
        {17, 690.0f, 710.0f, 0, 1.0f, 1.0f, 3, {16, 7, 26, -1}},
        {18, 850.0f, 710.0f, 0, 1.0f, 1.0f, 2, {19, 28, -1, -1}},
        {19, 930.0f, 710.0f, 0, 1.0f, 1.0f, 2, {18, 9, -1, -1}},
        {20, 50.0f, 630.0f, 0, 1.0f, 1.0f, 3, {10, 21, 37, -1}},
        {21, 130.0f, 630.0f, 0, 1.0f, 1.0f, 3, {38, 22, 20, -1}},
        {22, 210.0f, 630.0f, 0, 1.0f, 1.0f, 3, {30, 11, 21, -1}},
        {23, 290.0f, 630.0f, 0, 1.0f, 1.0f, 3, {24, 12, 31, -1}},
        {24, 370.0f, 630.0f, 0, 1.0f, 1.0f, 2, {23, 13, -1, -1}},
        {25, 530.0f, 630.0f, 0, 1.0f, 1.0f, 3, {33, 15, 26, -1}},
        {26, 690.0f, 630.0f, 0, 1.0f, 1.0f, 4, {25, 27, 35, 17}},
        {27, 770.0f, 630.0f, 0, 1.0f, 1.0f, 3, {8, 28, 26, -1}},
        {28, 850.0f, 630.0f, 0, 1.0f, 1.0f, 2, {27, 18, -1, -1}},
        {29, 930.0f, 630.0f, 0, 1.0f, 1.0f, 1, {45, -1, -1, -1}},
        {30, 210.0f, 550.0f, 0, 1.0f, 1.0f, 1, {22, -1, -1, -1}},
        {31, 290.0f, 550.0f, 0, 1.0f, 1.0f, 2, {23, 32, -1, -1}},
        {32, 450.0f, 550.0f, 0, 1.0f, 1.0f, 3, {14, 48, 31, -1}},
        {33, 530.0f, 550.0f, 0, 1.0f, 1.0f, 3, {49, 25, 34, -1}},
        {34, 610.0f, 550.0f, 0, 1.0f, 1.0f, 3, {33, 35, 50, -1}},
        {35, 690.0f, 550.0f, 0, 1.0f, 1.0f, 3, {26, 34, 36, -1}},
        {36, 850.0f, 550.0f, 0, 1.0f, 1.0f, 2, {44, 35, -1, -1}},
        {37, 50.0f, 470.0f, 0, 1.0f, 1.0f, 3, {55, 38, 20, -1}},
        {38, 130.0f, 470.0f, 0, 1.0f, 1.0f, 4, {56, 39, 21, 37}},
        {39, 210.0f, 470.0f, 0, 1.0f, 1.0f, 3, {57, 38, 40, -1}},
        {40, 290.0f, 470.0f, 0, 1.0f, 1.0f, 2, {46, 39, -1, -1}},
        {41, 370.0f, 470.0f, 0, 1.0f, 1.0f, 1, {47, -1, -1, -1}},
        {42, 690.0f, 470.0f, 0, 1.0f, 1.0f, 2, {51, 43, -1, -1}},
        {43, 770.0f, 470.0f, 0, 1.0f, 1.0f, 1, {42, -1, -1, -1}},
        {44, 850.0f, 470.0f, 0, 1.0f, 1.0f, 3, {45, 53, 36, -1}},
        {45, 930.0f, 470.0f, 0, 1.0f, 1.0f, 2, {44, 29, -1, -1}},
        {46, 290.0f, 390.0f, 0, 1.0f, 1.0f, 3, {58, 47, 40, -1}},
        {47, 370.0f, 390.0f, 0, 1.0f, 1.0f, 2, {46, 41, -1, -1}},
        {48, 450.0f, 390.0f, 0, 1.0f, 1.0f, 2, {49, 32, -1, -1}},
        {49, 530.0f, 390.0f, 0, 1.0f, 1.0f, 3, {33, 48, 61, -1}},
        {50, 610.0f, 390.0f, 0, 1.0f, 1.0f, 1, {34, -1, -1, -1}},
        {51, 690.0f, 390.0f, 0, 1.0f, 1.0f, 2, {42, 52, -1, -1}},
        {52, 770.0f, 390.0f, 0, 1.0f, 1.0f, 3, {51, 63, 53, -1}},
        {53, 850.0f, 390.0f, 0, 1.0f, 1.0f, 3, {44, 54, 52, -1}},
        {54, 930.0f, 390.0f, 0, 1.0f, 1.0f, 1, {53, -1, -1, -1}},
        {55, 50.0f, 310.0f, 0, 1.0f, 1.0f, 2, {56, 37, -1, -1}},
        {56, 130.0f, 310.0f, 0, 1.0f, 1.0f, 3, {55, 57, 38, -1}},
        {57, 210.0f, 310.0f, 0, 1.0f, 1.0f, 2, {56, 39, -1, -1}},
        {58, 290.0f, 310.0f, 0, 1.0f, 1.0f, 3, {46, 59, 68, -1}},
        {59, 370.0f, 310.0f, 0, 1.0f, 1.0f, 3, {60, 58, 75, -1}},
        {60, 450.0f, 310.0f, 0, 1.0f, 1.0f, 2, {59, 69, -1, -1}},
        {61, 530.0f, 310.0f, 0, 1.0f, 1.0f, 3, {49, 70, 62, -1}},
        {62, 610.0f, 310.0f, 0, 1.0f, 1.0f, 3, {72, 63, 61, -1}},
        {63, 770.0f, 310.0f, 0, 1.0f, 1.0f, 3, {62, 78, 52, -1}},
        {64, 850.0f, 310.0f, 0, 1.0f, 1.0f, 2, {65, 79, -1, -1}},
        {65, 930.0f, 310.0f, 0, 1.0f, 1.0f, 1, {64, -1, -1, -1}},
        {66, 50.0f, 230.0f, 0, 1.0f, 1.0f, 2, {81, 67, -1, -1}},
        {67, 130.0f, 230.0f, 0, 1.0f, 1.0f, 3, {82, 66, 68, -1}},
        {68, 290.0f, 230.0f, 0, 1.0f, 1.0f, 3, {58, 84, 67, -1}},
        {69, 450.0f, 230.0f, 0, 1.0f, 1.0f, 2, {60, 70, -1, -1}},
        {70, 530.0f, 230.0f, 0, 1.0f, 1.0f, 3, {76, 69, 61, -1}},
        {71, 930.0f, 230.0f, 0, 1.0f, 1.0f, 1, {80, -1, -1, -1}},
        {72, 610.0f, 220.0f, 0, 1.0f, 1.0f, 2, {62, 73, -1, -1}},
        {73, 690.0f, 220.0f, 0, 1.0f, 1.0f, 2, {72, 77, -1, -1}},
        {74, 210.0f, 140.0f, 0, 1.0f, 1.0f, 1, {83, -1, -1, -1}},
        {75, 370.0f, 140.0f, 0, 1.0f, 1.0f, 2, {76, 59, -1, -1}},
        {76, 530.0f, 140.0f, 0, 1.0f, 1.0f, 4, {70, 75, 85, 77}},
        {77, 690.0f, 140.0f, 0, 1.0f, 1.0f, 3, {76, 78, 73, -1}},
        {78, 770.0f, 140.0f, 0, 1.0f, 1.0f, 3, {63, 77, 79, -1}},
        {79, 850.0f, 140.0f, 0, 1.0f, 1.0f, 3, {80, 64, 78, -1}},
        {80, 930.0f, 140.0f, 0, 1.0f, 1.0f, 3, {88, 71, 79, -1}},
        {81, 50.0f, 60.0f, 0, 1.0f, 1.0f, 1, {66, -1, -1, -1}},
        {82, 130.0f, 60.0f, 0, 1.0f, 1.0f, 2, {83, 67, -1, -1}},
        {83, 210.0f, 60.0f, 0, 1.0f, 1.0f, 2, {82, 74, -1, -1}},
        {84, 290.0f, 60.0f, 0, 1.0f, 1.0f, 2, {85, 68, -1, -1}},
        {85, 530.0f, 60.0f, 0, 1.0f, 1.0f, 3, {86, 76, 84, -1}},
        {86, 690.0f, 60.0f, 0, 1.0f, 1.0f, 1, {85, -1, -1, -1}},
        {87, 770.0f, 60.0f, 0, 1.0f, 1.0f, 1, {88, -1, -1, -1}},
        {88, 930.0f, 60.0f, 0, 1.0f, 1.0f, 2, {80, 87, -1, -1}},
    },
    .edges = {{51, 52, 80.0f, 80.0f}, {39, 57, 160.0f, 160.0f}, {44, 53, 80.0f, 80.0f}, {37, 38, 80.0f, 80.0f}, {18, 19, 80.0f, 80.0f}, {78, 79, 80.0f, 80.0f}, {12, 13, 80.0f, 80.0f}, {38, 39, 80.0f, 80.0f}, {79, 80, 80.0f, 80.0f}, {52, 53, 80.0f, 80.0f}, {42, 43, 80.0f, 80.0f}, {48, 49, 80.0f, 80.0f}, {40, 46, 80.0f, 80.0f}, {39, 40, 80.0f, 80.0f}, {61, 62, 80.0f, 80.0f}, {81, 66, 170.0f, 170.0f}, {29, 45, 160.0f, 160.0f}, {78, 63, 170.0f, 170.0f}, {31, 32, 160.0f, 160.0f}, {34, 35, 80.0f, 80.0f}, {23, 31, 80.0f, 80.0f}, {59, 75, 170.0f, 170.0f}, {21, 38, 160.0f, 160.0f}, {3, 4, 160.0f, 160.0f}, {14, 15, 80.0f, 80.0f}, {59, 60, 80.0f, 80.0f}, {42, 51, 80.0f, 80.0f}, {41, 47, 80.0f, 80.0f}, {36, 44, 80.0f, 80.0f}, {13, 4, 90.0f, 90.0f}, {15, 25, 80.0f, 80.0f}, {37, 55, 160.0f, 160.0f}, {62, 63, 160.0f, 160.0f}, {79, 64, 170.0f, 170.0f}, {87, 88, 160.0f, 160.0f}, {85, 76, 80.0f, 80.0f}, {61, 70, 80.0f, 80.0f}, {27, 8, 170.0f, 170.0f}, {84, 85, 240.0f, 240.0f}, {14, 5, 90.0f, 90.0f}, {58, 68, 80.0f, 80.0f}, {83, 74, 80.0f, 80.0f}, {75, 76, 160.0f, 160.0f}, {69, 70, 80.0f, 80.0f}, {82, 67, 170.0f, 170.0f}, {72, 73, 80.0f, 80.0f}, {20, 37, 160.0f, 160.0f}, {19, 9, 90.0f, 90.0f}, {5, 6, 80.0f, 80.0f}, {2, 3, 160.0f, 160.0f}, {78, 77, 80.0f, 80.0f}, {8, 9, 160.0f, 160.0f}, {12, 23, 80.0f, 80.0f}, {17, 7, 90.0f, 90.0f}, {17, 26, 80.0f, 80.0f}, {3, 11, 90.0f, 90.0f}, {18, 28, 80.0f, 80.0f}, {76, 77, 160.0f, 160.0f}, {46, 47, 80.0f, 80.0f}, {84, 68, 170.0f, 170.0f}, {66, 67, 80.0f, 80.0f}, {4, 5, 80.0f, 80.0f}, {44, 45, 80.0f, 80.0f}, {23, 24, 80.0f, 80.0f}, {10, 11, 160.0f, 160.0f}, {34, 50, 160.0f, 160.0f}, {49, 61, 80.0f, 80.0f}, {25, 26, 160.0f, 160.0f}, {14, 32, 160.0f, 160.0f}, {32, 48, 160.0f, 160.0f}, {25, 33, 80.0f, 80.0f}, {60, 69, 80.0f, 80.0f}, {2, 10, 90.0f, 90.0f}, {70, 76, 90.0f, 90.0f}, {67, 68, 160.0f, 160.0f}, {53, 54, 80.0f, 80.0f}, {55, 56, 80.0f, 80.0f}, {82, 83, 80.0f, 80.0f}, {33, 34, 80.0f, 80.0f}, {56, 57, 80.0f, 80.0f}, {16, 17, 80.0f, 80.0f}, {73, 77, 80.0f, 80.0f}, {6, 7, 160.0f, 160.0f}, {46, 58, 80.0f, 80.0f}, {62, 72, 90.0f, 90.0f}, {38, 56, 160.0f, 160.0f}, {35, 36, 160.0f, 160.0f}, {80, 88, 80.0f, 80.0f}, {26, 35, 80.0f, 80.0f}, {11, 12, 80.0f, 80.0f}, {27, 28, 80.0f, 80.0f}, {10, 20, 80.0f, 80.0f}, {85, 86, 160.0f, 160.0f}, {15, 6, 90.0f, 90.0f}, {52, 63, 80.0f, 80.0f}, {21, 22, 80.0f, 80.0f}, {80, 71, 90.0f, 90.0f}, {13, 24, 80.0f, 80.0f}, {11, 22, 80.0f, 80.0f}, {22, 30, 80.0f, 80.0f}, {26, 27, 80.0f, 80.0f}, {64, 65, 80.0f, 80.0f}, {20, 21, 80.0f, 80.0f}, {33, 49, 160.0f, 160.0f}, {58, 59, 80.0f, 80.0f}},
    .node_count = 87,
    .edge_count = 105};

/***************************  工具函数实现  ********************************/
#define SKIP_1 0
#define SKIP_2 1

int get_node_index_by_id(const Map *map, uint8_t node_id)
{
    if (map == NULL)
    {
#if DEBUG_PRINT
        log_print("map->The map pointer is empty\r\n");
#endif
        return -1;
    }

    // 正常遍历查找其他节点
    for (uint8_t i = 0; i < map->node_count; i++)
    {
        if (map->nodes[i].node_id == node_id)
        {
#if DEBUG_PRINT
            log_print("map->node_id:%d neighbors_id:%d\n", map->nodes[i].node_id, node_id);
#endif
            return i;
        }
    }

    // 未找到时：判断是否为跳过的ID
    if (node_id != SKIP_1 && node_id != SKIP_2)
    {
// 非跳过ID，输出未找到日志
#if DEBUG_PRINT
        log_print("map->Node ID not found: %hhu\r\n", node_id);
#endif
    }
    // 无论是否为跳过ID，未找到都返回-1
    return -1;
}

bool is_nodes_connected(const Map *map, uint8_t node1_id, uint8_t node2_id)
{
    if (map == NULL)
    {
#if DEBUG_PRINT
        log_print("map->The map pointer is empty\r\n");
#endif
        return false;
    }
    int node1_idx = get_node_index_by_id(map, node1_id);
    if (node1_idx == -1)
    {
        return false;
    }
    for (uint8_t i = 0; i < map->nodes[node1_idx].neighbor_count; i++)
    {
        if (map->nodes[node1_idx].neighbors[i] == (int8_t)node2_id)
        {
            return true;
        }
    }
    return false;
}

bool is_edge_valid(const Map *map, const Edge *edge)
{
    if (map == NULL || edge == NULL)
    {
#if DEBUG_PRINT
        log_print("map->The map or edge pointer is empty\r\n");
#endif
        return false;
    }
    return (get_node_index_by_id(map, edge->node1) != -1 &&
            get_node_index_by_id(map, edge->node2) != -1);
}

// map_mark_danger_zone函数，基于邻节点标记危险区域
void map_mark_danger_zone(Map *map, uint8_t enemy_node_id)
{
    if (map == NULL)
    {
#if DEBUG_PRINT
        log_print("na->The map pointer is empty in map_mark_danger_zone\r\n");
#endif
        return;
    }

    // 获取敌人所在节点
    Node *enemy_node = get_node_by_id(enemy_node_id);
    if (enemy_node == NULL)
    {
#if DEBUG_PRINT
        log_print("na->Enemy node (ID=%d) not found\r\n", enemy_node_id);
#endif
        return;
    }

    // 标记敌人节点的所有邻节点为危险
    for (uint8_t i = 0; i < enemy_node->neighbor_count; i++)
    {
        int8_t neighbor_id = enemy_node->neighbors[i];
        if (neighbor_id == INVALID_NODE_ID)
            continue; // 跳过无效邻居

        Node *neighbor_node = get_node_by_id(neighbor_id);
        if (neighbor_node)
        {
            neighbor_node->is_dangerous = 1;                                         // 标记为危险节点
            neighbor_node->cost = neighbor_node->base_cost * DANGER_COST_MULTIPLIER; // 增加危险代价
#if DEBUG_PRINT
            log_print("na->Marked neighbor node (ID=%d) as dangerous\r\n", neighbor_id);
#endif
        }
    }

    // 可选：同时标记敌人自身节点为危险
    enemy_node->is_dangerous = 1;
    enemy_node->cost = enemy_node->base_cost * DANGER_COST_MULTIPLIER;
#if DEBUG_PRINT
    log_print("na->Marked enemy node (ID=%d) as dangerous\r\n", enemy_node_id);
#endif
}

bool map_validate(const Map *map)
{
    if (map == NULL || map->node_count == 0 || map->node_count > 100 || map->edge_count > 400)
    {
#if DEBUG_PRINT
        log_print("map->Invalid map: Empty or the number of nodes/edges exceeds the limit \r\n");
#endif
        return false;
    }

    for (uint8_t i = 0; i < map->node_count; i++)
    {
        if (map->nodes[i].node_id >= 100 || map->nodes[i].neighbor_count > 4 ||
            map->nodes[i].base_cost < 0.0f || map->nodes[i].cost < 0.0f)
        {
#if DEBUG_PRINT
            log_print("map->Node invalid: ID=%d, neighbor_count=%d, base_cost=%.2f, cost=%.2f\r\n",
                      map->nodes[i].node_id, map->nodes[i].neighbor_count,
                      map->nodes[i].base_cost, map->nodes[i].cost);
#endif
            return false;
        } // 检验自身的合理性

        for (uint8_t j = 0; j < map->nodes[i].neighbor_count; j++)
        {
            int8_t neighbor_id = map->nodes[i].neighbors[j];
            if (neighbor_id == -1)
                continue;
            if (neighbor_id >= 100 || get_node_index_by_id(map, neighbor_id) < 0)
            {
#if DEBUG_PRINT
                log_print("map->Invalid neighbor ID:%d in node %d\r\n", neighbor_id, map->nodes[i].node_id);
#endif
                return false;
            }
        } // 校验节点邻居的有效性
    }

    for (uint8_t i = 0; i < map->edge_count; i++)
    {
        if (map->edges[i].node1 >= 100 || map->edges[i].node2 >= 100 ||
            get_node_index_by_id(map, map->edges[i].node1) < 0 ||
            get_node_index_by_id(map, map->edges[i].node2) < 0 ||
            map->edges[i].distance < 0.0f || map->edges[i].cost < 0.0f)
        {
#if DEBUG_PRINT
            log_print("map->Invalid edge: %d -> %d, distance=%.2f, cost=%.2f\r\n",
                      map->edges[i].node1, map->edges[i].node2,
                      map->edges[i].distance, map->edges[i].cost);
#endif
            return false;
        }
    }

    return true;
}

void map_get_bounds(const Map *map, float *min_x, float *max_x, float *min_y, float *max_y)
{
    if (map == NULL || map->node_count == 0 || min_x == NULL || max_x == NULL ||
        min_y == NULL || max_y == NULL)
    {
#if DEBUG_PRINT
        log_print("map->The map or pointer is invalid and the boundary cannot be obtained\r\n");
        *min_x = *max_x = *min_y = *max_y = 0.0f;
#endif
        return;
    }

    *min_x = *max_x = map->nodes[0].x;
    *min_y = *max_y = map->nodes[0].y;

    for (uint8_t i = 1; i < map->node_count; i++)
    {
        if (map->nodes[i].x < *min_x)
            *min_x = map->nodes[i].x;
        if (map->nodes[i].x > *max_x)
            *max_x = map->nodes[i].x;
        if (map->nodes[i].y < *min_y)
            *min_y = map->nodes[i].y;
        if (map->nodes[i].y > *max_y)
            *max_y = map->nodes[i].y;
    }
}

// 初始化全局映射表（仅需调用一次）
void map_globalmap_init_id_map(Map *map)
{
    if (map == NULL || g_node_map_inited)
    {
        return; // 已初始化或地图无效，直接返回
    }

    // 最小修复：先把全表置 -1（原 `={-1}` 只初始化了 [0]，其余为 0，
    // 会导致未使用的 id 被误查到索引 0 → 返回错误节点）
    memset(g_node_id_to_idx, 0xFF, sizeof(g_node_id_to_idx));

    // 遍历所有节点，记录node_id对应的索引
    for (uint16_t i = 0; i < map->node_count; i++)
    {
        uint16_t node_id = map->nodes[i].node_id;
        if (node_id < MAX_NODE_ID)
        { // 确保不越界
            g_node_id_to_idx[node_id] = i;
        }
    }

    g_node_map_inited = true; // 标记为已初始化
}

// 新增：初始化边代价索引表
void edge_map_init(Map *map)
{
    if (g_edge_map_inited)
    {
#if DEBUG_PRINT
        log_print("map->The edge mapping of the map has been initialized");
#endif
        return;
    }

    // 初始化所有值为-1.0f（表示无对应边）
    memset(g_edge_cost_map, 0xFF, sizeof(g_edge_cost_map)); // 利用浮点数特性，0xFF对应-1.0f（视平台而定，也可循环赋值）

    // 遍历所有边，填充索引表
    for (uint8_t i = 0; i < global_map.edge_count; i++)
    {
        Edge *edge = &global_map.edges[i];
        uint8_t id1 = edge->node1;
        uint8_t id2 = edge->node2;
        if (id1 < MAX_NODE_ID && id2 < MAX_NODE_ID)
        {
            g_edge_cost_map[id1][id2] = edge->cost;
            g_edge_cost_map[id2][id1] = edge->cost; // 无向边双向映射
        }
    }

    g_edge_map_inited = true;
#if DEBUG_PRINT
    log_print("map->The initialization of the edge index table has been completed\r\n");
#endif
}

// 【核心优化】辅助函数：获取节点指针（通过ID）—— 用映射表替代遍历查找
Node *get_node_by_id(uint8_t node_id)
{
    // 1. 检查映射表是否已初始化（必须优先确保）
    if (!g_node_map_inited)
    {
#if DEBUG_PRINT
        log_print("na->Error: The mapping table is not initialized, and nodes cannot be obtained\n");
#endif
        return NULL;
    }

    // 2. 检查节点ID是否超出映射表范围（避免数组越界）
    if (node_id >= MAX_NODE_ID)
    {
#if DEBUG_PRINT
        log_print("na->Node ID invalid (out of range) :\n");
        log_print("%d\r\n", node_id);
#endif
        return NULL;
    }

    // 3. 从映射表直接获取索引（O(1)操作，替代原有的O(n)遍历）
    int16_t index = g_node_id_to_idx[node_id];

    // 4. 检查索引是否有效（确保在节点数组范围内）
    // 失败路径在导航循环中可能每周期触发，阻塞打印须受 DEBUG_PRINT 控制
    if (index < 0 || index >= global_map.node_count)
    {
#if DEBUG_PRINT
        log_print("na->node is not find %d  Index invalid:%d\r\n", node_id, index);
#endif
        return NULL;
    }

    // 5. 返回节点指针
    return &global_map.nodes[index];
}
