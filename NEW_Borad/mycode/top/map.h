#ifndef MAP_H
#define MAP_H
#include <stdint.h>
#include <stdbool.h>

// 左到右 x变大
// 下到上 y变大
// theta：全局坐标系下的角度(rad)，0为+x，逆时针为正

// 地图节点
typedef struct
{
    uint8_t node_id;        // 节点唯一ID
    float x;                // x坐标 (m)
    float y;                // y坐标 (m)
    uint8_t is_dangerous;   // 是否为危险节点（0-安全，1-危险）
    float base_cost;        // 基础通行代价
    float cost;             // 实际通行代价（base_cost + 危险惩罚）
    uint8_t neighbor_count; // 邻居节点数量（<=4）
    int8_t neighbors[4];    // 邻居节点ID列表（-1表示无邻居）
} Node;

// 节点间的边
typedef struct
{
    uint8_t node1;  // 起始节点ID
    uint8_t node2;  // 目标节点ID
    float distance; // 两点间距离 (m)
    float cost;     // 边的通行代价（distance + 节点危险惩罚）
} Edge;

// 完整地图结构
typedef struct
{
    Node nodes[100];    // 节点数组（最多100个节点）
    Edge edges[400];    // 边数组（最多400条边）
    uint8_t node_count; // 实际节点数量（<=100）
    uint16_t edge_count; // 实际边数量（<=400；原为 uint8_t，无法表示 400）
} Map;

// 全局地图
extern Map global_map;

// 全局映射表：node_id → 该节点在map->nodes数组中的索引（-1表示无效）
#define MAX_NODE_ID 100
extern int16_t g_node_id_to_idx[MAX_NODE_ID];
extern bool g_node_map_inited; // 映射表是否已初始化
// 注：原 float g_edge_cost_map[100][100]（40KB 稠密邻接矩阵）已删——F103C8T6 只有 20KB SRAM，
// 改为下面这个按需线性查询（走 global_map.edges）
extern bool g_edge_map_inited;

// 工具函数声明
int get_node_index_by_id(const Map *map, uint8_t node_id);
bool is_nodes_connected(const Map *map, uint8_t node1_id, uint8_t node2_id);
void map_mark_danger_zone(Map *map, uint8_t center_node_id);
bool is_edge_valid(const Map *map, const Edge *edge);
bool map_validate(const Map *map);
void map_get_bounds(const Map *map, float *min_x, float *max_x, float *min_y, float *max_y);
void map_globalmap_init_id_map(Map *map);
void edge_map_init(Map *map);
bool map_edge_cost(uint8_t id1, uint8_t id2, float *cost); // 查边代价（无向，找不到返回 false）
Node *get_node_by_id(uint8_t node_id);

#endif // MAP_H
