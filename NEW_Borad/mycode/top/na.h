#ifndef NA_H
#define NA_H

#include "map.h"
#include "arm_math.h" // CMSIS-DSP for optimized math
#include "utils.h"    // 用于距离计算等工具

// 最大节点数（与map.h一致）
#define MAX_NODES 100
// 危险节点代价倍增因子（可调，>1.0增加危险节点权重）
#define DANGER_COST_MULTIPLIER 2.0f
// 最大递归深度（防止A*无限递归）
#define MAX_ASTAR_RECURSION 2

// 方向枚举（上、下、左、右，对应四领域地图）
typedef enum
{
    DIRECTION_UP,    // +y 方向
    DIRECTION_DOWN,  // -y 方向
    DIRECTION_LEFT,  // -x 方向
    DIRECTION_RIGHT, // +x 方向
    DIRECTION_STOP   // 已到达目标，停止
} Direction;

// 导航模式枚举（追击、躲避、寻宝）
typedef enum
{
    MODE_PURSUIT, // 追击模式：向敌人位置导航
    MODE_EVADE,   // 躲避模式：远离敌人，寻找安全区域
    MODE_TREASURE // 寻宝模式：向固定目标导航，避开敌人（无敌人则直达）
} NavigationMode;

// 路径节点（用于A*算法内部）
typedef struct
{
    uint8_t node_id;     // 节点ID
    float g_cost;        // 从起点到当前节点的实际代价
    float h_cost;        // 到目标的启发式估计（欧氏距离）
    float f_cost;        // 总代价 (g + h)
    uint8_t parent_id;   // 父节点ID（用于回溯路径）
    uint16_t heap_index; // 堆中的索引（用于二进制堆）
    bool is_goal;        // 是否为目标节点
} PathNode;

// 导航模块接口
bool map_validate(const Map *map);
void map_clear_danger_zone(Map *map);
bool find_closest_node(float x, float y, uint8_t *node_id);
bool select_safe_target(uint8_t enemy_node_id, uint8_t *safe_node_id);
bool navigate(float self_x, float self_y, float enemy_x, float enemy_y, float target_x, float target_y, NavigationMode mode, Direction *direction);

#endif // NAVIGATION_H
