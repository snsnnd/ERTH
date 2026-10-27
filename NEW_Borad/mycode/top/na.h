#ifndef NA_H
#define NA_H

#include "map.h"
#include "utils.h" // 用于距离计算等工具（arm_math.h 已删：F1 工程无 CMSIS-DSP）

// 最大节点数（与map.h一致）
#define MAX_NODES 100
// 危险节点代价倍增因子（可调，>1.0增加危险节点权重）
#define DANGER_COST_MULTIPLIER 2.0f

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

// ---------------- 对战协议 → 策略层（纯数据接口，零 HAL 依赖） ----------------

// 鲁棒最坏情况决策开关：1=启用（PURSUIT/EVADE 优先走策略区目标，失败回落原逻辑）
// 选手替换策略区实现时改本开关或整体替换 na.c 中标注的策略区
#define STRATEGY_ROBUST_WORST 1

/**
 * @brief  转发平台上报的对手位置（ORP 解析结果）给策略层，由 FSM 主循环调用
 * @param  map_x/map_y: 对手上报位置（地图坐标）
 * @param  valid: 1=有效（FSM 负责 5s 超时清零）
 */
void na_set_opponent_report(float map_x, float map_y, uint8_t valid);

/* ====== 选手策略区 STRATEGY：比赛时可整体替换本区实现 ======
 * 仅依赖 map.c 图数据 API + 标准库（零 HAL 依赖，可用 host gcc 单测）
 * 距离度量：图 BFS 跳数（每条边权重视为 1，不使用边 distance）
 */

/**
 * @brief  谎报位置生成：在真实位置 ±2 图跳内选偏置节点（朝远离对手方向），
 *         结构上保证与真实位置的图距离 ≤2 节点（平台校验必过）
 * @param  true_x/true_y: 真实位置（地图坐标）
 * @param  oppo_x/oppo_y/oppo_valid: 对手上报位置与有效标志
 * @param  spoof_x/spoof_y: 输出谎报位置（失败或无需谎报时=真实位置）
 */
void strategy_spoof_pos(float true_x, float true_y,
                        float oppo_x, float oppo_y, uint8_t oppo_valid,
                        float *spoof_x, float *spoof_y);

/**
 * @brief  鲁棒最坏情况目标选择（minimax/maximin，图 BFS 距离）
 *  PURSUIT: S=对手 ±2 跳候选集；t* = argmin_{t∈S}[dist(我→t) + max_{s∈S}dist(t,s)]
 *  EVADE  : t* = argmax_{t∈候选(全图)} min_{s∈S} dist(t,s)
 * @retval 1=成功（*out_target_node_id 有效）；0=数据无效/无路径 → 回落原 navigate 逻辑
 */
int strategy_select_target(NavigationMode mode,
                           float self_x, float self_y,
                           float oppo_x, float oppo_y, uint8_t oppo_valid,
                           uint8_t *out_target_node_id);
/* ====== 策略区结束 ====== */

#endif // NAVIGATION_H
