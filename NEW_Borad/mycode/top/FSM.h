#ifndef FSM_H
#define FSM_H

#include "position.h"
#include "na.h"
#include "sensor.h"

// 机器人状态枚举
typedef enum
{
    ROBOT_IDLE = 0,
    ROBOT_NAVIGATING,
    ROBOT_CALIBRATING,
    ROBOT_ERROR
} RobotState;

typedef struct
{
    volatile uint8_t data_lock; // 简单的数据锁
    volatile uint8_t track_update_flag;
    volatile uint8_t position_update_flag;
    volatile uint8_t lidar_update_flag;
    volatile uint32_t irq_nest_count; // 新增：中断嵌套计数器
} InterruptFlags;

// 机器人主控制结构体
typedef struct
{
    RobotState state;
    LocalizationModule loc_module;
    NavigationMode nav_mode;

    // 目标信息
    float target_x, target_y;
    float enemy_x, enemy_y;

    // 对战协议：对手上报位置（平台经 ORP: 下发的校验后位置，地图坐标）
    float oppo_x, oppo_y;
    uint8_t oppo_valid;      // 1=有效（5s 内收到过 ORP）
    uint32_t oppo_last_ms;   // 最近一次收到 ORP 的时间戳（ms）

    // 对战协议：POS/RPOS 上报节流
    uint32_t last_pos_send_ms; // 上次上报时间戳（ms）
    float last_sent_x, last_sent_y; // 上次上报时的位置（变化阈值判断）
    uint8_t pos_sent_once;     // 是否已上报过至少一次

    Direction current_direction;

    // 状态标志
    uint8_t has_target : 1;
    uint8_t has_enemy : 1;
    uint8_t is_calibrated : 1;

    InterruptFlags irq_flags;
} RobotController;

// 初始化机器人
void robot_init(RobotController *robot, uint8_t init_id, float init_x, float init_y, float init_theta, float target_x, float target_y);
// 主更新函数
void robot_update(RobotController *robot);

// 设置导航目标
void robot_set_target(RobotController *robot, float x, float y, NavigationMode mode);

// 设置敌人位置
void robot_set_enemy(RobotController *robot, float x, float y);

// 获取当前位姿
void robot_get_pose(RobotController *robot, Pose *pose);

// 紧急停止
void robot_emergency_stop(RobotController *robot);

void init(RobotController *robot);
void loop(RobotController *robot);

void lidar_it(RobotController *robot);
void position_it(RobotController *robot);
void hd_it(RobotController *robot);

extern RobotController global_robot;

#endif
