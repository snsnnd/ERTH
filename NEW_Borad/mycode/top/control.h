#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "na.h"

// 机器人动作（输出：需要执行的操作）
typedef enum
{
    ROBOT_ACTION_FORWARD,     // 前进
    ROBOT_ACTION_TURN_LEFT,   // 左转（逆时针，90°）
    ROBOT_ACTION_TURN_RIGHT,  // 右转（顺时针，90°）
    ROBOT_ACTION_TURN_AROUND, // 掉头（180°）
    ROBOT_ACTION_STOP         // 停止
} RobotAction;

/**
 * @brief 将地图方向转换为机器人动作
 * @param map_dir  目标地图方向（MAP_DIR_xxx）
 * @param current_pose 机器人当前朝向角度指针（theta，单位：弧度）
 * @return 机器人需要执行的动作（RobotAction）
 */
int map_dir_to_robot_action(Direction map_dir, float *current_pose);

/**
 * @brief 打印机器人动作（调试用）
 * @param action 机器人动作（RobotAction）
 */
void print_robot_action(RobotAction action);

void forward(void);

void turn_left(int16_t speed);
void turn_right(int16_t speed);
void turn_back(int16_t speed);
#endif
