#include "FSM.h"
// arm_math.h 已删：STM32F103C8T6（Cortex-M3，无 FPU）工程不带 CMSIS-DSP，改用 math.h 的 sqrtf
// Lidra.h（雷达点云聚类）已随雷达模块删除：敌人坐标改由对战平台 ORP 下发
#include "control.h"
#include <math.h>
#include <string.h> // strncmp：对战协议下行解析
#include <stdlib.h> // strtof：对战协议下行解析
#include <stdio.h>  // snprintf：对战协议上行组包
#include "BT.h"
#include "sensor.h" // 原 "Sensor.h" 与磁盘文件名 sensor.h 大小写不符（Linux 构建失败）
#include "time.h"
#include "delay.h"
#include "tim.h"
#include "motor.h" // motor_init：电机 PWM 通道初始化（HAL_TIM_PWM_Start）

// 注：原 #define M_PI 已删除——本文件无 M_PI 使用，且 math.h（POSIX/newlib）
// 可能已定义 M_PI（double 版本），重复定义会触发重定义冲突
#define MAX_TURN_TIME 5.0f // 最大转向时间（秒），防止异常卡死
#define INIT_X 930.0f
#define INIT_Y 390.0f
#define INIT_T 0.0f
#define INIT_DIRECTION DIRECTION_LEFT
#define TARGET_X 450.0f
#define TARGET_Y 800.0f
#define INIT_ID 54
#define INIT_CALIB_X 850.0f
#define INIT_CALIB_Y 390.0f

#define TRACK_INTERVAL 2.0 // 循迹传感器间隔
#define LP_ALPHA 0.7

// ---------------- 对战协议（与上位机 final.py 对齐） ----------------
#define POS_SEND_MIN_MS 100       // POS/RPOS 最小发送间隔（ms）
#define POS_SEND_MAX_MS 200       // 位置无明显变化时的最大发送间隔（ms）
#define POS_CHANGE_THRESHOLD 20.0f // 位置变化阈值（地图单位，超过则提前发送）
#define OPPONENT_TIMEOUT_MS 5000  // ORP 超过 5s 未更新则失效
// 小偷 930.0f, 390.0f, 0.0f    54  地图方向为左 DIRECTION_LEFT 校准点：850.0f, 390.0f
// 警察 50.0f,  60.0f,  -1.5708f 81  地图方向为上 DIRECTION_UP   校准点：50.0f,  230.0f（初始角为弧度：-90°=-π/2=-1.5708）
// 宝藏 450.0f, 800.0f          5

#define ENTER_CRITICAL()                          \
    do                                            \
    {                                             \
        robot->irq_flags.data_lock = 1;           \
        if (robot->irq_flags.irq_nest_count == 0) \
        {                                         \
            __disable_irq();                      \
        }                                         \
        robot->irq_flags.irq_nest_count++;        \
    } while (0)

#define EXIT_CRITICAL()                           \
    do                                            \
    {                                             \
        robot->irq_flags.irq_nest_count--;        \
        if (robot->irq_flags.irq_nest_count == 0) \
        {                                         \
            __enable_irq();                       \
        }                                         \
        robot->irq_flags.data_lock = 0;           \
    } while (0)

RobotController global_robot;

// 运动控制
static void execute_movement(Direction dir, RobotController *robot)
{
    // 1. 传递current_pose.theta的地址（函数需要float*类型）
    // 2. 补充分号（语句结束标志）
    int ret = map_dir_to_robot_action(dir, &robot->loc_module.current_pose.theta);
#if DEBUG_PRINT
    // 可选：处理返回值，判断转换是否成功
    if (ret != 1)
    {
        log_print("execute_movement: dir transform\r\n");
    }
    else
    {
        log_print("excute_movement success\n");
    }
#endif
}

void robot_init(RobotController *robot, uint8_t init_id, float init_x, float init_y, float init_theta, float target_x, float target_y)
{
    if (robot == NULL)
        return;

    // 初始化状态
    robot->state = ROBOT_NAVIGATING;
    robot->nav_mode = MODE_TREASURE;
    if ((target_x == 0.0f) && (target_y == 0.0f))
    {
        robot->has_target = 0;
    }
    else
    {
        robot->has_target = 1;
    }

    robot->has_enemy = 0;
    robot->is_calibrated = 0;
    robot->current_direction = INIT_DIRECTION;
#if DEBUG_PRINT
    log_print("FSM->The robot's status initialization has been completed\n");
#endif

    // 初始化中断标志位和锁
    robot->irq_flags.track_update_flag = 0;
    robot->irq_flags.position_update_flag = 0;
    robot->irq_flags.data_lock = 0;
    robot->irq_flags.irq_nest_count = 0;

    // 初始化定位模块
    localization_init(&robot->loc_module, init_id, init_x, init_y, init_theta,
                      TRACK_INTERVAL, // 循迹传感器间隔2cm
                      LP_ALPHA);      // 低通滤波系数0.7
#if DEBUG_PRINT
    log_print("init_pos: (%.1f,%.1f)\n", init_x, init_y);
    log_print("FSM->The positioning initialization is completed\n");
    log_print("loc_init_pos: (%.1f,%.1f)\n", robot->loc_module.current_pose.x, robot->loc_module.current_pose.y);
#endif

    // 初始化目标位置
    robot->target_x = target_x;
    robot->target_y = target_y;
    robot->enemy_x = -1.0f; // 无效值
    robot->enemy_y = -1.0f;

    // 对战协议字段初始化
    robot->oppo_x = -1.0f;
    robot->oppo_y = -1.0f;
    robot->oppo_valid = 0;
    robot->oppo_last_ms = 0;
    robot->last_pos_send_ms = 0;
    robot->last_sent_x = init_x;
    robot->last_sent_y = init_y;
    robot->pos_sent_once = 0;

    // 校准点兜底：localization_init 已通过 map_globalmap_find_next_calib
    // 查找地图校准点，找到时 current_calib 即为该点；仅当未找到（保持
    // localization_init 写入的初始值 (0,0)，地图中不存在 (0,0) 节点，可作
    // “未找到”哨兵）时才用宏值兜底，避免覆盖找到的校准点
    if (robot->loc_module.current_calib.x == 0.0f &&
        robot->loc_module.current_calib.y == 0.0f)
    {
        robot->loc_module.current_calib.x = INIT_CALIB_X;
        robot->loc_module.current_calib.y = INIT_CALIB_Y;
    }

#if DEBUG_PRINT
    log_print("FSM->init_tartget: (%.1f,%.1f)\n", robot->target_x, robot->target_y);
#endif
}

/*
 * 对战协议接入（主循环上下文，非中断）：
 *  1) 轮询蓝牙下行整行，解析 ORP:（对手上报位置）/ ERR:RPOS,（上报被拒日志）
 *  2) ORP 超过 OPPONENT_TIMEOUT_MS(5s) 未更新 → oppo_valid = 0
 *  3) 将对手位置转发给 na.c 策略层（供鲁棒最坏情况目标选择）
 *  4) 按 100~200ms / 位置变化阈值节流发送 POS:（真实位置）与 RPOS:（策略谎报位置）
 * 注意：本函数内的发送均为阻塞发送，禁止移入任何临界区/中断
 */
static void robot_protocol_update(RobotController *robot, const Pose *pose)
{
    char line[BT_LINE_MAX];

    // ---- 1) 下行：取整行并解析（ORP / ERR:RPOS） ----
    while (bt_read_line(line, (int)sizeof(line)) > 0)
    {
        if (strncmp(line, "ORP:", 4) == 0)
        {
            char *end1 = NULL;
            float ox = strtof(line + 4, &end1);
            if (end1 != (line + 4) && *end1 == ',')
            {
                char *end2 = NULL;
                float oy = strtof(end1 + 1, &end2);
                if (end2 != (end1 + 1))
                {
                    ENTER_CRITICAL();
                    robot->oppo_x = ox;
                    robot->oppo_y = oy;
                    robot->oppo_valid = 1;
                    robot->oppo_last_ms = timer_get_ms();
                    EXIT_CRITICAL();
                    // ORP 接管敌人坐标（原雷达点云聚类已删）：
                    // ORP 为地图坐标，enemy_x/y 语义为真实（米）坐标
                    // （na.c find_closest_node_enemy 内做 real_to_map），
                    // 故经 map_to_real 换算后写入（内部含临界区）
                    robot_set_enemy(robot, map_to_real(ox), map_to_real(oy));
#if DEBUG_PRINT
                    log_print("FSM->ORP: (%.1f,%.1f)\r\n", ox, oy);
#endif
                    continue;
                }
            }
            log_print("FSM->bad ORP line: %s\r\n", line);
        }
        else if (strncmp(line, "ERR:RPOS,", 9) == 0)
        {
            // 平台拒绝本次 RPOS（仅日志）
            log_print("FSM->platform rejected RPOS: %s\r\n", line);
        }
        // 其余下行（含遥测回显等）一律忽略
    }

    // ---- 2) ORP 超时失效 ----
    uint32_t now = timer_get_ms();
    ENTER_CRITICAL();
    if (robot->oppo_valid &&
        GetTickDiffMs(robot->oppo_last_ms, now) > OPPONENT_TIMEOUT_MS)
    {
        robot->oppo_valid = 0;
        robot->has_enemy = 0; // 对手位置过期 → 敌人同步失效（原雷达的 has_enemy 归属）
        robot->enemy_x = -1.0f;
        robot->enemy_y = -1.0f;
    }
    float oppo_x = robot->oppo_x;
    float oppo_y = robot->oppo_y;
    uint8_t oppo_valid = robot->oppo_valid;
    EXIT_CRITICAL();

    // ---- 3) 转发给策略层（纯数据赋值，供 navigate() 内鲁棒决策使用） ----
    na_set_opponent_report(oppo_x, oppo_y, oppo_valid);

    // ---- 4) 节流发送 POS / RPOS ----
    now = timer_get_ms();
    float dx = pose->x - robot->last_sent_x;
    float dy = pose->y - robot->last_sent_y;
    float moved = sqrtf(dx * dx + dy * dy);
    uint32_t elapsed = GetTickDiffMs(robot->last_pos_send_ms, now);
    if (!robot->pos_sent_once ||
        (elapsed >= POS_SEND_MIN_MS &&
         (moved >= POS_CHANGE_THRESHOLD || elapsed >= POS_SEND_MAX_MS)))
    {
        float spoof_x = pose->x;
        float spoof_y = pose->y;
        strategy_spoof_pos(pose->x, pose->y, oppo_x, oppo_y, oppo_valid,
                           &spoof_x, &spoof_y);

        char buf[48];
        snprintf(buf, sizeof(buf), "POS:%.1f,%.1f", pose->x, pose->y);
        bt_send_line(buf);
        snprintf(buf, sizeof(buf), "RPOS:%.1f,%.1f", spoof_x, spoof_y);
        bt_send_line(buf);

        // 遥测：PID（四轮实际轮速，供上位机 PIDWidget）与 ENEMY（真实探测
        // 敌人坐标，地图单位）。原 localization_get_pose 内硬编码上报
        // （含假数据 ENEMY:300.0,400.0）已删除，统一并入本节流块：
        // 共享数据（轮速/敌人坐标由 ISR 更新）在临界区中复制后再组包发送，
        // 严禁在临界区内调用阻塞的 bt_send_line
        ENCODER_SensorTypeDef wheels;
        float enemy_x, enemy_y;
        uint8_t has_enemy;
        ENTER_CRITICAL();
        wheels = robot->loc_module.wheel_speeds;
        enemy_x = robot->enemy_x;
        enemy_y = robot->enemy_y;
        has_enemy = robot->has_enemy;
        EXIT_CRITICAL();

        snprintf(buf, sizeof(buf), "PID:0,%.3f,0.0", (double)wheels.lf_speed);
        bt_send_line(buf);
        snprintf(buf, sizeof(buf), "PID:1,%.3f,0.0", (double)wheels.rf_speed);
        bt_send_line(buf);
        snprintf(buf, sizeof(buf), "PID:2,%.3f,0.0", (double)wheels.lb_speed);
        bt_send_line(buf);
        snprintf(buf, sizeof(buf), "PID:3,%.3f,0.0", (double)wheels.rb_speed);
        bt_send_line(buf);
        if (has_enemy)
        {
            snprintf(buf, sizeof(buf), "ENEMY:%.1f,%.1f",
                     (double)real_to_map(enemy_x), (double)real_to_map(enemy_y));
            bt_send_line(buf);
        }

        robot->last_pos_send_ms = now;
        robot->last_sent_x = pose->x;
        robot->last_sent_y = pose->y;
        robot->pos_sent_once = 1;
    }
}

void robot_update(RobotController *robot)
{
    if (robot == NULL || robot->state == ROBOT_ERROR)
        return;
    Direction last_direction = robot->current_direction;

    // 处理定位更新标志位
    ENTER_CRITICAL();
    uint8_t pos_flag = robot->irq_flags.position_update_flag;
    if (pos_flag)
    {
        robot->irq_flags.position_update_flag = 0;
    }
    EXIT_CRITICAL();
    if (pos_flag)
    {
#if DEBUG_PRINT
        log_print("FSM->position updated\n");
#endif
    }

    // 雷达更新标志段已删除（雷达模块舍弃；敌人坐标改由对战协议 ORP 驱动，
    // 见 robot_protocol_update 中 ORP 解析 → robot_set_enemy）

    // 处理循迹数据标志位
    ENTER_CRITICAL();
    uint8_t track_flag = robot->irq_flags.track_update_flag;
#if DEBUG_PRINT
    log_print("\n");
#endif
    if (track_flag)
    {
        robot->irq_flags.track_update_flag = 0;
#if DEBUG_PRINT
        log_print("\n");
#endif
    }
    EXIT_CRITICAL();
    if (track_flag)
    {
        Track_SceneTypeDef scene;
        Track_SensorTypeDef temp_track_data;
        ENTER_CRITICAL();
        temp_track_data = robot->loc_module.track_data; // 仅复制
        EXIT_CRITICAL();
        scene = sensor_hd_judge_scene(&temp_track_data);

        if (scene == TRACK_CHECK_POINT)
        {
#if DEBUG_PRINT
            log_print("FSM->find checkpoint, update position\n");
#endif
            ENTER_CRITICAL();
            robot->state = ROBOT_CALIBRATING;
            EXIT_CRITICAL();
        }
    }

    // 在 robot_update 函数中读取当前位姿的位置（已处理，补充注释说明）
    Pose current_pose;
    ENTER_CRITICAL();
    // 临界区中读取定位数据，防止 position_it 中断同时修改
    localization_get_pose(&robot->loc_module, &current_pose);
    uint8_t is_calibrated = robot->loc_module.is_calibrated;
#if DEBUG_PRINT
    log_print("FSM->pose update\n");
#endif
    EXIT_CRITICAL();

    robot->is_calibrated = is_calibrated;

#if DEBUG_PRINT
    log_print("FSM->calibrated_state:%d\n", robot->is_calibrated);
#endif

    // 对战协议接入：解析 ORP/ERR、ORP 超时、节流上报 POS/RPOS
    // 并把对手位置转发给策略层（主循环上下文，非中断）
    robot_protocol_update(robot, &current_pose);

    // 状态机处理
    switch (robot->state)
    {
    case ROBOT_IDLE:
#if DEBUG_PRINT
        log_print("FSM->current_idle\n");
#endif

        // 有对手或有目标（任一为真）即转入导航
        if (robot->has_enemy || robot->has_target)
        {
#if DEBUG_PRINT
            log_print("FSM->has_enemy\n");
#endif

            ENTER_CRITICAL();
            robot->state = ROBOT_NAVIGATING;
            EXIT_CRITICAL();
        }
        break;

    case ROBOT_NAVIGATING:
#if DEBUG_PRINT
        log_print("FSM->current_nav\n");
#endif

        if (robot->has_target)
        {
            float current_target_x, current_target_y;
            float current_enemy_x, current_enemy_y;
            uint8_t has_current_enemy;
            NavigationMode current_nav_mode;

            // 原子读取所有导航相关数据
            ENTER_CRITICAL();
            current_target_x = robot->target_x;
            current_target_y = robot->target_y;
            current_enemy_x = robot->enemy_x;
            current_enemy_y = robot->enemy_y;
            has_current_enemy = robot->has_enemy;
            current_nav_mode = robot->nav_mode;
            EXIT_CRITICAL();

            // 执行导航计算
            bool nav_success = navigate(
                current_pose.x, current_pose.y,
                has_current_enemy ? current_enemy_x : -1.0f,
                has_current_enemy ? current_enemy_y : -1.0f,
                current_target_x, current_target_y,
                current_nav_mode,
                &robot->current_direction);

            if (!nav_success)
            {
#if DEBUG_PRINT
                log_print("FSM->nav failed, change to idle\r\n");
#endif

                ENTER_CRITICAL();
                robot->state = ROBOT_IDLE;
                robot->current_direction = DIRECTION_STOP;
                EXIT_CRITICAL();
            }

#if DEBUG_PRINT
            log_print("FSM->nav success\n");
#endif

            // 检查是否到达目标（原子读取目标位置）
            float dx, dy, dist;
            ENTER_CRITICAL();
            dx = robot->target_x - current_pose.x;
            dy = robot->target_y - current_pose.y;
            EXIT_CRITICAL();

            dist = sqrtf(dx * dx + dy * dy);
            dist = map_to_real(dist);

            if (dist < 0.01f)
            {
#if DEBUG_PRINT
                log_print("FSM->Reach the target position\r\n");
#endif

                ENTER_CRITICAL();
                robot->state = ROBOT_IDLE;
                robot->current_direction = DIRECTION_STOP;
                robot->has_target = 0;
                robot->nav_mode = MODE_EVADE;
                EXIT_CRITICAL();

#if DEBUG_PRINT
                log_print("FSM->change to evade mode\r\n");
#endif
            }
        }
        else
        {
            ENTER_CRITICAL();
            robot->state = ROBOT_IDLE;
            EXIT_CRITICAL();
        }
        break;

    case ROBOT_CALIBRATING:
#if DEBUG_PRINT
        log_print("FSM->current_state: calibrating\n");
#endif
        // 直接更新定位模块的坐标为预设校准点坐标
        ENTER_CRITICAL();
        // 使用当前
        robot->loc_module.current_pose.x = robot->loc_module.current_calib.x;
        robot->loc_module.current_pose.y = robot->loc_module.current_calib.y;
        // 同时更新校准标志
        robot->loc_module.is_calibrated = 1;
        robot->is_calibrated = 1;
        EXIT_CRITICAL();

#if DEBUG_PRINT
        log_print("calibrated pos update(%.1f, %.1f)\n",
                  robot->loc_module.current_pose.x, robot->loc_module.current_pose.y);
#endif

        // 校准状态保持停止，等待外部触发进入导航
        ENTER_CRITICAL();
        robot->current_direction = DIRECTION_STOP;
        robot->state = ROBOT_NAVIGATING;
        EXIT_CRITICAL();
        break;

    case ROBOT_ERROR:
#if DEBUG_PRINT
        // 错误状态，停止所有动作
        log_print("FSM->state_error\n");
#endif

        robot->current_direction = DIRECTION_STOP;
        break;
    }

    // 在状态机处理结束后，如果没有新方向则使用上一次方向：
    Direction current_dir;
    ENTER_CRITICAL();
    current_dir = robot->current_direction;
    // 如果当前方向是停止且是空闲状态，则使用上一次方向
    if (current_dir == DIRECTION_STOP && robot->state == ROBOT_IDLE)
    {
        current_dir = last_direction;
#if DEBUG_PRINT
        log_print("use last dir\n");
#endif
    }
    EXIT_CRITICAL();

#if DEBUG_PRINT
    log_print("current_dir:%d\nstart move\n", current_dir);
#endif
    // 执行运动控制
    execute_movement(current_dir, robot);
}

void robot_set_target(RobotController *robot, float x, float y, NavigationMode mode)
{
    if (robot == NULL)
        return;

    ENTER_CRITICAL();
    robot->target_x = x;
    robot->target_y = y;
    robot->nav_mode = mode;
    robot->has_target = 1;
    robot->state = ROBOT_NAVIGATING;
    EXIT_CRITICAL();

#if DEBUG_PRINT
    log_print("FSM->set target: (%.2f, %.2f), mode: %d\r\n", x, y, mode);
#endif
}

void robot_set_enemy(RobotController *robot, float x, float y)
{
    if (robot == NULL)
        return;

    ENTER_CRITICAL();
    robot->enemy_x = x;
    robot->enemy_y = y;
    robot->has_enemy = (x >= 0 && y >= 0);
    EXIT_CRITICAL();

#if DEBUG_PRINT
    log_print("FSM->update enemy_pos: (%.2f, %.2f)\r\n", x, y);
#endif
}

void robot_emergency_stop(RobotController *robot)
{
    if (robot == NULL)
        return;

    ENTER_CRITICAL();
    robot->state = ROBOT_IDLE;
    robot->current_direction = DIRECTION_STOP;
    robot->has_target = 0;
    EXIT_CRITICAL();

#if DEBUG_PRINT
    log_print("FSM->emergency_stop\r\n");
#endif
    execute_movement(DIRECTION_STOP, robot);
}

void robot_get_pose(RobotController *robot, Pose *pose)
{
    if (robot == NULL || pose == NULL)
        return;

    // 保护位置数据的读取
    ENTER_CRITICAL();
    localization_get_pose(&robot->loc_module, pose);
    EXIT_CRITICAL();
}

/*
 *初始化函数:
 *    蓝牙
 *    传感器：循迹，编码器，MPU6050（雷达已随硬件方案舍弃）
 *    工具：滤波器，地图，导航，定位
 */
void init(RobotController *robot)
{
    bt_init();

    // 初始化MPU6050
    uint8_t mpu_state = mpu6050_init();
    if (mpu_state != 0)
    {
        while (1)
        {
            log_print("MPU6050 init failed!\r\n");
        } // 初始化失败则卡死
    }
    else
    {
        log_print("MPU6050 init success!\r\n");
    }

    // （原 n10_init()：雷达 UART 接收中断启动——雷达模块已舍弃，删除）

    // 编码器初始化（encoder_init 内部已调用 encoder_setparam(13,4,6.5f)，
    // 此处原先重复调用一次，属死重复，已删除）
    encoder_init();
    // 电机 PWM 通道初始化（HAL_TIM_PWM_Start）：正常启动路径此前从未开启
    // PWM，只在 test_motor() 里调过 motor_init()，属 P0 缺陷
    motor_init();
    // TIM1 基准（10ms）启动：hd_it/position_it 的周期回调挂在 TIM1 上
    // （原方案 TIM7/TIM8 在 F103C8 上不存在，已合并为 TIM1 + 二分频）
    HAL_TIM_Base_Start_IT(&htim1);
#if DEBUG_PRINT
    // encoder_readspeed 返回 float 且带“读取即清零计数器”副作用：
    // 先各存入局部变量，再用 %f 打印（原 %u 为未定义行为）
    float sp_fl = encoder_readspeed(encoder_front_left);
    float sp_bl = encoder_readspeed(encoder_back_left);
    float sp_br = encoder_readspeed(encoder_back_right);
    float sp_fr = encoder_readspeed(encoder_front_right);
    log_print("%f,%f,%f,%f\n", sp_fl, sp_bl, sp_br, sp_fr);
#endif
    robot_init(robot, INIT_ID, INIT_X, INIT_Y, INIT_T, TARGET_X, TARGET_Y);
}

/*
 *主循环函数
 *  状态机: 空闲，导航，校准，错误
 *  读取灰度->定位更新->获取姿态->检查校准->状态机->执行动作
 */
void loop(RobotController *robot)
{
    robot_update(robot);
#if DEBUG_PRINT
    log_print("1 loop end\n");
#endif
}

/*
 *  中断函数，10ms一次，循迹检测
 */
void hd_it(RobotController *robot)
{
    if (robot == NULL)
        return;

    // 1. 先在临界区外读取传感器数据到临时变量（耗时操作移出）
    Track_SensorTypeDef temp_track_data;
    sensor_hd_read_digital(&temp_track_data);

    // 2. 仅在临界区中复制数据（快速操作）
    ENTER_CRITICAL();
    robot->loc_module.track_data = temp_track_data;
    EXIT_CRITICAL();

    robot->irq_flags.track_update_flag = 1;
}

/*
 *  中断函数，20ms一次，更新位置
 */
void position_it(RobotController *robot)
{
    if (robot == NULL)
        return;

    // 位置更新可能涉及多个变量的修改，需要保护
    ENTER_CRITICAL();
    localization_update(&robot->loc_module);
    EXIT_CRITICAL();

    robot->irq_flags.position_update_flag = 1;
}

// lidar_it() 已删除：雷达模块（N10/USART5 + 点云聚类）随硬件方案舍弃，
// 敌人坐标统一由对战平台 ORP 下发（robot_protocol_update → robot_set_enemy）

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 实际硬件（STM32F103C8T6）只有 TIM1~TIM4：TIM2/TIM4 编码器、TIM3 PWM，
    // 故周期任务统一挂 TIM1（10ms），20ms 定位节拍由 10ms 基准二分频得到
    // （原方案 TIM7=循迹 10ms / TIM8=定位 20ms 在 F103C8 上不存在）
    if (htim->Instance == TIM1)
    {
        static uint8_t s_tim1_tick; // 10ms 计数，二分频出 20ms

#if DEBUG_PRINT
        log_print("IT:10ms\n");
#endif
        // 注：TIM1 的启动在 init() 内 HAL_TIM_Base_Start_IT 完成；
        // 若外部未启动该定时器，本调用自然不会触发
        hd_it(&global_robot);

        if (++s_tim1_tick >= 2)
        {
            s_tim1_tick = 0;
#if DEBUG_PRINT
            log_print("IT:20ms\n");
#endif
            position_it(&global_robot);
        }
    }
    // TIM12（原 50ms 雷达中断）分支已删除：雷达模块舍弃后不再需要
}
