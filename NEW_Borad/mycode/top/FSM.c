#include "FSM.h"
#include "arm_math.h"
#include "Lidra.h"
#include "control.h"
#include <math.h>
#include "BT.h"
#include "Sensor.h"
#include "time.h"
#include "delay.h"
#include "tim.h"
#include "bee.h"

#define M_PI 3.14159265358979323846f
#define MAX_TURN_TIME 5.0f // 最大转向时间（秒），防止异常卡死
#define DEG2RAD 0.0174533f // 角度转弧度（π/180）
#define RAD2DEG 57.2958f   // 弧度转角度（180/π）

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
// 小偷 930.0f, 390.0f, 0.0f    54  地图方向为左 DIRECTION_LEFT 校准点：850.0f, 390.0f
// 警察 50.0f,  60.0f,  -90.0f  81  地图方向为上 DIRECTION_UP   校准点：50.0f,  230.0f
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
    if ((TARGET_X == 0.0f) && (TARGET_Y == 0.0f))
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
    robot->irq_flags.lidar_update_flag = 0;
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

    robot->loc_module.current_calib.x = INIT_CALIB_X;
    robot->loc_module.current_calib.y = INIT_CALIB_Y;

#if DEBUG_PRINT
    log_print("FSM->init_tartget: (%.1f,%.1f)\n", robot->target_x, robot->target_y);
#endif
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

    // 处理雷达更新标志位
    ENTER_CRITICAL();
    uint8_t lidar_flag = robot->irq_flags.lidar_update_flag;
#if DEBUG_PRINT
    log_print("lidar_flag:%d\n", lidar_flag);
#endif
    if (lidar_flag)
    {
        robot->irq_flags.lidar_update_flag = 0;
#if DEBUG_PRINT
        log_print("lidar_update_flag:%n\n", robot->irq_flags.lidar_update_flag);
#endif
    }
    EXIT_CRITICAL();

    if (lidar_flag)
    {
        if (robot->has_enemy)
        {
            float x = real_to_map(robot->enemy_x);
            float y = real_to_map(robot->enemy_y);
            log_print("enemy:(%.1f, %.1f)\n", x, y);
        }
        else
        {
            log_print("enemy:(-1.0, -1.0)\n");
        }
    }

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

    // 状态机处理
    switch (robot->state)
    {
    case ROBOT_IDLE:
#if DEBUG_PRINT
        log_print("FSM->current_idle\n");
#endif

        if (robot->has_enemy)
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

            arm_sqrt_f32(dx * dx + dy * dy, &dist);
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

        log_print("calibrated pos update(%.1f, %.1f)\n",
                  robot->loc_module.current_pose.x, robot->loc_module.current_pose.y);

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

    log_print("FSM->set target: (%.2f, %.2f), mode: %d\r\n", x, y, mode);
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

    log_print("FSM->update enemy_pos: (%.2f, %.2f)\r\n", x, y);
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

    log_print("FSM->emergency_stop\r\n");
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
 *    传感器：循迹，编码器，MPU6050，雷达
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

    // n10_init();

    encoder_setparam(13, 4, 6.5f);
    // 编码器初始化
    encoder_init();
#if DEBUG_PRINT
    log_print("%u,%u,%u,%u\n", encoder_readspeed(encoder_front_left), encoder_readspeed(encoder_back_left), encoder_readspeed(encoder_back_right), encoder_readspeed(encoder_front_right));
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

/*
 *  中断函数，50ms一次，检测敌人位置
 */
void lidar_it(RobotController *robot)
{
    if (robot == NULL)
        return;

    float distance = sensor_get_distance();
    float angle = sensor_get_angle();
    SingleMassPoint temp_mass_point;

    ENTER_CRITICAL(); // 合并临界区范围（仅修改共享数据）
    if (FilterModule_Process(distance, angle, &temp_mass_point) == 0)
    {
        robot->enemy_x = temp_mass_point.centroid_x;
        robot->enemy_y = temp_mass_point.centroid_y;
        robot->has_enemy = 1;
    }
    else
    {
        robot->has_enemy = 0;
    }
    EXIT_CRITICAL();

    robot->irq_flags.lidar_update_flag = 1; // 统一设置标志位
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    { // 判断是否为TIM7中断
#if DEBUG_PRINT
        log_print("IT:10ms\n");
#endif
        // hd_it(&global_robot);
    }

    if (htim->Instance == TIM8)
    {
#if DEBUG_PRINT
        log_print("IT:20ms\n");
#endif
        position_it(&global_robot);
    }

    if (htim->Instance == TIM12)
    { // 判断是否为TIM7中断
#if DEBUG_PRINT
        log_print("50ms\n");
#endif
        // lidar_it(&global_robot);
    }
}
