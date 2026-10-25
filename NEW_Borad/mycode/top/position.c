#include "position.h"
#include "arm_math.h" // 用于浮点运算加速
#include <math.h>     // 用于NaN检查
#include "map.h"
#include "BT.h"

// 定义全局中断开关宏，适配STM32F4系列
#define ENTER_CRITICAL() __disable_irq() // 关闭全局中断（进入临界区）
#define EXIT_CRITICAL() __enable_irq()   // 开启全局中断（退出临界区）

#define OMEGA_FUSION_ALPHA 0.8f    // omega融合系数（轮子:MPU = alpha:(1-alpha)）
#define DEFAULT_LP_ALPHA 0.7f      // 默认低通滤波系数（如果未指定）
#define DEG_TO_RAD (PI_F / 180.0f) // 度到弧度转换常数

// 角度约束（[-π, π]）
static inline float constrain_angle(float angle)
{
    angle = fmodf(angle, TWO_PI_F);
    if (angle > PI_F)
    {
        angle -= TWO_PI_F;
    }
    else if (angle < -PI_F)
    {
        angle += TWO_PI_F;
    }
    return angle;
}

// 计算两个角度的最小差值（返回[-π, π]）
static float angle_diff(float theta1, float theta2)
{
    return constrain_angle(theta1 - theta2);
}

static bool map_globalmap_find_next_calib(Map *map,
                                          CalibrationPoint last_point,
                                          float current_theta,
                                          float pos_threshold,
                                          CalibrationPoint *next_point)
{
    // 入参校验
    if (map == NULL || next_point == NULL)
    {
        return false;
    }

    // 确保映射表已初始化（首次调用时自动初始化）
    if (!g_node_map_inited)
    {
#if DEBUG_PRINT
        log_print("The map is not mapped\n");
#endif
        map_globalmap_init_id_map(map);
#if DEBUG_PRINT
        log_print("Mapping in progress\n");
#endif
    }

    // -------------------------- 第一步：通过映射表快速找到上一次校准点的节点 --------------------------
    const Node *last_node = NULL;
    if (last_point.node_id < MAX_NODE_ID)
    {
        // 优先通过node_id查找（效率最高，无浮点运算）
        int16_t last_idx = g_node_id_to_idx[last_point.node_id];
        if (last_idx != -1 && last_idx < map->node_count)
        {
            last_node = &map->nodes[last_idx];
        }
    }

    // 若node_id查找失败，降级用坐标匹配（兼容无node_id的场景）
    if (last_node == NULL)
    {
        for (uint16_t i = 0; i < map->node_count; i++)
        {
            if (fabsf(map->nodes[i].x - last_point.x) < 0.01f &&
                fabsf(map->nodes[i].y - last_point.y) < 0.01f)
            {
                last_node = &map->nodes[i];
                break;
            }
        }
    }

    // 未找到上一次校准点对应的节点，返回失败
    if (last_node == NULL)
    {
#if DEBUG_PRINT
        log_print("not find last calibration\n");
#endif
        return false;
    }

    // -------------------------- 第二步：预计算常量（减少重复计算） --------------------------
    const float cos_theta = cosf(current_theta);
    const float sin_theta = sinf(current_theta);
    const float DIR_THRESHOLD = 0.2618f; // 15°弧度
    const float last_x = last_point.x;
    const float last_y = last_point.y;
    const float half_pos_thresh = pos_threshold / 2.0f;

    float best_score = INFINITY;
    CalibrationPoint best_calib;
    bool found = false;

    // -------------------------- 第三步：遍历上一次节点的邻接节点（核心优化） --------------------------
    // 假设节点结构体中邻接ID列表字段为"adj_ids"（原代码中每个节点最后是{10,3,-1,-1}形式）
    const int8_t *adj_ids = last_node->neighbors;
    // 邻接列表长度（原代码中每个节点有4个元素，-1表示无效）
    const uint8_t adj_count = 4;

    for (uint8_t i = 0; i < adj_count; i++)
    {
        int8_t adj_id = adj_ids[i];
        if (adj_id == -1 || adj_id >= MAX_NODE_ID)
        {
            continue; // 跳过无效ID或超出范围的ID
        }

        // 通过全局映射表快速获取邻接节点的索引（O(1)操作，替代循环查找）
        int16_t adj_idx = g_node_id_to_idx[adj_id];
        if (adj_idx == -1 || adj_idx >= map->node_count)
        {
            continue; // 索引无效，跳过
        }
        const Node *adj_node = &map->nodes[adj_idx];

        // 构造候选校准点
        CalibrationPoint candidate;
        candidate.node_id = adj_node->node_id;
        candidate.x = adj_node->x;
        candidate.y = adj_node->y;

        // -------------------------- 第四步：方向与距离约束（保留原逻辑） --------------------------
        const float dx = candidate.x - last_x;
        const float dy = candidate.y - last_y;

        // 1. 距离过近过滤
        const float dist = fast_sqrt(dx * dx + dy * dy);
        if (dist < half_pos_thresh)
        {
            continue;
        }

        // 2. 前方方向过滤（点积判断）
        const float dot_product = dx * cos_theta + dy * sin_theta;
        if (dot_product <= 0.0f)
        { // 后方点跳过
            continue;
        }

        // 3. 方向偏差过滤
        const float candidate_dir = atan2f(dy, dx);
        const float dir_diff = fabsf(angle_diff(candidate_dir, current_theta));
        if (dir_diff > DIR_THRESHOLD)
        {
            continue;
        }

        // 4. 评分选最优
        const float score = dist + (dir_diff * 10.0f);
        if (score < best_score)
        {
            best_score = score;
            best_calib = candidate;
            found = true;
        }
    }

    if (found)
    {
        *next_point = best_calib;
        return true;
    }
    return false;
}

// 初始化定位模块
void localization_init(LocalizationModule *loc,
                       uint8_t init_id, float init_x, float init_y, float init_theta,
                       float track_interval, float lp_alpha)
{
    if (loc == NULL)
        return;

    map_globalmap_init_id_map(&global_map);
#if DEBUG_PRINT
    log_print("pos->The map mapping is completed.\n");
#endif
    edge_map_init(&global_map);
#if DEBUG_PRINT
    log_print("pos->The edge mapping is completed.\n");
#endif

    // 初始化当前位姿
    loc->current_pose.x = init_x;
    loc->current_pose.y = init_y;
    loc->current_pose.theta = constrain_angle(init_theta);
    loc->current_pose.vx = 0.0f;
    loc->current_pose.vy = 0.0f;
    loc->current_pose.omega = 0.0f;
#if DEBUG_PRINT
    log_print("pos->The current pose initialization is complete\n");
#endif

    // 初始化传感器数据
    loc->robot_vel.vx = 0.0f;
    loc->robot_vel.vy = 0.0f;
    loc->robot_vel.omega = 0.0f;
    loc->wheel_speeds.lf_speed = 0.0f;
    loc->wheel_speeds.lb_speed = 0.0f;
    loc->wheel_speeds.rf_speed = 0.0f;
    loc->wheel_speeds.rb_speed = 0.0f;
#if DEBUG_PRINT
    log_print("pos->The sensor data initialization has been completed\n");
#endif

    // 初始化MPU数据
    loc->mpu_data.TRA.acc_x = 0.0f;
    loc->mpu_data.TRA.acc_y = 0.0f;
    loc->mpu_data.TRA.acc_z = 0.0f;
    loc->mpu_data.TRA.gyro_x = 0.0f;
    loc->mpu_data.TRA.gyro_y = 0.0f;
    loc->mpu_data.TRA.gyro_z = 0.0f;
#if DEBUG_PRINT
    log_print("pos->The MPU data initialization has been completed\n");
#endif
    mpu6050_calibrate_gyro(loc->gyro_offset, 5000);
    log_print("mpu6050_calibrate_gyro:%f\n", loc->gyro_offset[2]);

    ComplementaryFilter_Init(&loc->yaw_attitude, loc->gyro_offset);
    loc->yaw_attitude.yaw = init_theta; // 设置初始偏航角
#if DEBUG_PRINT
    log_print("pos->The complementary filter has been initialized successfully\n");
#endif

    // 初始化低通滤波器（速度和角速度）
    float alpha = (lp_alpha > 0.0f && lp_alpha <= 1.0f) ? lp_alpha : DEFAULT_LP_ALPHA;
    low_pass_filter_init(&loc->vx_filter, alpha);
    low_pass_filter_init(&loc->vy_filter, alpha);
    low_pass_filter_init(&loc->omega_filter, alpha);
#if DEBUG_PRINT
    log_print("pos->The initialization of the low-pass filter is complete\n");
#endif

    // 初始化时间和校准状态
    loc->last_update_time = timer_get_ms();
    loc->last_calib_time = 0;
    loc->current_calib.x = 0.0f;
    loc->current_calib.y = 0.0f;
    loc->is_calibrated = 0;
    loc->calib_detected = 0;
    loc->has_last_calib = false;
#if DEBUG_PRINT
    log_print("pos->The initialization time and calibration status have been completed\n");
#endif

#if DEBUG_PRINT
    log_print("pos->The initialization of calibration parameters has been completed\n");
#endif

    // 从globalmap获取第一个校准点（若存在）
    CalibrationPoint first_calib = {init_id, init_x, init_y};
    if (map_globalmap_find_next_calib(&global_map, first_calib, init_theta, 0.3f, &loc->current_calib))
    {
        loc->is_calibrated = 0; // 找到校准点，等待匹配
#if DEBUG_PRINT
        log_print("pos->Find the calibration point: %d:(%d,%d)\n", init_id, init_x, init_y);
#endif
    }
    else
    {
        loc->is_calibrated = 0; // 无校准点，保持未校准状态
#if DEBUG_PRINT
        log_print("pos->The initialization has no calibration points\n");
#endif
    }
}

// 更新定位数据
void localization_update(LocalizationModule *loc)
{
    if (loc == NULL)
        return;
#if DEBUG_PRINT
    log_print("pos->A location update is about to be carried out\n");
#endif

    uint32_t current_time = timer_get_ms();
    float dt = (float)(current_time - loc->last_update_time) / 1000.0f;
    if (dt <= 0.0f)
        return; // 避免dt为0或负
    loc->last_update_time = current_time;
#if DEBUG_PRINT
    log_print("pos->time has update: %ums\n", current_time);
#endif

    // Step 1: 获取传感器数据
    sensor_encoder(&loc->wheel_speeds); // 获取轮速 (m/s)
    sensor_mpu_tra(&loc->mpu_data);
#if DEBUG_PRINT
    log_print("pos->encoder has update\n");
#endif

    // Step 2: 正向运动学计算机器人速度（基于轮速）
    mecanum_forward_kinematics(&loc->wheel_speeds, &loc->robot_vel);
#if DEBUG_PRINT
    log_print("pos->forward_kinematics speed has update\n");
#endif

    // Step 3: 融合角速度（轮子计算的omega与MPU的gyro_z）
    float omega_wheels = loc->robot_vel.omega;
    float omega_gyro = loc->mpu_data.TRA.gyro_z;
    float fused_omega = OMEGA_FUSION_ALPHA * omega_wheels + (1.0f - OMEGA_FUSION_ALPHA) * omega_gyro;
#if DEBUG_PRINT
    log_print("pos->omega has update\n");
#endif

    // Step 4: 低通滤波速度和角速度
    loc->current_pose.vx = low_pass_filter_update(&loc->vx_filter, loc->robot_vel.vx);
    loc->current_pose.vy = low_pass_filter_update(&loc->vy_filter, loc->robot_vel.vy);
    loc->current_pose.omega = low_pass_filter_update(&loc->omega_filter, fused_omega);
#if DEBUG_PRINT
    log_print("pos->low_pass_filter has update\n");
#endif

    // Step 5: 积分更新位姿（使用中点欧拉积分）
    float mid_vx = loc->current_pose.vx;
    float mid_vy = loc->current_pose.vy;
    float mid_omega = loc->current_pose.omega;
#if DEBUG_PRINT
    log_print("pos->Points update poses has update\n");
#endif

    // 计算轮子-based的角度测量
    float delta_theta_wheel = omega_wheels * dt;
    float wheel_based_theta = constrain_angle(loc->current_pose.theta + delta_theta_wheel);

    // 构造互补滤波所需的传感器数据（X右、Y前、Z上）
    SensorData sensor_data = {0};
    // 1. 填充实际加速度计数据（X右、Y前、Z上，单位：m/s²）
    sensor_data.acc_x = loc->mpu_data.TRA.acc_x; // 实际测量的X轴加速度（右为正）
    sensor_data.acc_y = loc->mpu_data.TRA.acc_y; // 实际测量的Y轴加速度（前为正）
    sensor_data.acc_z = loc->mpu_data.TRA.acc_z; // 实际测量的Z轴加速度（上为正，静态时≈9.81）

    // 2. 填充陀螺仪Z轴数据（已减零偏，单位：rad/s；绕Z轴顺时针为正，需与偏航角定义一致）
    sensor_data.gyro_z = omega_gyro - loc->gyro_offset[2];

    // 3. 核心：根据X右、Y前方向，计算加速度计偏航参考角（accel_yaw_ref）
    float accel_yaw_ref;
    // 水平加速度幅值（仅用XY轴，排除Z轴重力影响）
    float horizontal_acc_mag = sqrtf(sensor_data.acc_x * sensor_data.acc_x + sensor_data.acc_y * sensor_data.acc_y);

    // 鲁棒性判断：水平加速度足够大时才更新参考角（避免静态时噪声干扰）
    if (horizontal_acc_mag > 0.1f) // 阈值可根据实际场景调整（如0.05~0.2）
    {
        // 关键：X右、Y前方向下，偏航参考角 = atan2(侧向加速度, 前向加速度)
        // atan2(acc_x, acc_y) 逻辑：
        // - 仅向前运动（acc_y正，acc_x=0）→ atan2(0, 正) = 0° → 偏航参考0°（朝正前，正确）
        // - 仅向右运动（acc_x正，acc_y=0）→ atan2(正, 0) = 90° → 偏航参考90°（朝右，正确）
        // - 仅向左运动（acc_x负，acc_y=0）→ atan2(负, 0) = -90° → 偏航参考-90°（朝左，正确）
        // - 向右前方运动（acc_x正、acc_y正）→ 参考角0°~90°（符合实际朝向）
        accel_yaw_ref = atan2f(sensor_data.acc_x, sensor_data.acc_y);
    }
    else
    {
        // 水平加速度过小时，沿用上次参考角（避免静态噪声导致的参考角抖动）
        static float last_valid_accel_yaw = 0.0f; // 静态变量保存上次有效参考角
        accel_yaw_ref = last_valid_accel_yaw;
        last_valid_accel_yaw = accel_yaw_ref; // 保持参考角稳定
    }

    // 4. 互补滤波融合（陀螺仪高频预测 + 加速度计低频参考）
    float last_yaw = loc->yaw_attitude.yaw; // 保存上一时刻偏航角
    // 陀螺仪积分预测（高频动态：基于角速度更新偏航角）
    float gyro_predicted_yaw = last_yaw + sensor_data.gyro_z * dt;
    // 融合公式：alpha（0.8~0.95）越大，越依赖陀螺仪；越小越依赖加速度计
    loc->yaw_attitude.yaw = COMPLEMENTARY_ALPHA * gyro_predicted_yaw +
                            (1.0f - COMPLEMENTARY_ALPHA) * accel_yaw_ref;

    // 5. 偏航角范围约束（确保在 [-π, π] 内，避免角度溢出）
    loc->yaw_attitude.yaw = fmodf(loc->yaw_attitude.yaw + PI_F, TWO_PI_F) - PI_F;

// 调试打印：验证方向与数值正确性
#if DEBUG_PRINT
    log_print("YAW:%f,1.5708\n", loc->yaw_attitude.yaw);
#endif

    // 位置积分（使用中点theta）
    float mid_theta = constrain_angle(loc->current_pose.theta - (mid_omega * dt / 2.0f));
    float robot_dx = mid_vx * dt;
    float robot_dy = mid_vy * dt;
    float global_dx, global_dy;
    kinematics_robot_to_global(robot_dx, robot_dy, mid_theta, &global_dx, &global_dy);
    loc->current_pose.x += global_dx;
    loc->current_pose.y += global_dy;
#if DEBUG_PRINT
    log_print("pos->position has update x:%.1f y:%.1f\n", loc->current_pose.x, loc->current_pose.y);
#endif
}

// 获取当前位姿
void localization_get_pose(LocalizationModule *loc, Pose *pose)
{
    if (loc == NULL || pose == NULL)
        return;
    pose->x = loc->current_pose.x;
    pose->y = loc->current_pose.y;
    pose->theta = loc->current_pose.theta;
    pose->vx = loc->current_pose.vx;
    pose->vy = loc->current_pose.vy;
    pose->omega = loc->current_pose.omega;
    log_print("POS:%f,%f\nYAW:%f,0.0\nENEMY:300.0,400.0\nPID:0,%f,0.0\nPID:1,%f,0.0,\nPID:2,%f,0.0\nPID:3,%f,0.0\n",
              loc->current_pose.x, loc->current_pose.y, loc->current_pose.theta, loc->wheel_speeds.lf_speed, loc->wheel_speeds.rf_speed, loc->wheel_speeds.lb_speed, loc->wheel_speeds.rb_speed);
}

// float get_yaw_angle(Pose pose)
// {
//     float current_yaw;

//     // 进入临界区：禁用中断，避免读取时被中断更新theta
//     ENTER_CRITICAL();
//     // 读取全局位姿中的偏航角，并确保范围正确（双重保险）
//     current_yaw = constrain_angle(pose.theta);
//     // 退出临界区：恢复中断
//     EXIT_CRITICAL();

//     return current_yaw;
// }
