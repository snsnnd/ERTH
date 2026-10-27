#include "position.h"
#include <math.h> // NaN 检查 + sqrtf（arm_math.h 已删：F1 工程无 CMSIS-DSP）
#include "map.h"
#include "BT.h"

// 定义全局中断开关宏（Cortex-M3 通用 __disable_irq/__enable_irq）
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
        //    用精确 sqrtf 而不是 fast_sqrt：这里要跟 0.15 地图单位的阈值比较，
        //    Quake 快速算法的 ~0.17% 误差在阈值附近会改变判定结果。
        const float dist = sqrtf(dx * dx + dy * dy);
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

    (void)track_interval; // 保留接口参数：本版定位按地图节点做航位推算，不使用循迹间距
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
    loc->gyro_offset[0] = 0.0f;
    loc->gyro_offset[1] = 0.0f;
    loc->gyro_offset[2] = 0.0f;
#if DEBUG_PRINT
    log_print("pos->The MPU data initialization has been completed\n");
#endif
    // 陀螺零偏标定：取 1000 次 1ms 采样（约 1s）足够压住零偏，原 5000 次要白等约 5s
    if (!mpu6050_calibrate_gyro(loc->gyro_offset, 1000))
    {
#if DEBUG_PRINT
        log_print("pos->gyro calibration failed; using zero offset\n");
#endif
    }
#if DEBUG_PRINT
    log_print("mpu6050_calibrate_gyro:%f\n", loc->gyro_offset[2]);
#endif

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
    loc->current_calib.node_id = UINT8_MAX;
    loc->current_calib.x = 0.0f;
    loc->current_calib.y = 0.0f;
    loc->is_calibrated = 0;
    loc->calib_detected = 0;
    loc->has_current_calib = false;
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
        loc->has_current_calib = true;
        loc->is_calibrated = 0; // 找到校准点，等待匹配
#if DEBUG_PRINT
        log_print("pos->Find the calibration point: %u:(%.1f,%.1f)\n", init_id, init_x, init_y);
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
#if DEBUG_PRINT_VERBOSE
    log_print("pos->A location update is about to be carried out\n");
#endif

    uint32_t current_time = timer_get_ms();
    float dt = (float)(current_time - loc->last_update_time) / 1000.0f;
    if (dt <= 0.0f)
        return; // 避免dt为0或负

    // 先确认本次 MPU 采样完整，再消费编码器计数并推进时间戳。
    // 通信失败时保留上一份有效数据，本周期不积分。
    if (!sensor_mpu_tra(&loc->mpu_data))
    {
        return;
    }

    sensor_encoder(&loc->wheel_speeds); // 获取轮速 (m/s)

    // 上限保护：中断被长时间阻塞（例如串口/基准节拍被打断）后，单步积分不能吃掉几秒，
    // 否则一次跳变能把整局定位带飞。20ms 标称节拍，100ms 已是 5 倍余量。
    if (dt > 0.1f)
    {
        dt = 0.1f;
    }
    loc->last_update_time = current_time;
#if DEBUG_PRINT_VERBOSE
    log_print("pos->time has update: %ums\n", current_time);
#endif

    // Step 1: 传感器数据已在上方成功采集
#if DEBUG_PRINT_VERBOSE
    log_print("pos->encoder has update\n");
#endif

    // Step 2: 正向运动学计算机器人速度（基于轮速）
    mecanum_forward_kinematics(&loc->wheel_speeds, &loc->robot_vel);
#if DEBUG_PRINT_VERBOSE
    log_print("pos->forward_kinematics speed has update\n");
#endif

    // Step 3: 融合角速度（轮子计算的omega与MPU的gyro_z）
    float omega_wheels = loc->robot_vel.omega;
    // 扣零偏：位姿角速度（参与位置积分）与偏航角必须用同一路已校正的陀螺值。
    // 原实现只给 yaw 分支扣了 gyro_offset[2]，融合进 pose.omega 的是带零偏的原始值，
    // 零偏会被积分进位置（km/h 级的慢漂移就是这样来的）。
    float omega_gyro = loc->mpu_data.TRA.gyro_z - loc->gyro_offset[2];
    float fused_omega = OMEGA_FUSION_ALPHA * omega_wheels + (1.0f - OMEGA_FUSION_ALPHA) * omega_gyro;
#if DEBUG_PRINT_VERBOSE
    log_print("pos->omega has update\n");
#endif

    // Step 4: 低通滤波速度和角速度
    loc->current_pose.vx = low_pass_filter_update(&loc->vx_filter, loc->robot_vel.vx);
    loc->current_pose.vy = low_pass_filter_update(&loc->vy_filter, loc->robot_vel.vy);
    loc->current_pose.omega = low_pass_filter_update(&loc->omega_filter, fused_omega);
#if DEBUG_PRINT_VERBOSE
    log_print("pos->low_pass_filter has update\n");
#endif

    // Step 5: 积分更新位姿（使用中点欧拉积分）
    float mid_vx = loc->current_pose.vx;
    float mid_vy = loc->current_pose.vy;
    float mid_omega = loc->current_pose.omega;
#if DEBUG_PRINT_VERBOSE
    log_print("pos->Points update poses has update\n");
#endif

    // 平面车没有可观测 yaw 的绝对参考；用滤波后的轮速/陀螺融合角速度积分，
    // 再由地图校准点纠正长期漂移。加速度计比力不用于航向校正。
    loc->yaw_attitude.yaw += mid_omega * dt;

    // 4. 偏航角范围约束（确保在 [-π, π] 内，避免角度溢出）
    loc->yaw_attitude.yaw = fmodf(loc->yaw_attitude.yaw + PI_F, TWO_PI_F) - PI_F;

// 调试打印：验证方向与数值正确性
#if DEBUG_PRINT_VERBOSE
    log_print("YAW:%f,1.5708\n", loc->yaw_attitude.yaw);
#endif

    // 位置积分（中点欧拉：theta 仍为本周期旧值，故用 theta + omega*dt/2 取区间中点）
    // 原实现写成 theta - omega*dt/2，方向反了，等于用“滞后半拍”的姿态做旋转。
    float mid_theta = constrain_angle(loc->current_pose.theta + (mid_omega * dt / 2.0f));
    float robot_dx = mid_vx * dt; // 米（轮速单位 m/s）
    float robot_dy = mid_vy * dt;
    float global_dx, global_dy;
    kinematics_robot_to_global(robot_dx, robot_dy, mid_theta, &global_dx, &global_dy);
    // 积分结果为米，current_pose 为地图单位（1 地图单位=MAP_SCALE=0.01m），
    // 必须经 real_to_map() 换算后再累加，否则坐标差 100 倍
    loc->current_pose.x += real_to_map(global_dx);
    loc->current_pose.y += real_to_map(global_dy);

    // 位置积分已用完旧 theta，之后才把陀螺积分（并约束到 [-π,π]）的偏航角写回位姿朝向
    loc->current_pose.theta = loc->yaw_attitude.yaw;

#if DEBUG_PRINT_VERBOSE
    log_print("pos->position has update x:%.1f y:%.1f theta:%.1f\n", loc->current_pose.x, loc->current_pose.y, loc->current_pose.theta);
#endif
}

// 将位姿吸附到当前校准点，并为下一次经过校准线选择前方相邻节点。
bool localization_apply_calibration(LocalizationModule *loc)
{
    if (loc == NULL || !loc->has_current_calib)
    {
        return false;
    }

    CalibrationPoint reached = loc->current_calib;
    loc->current_pose.x = reached.x;
    loc->current_pose.y = reached.y;
    loc->is_calibrated = 1;

    CalibrationPoint next_calib;
    if (map_globalmap_find_next_calib(&global_map, reached,
                                      loc->current_pose.theta, 0.3f,
                                      &next_calib))
    {
        loc->current_calib = next_calib;
        loc->has_current_calib = true;
    }
    else
    {
        loc->current_calib.node_id = UINT8_MAX;
        loc->current_calib.x = 0.0f;
        loc->current_calib.y = 0.0f;
        loc->has_current_calib = false;
    }

    return true;
}

// 获取当前位姿
// 本函数仅做数据复制；禁止在此调用 log_print（阻塞串口输出，且原硬编码
// ENEMY:300.0,400.0 为假数据）。POS/RPOS 由 FSM 节流发送，
// PID/ENEMY 遥测统一在 robot_protocol_update 中随节流块上报。
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
}

