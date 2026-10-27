

// Get_Current_Yaw 给运动学模块 用于转向

// 获取时间 time.c

// position.c:
// #define CALIB_DEBOUNCE_MS 500      // 校准防抖时间（ms），防止连续触发
// #define YAW_KALMAN_Q_ANGLE 0.001f  // 角度过程噪声协方差
// #define YAW_KALMAN_Q_GYRO 0.003f   // 陀螺仪过程噪声协方差
// #define YAW_KALMAN_R_ANGLE 0.5f    // 测量噪声协方差
// #define OMEGA_FUSION_ALPHA 0.8f    // omega融合系数（轮子:MPU = alpha:(1-alpha)）
// #define DEFAULT_LP_ALPHA 0.7f      // 默认低通滤波系数（如果未指定）
