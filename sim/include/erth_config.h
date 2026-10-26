#ifndef ERTH_CONFIG_H
#define ERTH_CONFIG_H

/*
 * 实车参数：默认值按「校赛典型车 + 65 mm 轮 + 8 路数字灰度」估算，
 * 请用实测值替换。改这里就够了，仿真与后续固件共用同一份常数。
 *
 * 标 ★ 的是还没确认、会明显影响结论的量。
 */

/* ---- 赛道：闭环椭圆 ---- */
#define ERTH_SEMI_MAJOR_M      1.50f   /* 长半轴（沿 x） */
#define ERTH_SEMI_MINOR_M      1.00f   /* 短半轴（沿 y） */
#define ERTH_LINE_WIDTH_M      0.020f  /* 线宽 */

/* ---- 灰度阵列 ---- */
#define ERTH_SENSOR_COUNT      8
#define ERTH_SENSOR_SPACING_M  0.015f  /* 相邻探头间距 ★ */
#define ERTH_SENSOR_TO_AXLE_M  0.090f  /* 探头排到轴心 ★ */
/*
 * 灰度极性：固件 sensor.h 注明「0=黑，1=白」，即压线时该位为 0。
 * 8 位全 1（0xFF）= 全部见白 = 丢线，与 track_pid.c 的 mask==255 停车一致。
 */
#define ERTH_SENSOR_LEVEL_ON_LINE 0

/* ---- 底盘 ---- */
#define ERTH_TRACK_WIDTH_M     0.140f  /* 左右轮距 ★ */
#define ERTH_WHEEL_RADIUS_M    0.0325f /* 65 mm 轮 */
#define ERTH_V_MAX_MPS         1.20f   /* 满占空比稳态速度 ★ */

/* ---- 电机：一阶 + 死区 ---- */
#define ERTH_DEADBAND          0.07f   /* 占空比死区，用标定库测出来再填 */
#define ERTH_TAU_S             0.060f  /* 一阶时间常数 */
#define ERTH_GAIN_LEFT         1.00f   /* 左右轮增益差，1 = 一致 */
#define ERTH_GAIN_RIGHT        1.00f
#define ERTH_PWM_MAX           100

/* ---- 编码器 ---- */
/*
 * ★ 关键未确认项：编码器装在电机轴还是轮轴？
 * 固件 encoder_setparam(13, 4, 6.5f) 没有乘减速比，等价于「装在轮轴」，
 * 即每转 52 个计数 → 3.9 mm/计数 → 10 ms 内速度量化 0.39 m/s，
 * 这多半就是速度环被停掉的原因。
 * 如果实际是电机轴（常见 1:30），把 ERTH_GEAR_RATIO 改成 30，
 * 分辨率提高 30 倍，速度环才具备可行性。
 */
#define ERTH_ENCODER_PPR       13
#define ERTH_ENCODER_QUAD      4
#define ERTH_GEAR_RATIO        1.0f    /* 轮轴 1.0；电机轴填减速比 */

/* ---- 时序 ---- */
#define ERTH_DT_S              0.010f  /* 主循环/循迹节拍 100 Hz */
#define ERTH_DURATION_S        30.0f

/* ---- 现有循迹 PID（固件原值，勿改，作为对照基线） ---- */
#define ERTH_PID_KP            9.5f
#define ERTH_PID_KI            1.0f
#define ERTH_PID_KD            2.8f
#define ERTH_PID_OUT_MAX       30.0f
#define ERTH_PID_I_MAX         6.0f
#define ERTH_BASE_SPEED        15.0f
#define ERTH_MIN_SPEED         10.0f
#define ERTH_MAX_SPEED         30.0f
#define ERTH_OPEN_LOOP_GAIN    3.0f

#endif
