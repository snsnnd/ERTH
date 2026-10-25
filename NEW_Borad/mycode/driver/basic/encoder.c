#include "encoder.h"
#include "BT.h"

// 实际硬件（校赛原理图）：编码器接口只有 2 组
//   H15 = PA0/PA1 = TIM2_CH1/CH2 → 左侧（前/后轮共用一次采样）
//   H14 = PB6/PB7 = TIM4_CH1/CH2 → 右侧（前/后轮共用一次采样）
// 上层仍按 4 个枚举（前左/后左/前右/后右）读取，同侧两次读取复用
// 同一次采样缓存（计数器读取即清零，第二次读不能重复消费）。
#define ENCODER_LEFT_TIM  &htim2
#define ENCODER_RIGHT_TIM &htim4
#define ENCODER_SIDE_CNT  2
#define SIDE_LEFT 0
#define SIDE_RIGHT 1

// enum ENCODER 下标 → 侧别（顺序须与 encoder.h 枚举一致）
static const uint8_t enc_side_of[4] = {
    SIDE_LEFT,  // encoder_front_left
    SIDE_LEFT,  // encoder_back_left
    SIDE_RIGHT, // encoder_front_right
    SIDE_RIGHT  // encoder_back_right
};

static inline TIM_HandleTypeDef *enc_side_tim(uint8_t side)
{
    return (side == SIDE_LEFT) ? ENCODER_LEFT_TIM : ENCODER_RIGHT_TIM;
}

// 每侧的上一次采样时间戳（单位：毫秒）
static uint32_t last_sample_time[ENCODER_SIDE_CNT] = {0};
// 每侧同一次采样的缓存（供同侧第二次读取复用）
static float side_speed_cache[ENCODER_SIDE_CNT] = {0.0f, 0.0f};
static uint32_t side_cache_ms[ENCODER_SIDE_CNT] = {0xFFFFFFFFu, 0xFFFFFFFFu};
// 每脉冲对应的距离（单位：米/脉冲，与硬件相关）
static float distance_per_pulse = 0.0f;

void encoder_setparam(uint16_t ENCODER_PPR, uint8_t ENCODER_MULTIPLE, float WHEEL_DIAMETER)
{
    // 计算步骤解析：
    // 1. 轮子周长 = π × 直径（单位：cm）
    // 2. 每脉冲距离 = 周长 ÷（线数 × 倍频 x 100）（单位：m/脉冲）
    // 3. 速度系数 = 每脉冲距离 ÷ 采样间隔（单位：m/(脉冲·s)）
    distance_per_pulse = (WHEEL_DIAMETER * 3.1415926f) / (ENCODER_PPR * ENCODER_MULTIPLE * 100.0f);
}

void encoder_init(void)
{
    encoder_setparam(13, 4, 6.5f);
    HAL_TIM_Encoder_Start(ENCODER_LEFT_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(ENCODER_RIGHT_TIM, TIM_CHANNEL_ALL);
    encoder_clearcounter();
    // 初始化时间戳（首次采样前设为当前时间）
    for (int i = 0; i < ENCODER_SIDE_CNT; i++)
    {
        last_sample_time[i] = HAL_GetTick(); // 使用HAL库的毫秒级时间函数
        side_cache_ms[i] = 0xFFFFFFFFu;      // 尚无缓存
    }
}

void encoder_clearcounter(void)
{
    __HAL_TIM_SET_COUNTER(ENCODER_LEFT_TIM, 0);
    __HAL_TIM_SET_COUNTER(ENCODER_RIGHT_TIM, 0);
}

// 编码器异常检测
bool encoder_checkerror(enum ENCODER encoder)
{
    if ((unsigned)encoder > (unsigned)encoder_back_right)
    {
        return 1; // 无效电机编号
    }
    TIM_HandleTypeDef *TIM = enc_side_tim(enc_side_of[encoder]);

    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(TIM);
    // 异常条件：计数处于[32769, 63734]（接近溢出但未反转，可能是干扰）
    if (cnt > 32768 && cnt < 63735)
    {
        return 1; // 异常
    }
    return 0; // 正常
}

float encoder_readspeed(enum ENCODER encoder)
{
    uint16_t raw_counter; // 原始无符号计数器值（直接读定时器）
    int16_t pulse;        // 含方向的脉冲数（正转+，反转-）
    float speed = 0.0f;   // 速度（m/s，正负代表方向）
    uint32_t current_time; // 当前时间戳（ms）
    uint32_t dt_ms;        // 时间差（ms）
    float dt;              // 时间差（s）

    if ((unsigned)encoder > (unsigned)encoder_back_right)
    {
        return 0.0f; // 无效编码器
    }
    uint8_t side = enc_side_of[encoder];

    // 2. 获取当前时间并计算间隔
    current_time = HAL_GetTick();

    // 硬件只有 2 路编码器，同侧前后轮共用同一次采样：
    // 同 1ms 内的重复读取直接复用缓存（计数器读取即清零，
    // 否则同侧第二次读取会得到 0）。同侧所有枚举的方向校正规则
    // 一致（右负左正），缓存值可直接复用。
    if (current_time == side_cache_ms[side])
    {
        return side_speed_cache[side];
    }

    dt_ms = current_time - last_sample_time[side];
    dt = dt_ms / 1000.0f; // 转换为秒

    // 3. 避免除零（首次采样或间隔为0时返回0）
    if (dt <= 0.0f)
    {
        last_sample_time[side] = current_time; // 更新时间戳
        return 0.0f;
    }

    // 4. 读取并清零计数器（先存原始无符号值）
    TIM_HandleTypeDef *TIM = enc_side_tim(side);
    raw_counter = (uint16_t)__HAL_TIM_GET_COUNTER(TIM);
    __HAL_TIM_SET_COUNTER(TIM, 0);

    // 5. 处理方向：将无符号计数器值转换为含符号的脉冲数
    // 16位无符号范围：0~65535，中间值32767.5，以此区分正反转
    if (raw_counter > 32768)
    {
        // 反转：计数器因递减溢出，实际脉冲数为负（绝对值=65535 - raw_counter）
        pulse = -(65535 - raw_counter);
    }
    else
    {
        // 正转：计数器递增，脉冲数为正
        pulse = (int16_t)raw_counter;
    }

    // 6. 计算速度：含方向（脉冲数符号决定速度符号）
    speed = (float)pulse * distance_per_pulse / dt;

    // 7. （可选）硬件方向校准：若左右轮安装方向相反，修正符号
    // 例如：右轮编码器正转时实际输出反转，需加负号（根据实际测试调整）
    if (encoder == encoder_front_right || encoder == encoder_back_right)
    {
        speed = -speed;
    }

    // 8. 更新采样时间与同侧缓存（同侧第二次读取直接复用）
    last_sample_time[side] = current_time;
    side_speed_cache[side] = speed;
    side_cache_ms[side] = current_time;

    return speed;
}

#include "stdio.h" // 若需要使用sprintf等格式化输出

/**
 * @brief 测试指定编码器的正负方向
 * @param encoder：要测试的编码器（枚举类型）
 * @note 测试时需手动正转/反转对应轮子，函数会输出方向判断结果
 */
void encoder_test_direction(enum ENCODER encoder)
{
    TIM_HandleTypeDef *TIM;
    uint16_t raw_cnt_forward, raw_cnt_backward;
    int16_t signed_cnt_forward, signed_cnt_backward; // 转换为有符号数处理方向

    // 绑定编码器对应的定时器（同侧前后轮共用一路）
    if ((unsigned)encoder > (unsigned)encoder_back_right)
    {
        log_print("错误：无效的编码器编号！\r\n");
        return;
    }
    TIM = enc_side_tim(enc_side_of[encoder]);

    // 第一步：测试正向转动
    log_print("\n请正向转动编码器（如向前推对应轮子）...\r\n");
    encoder_clearcounter();                       // 清零计数器
    HAL_Delay(5000);                              // 等待2秒，留给用户操作时间
    raw_cnt_forward = (uint16_t)__HAL_TIM_GET_COUNTER(TIM); // 读取正向转动后的计数

    // 第二步：测试反向转动
    log_print("请反向转动编码器（如向后拉对应轮子）...\r\n");
    encoder_clearcounter();                        // 再次清零计数器
    HAL_Delay(5000);                               // 等待2秒
    raw_cnt_backward = (uint16_t)__HAL_TIM_GET_COUNTER(TIM); // 读取反向转动后的计数

    // 转换为有符号16位整数（处理16位计数器的溢出/方向特性）
    signed_cnt_forward = (int16_t)raw_cnt_forward;
    signed_cnt_backward = (int16_t)raw_cnt_backward;

    // 输出原始数据供参考
    log_print("测试结果：\r\n");
    log_print("正向转动计数（原始）：%u，（有符号）：%d\r\n", raw_cnt_forward, signed_cnt_forward);
    log_print("反向转动计数（原始）：%u，（有符号）：%d\r\n", raw_cnt_backward, signed_cnt_backward);

    // 判断方向是否符合预期（正转负转计数符号相反）
    if ((signed_cnt_forward > 0 && signed_cnt_backward < 0) ||
        (signed_cnt_forward < 0 && signed_cnt_backward > 0))
    {
        log_print("编码器方向测试通过：正转和反转计数符号相反\r\n");
    }
    else if (signed_cnt_forward == 0 && signed_cnt_backward == 0)
    {
        log_print("警告：未检测到转动，请确保轮子有明显转动\r\n");
    }
    else
    {
        log_print("编码器方向测试失败：正转和反转计数符号相同（可能接线或配置错误）\r\n");
    }
}
