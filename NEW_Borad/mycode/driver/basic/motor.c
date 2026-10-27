#include "motor.h"
#include "oled.h"
#include "delay.h"
#include "BT.h"

// ============================================================================
// 实际硬件适配（校赛原理图 U5 = TB6612，差速两驱）+ 参考车实车结论校正：
//   PWMB = PB1 = TIM3_CH4 + BIN1/BIN2 = 左轮（B 路）
//   PWMA = PB0 = TIM3_CH3 + AIN1/AIN2 = 右轮（A 路）
//   STBY = PA3（高电平使能，只在运动时拉高）
// 依据：参考工程 D:\efw_studio\examples\zhixianxunji\mishiduobao_f103
//       （同款硬件且能正常跑）中 MotorSet() 就是 left→BIN/CH4、right→AIN/CH3，
//       且 RIGHT_MOTOR_SIGN=-1。原工程按 A=左写成，实车表现为左右转向相反。
// 引脚宏 AIN1_GPIO_Port/AIN1_Pin 等由 CubeMX 用户标签生成于 main.h
// ============================================================================

static int16_t s_left_cmd = 0;  // 最近一次左轮指令（用于 STBY 联动）
static int16_t s_right_cmd = 0; // 最近一次右轮指令

static int16_t clamp_pwm(int32_t value)
{
    if (value > PWM_MAX)
        return PWM_MAX;
    if (value < -PWM_MAX)
        return -PWM_MAX;
    return (int16_t)value;
}

/**
 * @brief  单路 TB6612 输出（方向脚 + PWM）
 * @note   与参考车 MotorOne() 等价：正转 IN1=H/IN2=L，反转 IN1=L/IN2=H，停止双低（coast）
 */
static void motor_one(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                      GPIO_TypeDef *in2_port, uint16_t in2_pin,
                      uint32_t channel, int16_t cmd)
{
    uint16_t duty;

    if (cmd > 0)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        duty = (uint16_t)cmd;
    }
    else if (cmd < 0)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        duty = (uint16_t)(-cmd);
    }
    else
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        duty = 0U;
    }
    __HAL_TIM_SET_COMPARE(&htim3, channel, duty);
}

/**
 * @brief  左右轮同时输出（应用方向符号 + STBY 联动）
 * @param  left/right: 带符号指令，>0 前进，<0 后退，0 滑行
 */
static void motor_set(int16_t left, int16_t right)
{
    left = clamp_pwm((int32_t)left * LEFT_MOTOR_SIGN);
    right = clamp_pwm((int32_t)right * RIGHT_MOTOR_SIGN);

    // STBY 只在有输出时使能，停车即关断（与参考车一致，比常驻高电平更安全）
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin,
                      ((left != 0) || (right != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    motor_one(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, TIM_CHANNEL_4, left);
    motor_one(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, TIM_CHANNEL_3, right);
}

void motor_set_lr(int16_t left_pwm, int16_t right_pwm)
{
    motor_set(left_pwm, right_pwm);
}

/**
 * @brief  单侧更新（保持原 side_set 语义：只改一侧，另一侧沿用上次值）
 * @param  is_left: 1=左（B 路），0=右（A 路）
 * @param  pwm: 带符号速度，>0 正转，<0 反转，0 滑行停止
 */
static void side_set(uint8_t is_left, int16_t pwm)
{
    if (is_left)
        s_left_cmd = pwm;
    else
        s_right_cmd = pwm;
    motor_set(s_left_cmd, s_right_cmd);
}

void motor_init(void) // 电机初始化函数
{
    // TB6612 先关断，等有输出时再由 motor_set 拉高 STBY
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    // PWM 输出：TIM3 CH3（PB0=A 路=右轮）/ CH4（PB1=B 路=左轮）
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    s_left_cmd = 0;
    s_right_cmd = 0;
    motor_set(0, 0);
}

void motor_stop()
{
    side_set(1, 0);
    side_set(0, 0);
}

void motor_setspeed(enum MOTOR motor, int16_t pwm)
{
    // 差速两驱：四轮枚举按侧合并（上层对同侧两次调用值相同，后调覆盖无冲突）
    switch (motor)
    {
    case motor_front_left:
    case motor_back_left:
        side_set(1, pwm);
        break;
    case motor_front_right:
    case motor_back_right:
        side_set(0, pwm);
        break;
    case motor_all:
        side_set(1, pwm);
        side_set(0, pwm);
        break;
    case motor_back:
    case motor_front:
        // 差速两驱无前后轮独立控制（枚举保留仅为接口兼容），不动作
        break;
    default:
        break;
    }
}

void test_motor()
{
    motor_init();
    OLED_Init();
    delay_ms(1000);
    motor_setspeed(motor_back_right, 500); // 右轮 50% 占空比前进（量程已改为 0~999）
    OLED_Clear();                                // 清屏
    OLED_ShowString(0, 0, (u8 *)"PWM Test:"); // 提示文字（第1行）
    uint32_t count = TIM3->CCR3;                  // 右轮（A 路）占空比寄存器
    OLED_ShowNumber(0, 20, count, 5, 16);
    uint32_t in = TIM3->CCR4; // 左轮（B 路）占空比寄存器
    OLED_ShowNumber(0, 40, in, 5, 16);
    OLED_Refresh_Gram(); // 刷新显示
}
