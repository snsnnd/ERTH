#include "motor.h"
#include "oled.h"
#include "delay.h"
#include "BT.h"

// ============================================================================
// 实际硬件适配（校赛原理图 U5 = TB6612，差速两驱）：
//   PWMA = PB0 = TIM3_CH3 → 左轮（A 路）
//   PWMB = PB1 = TIM3_CH4 → 右轮（B 路）
//   AIN1/AIN2 = PA4/PA5（左轮方向），BIN1/BIN2 = PA6/PA7（右轮方向）
//   STBY = PA3（高电平使能）
// 引脚宏 AIN1_GPIO_Port/AIN1_Pin 等由 CubeMX 用户标签生成于 main.h
// （原方案：TIM10/11/9/1 八路 PWM 直驱四电机——已废弃）
// 若实车左右对调：交换 side_set 中 A/B 两组宏即可
// ============================================================================

#define MOTOR_PWM_A (TIM3->CCR3) // 左轮占空比寄存器
#define MOTOR_PWM_B (TIM3->CCR4) // 右轮占空比寄存器

// 原 static int16_t abs(...)：与 C 内建/stdlib 的 abs 冲突（gcc -Wbuiltin-declaration-mismatch），改名 iabs
static int16_t iabs(int16_t num)
{
    return (num < 0) ? -num : num;
}

/**
 * @brief  单侧电机输出（TB6612 速度模式：方向脚 + PWM 占空比）
 * @param  is_left: 1=左（A 路），0=右（B 路）
 * @param  pwm: 带符号速度，>0 正转，<0 反转，0 滑行停止
 */
static void side_set(uint8_t is_left, int16_t pwm)
{
    if (pwm > PWM_MAX)
        pwm = PWM_MAX;
    if (pwm < -PWM_MAX)
        pwm = -PWM_MAX;

    GPIO_PinState dir1, dir2;
    if (pwm > 0)
    {
        dir1 = GPIO_PIN_SET;   // 正转：IN1=H, IN2=L
        dir2 = GPIO_PIN_RESET;
    }
    else if (pwm < 0)
    {
        dir1 = GPIO_PIN_RESET; // 反转：IN1=L, IN2=H
        dir2 = GPIO_PIN_SET;
        pwm = iabs(pwm);
    }
    else
    {
        dir1 = GPIO_PIN_RESET; // 停止：IN1=L, IN2=L（coast）
        dir2 = GPIO_PIN_RESET;
    }

    if (is_left)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, dir1);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, dir2);
        MOTOR_PWM_A = (uint32_t)pwm;
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, dir1);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, dir2);
        MOTOR_PWM_B = (uint32_t)pwm;
    }
}

void motor_init(void) // 电机初始化函数
{
    // TB6612 使能：STBY 置高
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    // PWM 输出：TIM3 CH3（PB0=左）/ CH4（PB1=右）
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    motor_stop();
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
    motor_setspeed(motor_back_right, 100); // 右路（B）全速前进
    OLED_Clear();                                // 清屏
    OLED_ShowString(0, 0, (u8 *)"PWM Test:"); // 提示文字（第1行）
    uint32_t count = TIM3->CCR4;                  // 右路占空比寄存器
    OLED_ShowNumber(0, 20, count, 5, 16);
    uint32_t in = TIM3->CCR3; // 左路占空比寄存器
    OLED_ShowNumber(0, 40, in, 5, 16);
    OLED_Refresh_Gram(); // 刷新显示
}
