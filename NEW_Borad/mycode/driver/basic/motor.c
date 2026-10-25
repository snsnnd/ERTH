#include "motor.h"
#include "oled.h"
#include "delay.h"
#include "BT.h"
// 直接改变定时器的CCR的值
#define IN_1_1 TIM10->CCR1 // 编码器A  // 左前
#define IN_2_1 TIM9->CCR1  // 编码器B  // 左后
#define IN_3_1 TIM1->CCR1  // 编码器C  // 右后
#define IN_4_1 TIM1->CCR3  // 编码器D  // 右前

#define IN_1_2 TIM11->CCR1
#define IN_2_2 TIM9->CCR2
#define IN_3_2 TIM1->CCR2
#define IN_4_2 TIM1->CCR4

// 原 static int16_t abs(...)：与 C 内建/stdlib 的 abs 冲突（gcc -Wbuiltin-declaration-mismatch），改名 iabs
static int16_t iabs(int16_t num)
{
    return (num < 0) ? -num : num;
}

void motor_init(void) // 电机初始化函数
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); // 开启定时器二通道1的PWM功能
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); // 开启定时器二通道2的PWM功能

    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    motor_stop();
}

void motor_stop()
{
    IN_1_1 = 0;
    IN_1_2 = 0;

    IN_2_1 = 0;
    IN_2_2 = 0;

    IN_3_1 = 0;
    IN_3_2 = 0;

    IN_4_1 = 0;
    IN_4_2 = 0;
}

void motor_setspeed(enum MOTOR motor, int16_t pwm)
{
    if (pwm >= 0)
    {
        // log_print("+\t");
        if (pwm > PWM_MAX)
        {
            pwm = PWM_MAX;
        }
        switch (motor)
        {
        case motor_front_left:
            IN_1_1 = 0;
            IN_1_2 = pwm;
            break;
        case motor_front_right:
            IN_4_1 = pwm;
            IN_4_2 = 0;
            break;
        case motor_back_left:
            IN_2_1 = pwm;
            IN_2_2 = 0;
            break;
        case motor_back_right:
            IN_3_1 = 0;
            IN_3_2 = pwm;
            break;
        default:
            break;
        }
    }
    else
    {
        // log_print("-\t");
        pwm = iabs(pwm);
        if (pwm > PWM_MAX)
        {
            pwm = PWM_MAX;
        }
        switch (motor)
        {
        case motor_front_left:
            IN_1_1 = pwm;
            IN_1_2 = 0;
            break;
        case motor_front_right:
            IN_4_1 = 0;
            IN_4_2 = pwm;
            break;
        case motor_back_left:
            IN_2_1 = 0;
            IN_2_2 = pwm;
            break;
        case motor_back_right:
            IN_3_1 = pwm;
            IN_3_2 = 0;
            break;
        default:
            break;
        }
    }
}

void test_motor()
{
    motor_init();
    OLED_Init();
    delay_ms(1000);
    motor_setspeed(motor_back_right, 100);
    OLED_Clear();                                // 清屏
    OLED_ShowString(0, 0, (u8 *)"Number Test:"); // 提示文字（第1行）
    uint32_t count = TIM9->CCR1;
    OLED_ShowNumber(0, 20, count, 5, 16);
    uint32_t in = IN_2_1; // 原笔误：此处重复显示 count，导致 in 算而未用
    OLED_ShowNumber(0, 40, in, 5, 16); // 修正：显示 in（死变量由此变为有用）
    OLED_Refresh_Gram(); // 刷新显示
}
