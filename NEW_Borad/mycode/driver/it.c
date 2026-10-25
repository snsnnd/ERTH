#include "it.h"

/* 全局变量定义 */
uint8_t time_flag = 0;          // TIM12触发标志
uint16_t count = 0, _count = 0; // TIM6计数变量
uint8_t wave_flagL = 0, wave_flagR = 0, wave_flagF = 0, wave_flagB = 0;
uint16_t distanceL = 0, distanceR = 0, distanceF = 0, distanceB = 0;

/* 定时器启动函数 */
void TIM6_Start(void) { HAL_TIM_Base_Start_IT(&htim6); }

void TIM7_Start(void) { HAL_TIM_Base_Start_IT(&htim7); }

void TIM12_Start(void)
{
    __HAL_TIM_SET_COUNTER(&htim12, 0); // 重置计数器
    HAL_TIM_Base_Start_IT(&htim12);    // 启动中断
}

void TIM12_Stop(void) { HAL_TIM_Base_Stop_IT(&htim12); }

/* 定时器中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // TIM6：20ms周期，执行核心控制逻辑
        count++;
        _count++;
        Track_Pid();     // 路径跟踪PID（用户实现）
        Speed_Control(); // 电机速度控制（用户实现）
    }
    else if (htim->Instance == TIM7)
    {
        // TIM7：27ms超时检测
        if (wave_flagL)
        {
            distanceL = ERROR;
            wave_flagL = 0;
        }
        if (wave_flagR)
        {
            distanceR = ERROR;
            wave_flagR = 0;
        }
        if (wave_flagF)
        {
            distanceF = ERROR;
            wave_flagF = 0;
        }
        if (wave_flagB)
        {
            distanceB = ERROR;
            wave_flagB = 0;
        }
    }
    else if (htim->Instance == TIM12)
    {
        // TIM12：310ms单次触发
        buffer[6] = 0;
        buffer[7] = 0;
        time_flag = !time_flag; // 翻转标志位
        TIM12_Stop();           // 触发后停止
    }
}
