#include "bee.h"

// #define bee_Pin
// #define bee_GPIO_Port

void bee_on(uint8_t a, uint8_t b)
{
    // 外层循环：控制总次数（a次）
    for (uint8_t i = 0; i < a; i++)
    {
        // 内层循环：控制每次的响铃次数（b下）
        for (uint8_t j = 0; j < b; j++)
        {
            HAL_GPIO_WritePin(bee_GPIO_Port, bee_Pin, GPIO_PIN_SET); // 蜂鸣器响
            HAL_Delay(50);                                           // 单次响铃持续时间（可根据需要调整，如50ms）

            HAL_GPIO_WritePin(bee_GPIO_Port, bee_Pin, GPIO_PIN_RESET); // 蜂鸣器停
            if (j != b - 1)                                            // 若不是本次最后一下，添加间隔
            {
                HAL_Delay(50); // 同一次内，两下之间的间隔（可调整）
            }
        }

        if (i != a - 1) // 若不是最后一次，添加次与次之间的间隔
        {
            HAL_Delay(200); // 不同次之间的间隔（可调整，如200ms）
        }
    }
}

void test_bee()
{
    bee_on(5, 5);
}
