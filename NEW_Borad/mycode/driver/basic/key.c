#include "key.h"

// #define key_Pin
// #define key_GPIO_Port
#define u8 uint8_t

// uint8_t key_read()
// {
//     if (!HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin))
//     {
//         HAL_Delay(10);
//         if (!HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin))
//         {
//             bee_on(1, 1);

//             return 1;
//         }
//     }

//     return 0;
// }

void key_read(u8 *a, u8 *b)
{
    if (!HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin))
    {
        delay_ms(10);
        if (!HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin))
        {
            if (*b == 3)
            {
                *b = 0;
            }
            else
            {
                *b = 3;
            }
        }
        while (HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin))
            ;
    }
}
