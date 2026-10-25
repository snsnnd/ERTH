#include "wave.h"
#include "main.h"

int distance_Left = 0, distance_Right = 0, distance_Front = 0, distance_Back = 0;
_Bool wave_flagL = 0, wave_flagR = 0, wave_flagF = 0, wave_flagB = 0;
u8 Left_flag = 0, Right_flag = 0, Front_flag = 0, Right_flag_t = 0, Front_flag_t = 0, Left_flag_t = 0, Back_flag = 0, Right_flag_tt = 0;
extern _Bool special_start, special_start2;
u8 special_counter = 0, special_counter2 = 0;

extern u8 count;

void wave_start_Blue(void) // ������ģ����������
{
    static u8 h = 0, i = 0; // ��ֹ�ظ�ִ��

    if (count == 2 && h != count)
    {
        i = h = count;
        HAL_GPIO_WritePin(Trig_right_GPIO_Port, Trig_right_Pin, GPIO_PIN_SET);
        delay_us(12);
        HAL_GPIO_WritePin(Trig_right_GPIO_Port, Trig_right_Pin, GPIO_PIN_RESET);
    }
    if (count == 4 && i != count)
    {
        h = i = count;
        HAL_GPIO_WritePin(Trig_left_GPIO_Port, Trig_left_Pin, GPIO_PIN_SET);
        delay_us(12);
        HAL_GPIO_WritePin(Trig_left_GPIO_Port, Trig_left_Pin, GPIO_PIN_RESET);
    }
}

extern u8 _count;
void wave_start_Red(void)
{
    static u8 z = 0, x = 0, c = 0, v = 0;

    if (_count == 2 && z != count)
    {
        z = x = c = v = count;
        HAL_GPIO_WritePin(Trig_right_GPIO_Port, Trig_right_Pin, GPIO_PIN_SET);
        delay_us(12);
        HAL_GPIO_WritePin(Trig_right_GPIO_Port, Trig_right_Pin, GPIO_PIN_RESET);
    }
    if (_count == 4 && x != count)
    {
        z = x = c = v = count;
        HAL_GPIO_WritePin(Trig_left_GPIO_Port, Trig_left_Pin, GPIO_PIN_SET);
        delay_us(12);
        HAL_GPIO_WritePin(Trig_left_GPIO_Port, Trig_left_Pin, GPIO_PIN_RESET);
    }
    if (_count == 6 && c != count)
    {
        z = x = c = v = count;
        HAL_GPIO_WritePin(Trig_right_GPIO_Port, Trig_right_Pin, GPIO_PIN_SET);
        delay_us(12);
        HAL_GPIO_WritePin(Trig_right_GPIO_Port, Trig_right_Pin, GPIO_PIN_RESET);
    }
    if (_count == 8 && v != count)
    {
        z = x = c = v = count;
        HAL_GPIO_WritePin(Trig_back_GPIO_Port, Trig_back_Pin, GPIO_PIN_SET);
        delay_us(12);
        HAL_GPIO_WritePin(Trig_back_GPIO_Port, Trig_back_Pin, GPIO_PIN_RESET);
    }
}

void EXTI4_IRQHandler(void) // ���������   ǰ
{
    if (EXTI_GetITStatus(EXTI_Line4) == SET)
    {
        if (HAL_GPIO_ReadPin(Echo_right_GPIO_Port, Echo_right_Pin) == 1) // 右
        {
            TIM_SetCounter(TIM7, 0);
            wave_flagF = 1;
        }
        if (HAL_GPIO_ReadPin(Echo_right_GPIO_Port, Echo_right_Pin) == 0 && wave_flagF == 1)
        {
            wave_flagF = 0;
            distance_Front = (17 * TIM_GetCounter(TIM7)) / 1000;
            if (distance_Front > 3 && distance_Front < 60)
            {
                Front_flag++;
            }
            else
            {
                Front_flag = 0;
            }
            if (distance_Front > 3 && distance_Front < 91)
            {
                Front_flag_t++;
            }
            else
            {
                Front_flag_t = 0;
            }
        }
    }
    EXTI_ClearITPendingBit(EXTI_Line4);
}

void EXTI1_IRQHandler(void) // ���������   ��
{
    if (EXTI_GetITStatus(EXTI_Line1) == SET)
    {
        if (HAL_GPIO_ReadPin(Echo_left_GPIO_Port, Echo_left_Pin) == 1)
        {
            TIM_SetCounter(TIM7, 0);
            wave_flagR = 1;
        }
        if (PGin(1) == 0 && wave_flagR == 1)
        {
            wave_flagR = 0;
            distance_Right = (17 * TIM_GetCounter(TIM7)) / 1000;
            if (special_start == 1)
                special_counter++;
            if (distance_Right > 3 && distance_Right < 175)
            {
                Right_flag++;
            }
            else
            {
                Right_flag = 0;
            }
            if (distance_Right > 3 && distance_Right < 84)
            {
                Right_flag_t++;
            }
            else
            {
                Right_flag_t = 0;
            }
            if (distance_Right > 3 && distance_Right < 68)
            {
                Right_flag_tt++;
            }
            else
            {
                Right_flag_tt = 0;
            }
        }
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
}

void EXTI9_5_IRQHandler(void) // ���������
{
    if (EXTI_GetITStatus(EXTI_Line6) == SET)
    {
        if (PGin(6) == 1) // ��
        {
            TIM_SetCounter(TIM7, 0);
            wave_flagL = 1;
        }
        if (PGin(6) == 0 && wave_flagL == 1)
        {
            wave_flagL = 0;
            if (special_start2 == 1)
                special_counter2++;
            distance_Left = (17 * TIM_GetCounter(TIM7)) / 1000;
            if (distance_Left > 3 && distance_Left < 145)
            {
                Left_flag++;
            }
            else
            {
                Left_flag = 0;
            }
            if (distance_Left > 3 && distance_Left < 65)
            {
                Left_flag_t++;
            }
            else
            {
                Left_flag_t = 0;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line6);
    }

    if (EXTI_GetITStatus(EXTI_Line8) == SET)
    {
        if (PGin(8) == 1) // ��
        {
            TIM_SetCounter(TIM7, 0);
            wave_flagB = 1;
        }
        if (PGin(8) == 0 && wave_flagB == 1)
        {
            wave_flagB = 0;
            distance_Back = (17 * TIM_GetCounter(TIM7)) / 1000;
            if (distance_Back > 3 && distance_Back <= 55)
            {
                Back_flag++;
            }
            else
            {
                Back_flag = 0;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}
