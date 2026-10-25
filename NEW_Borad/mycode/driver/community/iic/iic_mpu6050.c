#include "iic_mpu6050.h"

#define SDA_PIN mpu6050_sda_Pin
#define SDA_PORT mpu6050_sda_GPIO_Port
#define SCL_PIN mpu6050_scl_Pin
#define SCL_PORT mpu6050_scl_GPIO_Port

int SCL_bit = 1;
int SDA_bit = 1;

/*
 *函数：I2C写SCL电平
 *参数：BitValue 当前需要写入SCL的电平，值为0或1
 *返回值：无
 *注意事项：当BitValue为0时，置SCL为低电平；当BitValue为1时，置SCL为高电平
 */
void SOFT_I2C_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, (BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET));
    SCL_bit = BitValue;
    RCCdelay_us(5);
}

/*
 *函数：I2C写SDA电平
 *参数：BitValue 当前需要写入SDA的电平，值为0或1
 *返回值：无
 *注意事项：当BitValue为0时，置SDA为低电平；当BitValue为1时，置SCL为高电平
 */
void SOFT_I2C_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, (BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET));
    SDA_bit = BitValue;
    RCCdelay_us(5);
}

/*
 *函数：I2C读取SDA引脚的电平状态
 *参数：无
 *返回值：读取出的电平
 *注意事项：低电平为0，高电平为1
 */
uint8_t SOFT_I2C_R_SDA(void)
{
    uint8_t BitValue;
    if (HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN) == GPIO_PIN_RESET)
    {
        BitValue = 0;
    }
    else
    {
        BitValue = 1;
    }
    return BitValue;
}

/*
 *函数：I2C开始信号
 *参数：无
 *返回值：无
 *注意事项：无
 */
void SOFT_I2C_Start(void)
{
    SOFT_I2C_W_SDA(1);
    SDA_bit = 1;
    SOFT_I2C_W_SCL(1);
    SCL_bit = 1;
    SOFT_I2C_W_SDA(0);
    SDA_bit = 0;
    SOFT_I2C_W_SCL(0);
    SCL_bit = 0;
}

/*
 *函数：I2C停止信号
 *参数：无
 *返回值：无
 *注意事项：无
 */
void SOFT_I2C_Stop(void)
{
    SOFT_I2C_W_SDA(0);
    SDA_bit = 0;
    SOFT_I2C_W_SCL(1);
    SCL_bit = 1;
    SOFT_I2C_W_SDA(1);
    SDA_bit = 1;
}

/*
 *函数：I2C发送8位数据
 *参数：数据
 *返回值：无
 *注意事项：无
 */
void SOFT_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        SOFT_I2C_W_SDA(Byte & (0x80 >> i));
        SDA_bit = (Byte & (0x80 >> i));
        SOFT_I2C_W_SCL(1);
        SCL_bit = 1;
        SOFT_I2C_W_SCL(0);
        SCL_bit = 0;
    }
}
/*
 *函数：I2C接收8位数据
 *参数：无
 *返回值：数据
 *注意事项：无
 */
uint8_t SOFT_I2C_ReceiveByte(void)
{
    uint8_t Byte = 0x00;
    SOFT_I2C_W_SDA(1); // 先拉高sda，之前主机一直占用着sda，释放sda，让从机占据。
    SDA_bit = 1;
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        SOFT_I2C_W_SCL(1); // 先拉高scl，从机会把数据放到sda上
        SCL_bit = 1;
        if (SOFT_I2C_R_SDA() == 1)
        { // 是0的时候不用改因为Byte是00000000
            Byte |= (0x80 >> i);
        }
        SOFT_I2C_W_SCL(0);
        SCL_bit = 0;
    }
    return Byte;
}
/*
 *函数：I2C发送应答位
 *参数：0或1
 *返回值：无
 *注意事项：无
 */
void SOFT_I2C_SendAck(uint8_t AckBit)
{
    SOFT_I2C_W_SDA(AckBit);
    SDA_bit = AckBit;
    SOFT_I2C_W_SCL(1);
    SCL_bit = 1;
    SOFT_I2C_W_SCL(0);
    SCL_bit = 0;
}
/*
 *函数：I2C接收应答位
 *参数：无
 *返回值：0或1
 *注意事项：无
 */
uint8_t SOFT_I2C_ReceiveAck(void)
{
    uint8_t AckBit;
    SOFT_I2C_W_SDA(1);
    SDA_bit = SOFT_I2C_R_SDA();
    SOFT_I2C_W_SCL(1);
    SCL_bit = 1;
    AckBit = SOFT_I2C_R_SDA();
    SOFT_I2C_W_SCL(0);
    SCL_bit = 0;
    return AckBit;
}

/*
 *函数：微秒延迟函数
 *参数：多少微秒
 *返回值：无
 *注意事项：无
 */
void RCCdelay_us(uint32_t udelay)
{
    delay_us(udelay);
}
