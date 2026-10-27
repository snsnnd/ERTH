#include "iic_mpu6050.h"

#define SDA_PIN mpu6050_sda_Pin
#define SDA_PORT mpu6050_sda_GPIO_Port
#define SCL_PIN mpu6050_scl_Pin
#define SCL_PORT mpu6050_scl_GPIO_Port

int SCL_bit = 1;
int SDA_bit = 1;

// 忙等超时上限（防止从机拉死 SDA/SCL 导致死循环，超时后走总线恢复）
#define SOFT_I2C_WAIT_TIMEOUT 10000U

/*
 * 底层：把一根线驱动到指定电平
 * 参数：level 1=释放总线（切输入+内部上拉），0=开漏拉低
 * 说明：STM32F1 的 GPIO 在“输出模式”下不会接通内部上/下拉（PUPD 只对输入模式有效），
 *       所以“释放=高”必须真的切到输入+上拉，否则总线只能靠外部上拉浮起来；
 *       而“拉低”用开漏输出，避免与从机拉低时形成推挽对打（原实现是推挽，读 SDA 时
 *       主机强驱动高、从机拉低 → 大电流冲突，偶发应答失败/读错位多来自这里）。
 */
static void soft_i2c_pin_drive(GPIO_TypeDef *port, uint16_t pin, uint8_t level)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pin;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    if (level)
    {
        gpio.Mode = GPIO_MODE_INPUT; // 释放：高阻 + 内部上拉
        HAL_GPIO_Init(port, &gpio);
    }
    else
    {
        gpio.Mode = GPIO_MODE_OUTPUT_OD; // 拉低：开漏
        HAL_GPIO_Init(port, &gpio);
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
    RCCdelay_us(5); // 位延时（含建立/保持，足够覆盖 100kHz 软 I2C）
}

/*
 *函数：I2C写SCL电平
 *参数：BitValue 当前需要写入SCL的电平，值为0或1
 *返回值：无
 *注意事项：当BitValue为0时，置SCL为低电平；当BitValue为1时，释放SCL（开漏+上拉）
 */
void SOFT_I2C_W_SCL(uint8_t BitValue)
{
    soft_i2c_pin_drive(SCL_PORT, SCL_PIN, (BitValue != 0) ? 1 : 0);
    SCL_bit = BitValue;
}

/*
 *函数：I2C写SDA电平
 *参数：BitValue 当前需要写入SDA的电平，值为0或1
 *返回值：无
 *注意事项：当BitValue为0时，置SDA为低电平；当BitValue为1时，释放SDA（开漏+上拉）
 */
void SOFT_I2C_W_SDA(uint8_t BitValue)
{
    soft_i2c_pin_drive(SDA_PORT, SDA_PIN, (BitValue != 0) ? 1 : 0);
    SDA_bit = BitValue;
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
 *函数：带超时计数地等待 SDA 释放为高电平
 *参数：无
 *返回值：1=SDA 已释放；0=超时（SDA 仍被从机拉低）
 *注意事项：忙等循环带上限计数，防止从机拉死 SDA 造成死循环
 */
static uint8_t soft_i2c_wait_sda_release(void)
{
    uint32_t timeout = SOFT_I2C_WAIT_TIMEOUT;
    while (SOFT_I2C_R_SDA() == 0)
    {
        if (timeout-- == 0U)
        {
            return 0; // 超时：SDA 被从机拉死
        }
    }
    return 1;
}

/*
 *函数：I2C开始信号
 *参数：无
 *返回值：无
 *注意事项：发送前先带超时等待 SDA 释放；若超时先执行总线恢复再发 START
 */
void SOFT_I2C_Start(void)
{
    if (soft_i2c_wait_sda_release() == 0)
    {
        SOFT_I2C_BusRecover(); // SDA 被拉死：9 个时钟脉冲 + STOP 释放总线
    }
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
 *函数：I2C总线恢复序列
 *参数：无
 *返回值：无
 *说明：等待超时或通信失败（应答超时/失败，如从机把 SDA 拉死）时调用：
 *      释放 SDA → 产生至多 9 个 SCL 时钟脉冲 → 发送 STOP。
 *      从机可在时钟脉冲中发完剩余字节并释放 SDA，STOP 使总线回到空闲态。
 */
void SOFT_I2C_BusRecover(void)
{
    uint8_t i;
    SOFT_I2C_W_SDA(1); // 释放 SDA
    for (i = 0; i < 9; i++)
    {
        SOFT_I2C_W_SCL(1); // 至多 9 个时钟脉冲
        SOFT_I2C_W_SCL(0);
    }
    SOFT_I2C_Stop(); // 发 STOP，总线回到空闲（SCL/SDA 均为高）
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
