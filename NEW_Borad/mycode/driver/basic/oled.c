#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"

u8 OLED_GRAM[128][8];

void OLED_Refresh_Gram(void)
{
    u8 i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // Set page address (0~7)
        OLED_WR_Byte(0x00, OLED_CMD);     // Set the display location - column low address
        OLED_WR_Byte(0x10, OLED_CMD);     // Set the display location - column height address
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
}

void OLED_WR_Byte(u8 dat, u8 cmd)
{
    u8 i;
    if (cmd)
        OLED_RS_Set();
    else
        OLED_RS_Clr();
    for (i = 0; i < 8; i++)
    {
        OLED_SCLK_Clr();
        if (dat & 0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat <<= 1;
    }
    OLED_RS_Set();
}

void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC command //SET DCDC����
    OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC command //SET DCDC����
    OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}

void OLED_Clear(void)
{
    u8 i, n;
    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0X00;
    OLED_Refresh_Gram(); // Update the display //������ʾ
}

void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
    u8 pos, bx, temp = 0;
    if (x > 127 || y > 63)
        return; // ������Χ��.
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode)
{
    u8 temp, t, t1;
    u8 y0 = y;
    chr = chr - ' '; // Get the offset value //�õ�ƫ�ƺ��ֵ
    for (t = 0; t < size; t++)
    {
        if (size == 12)
            temp = oled_asc2_1206[chr][t]; // Invoke 1206 font   //����1206����
        else
            temp = oled_asc2_1608[chr][t]; // Invoke the 1608 font //����1608����
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

u32 oled_pow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--)
        result *= m;
    return result;
}

void OLED_ShowNumber(u8 x, u8 y, u32 num, u8 len, u8 size)
{
    u8 t, temp;
    u8 enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}

void OLED_ShowString(u8 x, u8 y, const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while (*p != '\0')
    {
        if (x > MAX_CHAR_POSX)
        {
            x = 0;
            y += 16;
        }
        if (y > MAX_CHAR_POSY)
        {
            y = x = 0;
            OLED_Clear();
        }
        OLED_ShowChar(x, y, *p, 12, 1);
        x += 8;
        p++;
    }
}

void OLED_Init(void)
{
    OLED_RST_Clr();
    // delay_ms(100);
    OLED_RST_Set();

    OLED_WR_Byte(0xAE, OLED_CMD); // Close display //�ر���ʾ
    OLED_WR_Byte(0xD5, OLED_CMD); // The frequency frequency factor, the frequency of the shock //����ʱ�ӷ�Ƶ����,��Ƶ��
    OLED_WR_Byte(80, OLED_CMD);   //[3:0], the frequency dividing factor;[7:4], oscillation frequency //[3:0],��Ƶ����;[7:4],��Ƶ��
    OLED_WR_Byte(0xA8, OLED_CMD); // Set the number of driver paths //��������·��
    OLED_WR_Byte(0X3F, OLED_CMD); // Default 0x3f(1/64) //Ĭ��0X3F(1/64)
    OLED_WR_Byte(0xD3, OLED_CMD); // Setting display deviation //������ʾƫ��
    OLED_WR_Byte(0X00, OLED_CMD); // Default is 0//Ĭ��Ϊ0

    OLED_WR_Byte(0x40, OLED_CMD); // Sets the number of rows to display starting line [5:0] //������ʾ��ʼ�� [5:0],����

    OLED_WR_Byte(0x8D, OLED_CMD); // Charge pump setup //��ɱ�����
    OLED_WR_Byte(0x14, OLED_CMD); // Bit2, on/off //bit2������/�ر�
    OLED_WR_Byte(0x20, OLED_CMD); // Set up the memory address mode //�����ڴ��ַģʽ
    OLED_WR_Byte(0x02, OLED_CMD); //[1:0],00, column address mode;01, line address mode;10. Page address mode;The default 10; //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
    OLED_WR_Byte(0xA1, OLED_CMD); // Segment redefine setting,bit0:0,0- >;0;1, 0 - & gt;127; //���ض�������,bit0:0,0->0;1,0->127;
    OLED_WR_Byte(0xC0, OLED_CMD); // Set the COM scan direction;Bit3:0, normal mode;1, Re-define schema COM[n-1]- >;COM0;N: Number of driving paths//����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
    OLED_WR_Byte(0xDA, OLED_CMD); // Set the COM hardware pin configuration //����COMӲ����������
    OLED_WR_Byte(0x12, OLED_CMD); //[5:4]configuration //[5:4]����

    OLED_WR_Byte(0x81, OLED_CMD); // Contrast Settings //�Աȶ�����
    OLED_WR_Byte(0xEF, OLED_CMD); // 1~ 255; Default 0x7f (brightness Settings, the bigger the brighter) //1~255;Ĭ��0X7F (��������,Խ��Խ��)
    OLED_WR_Byte(0xD9, OLED_CMD); // Set the pre-charging cycle //����Ԥ�������
    OLED_WR_Byte(0xf1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
    OLED_WR_Byte(0xDB, OLED_CMD); // Setting vcomh voltage multiplier//����VCOMH ��ѹ����
    OLED_WR_Byte(0x30, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

    OLED_WR_Byte(0xA4, OLED_CMD); // Global display; Bit0:1, open; 0, close; (white screen/black screen)//ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
    OLED_WR_Byte(0xA6, OLED_CMD); // Settings display mode; Bit0:1, anti-phase display; 0, normal display//������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ
    OLED_WR_Byte(0xAF, OLED_CMD); // Open display //������ʾ
    OLED_Clear();
}

void OLED_ShowCHinese(u8 x, u8 y, u8 no, u8 font_width, u8 font_height)
{
    u8 t, i;
    for (i = 0; i < (font_height / 8); i++) // The maximum height of font_height is 32. this screen is only 8 pages (line), four digits per page
                                            // font_height���ֵΪ32��������ֻ��8��ҳ���У���ÿҳ4��λ
    {
        OLED_Set_Pos(x, y + i);
        for (t = 0; t < font_width; t++) // The maximum value of font_width is 128. the screen is only that large
                                         // font_width���ֵΪ128����Ļֻ����ô��
        {
            OLED_WR_Byte(Hzk16[(font_height / 8) * no + i][t], OLED_DATA);
        }
    }
}

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

void test_oled()
{

    /************************** 1. OLED初始化（必须第一步） **************************/
    OLED_Init();    // 初始化OLED（含清屏、开启显示）
    delay_ms(1000); // 延时1秒，等待OLED上电稳定

    /************************** 2. 测试1：显示单个ASCII字符（不同字体） **************************/
    OLED_Clear(); // 清屏，避免残留内容
    // 显示12号字体（oled_asc2_1206字库）：x=坐标，y=坐标，字符，字体大小，显示模式（1=正常）
    OLED_ShowChar(10, 10, 'A', 12, 1); // (10,10)显示'A'（12号）
    OLED_ShowChar(40, 10, 'B', 12, 1); // (40,10)显示'B'（12号）
    // 显示16号字体（oled_asc2_1608字库）
    OLED_ShowChar(70, 10, '1', 16, 1);  // (70,10)显示'1'（16号）
    OLED_ShowChar(100, 10, '2', 16, 1); // (100,10)显示'2'（16号）

    OLED_Refresh_Gram(); // 关键！修改显存后必须刷新才能显示
    delay_ms(2000);      // 停留2秒，观察字符效果

    /************************** 3. 测试2：显示字符串（自动换行处理） **************************/
    OLED_Clear(); // 清屏
    // 显示字符串（依赖oled_asc2_1206字库，12号字体）
    OLED_ShowString(0, 0, (u8 *)"OLED SPI Test");              // 第1行（y=0）："OLED SPI Test"
    OLED_ShowString(0, 16, (u8 *)"Char: A B 1 2");             // 第2行（y=16）：字符测试提示
    OLED_ShowString(0, 32, (u8 *)"Long String: Hello World!"); // 长字符串（自动换行）
    OLED_ShowString(0, 48, (u8 *)"OK!");                       // 第4行（y=48）：状态提示

    OLED_Refresh_Gram(); // 刷新显示
    delay_ms(2500);      // 停留2.5秒，观察字符串效果

    /************************** 4. 测试3：显示无符号数字（不同位数/字体） **************************/
    OLED_Clear();                                // 清屏
    OLED_ShowString(0, 0, (u8 *)"Number Test:"); // 提示文字（第1行）

    // 显示数字：x=坐标，y=坐标，数字，位数，字体大小
    OLED_ShowNumber(0, 16, 1234, 4, 12);   // (0,16)显示1234（4位，12号字体）
    OLED_ShowNumber(60, 16, 56789, 5, 12); // (60,16)显示56789（5位，12号字体）
    OLED_ShowNumber(0, 32, 9876, 4, 16);   // (0,32)显示9876（4位，16号字体）
    OLED_ShowNumber(60, 32, 100, 3, 16);   // (60,32)显示100（3位，16号字体，自动补0）

    OLED_Refresh_Gram(); // 刷新显示
    delay_ms(3000);      // 停留3秒，观察数字效果

    /************************** 6. 测试5：绘制图形（点、线） **************************/
    OLED_Clear();                              // 清屏
    OLED_ShowString(0, 0, (u8 *)"Draw Test:"); // 提示文字（第1行）

    // 6.1 画4个点（组成小方块）
    OLED_DrawPoint(20, 20, 1); // (20,20)画点（1=填充）
    OLED_DrawPoint(25, 20, 1); // (25,20)画点
    OLED_DrawPoint(20, 25, 1); // (20,25)画点
    OLED_DrawPoint(25, 25, 1); // (25,25)画点

    // 6.2 画水平线（x从40→100，y=20）
    for (u8 i = 40; i <= 100; i++)
    {
        OLED_DrawPoint(i, 20, 1);
    }

    // 6.3 画垂直线（x=70，y从25→45）
    for (u8 i = 25; i <= 45; i++)
    {
        OLED_DrawPoint(70, i, 1);
    }

    OLED_Refresh_Gram(); // 刷新显示
    delay_ms(3000);      // 停留3秒，观察图形效果

    /************************** 7. 测试6：动态循环显示（验证刷新功能） **************************/
    u32 count = 0; // 动态计数变量（0~99999循环）
    while (1)
    {
        OLED_Clear(); // 清屏避免计数重叠（也可局部清屏，此处简化用全清）

        // 显示固定提示文字
        OLED_ShowString(0, 0, (u8 *)"Dynamic Count:");  // 第1行提示
        OLED_ShowString(0, 40, (u8 *)"Refresh: 500ms"); // 第4行提示

        // 显示当前计数（5位，16号字体，超过5位重置为0）
        OLED_ShowNumber(0, 20, count, 5, 16);

        // 刷新显示并更新计数
        OLED_Refresh_Gram();
        count++;
        if (count > 99999)
            count = 0; // 5位计数满后重置

        delay_ms(500); // 500ms刷新一次，肉眼可清晰观察
    }
}

// 显示二进制数（len为二进制位数，如8/16/32，需确保num不超过该位数能表示的范围）
void OLED_ShowBinary(u8 x, u8 y, u32 num, u8 len, u8 size)
{
    u8 t, temp;
    for (t = 0; t < len; t++)
    {
        // 提取第t位二进制数（从高位到低位）：右移(len-t-1)位后与1，得到0或1
        temp = (num >> (len - t - 1)) & 1;
        // 直接显示0或1（二进制无需跳过前导0，全部显示）
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}
