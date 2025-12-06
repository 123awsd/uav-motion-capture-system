#include "bmp280.h"
#include "IIC.h"
#include "main.h"  // 用于HAL_Delay

#include <math.h>

// 全局变量：存储最新的测量结果（缓存）
int32_t press_Val = 0;       // 气压值（Pa）
int32_t temp_Val = 0;        // 温度值（℃×100）
float   alt_Val = 0.0f;      // 绝对高度值（m）
float   relative_alt_Val = 0.0f;  // 相对高度值（m）
float   alt_reference = 0.0f;     // 参考点高度（初始化为0m的位置）
uint8_t has_reference = 0;        // 参考点是否已设置（0=未设置，1=已设置）

// 校准参数
unsigned short dig_T1;
short dig_T2;
short dig_T3;
unsigned short dig_P1;
short dig_P2;
short dig_P3;
short dig_P4;
short dig_P5;
short dig_P6;
short dig_P7;
short dig_P8;
short dig_P9;
int32_t t_fine;

// 软件微秒延时
void Simple_Delay_us(uint32_t us)
{
    uint32_t i;
    for (i = 0; i < us * 8; i++)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
}

// IIC读写函数（保持不变）
uint8_t bmp280_ReadByte(uint8_t addr)
{
    uint8_t temp;
    IIC_Start();
    IIC_Send_Byte(0xEC);
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(0xED);
    IIC_Wait_Ack();
    temp = IIC_Read_Byte(0);
    IIC_NAck();
    IIC_Stop();
    return temp;
}

void bmp280_WriteByte(uint8_t addr, uint8_t dat)
{
    IIC_Start();
    IIC_Send_Byte(0xEC);
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();
    IIC_Send_Byte(dat);
    IIC_Wait_Ack();
    IIC_Stop();
    Simple_Delay_us(10);
}

uint16_t bmp280_MultipleReadTwo(uint8_t addr)
{
    uint8_t msb, lsb;
    IIC_Start();
    IIC_Send_Byte(0xEC);
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(0xED);
    IIC_Wait_Ack();
    lsb = IIC_Read_Byte(1);
    msb = IIC_Read_Byte(0);
    IIC_Stop();
    return ((uint16_t)msb << 8) | lsb;
}

uint32_t bmp280_MultipleReadThree(uint8_t addr)
{
    uint8_t msb, lsb, xlsb;
    IIC_Start();
    IIC_Send_Byte(0xEC);
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(0xED);
    IIC_Wait_Ack();
    msb = IIC_Read_Byte(1);
    lsb = IIC_Read_Byte(1);
    xlsb = IIC_Read_Byte(0);
    IIC_Stop();
    return ((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | ((xlsb >> 4) & 0x0F);
}

// 初始化函数
void bmp280_Init(void)
{
    IIC_GPIO_Init();
    Simple_Delay_us(100);
    
    // 复位传感器
    bmp280_WriteByte(0xE0, 0xB6);
    HAL_Delay(5);
    
    // 配置测量模式
    bmp280_WriteByte(0xF4, 0xFF);
    bmp280_WriteByte(0xF5, 0x00);
    HAL_Delay(10);
    
    // 读取校准参数
    dig_T1 = bmp280_MultipleReadTwo(0x88);
    dig_T2 = bmp280_MultipleReadTwo(0x8A);
    dig_T3 = bmp280_MultipleReadTwo(0x8C);
    dig_P1 = bmp280_MultipleReadTwo(0x8E);
    dig_P2 = bmp280_MultipleReadTwo(0x90);
    dig_P3 = bmp280_MultipleReadTwo(0x92);
    dig_P4 = bmp280_MultipleReadTwo(0x94);
    dig_P5 = bmp280_MultipleReadTwo(0x96);
    dig_P6 = bmp280_MultipleReadTwo(0x98);
    dig_P7 = bmp280_MultipleReadTwo(0x9A);
    dig_P8 = bmp280_MultipleReadTwo(0x9C);
    dig_P9 = bmp280_MultipleReadTwo(0x9E);
    
    HAL_Delay(20);
    
    // 初始化时未设置参考点
    has_reference = 0;
    alt_reference = 0.0f;
    relative_alt_Val = 0.0f;
}

// 温度获取（更新缓存）
int32_t bmp280_GetTemperature(void)
{
    int32_t adc_T = bmp280_MultipleReadThree(0xFA);
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    temp_Val = T;  // 更新温度缓存
    return T;
}

// 气压获取（更新缓存）
int32_t bmp280_GetPressure(void)
{
    int32_t adc_P = bmp280_MultipleReadThree(0xF7);
    int64_t var1, var2, p;
    
    bmp280_GetTemperature();  // 确保t_fine更新
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    
    press_Val = (int32_t)(p >> 8);  // 更新气压缓存
    return press_Val;
}

// 绝对高度获取（更新缓存）
float bmp280_GetAltitude(void)
{
    const float sea_level_pressure = 101325.0f;
    float pressure = (float)bmp280_GetPressure();  // 确保气压缓存更新
    
    alt_Val = 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 1.0f / 5.255f));
    
    // 如果已设置参考点，同时更新相对高度
    if (has_reference)
    {
        relative_alt_Val = alt_Val - alt_reference;
    }
    
    return alt_Val;
}

// 新增：设置当前位置为相对高度参考点（0m）
void bmp280_SetRelativeZero(void)
{
    // 获取当前绝对高度作为参考点
    alt_reference = bmp280_GetAltitude();
	printf("%.2f      ",alt_reference);
    // 标记参考点已设置
    has_reference = 1;
    // 初始化相对高度为0
    relative_alt_Val = 0.0f;
}

// 新增：获取相对高度（相对于参考点）
float bmp280_GetRelativeAltitude(void)
{
    // 如果未设置参考点，返回0并提示需要先校准
    if (!has_reference)
    {
        return 0.0f;
    }
    
    // 刷新绝对高度，内部会自动更新相对高度
    bmp280_GetAltitude();
    
    return relative_alt_Val;
}

// 缓存读取函数
int32_t bmp280_Read_Pressure(void)
{
    return press_Val;
}

int32_t bmp280_Read_Temperature(void)
{
    return temp_Val;
}

float bmp280_Read_Altitude(void)
{
    return alt_Val;
}

// 新增：读取缓存的相对高度
float bmp280_Read_RelativeAltitude(void)
{
    return relative_alt_Val;
}







