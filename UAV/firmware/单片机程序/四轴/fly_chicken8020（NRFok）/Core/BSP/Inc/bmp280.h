#ifndef __BMP280_H
#define __BMP280_H
#include "main.h"
// 设备和寄存器定义
#define BMP280_ADDR         0xEC
#define BMP280_TEMP_ADDR    0xFA
#define BMP280_PRESS_ADDR   0xF7
#define BMP280_REG_RESET    0xE0
#define BMP280_RESET_CMD    0xB6

// 全局变量声明
extern int32_t press_Val;
extern int32_t temp_Val;
extern float   alt_Val;
extern float   relative_alt_Val;
extern float   alt_reference;
extern uint8_t has_reference;
extern int32_t t_fine;
extern unsigned short dig_T1;
extern short dig_T2;
extern short dig_T3;
extern unsigned short dig_P1;
extern short dig_P2;
extern short dig_P3;
extern short dig_P4;
extern short dig_P5;
extern short dig_P6;
extern short dig_P7;
extern short dig_P8;
extern short dig_P9;

// 函数声明
void bmp280_Init(void);  // 初始化BMP280传感器
uint8_t bmp280_ReadByte(uint8_t addr);  // 读取单个寄存器值
void bmp280_WriteByte(uint8_t addr, uint8_t dat);  // 写入单个寄存器值
uint16_t bmp280_MultipleReadTwo(uint8_t addr);  // 连续读取两个字节
uint32_t bmp280_MultipleReadThree(uint8_t addr);  // 连续读取三个字节

// 带计算的获取函数（更新缓存）
int32_t bmp280_GetTemperature(void);  // 获取温度值（更新缓存）
int32_t bmp280_GetPressure(void);  // 获取气压值（更新缓存）
float bmp280_GetAltitude(void);  // 获取绝对高度（更新缓存）

// 相对高度功能
void bmp280_SetRelativeZero(void);  // 设置当前位置为相对高度0点
float bmp280_GetRelativeAltitude(void);  // 获取相对于参考点的高度

// 缓存读取函数
int32_t bmp280_Read_Pressure(void);  // 读取缓存的气压值
int32_t bmp280_Read_Temperature(void);  // 读取缓存的温度值
float bmp280_Read_Altitude(void);  // 读取缓存的绝对高度
float bmp280_Read_RelativeAltitude(void);  // 读取缓存的相对高度

// 延时函数
void Simple_Delay_us(uint32_t us);  // 软件微秒级延时

#endif /* __BMP280_H */




