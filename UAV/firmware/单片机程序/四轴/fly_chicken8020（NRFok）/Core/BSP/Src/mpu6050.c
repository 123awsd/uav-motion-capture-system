#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include "IIC.h"
#include "main.h"
#include <math.h>
#include <stdbool.h>

/******************************************************
 *  改进说明（2025-11-03）
 *  1) 采样率: 1 kHz（SMPLRT_DIV=0），与 DLPF 解耦配置
 *  2) DLPF: 98 Hz（可按噪声调到 42/188 Hz）
 *  3) 读取方式: 14 字节突发读（ax..gz 一次取完）
 *  4) 单位: 输出为 SI 制（rad/s, m/s^2, °C）
 *  5) 校准: 修复加速度归一计算、陀螺零偏平均
 *  6) 初始化逻辑修复: do{ }while(err) 而不是 while(ERR) 未初始化
 *  7) 删除“采样率自动把 LPF=rate/2”的耦合，改为显式设置
 ******************************************************/

/* === 原代码中的换算常量改为 SI 单位输出 ===
 * 原:
 *   Gyro_Val_Unit = (65536/2)/2000 = 16.384 LSB/dps
 *   Acc_Val_Unit  = (65536/2)/2    = 16384 LSB/g
 * 现:
 *   我们直接把原始值换成 rad/s 和 m/s^2
 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GYRO_LSB_PER_DPS   (16.384f)      // 2000 dps 满量程
#define ACC_LSB_PER_G      (16384.0f)     // 2 g 满量程
#define DEG2RAD            (M_PI/180.0f)
#define ONE_G              (9.80665f)

// 公开的比例常数（如需兼容旧接口，可保留，但当前未使用）
const float Gyro_Val_Unit = GYRO_LSB_PER_DPS;  // LSB per dps
const float Acc_Val_Unit  = ACC_LSB_PER_G;     // LSB per g
const float T_Val_Unit    = 100.f;             // 摄氏度*100（仅用于旧接口示例）

/******************** 本地静态函数声明 ********************/
static uint8_t MPU_SoftResetAndWake(void);
static uint8_t MPU_ConfigFSR(void);
static uint8_t MPU_ConfigSampling(uint16_t odr_hz, uint16_t dlpf_hz);
static uint8_t MPU_Read_All_Raw(short *ax, short *ay, short *az,
                                short *gx, short *gy, short *gz,
                                short *temp);

/******************** 校准与数据缓存 ********************/
MPU_ACC_GYRO_DATA_STR   Mpu_Err;              // 零偏与静态重力期望
float Acc_ERR_Proportion  = 1.0f;             // 加速度尺度归一系数
bool Check_Flag = false;                       // 启用误差修正标志

/******************** MPU 基础寄存器配置 ******************/
// 初始化MPU6050
// 返回值: 0 成功; 其他 错误码
uint8_t MPU_Init(void)
{
    uint8_t res;

    MPU_IIC_Init();                 // 初始化 I2C 总线

    res = MPU_SoftResetAndWake();
    if (res) return res;

    res = MPU_ConfigFSR();          // ±2000 dps, ±2 g
    if (res) return res;

    // 1 kHz 采样, DLPF 98 Hz（可按机架噪声改 42/188）
    res = MPU_ConfigSampling(1000, 98);
    if (res) return res;

    // 关闭中断/FIFO/主 I2C
    if (MPU_Write_Byte(MPU_INT_EN_REG,   0x00)) return 1;  // 关所有中断
    if (MPU_Write_Byte(MPU_USER_CTRL_REG,0x00)) return 2;  // 关闭主模式/I2C Master
    if (MPU_Write_Byte(MPU_FIFO_EN_REG,  0x00)) return 3;  // 关闭 FIFO
    if (MPU_Write_Byte(MPU_INTBP_CFG_REG,0x80)) return 4;  // INT 低电平有效

    // 读 ID
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res != MPU_ADDR) return 5;

    // 选择时钟源：PLL X 轴
    if (MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x01)) return 6;
    if (MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00)) return 7;

    return 0;
}

// 复位并唤醒
static uint8_t MPU_SoftResetAndWake(void)
{
    if (MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80)) return 1; // 软复位
    HAL_Delay(100);
    if (MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00)) return 2; // 唤醒
    HAL_Delay(10);
    return 0;
}

// 满量程范围配置：陀螺 ±2000 dps，加计 ±2 g
static uint8_t MPU_ConfigFSR(void)
{
    if (MPU_Set_Gyro_Fsr(3))  return 1;  // 3 -> ±2000 dps
    if (MPU_Set_Accel_Fsr(0)) return 2;  // 0 -> ±2 g
    return 0;
}

// 采样率与 DLPF 显式配置（解耦）
// odr_hz: 采样率(Hz)，典型 1000/500/250
// dlpf_hz: DLPF 目标带宽，映射到芯片档位（5/10/20/42/98/188/256）
static uint8_t MPU_ConfigSampling(uint16_t odr_hz, uint16_t dlpf_hz)
{
    // DLPF 档位
    uint8_t dlpf_cfg = 0; // 0: 256Hz(或 260)，后续根据表选择
    if      (dlpf_hz >= 188) dlpf_cfg = 1;
    else if (dlpf_hz >= 98)  dlpf_cfg = 2;
    else if (dlpf_hz >= 42)  dlpf_cfg = 3;
    else if (dlpf_hz >= 20)  dlpf_cfg = 4;
    else if (dlpf_hz >= 10)  dlpf_cfg = 5;
    else                     dlpf_cfg = 6; // ~5 Hz

    // 写 DLPF（CONFIG）
    if (MPU_Write_Byte(MPU_CFG_REG, dlpf_cfg)) return 1;

    // 采样率分频：当 DLPF!=0 时，内部基准为 1 kHz
    if (odr_hz > 1000) odr_hz = 1000;
    if (odr_hz < 4)    odr_hz = 4;
    uint8_t smplrt_div = (uint8_t)(1000/odr_hz - 1);
    if (MPU_Write_Byte(MPU_SAMPLE_RATE_REG, smplrt_div)) return 2;

    return 0;
}

// === 兼容保留：原始接口 ===
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{ return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr<<3); }

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{ return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr<<3); }

uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if      (lpf>=188) data=1;
    else if (lpf>=98)  data=2;
    else if (lpf>=42)  data=3;
    else if (lpf>=20)  data=4;
    else if (lpf>=10)  data=5;
    else               data=6;
    return MPU_Write_Byte(MPU_CFG_REG,data);
}

// 仅设置采样率（取消"自动把 LPF=rate/2"的耦合）
uint8_t MPU_Set_Rate(uint16_t rate)
{
    if (rate>1000) rate=1000;
    if (rate<4)    rate=4;
    uint8_t div = (uint8_t)(1000/rate - 1);
    return MPU_Write_Byte(MPU_SAMPLE_RATE_REG, div);
}

/******************** 数据读取（优化：14字节突发） ********************/
static uint8_t MPU_Read_All_Raw(short *ax, short *ay, short *az,
                                short *gx, short *gy, short *gz,
                                short *temp)
{
    uint8_t buf[14];
    uint8_t res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 14, buf);
    if (res) return res;

    // ACCEL
    *ax = (short)((buf[0]  << 8) | buf[1]);
    *ay = (short)((buf[2]  << 8) | buf[3]);
    *az = (short)((buf[4]  << 8) | buf[5]);
    // TEMP
    if (temp) *temp = (short)((buf[6]  << 8) | buf[7]);
    // GYRO
    *gx = (short)((buf[8]  << 8) | buf[9]);
    *gy = (short)((buf[10] << 8) | buf[11]);
    *gz = (short)((buf[12] << 8) | buf[13]);

    return 0;
}

// （如需保留旧单项读取接口，仍提供，但建议上层改为一次读完）
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    short ax, ay, az, t;
    return MPU_Read_All_Raw(&ax,&ay,&az,gx,gy,gz,&t);
}

uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    short gx, gy, gz, t;
    return MPU_Read_All_Raw(ax,ay,az,&gx,&gy,&gz,&t);
}

short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = (short)((buf[0] << 8) | buf[1]);
    temp = 36.53f + ((float)raw)/340.0f; // °C
    return (short)(temp * 100.0f);
}

/******************** I2C 读写（保持原实现） ********************/
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|0);
    if(MPU_IIC_Wait_Ack()){ MPU_IIC_Stop(); return 1; }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    for(i=0; i<len; i++){
        MPU_IIC_Send_Byte(buf[i]);
        if(MPU_IIC_Wait_Ack()){ MPU_IIC_Stop(); return 1; }
    }
    MPU_IIC_Stop();
    return 0;
}

uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|0);
    if(MPU_IIC_Wait_Ack()){ MPU_IIC_Stop(); return 1; }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|1);
    MPU_IIC_Wait_Ack();
    while(len){
        if(len==1)*buf=MPU_IIC_Read_Byte(0);
        else       *buf=MPU_IIC_Read_Byte(1);
        len--; buf++;
    }
    MPU_IIC_Stop();
    return 0;
}

uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);
    if(MPU_IIC_Wait_Ack()){ MPU_IIC_Stop(); return 1; }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(data);
    if(MPU_IIC_Wait_Ack()){ MPU_IIC_Stop(); return 1; }
    MPU_IIC_Stop();
    return 0;
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);
    MPU_IIC_Wait_Ack();
    res=MPU_IIC_Read_Byte(0);
    MPU_IIC_Stop();
    return res;
}

/******************** 新增：一站式获取（SI 单位 + 校准） ********************/
// 快速平方根倒数（若你已有实现，可替换）
static inline float InvSqrt_fast(float x)
{
    return 1.0f / sqrtf(x);
}

/**
  * @brief 获取 MPU 数据（一次读取 + 零偏/尺度修正 + SI 单位）
  * @return MPU_ACC_GYRO_DATA_STR，字段单位：
  *         gx,gy,gz -> rad/s； ax,ay,az -> m/s^2
  * @note   典型执行时间（软件 I2C）: < 800 us
  */
MPU_ACC_GYRO_DATA_STR My_Get_MPU_Data()
{
    float Acc_Sum;
    MPU_ACC_GYRO_DATA_STR out = {0};
    short gx, gy, gz, ax, ay, az, t;

    MPU_Read_All_Raw(&ax,&ay,&az,&gx,&gy,&gz,&t);

    // 原始 -> SI 单位
    float gx_dps = ((float)gx) / GYRO_LSB_PER_DPS;     // dps
    float gy_dps = ((float)gy) / GYRO_LSB_PER_DPS;
    float gz_dps = ((float)gz) / GYRO_LSB_PER_DPS;

		out.gx = ((float)gx) / GYRO_LSB_PER_DPS;   // deg/s
		out.gy = ((float)gy) / GYRO_LSB_PER_DPS;   // deg/s
		out.gz = ((float)gz) / GYRO_LSB_PER_DPS;   // deg/s

    float ax_g = ((float)ax) / ACC_LSB_PER_G;          // g
    float ay_g = ((float)ay) / ACC_LSB_PER_G;
    float az_g = ((float)az) / ACC_LSB_PER_G;

    out.ax = ax_g * ONE_G;                             // m/s^2
    out.ay = ay_g * ONE_G;
    out.az = az_g * ONE_G;

    if(Check_Flag){
        // 陀螺零偏
        out.gx -= Mpu_Err.gx;
        out.gy -= Mpu_Err.gy;
        out.gz -= Mpu_Err.gz;

        // 加计偏置
        out.ax -= Mpu_Err.ax;
        out.ay -= Mpu_Err.ay;
        out.az -= Mpu_Err.az;

        // 加计尺度归一（把 |a| 调整到 ≈ 1g）
        Acc_Sum = out.ax*out.ax + out.ay*out.ay + out.az*out.az; // (m/s^2)^2
        if (Acc_Sum > 1e-6f){
            Acc_ERR_Proportion = InvSqrt_fast(Acc_Sum / (ONE_G*ONE_G)); // -> 1/|a|_g
            out.ax *= Acc_ERR_Proportion;
            out.ay *= Acc_ERR_Proportion;
            out.az *= Acc_ERR_Proportion;
        }
    }

    return out;
}

/******************** 校准：静置平均 ********************/
void Mpu_Gyro_Get_ERR(uint16_t Num)
{
    MPU_ACC_GYRO_DATA_STR sum = {0};
    for(uint16_t i=0;i<Num;i++){
        MPU_ACC_GYRO_DATA_STR d = My_Get_MPU_Data();
        sum.gx += d.gx; sum.gy += d.gy; sum.gz += d.gz;
        HAL_Delay(2); // 按 500 Hz 读取时适当延时
    }
    Mpu_Err.gx = sum.gx / Num;
    Mpu_Err.gy = sum.gy / Num;
    Mpu_Err.gz = sum.gz / Num;
//		
//		    Mpu_Err.gx = 0;
//    Mpu_Err.gy = 0;
//    Mpu_Err.gz = 0;
}

void Mpu_Acc_Get_ERR(uint16_t Num)
{
    MPU_ACC_GYRO_DATA_STR sum = {0};
    for(uint16_t i=0;i<Num;i++){
        MPU_ACC_GYRO_DATA_STR d = My_Get_MPU_Data();
        sum.ax += d.ax; sum.ay += d.ay; sum.az += d.az;
        HAL_Delay(2);
    }
    // 期望水平静止：[0,0,1g]
    Mpu_Err.ax = sum.ax/Num - 0.0f;
    Mpu_Err.ay = sum.ay/Num - 0.0f;
    Mpu_Err.az = sum.az/Num - ONE_G;
		
//		Mpu_Err.ax =  0.0f;
//    Mpu_Err.ay =  0.0f;
//    Mpu_Err.az = 0;
}

/******************** 统一初始化入口 ********************/
void My_Mpu_Init()
{
    uint8_t ERR;
    do{
        ERR = MPU_Init();
        if(ERR) printf("MPU_ERR %d\r\n", ERR);
    }while(ERR);

    HAL_Delay(500); // 热机/静置

    // 在静止条件下做零偏与加计期望
    Mpu_Gyro_Get_ERR(200);
    Mpu_Acc_Get_ERR(200);

    Check_Flag = true;
    printf("MPU 初始化成功 (1kHz/98Hz DLPF, SI 单位)！\r\n");
}
