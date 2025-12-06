#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"



/**
 * @brief 更新IMU的欧拉角（姿态角）
 * @param dt 时间间隔（单位：秒），用于积分计算姿态角变化,当前采用定时器获取动态dt
 * @note 通常在定时器中断或主循环中周期性调用，根据陀螺仪和加速度计数据更新pitch/roll/yaw
 */
void Imu_Euler_Angle_Update(float dt);

/**
 * @brief 获取当前计算出的欧拉角数据
 * @param pitch 输出参数：俯仰角（单位：弧度或度，取决于实现）
 * @param roll  输出参数：横滚角（单位：弧度或度，取决于实现）
 * @param yaw   输出参数：偏航角（单位：弧度或度，取决于实现）
 * @note 需要先调用Imu_Euler_Angle_Update()更新数据后再获取
 */
void Imu_Get_Euler_Angle_Data(float *pitch, float *roll, float *yaw);

/**
 * @brief 获取加速度计数据
 * @param ax 输出参数：x轴加速度（单位：通常为m/s2或g）
 * @param ay 输出参数：y轴加速度（单位：通常为m/s2或g）
 * @param az 输出参数：z轴加速度（单位：通常为m/s2或g）
 */
void Imu_Get_Acc_Data(float *ax, float *ay, float *az);

/**
 * @brief 获取陀螺仪数据
 * @param gx 输出参数：x轴角速度（单位：通常为rad/s或°/s）
 * @param gy 输出参数：y轴角速度（单位：通常为rad/s或°/s）
 * @param gz 输出参数：z轴角速度（单位：通常为rad/s或°/s）
 */
void Imu_Get_Gyro_Data(float *gx, float *gy, float *gz);
/**
 * @brief 更新陀螺仪数据
 * @param 无
 */
void Mpu_Data_Update(void);
#endif

