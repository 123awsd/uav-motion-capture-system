#include "imu.h"
// 具体计算过程查看笔记
// obsidian://open?vault=Obsidian%20Vault&file=%E6%9A%91%E5%81%87%E5%B0%8F%E9%A3%9E%E6%9C%BA%E8%AE%A1%E5%88%92%2F%E5%A7%BF%E6%80%81%E8%A7%A3%E7%AE%97

// 便于更改更高效的三角函数运算
#define   Imu_Sin         sinf
#define   Imu_Asin        asinf
#define   Imu_Cos         cosf
#define   Imu_Acos        acosf
#define   Imu_Tan         tanf
#define   Imu_Atan        atanf
#define   Imu_Atan2       atan2f

#define   Imu_Gyro_z_min  0.3f // 陀螺仪z轴最小值，小于则视为0（抑制低速漂移）
#define   Imu_Gyro_y_min  0.3f // 陀螺仪y轴最小值
// 你原来把 Square_root 定义为 InvSqrt（名字和功能可能不一致）。此处避免歧义，直接用 sqrtf。
// #define   Square_root     InvSqrt

/* 欧拉角结构体 */
typedef struct{
    float yaw;   // 航向角 (deg)
    float pitch; // 俯仰角 (deg)
    float roll;  // 横滚角 (deg)
} IMU_EULER_ANGLE_STR;

/* 四元数结构体（w,x,y,z） */
typedef struct{
    float q0; // w
    float q1; // x
    float q2; // y
    float q3; // z
} IMU_QUATERNION_STR;

/* ====================== 全局/静态状态 ====================== */
static MPU_ACC_GYRO_DATA_STR   mpu_data;                   // 原始 IMU 数据
static IMU_EULER_ANGLE_STR     K_Euler_angle = {0,0,0};    // 输出欧拉角（融合后）
static IMU_QUATERNION_STR      K_Quat = {1,0,0,0};         // 姿态四元数（机体→地球）

// Mahony 参数（可按实际调参）
static float Mahony_Kp = 2.0f;  // 比例增益（响应速度/稳态误差）
static float Mahony_Ki = 0.0f;  // 积分增益（抑制零偏），先设0以避免积分饱和

// 积分项（用于补偿陀螺零偏）
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

static inline float radians_to_deg(float rad){ return rad * (180.0f / My_Pi); }
static inline float deg_to_radians(float deg){ return deg * (My_Pi / 180.0f); }

// 角度封装到 [0, 360)
static inline float angle_wrap_360(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg <    0.0f) deg += 360.0f;
    return deg;
}

// 四元数归一化
static inline void quat_normalize(IMU_QUATERNION_STR *q)
{
    float n = q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3;
    if(n > 0.0f){
        float inv = 1.0f / sqrtf(n);
        q->q0 *= inv; q->q1 *= inv; q->q2 *= inv; q->q3 *= inv;
    }else{
        q->q0 = 1.0f; q->q1 = q->q2 = q->q3 = 0.0f;
    }
}

// 四元数微分：q_dot = 0.5 * q ? [0, wx, wy, wz]
static inline void quat_derivative(const IMU_QUATERNION_STR *q, float wx, float wy, float wz,
                                   float *dq0, float *dq1, float *dq2, float *dq3)
{
    float half = 0.5f;
    *dq0 = (-q->q1*wx - q->q2*wy - q->q3*wz) * half;
    *dq1 = ( q->q0*wx + q->q2*wz - q->q3*wy) * half;
    *dq2 = ( q->q0*wy - q->q1*wz + q->q3*wx) * half;
    *dq3 = ( q->q0*wz + q->q1*wy - q->q2*wx) * half;
}

// 由四元数得到欧拉角（Z-Y-X：yaw-pitch-roll），输出单位：度
static void quat_to_euler_deg(const IMU_QUATERNION_STR *q, float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    // 参考常用公式（右手、主体→地球）
    float q0=q->q0, q1=q->q1, q2=q->q2, q3=q->q3;

    // roll (x)
    float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
    float roll = Imu_Atan2(sinr_cosp, cosr_cosp);

    // pitch (y)
    float sinp = 2.0f * (q0*q2 - q3*q1);
    float pitch;
    if (sinp >= 1.0f) pitch =  My_Pi / 2.0f;
    else if (sinp <= -1.0f) pitch = -My_Pi / 2.0f;
    else pitch = Imu_Asin(sinp);

    // yaw (z)
    float siny_cosp = 2.0f * (q0*q3 + q1*q2);
    float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
    float yaw = Imu_Atan2(siny_cosp, cosy_cosp);

    *roll_deg  = radians_to_deg(roll);
    *pitch_deg = radians_to_deg(pitch);
    *yaw_deg   = angle_wrap_360(radians_to_deg(yaw)); // 航向角做 0~360° 封装
}

/* ====================== 你原来保留的全局量（若无用也可保留以兼容） ====================== */
//（以下矩阵在 Mahony 里不再使用，但保留可避免你项目里其他地方的依赖/编译报错）
float P_k[2][2] = { {1,0}, {0,1} };
float Q_e[2][2] = { {0.01f,0}, {0,0.01f} };
float R_e[2][2] = { {0.03f,0}, {0,0.03f} };
float K_k[2][2] = { {0,0}, {0,0} };

float angle_to_radians  = (My_Pi/180.f);
float radians_to_angle  = (180.f/My_Pi) ;

// 测试动态dt
u16  dt_test = 0;

/**
  * @brief  Mahony：四元数积分 + 加计重力矢量修正（无欧拉奇异）
  * @param  dt 为执行一次的时间（ms）
  * @note   输出欧拉角单位：度；内部惯导单位：rad/s
  */
void Imu_Euler_Angle_Update(float dt)
{
    // ========= 1) 采样周期 =========
    // 动态计时（与你原有的一致）
    dt_test = TIM4->CNT;
    TIM4->CNT = 0;
    dt = (float)dt_test / 1000.0f; // ms

    // 若你想传入固定 dt，可注释上面三行，并把 dt 视为 ms
    dt = dt / 1000.0f; // 转成 s

    // ========= 2) 读取原始 IMU =========
    mpu_data = My_Get_MPU_Data();

    // 可选的低速去噪（保持你原宏定义）
    MPU_ACC_GYRO_DATA_STR gyro = mpu_data;
    if (ABS(gyro.gz) < Imu_Gyro_z_min) gyro.gz = 0.0f;
    if (ABS(gyro.gy) < Imu_Gyro_y_min) gyro.gy = 0.0f;

    // 陀螺是 deg/s? 若是，需转为 rad/s；若已是 rad/s，请删去这两行
    float gx = deg_to_radians(gyro.gx);
    float gy = deg_to_radians(gyro.gy);
    float gz = deg_to_radians(gyro.gz);

    // ========= 3) 加计单位化（重力方向观测）=========
    float ax = mpu_data.ax, ay = mpu_data.ay, az = mpu_data.az;
    float anorm = sqrtf(ax*ax + ay*ay + az*az);
    if (anorm > 1e-6f) {
        ax /= anorm; ay /= anorm; az /= anorm;
    } else {
        // 遇到异常时跳过修正，只做纯陀螺积分
        anorm = 0.0f;
    }

    // ========= 4) 由当前四元数估计重力方向 g_est (机体系) =========
    // Mahony/Madgwick 常用表达：g_est = [2(q1q3 - q0q2), 2(q0q1 + q2q3), (q0^2 - q1^2 - q2^2 + q3^2)]
    float q0=K_Quat.q0, q1=K_Quat.q1, q2=K_Quat.q2, q3=K_Quat.q3;
    float gx_est = 2.0f*(q1*q3 - q0*q2);
    float gy_est = 2.0f*(q0*q1 + q2*q3);
    float gz_est = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // ========= 5) 误差 = 加计方向 与 估计重力方向 的叉乘 =========
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    if (anorm > 0.0f) {
        // e = a_norm × g_est
        ex = (ay*gz_est - az*gy_est);
        ey = (az*gx_est - ax*gz_est);
        ez = (ax*gy_est - ay*gx_est);

        // 积分零偏项
        if (Mahony_Ki > 0.0f) {
            exInt += Mahony_Ki * ex * dt;
            eyInt += Mahony_Ki * ey * dt;
            ezInt += Mahony_Ki * ez * dt;
        } else {
            exInt = eyInt = ezInt = 0.0f;
        }

        // 对陀螺进行比例+积分修正
        gx += Mahony_Kp * ex + exInt;
        gy += Mahony_Kp * ey + eyInt;
        gz += Mahony_Kp * ez + ezInt;
    }

    // ========= 6) 四元数积分 =========
    float dq0, dq1, dq2, dq3;
    quat_derivative(&K_Quat, gx, gy, gz, &dq0, &dq1, &dq2, &dq3);

    K_Quat.q0 += dq0 * dt;
    K_Quat.q1 += dq1 * dt;
    K_Quat.q2 += dq2 * dt;
    K_Quat.q3 += dq3 * dt;

    quat_normalize(&K_Quat);

    // ========= 7) 四元数 → 欧拉角(度) =========
    quat_to_euler_deg(&K_Quat, &K_Euler_angle.yaw, &K_Euler_angle.pitch, &K_Euler_angle.roll);

    // （可选）如果你更习惯把航向输出到 [-180,180)，可在此改封装
    // K_Euler_angle.yaw = angle_wrap_180(K_Euler_angle.yaw);

    // 调试输出保留（如需）
    // printf("EULER deg: pitch=%f, roll=%f, yaw=%f\r\n",
    //         K_Euler_angle.pitch, K_Euler_angle.roll, K_Euler_angle.yaw);
}

/* ====================== Getter 接口（保持原样） ====================== */
//mpu数据更新
void Mpu_Data_Update()
{
		mpu_data = My_Get_MPU_Data();
}
void Imu_Get_Gyro_Data(float * gx , float *gy , float *gz)
{
    (*gx) = mpu_data.gx;
    (*gy) = mpu_data.gy;
    (*gz) = mpu_data.gz;
}

void Imu_Get_Euler_Angle_Data(float * pitch , float *roll , float *yaw)
{
    (*pitch) = K_Euler_angle.pitch;
    (*roll)  = K_Euler_angle.roll;
    (*yaw)   = K_Euler_angle.yaw;
}

void Imu_Get_Acc_Data(float * ax , float *ay , float *az)
{
    (*ax) = mpu_data.ax;
    (*ay) = mpu_data.ay;
    (*az) = mpu_data.az;
}

// 新增：四元数读取（若不需要，可删）
void Imu_Get_Quaternion_Data(float *q0, float *q1, float *q2, float *q3)
{
    *q0 = K_Quat.q0; // w
    *q1 = K_Quat.q1; // x
    *q2 = K_Quat.q2; // y
    *q3 = K_Quat.q3; // z
}
