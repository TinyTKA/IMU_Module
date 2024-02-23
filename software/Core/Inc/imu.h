#ifndef __IMU_H
#define __IMU_H
#include "main.h"

typedef volatile struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

typedef struct
{
    volatile float pitch; // 32位
    volatile float roll;
    volatile float yaw;
    volatile float aac_x; // 16位
    volatile float aac_y;
    volatile float aac_z;
    volatile int16_t aac_x_raw; // 16位
    volatile int16_t aac_y_raw;
    volatile int16_t aac_z_raw;
    volatile float gyro_x;
    volatile float gyro_y;
    volatile float gyro_z;
    volatile int16_t gyro_x_raw;
    volatile int16_t gyro_y_raw;
    volatile int16_t gyro_z_raw;

    uint8_t AC_buffer[6]; // 数据缓存
    uint8_t GY_buffer[6];
    uint8_t bias_gyro[6]; // 计算偏差
    uint8_t bias_acc[6];

    uint8_t address;     // 器件地址
    float sample_rate; // 采样周期 单位ms
    float temp;
    float kp;
    float ki;
    float q[4];
    float eInt[3];

} _imu_dev;

#define dt 0.020f
// 四元数

#define LSB_PER_G 16384 // 加速度计灵敏度
#define LSB_PER_S 16.4  // 陀螺仪灵敏度

#define Kp 2.0f   // 加速度权重，越大则加速度测量值收敛越快
#define Ki 0.001f // 误差积分增益
#define squa(Sq) (((float)Sq) * ((float)Sq))
#define YAW_GYRO
extern _imu_dev dev;
// IMU部分
void IMU_self_test(_imu_dev *dev);

void IMU_get_origion_data(_imu_dev *dev);

void IMU_get_euler(_imu_dev *dev);

// 滤波
typedef struct
{
    float LastP; // 上次的协方差
    float Now_P; // 本次协方差
    float out;   // 卡尔曼滤波后本次输出
    float Kg;    // 卡尔曼增益
    float Q;     // 系统过程的协方差
    float R;     // 测量的协方差
} _1_ekf_filter;

typedef struct
{
    float Q_angle; // 角度数据置信度，角度噪声的协方差
    float Q_gyro;  // 角速度数据置信度，角速度噪声的协方差
    float R_angle; // 加速度计测量噪声的协方差
    float m_dt;    // 采样周期即计算任务周期10ms
    float Q_bias;  // Q_bias:陀螺仪的偏差

    float PP[2][2]; // 过程协方差矩阵P，初始值为单位阵
    float K_0;      // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差
    float K_1;

} Kalman;

/*
    一阶卡尔曼滤波
    @param ekf 一阶卡尔曼滤波参数
    @param input 滤波输入
*/
void kalman_1(_1_ekf_filter *ekf, float input);
void kalman_filter(_imu_dev *dev);
void kalman_get_angle_pitch(float *angle, float gyro, float acc);
void kalman_get_angle_roll(float *angle, float gyro, float acc);
void MahonyAHRS(float samplePeriod, float kp, float ki);
void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, _imu_dev *dev);
float Q_rsqrt(float number);
#endif
