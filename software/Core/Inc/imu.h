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
    volatile float pitch; // 32λ
    volatile float roll;
    volatile float yaw;
    volatile float aac_x; // 16λ
    volatile float aac_y;
    volatile float aac_z;
    volatile int16_t aac_x_raw; // 16λ
    volatile int16_t aac_y_raw;
    volatile int16_t aac_z_raw;
    volatile float gyro_x;
    volatile float gyro_y;
    volatile float gyro_z;
    volatile int16_t gyro_x_raw;
    volatile int16_t gyro_y_raw;
    volatile int16_t gyro_z_raw;

    uint8_t AC_buffer[6]; // ���ݻ���
    uint8_t GY_buffer[6];
    uint8_t bias_gyro[6]; // ����ƫ��
    uint8_t bias_acc[6];

    uint8_t address;     // ������ַ
    float sample_rate; // �������� ��λms
    float temp;
    float kp;
    float ki;
    float q[4];
    float eInt[3];

} _imu_dev;

#define dt 0.020f
// ��Ԫ��

#define LSB_PER_G 16384 // ���ٶȼ�������
#define LSB_PER_S 16.4  // ������������

#define Kp 2.0f   // ���ٶ�Ȩ�أ�Խ������ٶȲ���ֵ����Խ��
#define Ki 0.001f // ����������
#define squa(Sq) (((float)Sq) * ((float)Sq))
#define YAW_GYRO
extern _imu_dev dev;
// IMU����
void IMU_self_test(_imu_dev *dev);

void IMU_get_origion_data(_imu_dev *dev);

void IMU_get_euler(_imu_dev *dev);

// �˲�
typedef struct
{
    float LastP; // �ϴε�Э����
    float Now_P; // ����Э����
    float out;   // �������˲��󱾴����
    float Kg;    // ����������
    float Q;     // ϵͳ���̵�Э����
    float R;     // ������Э����
} _1_ekf_filter;

typedef struct
{
    float Q_angle; // �Ƕ��������Ŷȣ��Ƕ�������Э����
    float Q_gyro;  // ���ٶ��������Ŷȣ����ٶ�������Э����
    float R_angle; // ���ٶȼƲ���������Э����
    float m_dt;    // �������ڼ�������������10ms
    float Q_bias;  // Q_bias:�����ǵ�ƫ��

    float PP[2][2]; // ����Э�������P����ʼֵΪ��λ��
    float K_0;      // ����������  K_0:���ڼ������Ź���ֵ  K_1:���ڼ������Ź���ֵ��ƫ��
    float K_1;

} Kalman;

/*
    һ�׿������˲�
    @param ekf һ�׿������˲�����
    @param input �˲�����
*/
void kalman_1(_1_ekf_filter *ekf, float input);
void kalman_filter(_imu_dev *dev);
void kalman_get_angle_pitch(float *angle, float gyro, float acc);
void kalman_get_angle_roll(float *angle, float gyro, float acc);
void MahonyAHRS(float samplePeriod, float kp, float ki);
void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, _imu_dev *dev);
float Q_rsqrt(float number);
#endif
