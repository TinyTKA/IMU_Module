#include "imu.h"
#include "math.h"
#include "bmi088.h"
#include "qmc5883.h"
#include "debug.h"

_imu_dev dev;
static float NormAcc;
const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
static float Q_angle = 0.001;             // �Ƕ��������Ŷȣ��Ƕ�������Э����
static float Q_gyro = 0.003;              // ���ٶ��������Ŷȣ����ٶ�������Э����
static float R_angle = 0.5;               // ���ٶȼƲ���������Э����
static float mdt = 0.001;                 // �������ڼ�������������10ms
static float Q_bias;                      // Q_bias:�����ǵ�ƫ��
static float K_0, K_1;                    // ����������  K_0:���ڼ������Ź���ֵ  K_1:���ڼ������Ź���ֵ��ƫ��
static float PP[2][2] = {{1, 0}, {0, 1}}; // ����Э�������P����ʼֵΪ��λ��

// �����ǽ��ٶ�ԭʼֵת��Ϊʵ�ʵ�ֵ
// ���̡�2000ʱ��65536/4000=16.384,����ʵ��ת��ʱ�ǣ�ԭʼֵ/16.384,��ԭʼֵ*1/16.384��ԭʼֵ*0.061035
const float Gyro_G = 0.03051756f * 2; // �����ǳ�ʼ������+-2000��ÿ����1 / (65536 /s 4000) = 0.03051756*2
// �Ƕ�ת��Ϊ����
const float Gyro_Gr = 65.536 * DEG2SEC; // ������ÿ��,ת������ÿ���� 2*0.03051756	 * 0.0174533f = 0.0005326*2

void IMU_self_test(_imu_dev *dev)
{
    short sum_Acc[3];
    short sum_gyro[3];

    for (uint8_t j = 0; j < 3; j++)
    {
        dev->bias_acc[j] = 0;
        dev->bias_gyro[j] = 0;
    } // ȫ����0
    // ���ܺ�
    for (uint8_t i = 0; i < 10; i++)
    {
        // MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, &(dev->AC_buffer[0]));
        // MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, &(dev->GY_buffer[0]));
        sum_Acc[0] += ((short)(dev->AC_buffer[0]) << 8) | (dev->AC_buffer[1]);
        sum_Acc[1] += ((short)(dev->AC_buffer[2]) << 8) | (dev->AC_buffer[3]);
        sum_Acc[2] += ((short)(dev->AC_buffer[4]) << 8) | dev->AC_buffer[5];
        sum_gyro[0] += ((short)(dev->GY_buffer[0]) << 8) | (dev->GY_buffer[1]);
        sum_gyro[1] += ((short)(dev->GY_buffer[2]) << 8) | (dev->GY_buffer[3]);
        sum_gyro[2] += ((short)(dev->GY_buffer[4]) << 8) | (dev->GY_buffer[5]);
        HAL_Delay(5);
    }

    for (uint8_t j = 0; j < 3; j++)
    {
        dev->bias_acc[j] = sum_Acc[j] / 10;
        dev->bias_gyro[j] = sum_gyro[j] / 10;
    }
}

void IMU_get_origion_data(_imu_dev *dev)
{
    // ������ϵ��
    static _1_ekf_filter ekf[3] = {{0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}};
    // һ�׵�ͨ�˲�ϵ��
    const float factor = 0.15f;
    // �����ϴ�����
    static float acc_buff[3];
    static float gyro_buff[3];
    uint8_t range;
    // ReadSingleDataFromAcc(ACC_RANGE_ADDR, &range);
    // printf("sample:%d\n",range);
    ReadMultiDataFromGyro(GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, dev->GY_buffer);
    ReadMultiDataFromAcc(ACC_X_LSB_ADDR, 6, dev->AC_buffer);
    dev->aac_x_raw = ((int16_t)(dev->AC_buffer[1]) << 8) | (dev->AC_buffer[0]);
    dev->aac_y_raw = ((int16_t)(dev->AC_buffer[3]) << 8) | (dev->AC_buffer[2]);
    dev->aac_z_raw = ((int16_t)(dev->AC_buffer[5]) << 8) | (dev->AC_buffer[4]);
    dev->gyro_x_raw = ((int16_t)(dev->GY_buffer[1]) << 8) | (dev->GY_buffer[0]);
    dev->gyro_y_raw = ((int16_t)(dev->GY_buffer[3]) << 8) | (dev->GY_buffer[2]);
    dev->gyro_z_raw = ((int16_t)(dev->GY_buffer[5]) << 8) | (dev->GY_buffer[4]);
    // ��ȥƫ��
    //  dev->aac_x -= dev->bias_acc[0];
    //  dev->aac_y -= dev->bias_acc[1];

    // dev->gyro_x -= dev->bias_gyro[0];
    // dev->gyro_y -= dev->bias_gyro[1];
    // dev->gyro_z -= dev->bias_gyro[2];

    // �Լ��ٶ���һά�������˲�
    // kalman_1(ekf + 0, (float)dev->aac_x_raw);
    // dev->aac_x_raw = (int16_t)ekf[0].out;
    // kalman_1(ekf + 1, (float)dev->aac_y_raw);
    // dev->aac_y_raw = (int16_t)ekf[1].out;
    // kalman_1(ekf + 2, (float)dev->aac_z_raw);
    // dev->aac_z_raw = (int16_t)ekf[2].out;

    // �Խ��ٶ���һ�׵�ͨ�˲�
    dev->gyro_x_raw = gyro_buff[0] * (1 - factor) + dev->gyro_x_raw * factor;
    dev->gyro_y_raw = gyro_buff[1] * (1 - factor) + dev->gyro_y_raw * factor;
    dev->gyro_z_raw = gyro_buff[2] * (1 - factor) + dev->gyro_z_raw * factor;
    gyro_buff[0] = dev->gyro_x_raw;
    gyro_buff[1] = dev->gyro_y_raw;
    gyro_buff[2] = dev->gyro_z_raw;
    dev->aac_x_raw = acc_buff[0] * (1 - factor) + dev->aac_x_raw * factor;
    dev->aac_y_raw = acc_buff[1] * (1 - factor) + dev->aac_y_raw * factor;
    dev->aac_z_raw = acc_buff[2] * (1 - factor) + dev->aac_z_raw * factor;
    acc_buff[0] = dev->aac_x_raw;
    acc_buff[1] = dev->aac_y_raw;
    acc_buff[2] = dev->aac_z_raw;

    dev->aac_x = (float)dev->aac_x_raw * BMI088_ACCEL_3G_SEN;
    dev->aac_y = (float)dev->aac_y_raw * BMI088_ACCEL_3G_SEN;
    dev->aac_z = (float)dev->aac_z_raw * BMI088_ACCEL_3G_SEN;

    dev->gyro_x = (float)dev->gyro_x_raw / 65.536 * DEG2SEC;
    dev->gyro_y = (float)dev->gyro_y_raw / 65.536 * DEG2SEC;
    dev->gyro_z = (float)dev->gyro_z_raw / 65.536 * DEG2SEC;
}

/*



*/

void IMU_get_euler(_imu_dev *dev)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, error_angle;

    static struct V GyroIntegError = {0};
    static float KpDef = 0.999f;
    static float KiDef = 0.001f;
    static Quaternion NumQ = {1, 0, 0, 0}; // ��ʱ����Ԫ����ֵ  w i j k
    float q0_t, q1_t, q2_t, q3_t;
    // float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // ��ȡ��Ч��ת�����е���������������Ԫ���ĵ�Ч���Ҿ����е���������
    // �����Ǵ���Ԫ������ȡ�����������������ٶ�
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // ���ٶȹ�һ��,��һ������Ҳʡȥ��ԭʼֵ��ʵ��ֵ��ת��t
    // ������ʵ��ͨ�����ٶȴ��������������ļ��ٶ�
    NormAcc = Q_rsqrt(squa(dev->aac_x_raw) + squa(dev->aac_y_raw) + squa(dev->aac_z_raw));
    Acc.x = dev->aac_x_raw * NormAcc;
    Acc.y = dev->aac_y_raw * NormAcc;
    Acc.z = dev->aac_z_raw * NormAcc;
    // ���ٶȼƻ�õ�����������һ��֮���ֵ����ȡ����̬���������������˻�ȡ��̬��������˵ó���ֵ
    // ������ʵ������˵ó����Ƕ�

    error_angle.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    error_angle.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    error_angle.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    /*PI�����������Ʋ����Ĵ�С�뾫��*/

    /*���ֲ���*/
    // �������̻��֣��Ӷ��������������ٶȻ��ֲ������ٶȵĲ���ֵ
    GyroIntegError.x += error_angle.x * KiDef;
    GyroIntegError.y += error_angle.y * KiDef;
    GyroIntegError.z += error_angle.z * KiDef;
    // ���ٶ��ںϼ��ٶȻ��ֲ���ֵ *gyro_x����ͬʱ����ԭʼֵ��ʵ��ֵ��ת��
    Gyro.x = dev->gyro_x_raw * Gyro_Gr + KpDef * error_angle.x + GyroIntegError.x; // ������
    Gyro.y = dev->gyro_y_raw * Gyro_Gr + KpDef * error_angle.y + GyroIntegError.y;
    Gyro.z = dev->gyro_z_raw * Gyro_Gr + KpDef * error_angle.z + GyroIntegError.z;

    // һ�����������, ������Ԫ��
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    // ��Ԫ����һ��
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // ��Ԫ��תŷ����
    {

#ifdef YAW_GYRO
        dev->yaw = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA; // yaw
#else
        float yaw_G = dev->gyro_z * Gyro_G;
        if ((yaw_G > 1.2f) || (yaw_G < -1.2f)) // ����̫С������Ϊ�Ǹ��ţ�����ƫ������
        {
            dev->yaw += yaw_G * dt;
        }
#endif
        dev->pitch = asin(2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;

        dev->roll = atan2(2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA; // PITCH
    }
}
/*********************************************************************************************************************
**�������˲�
**@brief: �������������˲�
**@param[in]  InputData �˲�ǰ�����ݣ�QR���
**@param[out] None
**@return �˲��������
**@remark: ͨ���޸Ĺ��������Ͳ�������R,Qֵ�Ż����ֵ
X(k)=A X(k-1)+B U(k)+W(k)
Z(k)=H X(k)+V(k)
AB��ϵͳ����
X?Kʱ�̵�ϵͳ״̬
H������ϵͳ�Ĳ���
Z��Kʱ�̵Ĳ���ֵ
WV�����̺Ͳ�������

X(k|k-1)=X(k-1|k-1) Ԥ����һ״̬��ϵͳ
P(k|k-1)=P(k-1|k-1) +Q      //Ԥ��Э����
Kg(k)= P(k|k-1) / (P(k|k-1) + R)   ����Kg����������
X(k|k)= X(k|k-1)+Kg(k) (Z(k)-X(k|k-1))   ����Ԥ��ֵ��Ϲ���ֵ�ó���ǰ״ֵ̬
P(k|k)=(1-Kg(k))P(k|k-1)  ����Э����


(k-1|k-1)��һ��״̬����������
(k|k-1) ����һ��״̬����������Ԥ�⵱ǰ״̬����������
(k|k)  ��Ԥ�Ȿ״̬����������ʵ����������

Q:ϵͳ���̵�Э����
R:������Э����
��˹�� = Q+R
Kg������������
P��Э����

ע:���������˲������Ե�ģ�ͣ�������H,I��Ϊ1��û�п�����U=0��ͨ����A,B��ʼֵȡ1
ע��X����������X(0|0)��ʼ����ֵ״̬���ݣ�����ǰ����ֵ������
ע :   P (0|0)һ�㲻ȡ0��ȡ0��ζ��0ʱ��ķ���Ϊ0��ϵͳ��Ϊ0ʱ�̵�ֵ�����ŵġ�Ȼ����ʵ��ϵͳ������0ʱ�̲������ŵ�
*/
void kalman_1(_1_ekf_filter *ekf, float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;                   // Ԥ�Ȿ��Э����
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);       // ���㿨��������
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out); // ����Ԥ��ֵ��Ϲ���ֵ�ó���ǰ״ֵ̬
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;            // ����Э����
}
void kalman_filter(_imu_dev *dev)
{
    static uint8_t cnt = 0;
    float NormAcc;
    float aac[3], aac_backup[3];
    // dev->gyro_x = dev->gyro_x * Gyro_G;
    // dev->gyro_y = dev->gyro_y * Gyro_G;

    NormAcc = Q_rsqrt(squa(dev->aac_x) + squa(dev->aac_y) + squa(dev->aac_z));
    aac[0] = dev->aac_x * NormAcc;
    aac[1] = dev->aac_y * NormAcc;
    aac[2] = dev->aac_z * NormAcc;
    aac_backup[0] = aac[0];
    aac_backup[1] = aac[1];
    aac_backup[2] = aac[2];
    aac[0] = atan2(-aac_backup[0], sqrt(squa(aac_backup[1]) + squa(aac_backup[2]))) * (180 / 3.14);
    aac[1] = atan2(aac_backup[1], sqrt(squa(aac_backup[0]) + squa(aac_backup[2]))) * (180 / 3.14);
    if (cnt == 0)
    {
        dev->pitch = 0;
        dev->roll = 0;
        ++cnt;
    }
    kalman_get_angle_pitch(&(dev->pitch), dev->gyro_x, aac[0]);
    kalman_get_angle_roll(&(dev->roll), dev->gyro_y, aac[1]);
}
void kalman_get_angle_pitch(float *angle, float gyro, float acc)
{
    static float Q_bias;                      // Q_bias:�����ǵ�ƫ��  Angle_err:�Ƕ�ƫ��
    static float K_0, K_1;                    // ����������  K_0:���ڼ������Ź���ֵ  K_1:���ڼ������Ź���ֵ��ƫ�� t_0/1:�м����
    static float PP[2][2] = {{1, 0}, {0, 1}}; // ����Э�������P����ʼֵΪ��λ��
    *angle += (gyro - Q_bias) * dt;           // ״̬����,�Ƕ�ֵ�����ϴ����ŽǶȼӽ��ٶȼ���Ư�����
    PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
    PP[0][1] = PP[0][1] - PP[1][1] * dt;
    PP[1][0] = PP[1][0] - PP[1][1] * dt;
    PP[1][1] = PP[1][1] + Q_gyro;
    K_0 = PP[0][0] / (PP[0][0] + R_angle);
    K_1 = PP[1][0] / (PP[0][0] + R_angle);
    *angle = *angle + K_0 * (acc - *angle);
    Q_bias = Q_bias + K_1 * (acc - *angle);
    PP[0][0] = PP[0][0] - K_0 * PP[0][0];
    PP[0][1] = PP[0][1] - K_0 * PP[0][1];
    PP[1][0] = PP[1][0] - K_1 * PP[0][0];
    PP[1][1] = PP[1][1] - K_1 * PP[0][1];
}
void kalman_get_angle_roll(float *angle, float gyro, float acc)
{
    static float Q_bias;                      // Q_bias:�����ǵ�ƫ��  Angle_err:�Ƕ�ƫ��
    static float K_0, K_1;                    // ����������  K_0:���ڼ������Ź���ֵ  K_1:���ڼ������Ź���ֵ��ƫ�� t_0/1:�м����
    static float PP[2][2] = {{1, 0}, {0, 1}}; // ����Э�������P����ʼֵΪ��λ��
    *angle += (gyro - Q_bias) * dt;           // ״̬����,�Ƕ�ֵ�����ϴ����ŽǶȼӽ��ٶȼ���Ư�����
    PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
    PP[0][1] = PP[0][1] - PP[1][1] * dt;
    PP[1][0] = PP[1][0] - PP[1][1] * dt;
    PP[1][1] = PP[1][1] + Q_gyro;
    K_0 = PP[0][0] / (PP[0][0] + R_angle);
    K_1 = PP[1][0] / (PP[0][0] + R_angle);
    *angle = *angle + K_0 * (acc - *angle);
    Q_bias = Q_bias + K_1 * (acc - *angle);
    PP[0][0] = PP[0][0] - K_0 * PP[0][0];
    PP[0][1] = PP[0][1] - K_0 * PP[0][1];
    PP[1][0] = PP[1][0] - K_1 * PP[0][0];
    PP[1][1] = PP[1][1] - K_1 * PP[0][1];
}

float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration ����һ��ţ�ٵ�����
    return y;
}

void MahonyAHRS(float samplePeriod, float kp, float ki)
{
    dev.sample_rate = samplePeriod;
    dev.kp = kp;
    dev.ki = ki;
    dev.q[0] = 1.0f;
    dev.q[1] = 0.0f;
    dev.q[2] = 0.0f;
    dev.q[3] = 0.0f;
    dev.eInt[0] = 0.0f;
    dev.eInt[1] = 0.0f;
    dev.eInt[2] = 0.0f;
}
void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, _imu_dev *dev)
{
    float q1 = dev->q[0], q2 = dev->q[1], q3 = dev->q[2], q4 = dev->q[3]; // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    // ����1 ����һ�����ٶ�
    // ax,ay,az�������Ǽ��ٶ�ԭʼ���ݣ�Ҳ�������Ѿ������˲��������
    norm = (float)Q_rsqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    // ����2����һ���ش�����
    // mx,my,mz ���ش�ԭʼ���ݣ����˲��������
    norm = (float)Q_rsqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return; // handle NaN
    mx *= norm;
    my *= norm;
    mz *= norm;
    // ����3���ų�������bϵ��ת��nϵ�����ӻ�������ϵ��ת���������ϵ
    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = (float)Q_rsqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);
    // ����4���������ٶȣ���bϵ��ת��nϵ
    //  Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);
    // �������
    //  Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        dev->eInt[0] += ex; // accumulate integral error
        dev->eInt[1] += ey;
        dev->eInt[2] += ez;
    }
    else
    {
        dev->eInt[0] = 0.0f; // prevent integral wind up
        dev->eInt[1] = 0.0f;
        dev->eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * dev->eInt[0];
    gy = gy + Kp * ey + Ki * dev->eInt[1];
    gz = gz + Kp * ez + Ki * dev->eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * dev->sample_rate);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * dev->sample_rate);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * dev->sample_rate);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * dev->sample_rate);

    // Normalise quaternion
    norm = (float)Q_rsqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    // norm = 1.0f / norm;
    dev->q[0] = q1 * norm;
    dev->q[1] = q2 * norm;
    dev->q[2] = q3 * norm;
    dev->q[3] = q4 * norm;
    // ��Ԫ��תŷ����
    {

#ifdef YAW_GYRO
        dev->yaw = atan2f(2 * dev->q[1] * dev->q[2] + 2 * dev->q[0] * dev->q[3], 1 - 2 * dev->q[2] * dev->q[2] - 2 * dev->q[3] * dev->q[3]) * RtA; // yaw
#else
        float yaw_G = dev->gyro_z * Gyro_G;
        if ((yaw_G > 1.2f) || (yaw_G < -1.2f)) // ����̫С������Ϊ�Ǹ��ţ�����ƫ������
        {
            dev->yaw += yaw_G * dt;
        }
#endif
        dev->pitch = asin(2 * dev->q[0] * dev->q[2] - 2 * dev->q[1] * dev->q[3]) * RtA;

        dev->roll = atan2(2 * dev->q[2] * dev->q[3] + 2 * dev->q[0] * dev->q[1], 1 - 2 * dev->q[1] * dev->q[1] - 2 * dev->q[2] * dev->q[2]) * RtA; // PITCH
    }
}
