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
static float Q_angle = 0.001;             // 角度数据置信度，角度噪声的协方差
static float Q_gyro = 0.003;              // 角速度数据置信度，角速度噪声的协方差
static float R_angle = 0.5;               // 加速度计测量噪声的协方差
static float mdt = 0.001;                 // 采样周期即计算任务周期10ms
static float Q_bias;                      // Q_bias:陀螺仪的偏差
static float K_0, K_1;                    // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差
static float PP[2][2] = {{1, 0}, {0, 1}}; // 过程协方差矩阵P，初始值为单位阵

// 陀螺仪角速度原始值转换为实际的值
// 量程±2000时，65536/4000=16.384,所以实际转换时是：原始值/16.384,即原始值*1/16.384即原始值*0.061035
const float Gyro_G = 0.03051756f * 2; // 陀螺仪初始化量程+-2000度每秒于1 / (65536 /s 4000) = 0.03051756*2
// 角度转换为弧度
const float Gyro_Gr = 65.536 * DEG2SEC; // 面计算度每秒,转换弧度每秒则 2*0.03051756	 * 0.0174533f = 0.0005326*2

void IMU_self_test(_imu_dev *dev)
{
    short sum_Acc[3];
    short sum_gyro[3];

    for (uint8_t j = 0; j < 3; j++)
    {
        dev->bias_acc[j] = 0;
        dev->bias_gyro[j] = 0;
    } // 全部清0
    // 求总和
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
    // 卡尔曼系数
    static _1_ekf_filter ekf[3] = {{0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}};
    // 一阶低通滤波系数
    const float factor = 0.15f;
    // 保存上次数据
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
    // 减去偏差
    //  dev->aac_x -= dev->bias_acc[0];
    //  dev->aac_y -= dev->bias_acc[1];

    // dev->gyro_x -= dev->bias_gyro[0];
    // dev->gyro_y -= dev->bias_gyro[1];
    // dev->gyro_z -= dev->bias_gyro[2];

    // 对加速度做一维卡尔曼滤波
    // kalman_1(ekf + 0, (float)dev->aac_x_raw);
    // dev->aac_x_raw = (int16_t)ekf[0].out;
    // kalman_1(ekf + 1, (float)dev->aac_y_raw);
    // dev->aac_y_raw = (int16_t)ekf[1].out;
    // kalman_1(ekf + 2, (float)dev->aac_z_raw);
    // dev->aac_z_raw = (int16_t)ekf[2].out;

    // 对角速度做一阶低通滤波
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
    static Quaternion NumQ = {1, 0, 0, 0}; // 零时刻四元数初值  w i j k
    float q0_t, q1_t, q2_t, q3_t;
    // float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 提取等效旋转矩阵中的重力分量，即四元数的等效余弦矩阵中的重力分量
    // 这里是从四元数中提取出来的理论重力加速度
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // 加速度归一化,归一化操作也省去了原始值向实际值的转换t
    // 这里是实际通过加速度传感器测量出来的加速度
    NormAcc = Q_rsqrt(squa(dev->aac_x_raw) + squa(dev->aac_y_raw) + squa(dev->aac_z_raw));
    Acc.x = dev->aac_x_raw * NormAcc;
    Acc.y = dev->aac_y_raw * NormAcc;
    Acc.z = dev->aac_z_raw * NormAcc;
    // 加速度计获得的重力向量归一化之后的值与提取的姿态矩阵的重力向量叉乘获取姿态误差，向量叉乘得出的值
    // 理论与实际做叉乘得出误差角度

    error_angle.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    error_angle.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    error_angle.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    /*PI控制器来控制补偿的大小与精度*/

    /*积分部分*/
    // 对误差进程积分，从而消除误差，再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += error_angle.x * KiDef;
    GyroIntegError.y += error_angle.y * KiDef;
    GyroIntegError.z += error_angle.z * KiDef;
    // 角速度融合加速度积分补偿值 *gyro_x操作同时暗含原始值向实际值的转换
    Gyro.x = dev->gyro_x_raw * Gyro_Gr + KpDef * error_angle.x + GyroIntegError.x; // 弧度制
    Gyro.y = dev->gyro_y_raw * Gyro_Gr + KpDef * error_angle.y + GyroIntegError.y;
    Gyro.z = dev->gyro_z_raw * Gyro_Gr + KpDef * error_angle.z + GyroIntegError.z;

    // 一阶龙格库塔法, 更新四元数
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    // 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // 四元数转欧拉角
    {

#ifdef YAW_GYRO
        dev->yaw = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA; // yaw
#else
        float yaw_G = dev->gyro_z * Gyro_G;
        if ((yaw_G > 1.2f) || (yaw_G < -1.2f)) // 数据太小可以认为是干扰，不是偏航动作
        {
            dev->yaw += yaw_G * dt;
        }
#endif
        dev->pitch = asin(2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;

        dev->roll = atan2(2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA; // PITCH
    }
}
/*********************************************************************************************************************
**卡尔曼滤波
**@brief: 线性最优评估滤波
**@param[in]  InputData 滤波前的数据，QR误差
**@param[out] None
**@return 滤波后的数据
**@remark: 通过修改过程噪声和测量噪声R,Q值优化输出值
X(k)=A X(k-1)+B U(k)+W(k)
Z(k)=H X(k)+V(k)
AB是系统参数
X?K时刻的系统状态
H：测量系统的参数
Z：K时刻的测量值
WV：过程和测量噪声

X(k|k-1)=X(k-1|k-1) 预测下一状态的系统
P(k|k-1)=P(k-1|k-1) +Q      //预测协方差
Kg(k)= P(k|k-1) / (P(k|k-1) + R)   计算Kg卡尔曼增益
X(k|k)= X(k|k-1)+Kg(k) (Z(k)-X(k|k-1))   根据预测值结合估算值得出当前状态值
P(k|k)=(1-Kg(k))P(k|k-1)  更新协方差


(k-1|k-1)上一个状态的最优评估
(k|k-1) 由上一个状态的最优评估预测当前状态的最优评估
(k|k)  由预测本状态的评估具体实现最优评估

Q:系统过程的协方差
R:测量的协方差
高斯白 = Q+R
Kg：卡尔曼增益
P：协方差

注:本卡尔曼滤波器争对单模型，单测量H,I均为1，没有控制量U=0，通常对A,B初始值取1
注：X会逐渐收敛，X(0|0)初始测量值状态根据，测量前的数值而定。
注 :   P (0|0)一般不取0，取0意味在0时候的方差为0，系统认为0时刻的值是最优的。然而在实际系统中往往0时刻不是最优的
*/
void kalman_1(_1_ekf_filter *ekf, float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;                   // 预测本次协方差
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);       // 计算卡尔曼增益
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out); // 根据预测值结合估算值得出当前状态值
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;            // 更新协方差
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
    static float Q_bias;                      // Q_bias:陀螺仪的偏差  Angle_err:角度偏量
    static float K_0, K_1;                    // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
    static float PP[2][2] = {{1, 0}, {0, 1}}; // 过程协方差矩阵P，初始值为单位阵
    *angle += (gyro - Q_bias) * dt;           // 状态方程,角度值等于上次最优角度加角速度减零漂后积分
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
    static float Q_bias;                      // Q_bias:陀螺仪的偏差  Angle_err:角度偏量
    static float K_0, K_1;                    // 卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
    static float PP[2][2] = {{1, 0}, {0, 1}}; // 过程协方差矩阵P，初始值为单位阵
    *angle += (gyro - Q_bias) * dt;           // 状态方程,角度值等于上次最优角度加角速度减零漂后积分
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
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration （第一次牛顿迭代）
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
    // 步骤1 ：归一化加速度
    // ax,ay,az：可以是加速度原始数据，也可以是已经处理滤波后的数据
    norm = (float)Q_rsqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    // 步骤2：归一化地磁向量
    // mx,my,mz ：地磁原始数据，或滤波后的数据
    norm = (float)Q_rsqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return; // handle NaN
    mx *= norm;
    my *= norm;
    mz *= norm;
    // 步骤3：磁场向量从b系旋转到n系，即从机体坐标系旋转到大地坐标系
    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = (float)Q_rsqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);
    // 步骤4：重力加速度，从b系旋转到n系
    //  Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);
    // 计算误差
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
    // 四元数转欧拉角
    {

#ifdef YAW_GYRO
        dev->yaw = atan2f(2 * dev->q[1] * dev->q[2] + 2 * dev->q[0] * dev->q[3], 1 - 2 * dev->q[2] * dev->q[2] - 2 * dev->q[3] * dev->q[3]) * RtA; // yaw
#else
        float yaw_G = dev->gyro_z * Gyro_G;
        if ((yaw_G > 1.2f) || (yaw_G < -1.2f)) // 数据太小可以认为是干扰，不是偏航动作
        {
            dev->yaw += yaw_G * dt;
        }
#endif
        dev->pitch = asin(2 * dev->q[0] * dev->q[2] - 2 * dev->q[1] * dev->q[3]) * RtA;

        dev->roll = atan2(2 * dev->q[2] * dev->q[3] + 2 * dev->q[0] * dev->q[1], 1 - 2 * dev->q[1] * dev->q[1] - 2 * dev->q[2] * dev->q[2]) * RtA; // PITCH
    }
}
