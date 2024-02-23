#include "bmi088.h"
/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置BMI088陀螺仪 加速度计的各项参数并读取数据
 * @author:
 * @date:2022/05/31
 * @note:
 ****************************************************/
#include "spi.h"
#include "debug.h"
#include "math.h"
#include "imu.h"
#include "mDelay.h"
debug_info_dev debug_dev;

#define CS_GYRO 0
#define CS_ACC 1

#define BOARD_IMU 0
#define FLOAT_IMU 1

bmi088_error_e BMI088_INIT(void)
{
    bmi088_error_e error = NO_ERROR;

    // CSB1上产生上升沿，使ACC进入SPI模式
    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_SET);

    BMI088_CONF_INIT();
    error |= VerifyAccChipID();
    error |= VerifyGyroChipID();
    while (error != NO_ERROR)
    {
        HAL_Delay(100);
        error |= VerifyAccChipID();
        error |= VerifyGyroChipID();
    }

    return error;
}
void WriteDataToAcc(uint8_t addr, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = addr & BMI088_SPI_WRITE_CODE;
    tx[1] = data;
    // 拉低AAC_cS
    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_SPI_Transmit(&hspi2, &tx[0], 1, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    delay_us(2);
    HAL_SPI_Transmit(&hspi2, &tx[1], 1, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    delay_us(2);
    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_SET);
}

void WriteDataToGyro(uint8_t addr, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = addr & BMI088_SPI_WRITE_CODE;
    tx[1] = data;

    HAL_GPIO_WritePin(SPI2_GYR_CS_GPIO_Port, SPI2_GYR_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_SPI_Transmit(&hspi2, tx, 2, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    delay_us(2);
    HAL_GPIO_WritePin(SPI2_GYR_CS_GPIO_Port, SPI2_GYR_CS_Pin, GPIO_PIN_SET);
}

void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data)
{
    delay_us(2);
    HAL_GPIO_WritePin(SPI2_GYR_CS_GPIO_Port, SPI2_GYR_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);

    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&hspi2, data, 1, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
        ;
    delay_us(2);
    HAL_GPIO_WritePin(SPI2_GYR_CS_GPIO_Port, SPI2_GYR_CS_Pin, GPIO_PIN_SET);
}
void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data)
{
    uint8_t rec[2];
    uint8_t txdata = addr | BMI088_SPI_READ_CODE;
    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_SPI_Transmit(&hspi2, &txdata, 1, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&hspi2, rec, 2, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
        ;
    *data = rec[1];
    delay_us(2);
    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_SET);
}

void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data)
{

    uint8_t rec[len + 1];
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;

    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&hspi2, &pRxData, 1, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
        ;
    for (uint8_t i = 0; i < len; i++)
    {
        HAL_SPI_Receive(&hspi2, &pRxData, 1, 10);
        while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    delay_us(2);
    HAL_GPIO_WritePin(SPI2_ACC_CS_GPIO_Port, SPI2_ACC_CS_Pin, GPIO_PIN_SET);
}

void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data)
{

    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_GPIO_WritePin(SPI2_GYR_CS_GPIO_Port, SPI2_GYR_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 10);
    HAL_SPI_Receive(&hspi2, data, len, 100);
    delay_us(2);
    HAL_GPIO_WritePin(SPI2_GYR_CS_GPIO_Port, SPI2_GYR_CS_Pin, GPIO_PIN_SET);
}

void BMI088_CONF_INIT(void)
{
    uint8_t err;
    // 加速度计初始化
    // 先软重启，清空所有寄存器
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);

    // 加速度计变成正常模式
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    HAL_Delay(10);
    // 加速度计配置写入
    // 写入范围，+-3g的测量范围
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    delay_us(100);
    // 写入配置，正常带宽，1600hz输出频率
    WriteDataToAcc(ACC_CONF_ADDR, ACC_CONF_BWP_NORM | ACC_CONF_ODR_1600_Hz);
    delay_us(100);
    // 打开加速度计电源,一定要把打开电源放到最后
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    HAL_Delay(10);

    // 陀螺仪初始化
    // 先软重启，清空所有寄存器
    WriteDataToGyro(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);

    // 陀螺仪配置写入
    // 写入范围，+-500°/s的测量范围
    WriteDataToGyro(GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    delay_us(100);
    // 写入带宽，2000Hz输出频率，532Hz滤波器带宽
    WriteDataToGyro(GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
    delay_us(100);
    // 陀螺仪变成正常模式
    WriteDataToGyro(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);
    HAL_Delay(10);
    err = VerifyAccSelfTest();
    if (err != NO_ERROR)
    {
        while (1)
        {
            // LL_mDelay(500);
            delay_us(1000 * 100);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        }
    }
    // WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    // HAL_Delay(10);
}

bmi088_error_e VerifyAccChipID(void)
{
    uint8_t chip_id;
    ReadSingleDataFromAcc(ACC_CHIP_ID_ADDR, &chip_id);
    if (chip_id != ACC_CHIP_ID_VAL)
    {
        return ACC_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyGyroChipID(void)
{
    uint8_t chip_id;
    ReadSingleDataFromGyro(GYRO_CHIP_ID_ADDR, &chip_id);
    if (chip_id != GYRO_CHIP_ID_VAL)
    {
        return GYRO_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyAccSelfTest(void)
{
    acc_raw_data_t pos_data, neg_data;
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_24G);
    WriteDataToAcc(ACC_CONF_ADDR, 0xA7);
    HAL_Delay(10);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_POS);
    HAL_Delay(100);
    ReadAccData(&dev);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_NEG);
    HAL_Delay(100);
    ReadAccData(&dev);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_OFF);
    HAL_Delay(100);
    // if ((fabs(pos_data.x - neg_data.x) > 0.1f) || (fabs(pos_data.y - neg_data.y) > 0.1f) || (fabs(pos_data.z - neg_data.z) > 0.1f))
    // {
    //     return ACC_DATA_ERR;
    // }
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(10);
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    HAL_Delay(10);

    WriteDataToAcc(ACC_CONF_ADDR, ACC_CONF_BWP_NORM | (ACC_CONF_ODR_1600_Hz));
    HAL_Delay(10);
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    HAL_Delay(10);
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    HAL_Delay(10);
    return NO_ERROR;
}

bmi088_error_e VerifyGyroSelfTest(void)
{
    WriteDataToGyro(GYRO_SELF_TEST_ADDR, GYRO_SELF_TEST_ON);
    uint8_t bist_rdy = 0x00, bist_fail;
    while (bist_rdy == 0)
    {
        ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_rdy);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
    ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_fail);
    bist_fail = (bist_fail & 0x04) >> 2;
    if (bist_fail == 0)
    {
        return NO_ERROR;
    }
    else
    {
        return GYRO_DATA_ERR;
    }
}

void ReadAccData(_imu_dev *dev)
{
    uint8_t buf[6], range;
    int16_t acc[3];
    ReadSingleDataFromAcc(ACC_RANGE_ADDR, &range);
    ReadMultiDataFromAcc(ACC_X_LSB_ADDR, 6, buf);
    acc[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    acc[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    acc[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];

    dev->aac_x = (float)acc[0] * BMI088_ACCEL_3G_SEN;
    dev->aac_y = (float)acc[1] * BMI088_ACCEL_3G_SEN;
    dev->aac_z = (float)acc[2] * BMI088_ACCEL_3G_SEN;
}

void ReadGyroData(_imu_dev *dev)
{
    uint8_t buf[GYRO_XYZ_LEN], range;
    int16_t gyro[3];
    float unit;
    ReadSingleDataFromGyro(GYRO_RANGE_ADDR, &range);
    switch (range)
    {
    case 0x00:
        unit = 16.384;
        break;
    case 0x01:
        unit = 32.768;
        break;
    case 0x02:
        unit = 65.536;
        break;
    case 0x03:
        unit = 131.072;
        break;
    case 0x04:
        unit = 262.144;
        break;
    default:
        unit = 16.384;
        break;
    }
    ReadMultiDataFromGyro(GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, buf);
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    dev->gyro_x = (float)gyro[0] / unit * DEG2SEC;
    dev->gyro_y = (float)gyro[1] / unit * DEG2SEC;
    dev->gyro_z = (float)gyro[2] / unit * DEG2SEC;
    debug_dev.head = 0xaa;
    debug_dev.addr = 0xff;
    debug_dev.id = 0xf1;
    debug_dev.databuf = buf;
    debug_dev.length = 6;
    // ano_report(&debug_dev);
}

void ReadAccSensorTime(float *time)
{
    uint8_t buf[SENSORTIME_LEN];
    ReadMultiDataFromAcc(SENSORTIME_0_ADDR, SENSORTIME_LEN, buf);
    *time = buf[0] * SENSORTIME_0_UNIT + buf[1] * SENSORTIME_1_UNIT + buf[2] * SENSORTIME_2_UNIT;
}

void ReadAccTemperature(_imu_dev *dev)
{
    uint8_t buf[TEMP_LEN];
    ReadMultiDataFromAcc(TEMP_MSB_ADDR, TEMP_LEN, buf);
    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023)
    {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    }
    else
    {
        temp_int11 = (int16_t)temp_uint11;
    }
    dev->temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}