#include "qmc5883.h"
#include "main.h"
#include "i2c.h"
#include "debug.h"
#include "math.h"
QMC5883L_DEV QMC5883LSample;
QMC5883L_RESULT_SAMPLE QMC5883err;
/// @brief qmc5883l 写寄存器
/// @param addr     地址
/// @param val
void QMC5883L_Register_Write(uint8_t addr, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 0XFF);
}
/// @brief QMC5883L 读寄存器
/// @param addr     地址
/// @return     返回寄存器值
uint8_t QMC5883L_Register_Read(uint8_t addr)
{
    uint8_t temp = 0x00;
    HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDR_READ, addr, I2C_MEMADD_SIZE_8BIT, &temp, 1, 0xff);
    return temp;
}

/// @brief 读取QMC5883L的传感器数据
/// @param x
/// @param y
/// @param z
void QMC5883L_Read_XYZ(QMC5883L_DEV *dev)
{
    uint8_t buf[QMC5883L_XYZBUF_LEN];
    int16_t dataX, dataY, dataZ;
    float fdatax, fdatay, fdataz;
    int32_t sum[3] = {0, 0, 0};
    uint8_t rx;

    rx = QMC5883L_Register_Read(QMC5883L_ADDR_STATUS);
    if (rx & 0x01 != 0x01)
    {
        return;
    }

    HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDR_READ, QMC5883L_ADDR_XOUTL, I2C_MEMADD_SIZE_8BIT, buf, QMC5883L_XYZBUF_LEN, 0xff);
    rx = QMC5883L_Register_Read(QMC5883L_ADDR_STATUS);
    if (rx & 0x01 != 0x01)
    {
        // return;
    }
    // HAL_Delay(20);
    dataX = (int16_t)((buf[1] << 8) | buf[0]);
    dataY = (int16_t)((buf[3] << 8) | buf[2]);
    dataZ = (int16_t)((buf[5] << 8) | buf[4]);
    dataX = dataX / QMC5883L_SENSITIVITY_8G;
    dataY = dataY / QMC5883L_SENSITIVITY_8G;
    dataZ = dataZ / QMC5883L_SENSITIVITY_8G;

    fdatax = (dataX - dev->X_Offset) * dev->Kx;
    fdatay = (dataY - dev->Y_Offset) * dev->Ky;
    fdataz = (dataZ - dev->Z_Offset) * dev->Kz;
    dev->X_Data = fdatax;
    dev->Y_Data = fdatay;
    dev->Z_Data = fdataz;

    dev->XY_Angle = (atan2(fdatay, fdatax) * (180 / 3.14159265));
    if (dev->XY_Angle < 0)
    {
        dev->XY_Angle += 360;
    }
    // if (dev->XY_Angle < 0)
    // {
    //     dev->XY_Angle += 360;
    // }

    dev->XZ_Angle = (atan2(fdataz, fdatax) * (180 / 3.14159265));
    dev->YZ_Angle = (atan2(fdataz, fdatay) * (180 / 3.14159265));
}
void QMC5883L_Read_TEMP(float *temp)
{
    uint8_t buf[QMC5883L_TEMBUF_LEN];
    int16_t dataT;
    HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDR_READ, QMC5883L_ADDR_TEMPL, I2C_MEMADD_SIZE_8BIT, buf, QMC5883L_TEMBUF_LEN, 0xff);

    dataT = (int16_t)((buf[1] << 8) + buf[0]);
    *temp = dataT / 100.0;
}

uint8_t QMC5883L_Wait_Data_Update(uint8_t max_cnt)
{
    for (; max_cnt > 0x00; max_cnt--)
    {
        if (QMC5883L_Register_Read(QMC5883L_ADDR_STATUS) & 0x01)
        {
            return 0x01;
        }
        HAL_Delay(1);
    }
    return 0x00;
}

uint8_t QMC5883L_Read_Average(uint16_t *x, uint16_t *y, uint16_t *z, uint8_t times)
{
    uint8_t i, j;
    int16_t tx, ty, tz;
    uint8_t temp_serial[10], tmp_serial;
    int16_t temp_x[10], temp_y[10], temp_z[10];
    int32_t tmp = 0;

    *x = 0;
    *y = 0;
    *z = 0;

    if (times)
    {
        for (i = 0; i < times; i++)
        {
            if (!QMC5883L_Wait_Data_Update(10))
            {
                return 0x00;
            }
            // QMC5883L_Read_XYZ(&tx, &ty, &tz);
            temp_x[i] = tx;
            temp_y[i] = ty;
            temp_z[i] = tz;
        }
    }

    /**********************************计算X轴***********************************/
    for (j = 0; j < times; j++)
    {
        temp_serial[j] = j;
    }

    for (i = times; i > 0; i--)
    {
        for (j = 0; j < (i - 1); j++)
        {
            if (temp_x[temp_serial[j]] > temp_x[temp_serial[j + 1]])
            {
                tmp_serial = temp_serial[j];
                temp_serial[j] = temp_serial[j + 1];
                temp_serial[j + 1] = tmp_serial;
            }
        }
    }

    for (i = 1; i < times - 1; i++)
    {
        tmp += temp_x[temp_serial[i]];
    }

    *x = tmp / (times - 2);

    /**********************************计算Y轴***********************************/
    tmp = 0;

    for (j = 0; j < times; j++)
    {
        temp_serial[j] = j;
    }

    for (i = times; i > 0; i--)
    {
        for (j = 0; j < (i - 1); j++)
        {
            if (temp_y[temp_serial[j]] > temp_y[temp_serial[j + 1]])
            {
                tmp_serial = temp_serial[j];
                temp_serial[j] = temp_serial[j + 1];
                temp_serial[j + 1] = tmp_serial;
            }
        }
    }

    for (i = 1; i < times - 1; i++)
    {
        tmp += temp_y[temp_serial[i]];
    }

    *y = tmp / (times - 2);

    /**********************************计算Z轴***********************************/
    tmp = 0;

    for (j = 0; j < times; j++)
    {
        temp_serial[j] = j;
    }

    for (i = times; i > 0; i--)
    {
        for (j = 0; j < (i - 1); j++)
        {
            if (temp_z[temp_serial[j]] > temp_z[temp_serial[j + 1]])
            {
                tmp_serial = temp_serial[j];
                temp_serial[j] = temp_serial[j + 1];
                temp_serial[j + 1] = tmp_serial;
            }
        }
    }

    for (i = 1; i < times - 1; i++)
    {
        tmp += temp_z[temp_serial[i]];
    }

    *z = tmp / (times - 2);

    return 0x01;
}

uint8_t QMC5883L_Init(void)
{
    uint8_t uErrTime = 0;
    uint8_t data = 0;
    /* 读取器件ID */
    data = QMC5883L_Register_Read(QMC5883L_ADDR_CHIPID);
    while (data != QMC5883L_CHIPID_VALUE)
    {
        uErrTime++;
        if (uErrTime > 250)
        {
            return QMC5883L_DATA_ERROR_VALUE;
        }
        HAL_Delay(2);
        data = QMC5883L_Register_Read(QMC5883L_ADDR_CHIPID);
    }

    QMC5883L_Register_Write(QMC5883L_ADDR_CFGB, 0x80);    // 软件复位
    HAL_Delay(100);                                       // 软件复位后进行延时等待
    QMC5883L_Register_Write(QMC5883L_ADDR_PERIORC, 0x01); // 0x0b写入0x00
    HAL_Delay(10);
    QMC5883L_Register_Write(QMC5883L_ADDR_CFGB, 0x41); // 禁止中断引脚，使能数据指针自增
    QMC5883L_Register_Write(QMC5883L_ADDR_CFGC, QMC5883L_CFGC_VALUE);
    QMC5883L_Register_Write(QMC5883L_ADDR_CFGD, QMC5883L_CFGD_VALUE);
    QMC5883L_Register_Write(QMC5883L_ADDR_CFGA, QMC5883L_CFGA_VALUE_CONTINUE); /* OSR = 512;RNG = 8G(0x1d RNG=8G);ODR=200Hz;MODE:待机模式*/

    //
    if (QMC5883L_Register_Read(QMC5883L_ADDR_CFGA) != QMC5883L_CFGA_VALUE_CONTINUE)
    {
        return QMC5883L_DATA_ERROR_VALUE;
    }
    QMC5883LSample.Kx = 1;
    QMC5883LSample.Ky = 1;
    QMC5883LSample.Kz = 1;
    QMC5883_GetOffSet(&QMC5883LSample);

    return QMC5883L_DATA_SUCCESS;
}

void QMC5883L_Soft_Reset(void)
{
    QMC5883L_Register_Write(QMC5883L_ADDR_CFGB, QMC5883L_CFGB_VALUE_REBOOT);
    HAL_Delay(50);
}

void QMC5883L_Hard_Reset(void)
{
    // QMC5883L_POWER_OFF();
    // HAL_Delay(1000);
    // QMC5883L_POWER_ON();
}
QMC5883L_RESULT_SAMPLE QMC5883L_Read_MagDensity(void)
{
    uint8_t i, param_vali;
    uint16_t MagDensity = 0x00;
    int16_t fx = 0x00, fy = 0x00, fz = 0x00;
    int16_t x = 0x00, y = 0x00, z = 0x00;
    int16_t Temp = 0x00;
    // SENSOR_DATA_ELEMENT_T Q_Sample;
    /* 检查传感器 */
    if (QMC5883L_Register_Read(QMC5883L_ADDR_CHIPID) != QMC5883L_CHIPID_VALUE)
    {
        QMC5883LSample.HardFault = 0x01;
        return QMC5883L_DATA_ERROR_ADDR;
    }
    else
    {
        QMC5883LSample.HardFault = 0x00;
    }
    /* QMC5883L开启连续模式 */
    QMC5883L_Register_Write(QMC5883L_ADDR_CFGA, QMC5883L_CFGA_VALUE_CONTINUE);
    for (i = 0; i < 2; i++)
    {
        if (QMC5883L_Wait_Data_Update(6))
        {
            // QMC5883L_Read_XYZ(&x, &y, &z);
        }
        else
        {
            QMC5883L_Register_Write(QMC5883L_ADDR_CFGA, QMC5883L_CFGA_VALUE_STANDBY);
            return QMC5883L_DATA_ERROR_TIMEOUT;
        }
    }
    /* QMC5883L开启待机模式 */
    QMC5883L_Register_Write(QMC5883L_ADDR_CFGA, QMC5883L_CFGA_VALUE_STANDBY);

    // #ifdef QMC5883L_READ_TEMP
    //     QMC5883L_Read_TEMP(&Temp);
    // #endif
    if (((x < 3) && (x > -3)) && ((y < 3) && (y > -3)) && ((z < 3) && (z > -3)))
    {
        param_vali = 0x00;
        QMC5883LSample.DetectInvalid = 0x01;
    }
    else
    {
        param_vali = 0x01;
        QMC5883LSample.DetectInvalid = 0x00;
    }

    QMC5883LSample.X_Data = x;
    QMC5883LSample.Y_Data = y;
    QMC5883LSample.Z_Data = z;
    printf("sample:%.2f,%.2f,%.2f\n", x, y, z);
    // fx = QMC5883LSample.X_Data - QMC5883LSave.X_Offset;
    // fy = QMC5883LSample.Y_Data - QMC5883LSave.Y_Offset;
    // fz = QMC5883LSample.Z_Data - QMC5883LSave.Z_Offset;

    fx = QMC5883LSample.X_Data;
    fy = QMC5883LSample.Y_Data;
    fz = QMC5883LSample.Z_Data;
    MagDensity = (uint16_t)(sqrt(fx * fx + fy * fy + fz * fz));

    // Q_Sample.Bx = fx;
    // Q_Sample.By = fy;
    // Q_Sample.Bz = fz;
    // Q_Sample.Temp = Temp;
    // Q_Sample.BNorm = MagDensity;
    // Q_Enqueue(&Q_Sample, &Q_CtlM);

    if (!param_vali)
    {
        return QMC5883L_DATA_ERROR_VALUE;
    }
    return QMC5883L_DATA_SUCCESS;
}

uint8_t QMC5883_GetOffSet(QMC5883L_DEV *dev)
{
#if GETOFFSET == 1
    uint8_t buf[QMC5883L_XYZBUF_LEN];
    int16_t Xmax = 0, Xmin = 0, Ymax = 0, Ymin = 0, Zmax = 0, Zmin = 0;

    int16_t dataX, dataY, dataZ;
    int32_t sum[3] = {0, 0, 0};
    uint8_t rx;
    for (uint16_t time = 0; time < 500; ++time)
    {
        rx = QMC5883L_Register_Read(QMC5883L_ADDR_STATUS);
        if (rx & 0x01 != 0x01)
        {
            return 0x01;
        }
        HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDR_READ, QMC5883L_ADDR_XOUTL, I2C_MEMADD_SIZE_8BIT, buf, QMC5883L_XYZBUF_LEN, 0xff);
        HAL_Delay(20);
        dataX = (int16_t)((buf[1] << 8) | buf[0]);
        dataY = (int16_t)((buf[3] << 8) | buf[2]);
        dataZ = (int16_t)((buf[5] << 8) | buf[4]);
        dataX = dataX / QMC5883L_SENSITIVITY_8G;
        dataY = dataY / QMC5883L_SENSITIVITY_8G;
        dataZ = dataZ / QMC5883L_SENSITIVITY_8G;
        Xmax = Xmax > dataX ? Xmax : dataX; // 确定最大值
        Xmin = Xmin < dataX ? Xmin : dataX; // 确定最小值
        Ymax = Ymax > dataY ? Ymax : dataY;
        Ymin = Ymin < dataY ? Ymin : dataY;
        Zmax = Zmax > dataZ ? Zmax : dataZ;
        Zmin = Zmin < dataZ ? Zmin : dataZ;
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
    dev->X_Offset = (Xmax + Xmin) / 2;
    dev->Y_Offset = (Ymax + Ymin) / 2;
    dev->Z_Offset = (Zmax + Zmin) / 2;
    dev->Kx = 1;
    dev->Ky = (float)(Xmax - Xmin) / (Ymax - Ymin);
    dev->Kz = (float)(Xmax - Xmin) / (Zmax - Zmin);
#else
    dev->X_Offset = -89;
    dev->Y_Offset = -147;
    dev->Z_Offset = 71;
    dev->Kx = 1;
    dev->Ky = 0.99;
    dev->Kz = 1.06;
#endif
    // dev->Kx = (float)2.0 / (Xmax - Xmin);
    // dev->Ky = (float)2.0 / (Ymax - Ymin);
    // dev->Kz = (float)2.0 / (Zmax - Zmin);
}

uint8_t QMC5883L_HardState(void)
{
    return QMC5883LSample.HardFault;
}

uint8_t QMC5883L_DetectState(void)
{
    return QMC5883LSample.DetectInvalid;
}
