#include "spl06.h"
#include "mDelay.h"
#include "debug.h"
#include "math.h"
SPL06_DEV spl06;

const float T0 = 288.15f; // 参考温度（单位：开尔文）
const float L = -0.0065f; // 温度梯度（单位：K/m）
const float p0 = 101325;  // 参考气压（单位：帕斯卡）
const float R = 8.314;    // 气体常数（单位：J/(mol·K)）
const float g = 9.80665;  // 重力加速度（单位：m/s^2）

// 高度计算

void SPL06_Init(SPL06_DEV *dev)
{
    uint8_t err = 0;
    HAL_Delay(10);
    SPL_06_Write_Reg(RESET_REG, SOFT_RESET_FIFO_FLUSH_VALUE); // 清空FIFO,软件重置芯片
    HAL_Delay(100);
    SPL_06_Read_Reg(ID_REG, &err, 1); // 读取ID
    if (err != 0x10)
    {
        printf("SPL06 ID error!\n");
        while (1)
        {
            HAL_Delay(100);
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
    }
    SPL_06_Read_Reg(MEAS_CFG_REG, &err, 1);
    while (err & 0xc0 != 0xc0)
    {
        HAL_Delay(100);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        SPL_06_Read_Reg(MEAS_CFG_REG, &err, 1);
    }

    SPL_06_Write_Reg(CFG_REG_REG, 0x80); // 关闭所有中断，禁止FIFO,设置4线SPI
    // 读取校准系数
    SPL_06_Read_COEF();

    SPL_06_Config_Pres(dev, PREM_RATE_64, OVERSAMPLE_16); // 每秒采32次，每次采8次
    SPL_06_Config_Temp(dev, TEMP_RATE_64, OVERSAMPLE_16); // 每秒采集16次温度，每次采集只采样4次

    SPL_06_Config_Mode(MODE_CONTINUE_PRES_TEMP_MEAS); // 气压和温度持续采集模式
    HAL_Delay(100);
}

void SPL_06_Config_Pres(SPL06_DEV *dev, uint8_t rate, uint8_t sample)
{
    uint8_t temp = 0;
    switch (sample)
    {
    case 0:
        dev->kp = 524728.0f;
        break;
    case 1:
        dev->kp = 1572864.0f;
        break;
    case 2:
        dev->kp = 3670016.0f;
        break;
    case 3:
        dev->kp = 7864320.0f;
        break;
    case 4:
        dev->kp = 253952.0f;
        break;
    case 5:
        dev->kp = 516096.0f;
        break;
    case 6:
        dev->kp = 1040384.0f;
        break;
    case 7:
        dev->kp = 2088960.0f;
        break;
    }
    // 气压配置寄存器
    SPL_06_Write_Reg(PRS_CFG_REG, rate | sample);
    SPL_06_Read_Reg(CFG_REG_REG, &temp, 1);
    if (sample > 3)
    {
        temp = temp | (1 << 2); // P_SHIFT 位置1
        SPL_06_Write_Reg(CFG_REG_REG, temp);
    }
    else
    {
        temp = temp & 0xfb; // bit2 清0
        SPL_06_Write_Reg(CFG_REG_REG, temp);
    }
}
void SPL_06_Config_Temp(SPL06_DEV *dev, uint8_t rate, uint8_t sample)
{
    uint8_t temp = 0;
    switch (sample)
    {
    case 0:
        dev->kt = 524728.0f;
        break;
    case 1:
        dev->kt = 1572864.0f;
        break;
    case 2:
        dev->kt = 3670016.0f;
        break;
    case 3:
        dev->kt = 7864320.0f;
        break;
    case 4:
        dev->kt = 253952.0f;
        break;
    case 5:
        dev->kt = 516096.0f;
        break;
    case 6:
        dev->kt = 1040384.0f;
        break;
    case 7:
        dev->kt = 2088960.0f;
        break;
    }
    // 气压配置寄存器
    SPL_06_Write_Reg(TMP_CFG_REG, (1 << 7) | rate | sample);
    SPL_06_Read_Reg(CFG_REG_REG, &temp, 1);
    if (sample > 3)
    {
        temp = temp | (1 << 3); // T_SHIFT 位置1
        SPL_06_Write_Reg(CFG_REG_REG, temp);
    }
    else
    {
        temp = temp & 0xf7; // bit3 清0
        SPL_06_Write_Reg(CFG_REG_REG, temp);
    }
}

void SPL_06_Config_Mode(uint8_t mode)
{
    SPL_06_Write_Reg(MEAS_CFG_REG, mode);
}
void SPL_06_Read_Reg(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t ptxdata = 0;
    HAL_GPIO_WritePin(SPI1_SPL_CS_GPIO_Port, SPI1_SPL_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    ptxdata = addr | SPI1_SPL06_READ;
    HAL_SPI_Transmit(&hspi1, &ptxdata, 1, 100);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&hspi1, data, len, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
        ;
    delay_us(2);
    HAL_GPIO_WritePin(SPI1_SPL_CS_GPIO_Port, SPI1_SPL_CS_Pin, GPIO_PIN_SET);
}
void SPL_06_Write_Reg(uint8_t addr, uint8_t data)
{
    uint8_t ptxdata[2];

    HAL_GPIO_WritePin(SPI1_SPL_CS_GPIO_Port, SPI1_SPL_CS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    ptxdata[0] = addr & SPI1_SPL06_WRITE;
    ptxdata[1] = data;
    HAL_SPI_Transmit(&hspi1, ptxdata, 2, 10);
    // HAL_SPI_Transmit(&hspi1, ptxdata, 2, 10);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    delay_us(2);
    HAL_GPIO_WritePin(SPI1_SPL_CS_GPIO_Port, SPI1_SPL_CS_Pin, GPIO_PIN_SET);
}

void SPL_06_Read_COEF(void)
{
    uint8_t COEF[18];

    SPL_06_Read_Reg(0x10, COEF, 18);

    spl06.c0 = ((uint16_t)COEF[0]) << 4 | (uint16_t)COEF[1] >> 4;
    spl06.c0 = (spl06.c0 & 0x800) ? (0xf000 | spl06.c0) : spl06.c0;
    spl06.c1 = (uint16_t)(COEF[1] & 0x0F) << 8 | (uint16_t)COEF[2];
    spl06.c1 = (spl06.c1 & 0x800) ? (0xf000 | spl06.c1) : spl06.c1;
    spl06.c00 = (uint32_t)(COEF[3]) << 12 | (uint32_t)(COEF[4]) << 4 | COEF[5] >> 4;
    spl06.c00 = (spl06.c00 & 0x80000) ? (0xfff00000 | spl06.c00) : spl06.c00;
    spl06.c10 = ((uint32_t)(COEF[5]) & 0x0F) << 16 | (uint32_t)(COEF[6]) << 8 | (uint32_t)COEF[7];
    spl06.c10 = (spl06.c10 & 0x80000) ? (0xfff00000 | spl06.c10) : spl06.c10;
    spl06.c01 = (uint16_t)(COEF[8]) << 8 | COEF[9];
    spl06.c11 = (uint16_t)(COEF[10]) << 8 | COEF[11];
    spl06.c20 = (uint16_t)(COEF[12]) << 8 | COEF[13];
    spl06.c21 = (uint16_t)(COEF[14]) << 8 | COEF[15];
    spl06.c30 = (uint16_t)(COEF[16]) << 8 | COEF[17];
}

void SPL_06_Get_Height(SPL06_DEV *dev)
{
    uint8_t mbuff[6];
    static uint8_t cnt = 1;
    uint8_t rx;
    int32_t Praw;
    int32_t Traw;
    float Praw_sc;
    float Traw_sc;
    float qua2;
    float qua3;
    // SPL_06_Read_Reg(MEAS_CFG_REG, &rx, 1); // 检查是否有新数据

    SPL_06_Read_Reg(PRS_B2_REG, mbuff, 6);
    Praw = (uint32_t)mbuff[0] << 16 | (uint32_t)mbuff[1] << 8 | mbuff[2];
    Praw = (Praw & 0x800000) ? (0xff000000 | Praw) : Praw;
    Traw = (uint32_t)mbuff[3] << 16 | (uint32_t)mbuff[4] << 8 | mbuff[5];
    Traw = (Traw & 0x800000) ? (0xff000000 | Traw) : Traw;
    Praw_sc = (float)Praw / dev->kp;
    Traw_sc = (float)Traw / dev->kt;
    qua2 = (float)(dev->c10) + Praw_sc * ((float)dev->c20 + Praw_sc * (float)dev->c30);
    qua3 = Traw_sc * Praw_sc * ((float)dev->c11 + Praw_sc * (float)dev->c21);
    dev->P = (float)dev->c00 + Praw_sc * (qua2) + Traw_sc * (float)dev->c01 + qua3;
    dev->temp = (float)dev->c0 * 0.5f + (float)dev->c1 * Traw_sc;
    // double m = (double)dev->P / 101325.0f;
    double mi = 1 / 5.255;
    // double m2 = pow(m, mi);
    // dev->height = -(101325.0f - dev->P) * 0.0843f / 100.0f;
    // dev->height = ((dev->P-102000.f) * 78.740f)/1000.0f;
    //  dev->height = (T0 / L) * (1.0f - powf(dev->P / p0, R * L / g));
    dev->height = (double)44330.0f * (1 - pow((double)dev->P / 101325.0f, mi));
    if (cnt == 1)
    {
        dev->Ori_height = dev->height;
        ++cnt;
    }
    else
    {
        dev->height = dev->height - dev->Ori_height;
    }
}
void SPL_O6_Get_Temp(SPL06_DEV *dev)
{
}