#ifndef __SPL06_H
#define __SPL06_H
#include "main.h"

#include "spi.h"
#define SPI1_SPL06_READ 0x80
#define SPI1_SPL06_WRITE 0x7f

/*SPL06-001 寄存器列表*/
#define PRS_B2_REG 0x00
#define PRS_B1_REG 0x01
#define PRS_B0_REG 0x02
#define TMP_B2_REG 0x03
#define TMP_B1_REG 0x04
#define TMP_B0_REG 0x05
#define PRS_CFG_REG 0x06
#define TMP_CFG_REG 0x07
#define MEAS_CFG_REG 0x08
#define CFG_REG_REG 0x09
#define INT_STS_REG 0x0A
#define FIFO_STS_REG 0x0B
#define RESET_REG 0x0C
#define ID_REG 0x0D
#define COEF_REG 0X10

// 气压采样频率
#define PREM_RATE_1 0 << 4
#define PREM_RATE_2 1 << 4
#define PREM_RATE_4 2 << 4
#define PREM_RATE_8 3 << 4
#define PREM_RATE_16 4 << 4
#define PREM_RATE_32 5 << 4
#define PREM_RATE_64 6 << 4
#define PREM_RATE_128 7 << 4

// 温度采样频率
#define TEMP_RATE_1 0 << 4
#define TEMP_RATE_2 1 << 4
#define TEMP_RATE_4 2 << 4
#define TEMP_RATE_8 3 << 4
#define TEMP_RATE_16 4 << 4
#define TEMP_RATE_32 5 << 4
#define TEMP_RATE_64 6 << 4
#define TEMP_RATE_128 7 << 4
// 过采样率
#define OVERSAMPLE_1 0
#define OVERSAMPLE_2 1
#define OVERSAMPLE_4 2
#define OVERSAMPLE_8 3
#define OVERSAMPLE_16 4
#define OVERSAMPLE_32 5
#define OVERSAMPLE_64 6
#define OVERSAMPLE_128 7
// 重置芯片
#define SOFT_RESET_FIFO_FLUSH_VALUE ((1 << 7) | 9)
// 设置测量模式
#define MODE_STANBY 0
#define MODE_PRESSURE_MEAS 1
#define MODE_TEMP_MEAS 2
#define MODE_CONTINUE_PRES_MEAS ((1 << 2) | (0 << 1) | (1 << 0))
#define MODE_CONTIMUE_TEMP_MEAS ((1 << 2) | (1 << 1) | (0 << 0))
#define MODE_CONTINUE_PRES_TEMP_MEAS ((1 << 2) | (1 << 1) | (1 << 0))

typedef struct
{
    uint8_t Pmeas;
    uint8_t Poversample;
    uint8_t Tmeas;
    uint8_t Toversample;
    uint8_t originPSR[3];
    uint8_t originTMP[3];
    float temp;
    float height;
    float Ori_height;   //启动时的高度
    float P; // 气压
    uint8_t ID;
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
    float kp;
    float kt;

} SPL06_DEV;

// 初始化
void SPL06_Init(SPL06_DEV *dev);

void SPL_06_Config_Pres(SPL06_DEV *dev, uint8_t rate, uint8_t sample);
void SPL_06_Config_Temp(SPL06_DEV *dev, uint8_t rate, uint8_t sample);
void SPL_06_Config_Mode(uint8_t mode);
void SPL_06_Read_Reg(uint8_t addr, uint8_t *data, uint8_t len);
void SPL_06_Write_Reg(uint8_t addr, uint8_t data);
void SPL_06_Read_COEF(void);
void SPL_06_Get_Height(SPL06_DEV *dev);
void SPL_O6_Get_Temp(SPL06_DEV *dev);
void SPL_06_Config(SPL06_DEV *dev);
#endif
