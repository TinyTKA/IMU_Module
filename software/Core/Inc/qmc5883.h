#ifndef __QMC5883_H
#define __QMC5883_H
#include "main.h"

#define QMC_I2C I2C1

/* ������ַ���豸�ӵ�ַ+��дѡ�� */
#define QMC5883L_ADDR_WRITE 0x1A
#define QMC5883L_ADDR_READ 0x1B

/* �豸�Ĵ�����ַ */
#define QMC5883L_ADDR_XOUTL 0x00
#define QMC5883L_ADDR_XOUTH 0x01
#define QMC5883L_ADDR_YOUTHL 0x02
#define QMC5883L_ADDR_YOUTH 0x03
#define QMC5883L_ADDR_ZOUTL 0x04
#define QMC5883L_ADDR_ZOUTH 0x05

#define QMC5883L_ADDR_STATUS 0x06
#define QMC5883L_ADDR_TEMPL 0x07
#define QMC5883L_ADDR_TEMPH 0x08
#define QMC5883L_ADDR_CFGA 0x09
#define QMC5883L_ADDR_CFGB 0x0A
#define QMC5883L_ADDR_PERIORC 0x0B
#define QMC5883L_ADDR_CHIPID 0x0D

#define QMC5883L_ADDR_CFGC 0x20
#define QMC5883L_ADDR_CFGD 0x21

/* �豸�Ĵ�������ֵ */
#define QMC5883L_CFGA_OSR_512 (0 << 7) | (0 << 6)
#define QMC5883L_CFGA_OSR_256 (0 << 7) | (1 << 6)
#define QMC5883L_CFGA_OSR_128 (1 << 7) | (0 << 6)
#define QMC5883L_CFGA_OSR_64 (1 << 7) | (1 << 6)
#define QMC5883L_CFGA_RNG_2G (0 << 5) | (0 << 4)
#define QMC5883L_CFGA_RNG_8G (0 << 5) | (1 << 4)
#define QMC5883L_CFGA_ODR_10HZ (0 << 3) | (0 << 2)
#define QMC5883L_CFGA_ODR_50HZ (0 << 3) | (1 << 2)
#define QMC5883L_CFGA_ODR_100HZ (1 << 3) | (0 << 2)
#define QMC5883L_CFGA_ODR_200HZ (1 << 3) | (1 << 2)
#define QMC5883L_CFGA_MODE_STANDBY (0 << 1) | (0 << 0)
#define QMC5883L_CFGA_MODE_CONTINUE (0 << 1) | (1 << 0)

#define QMC5883L_CFGB_SOFT_RST (1 << 7)
#define QMC5883L_CFGB_ROL_PNT (1 << 6)
#define QMC5883L_CFGB_INT_ENB (1 << 0)

#define QMC5883L_CHIPID_VALUE 0xFF /* ������ʶ,0XFF */

#define QMC5883L_CFGA_VALUE_STANDBY (QMC5883L_CFGA_OSR_512 | QMC5883L_CFGA_RNG_8G | QMC5883L_CFGA_ODR_50HZ | QMC5883L_CFGA_MODE_STANDBY) /* OSR = 512;RNG = 8G;ODR=50Hz;MODE:����ģʽ */

#define QMC5883L_CFGA_VALUE_CONTINUE (QMC5883L_CFGA_OSR_512 | QMC5883L_CFGA_RNG_8G | QMC5883L_CFGA_ODR_50HZ | QMC5883L_CFGA_MODE_CONTINUE) /* OSR = 512;RNG = 8G;ODR=50Hz;MODE:����ģʽ */

#define QMC5883L_CFGB_VALUE_REBOOT (QMC5883L_CFGB_SOFT_RST) /* QMC5883L������� */

#define QMC5883L_CFGC_VALUE 0x40
#define QMC5883L_CFGD_VALUE 0x01
#define QMC5883L_PERIORC_VALUE 0x01

#define QMC5883L_XYZBUF_LEN 0x06
#define QMC5883L_TEMBUF_LEN 0x02

#define QMC5883L_SENSITIVITY_2G (12)
#define QMC5883L_SENSITIVITY_8G (3)

#define QMC5883L_CUM_REBOOT_MAXCNT (100)                      /* �������ۼ����������� */
#define QMC5883L_CON_REBOOT_MAXCNT QMC5883L_CUM_REBOOT_MAXCNT /* �������������������� */


//����� 0��ʹ�ù̶�ֵ 1��ÿ���ϵ�У׼
#define GETOFFSET 0


/* QMC5883L ������� */

/* QMC5883L �ܽ� */
#define QMC5883L_PIN_SCL BSP_IIC1_PIN_SCL
#define QMC5883L_PIN_SDA BSP_IIC1_PIN_SDA
#define QMC5883L_PORT_SDA_SCL BSP_IIC1_PORT_SDA_SCL

#define QMC5883L_READ_TEMP /* ʹ��QMC58883L�ڲ��¶� */

typedef struct
{
    float X_Data;        /* ������X������ */
    float Y_Data;        /* ������Y������ */
    float Z_Data;        /* ������Z������ */
    int16_t T_Data;      /* �������ڲ��¶� */
    uint16_t MagDensity; /* �������ܴ���ǿ�� */
    float XY_Angle;      /* ������X��Y��н� */
    float XZ_Angle;      /* ������X��Z��н� */
    float YZ_Angle;      /* ������Y��Z��н� */

    int16_t X_Offset; /* ������X��OFFSET */
    int16_t Y_Offset; /* ������Y��OFFSET */
    int16_t Z_Offset; /* ������Z��OFFSET */
    float Kx;
    float Ky;
    float Kz;

    uint8_t HardFault;     /* ������Ӳ���� */
    uint8_t DetectInvalid; /* ���������ʧЧ */
} QMC5883L_DEV;

typedef enum
{
    QMC5883L_DATA_SUCCESS,       /* ��ȡ���ݳɹ� */
    QMC5883L_DATA_ERROR_ADDR,    /* ������ͨ�ŵ�ַ���� */
    QMC5883L_DATA_ERROR_TIMEOUT, /* ��������ȡ��ʱ */
    QMC5883L_DATA_ERROR_VALUE,   /* ��������ֵ���� */
} QMC5883L_RESULT_SAMPLE;
/*
***********************************************************************************************
*                                            EXTERNS
***********************************************************************************************
*/
extern QMC5883L_RESULT_SAMPLE QMC5883err;
/*
***********************************************************************************************
*                                  GLOBAL FUNCTION PROTOTYPES
***********************************************************************************************
*/
// �������
void QMC5883L_Soft_Reset(void);
// ��ʼ��
uint8_t QMC5883L_Init(void);
// ���״̬
uint8_t QMC5883L_HardState(void);
uint8_t QMC5883L_DetectState(void);
// uint8_t QMC5883L_Adjust_Magnetic(void);
QMC5883L_RESULT_SAMPLE QMC5883L_Read_MagDensity(void);

void QMC5883L_Register_Write(uint8_t addr, uint8_t val);
uint8_t QMC5883L_Register_Read(uint8_t addr);
void QMC5883L_Read_XYZ(QMC5883L_DEV *dev);
void QMC5883L_Read_TEMP(float *temp);

uint8_t QMC5883L_Wait_Data_Update(uint8_t max_cnt);
uint8_t QMC5883L_Read_Average(uint16_t *x, uint16_t *y, uint16_t *z, uint8_t times);
uint8_t QMC5883_GetOffSet(QMC5883L_DEV *dev);
#endif
