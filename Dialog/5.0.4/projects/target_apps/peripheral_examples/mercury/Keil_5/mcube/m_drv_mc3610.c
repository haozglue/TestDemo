/*****************************************************************************
 *
 * Copyright (c) 2015 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

/*******************************************************************************
 *** INFORMATION
 *******************************************************************************/
#define M_DRV_MC3610_VERSION    "1.0.4"

/*******************************************************************************
 *** CONFIGURATION
 *******************************************************************************/
#define M_DRV_MC3610_CFG_BUS_I2C    // !!! DO NOT use both I2C and SPI simutaneously
//#define M_DRV_MC3610_CFG_BUS_SPI

#if (!defined (M_DRV_MC3610_CFG_BUS_SPI) && !defined (M_DRV_MC3610_CFG_BUS_I2C))
    #error "MUST use one bus to access register!"
#endif

#if (defined (M_DRV_MC3610_CFG_BUS_SPI) && defined (M_DRV_MC3610_CFG_BUS_I2C))
    #error "DO NOT use both SPI and I2C simutaneously!"
#endif

#define M_DRV_MC3610_CFG_I2C_ADDR    (0x4C)

#define M_DRV_MC3610_CFG_SAMPLE_RATE_CWAKE_DEFAULT    E_M_DRV_MC3610_CWAKE_SR_LP_50Hz
#define M_DRV_MC3610_CFG_SAMPLE_RATE_SNIFF_DEFAULT    E_M_DRV_MC3610_SNIFF_SR_6Hz     // DO NOT MODIFY THIS

#define M_DRV_MC3610_CFG_RANGE         E_M_DRV_MC3610_RANGE_4G
#define M_DRV_MC3610_CFG_RESOLUTION    E_M_DRV_MC3610_RESOLUTION_12BIT

#define M_DRV_MC3610_CFG_ORIENTATION_MAP    E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_UP

#define M_DRV_MC3610_CFG_WA_1
#define M_DRV_MC3610_CFG_WA_2
#define M_DRV_MC3610_CFG_WA_7
#define M_DRV_MC3610_CFG_WA_8
#define M_DRV_MC3610_CFG_WA_9
#define M_DRV_MC3610_CFG_WA_10
#define M_DRV_MC3610_CFG_WA_12

#define M_DRV_MC3610_CFG_ASC

//#define M_DRV_MC3610_OPERATE_MODE_WAKE_WHEN_READ
#define M_DRV_MC3610_OPERATE_MODE_ALWAYS_WAKE

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/
#include <stdint.h>

#ifdef M_DRV_MC3610_CFG_BUS_I2C
    // hook by custom, include your i2c herader file
#else
    // hook by custom, include your spi header file
#endif

#include "m_drv_mc3610.h"
#include "m_drv_mc_utility.h"

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define M_DRV_MC3610_I2C_WRITE_ADDR    ((M_DRV_MC3610_CFG_I2C_ADDR << 1) | 0x00)
#define M_DRV_MC3610_I2C_READ_ADDR     ((M_DRV_MC3610_CFG_I2C_ADDR << 1) | 0x01)

//=============================================
#define M_DRV_MC3610_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define M_DRV_MC3610_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define M_DRV_MC3610_INTR_C_IAH_ACTIVE_LOW     (0x00)
#define M_DRV_MC3610_INTR_C_IAH_ACTIVE_HIGH    (0x02)

//=============================================
#define M_DRV_MC3610_REG_EXT_STAT_1       (0x00)
#define M_DRV_MC3610_REG_EXT_STAT_2       (0x01)
#define M_DRV_MC3610_REG_XOUT_LSB         (0x02)
#define M_DRV_MC3610_REG_XOUT_MSB         (0x03)
#define M_DRV_MC3610_REG_YOUT_LSB         (0x04)
#define M_DRV_MC3610_REG_YOUT_MSB         (0x05)
#define M_DRV_MC3610_REG_ZOUT_LSB         (0x06)
#define M_DRV_MC3610_REG_ZOUT_MSB         (0x07)
#define M_DRV_MC3610_REG_STATUS_1         (0x08)
#define M_DRV_MC3610_REG_STATUS_2         (0x09)
#define M_DRV_MC3610_REG_MODE_C           (0x10)
#define M_DRV_MC3610_REG_WAKE_C           (0x11)
#define M_DRV_MC3610_REG_SNIFF_C          (0x12)
#define M_DRV_MC3610_REG_SNIFFTH_C        (0x13)
#define M_DRV_MC3610_REG_IO_C             (0x14)
#define M_DRV_MC3610_REG_RANGE_C          (0x15)
#define M_DRV_MC3610_REG_FIFO_C           (0x16)
#define M_DRV_MC3610_REG_INTR_C           (0x17)
#define M_DRV_MC3610_REG_DMX              (0x20)
#define M_DRV_MC3610_REG_DMY              (0x21)
#define M_DRV_MC3610_REG_DMZ              (0x22)
#define M_DRV_MC3610_REG_XOFFL            (0x2A)
#define M_DRV_MC3610_REG_XOFFH            (0x2B)
#define M_DRV_MC3610_REG_YOFFL            (0x2C)
#define M_DRV_MC3610_REG_YOFFH            (0x2D)
#define M_DRV_MC3610_REG_ZOFFL            (0x2E)
#define M_DRV_MC3610_REG_ZOFFH            (0x2F)
#define M_DRV_MC3610_REG_XGAIN            (0x30)
#define M_DRV_MC3610_REG_YGAIN            (0x31)
#define M_DRV_MC3610_REG_ZGAIN            (0x32)
#define M_DRV_MC3610_REG_OPT              (0x3B)
#define M_DRV_MC3610_REG_LOC_X            (0x3C)
#define M_DRV_MC3610_REG_LOC_Y            (0x3D)
#define M_DRV_MC3610_REG_LOT_dAOFSZ       (0x3E)
#define M_DRV_MC3610_REG_WAF_LOT          (0x3F)

#define M_DRV_MC3610_NULL_ADDR    (0)

#ifdef M_DRV_MC3610_CFG_ASC
    #define M_DRV_MC3610_ASC_INIT_DIFF_X    (16)
    #define M_DRV_MC3610_ASC_INIT_DIFF_Y    (16)
    #define M_DRV_MC3610_ASC_INIT_DIFF_Z    (16)

    #define M_DRV_MC3610_ASC_DATA_SIZE    (6)

    #define M_DRV_MC3610_ASC_TARGET_XY_LOW     (12300)
    #define M_DRV_MC3610_ASC_TARGET_XY_MID     (13100)
    #define M_DRV_MC3610_ASC_TARGET_XY_HIGH    (13900)
    
    #define M_DRV_MC3610_ASC_TARGET_Z_LOW     (-13400)
    #define M_DRV_MC3610_ASC_TARGET_Z_MID     (-12600)
    #define M_DRV_MC3610_ASC_TARGET_Z_HIGH    (-11800)
#endif

#define M_DRV_MC3610_SR_MODE_UNKNOWN            (0xFF)
#define M_DRV_MC3610_SR_MODE_ULTRA_LOW_POWER    (0x03)
#define M_DRV_MC3610_SR_MODE_LOW_POWER          (0x00)
#define M_DRV_MC3610_SR_MODE_PRECISION          (0x02)

/*******************************************************************************
 *** MACRO
 *******************************************************************************/
// hook by custom, replace "M_DRV_I2C_Write" by your I2C Write function, if using I2C
// hook by custom, replace "M_DRV_I2C_Read" by your I2C Read function, if using I2C
// hook by custom, replace "M_DRV_SPI_Write" by your SPI Write function, if using SPI
// hook by custom, replace "M_DRV_SPI_Read" by your SPI Read function, if using SPI
#ifdef M_DRV_MC3610_CFG_BUS_I2C
    #define _M_DRV_MC3610_REG_WRITE(bRegAddr, pbDataBuf, bLength)    M_DRV_I2C_Write(M_DRV_MC3610_I2C_WRITE_ADDR, bRegAddr, pbDataBuf, bLength)
    #define _M_DRV_MC3610_REG_READ(bRegAddr, pbDataBuf, bLength)     M_DRV_I2C_Read(M_DRV_MC3610_I2C_READ_ADDR, bRegAddr, pbDataBuf, bLength)
#else
    #ifdef M_DRV_MC3610_CFG_WA_1
        #define _M_DRV_MC3610_REG_WRITE(bRegAddr, pbDataBuf, bLength)                           \
                {                                                                               \
                    if (1 == bLength)                                                           \
                        M_DRV_SPI_Write(bRegAddr, pbDataBuf, 1);                                \
                    else if (1 < bLength)                                                       \
                    {                                                                           \
                        unsigned char    _bIndex = 0;                                           \
                                                                                                \
                        for (_bIndex = 0; _bIndex < bLength; _bIndex++)                         \
                            M_DRV_SPI_Write((bRegAddr + _bIndex), (pbDataBuf + _bIndex), 1);    \
                    }                                                                           \
                }
    #else
        #define _M_DRV_MC3610_REG_WRITE(bRegAddr, pbDataBuf, bLength)    M_DRV_SPI_Write(bRegAddr, pbDataBuf, bLength)
    #endif

    #ifdef M_DRV_MC3610_CFG_WA_2
        #define _M_DRV_MC3610_REG_READ(bRegAddr, pbDataBuf, bLength)                                        \
                {                                                                                           \
                    if (1 == bLength)                                                                       \
                        M_DRV_SPI_Read((bRegAddr | 0x80), pbDataBuf, 1);                                    \
                    else if (1 < bLength)                                                                   \
                    {                                                                                       \
                        if ((6 == bLength) && (M_DRV_MC3610_REG_XOUT_LSB == bRegAddr))                      \
                            M_DRV_SPI_Read((bRegAddr | 0x80), pbDataBuf, bLength);                          \
                        else                                                                                \
                        {                                                                                   \
                            unsigned char    _bIndex = 0;                                                   \
                                                                                                            \
                            for (_bIndex = 0; _bIndex < bLength; _bIndex++)                                 \
                                M_DRV_SPI_Read(((bRegAddr + _bIndex) | 0x80), (pbDataBuf + _bIndex), 1);    \
                        }                                                                                   \
                    }                                                                                       \
                }
    #else
        #define _M_DRV_MC3610_REG_READ(bRegAddr, pbDataBuf, bLength)    M_DRV_SPI_Read((bRegAddr | 0x80), pbDataBuf, bLength)
    #endif
#endif

#define _M_DRV_MC3610_REG_STATUS_1_MODE(bRegStatus1)           (bRegStatus1 & 0x07)
#define _M_DRV_MC3610_REG_STATUS_1_NEW_DATA(bRegStatus1)       (bRegStatus1 & 0x08)
#define _M_DRV_MC3610_REG_STATUS_1_FIFO_EMPTY(bRegStatus1)     (bRegStatus1 & 0x10)
#define _M_DRV_MC3610_REG_STATUS_1_FIFO_FULL(bRegStatus1)      (bRegStatus1 & 0x20)
#define _M_DRV_MC3610_REG_STATUS_1_FIFO_THRESH(bRegStatus1)    (bRegStatus1 & 0x40)
#define _M_DRV_MC3610_REG_STATUS_1_INT_PEND(bRegStatus1)       (bRegStatus1 & 0x80)

#define _M_DRV_MC3610_REG_STATUS_2_INT_WAKE(bRegStatus2)           ((bRegStatus2 >> 2) & 0x01)
#define _M_DRV_MC3610_REG_STATUS_2_INT_ACQ(bRegStatus2)            ((bRegStatus2 >> 3) & 0x01)
#define _M_DRV_MC3610_REG_STATUS_2_INT_FIFO_EMPTY(bRegStatus2)     ((bRegStatus2 >> 4) & 0x01)
#define _M_DRV_MC3610_REG_STATUS_2_INT_FIFO_FULL(bRegStatus2)      ((bRegStatus2 >> 5) & 0x01)
#define _M_DRV_MC3610_REG_STATUS_2_INT_FIFO_THRESH(bRegStatus2)    ((bRegStatus2 >> 6) & 0x01)

#define _M_DRV_MC3610_REG_MODE_C_MODE(bRegMODE_C)           (bRegMODE_C & 0x07)

#define _M_DRV_MC3610_REG_RANGE_C_RES(bRegRANGE_C)      ( bRegRANGE_C       & 0x07)
#define _M_DRV_MC3610_REG_RANGE_C_RANGE(bRegRANGE_C)    ((bRegRANGE_C >> 4) & 0x07)

#define _M_DRV_MC3610_REG_FIFO_C_FIFO_EN(bRegFIFO_C)    (bRegFIFO_C & 0x40)

#define _M_DRV_MC3610_M_ABS(x)    (((x) < 0) ? (-(x)) : (x))

/*******************************************************************************
 *** STATIC VARIABLE
 *******************************************************************************/
static float    s_fMC3XXX_Sensitivity = 0;    // unit: SI/LSB, SI: m/s^2

static E_M_DRV_MC3610_RANGE         s_eRange      = M_DRV_MC3610_CFG_RANGE;
static E_M_DRV_MC3610_RESOLUTION    s_eResolution = M_DRV_MC3610_CFG_RESOLUTION;

static E_M_DRV_MC3610_CWAKE_SR    s_eSR_CWAKE = M_DRV_MC3610_CFG_SAMPLE_RATE_CWAKE_DEFAULT;
static E_M_DRV_MC3610_SNIFF_SR    s_eSR_SNIFF = M_DRV_MC3610_CFG_SAMPLE_RATE_SNIFF_DEFAULT;

static unsigned char    s_bCfgRngResol = 0x00;
static unsigned char    s_bCfgSniffThr = 0x00;
static unsigned char    s_bCfgFifo     = 0x00;
static unsigned char    s_bCfgINT      = 0x00;
static unsigned char    s_bCfgSRMode   = M_DRV_MC3610_SR_MODE_UNKNOWN;

#ifdef M_DRV_MC3610_CFG_ASC
    static unsigned char    s_baASC_W[M_DRV_MC3610_ASC_DATA_SIZE];
    static unsigned char    s_baASC_S[M_DRV_MC3610_ASC_DATA_SIZE];
#endif

/*******************************************************************************
 *** STATIC FUNCTION
 *******************************************************************************/
/*********************************************************************
 *** _M_DRV_MC3610_Delay
 *********************************************************************/
int    _M_DRV_MC3610_Delay(uint32_t dwMs)
{
    // hook by custom to your delay function

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3610_SetBusIF
 *****************************************/
static int    _M_DRV_MC3610_SetBusIF(void)
{
    unsigned char    _bRegSNIFFTH_C = 0;
    unsigned char    _bRegIO_C      = 0;

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_SNIFFTH_C, &_bRegSNIFFTH_C, 1);
    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_IO_C     , &_bRegIO_C     , 1);

    #ifdef M_DRV_MC3610_CFG_BUS_I2C
        _bRegSNIFFTH_C |= 0x80;
        _bRegIO_C      &= 0x7F;
    #else    //#ifdef M_DRV_MC3610_CFG_BUS_SPI
        _bRegSNIFFTH_C &= 0x7F;
        _bRegIO_C      |= 0x80;
    #endif

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_SNIFFTH_C, &_bRegSNIFFTH_C, 1);
    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_IO_C     , &_bRegIO_C     , 1);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3610_ResetChip
 *****************************************/
static void    _M_DRV_MC3610_ResetChip(void)
{
    unsigned char    _bRegData = 0x01;

    _M_DRV_MC3610_REG_WRITE(0x10, &_bRegData, 1);

    _M_DRV_MC3610_Delay(10);

    _bRegData = 0x40;
    _M_DRV_MC3610_REG_WRITE(0x24, &_bRegData, 1);

    #ifdef M_DRV_MC3610_CFG_BUS_SPI
        _M_DRV_MC3610_SetBusIF();
    #endif

    _M_DRV_MC3610_Delay(50);

    _M_DRV_MC3610_SetBusIF();

    _bRegData = 0x01;
    _M_DRV_MC3610_REG_WRITE(0x10, &_bRegData, 1);

    _M_DRV_MC3610_Delay(10);
}

/*****************************************
 *** _M_DRV_MC3610_ValidateSensorIC
 *****************************************/
static int    _M_DRV_MC3610_ValidateSensorIC(void)
{
    unsigned char    _bRegData = 0;

    _M_DRV_MC3610_REG_READ(0x18, &_bRegData, 1);

    if (0x70 == _bRegData)
    {
        _M_DRV_MC3610_REG_READ(0x34, &_bRegData, 1);

        if (0x30 == _bRegData)
        {
            _M_DRV_MC3610_ResetChip();

            _M_DRV_MC3610_REG_READ(0x3B, &_bRegData, 1);

            if (0x10 == (_bRegData & 0xF3))
                return (M_DRV_MC3610_RETCODE_SUCCESS);
        }
    }

    return (M_DRV_MC3610_RETCODE_ERROR_IDENTIFICATION);
}

/*****************************************
 *** _M_DRV_MC3610_SetSampleRateMode
 *****************************************/
static void    _M_DRV_MC3610_SetSampleRateMode(uint8_t bDesiredSRMode)
{
    unsigned char    _bRegData = 0x00;

    if (s_bCfgSRMode == bDesiredSRMode)
        return;

    _M_DRV_MC3610_REG_READ(0x01, &_bRegData, 1);

    if (0x00 == (_bRegData & 0x10))
    {
        _bRegData = 0x97;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);
    
        _bRegData = 0x5E;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);
    
        _bRegData = 0xF1;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);
    }

    _M_DRV_MC3610_REG_READ(0x3B, &_bRegData, 1);

    _bRegData &= 0xFC;
    _bRegData |= bDesiredSRMode;

    _M_DRV_MC3610_REG_WRITE(0x3B, &_bRegData, 1);

    s_bCfgSRMode = bDesiredSRMode;
}

#ifdef M_DRV_MC3610_CFG_ASC
/*****************************************
*** _M_DRV_MC3610_ASC_ReadSensorData
*****************************************/
static void    _M_DRV_MC3610_ASC_ReadSensorData(unsigned char *pbData)
{
    unsigned char    _bRegData    = 0x05;
    unsigned char    _bCheckCount = 0x00;

    _M_DRV_MC3610_REG_WRITE(0x10, &_bRegData, 1);

    for (_bCheckCount = 0; _bCheckCount < 16; _bCheckCount++)
    {
        _M_DRV_MC3610_Delay(50);

        _M_DRV_MC3610_REG_READ(0x08, &_bRegData, 1);

        if (_bRegData & 0x08)
            break;
    }

    _M_DRV_MC3610_REG_READ(0x02, pbData, 6);

    _bRegData = 0x01;
    _M_DRV_MC3610_REG_WRITE(0x10, &_bRegData, 1);
}

/*****************************************
*** _M_DRV_MC3610_ASC_Handler
*****************************************/
static void    _M_DRV_MC3610_ASC_Handler(uint8_t bAxis, unsigned char *pbAofs)
{
    #define _ASC_TARGET_LOW_      (0)
    #define _ASC_TARGET_MID_      (1)
    #define _ASC_TARGET_HIGH_     (2)
    #define _ASC_TARGET_COUNT_    (3)

    signed short    _wInitData = 0;
    signed short    _wData     = 0;
    signed short    _wSlope    = 0;
    signed short    _wDiff     = 0;
    signed short    _wFSft     = 1;

    unsigned char    _baData[6] = { 0 };

    unsigned char    _baInitDiff[M_DRV_MC3610_AXES_NUM] = { M_DRV_MC3610_ASC_INIT_DIFF_X, M_DRV_MC3610_ASC_INIT_DIFF_Y, M_DRV_MC3610_ASC_INIT_DIFF_Z};

    signed short     _waTarget[M_DRV_MC3610_AXES_NUM][_ASC_TARGET_COUNT_] =
                         { 
                             {M_DRV_MC3610_ASC_TARGET_XY_LOW, M_DRV_MC3610_ASC_TARGET_XY_MID, M_DRV_MC3610_ASC_TARGET_XY_HIGH},
                             {M_DRV_MC3610_ASC_TARGET_XY_LOW, M_DRV_MC3610_ASC_TARGET_XY_MID, M_DRV_MC3610_ASC_TARGET_XY_HIGH},
                             {M_DRV_MC3610_ASC_TARGET_Z_LOW , M_DRV_MC3610_ASC_TARGET_Z_MID , M_DRV_MC3610_ASC_TARGET_Z_HIGH }
                         };

    M_PRINTF("[%s] %d", __func__, bAxis);

    _M_DRV_MC3610_ASC_ReadSensorData(_baData);

    _wInitData = ((signed short) ((_baData[(bAxis << 1)]) | (_baData[((bAxis << 1) + 1)] << 8)));

    M_PRINTF("    <_wInitData(%d)> %d, 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             bAxis, _wInitData, _baData[0], _baData[1], _baData[2], _baData[3], _baData[4], _baData[5]);

    pbAofs[ (bAxis << 1)     ] -= _baInitDiff[bAxis];
    pbAofs[((bAxis << 1) + 1)] += _baInitDiff[bAxis];

    _M_DRV_MC3610_REG_WRITE((0x35 + (bAxis << 1)), &pbAofs[(bAxis << 1)], 2); 

    _M_DRV_MC3610_ASC_ReadSensorData(_baData);

    _wData = ((signed short) ((_baData[(bAxis << 1)]) | (_baData[((bAxis << 1) + 1)] << 8)));

    M_PRINTF("    <_wData(%d)> %d, 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             bAxis, _wData, _baData[0], _baData[1], _baData[2], _baData[3], _baData[4], _baData[5]);

    _wSlope = ((_wData - _wInitData) / _baInitDiff[bAxis]);

    M_PRINTF("    <_wSlope(%d)> %d", bAxis, _wSlope);

    if (_M_DRV_MC3610_M_ABS(_wData) >= 16383)
    {
        pbAofs[ (bAxis << 1)     ] += 8;
        pbAofs[((bAxis << 1) + 1)] -= 8;
    
        _M_DRV_MC3610_REG_WRITE((0x35 + (bAxis << 1)), &pbAofs[(bAxis << 1)], 2); 
    
        _M_DRV_MC3610_ASC_ReadSensorData(_baData);
    
        _wData = ((signed short) ((_baData[(bAxis << 1)]) | (_baData[((bAxis << 1) + 1)] << 8)));

        M_PRINTF("    <_wData(%d)> %d, 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                 bAxis, _wData, _baData[0], _baData[1], _baData[2], _baData[3], _baData[4], _baData[5]);

        _wSlope = ((_wData - _wInitData) / 8);

        M_PRINTF("    <_wSlope(%d)> %d", bAxis, _wSlope);
    }

    if (M_DRV_MC3610_AXIS_Z == bAxis)
    {
        _wDiff = (_waTarget[bAxis][_ASC_TARGET_MID_] - _wData);

        if (_M_DRV_MC3610_M_ABS(_wDiff) >= _M_DRV_MC3610_M_ABS(_wSlope))
            _wFSft = (_wDiff / _M_DRV_MC3610_M_ABS(_wSlope));
    }
    else
    {
        _wDiff = (_waTarget[bAxis][_ASC_TARGET_MID_] - _M_DRV_MC3610_M_ABS(_wData));

        if (_M_DRV_MC3610_M_ABS(_wDiff) >= _wSlope)
            _wFSft = _M_DRV_MC3610_M_ABS(_wDiff / _wSlope);
    }

    M_PRINTF("    <_wDiff(%d)> %d", bAxis, _wDiff);
    M_PRINTF("    <_wFSft(%d)> %d", bAxis, _wFSft);

    if (_wData < _waTarget[bAxis][_ASC_TARGET_LOW_])
    {
        if (M_DRV_MC3610_AXIS_Z == bAxis)
        {
            pbAofs[ (bAxis << 1)     ] += _M_DRV_MC3610_M_ABS(_wFSft);
            pbAofs[((bAxis << 1) + 1)] -= _M_DRV_MC3610_M_ABS(_wFSft);
        }
        else
        {
            pbAofs[ (bAxis << 1)     ] -= _wFSft;
            pbAofs[((bAxis << 1) + 1)] += _wFSft;
        }
    }
    else if (_wData > _waTarget[bAxis][_ASC_TARGET_HIGH_])
    {
        pbAofs[ (bAxis << 1)     ] += _wFSft;
        pbAofs[((bAxis << 1) + 1)] -= _wFSft;
    }

    _M_DRV_MC3610_REG_WRITE((0x35 + (bAxis << 1)), &pbAofs[(bAxis << 1)], 2); 

    M_PRINTF("    <_baAofs(%d)> 0x%02X 0x%02X", bAxis, pbAofs[(bAxis << 1)], pbAofs[((bAxis << 1) + 1)]);

    #undef _ASC_TARGET_LOW_
    #undef _ASC_TARGET_MID_
    #undef _ASC_TARGET_HIGH_
    #undef _ASC_TARGET_COUNT_
}

/*****************************************
*** _M_DRV_MC3610_ASC
*****************************************/
static int    _M_DRV_MC3610_ASC(void)
{
    unsigned char    _bRegData                           = 0x04;
    unsigned char    _baAofs[M_DRV_MC3610_ASC_DATA_SIZE] = { 0 };

    M_PRINTF("[%s]", __func__);

    _M_DRV_MC3610_ResetChip();

    _M_DRV_MC3610_REG_READ(0x35, s_baASC_W, M_DRV_MC3610_ASC_DATA_SIZE);

    M_PRINTF("    <s_baASC_W> 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             s_baASC_W[0], s_baASC_W[1], s_baASC_W[2], s_baASC_W[3], s_baASC_W[4], s_baASC_W[5]);

    _M_DRV_MC3610_REG_WRITE(0x15, &_bRegData, 1);

    _bRegData = 0x07;
    _M_DRV_MC3610_REG_WRITE(0x11, &_bRegData, 1);

    _bRegData = 0x97;
    _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);

    _bRegData = 0x5E;
    _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);

    _bRegData = 0xF1;
    _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);

    _bRegData = 0x21;
    _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);

    _bRegData = 0x89;
    _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);

    _bRegData = 0xE7;
    _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);

    _bRegData = 0x02;
    _M_DRV_MC3610_REG_WRITE(0x20, &_bRegData, 1);

    _baAofs[0] = s_baASC_W[0];
    _baAofs[1] = s_baASC_W[1];
    _baAofs[2] = s_baASC_W[2];
    _baAofs[3] = s_baASC_W[3];
    _baAofs[4] = s_baASC_W[4];
    _baAofs[5] = s_baASC_W[5];

    _M_DRV_MC3610_ASC_Handler(M_DRV_MC3610_AXIS_X, _baAofs);
    _M_DRV_MC3610_ASC_Handler(M_DRV_MC3610_AXIS_Y, _baAofs);
    _M_DRV_MC3610_ASC_Handler(M_DRV_MC3610_AXIS_Z, _baAofs);

    s_baASC_S[0] = _baAofs[0];
    s_baASC_S[1] = _baAofs[1];
    s_baASC_S[2] = _baAofs[2];
    s_baASC_S[3] = _baAofs[3];
    s_baASC_S[4] = _baAofs[4];
    s_baASC_S[5] = _baAofs[5];

    M_PRINTF("    <s_baASC_S> 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             s_baASC_S[0], s_baASC_S[1], s_baASC_S[2], s_baASC_S[3], s_baASC_S[4], s_baASC_S[5]);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}
#endif

#ifdef M_DRV_MC3610_OPERATE_MODE_WAKE_WHEN_READ
#ifdef M_DRV_MC3610_CFG_WA_8
/*****************************************
*** _M_DRV_MC3610_InitiateSleepMode
*****************************************/
static int    _M_DRV_MC3610_InitiateSleepMode(void)
{
    unsigned char    _bRegData = 0x01;

    _M_DRV_MC3610_REG_WRITE(0x1A, &_bRegData, 1);

    _bRegData = 0x34;

    _M_DRV_MC3610_REG_WRITE(0x1B, &_bRegData, 1);

    _bRegData = 0x00;

    _M_DRV_MC3610_REG_WRITE(0x1A, &_bRegData, 1);

    _bRegData = 0x05;

    _M_DRV_MC3610_REG_WRITE(0x10, &_bRegData, 1);

    _M_DRV_MC3610_Delay(20);

    _bRegData = 0x00;

    _M_DRV_MC3610_REG_WRITE(0x10, &_bRegData, 1);

    _bRegData = 0x0F;

    _M_DRV_MC3610_REG_WRITE(0x23, &_bRegData, 1);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}
#endif
#endif

/*****************************************
 *** _M_DRV_MC3610_SetCwakeSampleRateUltraLowPowerMode
 *****************************************/
static void    _M_DRV_MC3610_SetCwakeSampleRateUltraLowPowerMode(E_M_DRV_MC3610_CWAKE_SR eSR)
{
    unsigned char    _bRegWAKE = eSR;

    _M_DRV_MC3610_SetSampleRateMode(M_DRV_MC3610_SR_MODE_ULTRA_LOW_POWER);

    if (E_M_DRV_MC3610_CWAKE_SR_UL_46Hz == eSR)
    {
        unsigned char    _bRegSNIFF = 0x08;

        _bRegWAKE = 0x82;

        _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_SNIFF_C, &_bRegSNIFF, 1);
    }

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_WAKE_C , &_bRegWAKE , 1);
}

/*****************************************
 *** _M_DRV_MC3610_SetCwakeSampleRateLowPowerMode
 *****************************************/
static void    _M_DRV_MC3610_SetCwakeSampleRateLowPowerMode(E_M_DRV_MC3610_CWAKE_SR eSR)
{
    unsigned char    _bRegWAKE = (eSR - E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_BASE);

    _M_DRV_MC3610_SetSampleRateMode(M_DRV_MC3610_SR_MODE_LOW_POWER);

    if ((E_M_DRV_MC3610_CWAKE_SR_LP_25Hz == eSR) || (E_M_DRV_MC3610_CWAKE_SR_LP_50Hz == eSR))
    {
        unsigned char    _bRegSNIFF[2] = { 0x0C, 0x08 };

        _bRegWAKE = 0x82;

        _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_SNIFF_C, &_bRegSNIFF[(eSR - E_M_DRV_MC3610_CWAKE_SR_LP_25Hz)], 1);
    }

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_WAKE_C , &_bRegWAKE , 1);
}

/*****************************************
 *** _M_DRV_MC3610_SetCwakeSampleRatePrecisionMode
 *****************************************/
static void    _M_DRV_MC3610_SetCwakeSampleRatePrecisionMode(E_M_DRV_MC3610_CWAKE_SR eSR)
{
    unsigned char    _bRegWAKE = (eSR - E_M_DRV_MC3610_CWAKE_SR_PR_DUMMY_BASE);

    _M_DRV_MC3610_SetSampleRateMode(M_DRV_MC3610_SR_MODE_PRECISION);

    if ((E_M_DRV_MC3610_CWAKE_SR_PR_3Hz == eSR) || (E_M_DRV_MC3610_CWAKE_SR_PR_14Hz == eSR))
    {
        unsigned char    _bRegSNIFF[2] = { 0x0C, 0x08 };

        _bRegWAKE = 0x82;

        _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_SNIFF_C, &_bRegSNIFF[(eSR - E_M_DRV_MC3610_CWAKE_SR_PR_3Hz)], 1);
    }

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_WAKE_C , &_bRegWAKE , 1);
}

/*****************************************
 *** _M_DRV_MC3610_SetCwakeSampleRate
 *****************************************/
static int    _M_DRV_MC3610_SetCwakeSampleRate(E_M_DRV_MC3610_CWAKE_SR eSR)
{
    if ((E_M_DRV_MC3610_CWAKE_SR_UL_DUMMY_BASE < eSR) && (eSR <E_M_DRV_MC3610_CWAKE_SR_UL_DUMMY_END))
    {
        _M_DRV_MC3610_SetCwakeSampleRateUltraLowPowerMode(eSR);
    }
    else if ((E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_BASE < eSR) && (eSR <E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_END))
    {
        _M_DRV_MC3610_SetCwakeSampleRateLowPowerMode(eSR);
    }
    else if ((E_M_DRV_MC3610_CWAKE_SR_PR_DUMMY_BASE < eSR) && (eSR <E_M_DRV_MC3610_CWAKE_SR_PR_DUMMY_END))
    {
        _M_DRV_MC3610_SetCwakeSampleRatePrecisionMode(eSR);
    }
    else
    {
        return (M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT);
    }

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3610_CheckCwakeSampleRate
 *****************************************/
static int    _M_DRV_MC3610_CheckCwakeSampleRate(E_M_DRV_MC3610_CWAKE_SR eSR)
{
    if (   (!((E_M_DRV_MC3610_CWAKE_SR_UL_DUMMY_BASE < eSR) && (eSR <E_M_DRV_MC3610_CWAKE_SR_UL_DUMMY_END)))
        && (!((E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_BASE < eSR) && (eSR <E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_END)))
        && (!((E_M_DRV_MC3610_CWAKE_SR_PR_DUMMY_BASE < eSR) && (eSR <E_M_DRV_MC3610_CWAKE_SR_PR_DUMMY_END))))
    {
        return (M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT);
    }

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3610_SetSniffSampleRate
 *****************************************/
static int    _M_DRV_MC3610_SetSniffSampleRate(E_M_DRV_MC3610_SNIFF_SR eSR)
{
    unsigned char    _bRegWAKE  = 0x00;
    unsigned char    _bRegSNIFF = 0xC0;

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_WAKE_C , &_bRegWAKE , 1);

    _M_DRV_MC3610_SetSampleRateMode(M_DRV_MC3610_SR_MODE_LOW_POWER);

    _bRegSNIFF |= eSR;
    _bRegWAKE  &= 0x7F;

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_SNIFF_C, &_bRegSNIFF, 1);
    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_WAKE_C , &_bRegWAKE , 1);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*********************************************************************
 *** _M_DRV_MC3610_CfgSniff
 *********************************************************************/
static void    _M_DRV_MC3610_CfgSniff(void)
{
    unsigned char    _bRegData = 0x97;

    _M_DRV_MC3610_ResetChip();

    _M_DRV_MC3610_SetBusIF();

    _M_DRV_MC3610_SetSniffSampleRate(s_eSR_SNIFF);

    _M_DRV_MC3610_REG_WRITE(0x13, &s_bCfgSniffThr, 1);

    _M_DRV_MC3610_REG_WRITE(0x15, &s_bCfgRngResol, 1);

    _M_DRV_MC3610_REG_WRITE(0x16, &s_bCfgFifo    , 1);

    _M_DRV_MC3610_REG_WRITE(0x17, &s_bCfgINT     , 1);

    #ifdef M_DRV_MC3610_CFG_ASC
        _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);
    
        _bRegData = 0x5E;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);
    
        _bRegData = 0xF1;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bRegData, 1);

        _M_DRV_MC3610_REG_WRITE(0x35, s_baASC_S, M_DRV_MC3610_ASC_DATA_SIZE);
    #endif
}

/*********************************************************************
 *** _M_DRV_MC3610_CfgWake
 *********************************************************************/
static void    _M_DRV_MC3610_CfgWake(void)
{
    #ifdef M_DRV_MC3610_CFG_ASC
        unsigned char    _bData = 0x01;
    
        _M_DRV_MC3610_REG_WRITE(0x10, &_bData, 1);
    
        _M_DRV_MC3610_Delay(10);

        _bData = 0x97;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bData, 1);
    
        _bData = 0x5E;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bData, 1);
    
        _bData = 0xF1;
        _M_DRV_MC3610_REG_WRITE(0x25, &_bData, 1);

        _M_DRV_MC3610_REG_WRITE(0x35, s_baASC_W, M_DRV_MC3610_ASC_DATA_SIZE);
    #endif
}

/*********************************************************************
 *** _M_DRV_MC3610_SetMode
 *********************************************************************/
static int    _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE eNextMode)
{
    unsigned char    _bCurrMode  = 0;
    unsigned char    _bRegMODE_C = 0;
    unsigned char    _bGuard     = 0;

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_STATUS_1, &_bCurrMode , 1);

    if (eNextMode == _M_DRV_MC3610_REG_STATUS_1_MODE(_bCurrMode))
        return (M_DRV_MC3610_RETCODE_ERROR_STATUS);

    #ifdef M_DRV_MC3610_CFG_WA_8
        if ((E_M_DRV_MC3610_MODE_SLEEP == _M_DRV_MC3610_REG_STATUS_1_MODE(_bCurrMode)) && (E_M_DRV_MC3610_MODE_SLEEP != eNextMode))
        {
            unsigned char    _bRegData = 0x00;

            _M_DRV_MC3610_REG_WRITE(0x23, &_bRegData, 1);
        }
    #endif

    if (E_M_DRV_MC3610_MODE_SNIFF == eNextMode)
    {
        _M_DRV_MC3610_CfgSniff();
    }
    else
    {
        _M_DRV_MC3610_CfgWake();

        if (E_M_DRV_MC3610_MODE_CWAKE == eNextMode)
            _M_DRV_MC3610_SetCwakeSampleRate(s_eSR_CWAKE);
    }

    _bRegMODE_C |= eNextMode;

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_MODE_C, &_bRegMODE_C, 1);

    #ifdef M_DRV_MC3610_CFG_WA_8
        if (E_M_DRV_MC3610_MODE_SLEEP == eNextMode)
        {
            unsigned char    _bRegData = 0x0F;

            _M_DRV_MC3610_REG_WRITE(0x23, &_bRegData, 1);
        }
    #endif

    while (1)
    {
        _bGuard++;

        _M_DRV_MC3610_Delay(1);

        _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_STATUS_1, &_bCurrMode , 1);
    
        if (eNextMode == _M_DRV_MC3610_REG_STATUS_1_MODE(_bCurrMode))
            break;

        #ifdef M_DRV_MC3610_CFG_WA_9
            if ((E_M_DRV_MC3610_MODE_SLEEP == eNextMode) && (E_M_DRV_MC3610_MODE_STANDBY == _M_DRV_MC3610_REG_STATUS_1_MODE(_bCurrMode)))
                break;
        #endif

        if (_bGuard > 64)
            return (M_DRV_MC3610_RETCODE_ERROR_SETUP);
    }

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3610_SetSniffThreshold
 *****************************************/
static int    _M_DRV_MC3610_SetSniffThreshold(void)
{
    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);

    s_bCfgSniffThr |= 0x07;

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*********************************************************************
 *** _M_DRV_MC3610_ReadData
 *********************************************************************/
static int    _M_DRV_MC3610_ReadData(float faOutput[M_DRV_MC3610_AXES_NUM])
{
    short            _waRaw[M_DRV_MC3610_AXES_NUM] = { 0 };
    unsigned char    _baData[6]                    = { 0 };

    const S_M_DRV_MC_UTIL_OrientationReMap   *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC3610_CFG_ORIENTATION_MAP];

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_XOUT_LSB, _baData, 6);

    _waRaw[M_DRV_MC3610_AXIS_X] = ((signed short) ((_baData[0]) | (_baData[1] << 8)));
    _waRaw[M_DRV_MC3610_AXIS_Y] = ((signed short) ((_baData[2]) | (_baData[3] << 8)));
    _waRaw[M_DRV_MC3610_AXIS_Z] = ((signed short) ((_baData[4]) | (_baData[5] << 8)));

    faOutput[M_DRV_MC3610_AXIS_X] = ((float)(_ptOrienMap->bSign[M_DRV_MC3610_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3610_AXIS_X]]));
    faOutput[M_DRV_MC3610_AXIS_Y] = ((float)(_ptOrienMap->bSign[M_DRV_MC3610_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3610_AXIS_Y]]));
    faOutput[M_DRV_MC3610_AXIS_Z] = ((float)(_ptOrienMap->bSign[M_DRV_MC3610_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3610_AXIS_Z]]));

    faOutput[M_DRV_MC3610_AXIS_X] = -faOutput[M_DRV_MC3610_AXIS_X];
    faOutput[M_DRV_MC3610_AXIS_Y] = -faOutput[M_DRV_MC3610_AXIS_Y];

    faOutput[M_DRV_MC3610_AXIS_X] = (faOutput[M_DRV_MC3610_AXIS_X] * s_fMC3XXX_Sensitivity);
    faOutput[M_DRV_MC3610_AXIS_Y] = (faOutput[M_DRV_MC3610_AXIS_Y] * s_fMC3XXX_Sensitivity);
    faOutput[M_DRV_MC3610_AXIS_Z] = (faOutput[M_DRV_MC3610_AXIS_Z] * s_fMC3XXX_Sensitivity);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*******************************************************************************
 *** FUNCTION
 *******************************************************************************/

/*********************************************************************
 *** M_DRV_MC3610_SetMode
 *********************************************************************/
int    M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE eNextMode)
{
    _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_STANDBY);

    #ifdef M_DRV_MC3610_CFG_WA_12
        if (E_M_DRV_MC3610_MODE_REARM == eNextMode)
            eNextMode = E_M_DRV_MC3610_MODE_SNIFF;
    #endif

    _M_DRV_MC3610_SetMode(eNextMode);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*****************************************
 *** M_DRV_MC3610_ConfigRegRngResCtrl
 *****************************************/
int    M_DRV_MC3610_ConfigRegRngResCtrl(E_M_DRV_MC3610_RANGE eCfgRange, E_M_DRV_MC3610_RESOLUTION eCfgResolution)
{
    unsigned char    _bPreMode                                    = 0;
    float            _faRange[E_M_DRV_MC3610_RANGE_END]           = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f, 235.368f };
    float            _faResolution[E_M_DRV_MC3610_RESOLUTION_END] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f };

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_MODE_C, &_bPreMode, 1);

    _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_STANDBY);

    #ifdef M_DRV_MC3610_CFG_WA_10
        if (E_M_DRV_MC3610_RESOLUTION_12BIT < eCfgResolution)
        {
            s_bCfgFifo = 0x80;
    
            _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_FIFO_C, &s_bCfgFifo, 1);            

            s_bCfgFifo = 0x00;

            _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_FIFO_C, &s_bCfgFifo, 1);            
        }
    #endif

    s_bCfgRngResol = (((eCfgRange << 4) & 0x70) | eCfgResolution);

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_RANGE_C, &s_bCfgRngResol, 1);

    s_fMC3XXX_Sensitivity = (_faRange[eCfgRange] / _faResolution[eCfgResolution]);

    _M_DRV_MC3610_SetMode(_M_DRV_MC3610_REG_MODE_C_MODE(_bPreMode));

    s_eRange      = eCfgRange;
    s_eResolution = eCfgResolution;

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*****************************************
 *** M_DRV_MC3610_SetSampleRate
 *****************************************/
int    M_DRV_MC3610_SetSampleRate(E_M_DRV_MC3610_CWAKE_SR eCwakeSR, E_M_DRV_MC3610_SNIFF_SR eSniffSR)
{
    unsigned char    _bPreMode  = 0;

    if (M_DRV_MC3610_RETCODE_SUCCESS != _M_DRV_MC3610_CheckCwakeSampleRate(eCwakeSR))
        return (M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT);

    s_eSR_CWAKE = eCwakeSR;
    s_eSR_SNIFF = eSniffSR;

    #ifdef M_DRV_MC3610_CFG_WA_7
        s_eSR_SNIFF = E_M_DRV_MC3610_SNIFF_SR_6Hz;
    #endif

    // Sample Rate takes effect when switching mode (driver implementation)
    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_MODE_C, &_bPreMode, 1);

    _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_STANDBY);

    _M_DRV_MC3610_SetMode(_M_DRV_MC3610_REG_MODE_C_MODE(_bPreMode));

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3610_Init
 *********************************************************************/
int    M_DRV_MC3610_Init(void)
{
    M_PRINTF("[%s]", __func__);

    if (M_DRV_MC3610_RETCODE_SUCCESS != _M_DRV_MC3610_ValidateSensorIC())
        return (M_DRV_MC3610_RETCODE_ERROR_IDENTIFICATION);

    #ifdef M_DRV_MC3610_CFG_ASC
        _M_DRV_MC3610_ASC();
    #endif

    _M_DRV_MC3610_ResetChip();

    _M_DRV_MC3610_SetBusIF();

    M_DRV_MC3610_ConfigRegRngResCtrl(s_eRange, s_eResolution);

    _M_DRV_MC3610_SetSniffThreshold();

    _M_DRV_MC3610_SetSniffSampleRate(s_eSR_SNIFF);

    _M_DRV_MC3610_SetCwakeSampleRate(s_eSR_CWAKE);

    #ifdef M_DRV_MC3610_OPERATE_MODE_WAKE_WHEN_READ
        #ifdef M_DRV_MC3610_CFG_WA_8
            _M_DRV_MC3610_SetCwakeSampleRate(E_M_DRV_MC3610_CWAKE_SR_DEFAULT_50Hz);
            _M_DRV_MC3610_InitiateSleepMode();
            _M_DRV_MC3610_SetCwakeSampleRate(s_eSR_CWAKE);
        #else
            _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_SLEEP);
        #endif
    #endif

    #ifdef M_DRV_MC3610_OPERATE_MODE_ALWAYS_WAKE
        _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_CWAKE);
    #endif

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_FIFO_C, &s_bCfgFifo, 1);
    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_INTR_C, &s_bCfgINT , 1);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3610_EnableFIFO
 *********************************************************************/
int    M_DRV_MC3610_EnableFIFO(E_M_DRV_MC3610_FIFO_CONTROL eCtrl, E_M_DRV_MC3610_FIFO_MODE eMode, unsigned char bThreshold)
{
    unsigned char    _bPreMode = 0;

    if (eCtrl >= E_M_DRV_MC3610_FIFO_CONTROL_END)
        return (M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT);

    if (eMode >= E_M_DRV_MC3610_FIFO_MODE_END)
        return (M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT);

    if (bThreshold > 31)
        return (M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT);

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_MODE_C, &_bPreMode, 1);

    _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_STANDBY);

    #ifdef M_DRV_MC3610_CFG_WA_10
    {
        unsigned char    _bRegRANGE_C = 0;

        _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_RANGE_C, &_bRegRANGE_C, 1);

        if (E_M_DRV_MC3610_RESOLUTION_12BIT < _M_DRV_MC3610_REG_RANGE_C_RES(_bRegRANGE_C))
            M_DRV_MC3610_ConfigRegRngResCtrl(_M_DRV_MC3610_REG_RANGE_C_RANGE(_bRegRANGE_C), E_M_DRV_MC3610_RESOLUTION_12BIT);
    }
    #endif

    s_bCfgFifo = ((eCtrl << 6) | (eMode << 5) | bThreshold);

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_FIFO_C, &s_bCfgFifo, 1);

    _M_DRV_MC3610_SetMode(_M_DRV_MC3610_REG_MODE_C_MODE(_bPreMode));

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3610_ConfigINT
 *********************************************************************/
int    M_DRV_MC3610_ConfigINT(unsigned char bFifoThreshEnable,
                              unsigned char bFifoFullEnable  ,
                              unsigned char bFifoEmptyEnable ,
                              unsigned char bACQEnable       ,
                              unsigned char bWakeEnable       )
{
    unsigned char    _bPreMode = 0;

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_MODE_C, &_bPreMode, 1);

    _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_STANDBY);

    s_bCfgINT = (  ((bFifoThreshEnable & 0x01) << 6)
                 | ((bFifoFullEnable   & 0x01) << 5)
                 | ((bFifoEmptyEnable  & 0x01) << 4)
                 | ((bACQEnable        & 0x01) << 3)
                 | ((bWakeEnable       & 0x01) << 2)
                 | M_DRV_MC3610_INTR_C_IAH_ACTIVE_LOW
                 | M_DRV_MC3610_INTR_C_IPP_MODE_PUSH_PULL);

    _M_DRV_MC3610_REG_WRITE(M_DRV_MC3610_REG_INTR_C, &s_bCfgINT, 1);

    _M_DRV_MC3610_SetMode(_M_DRV_MC3610_REG_MODE_C_MODE(_bPreMode));

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3610_ReadData
 *********************************************************************/
int    M_DRV_MC3610_ReadData(float faOutput[M_DRV_MC3610_FIFO_DEPTH][M_DRV_MC3610_AXES_NUM], int nNumOfSample)
{
    int              _nDataCount  = 0;
    unsigned char    _bRegStatus1 = 0;
    unsigned char    _bRegFIFO_C  = 0;

    if ((M_DRV_MC3610_NULL_ADDR == faOutput) || (0 == nNumOfSample))
        return (M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT);

    #ifdef M_DRV_MC3610_OPERATE_MODE_WAKE_WHEN_READ
        _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_STANDBY);
        _M_DRV_MC3610_SetCwakeSampleRate(s_eSR_CWAKE);
        _M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE_CWAKE);
    #endif

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_STATUS_1, &_bRegStatus1, 1);
    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_FIFO_C  , &_bRegFIFO_C , 1);

    if (_M_DRV_MC3610_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
    {
        if (M_DRV_MC3610_FIFO_DEPTH < nNumOfSample)
            nNumOfSample = M_DRV_MC3610_FIFO_DEPTH;

        if (_M_DRV_MC3610_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            return (M_DRV_MC3610_RETCODE_ERROR_NO_DATA);

        for (_nDataCount = 0; _nDataCount < nNumOfSample; _nDataCount++)
        {
            _M_DRV_MC3610_ReadData(faOutput[_nDataCount]);

            _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_STATUS_1, &_bRegStatus1, 1);

            if (_M_DRV_MC3610_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            {
                _nDataCount++;

                break;
            }
        }
    }
    else
    {
        if (!_M_DRV_MC3610_REG_STATUS_1_NEW_DATA(_bRegStatus1))
            return (M_DRV_MC3610_RETCODE_ERROR_NO_DATA);

        _M_DRV_MC3610_ReadData(faOutput[0]);

        _nDataCount = 1;
    }

    return (_nDataCount);
}

/*********************************************************************
 *** M_DRV_MC3610_HandleINT
 *********************************************************************/
int    M_DRV_MC3610_HandleINT(S_M_DRV_MC3610_InterruptEvent *ptINT_Event)
{
    unsigned char    _bRegStatus2 = 0;

    _M_DRV_MC3610_REG_READ(M_DRV_MC3610_REG_STATUS_2, &_bRegStatus2, 1);

    ptINT_Event->bWAKE           = _M_DRV_MC3610_REG_STATUS_2_INT_WAKE(_bRegStatus2);
    ptINT_Event->bACQ            = _M_DRV_MC3610_REG_STATUS_2_INT_ACQ(_bRegStatus2);
    ptINT_Event->bFIFO_EMPTY     = _M_DRV_MC3610_REG_STATUS_2_INT_FIFO_EMPTY(_bRegStatus2);
    ptINT_Event->bFIFO_FULL      = _M_DRV_MC3610_REG_STATUS_2_INT_FIFO_FULL(_bRegStatus2);
    ptINT_Event->bFIFO_THRESHOLD = _M_DRV_MC3610_REG_STATUS_2_INT_FIFO_THRESH(_bRegStatus2);

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

// for debug
/*********************************************************************
 *** M_DRV_MC3610_ReadRegMap
 *********************************************************************/
int    M_DRV_MC3610_ReadRegMap(unsigned char baRegMap[M_DRV_MC3610_REG_MAP_SIZE])
{
    uint8_t    _bIndex = 0;
    uint8_t    _bRegData = 0;

    for (_bIndex = 0; _bIndex < M_DRV_MC3610_REG_MAP_SIZE; _bIndex++)
    {
        _M_DRV_MC3610_REG_READ(_bIndex, &_bRegData, 1);

        M_PRINTF("REG[0x%02X] 0x%02X", _bIndex, _bRegData);

        if (0 != baRegMap)
            baRegMap[_bIndex] = _bRegData;
    }

    return (M_DRV_MC3610_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3610_ReadReg
 *********************************************************************/
unsigned char    M_DRV_MC3610_ReadReg(unsigned char bRegAddr)
{
    uint8_t    _bData = 0;

    _M_DRV_MC3610_REG_READ(bRegAddr, &_bData, 1);

    return (_bData);
}

