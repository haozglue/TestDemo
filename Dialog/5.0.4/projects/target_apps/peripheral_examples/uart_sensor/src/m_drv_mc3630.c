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
#if CFG_MODULE_ACCEL_DRIVER == 3630

/*******************************************************************************
 *** INFORMATION
 *******************************************************************************/
#define M_DRV_MC3630_VERSION    "2.0.0"

/*******************************************************************************
 *** CONFIGURATION
 *******************************************************************************/
#define M_DRV_MC3630_CFG_BUS_I2C    // !!! DO NOT use both I2C and SPI at the same time
//#define M_DRV_MC3630_CFG_BUS_SPI

#if (!defined (M_DRV_MC3630_CFG_BUS_SPI) && !defined (M_DRV_MC3630_CFG_BUS_I2C))
#   error "MUST use one bus to access register!"
#endif

#if (defined (M_DRV_MC3630_CFG_BUS_SPI) && defined (M_DRV_MC3630_CFG_BUS_I2C))
#   error "DO NOT use both SPI and I2C simutaneously!"
#endif

#include "mc3xxx_port.h"
#include "m_drv_mc3630.h"
#include "m_drv_mc_utility.h"



/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/
typedef unsigned char uint8_t;

#include <stdint.h>

//#ifdef M_DRV_MC3630_CFG_BUS_I2C
//#include "m_drv_i2c.h"    // hook by custom
//#else
//#include "m_drv_spi.h"    // hook by custom
//#endif

// Remove from formal release!!!
//#include "m_drv_uart.h"
//#include "m_drv_timer.h"

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/

/*******************************************************************************
 *** MACRO
 *******************************************************************************/
#ifdef M_DRV_MC3630_CFG_BUS_I2C
//#define _M_DRV_MC3630_REG_WRITE(bRegAddr, pbDataBuf, bLength)    M_DRV_I2C_Write(M_DRV_MC3630_I2C_WRITE_ADDR, bRegAddr, pbDataBuf, bLength)
//#define _M_DRV_MC3630_REG_READ(bRegAddr, pbDataBuf, bLength)     M_DRV_I2C_Read(M_DRV_MC3630_I2C_READ_ADDR, bRegAddr, pbDataBuf, bLength)
#define _M_DRV_MC3630_REG_WRITE(bRegAddr, pbDataBuf, bLength)    \
        {                                                        \
            I2C_Init();                                          \
            I2C_WriteReg(bRegAddr, *pbDataBuf);                  \
            I2C_Release();                                       \
        }

#define _M_DRV_MC3630_REG_READ(bRegAddr, pbDataBuf, bLength)     \
        {                                                        \
            I2C_Init();                                          \
            I2C_ReadData(pbDataBuf, bRegAddr, bLength);          \
            I2C_Release();                                       \
        }
#else
#define _M_DRV_MC3630_REG_WRITE(bRegAddr, pbDataBuf, bLength)    M_DRV_SPI_Write(bRegAddr, pbDataBuf, bLength)
#define _M_DRV_MC3630_REG_READ(bRegAddr, pbDataBuf, bLength)     M_DRV_SPI_Read((bRegAddr | 0x80), pbDataBuf, bLength)
#endif

#define _M_DRV_MC3630_REG_STATUS_1_MODE(bRegStatus1)            (bRegStatus1 & 0x07)
#define _M_DRV_MC3630_REG_STATUS_1_NEW_DATA(bRegStatus1)        (bRegStatus1 & 0x08)
#define _M_DRV_MC3630_REG_STATUS_1_FIFO_EMPTY(bRegStatus1)      (bRegStatus1 & 0x10)
#define _M_DRV_MC3630_REG_STATUS_1_FIFO_FULL(bRegStatus1)       (bRegStatus1 & 0x20)
#define _M_DRV_MC3630_REG_STATUS_1_FIFO_THRESH(bRegStatus1)     (bRegStatus1 & 0x40)
#define _M_DRV_MC3630_REG_STATUS_1_INT_PEND(bRegStatus1)        (bRegStatus1 & 0x80)

#define _M_DRV_MC3630_REG_STATUS_2_INT_WAKE(bRegStatus2)        ((bRegStatus2 >> 2) & 0x01)
#define _M_DRV_MC3630_REG_STATUS_2_INT_ACQ(bRegStatus2)         ((bRegStatus2 >> 3) & 0x01)
#define _M_DRV_MC3630_REG_STATUS_2_INT_FIFO_EMPTY(bRegStatus2)  ((bRegStatus2 >> 4) & 0x01)
#define _M_DRV_MC3630_REG_STATUS_2_INT_FIFO_FULL(bRegStatus2)   ((bRegStatus2 >> 5) & 0x01)
#define _M_DRV_MC3630_REG_STATUS_2_INT_FIFO_THRESH(bRegStatus2) ((bRegStatus2 >> 6) & 0x01)
#define _M_DRV_MC3630_REG_STATUS_2_INT_SWAKE_SNIFF(bRegStatus2) ((bRegStatus2 >> 7) & 0x01)

#define _M_DRV_MC3630_REG_MODE_C_MODE(bRegMODE_C)               (bRegMODE_C & 0x07)
#define _M_DRV_MC3630_REG_RANGE_C_RES(bRegRANGE_C)              (bRegRANGE_C & 0x07)
#define _M_DRV_MC3630_REG_RANGE_C_RANGE(bRegRANGE_C)            ((bRegRANGE_C >> 4) & 0x07)

#define _M_DRV_MC3630_REG_FIFO_C_FIFO_EN(bRegFIFO_C)            (bRegFIFO_C & 0x40)
#define _M_DRV_MC3630_FIFO_VDD_EN(bRegPwrCtrl)                  (bRegPwrCtrl | 0x42);

#define _M_DRV_MC3630_M_ABS(x)                                  (((x) < 0) ? (-(x)) : (x))

#ifdef M_DRV_MC3630_SUPPORT_LPF
#define _M_DRV_MC3630_SENSOR_FILTER(last_data, curr_data)       (( ALPHA*(last_data)) + ((1-ALPHA)*(curr_data)))
#endif

#define M_PRINTF                                                Debug_Printf

/*******************************************************************************
 *** STATIC VARIABLE
 *******************************************************************************/
static float s_fMC3XXX_Sensitivity = 0;    // unit: SI/LSB, SI: m/s^2

static E_M_DRV_MC3630_RANGE s_eRange = M_DRV_MC3630_CFG_RANGE;
static E_M_DRV_MC3630_RESOLUTION s_eResolution = M_DRV_MC3630_CFG_RESOLUTION;
static E_M_DRV_MC3630_CWAKE_SR s_eSR_CWAKE = M_DRV_MC3630_CFG_SAMPLE_RATE_CWAKE_DEFAULT;
static E_M_DRV_MC3630_SNIFF_SR s_eSR_SNIFF = M_DRV_MC3630_CFG_SAMPLE_RATE_SNIFF_DEFAULT;
static E_M_DRV_MC3630_WAKE_GAIN s_eGAIN_WAKE = M_DRV_MC3630_CFG_WAKE_GAIN_DEFAULT;
static E_M_DRV_MC3630_SNIFF_GAIN s_eGAIN_SNIFF =  M_DRV_MC3630_CFG_SNIFF_GAIN_DEFAULT;

static unsigned char s_bCfgRngResol = 0x00;
static unsigned char s_bCfgSniffThr = 0x00;
//static unsigned char s_bCfgSniffCfg = 0x00;
static unsigned char s_bCfgFifo = 0x00;
static unsigned char s_bCfgINT = 0x00;
static unsigned char s_bCfgFifoVdd = 0x42;
static unsigned char s_bCfgWakeSRMode = M_DRV_MC3630_SR_MODE_UNKNOWN;
static unsigned char s_bCfgSniffSRMode = M_DRV_MC3630_SR_MODE_UNKNOWN;

static unsigned char s_debug=0x00;

#ifdef M_DRV_MC3630_SUPPORT_LPF
static short _saLPFPrevData[M_DRV_MC3630_AXES_NUM]={0};
#endif

/*******************************************************************************
 *** STATIC FUNCTION
 *******************************************************************************/
/*********************************************************************
 *** _M_DRV_MC3630_Delay
 *********************************************************************/
int _M_DRV_MC3630_Delay(uint32_t dwMs)
{
    // hook by custom

    System_Deley(dwMs);
    
    return dwMs;
}

/*********************************************************************
 *** _M_DRV_MC3630_SetBusIF
 *********************************************************************/
static int _M_DRV_MC3630_SetBusIF(void)
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    /*Orion I2C/SPI interface Setup */
    unsigned char _bRegIO_C = 0;
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_FEATURE_C_1, &_bRegIO_C, 1); //0x0D

#ifdef M_DRV_MC3630_CFG_BUS_I2C
    _bRegIO_C &= 0x3F;
    _bRegIO_C |= 0x40;
#else    //#ifdef M_DRV_MC3630_CFG_BUS_SPI
    _bRegIO_C &= 0x3F;
    _bRegIO_C |= 0x80;
#endif

    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_FEATURE_C_1, &_bRegIO_C, 1);

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3630_SetSniffAGAIN
 *****************************************/
static int _M_DRV_MC3630_SetSniffAGAIN(uint8_t SniffGain)
{
    if (s_debug) M_PRINTF("[%s] SniffGain= 0x%02X", __func__, SniffGain);
    unsigned char _bRegAGain = 0x00;

    if (SniffGain > E_M_DRV_MC3630_SNIFF_GAIN_END) {
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);
    }
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_DMX, &_bRegAGain, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_DMX, _bRegAGain);

    _bRegAGain = 0x00;
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_DMY, &_bRegAGain, 1);
    _bRegAGain &= 0x3F;
    _bRegAGain |= (SniffGain << 6 );

    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_DMY, &_bRegAGain, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_DMY, _bRegAGain);

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3630_SetWakeAGAIN
 *****************************************/
static int _M_DRV_MC3630_SetWakeAGAIN(uint8_t WakeGain)
{
    if (s_debug) M_PRINTF("[%s] WakeGain= 0x%02X", __func__, WakeGain);
    unsigned char _bRegAGain = 0x01;

    if (WakeGain > E_M_DRV_MC3630_WAKE_GAIN_END) {
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);
    }

    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_DMX, &_bRegAGain, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_DMX, _bRegAGain);

    _bRegAGain = 0x00;
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_DMY, &_bRegAGain, 1);

    _bRegAGain &= 0x3F;
    _bRegAGain |= (WakeGain << 6);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_DMY, &_bRegAGain, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_DMY, _bRegAGain);

    _bRegAGain = 0x00;
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_DMX, &_bRegAGain, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_DMX, _bRegAGain);

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3630_ResetChip
 *****************************************/
static void _M_DRV_MC3630_ResetChip(void)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegData = 0x01;

    _M_DRV_MC3630_REG_WRITE(0x10, &_bRegData, 1);

    _M_DRV_MC3630_Delay(10);

    _bRegData = 0x40;
    _M_DRV_MC3630_REG_WRITE(0x24, &_bRegData, 1);

    _M_DRV_MC3630_Delay(50);

    _M_DRV_MC3630_SetBusIF();

    _M_DRV_MC3630_SetWakeAGAIN(E_M_DRV_MC3630_WAKE_GAIN_LOW);
    _M_DRV_MC3630_SetSniffAGAIN(E_M_DRV_MC3630_SNIFF_GAIN_HIGH);

    _bRegData = 0x01;
    _M_DRV_MC3630_REG_WRITE(0x10, &_bRegData, 1);

    _M_DRV_MC3630_Delay(10);
}

/*****************************************
 *** _M_DRV_MC3630_ValidateSensorIC
 *****************************************/
static int _M_DRV_MC3630_ValidateSensorIC(void)
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    unsigned char _bRegData = 0;

    _M_DRV_MC3630_REG_READ(0x18, &_bRegData, 1);

    if (0x71 == _bRegData) {
        _M_DRV_MC3630_REG_READ(0x19, &_bRegData, 1);

        if ((0x00 == _bRegData) || (0x01 == _bRegData)) {
            return(M_DRV_MC3630_RETCODE_SUCCESS);
        }
    }

    return(M_DRV_MC3630_RETCODE_ERROR_IDENTIFICATION);
}

/*****************************************
 *** _M_DRV_MC3630_SetOTPSampleRate
 *****************************************/
static void    _M_DRV_MC3630_SetOTPSampleRate(uint8_t mode, uint8_t bDesiredSRMode)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char    _bRegData = 0x00;

    _M_DRV_MC3630_REG_READ(0x01, &_bRegData, 1);

    if (0x00 == (_bRegData & 0x10)) {
        _bRegData = 0x97;
        _M_DRV_MC3630_REG_WRITE(0x25, &_bRegData, 1);

        _bRegData = 0x5E;
        _M_DRV_MC3630_REG_WRITE(0x25, &_bRegData, 1);

        _bRegData = 0xF1;
        _M_DRV_MC3630_REG_WRITE(0x25, &_bRegData, 1);
    }

    _bRegData = 0x00;
    _M_DRV_MC3630_REG_READ(0x3B, &_bRegData, 1);

    if (mode == E_M_DRV_MC3630_MODE_SNIFF) {
        _bRegData &= 0xF3;
        _bRegData |= bDesiredSRMode;
    } else if (mode >= E_M_DRV_MC3630_MODE_PWAKE && mode < E_M_DRV_MC3630_MODE_END) {
        _bRegData &= 0xFC;
        _bRegData |= bDesiredSRMode;
    } else
        M_PRINTF("[%s] Wrong mode", __func__);

    _M_DRV_MC3630_REG_WRITE(0x3B, &_bRegData, 1);

}

/*****************************************
 *** _M_DRV_MC3630_SetSniffOverSampleRate
 *****************************************/
static void _M_DRV_MC3630_SetSniffOverSampleRate(uint8_t bDesiredSRMode)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegData = 0x00;

    if (s_bCfgSniffSRMode == bDesiredSRMode)
        return;

    _M_DRV_MC3630_REG_READ(0x1C, &_bRegData, 1);
    _bRegData &= 0x8F;
    _bRegData |= (bDesiredSRMode<< 4);

    _M_DRV_MC3630_REG_WRITE(0x1C, &_bRegData, 1);
    if (s_debug) M_PRINTF("[%s] REG[0x1C] 0x%02X", __func__, _bRegData );
    s_bCfgSniffSRMode = bDesiredSRMode;

    _M_DRV_MC3630_SetOTPSampleRate(E_M_DRV_MC3630_MODE_SNIFF,s_bCfgSniffSRMode);
}

/*****************************************
 *** _M_DRV_MC3630_SetWakeOverSampleRate
 *****************************************/
static void _M_DRV_MC3630_SetWakeOverSampleRate(uint8_t bDesiredSRMode)
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    unsigned char _bRegData = 0x00;
    if (s_bCfgWakeSRMode == bDesiredSRMode)
        return;

    _M_DRV_MC3630_REG_READ(0x1C, &_bRegData, 1);
    _bRegData &= 0xF8;
    _bRegData |= bDesiredSRMode;

    _M_DRV_MC3630_REG_WRITE(0x1C, &_bRegData, 1);
    if (s_debug) M_PRINTF("[%s] REG[0x1C] 0x%02X", __func__, _bRegData );
    s_bCfgWakeSRMode = bDesiredSRMode;

    _M_DRV_MC3630_SetOTPSampleRate(E_M_DRV_MC3630_MODE_CWAKE, s_bCfgWakeSRMode);
}

/*****************************************
 *** _M_DRV_MC3630_SetCwakeSampleRateUltraLowPowerMode
 *****************************************/
static void _M_DRV_MC3630_SetCwakeSampleRateUltraLowPowerMode(E_M_DRV_MC3630_CWAKE_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegWAKE = eSR;

    _M_DRV_MC3630_SetWakeOverSampleRate(M_DRV_MC3630_WAKE_SR_MODE_ULTRA_LOW_POWER);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_WAKE_C, &_bRegWAKE, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_WAKE_C, _bRegWAKE );
}

/*****************************************
 *** _M_DRV_MC3630_SetCwakeSampleRateLowPowerMode
 *****************************************/
static void _M_DRV_MC3630_SetCwakeSampleRateLowPowerMode(E_M_DRV_MC3630_CWAKE_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegWAKE = (eSR - E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_BASE);

    _M_DRV_MC3630_SetWakeOverSampleRate(M_DRV_MC3630_WAKE_SR_MODE_LOW_POWER);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_WAKE_C, &_bRegWAKE, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_WAKE_C, _bRegWAKE );
}

/*****************************************
 *** _M_DRV_MC3630_SetCwakeSampleRateLowPrecisionMode
 *****************************************/
static void _M_DRV_MC3630_SetCwakeSampleRateLowPrecisionMode(E_M_DRV_MC3630_CWAKE_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegWAKE = (eSR - E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_BASE);

    _M_DRV_MC3630_SetWakeOverSampleRate(M_DRV_MC3630_WAKE_SR_MODE_LOW_PRECISION);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_WAKE_C, &_bRegWAKE, 1);
}

/*****************************************
 *** _M_DRV_MC3630_SetCwakeSampleRatePrecisionMode
 *****************************************/
static void _M_DRV_MC3630_SetCwakeSampleRatePrecisionMode(E_M_DRV_MC3630_CWAKE_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegWAKE = (eSR - E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_BASE);

    _M_DRV_MC3630_SetWakeOverSampleRate(M_DRV_MC3630_WAKE_SR_MODE_PRECISION);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_WAKE_C, &_bRegWAKE, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_WAKE_C, _bRegWAKE );
}

/*****************************************
 *** _M_DRV_MC3630_SetCwakeSampleRateHighPrecisionMode
 *****************************************/
static void _M_DRV_MC3630_SetCwakeSampleRateHighPrecisionMode(E_M_DRV_MC3630_CWAKE_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegWAKE = (eSR - E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_DUMMY_BASE);

    _M_DRV_MC3630_SetWakeOverSampleRate(M_DRV_MC3630_WAKE_SR_MODE_HIGH_PRECISION);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_WAKE_C, &_bRegWAKE, 1);
}


/*****************************************
 *** _M_DRV_MC3630_SetCwakeSampleRate
 *****************************************/
static int _M_DRV_MC3630_SetCwakeSampleRate(E_M_DRV_MC3630_CWAKE_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    if ((E_M_DRV_MC3630_CWAKE_SR_UL_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_UL_DUMMY_END)) {
        _M_DRV_MC3630_SetCwakeSampleRateUltraLowPowerMode(eSR);
    } else if ((E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_END)) {
        _M_DRV_MC3630_SetCwakeSampleRateLowPowerMode(eSR);
    } else if ((E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_END)) {
        _M_DRV_MC3630_SetCwakeSampleRatePrecisionMode(eSR);
    } else if ((E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_DUMMY_END)) {
        _M_DRV_MC3630_SetCwakeSampleRateHighPrecisionMode(eSR);
    } else if ((E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_END)) {
        _M_DRV_MC3630_SetCwakeSampleRateLowPrecisionMode(eSR);
    } else {
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);
    }

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3630_CheckCwakeSampleRate
 *****************************************/
static int _M_DRV_MC3630_CheckCwakeSampleRate(E_M_DRV_MC3630_CWAKE_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    if ((!((E_M_DRV_MC3630_CWAKE_SR_UL_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_UL_DUMMY_END)))
        && (!((E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_END)))
        && (!((E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_END)))
        && (!((E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_DUMMY_END)))
        && (!((E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_END)))) {
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);
    }

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC3630_SetSniffSampleRate
 *****************************************/
static int _M_DRV_MC3630_SetSniffSampleRate(E_M_DRV_MC3630_SNIFF_SR eSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    unsigned char _bRegWAKE = 0x00;
    unsigned char _bRegSNIFF = 0xC0;

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_WAKE_C, &_bRegWAKE, 1);
    _M_DRV_MC3630_SetSniffOverSampleRate(M_DRV_MC3630_SNIFF_SR_MODE_LOW_POWER);

    _bRegSNIFF |= eSR;
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_C, &_bRegSNIFF, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_SNIFF_C, _bRegSNIFF );


    _bRegWAKE  &= 0x7F;
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_WAKE_C, &_bRegWAKE, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_WAKE_C, _bRegWAKE );

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

#ifdef M_DRV_MC3630_SUPPORT_LPF
/*****************************************
 *** _M_DRV_MC3630_LowPassFilter
 *****************************************/
static void _M_DRV_MC3630_LowPassFilter(signed short _saData[M_DRV_MC3630_AXES_NUM])
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    if (s_debug) M_PRINTF("[CurrData]     %d    %d    %d",
                          _saData[M_DRV_MC3630_AXIS_X], _saData[M_DRV_MC3630_AXIS_Y], _saData[M_DRV_MC3630_AXIS_Z]);

    if (s_debug) M_PRINTF("[PreData]     %d    %d    %d",
                          _saLPFPrevData[M_DRV_MC3630_AXIS_X], _saLPFPrevData[M_DRV_MC3630_AXIS_Y], _saLPFPrevData[M_DRV_MC3630_AXIS_Z]);

    _saData[M_DRV_MC3630_AXIS_X] = _M_DRV_MC3630_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC3630_AXIS_X], _saData[M_DRV_MC3630_AXIS_X]);
    _saData[M_DRV_MC3630_AXIS_Y] = _M_DRV_MC3630_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC3630_AXIS_Y], _saData[M_DRV_MC3630_AXIS_Y]);
    _saData[M_DRV_MC3630_AXIS_Z] = _M_DRV_MC3630_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC3630_AXIS_Z], _saData[M_DRV_MC3630_AXIS_Z]);

    _saLPFPrevData[M_DRV_MC3630_AXIS_X] = _saData[M_DRV_MC3630_AXIS_X];
    _saLPFPrevData[M_DRV_MC3630_AXIS_Y] = _saData[M_DRV_MC3630_AXIS_Y];
    _saLPFPrevData[M_DRV_MC3630_AXIS_Z] = _saData[M_DRV_MC3630_AXIS_Z];

    if (s_debug) M_PRINTF("[M_DRV_MC3630_SUPPORT_LPF]     %d    %d    %d",
                          _saData[M_DRV_MC3630_AXIS_X], _saData[M_DRV_MC3630_AXIS_Y], _saData[M_DRV_MC3630_AXIS_Z]);
}
#endif    // END OF #ifdef M_DRV_MC3630_SUPPORT_LPF

/*****************************************
 *** _M_DRV_MC3630_SetSniffThreshold
 *****************************************/
static int _M_DRV_MC3630_SetSniffThreshold(int axis, uint8_t sniff_thr )
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);
    unsigned char _bRegSniff_addr = 0;
    switch (axis) {
    case M_DRV_MC3630_AXIS_X:
        _bRegSniff_addr = 0x01; //Put X-axis th active
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_CFG, &_bRegSniff_addr, 1);
        break;
    case M_DRV_MC3630_AXIS_Y: //Put Y-axis th active
        _bRegSniff_addr = 0x02;
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_CFG, &_bRegSniff_addr, 1);
        break;
    case M_DRV_MC3630_AXIS_Z: //Put Z-axis th active
        _bRegSniff_addr = 0x03;
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_CFG, &_bRegSniff_addr, 1);
        break;
    default:
        break;
    }

    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_SNIFF_CFG, _bRegSniff_addr);

    s_bCfgSniffThr |= sniff_thr;
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);

    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_SNIFFTH_C, s_bCfgSniffThr);

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*********************************************************************
 *** _M_DRV_MC3630_CfgSniff
 *********************************************************************/
static void _M_DRV_MC3630_CfgSniff(void)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    _M_DRV_MC3630_ResetChip();
    _M_DRV_MC3630_SetBusIF();

    _M_DRV_MC3630_SetSniffAGAIN(E_M_DRV_MC3630_SNIFF_GAIN_HIGH);
    _M_DRV_MC3630_SetSniffSampleRate(s_eSR_SNIFF);

    /* Orion 2X SNIFF threshold*/
    _M_DRV_MC3630_SetSniffThreshold(M_DRV_MC3630_AXIS_X, 4);
    _M_DRV_MC3630_SetSniffThreshold(M_DRV_MC3630_AXIS_Y, 4);
    _M_DRV_MC3630_SetSniffThreshold(M_DRV_MC3630_AXIS_Z, 4);

    _M_DRV_MC3630_REG_WRITE(0x15, &s_bCfgRngResol, 1);
    _M_DRV_MC3630_REG_WRITE(0x16, &s_bCfgFifo, 1);
    _M_DRV_MC3630_REG_WRITE(0x17, &s_bCfgINT, 1);
}

#if 0
/*********************************************************************
 *** _M_DRV_MC3630_CfgWake
 *********************************************************************/
static void _M_DRV_MC3630_CfgWake(void)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    _M_DRV_MC3630_SetBusIF();
    _M_DRV_MC3630_SetWakeAGAIN(E_M_DRV_MC3630_WAKE_GAIN_LOW);
    _M_DRV_MC3630_SetCwakeSampleRate(s_eSR_CWAKE);
}
#endif

/*********************************************************************
 *** _M_DRV_MC3630_SetMode
 *********************************************************************/
static int _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE eNextMode)
{

    unsigned char _bCurrMode = 0;
    unsigned char _bRegMODE_C = 0;
    unsigned char _bGuard = 0;

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_1, &_bCurrMode, 1);

    if (eNextMode == _M_DRV_MC3630_REG_STATUS_1_MODE(_bCurrMode))
        return(M_DRV_MC3630_RETCODE_ERROR_STATUS);

    if (E_M_DRV_MC3630_MODE_SNIFF == eNextMode) {
        if (s_debug) M_PRINTF("[%s] MODE_SNIFF", __func__);
        _M_DRV_MC3630_CfgSniff();
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_PWR_CONTROL, &s_bCfgFifoVdd, 1);
    } else if (E_M_DRV_MC3630_MODE_SLEEP == eNextMode) {
        if (s_debug) M_PRINTF("[%s] MODE_SLEEP", __func__);
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_PWR_CONTROL, &s_bCfgFifoVdd, 1);
    } else if (E_M_DRV_MC3630_MODE_STANDBY == eNextMode) {
        if (s_debug) M_PRINTF("[%s] MODE_STANDBY", __func__);
    } else {
        if (s_debug) M_PRINTF("[%s] MODE WAKE", __func__);
        if (E_M_DRV_MC3630_MODE_CWAKE == eNextMode) {
            if (s_debug) M_PRINTF("[%s] MODE CWAKE", __func__);
            _M_DRV_MC3630_SetCwakeSampleRate(s_eSR_CWAKE);
        }

    }

    _bRegMODE_C |= eNextMode;

    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_MODE_C, &_bRegMODE_C, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_MODE_C, _bRegMODE_C);

    while (1) {
        _bGuard++;

        _M_DRV_MC3630_Delay(1);

        _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_1, &_bCurrMode, 1);

        if (eNextMode == _M_DRV_MC3630_REG_STATUS_1_MODE(_bCurrMode))
            break;

        if (_bGuard > 64)
            return(M_DRV_MC3630_RETCODE_ERROR_SETUP);
    }

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

#if 0
/*****************************************
 *** _M_DRV_MC3630_SetSniffAndOrN
 *****************************************/
static int _M_DRV_MC3630_SetSniffAndOrN(uint8_t LogicAndOr)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bRegAndOrN = 0x00;
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_SNIFFTH_C, &_bRegAndOrN, 1);

    if (LogicAndOr == 1)
        _bRegAndOrN |= (LogicAndOr << 6);
    else if (LogicAndOr == 0)
        _bRegAndOrN &= (LogicAndOr << 6);
    else
        return M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT;

    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_DMY, &_bRegAndOrN, 1);
    return(M_DRV_MC3630_RETCODE_SUCCESS);
}
#endif

#if 0
/*****************************************
 *** _M_DRV_MC3630_SetSniffDetectCount
 *****************************************/
static int _M_DRV_MC3630_SetSniffDetectCount(uint8_t axis, uint8_t SniffCount)
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);

    unsigned char _bRegSniff_Axis = 0;
    unsigned char _bRegSniff_Count = SniffCount; //unsigned SNIFF event count, 1 to 62 events, independent from other channels
    unsigned char _bRegSniff_Count_En = 0x08;
    switch (axis) {
    case M_DRV_MC3630_AXIS_X:
        _bRegSniff_Axis = 0x05; //Select X detection count shadow register
        s_bCfgSniffCfg |= _bRegSniff_Axis;
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);

        break;
    case M_DRV_MC3630_AXIS_Y:
        _bRegSniff_Axis = 0x06; //Select Y detection count shadow register
        s_bCfgSniffCfg |= _bRegSniff_Axis;
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);
        break;
    case M_DRV_MC3630_AXIS_Z:
        _bRegSniff_Axis = 0x07; //Select Z detection count shadow register
        s_bCfgSniffCfg |= _bRegSniff_Axis;
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);
        break;
    default:
        break;
    }

    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_SNIFF_CFG, s_bCfgSniffCfg);

    /*Set detection count as (count +1) */
    s_bCfgSniffThr |= _bRegSniff_Count;
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_SNIFFTH_C, s_bCfgSniffThr);


    /*Enable SNIFF detection counts, required for valid SNIFF wakeup*/
    s_bCfgSniffCfg |= _bRegSniff_Count_En;
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);
    if (s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_SNIFF_CFG, s_bCfgSniffCfg);

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}
#endif

/*********************************************************************
 *** _M_DRV_MC3630_ReadData
 *********************************************************************/
static int _M_DRV_MC3630_ReadData(float faOutput[M_DRV_MC3630_AXES_NUM], int bConverted)
{
    if (s_debug) M_PRINTF("[%s] ", __func__);

    signed short _waRaw[M_DRV_MC3630_AXES_NUM] = { 0 };
    unsigned char _baData[6] = { 0 };

    const S_M_DRV_MC_UTIL_OrientationReMap *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC3630_CFG_ORIENTATION_MAP];

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_XOUT_LSB, _baData, 6);

    _waRaw[M_DRV_MC3630_AXIS_X] = ((signed short) ((_baData[0]) | (_baData[1] << 8)));
    _waRaw[M_DRV_MC3630_AXIS_Y] = ((signed short) ((_baData[2]) | (_baData[3] << 8)));
    _waRaw[M_DRV_MC3630_AXIS_Z] = ((signed short) ((_baData[4]) | (_baData[5] << 8)));

#ifdef M_DRV_MC3630_SUPPORT_LPF
    _M_DRV_MC3630_LowPassFilter(_waRaw);
    _M_DRV_MC3630_LowPassFilter(_waRaw);
    _M_DRV_MC3630_LowPassFilter(_waRaw);
#endif

    faOutput[M_DRV_MC3630_AXIS_X] =
    ((float) (_ptOrienMap->bSign[M_DRV_MC3630_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3630_AXIS_X]]));
    faOutput[M_DRV_MC3630_AXIS_Y] =
    ((float) (_ptOrienMap->bSign[M_DRV_MC3630_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3630_AXIS_Y]]));
    faOutput[M_DRV_MC3630_AXIS_Z] =
    ((float) (_ptOrienMap->bSign[M_DRV_MC3630_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3630_AXIS_Z]]));

    faOutput[M_DRV_MC3630_AXIS_X] = -faOutput[M_DRV_MC3630_AXIS_X];
    faOutput[M_DRV_MC3630_AXIS_Y] = -faOutput[M_DRV_MC3630_AXIS_Y];

    if (bConverted) {
        faOutput[M_DRV_MC3630_AXIS_X] = (faOutput[M_DRV_MC3630_AXIS_X] * s_fMC3XXX_Sensitivity);
        faOutput[M_DRV_MC3630_AXIS_Y] = (faOutput[M_DRV_MC3630_AXIS_Y] * s_fMC3XXX_Sensitivity);
        faOutput[M_DRV_MC3630_AXIS_Z] = (faOutput[M_DRV_MC3630_AXIS_Z] * s_fMC3XXX_Sensitivity);
    }

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*******************************************************************************
 *** FUNCTION
 *******************************************************************************/

/*********************************************************************
 *** M_DRV_MC3630_SetMode
 *********************************************************************/
int M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE eNextMode)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);
    _M_DRV_MC3630_SetMode(eNextMode);

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*****************************************
 *** M_DRV_MC3630_ConfigRegRngResCtrl
 *****************************************/
int M_DRV_MC3630_ConfigRegRngResCtrl(E_M_DRV_MC3630_RANGE eCfgRange, E_M_DRV_MC3630_RESOLUTION eCfgResolution)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bPreMode = 0;
    float _faRange[E_M_DRV_MC3630_RANGE_END] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f, 235.368f };
    float _faResolution[E_M_DRV_MC3630_RESOLUTION_END] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f };

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_MODE_C, &_bPreMode, 1);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);

#ifdef M_DRV_MC3630_CFG_WA_10
    if (E_M_DRV_MC3630_RESOLUTION_12BIT < eCfgResolution) {
        s_bCfgFifo = 0x80;
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_FIFO_C, &s_bCfgFifo, 1);
        s_bCfgFifo = 0x00;
        _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_FIFO_C, &s_bCfgFifo, 1);
    }
#endif

    s_bCfgRngResol = (((eCfgRange << 4) & 0x70) | eCfgResolution);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_RANGE_C, &s_bCfgRngResol, 1);
    if (s_debug) M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC3630_REG_RANGE_C, s_bCfgRngResol );

    s_fMC3XXX_Sensitivity = (_faRange[eCfgRange] / _faResolution[eCfgResolution]);
    if (s_debug) M_PRINTF("[%s] s_fMC3XXX_Sensitivity=%f", __func__, s_fMC3XXX_Sensitivity);
    _M_DRV_MC3630_SetMode((E_M_DRV_MC3630_MODE)_M_DRV_MC3630_REG_MODE_C_MODE(_bPreMode));
    s_eRange = eCfgRange;
    s_eResolution = eCfgResolution;

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*****************************************
 *** M_DRV_MC3630_SetSampleRate
 *****************************************/
int M_DRV_MC3630_SetSampleRate(E_M_DRV_MC3630_CWAKE_SR eCwakeSR, E_M_DRV_MC3630_SNIFF_SR eSniffSR)
{
    if (s_debug) M_PRINTF("[%s]", __func__);
    unsigned char _bPreMode = 0;

    if (M_DRV_MC3630_RETCODE_SUCCESS != _M_DRV_MC3630_CheckCwakeSampleRate(eCwakeSR))
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);

    s_eSR_CWAKE = eCwakeSR;
    s_eSR_SNIFF = eSniffSR;

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_MODE_C, &_bPreMode, 1);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);
    _M_DRV_MC3630_SetMode((E_M_DRV_MC3630_MODE)_M_DRV_MC3630_REG_MODE_C_MODE(_bPreMode));

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}



/*********************************************************************
 *** M_DRV_MC3630_Init
 *********************************************************************/
int M_DRV_MC3630_Init(void)
{
    if (s_debug) M_PRINTF("[%s]", __func__);

    if (M_DRV_MC3630_RETCODE_SUCCESS != _M_DRV_MC3630_ValidateSensorIC())
        return(M_DRV_MC3630_RETCODE_ERROR_IDENTIFICATION);

    _M_DRV_MC3630_ResetChip();

    /* Config Bus Type either SPI or I2C */
    _M_DRV_MC3630_SetBusIF();

    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_PWR_CONTROL, &s_bCfgFifoVdd, 1);

    /*Config Range and Resolution */
    M_DRV_MC3630_ConfigRegRngResCtrl(s_eRange, s_eResolution);

    /*Config Sniff and CWake Sample Rate */
    _M_DRV_MC3630_SetSniffSampleRate(s_eSR_SNIFF);
    _M_DRV_MC3630_SetCwakeSampleRate(s_eSR_CWAKE);

    /* Config Sniff and CWake Analog Gain */
    _M_DRV_MC3630_SetWakeAGAIN(s_eGAIN_WAKE);
    _M_DRV_MC3630_SetSniffAGAIN(s_eGAIN_SNIFF);

#ifdef M_DRV_MC3630_OPERATE_MODE_WAKE_WHEN_READ
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_SLEEP);
#endif

#ifdef M_DRV_MC3630_OPERATE_MODE_ALWAYS_WAKE
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_CWAKE);
#endif

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_FIFO_C, &s_bCfgFifo, 1);
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_INTR_C, &s_bCfgINT, 1);

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}



/*********************************************************************
 *** M_DRV_MC3630_SetGain
 *********************************************************************/
int M_DRV_MC3630_SetGain(E_M_DRV_MC3630_WAKE_GAIN eWakeGain, E_M_DRV_MC3630_SNIFF_GAIN eSniffGain)
{
    if (s_debug) M_PRINTF("[%s] ", __func__);

    unsigned char _bPreMode = 0x00;

    s_eGAIN_WAKE = eWakeGain;
    s_eGAIN_SNIFF = eSniffGain;

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_MODE_C, &_bPreMode, 1);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);
    _M_DRV_MC3630_SetMode((E_M_DRV_MC3630_MODE)_M_DRV_MC3630_REG_MODE_C_MODE(_bPreMode));

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3630_EnableFIFO
 *********************************************************************/
int M_DRV_MC3630_EnableFIFO(E_M_DRV_MC3630_FIFO_CONTROL eCtrl,
                            E_M_DRV_MC3630_FIFO_MODE eMode,
                            unsigned char bThreshold)
{
    if (s_debug) M_PRINTF("[%s] ", __func__);
    unsigned char _bPreMode = 0;
#ifdef ENABLE_MC3630_I2C_BURST_MODE
    unsigned char _bRegValue = 0;
#endif

    if (eCtrl >= E_M_DRV_MC3630_FIFO_CONTROL_END)
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);

    if (eMode >= E_M_DRV_MC3630_FIFO_MODE_END)
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);

    if (bThreshold > 31)
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_MODE_C, &_bPreMode, 1);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);

#ifdef M_DRV_MC3630_CFG_WA_10
    {
        unsigned char _bRegRANGE_C = 0;
        _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_RANGE_C, &_bRegRANGE_C, 1);

        if (E_M_DRV_MC3630_RESOLUTION_12BIT < _M_DRV_MC3630_REG_RANGE_C_RES(_bRegRANGE_C))
            M_DRV_MC3630_ConfigRegRngResCtrl((E_M_DRV_MC3630_RANGE)_M_DRV_MC3630_REG_RANGE_C_RANGE(_bRegRANGE_C), E_M_DRV_MC3630_RESOLUTION_12BIT);
    }
#endif

    s_bCfgFifo = ((eCtrl << 6) | (eMode << 5) | bThreshold);
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_FIFO_C, &s_bCfgFifo, 1);
#ifdef ENABLE_MC3630_I2C_BURST_MODE
    _bRegValue = 0x02;
    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_FEATURE_C_2, &_bRegValue, 1);
#endif	
    _M_DRV_MC3630_SetMode((E_M_DRV_MC3630_MODE)_M_DRV_MC3630_REG_MODE_C_MODE(_bPreMode));

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3630_ConfigINT
 *********************************************************************/
int M_DRV_MC3630_ConfigINT(unsigned char bFifoThreshEnable,
                           unsigned char bFifoFullEnable,
                           unsigned char bFifoEmptyEnable,
                           unsigned char bACQEnable,
                           unsigned char bWakeEnable)
{

    if (s_debug) M_PRINTF("[%s] ", __func__);

    unsigned char _bPreMode = 0;

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_MODE_C, &_bPreMode, 1);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);

    s_bCfgINT = (((bFifoThreshEnable & 0x01) << 6)
                 | ((bFifoFullEnable & 0x01) << 5)
                 | ((bFifoEmptyEnable & 0x01) << 4)
                 | ((bACQEnable & 0x01) << 3)
                 | ((bWakeEnable & 0x01) << 2)
                 | M_DRV_MC3630_INTR_C_IAH_ACTIVE_LOW
                 | M_DRV_MC3630_INTR_C_IPP_MODE_PUSH_PULL);

    _M_DRV_MC3630_REG_WRITE(M_DRV_MC3630_REG_INTR_C, &s_bCfgINT, 1);
    _M_DRV_MC3630_SetMode((E_M_DRV_MC3630_MODE)_M_DRV_MC3630_REG_MODE_C_MODE(_bPreMode));

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3630_ReadData
 *********************************************************************/
int M_DRV_MC3630_ReadData(float faOutput[M_DRV_MC3630_FIFO_DEPTH][M_DRV_MC3630_AXES_NUM], int nNumOfSample)
{
    int _nDataCount = 0;
    unsigned char _bRegStatus1 = 0;
    unsigned char _bRegFIFO_C = 0;

    if ((M_DRV_MC3630_NULL_ADDR == faOutput) || (0 == nNumOfSample))
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);

#ifdef M_DRV_MC3630_OPERATE_MODE_WAKE_WHEN_READ
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);
    _M_DRV_MC3630_SetCwakeSampleRate(s_eSR_CWAKE);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_CWAKE);
#endif

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_1, &_bRegStatus1, 1);
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_FIFO_C, &_bRegFIFO_C, 1);

    /* FIFO Mode */
    if (_M_DRV_MC3630_REG_FIFO_C_FIFO_EN(_bRegFIFO_C)) {

        if (M_DRV_MC3630_FIFO_DEPTH < nNumOfSample)
            nNumOfSample = M_DRV_MC3630_FIFO_DEPTH;

        if (s_debug) M_PRINTF("[%s] FIFO mode read data", __func__);

        if (_M_DRV_MC3630_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            return(M_DRV_MC3630_RETCODE_ERROR_NO_DATA);

        for (_nDataCount = 0; _nDataCount < nNumOfSample; _nDataCount++) {
            _M_DRV_MC3630_ReadData(faOutput[_nDataCount], 1);
            _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_1, &_bRegStatus1, 1);

            if (_M_DRV_MC3630_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1)) {
                _nDataCount++;
                break;
            }
        }
    }
    /* Normal Mode */
    else {
        if (s_debug) M_PRINTF("[%s] Normal mode read data", __func__);
        /*
         * 0: No new sample data has arrived since last read.
         * 1: New sample data has arrived and has been written to FIFO/registers
         */
        if (!_M_DRV_MC3630_REG_STATUS_1_NEW_DATA(_bRegStatus1))
            return(M_DRV_MC3630_RETCODE_ERROR_NO_DATA);

        _M_DRV_MC3630_ReadData(faOutput[0], 1);
        _nDataCount = 1;
    }

    return(_nDataCount);
}

int M_DRV_MC3630_ReadDataLSB(short aOutput[M_DRV_MC3630_FIFO_DEPTH][M_DRV_MC3630_AXES_NUM],
                             int nNumOfSample)
{
    float faOutput[M_DRV_MC3630_FIFO_DEPTH][M_DRV_MC3630_AXES_NUM] = {0};
#ifdef ENABLE_MC3630_I2C_BURST_MODE
    signed short _baData[M_DRV_MC3630_FIFO_DEPTH][M_DRV_MC3630_AXES_NUM] = {0};
    const S_M_DRV_MC_UTIL_OrientationReMap *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC3630_CFG_ORIENTATION_MAP];
    signed short tValue = 0;
#endif    
    int _nDataCount = 0;
    unsigned char _bRegStatus1 = 0;
    unsigned char _bRegFIFO_C = 0;
    int i = 0, j = 0;

    if ((M_DRV_MC3630_NULL_ADDR == faOutput) || (0 == nNumOfSample))
        return(M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT);

#ifdef M_DRV_MC3630_OPERATE_MODE_WAKE_WHEN_READ
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_STANDBY);
    _M_DRV_MC3630_SetCwakeSampleRate(s_eSR_CWAKE);
    _M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE_CWAKE);
#endif

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_1, &_bRegStatus1, 1);
    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_FIFO_C, &_bRegFIFO_C, 1);

    /* FIFO Mode */
    if (_M_DRV_MC3630_REG_FIFO_C_FIFO_EN(_bRegFIFO_C)) {

        if (M_DRV_MC3630_FIFO_DEPTH < nNumOfSample)
            nNumOfSample = M_DRV_MC3630_FIFO_DEPTH;

        if (s_debug) M_PRINTF("[%s] FIFO mode read data", __func__);

#ifdef ENABLE_MC3630_I2C_BURST_MODE
        while(1)
        {
           if(_M_DRV_MC3630_REG_STATUS_1_FIFO_FULL(_bRegStatus1))
           {
              _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_2, &_bRegStatus1, 1);
              break;
           }
           _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_1, &_bRegStatus1, 1);
        }
        

        _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_XOUT_LSB, (uint8_t*)_baData, nNumOfSample * 6);

        for(i=0; i<nNumOfSample; i++)
        {
            _baData[i][M_DRV_MC3630_AXIS_X] = ((_baData[i][M_DRV_MC3630_AXIS_X] & 0x00FF) << 8) | ((_baData[i][M_DRV_MC3630_AXIS_X] & 0xFF00) >> 8);
            _baData[i][M_DRV_MC3630_AXIS_Y] = ((_baData[i][M_DRV_MC3630_AXIS_Y] & 0x00FF) << 8) | ((_baData[i][M_DRV_MC3630_AXIS_Y] & 0xFF00) >> 8);
            _baData[i][M_DRV_MC3630_AXIS_Z] = ((_baData[i][M_DRV_MC3630_AXIS_Z] & 0x00FF) << 8) | ((_baData[i][M_DRV_MC3630_AXIS_Z] & 0xFF00) >> 8);
            
#ifdef M_DRV_MC3630_SUPPORT_LPF
            _M_DRV_MC3630_LowPassFilter(_baData[i]);
            _M_DRV_MC3630_LowPassFilter(_baData[i]);
            _M_DRV_MC3630_LowPassFilter(_baData[i]);
#endif

            faOutput[i][M_DRV_MC3630_AXIS_X] =
            ((float) (_ptOrienMap->bSign[M_DRV_MC3630_AXIS_X] * _baData[i][_ptOrienMap->bMap[M_DRV_MC3630_AXIS_X]]));
            faOutput[i][M_DRV_MC3630_AXIS_Y] =
            ((float) (_ptOrienMap->bSign[M_DRV_MC3630_AXIS_Y] * _baData[i][_ptOrienMap->bMap[M_DRV_MC3630_AXIS_Y]]));
            faOutput[i][M_DRV_MC3630_AXIS_Z] =
            ((float) (_ptOrienMap->bSign[M_DRV_MC3630_AXIS_Z] * _baData[i][_ptOrienMap->bMap[M_DRV_MC3630_AXIS_Z]]));

            faOutput[i][M_DRV_MC3630_AXIS_X] = -faOutput[i][M_DRV_MC3630_AXIS_X];
            faOutput[i][M_DRV_MC3630_AXIS_Y] = -faOutput[i][M_DRV_MC3630_AXIS_Y];
#if 0
            if (bConverted) {
                faOutput[i][M_DRV_MC3630_AXIS_X] = (faOutput[i][M_DRV_MC3630_AXIS_X] * s_fMC3XXX_Sensitivity);
                faOutput[i][M_DRV_MC3630_AXIS_Y] = (faOutput[i][M_DRV_MC3630_AXIS_Y] * s_fMC3XXX_Sensitivity);
                faOutput[i][M_DRV_MC3630_AXIS_Z] = (faOutput[i][M_DRV_MC3630_AXIS_Z] * s_fMC3XXX_Sensitivity);
            }
#endif            
        }
        _nDataCount = nNumOfSample;

#else // ENABLE_MC3630_I2C_BURST_MODE

        if (_M_DRV_MC3630_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            return(M_DRV_MC3630_RETCODE_ERROR_NO_DATA);
			
        for (_nDataCount = 0; _nDataCount < nNumOfSample; _nDataCount++) {
            _M_DRV_MC3630_ReadData(faOutput[_nDataCount], 0);
            _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_1, &_bRegStatus1, 1);

            if (_M_DRV_MC3630_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1)) {
                _nDataCount++;
                break;
            }
        }
#endif

    }
    /* Normal Mode */
    else {
        if (s_debug) M_PRINTF("[%s] Normal mode read data", __func__);
        /*
         * 0: No new sample data has arrived since last read.
         * 1: New sample data has arrived and has been written to FIFO/registers
         */
        if (!_M_DRV_MC3630_REG_STATUS_1_NEW_DATA(_bRegStatus1))
            return(M_DRV_MC3630_RETCODE_ERROR_NO_DATA);

        _M_DRV_MC3630_ReadData(faOutput[0], 0);
        _nDataCount = 1;
    }

    for (i = 0; i < _nDataCount; ++i) {
        for (j = 0; j < M_DRV_MC3630_AXES_NUM; ++j) {
            aOutput[i][j] = (short)((long)faOutput[i][j]);
        }
    }

    return (_nDataCount);
}

void M_DRV_MC3630_LSBToSTD(short Output[M_DRV_MC3630_AXES_NUM],
                           float fOutput[M_DRV_MC3630_AXES_NUM])
{
    int i;

    for (i = 0; i < M_DRV_MC3630_AXES_NUM; ++i)
        fOutput[i] = ((float)(long)Output[i])*s_fMC3XXX_Sensitivity;
}

/*********************************************************************
 *** M_DRV_MC3630_HandleINT
 *********************************************************************/
int M_DRV_MC3630_HandleINT(S_M_DRV_MC3630_InterruptEvent *ptINT_Event)
{
    unsigned char _bRegStatus2 = 0;

    _M_DRV_MC3630_REG_READ(M_DRV_MC3630_REG_STATUS_2, &_bRegStatus2, 1);

    ptINT_Event->bWAKE = _M_DRV_MC3630_REG_STATUS_2_INT_WAKE(_bRegStatus2);
    ptINT_Event->bACQ = _M_DRV_MC3630_REG_STATUS_2_INT_ACQ(_bRegStatus2);
    ptINT_Event->bFIFO_EMPTY = _M_DRV_MC3630_REG_STATUS_2_INT_FIFO_EMPTY(_bRegStatus2);
    ptINT_Event->bFIFO_FULL = _M_DRV_MC3630_REG_STATUS_2_INT_FIFO_FULL(_bRegStatus2);
    ptINT_Event->bFIFO_THRESHOLD = _M_DRV_MC3630_REG_STATUS_2_INT_FIFO_THRESH(_bRegStatus2);
    ptINT_Event->bSWAKE_SNIFF = _M_DRV_MC3630_REG_STATUS_2_INT_SWAKE_SNIFF(_bRegStatus2);
    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3630_ReadRegMap
 *********************************************************************/
int M_DRV_MC3630_ReadRegMap(unsigned char baRegMap[M_DRV_MC3630_REG_MAP_SIZE])
{
    uint8_t _bIndex = 0;
    uint8_t _bRegData = 0;

    for (_bIndex = 0; _bIndex < M_DRV_MC3630_REG_MAP_SIZE; _bIndex++) {
        _M_DRV_MC3630_REG_READ(_bIndex, &_bRegData, 1);

        M_PRINTF("REG[0x%02X] 0x%02X", _bIndex, _bRegData);

        if (0 != baRegMap)
            baRegMap[_bIndex] = _bRegData;
    }

    return(M_DRV_MC3630_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3630_ReadReg
 *********************************************************************/
unsigned char M_DRV_MC3630_ReadReg(unsigned char bRegAddr)
{
    uint8_t _bData = 0;
    _M_DRV_MC3630_REG_READ(bRegAddr, &_bData, 1);

    M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, bRegAddr, _bData );
    return(_bData);
}

#if CFG_MODULE_ACCEL_DRIVER == 3630
/*********************************************************************
 *** mcube_brust_read
 *********************************************************************/
uint32_t mcube_burst_read(const uint32_t address, uint8_t* buf, const uint32_t size)
{
    _M_DRV_MC3630_REG_READ(address, buf, size);

    return(size);
}
#endif

#endif /**< CFG_MODULE_ACCEL_DRIVER == 3630 */
