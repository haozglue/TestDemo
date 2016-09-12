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

#ifndef _M_DRV_MC3630_H_
#define _M_DRV_MC3630_H_

#if CFG_MODULE_ACCEL_DRIVER == 3630

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define M_DRV_MC3630_CFG_I2C_ADDR                     (0x4C)

#define M_DRV_MC3630_CFG_SAMPLE_RATE_CWAKE_DEFAULT    E_M_DRV_MC3630_CWAKE_SR_LP_54Hz
#define M_DRV_MC3630_CFG_SAMPLE_RATE_SNIFF_DEFAULT    E_M_DRV_MC3630_SNIFF_SR_6Hz
#define M_DRV_MC3630_CFG_RANGE                        E_M_DRV_MC3630_RANGE_8G
#define M_DRV_MC3630_CFG_RESOLUTION                   E_M_DRV_MC3630_RESOLUTION_12BIT
#define M_DRV_MC3630_CFG_ORIENTATION_MAP              E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_UP
#define M_DRV_MC3630_CFG_WAKE_GAIN_DEFAULT            E_M_DRV_MC3630_WAKE_GAIN_LOW
#define M_DRV_MC3630_CFG_SNIFF_GAIN_DEFAULT           E_M_DRV_MC3630_SNIFF_GAIN_HIGH
#define M_DRV_MC3630_CFG_WA_10

//#define M_DRV_MC3630_OPERATE_MODE_WAKE_WHEN_READ
#define M_DRV_MC3630_OPERATE_MODE_ALWAYS_WAKE
//#define M_DRV_MC3630_SUPPORT_LPF

#define M_DRV_MC3630_I2C_WRITE_ADDR                 ((M_DRV_MC3630_CFG_I2C_ADDR << 1) | 0x00)
#define M_DRV_MC3630_I2C_READ_ADDR                  ((M_DRV_MC3630_CFG_I2C_ADDR << 1) | 0x01)

//=============================================     
#define M_DRV_MC3630_INTR_C_IPP_MODE_OPEN_DRAIN     (0x00)
#define M_DRV_MC3630_INTR_C_IPP_MODE_PUSH_PULL      (0x01)

#define M_DRV_MC3630_INTR_C_IAH_ACTIVE_LOW          (0x00)
#define M_DRV_MC3630_INTR_C_IAH_ACTIVE_HIGH         (0x02)

//=============================================    
#define M_DRV_MC3630_REG_EXT_STAT_1                 (0x00)
#define M_DRV_MC3630_REG_EXT_STAT_2                 (0x01)
#define M_DRV_MC3630_REG_XOUT_LSB                   (0x02)
#define M_DRV_MC3630_REG_XOUT_MSB                   (0x03)
#define M_DRV_MC3630_REG_YOUT_LSB                   (0x04)
#define M_DRV_MC3630_REG_YOUT_MSB                   (0x05)
#define M_DRV_MC3630_REG_ZOUT_LSB                   (0x06)
#define M_DRV_MC3630_REG_ZOUT_MSB                   (0x07)
#define M_DRV_MC3630_REG_STATUS_1                   (0x08)
#define M_DRV_MC3630_REG_STATUS_2                   (0x09)
#define M_DRV_MC3630_REG_FEATURE_C_1                (0X0D)
#define M_DRV_MC3630_REG_FEATURE_C_2                (0X0E)
#define M_DRV_MC3630_REG_PWR_CONTROL                (0X0F)
#define M_DRV_MC3630_REG_MODE_C                     (0x10)
#define M_DRV_MC3630_REG_WAKE_C                     (0x11)
#define M_DRV_MC3630_REG_SNIFF_C                    (0x12)
#define M_DRV_MC3630_REG_SNIFFTH_C                  (0x13)
#define M_DRV_MC3630_REG_SNIFF_CFG                  (0x14)
#define M_DRV_MC3630_REG_RANGE_C                    (0x15)
#define M_DRV_MC3630_REG_FIFO_C                     (0x16)
#define M_DRV_MC3630_REG_INTR_C                     (0x17)
#define M_DRV_MC3630_REG_DMX                        (0x20)
#define M_DRV_MC3630_REG_DMY                        (0x21)
#define M_DRV_MC3630_REG_DMZ                        (0x22)
#define M_DRV_MC3630_REG_XOFFL                      (0x2A)
#define M_DRV_MC3630_REG_XOFFH                      (0x2B)
#define M_DRV_MC3630_REG_YOFFL                      (0x2C)
#define M_DRV_MC3630_REG_YOFFH                      (0x2D)
#define M_DRV_MC3630_REG_ZOFFL                      (0x2E)
#define M_DRV_MC3630_REG_ZOFFH                      (0x2F)
#define M_DRV_MC3630_REG_XGAIN                      (0x30)
#define M_DRV_MC3630_REG_YGAIN                      (0x31)
#define M_DRV_MC3630_REG_ZGAIN                      (0x32)
#define M_DRV_MC3630_REG_OPT                        (0x3B)
#define M_DRV_MC3630_REG_LOC_X                      (0x3C)
#define M_DRV_MC3630_REG_LOC_Y                      (0x3D)
#define M_DRV_MC3630_REG_LOT_dAOFSZ                 (0x3E)
#define M_DRV_MC3630_REG_WAF_LOT                    (0x3F)

#define M_DRV_MC3630_NULL_ADDR                      (0)

//Sniff OSR
#define M_DRV_MC3630_SR_MODE_UNKNOWN                (0xFF)
#define M_DRV_MC3630_SNIFF_SR_MODE_ULTRA_LOW_POWER  (0x03)
#define M_DRV_MC3630_SNIFF_SR_MODE_LOW_POWER        (0x00)
#define M_DRV_MC3630_SNIFF_SR_MODE_PRECISION        (0x02)
#define M_DRV_MC3630_SNIFF_SR_MODE_LOW_PRECISION    (0x01)
#define M_DRV_MC3630_SNIFF_SR_MODE_HIGH_PRECISION   (0x04)

//Wake OSR
#define M_DRV_MC3630_WAKE_SR_MODE_ULTRA_LOW_POWER   (0x03)
#define M_DRV_MC3630_WAKE_SR_MODE_LOW_POWER         (0x00)
#define M_DRV_MC3630_WAKE_SR_MODE_PRECISION         (0x02)
#define M_DRV_MC3630_WAKE_SR_MODE_LOW_PRECISION     (0x01)
#define M_DRV_MC3630_WAKE_SR_MODE_HIGH_PRECISION    (0x04)

#ifdef M_DRV_MC3630_SUPPORT_LPF
#define SAMPLE_RATE                                 50 //(Hz) Sampling freq
#define SAMPLE_INTERVAL                             1/SAMPLE_RATE //Sampling interval
#define CUT_OFF_FREQUENCY                           4 //(Hz) Cut-off freq
#define PI                                          3.14
#define ALPHA                                       SAMPLE_INTERVAL/( 1/(2*PI*CUT_OFF_FREQUENCY) + SAMPLE_INTERVAL) //LPF coeff
#endif


#define M_DRV_MC3630_RETCODE_SUCCESS                    (0)
#define M_DRV_MC3630_RETCODE_ERROR_BUS                  (-1)
#define M_DRV_MC3630_RETCODE_ERROR_NULL_POINTER         (-2)
#define M_DRV_MC3630_RETCODE_ERROR_STATUS               (-3)
#define M_DRV_MC3630_RETCODE_ERROR_SETUP                (-4)
#define M_DRV_MC3630_RETCODE_ERROR_GET_DATA             (-5)
#define M_DRV_MC3630_RETCODE_ERROR_IDENTIFICATION       (-6)
#define M_DRV_MC3630_RETCODE_ERROR_NO_DATA              (-7)
#define M_DRV_MC3630_RETCODE_ERROR_WRONG_ARGUMENT       (-8)

#define M_DRV_MC3630_AXIS_X                             0
#define M_DRV_MC3630_AXIS_Y                             1
#define M_DRV_MC3630_AXIS_Z                             2
#define M_DRV_MC3630_AXES_NUM                           3

#define M_DRV_MC3630_FIFO_DEPTH                         32
#define M_DRV_MC3630_REG_MAP_SIZE                       64

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
typedef enum {
    E_M_DRV_MC3630_MODE_SLEEP = 0,
    E_M_DRV_MC3630_MODE_STANDBY,
    E_M_DRV_MC3630_MODE_SNIFF,
    E_M_DRV_MC3630_MODE_FSNIFF,
    E_M_DRV_MC3630_MODE_PWAKE,
    E_M_DRV_MC3630_MODE_CWAKE,
    E_M_DRV_MC3630_MODE_SWAKE,
    E_M_DRV_MC3630_MODE_TRIG,
    E_M_DRV_MC3630_MODE_END
}   E_M_DRV_MC3630_MODE;

typedef enum {
    E_M_DRV_MC3630_RANGE_2G = 0,
    E_M_DRV_MC3630_RANGE_4G,
    E_M_DRV_MC3630_RANGE_8G,
    E_M_DRV_MC3630_RANGE_16G,
    E_M_DRV_MC3630_RANGE_12G,
    E_M_DRV_MC3630_RANGE_24G,
    E_M_DRV_MC3630_RANGE_END
}   E_M_DRV_MC3630_RANGE;

typedef enum {
    E_M_DRV_MC3630_RESOLUTION_6BIT = 0,
    E_M_DRV_MC3630_RESOLUTION_7BIT,
    E_M_DRV_MC3630_RESOLUTION_8BIT,
    E_M_DRV_MC3630_RESOLUTION_10BIT,
    E_M_DRV_MC3630_RESOLUTION_12BIT,
    E_M_DRV_MC3630_RESOLUTION_14BIT,
    E_M_DRV_MC3630_RESOLUTION_END,
}   E_M_DRV_MC3630_RESOLUTION;

typedef enum {
    E_M_DRV_MC3630_CWAKE_SR_UL_DUMMY_BASE = 0,                                       // DO NOT USE (mark)
    E_M_DRV_MC3630_CWAKE_SR_UL_0p4Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_0p8Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_1p6Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_6p5Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_13Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_26Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_51Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_100Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_197Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_389Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_761Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_1122Hz,
    E_M_DRV_MC3630_CWAKE_SR_UL_DUMMY_END,

    E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_BASE = E_M_DRV_MC3630_CWAKE_SR_UL_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC3630_CWAKE_SR_LP_0p4Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_0p8Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_1p7Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_7Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_14Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_27Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_54Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_106Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_210Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_411Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_606Hz,
    E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_END,

    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_BASE = E_M_DRV_MC3630_CWAKE_SR_LP_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_0p2Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_0p9Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_3p6Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_7p3Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_14Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_28Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_55Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_109Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_213Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_315Hz,
    E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_END,                                            // DO NOT USE (mark)

    E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_BASE = E_M_DRV_MC3630_CWAKE_SR_LOW_PR_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC3630_CWAKE_SR_PR_0p2Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_0p4Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_1p8Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_3p7Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_14Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_28Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_55Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_109Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_161Hz,
    E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_END,                                                // DO NOT USE (mark)

    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_DUMMY_BASE = E_M_DRV_MC3630_CWAKE_SR_PR_DUMMY_END,   // DO NOT USE (mark)
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_0p2Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_0p4Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_0p9Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_7p2Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_14Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_28Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_55Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_81Hz,
    E_M_DRV_MC3630_CWAKE_SR_HIGH_PR_DUMMY_END,

}   E_M_DRV_MC3630_CWAKE_SR;

typedef enum {
    E_M_DRV_MC3630_SNIFF_SR_DEFAULT_6Hz = 0,
    E_M_DRV_MC3630_SNIFF_SR_0p4Hz,
    E_M_DRV_MC3630_SNIFF_SR_0p8Hz,
    E_M_DRV_MC3630_SNIFF_SR_2Hz,
    E_M_DRV_MC3630_SNIFF_SR_6Hz,
    E_M_DRV_MC3630_SNIFF_SR_13Hz,
    E_M_DRV_MC3630_SNIFF_SR_26Hz,
    E_M_DRV_MC3630_SNIFF_SR_50Hz,
    E_M_DRV_MC3630_SNIFF_SR_100Hz,
    E_M_DRV_MC3630_SNIFF_SR_200Hz,
    E_M_DRV_MC3630_SNIFF_SR_400Hz,
    E_M_DRV_MC3630_SNIFF_SR_END,
}   E_M_DRV_MC3630_SNIFF_SR;

typedef enum {
    E_M_DRV_MC3630_FIFO_CONTROL_DISABLE = 0,
    E_M_DRV_MC3630_FIFO_CONTROL_ENABLE,
    E_M_DRV_MC3630_FIFO_CONTROL_END,
}   E_M_DRV_MC3630_FIFO_CONTROL;

typedef enum {
    E_M_DRV_MC3630_FIFO_MODE_NORMAL = 0,
    E_M_DRV_MC3630_FIFO_MODE_WATERMARK,
    E_M_DRV_MC3630_FIFO_MODE_END,
}   E_M_DRV_MC3630_FIFO_MODE;

typedef struct {
    unsigned char bWAKE;
    unsigned char bACQ;
    unsigned char bFIFO_EMPTY;
    unsigned char bFIFO_FULL;
    unsigned char bFIFO_THRESHOLD;
    unsigned char bRESV;
    unsigned char bSWAKE_SNIFF;
    unsigned char baPadding[2];
} S_M_DRV_MC3630_InterruptEvent;

typedef enum {
    E_M_DRV_MC3630_WAKE_GAIN_HIGH= 0,
    E_M_DRV_MC3630_WAKE_GAIN_MED,
    E_M_DRV_MC3630_WAKE_GAIN_LOW,
    E_M_DRV_MC3630_WAKE_GAIN_END,
} E_M_DRV_MC3630_WAKE_GAIN;

typedef enum {
    E_M_DRV_MC3630_SNIFF_GAIN_HIGH = 0,
    E_M_DRV_MC3630_SNIFF_GAIN_MED,
    E_M_DRV_MC3630_SNIFF_GAIN_LOW,
    E_M_DRV_MC3630_SNIFF_GAIN_END,
} E_M_DRV_MC3630_SNIFF_GAIN;

/***********************************************
 *** I2C CONFIGURATION
 ***********************************************/
#define I2C_SLAVE_ADDRESS                       0x4C    // Slave address
#define I2C_SPEED_MODE                          2       // Speed mode: 1 (100 kbits/s), 2 (400 kbits/s)
#define I2C_ADDRESS_MODE                        0       // Addressing mode: 0 (7 bit address), 1 (10 bit address)

// I2C initialization
#define I2C_Init()                              i2c_init(I2C_SLAVE_ADDRESS,I2C_SPEED_MODE,I2C_ADDRESS_MODE)

// I2C release
#define I2C_Release()                           i2c_release()

// I2C write byte function
#define I2C_WriteReg(address,value)             i2c_write_byte((address), (value))

// I2C read byte function
#define I2C_ReadReg(address)                    i2c_read_byte((address))

// I2C read data function
#define I2C_ReadData(buf,address,size)          i2c_read_data((buf),(address),(size))

/*******************************************************************************
 *** EXTERNAL FUNCTION
 *******************************************************************************/
extern int M_DRV_MC3630_Init(void);

extern int M_DRV_MC3630_SetMode(E_M_DRV_MC3630_MODE eNextMode);

extern int M_DRV_MC3630_ConfigRegRngResCtrl(E_M_DRV_MC3630_RANGE eCfgRange,
                                            E_M_DRV_MC3630_RESOLUTION eCfgResolution);

extern int M_DRV_MC3630_SetSampleRate(E_M_DRV_MC3630_CWAKE_SR eCwakeSR,
                                      E_M_DRV_MC3630_SNIFF_SR eSniffSR);

extern int M_DRV_MC3630_EnableFIFO(E_M_DRV_MC3630_FIFO_CONTROL eCtrl,
                                   E_M_DRV_MC3630_FIFO_MODE eMode,
                                   unsigned char bThreshold);

extern int M_DRV_MC3630_ConfigINT(unsigned char bFifoThreshEnable,
                                  unsigned char bFifoFullEnable,
                                  unsigned char bFifoEmptyEnable,
                                  unsigned char bACQEnable,
                                  unsigned char bWakeEnable);

extern int M_DRV_MC3630_ReadData(float faOutput[M_DRV_MC3630_FIFO_DEPTH][M_DRV_MC3630_AXES_NUM],
                                 int nNumOfSample);

extern int M_DRV_MC3630_ReadDataLSB(short aOutput[M_DRV_MC3630_FIFO_DEPTH][M_DRV_MC3630_AXES_NUM],
                                    int nNumOfSample);

extern void M_DRV_MC3630_LSBToSTD(short Output[M_DRV_MC3630_AXES_NUM],
                                  float fOutput[M_DRV_MC3630_AXES_NUM]);

extern int M_DRV_MC3630_HandleINT(S_M_DRV_MC3630_InterruptEvent *ptINT_Event);

extern int M_DRV_MC3630_SetGain(E_M_DRV_MC3630_WAKE_GAIN eWakeGain,
                                E_M_DRV_MC3630_SNIFF_GAIN eSniffGain);

// for debug
extern int M_DRV_MC3630_ReadRegMap(unsigned char baRegMap[M_DRV_MC3630_REG_MAP_SIZE]);

extern unsigned char M_DRV_MC3630_ReadReg(unsigned char bRegAddr);

#endif    // END of _M_DRV_MC3630_H_

#endif /**< CFG_MODULE_ACCEL_DRIVER == 3630 */
