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

#ifndef _M_DRV_MC3610_H_
    #define _M_DRV_MC3610_H_

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define M_DRV_MC3610_RETCODE_SUCCESS                 (0)
#define M_DRV_MC3610_RETCODE_ERROR_BUS               (-1)
#define M_DRV_MC3610_RETCODE_ERROR_NULL_POINTER      (-2)
#define M_DRV_MC3610_RETCODE_ERROR_STATUS            (-3)
#define M_DRV_MC3610_RETCODE_ERROR_SETUP             (-4)
#define M_DRV_MC3610_RETCODE_ERROR_GET_DATA          (-5)
#define M_DRV_MC3610_RETCODE_ERROR_IDENTIFICATION    (-6)
#define M_DRV_MC3610_RETCODE_ERROR_NO_DATA           (-7)
#define M_DRV_MC3610_RETCODE_ERROR_WRONG_ARGUMENT    (-8)

#define M_DRV_MC3610_AXIS_X      0
#define M_DRV_MC3610_AXIS_Y      1
#define M_DRV_MC3610_AXIS_Z      2
#define M_DRV_MC3610_AXES_NUM    3

#define M_DRV_MC3610_FIFO_DEPTH    32

#define M_DRV_MC3610_REG_MAP_SIZE    64

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
typedef enum
{
    E_M_DRV_MC3610_MODE_SLEEP = 0,
    E_M_DRV_MC3610_MODE_STANDBY,
    E_M_DRV_MC3610_MODE_SNIFF,
    E_M_DRV_MC3610_MODE_FSNIFF,
    E_M_DRV_MC3610_MODE_PWAKE,
    E_M_DRV_MC3610_MODE_CWAKE,
    E_M_DRV_MC3610_MODE_REARM,
    E_M_DRV_MC3610_MODE_TRIG,
    E_M_DRV_MC3610_MODE_END
}   E_M_DRV_MC3610_MODE;

typedef enum
{
    E_M_DRV_MC3610_RANGE_2G = 0,
    E_M_DRV_MC3610_RANGE_4G,
    E_M_DRV_MC3610_RANGE_8G,
    E_M_DRV_MC3610_RANGE_16G,
    E_M_DRV_MC3610_RANGE_12G,
    E_M_DRV_MC3610_RANGE_24G,    
    E_M_DRV_MC3610_RANGE_END
}   E_M_DRV_MC3610_RANGE;

typedef enum
{
    E_M_DRV_MC3610_RESOLUTION_6BIT = 0,
    E_M_DRV_MC3610_RESOLUTION_7BIT,
    E_M_DRV_MC3610_RESOLUTION_8BIT,
    E_M_DRV_MC3610_RESOLUTION_10BIT,
    E_M_DRV_MC3610_RESOLUTION_12BIT,
    E_M_DRV_MC3610_RESOLUTION_14BIT,
    E_M_DRV_MC3610_RESOLUTION_END,
}   E_M_DRV_MC3610_RESOLUTION;

typedef enum
{
    E_M_DRV_MC3610_CWAKE_SR_UL_DUMMY_BASE = 0,                                       // DO NOT USE (mark)
    E_M_DRV_MC3610_CWAKE_SR_UL_0p4Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_0p8Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_1p4Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_5p5Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_11Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_23Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_46Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_90Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_190Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_370Hz,
    E_M_DRV_MC3610_CWAKE_SR_UL_DUMMY_END,                                            // DO NOT USE (mark)

    E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_BASE = E_M_DRV_MC3610_CWAKE_SR_UL_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC3610_CWAKE_SR_LP_0p4Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_0p8Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_1p5Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_6Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_13Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_25Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_50Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_100Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_200Hz,
    E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_END,                                            // DO NOT USE (mark)

    E_M_DRV_MC3610_CWAKE_SR_PR_DUMMY_BASE = E_M_DRV_MC3610_CWAKE_SR_LP_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC3610_CWAKE_SR_PR_0p2Hz,
    E_M_DRV_MC3610_CWAKE_SR_PR_0p4Hz,
    E_M_DRV_MC3610_CWAKE_SR_PR_1p6Hz,
    E_M_DRV_MC3610_CWAKE_SR_PR_3Hz,
    E_M_DRV_MC3610_CWAKE_SR_PR_14Hz,
    E_M_DRV_MC3610_CWAKE_SR_PR_26Hz,
    E_M_DRV_MC3610_CWAKE_SR_PR_53Hz,
    E_M_DRV_MC3610_CWAKE_SR_PR_DUMMY_END,                                            // DO NOT USE (mark)
}   E_M_DRV_MC3610_CWAKE_SR;

typedef enum
{
    E_M_DRV_MC3610_SNIFF_SR_DEFAULT_6Hz = 0,
    E_M_DRV_MC3610_SNIFF_SR_0p4Hz,
    E_M_DRV_MC3610_SNIFF_SR_0p8Hz,
    E_M_DRV_MC3610_SNIFF_SR_2Hz,
    E_M_DRV_MC3610_SNIFF_SR_6Hz,    // <<----- !!! Use this only !!!
    E_M_DRV_MC3610_SNIFF_SR_13Hz,
    E_M_DRV_MC3610_SNIFF_SR_25Hz,
    E_M_DRV_MC3610_SNIFF_SR_50Hz,
    E_M_DRV_MC3610_SNIFF_SR_100Hz,
    E_M_DRV_MC3610_SNIFF_SR_200Hz,
    E_M_DRV_MC3610_SNIFF_SR_400Hz,
    E_M_DRV_MC3610_SNIFF_SR_END,
}   E_M_DRV_MC3610_SNIFF_SR;

typedef enum
{
    E_M_DRV_MC3610_FIFO_CONTROL_DISABLE = 0,
    E_M_DRV_MC3610_FIFO_CONTROL_ENABLE,
    E_M_DRV_MC3610_FIFO_CONTROL_END,
}   E_M_DRV_MC3610_FIFO_CONTROL;

typedef enum
{
    E_M_DRV_MC3610_FIFO_MODE_NORMAL = 0,
    E_M_DRV_MC3610_FIFO_MODE_WATERMARK,
    E_M_DRV_MC3610_FIFO_MODE_END,
}   E_M_DRV_MC3610_FIFO_MODE;

typedef struct
{
    unsigned char    bWAKE;              // Sensor wakes from sniff mode.
    unsigned char    bACQ;               // New sample is ready and acquired.
    unsigned char    bFIFO_EMPTY;        // FIFO is empty.
    unsigned char    bFIFO_FULL;         // FIFO is fyll.
    unsigned char    bFIFO_THRESHOLD;    // FIFO sample count is equal to or greater than the threshold count.
    unsigned char    bRESV;
    unsigned char    baPadding[2];
}   S_M_DRV_MC3610_InterruptEvent;

/*******************************************************************************
 *** EXTERNAL FUNCTION
 *******************************************************************************/
extern int    M_DRV_MC3610_Init(void);
extern int    M_DRV_MC3610_SetMode(E_M_DRV_MC3610_MODE eNextMode);
extern int    M_DRV_MC3610_ConfigRegRngResCtrl(E_M_DRV_MC3610_RANGE eCfgRange, E_M_DRV_MC3610_RESOLUTION eCfgResolution);
extern int    M_DRV_MC3610_SetSampleRate(E_M_DRV_MC3610_CWAKE_SR eCwakeSR, E_M_DRV_MC3610_SNIFF_SR eSniffSR);

extern int    M_DRV_MC3610_EnableFIFO(E_M_DRV_MC3610_FIFO_CONTROL eCtrl, E_M_DRV_MC3610_FIFO_MODE eMode, unsigned char bThreshold);

extern int    M_DRV_MC3610_ConfigINT(unsigned char bFifoThreshEnable,
                                     unsigned char bFifoFullEnable  ,
                                     unsigned char bFifoEmptyEnable ,
                                     unsigned char bACQEnable       ,
                                     unsigned char bWakeEnable       );

extern int    M_DRV_MC3610_ReadData(float faOutput[M_DRV_MC3610_FIFO_DEPTH][M_DRV_MC3610_AXES_NUM], int nNumOfSample);

extern int    M_DRV_MC3610_HandleINT(S_M_DRV_MC3610_InterruptEvent *ptINT_Event);

// for debug
extern int    M_DRV_MC3610_ReadRegMap(unsigned char baRegMap[M_DRV_MC3610_REG_MAP_SIZE]);
extern unsigned char    M_DRV_MC3610_ReadReg(unsigned char bRegAddr);

#endif    // END of _M_DRV_MC3610_H_

