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

