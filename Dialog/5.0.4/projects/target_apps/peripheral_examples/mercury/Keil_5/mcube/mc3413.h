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
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
 *
 * Accelerometer Sensor Driver
 * 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/

#ifndef MC3413_H
#define MC3413_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>

/*****************************************
 *** Mode Register
 *****************************************/
typedef struct {
    uint8_t OPCON_0: 1;
    uint8_t OPCON_1: 1;
    uint8_t : 1;
    uint8_t Resv: 1;
    uint8_t I2C_WDT_NEG: 1;
    uint8_t I2C_WDT_POS: 1;
    uint8_t IPP: 1;
    uint8_t IAH: 1;
} mc3413_mode_t;

#define MC3413_MODE                         0x07
#define MC3413_MODE_POR                     0x00
#define MC3413_MODE_BASE                    0x00
#define MC3413_MODE_OPCON                   0x03
#define MC3413_MODE_OPCON_STANDBY           0x00
#define MC3413_MODE_OPCON_WAKE              0x01
#define MC3413_MODE_I2C_WDT_NEG             0x10
#define MC3413_MODE_I2C_WDT_NEG_DISABLE     0x00
#define MC3413_MODE_I2C_WDT_NEG_ENABLE      0x01
#define MC3413_MODE_I2C_WDT_POS             0x20
#define MC3413_MODE_I2C_WDT_POS_DISABLE     0x00
#define MC3413_MODE_I2C_WDT_POS_ENABLE      0x01
#define MC3413_MODE_IPP                     0x40
#define MC3413_MODE_IPP_OPEN_DRAIN          0x00
#define MC3413_MODE_IPP_PUSH_PULL           0x01
#define MC3413_MODE_IAH                     0x80
#define MC3413_MODE_IAH_ACTIVE_LOW          0x00
#define MC3413_MODE_IAH_ACTIVE_HIGH         0x01

/*****************************************
 *** Output Configuration Register
 *****************************************/
typedef struct {
    uint8_t RES: 3;
    uint8_t Resv: 1;
    uint8_t RANGE: 3;
    uint8_t: 1;
} mc3413_outcfg_t;

#define MC3413_OUTCFG                       0x20
#define MC3413_OUTCFG_POR                   0x00
#define MC3413_OUTCFG_BASE                  0x00
#define MC3413_OUTCFG_RES                   0x07
#define MC3413_OUTCFG_RES_B6                0x00
#define MC3413_OUTCFG_RES_B7                0x01
#define MC3413_OUTCFG_RES_B8                0x02
#define MC3413_OUTCFG_RES_B10               0x03
#define MC3413_OUTCFG_RES_B12               0x04
#define MC3413_OUTCFG_RES_B14               0x05
#define MC3413_OUTCFG_RANGE                 0x70
#define MC3413_OUTCFG_RANGE_G2              0x00
#define MC3413_OUTCFG_RANGE_G4              0x01
#define MC3413_OUTCFG_RANGE_G8              0x02
#define MC3413_OUTCFG_RANGE_G16             0x03
#define MC3413_OUTCFG_RANGE_G12             0x04

/*****************************************
 *** X Axis Extended Accelerometer Register
 *****************************************/
typedef struct {
    uint8_t XOUT_EX_L: 8;
} mc3413_xout_ex_l_t;

#define MC3413_XOUT_EX_L                    0x0D
#define MC3413_XOUT_EX_L_POR                0x00
#define MC3413_XOUT_EX_L_BASE               0x00
#define MC3413_XOUT_EX_L_XOUT_EX            0xff

typedef struct {
    uint8_t XOUT_EX_H: 8;
} mc3413_xout_ex_h_t;

#define MC3413_XOUT_EX_H                    0x0E
#define MC3413_XOUT_EX_H_POR                0x00
#define MC3413_XOUT_EX_H_BASE               0x00
#define MC3413_XOUT_EX_H_XOUT_EX            0xff

/*****************************************
 *** Y Axis Extended Accelerometer Register
 *****************************************/
typedef struct {
    uint8_t YOUT_EX_L: 8;
} mc3413_yout_ex_l_t;

#define MC3413_YOUT_EX_L                    0x0F
#define MC3413_YOUT_EX_L_POR                0x00
#define MC3413_YOUT_EX_L_BASE               0x00
#define MC3413_YOUT_EX_L_YOUT_EX            0xff

typedef struct {
    uint8_t YOUT_EX_H: 8;
} mc3413_yout_ex_h_t;

#define MC3413_YOUT_EX_H                    0x10
#define MC3413_YOUT_EX_H_POR                0x00
#define MC3413_YOUT_EX_H_BASE               0x00
#define MC3413_YOUT_EX_H_YOUT_EX            0xff

/*****************************************
 *** Z Axis Extended Accelerometer Register
 *****************************************/
typedef struct {
    uint8_t ZOUT_EX_L: 8;
} mc3413_zout_ex_l_t;

#define MC3413_ZOUT_EX_L                    0x11
#define MC3413_ZOUT_EX_L_POR                0x00
#define MC3413_ZOUT_EX_L_BASE               0x00
#define MC3413_ZOUT_EX_L_ZOUT_EX            0xff

typedef struct {
    uint8_t ZOUT_EX_H: 8;
} mc3413_zout_ex_h_t;

#define MC3413_ZOUT_EX_H                    0x12
#define MC3413_ZOUT_EX_H_POR                0x00
#define MC3413_ZOUT_EX_H_BASE               0x00
#define MC3413_ZOUT_EX_H_ZOUT_EX            0xff

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MC3413_H
