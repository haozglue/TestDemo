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
#ifndef MC3XXX_H
#define MC3XXX_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************
 *** RETURN CODE
 ***********************************************/
#define MC3XXX_RETCODE_SUCCESS                  (0)
#define MC3XXX_RETCODE_ERROR_I2C                (-1)
#define MC3XXX_RETCODE_ERROR_NULL_POINTER       (-2)
#define MC3XXX_RETCODE_ERROR_STATUS             (-3)
#define MC3XXX_RETCODE_ERROR_SETUP              (-4)
#define MC3XXX_RETCODE_ERROR_GET_DATA           (-5)
#define MC3XXX_RETCODE_ERROR_IDENTIFICATION     (-6)

#define MC3XXX_IS_SUCCEEDED(retcode)            (!MC3XXX_IS_FAILED(retcode))
#define MC3XXX_IS_FAILED(retcode)               ((retcode) < 0)

/***********************************************
 *** PRODUCT ID
 ***********************************************/
#define MC3XXX_PCODE_3210              0x90
#define MC3XXX_PCODE_3230              0x19
#define MC3XXX_PCODE_3250              0x88
#define MC3XXX_PCODE_3410              0xA8
#define MC3XXX_PCODE_3410N             0xB8
#define MC3XXX_PCODE_3430              0x29
#define MC3XXX_PCODE_3430N             0x39
#define MC3XXX_PCODE_3510              0x40
#define MC3XXX_PCODE_3530              0x30
#define MC3XXX_PCODE_3216              0x10
#define MC3XXX_PCODE_3236              0x60
#define MC3XXX_PCODE_RESERVE_1         0x20
#define MC3XXX_PCODE_RESERVE_2         0x11
#define MC3XXX_PCODE_RESERVE_3         0x21
#define MC3XXX_PCODE_RESERVE_4         0x61
#define MC3XXX_PCODE_RESERVE_5         0xA0
#define MC3XXX_PCODE_RESERVE_6         0xE0
#define MC3XXX_PCODE_RESERVE_7         0x91
#define MC3XXX_PCODE_RESERVE_8         0xA1
#define MC3XXX_PCODE_RESERVE_9         0xE1
#define MC3XXX_PCODE_RESERVE_10        0x99

/***********************************************
 *** ENUMERATION
 ***********************************************/
typedef enum {
    MC3XXX_IPP_OPEN_DRAIN = 0,
    MC3XXX_IPP_PUSH_PULL,

    MC3XXX_IPP_NUM,
} MC3XXX_IPP_ENUM;

typedef enum {
    MC3XXX_IAH_ACTIVE_LOW = 0,
    MC3XXX_IAH_ACTIVE_HIGH,

    MC3XXX_IAH_NUM,
} MC3XXX_IAH_ENUM;

typedef enum {
    MC3XXX_CTRL_STANDBY = 0,
    MC3XXX_CTRL_WAKE,

    MC3XXX_CTRL_NUM,
} MC3XXX_CTRL_ENUM;

typedef enum {
    MC3XXX_AXIS_X = 0,
    MC3XXX_AXIS_Y,
    MC3XXX_AXIS_Z,

    MC3XXX_AXIS_NUM,
} MC3XXX_AXIS_ENUM;

typedef enum {
    MC3XXX_OUTPUT_UNIT_RAW = 0,
    MC3XXX_OUTPUT_UNIT_MPSS,

    MC3XXX_OUTPUT_UNIT_NUM,
} MC3XXX_OUTPUT_UNIT_ENUM;

typedef enum {
    MC3XXX_RESOLUTION_HIGH,           // 8g, 14bits

    MC3XXX_RESOLUTION_NUM,
} MC3XXX_RESOLUTION_ENUM;

/***********************************************
 *** TYPE DEFINITION
 ***********************************************/
typedef int8_t MC3XXX_RETCODE;

typedef struct {
    MC3XXX_IPP_ENUM IPP;
    MC3XXX_IAH_ENUM IAH;
    MC3XXX_RESOLUTION_ENUM resolution;
} mc3xxx_config_t;

typedef union {
    int16_t raw[MC3XXX_AXIS_NUM];
    float mpss[MC3XXX_AXIS_NUM];
} mc3xxx_output_t;

MC3XXX_RETCODE mc3xxx_probe(const mc3xxx_config_t config);
void mc3xxx_release(void);
MC3XXX_RETCODE mc3xxx_reset(const mc3xxx_config_t config);
void mc3xxx_power(bool on);
mc3xxx_output_t mc3xxx_output(const MC3XXX_OUTPUT_UNIT_ENUM unit);
uint32_t mcube_burst_read(const uint32_t address, uint8_t* buf, const uint32_t size);

#ifdef __cplusplus
}
#endif

#endif // MC3XXX_H
