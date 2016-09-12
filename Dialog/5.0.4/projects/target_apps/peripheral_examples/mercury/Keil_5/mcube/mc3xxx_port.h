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

#ifndef MC3XXX_PORT_H
#define MC3XXX_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#define MC3XXX_BUILD_TARGET                 MCUBE_MC3XXX
#define MC3XXX_BUILD_PLATFORM               MCUBE_PLATFORM_DA1458X
#define MC3XXX_WAKEUP_DELAY                 4
#define MC3XXX_STANDBY_DELAY                0
#define MC3XXX_I2C_INIT_DELAY               0

/***********************************************
 *** Platform related headers and macros
 ***********************************************/
#include "app_api.h"

#ifndef _I2C_CORE_H
#   define _I2C_CORE_H
#   define i2c_init(addr,speed,mode)            {}
#   define i2c_release()                        {}
#   define i2c_write_byte(addr,value)           {}
#   define i2c_read_byte(addr)                  (0)
#   define i2c_read_data(buf,addr,size)         (0)
#endif
    
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

/*****************************************
 *** DELAY CONFIGURATION
 *****************************************/
#define System_Deley(ms)                        systick_wait((ms)*1000) // ms

/*****************************************
 *** DEBUG CONFIGURATION
 *****************************************/
#define Null_Printf(fmt, ...)                   ((void)0)
#define Debug_Printf                            Null_Printf
#define Error_Printf                            Null_Printf

#ifdef __cplusplus
}
#endif

#endif // MC3XXX_PORT_H
