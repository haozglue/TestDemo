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

#if (USE_SMDEVICE == SMDEVICE_MC3XXX)

#include "mc3xxx_port.h"
#include "mc3xxx.h"
#include "mc3410.h"
#include "mc3413.h"

/***********************************************
 *** COMPILATION WARNNING
 ***********************************************/
#ifndef MC3XXX_BUILD_TARGET
#error "Please define MC3XXX_BUILD_TARGET in mc3xxx_port.h"
#endif // MC3XXX_BUILD_TARGET

#ifndef MC3XXX_BUILD_PLATFORM
#error "Please define MC3XXX_BUILD_PLATFORM in mc3xxx_port.h"
#endif // MC3XXX_BUILD_PLATFORM

/***********************************************
 *** SUPPORT CHIPS
 ***********************************************/
#define MCUBE_MC3410                            (1)
#define MCUBE_MC3413                            (2)
#define MCUBE_MC3XXX                            (3)

/***********************************************
 *** SUPPORT DRIVERS
 ***********************************************/
#define MCUBE_DRIVER_PRECOMPILED                (1)
#define MCUBE_DRIVER_UNIFIED                    (2)

/***********************************************
 *** SUPPORT PLATFORMS
 ***********************************************/
#define MCUBE_PLATFORM_DA1458X                  (1)
#define MCUBE_PLATFORM_ARMM0                    (2)

/***********************************************
 *** DRIVER TYPE
 ***********************************************/
#if (MC3XXX_BUILD_TARGET == MCUBE_MC3410) \
    || (MC3XXX_BUILD_TARGET == MCUBE_MC3413)
#define MCUBE_DRIVER                            MCUBE_DRIVER_PRECOMPILED
#elif (MC3XXX_BUILD_TARGET == MCUBE_MC3XXX)
#define MCUBE_DRIVER                            MCUBE_DRIVER_UNIFIED
#else
#error "Please define valid MC3XXX_BUILD_TARGET in mc3xxx_port.h"
#endif

/***********************************************
 *** TARGET PLATFORM
 ***
 *** Supported platforms:
 ***
 ***    Dialog DA1458X
 ***********************************************/
#if (MC3XXX_BUILD_PLATFORM == MCUBE_PLATFORM_DA1458X)
#define MCUBE_MEMORY_RETAINED                   __attribute__((section("retention_mem_area0"), zero_init))
#else
#warning "Please define valid MC3XXX_BUILD_PLATFORM in mc3xxx_port.h"
#define MCUBE_MEMORY_RETAINED
#endif // (MC3XXX_BUILD_PLATFORM == MCUBE_PLATFORM_DA1458X)

/***********************************************
 *** HELP MACROS
 ***********************************************/
#define _MC3XXX_GLUE2(x,y)                      x ## _ ## y
#define MC3XXX_GLUE2(x,y)                       _MC3XXX_GLUE2(x,y)

#define _MC3XXX_GLUE3(x,y,z)                    x ## _ ## y ## _ ## z
#define MC3XXX_GLUE3(x,y,z)                     _MC3XXX_GLUE3(x,y,z)

#define _MC3XXX_GLUE4(x,y,z,a)                  x ## _ ## y ## _ ## z ## _ ## a
#define MC3XXX_GLUE4(x,y,z,a)                   _MC3XXX_GLUE4(x,y,z,a)

#define MC3XXX_REG(chip,reg)                    MC3XXX_GLUE2(chip,reg)
#define MC3XXX_BTF(chip,reg,btf)                MC3XXX_GLUE3(chip,reg,btf)
#define MC3XXX_OP(chip,reg,btf,op)              MC3XXX_GLUE4(chip,reg,btf,op)
#define MC3XXX_POR(chip,reg)                    MC3XXX_GLUE3(chip,reg,POR)
#define MC3XXX_BASE(chip,reg)                   MC3XXX_GLUE3(chip,reg,BASE)

#define MC3XXX_SHIFT(btf)                       ((btf)&0x01?0: (btf)&0x02?1: (btf)&0x04?2: (btf)&0x08?3:\
                                                (btf)&0x10?4: (btf)&0x20?5: (btf)&0x40?6: 7)

#define MC3XXX_MAKECMD(op,btf,base)             (((op) << MC3XXX_SHIFT(btf)) | (base))

//----------------------------------------------
#if (MCUBE_DRIVER == MCUBE_DRIVER_PRECOMPILED)
/***********************************************
 *** TARGET CHIP
 ***
 *** Supported chips:
 ***
 ***    MC3410, MC3413
 ***********************************************/
#if (MC3XXX_BUILD_TARGET == MCUBE_MC3410)
#define MC3XXX_CHIP                             MC3410
#elif (MC3XXX_BUILD_TARGET == MCUBE_MC3413)
#define MC3XXX_CHIP                             MC3413
#else
#error "Unsuuported chip"
#endif

/***********************************************
 *** REGISTER MAP
 ***********************************************/
/* MODE */
#define MC3XXX_MODE                             MC3XXX_REG(MC3XXX_CHIP, MODE)
#define MC3XXX_MODE_POR                         MC3XXX_POR(MC3XXX_CHIP, MODE)
#define MC3XXX_MODE_BASE                        MC3XXX_BASE(MC3XXX_CHIP, MODE)
#define MC3XXX_MODE_OPCON                       MC3XXX_BTF(MC3XXX_CHIP, MODE, OPCON)
#define MC3XXX_MODE_OPCON_STANDBY               MC3XXX_OP(MC3XXX_CHIP, MODE, OPCON, STANDBY)
#define MC3XXX_MODE_OPCON_WAKE                  MC3XXX_OP(MC3XXX_CHIP, MODE, OPCON, WAKE)
#define MC3XXX_MODE_IPP                         MC3XXX_BTF(MC3XXX_CHIP, MODE, IPP)
#define MC3XXX_MODE_IPP_OPEN_DRAIN              MC3XXX_OP(MC3XXX_CHIP, MODE, IPP, OPEN_DRAIN)
#define MC3XXX_MODE_IPP_PUSH_PULL               MC3XXX_OP(MC3XXX_CHIP, MODE, IPP, PUSH_PULL)
#define MC3XXX_MODE_IAH                         MC3XXX_BTF(MC3XXX_CHIP, MODE, IAH)
#define MC3XXX_MODE_IAH_ACTIVE_LOW              MC3XXX_OP(MC3XXX_CHIP, MODE, IAH, ACTIVE_LOW)
#define MC3XXX_MODE_IAH_ACTIVE_HIGH             MC3XXX_OP(MC3XXX_CHIP, MODE, IAH, ACTIVE_HIGH)
#if (MC3XXX_BUILD_TARGET == MCUBE_MC3413)
    #define MC3XXX_MODE_I2C_WDT_NEG             MC3XXX_BTF(MC3XXX_CHIP, MODE, I2C_WDT_NEG)
    #define MC3XXX_MODE_I2C_WDT_NEG_DISABLE     MC3XXX_OP(MC3XXX_CHIP, MODE, I2C_WDT_NEG, DISABLE)
    #define MC3XXX_MODE_I2C_WDT_NEG_ENABLE      MC3XXX_OP(MC3XXX_CHIP, MODE, I2C_WDT_NEG, ENABLE)
    #define MC3XXX_MODE_I2C_WDT_POS             MC3XXX_BTF(MC3XXX_CHIP, MODE, I2C_WDT_POS)
    #define MC3XXX_MODE_I2C_WDT_POS_DISABLE     MC3XXX_OP(MC3XXX_CHIP, MODE, I2C_WDT_POS, DISABLE)
    #define MC3XXX_MODE_I2C_WDT_POS_ENABLE      MC3XXX_OP(MC3XXX_CHIP, MODE, I2C_WDT_POS, ENABLE)
#endif

/* OUTCFG */
#define MC3XXX_OUTCFG                           MC3XXX_REG(MC3XXX_CHIP, OUTCFG)
#define MC3XXX_OUTCFG_POR                       MC3XXX_POR(MC3XXX_CHIP, OUTCFG)
#define MC3XXX_OUTCFG_BASE                      MC3XXX_BASE(MC3XXX_CHIP, OUTCFG)
#if (MC3XXX_BUILD_TARGET == MCUBE_MC3410)
    #define MC3XXX_OUTCFG_HIRES                 MC3XXX_BTF(MC3XXX_CHIP, OUTCFG, HIRES)
    #define MC3XXX_OUTCFG_HIRES_G2B10           MC3XXX_OP(MC3XXX_CHIP, OUTCFG, HIRES, G2B10)
    #define MC3XXX_OUTCFG_HIRES_G4B10           MC3XXX_OP(MC3XXX_CHIP, OUTCFG, HIRES, G4B10)
    #define MC3XXX_OUTCFG_HIRES_G8B10           MC3XXX_OP(MC3XXX_CHIP, OUTCFG, HIRES, G8B10)
    #define MC3XXX_OUTCFG_HIRES_G8B14           MC3XXX_OP(MC3XXX_CHIP, OUTCFG, HIRES, G8B14)
    #define MC3XXX_OUTCFG_LPF                   MC3XXX_BTF(MC3XXX_CHIP, OUTCFG, LPF)
    #define MC3XXX_OUTCFG_LPF_HZ512             MC3XXX_OP(MC3XXX_CHIP, OUTCFG, LPF, HZ512)
    #define MC3XXX_OUTCFG_LPF_HZ256             MC3XXX_OP(MC3XXX_CHIP, OUTCFG, LPF, HZ256)
    #define MC3XXX_OUTCFG_LPF_HZ128             MC3XXX_OP(MC3XXX_CHIP, OUTCFG, LPF, HZ128)
    #define MC3XXX_OUTCFG_LPF_HZ64              MC3XXX_OP(MC3XXX_CHIP, OUTCFG, LPF, HZ64)
    #define MC3XXX_OUTCFG_LPF_HZ32              MC3XXX_OP(MC3XXX_CHIP, OUTCFG, LPF, HZ32)
    #define MC3XXX_OUTCFG_LPF_HZ16              MC3XXX_OP(MC3XXX_CHIP, OUTCFG, LPF, HZ16)
    #define MC3XXX_OUTCFG_LPF_HZ8               MC3XXX_OP(MC3XXX_CHIP, OUTCFG, LPF, HZ8)
    #define MC3XXX_OUTCFG_IRATE                 MC3XXX_BTF(MC3XXX_CHIP, OUTCFG, IRATE)
    #define MC3XXX_OUTCFG_IRATE_DISABLE         MC3XXX_OP(MC3XXX_CHIP, OUTCFG, IRATE, DISABLE)
    #define MC3XXX_OUTCFG_IRATE_ENABLE          MC3XXX_OP(MC3XXX_CHIP, OUTCFG, IRATE, ENABLE)
#elif (MC3XXX_BUILD_TARGET == MCUBE_MC3413)
    #define MC3XXX_OUTCFG_RES                   MC3XXX_BTF(MC3XXX_CHIP, OUTCFG, RES)
    #define MC3XXX_OUTCFG_RES_B6                MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RES, B6)
    #define MC3XXX_OUTCFG_RES_B7                MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RES, B7)
    #define MC3XXX_OUTCFG_RES_B8                MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RES, B8)
    #define MC3XXX_OUTCFG_RES_B10               MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RES, B10)
    #define MC3XXX_OUTCFG_RES_B12               MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RES, B12)
    #define MC3XXX_OUTCFG_RES_B14               MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RES, B14)
    #define MC3XXX_OUTCFG_RANGE                 MC3XXX_BTF(MC3XXX_CHIP, OUTCFG, RANGE)
    #define MC3XXX_OUTCFG_RANGE_G2              MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RANGE, G2)
    #define MC3XXX_OUTCFG_RANGE_G4              MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RANGE, G4)
    #define MC3XXX_OUTCFG_RANGE_G8              MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RANGE, G8)
    #define MC3XXX_OUTCFG_RANGE_G16             MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RANGE, G16)
    #define MC3XXX_OUTCFG_RANGE_G12             MC3XXX_OP(MC3XXX_CHIP, OUTCFG, RANGE, G12)
#endif // MC3XXX

/* XOUT_EX_L */
#define MC3XXX_XOUT_EX_L                        MC3XXX_REG(MC3XXX_CHIP, XOUT_EX_L)
#define MC3XXX_XOUT_EX_L_POR                    MC3XXX_POR(MC3XXX_CHIP, XOUT_EX_L)
#define MC3XXX_XOUT_EX_L_BASE                   MC3XXX_BASE(MC3XXX_CHIP, XOUT_EX_L)
#define MC3XXX_XOUT_EX_L_XOUT_EX                MC3XXX_BTF(MC3XXX_CHIP, XOUT_EX_L, XOUT_EX)

/* XOUT_EX_H */
#define MC3XXX_XOUT_EX_H                        MC3XXX_REG(MC3XXX_CHIP, XOUT_EX_H)
#define MC3XXX_XOUT_EX_H_POR                    MC3XXX_POR(MC3XXX_CHIP, XOUT_EX_H)
#define MC3XXX_XOUT_EX_H_BASE                   MC3XXX_BASE(MC3XXX_CHIP, XOUT_EX_H)
#define MC3XXX_XOUT_EX_H_XOUT_EX                MC3XXX_BTF(MC3XXX_CHIP, XOUT_EX_H, XOUT_EX)

/* YOUT_EX_L */
#define MC3XXX_YOUT_EX_L                        MC3XXX_REG(MC3XXX_CHIP, YOUT_EX_L)
#define MC3XXX_YOUT_EX_L_POR                    MC3XXX_POR(MC3XXX_CHIP, YOUT_EX_L)
#define MC3XXX_YOUT_EX_L_BASE                   MC3XXX_BASE(MC3XXX_CHIP, YOUT_EX_L)
#define MC3XXX_YOUT_EX_L_YOUT_EX                MC3XXX_BTF(MC3XXX_CHIP, YOUT_EX_L, YOUT_EX)

/* YOUT_EX_H */
#define MC3XXX_YOUT_EX_H                        MC3XXX_REG(MC3XXX_CHIP, YOUT_EX_H)
#define MC3XXX_YOUT_EX_H_POR                    MC3XXX_POR(MC3XXX_CHIP, YOUT_EX_H)
#define MC3XXX_YOUT_EX_H_BASE                   MC3XXX_BASE(MC3XXX_CHIP, YOUT_EX_H)
#define MC3XXX_YOUT_EX_H_YOUT_EX                MC3XXX_BTF(MC3XXX_CHIP, YOUT_EX_H, YOUT_EX)

/* ZOUT_EX_L */
#define MC3XXX_ZOUT_EX_L                        MC3XXX_REG(MC3XXX_CHIP, ZOUT_EX_L)
#define MC3XXX_ZOUT_EX_L_POR                    MC3XXX_POR(MC3XXX_CHIP, ZOUT_EX_L)
#define MC3XXX_ZOUT_EX_L_BASE                   MC3XXX_BASE(MC3XXX_CHIP, ZOUT_EX_L)
#define MC3XXX_ZOUT_EX_L_ZOUT_EX                MC3XXX_BTF(MC3XXX_CHIP, ZOUT_EX_L, ZOUT_EX)

/* ZOUT_EX_H */
#define MC3XXX_ZOUT_EX_H                        MC3XXX_REG(MC3XXX_CHIP, ZOUT_EX_H)
#define MC3XXX_ZOUT_EX_H_POR                    MC3XXX_POR(MC3XXX_CHIP, ZOUT_EX_H)
#define MC3XXX_ZOUT_EX_H_BASE                   MC3XXX_BASE(MC3XXX_CHIP, ZOUT_EX_L)
#define MC3XXX_ZOUT_EX_H_ZOUT_EX                MC3XXX_BTF(MC3XXX_CHIP, ZOUT_EX_H, ZOUT_EX)

/***********************************************
 *** COMMANDS
 ***********************************************/
/* OUTPUT_EX (VIRTUAL) */
#if (MC3XXX_BUILD_TARGET == MCUBE_MC3410)
    #define MC3XXX_CMD_OUTCFG_HIGH              MC3XXX_MAKECMD(MC3XXX_OUTCFG_HIRES_G8B14, MC3XXX_OUTCFG_HIRES, MC3XXX_OUTCFG_BASE)
    #define MC3XXX_RAW2MPSS_DIV_HIGH            (1024.0f)
#elif (MC3XXX_BUILD_TARGET == MCUBE_MC3413)
    #define MC3XXX_CMD_OUTCFG_HIGH              (MC3XXX_MAKECMD(MC3XXX_OUTCFG_RANGE_G8, MC3XXX_OUTCFG_RANGE, MC3XXX_OUTCFG_BASE) \
                                                | MC3XXX_MAKECMD(MC3XXX_OUTCFG_RES_B14, MC3XXX_OUTCFG_RES, MC3XXX_OUTCFG_BASE))
    #define MC3XXX_RAW2MPSS_DIV_HIGH            (1024.0f)
#endif // MC3XXX_BUILD_TARGET ?
#endif // (MCUBE_DRIVER == MCUBE_DRIVER_PRECOMPILED)

#define IS_MCFM12()                             ((0xC0 <= s_Config.s_bHWID) && (s_Config.s_bHWID <= 0xCF))
#define IS_MCFM3X()                             ((0x20 == s_Config.s_bHWID) || ((0x22 <= s_Config.s_bHWID) && (s_Config.s_bHWID <= 0x2F)))

#define MC3XXX_DelayMS(ms)                      (ms > 0) ? System_Deley(ms) : ((void)0)

/***********************************************
 *** CONSTANTS
 ***********************************************/
#define MC3XXX_STD_G                            (9.80665f)

/***********************************************
 *** TYPE DEFINITION
 ***********************************************/
typedef union {
    uint8_t byte;
    mc3410_mode_t mc3410;
    mc3413_mode_t mc3413;
} mc3xxx_mode_t;

typedef union {
    uint8_t byte;
    mc3410_outcfg_t mc3410;
    mc3413_outcfg_t mc3413;
} mc3xxx_outcfg_t;

typedef union {
    uint8_t byte;
    mc3410_xout_ex_l_t mc3410;
    mc3413_xout_ex_l_t mc3413;
} mc3xxx_xout_ex_l_t;

typedef union {
    uint8_t byte;
    mc3410_xout_ex_h_t mc3410;
    mc3413_xout_ex_h_t mc3413;
} mc3xxx_xout_ex_h_t;

typedef union {
    uint8_t byte;
    mc3410_yout_ex_l_t mc3410;
    mc3413_yout_ex_l_t mc3413;
} mc3xxx_yout_ex_l_t;

typedef union {
    uint8_t byte;
    mc3410_yout_ex_h_t mc3410;
    mc3413_yout_ex_h_t mc3413;
} mc3xxx_yout_ex_h_t;

typedef union {
    uint8_t byte;
    mc3410_zout_ex_l_t mc3410;
    mc3413_zout_ex_l_t mc3413;
} mc3xxx_zout_ex_l_t;

typedef union {
    uint8_t byte;
    mc3410_zout_ex_h_t mc3410;
    mc3413_zout_ex_h_t mc3413;
} mc3xxx_zout_ex_h_t;

typedef struct {
    MC3XXX_IPP_ENUM IPP;
    MC3XXX_IAH_ENUM IAH;
    MC3XXX_RESOLUTION_ENUM resolution;
    // Internal attrbutes
    uint8_t s_bPCODE;
    uint8_t s_bHWID;
    MC3XXX_CTRL_ENUM s_CTRL;
} mc3xxx_internal_config_t;

/*****************************************
 *** LOCAL STATIC FUNCTIONS DECLARATION
 *****************************************/
static uint32_t mc3xxx_burst_read(const uint32_t address, uint8_t* buf, const uint32_t size);
static uint32_t mc3xxx_wakeup_and_burst_read(const uint32_t address, uint8_t* buf, const uint32_t size);
static void mc3xxx_write_register(const uint32_t address, uint8_t value);
static MC3XXX_RETCODE mc3xxx_get_pcode_and_hwid(uint8_t *pbPCode, uint8_t *pbHwID);
static void mc3xxx_set_resolution(const MC3XXX_RESOLUTION_ENUM resolution);
static float mc3xxx_raw2mpss(const MC3XXX_RESOLUTION_ENUM resolution, const int16_t raw);
static void mc3xxx_control(const MC3XXX_CTRL_ENUM ctrl);
static MC3XXX_RETCODE mc3xxx_setup(const mc3xxx_config_t config);

/*****************************************
 *** LOCAL VARIABLES
 *****************************************/
mc3xxx_internal_config_t s_Config MCUBE_MEMORY_RETAINED;

#ifdef DEBUG_MC3XXX
/*****************************************
 *** DUMP REGISTERS MAP
 *****************************************/
void mc3xxx_dump(void)
{
    uint8_t i, data[8] = {0};

    mc3xxx_burst_read(0x00, &data[0], 8);

    for (i = 0; i < 8; ++i) {
        Debug_Printf("REG[%02x] = %02x\r\n", i, data[i]);
    }

    mc3xxx_burst_read(0x20, &data[0], 1);

    for (i = 0; i < 1; ++i) {
        Debug_Printf("REG[%02x] = %02x\r\n", 0x20+i, data[i]);
    }

    mc3xxx_burst_read(0x0D, &data[0], 6);

    for (i = 0; i < 6; ++i) {
        Debug_Printf("REG[%02x] = %02x\r\n", 0x0D+i, data[i]);
    }

    return;
}

/*****************************************
 *** DUMP REGISTERS
 *****************************************/
void mc3xxx_dump_reg(uint8_t addr)
{
    uint8_t data[1] = {0};

    mc3xxx_burst_read(addr, &data[0], 1);

    Debug_Printf("REG[%02x] = %02x\r\n", addr, data[0]);
}
#else // ? DEBUG_MC3XXX
    #define mc3xxx_dump() {}
    #define mc3xxx_dump_reg(addr) {}
#endif // DEBUG_MC3XXX

/*****************************************
 *** WAKEUP AND BRUST READ
 *****************************************/
uint32_t mc3xxx_wakeup_and_burst_read(const uint32_t address, uint8_t* buf, const uint32_t size)
{
    uint32_t ret;

    mc3xxx_control(MC3XXX_CTRL_WAKE);
    ret = mc3xxx_burst_read(address, buf, size);
    mc3xxx_control(MC3XXX_CTRL_STANDBY);

    return ret;
}

/*****************************************
 *** WRITE REGISTER
 *****************************************/
void mc3xxx_write_register(const uint32_t address, uint8_t value)
{
    I2C_Init();

    MC3XXX_DelayMS(MC3XXX_I2C_INIT_DELAY);

    I2C_WriteReg(address, value);

    I2C_Release();
}

/*****************************************
 *** GET PCODE & HWID
 *****************************************/
MC3XXX_RETCODE mc3xxx_get_pcode_and_hwid(uint8_t *pbPCode, uint8_t *pbHwID)
{
    uint8_t pcode = 0, hwid = 0;

    if (!pbPCode || !pbHwID) {
        goto EXIT;
    }

    pcode = *pbPCode;
    hwid = *pbHwID;

    if (pcode == 0x00) {
        mc3xxx_burst_read(0x3B, &pcode, 1);
    }

    if (hwid == 0x00) {
        mc3xxx_burst_read(0x18, &hwid, 1);
    }

    if ((0x01 == hwid) || (0x03 == hwid) || ((0x04 <= hwid) && (hwid <= 0x0F))) {
        if ((MC3XXX_PCODE_3210 == pcode) || (MC3XXX_PCODE_3250 == pcode)) {
            goto EXIT_SUCCESS;
        }
    } else if ((0x02 == hwid) || (0x21 == hwid) || ((0x10 <= hwid) && (hwid <= 0x1F))) {
        if ((MC3XXX_PCODE_3210 == pcode) || (MC3XXX_PCODE_3250 == pcode) ||
            (MC3XXX_PCODE_3410 == pcode) || (MC3XXX_PCODE_3410N == pcode)) {
            goto EXIT_SUCCESS;
        }
    } else if ((0x20 == hwid) || ((0x22 <= hwid) && (hwid <= 0x2F))) {
        pcode = (pcode & 0xF1);
        if ((MC3XXX_PCODE_3210 == pcode) || (MC3XXX_PCODE_3216 == pcode)) {
            goto EXIT_SUCCESS;
        }
    }

EXIT:
    //Debug_Printf("pcode %02x hwid %02x\r\n", s_Config.s_bPCODE, s_Config.s_bHWID);
    return(MC3XXX_RETCODE_ERROR_IDENTIFICATION);

EXIT_SUCCESS:
    *pbPCode = pcode;
    *pbHwID = hwid;
    //Debug_Printf("pcode %02x hwid %02x\r\n", s_Config.s_bPCODE, s_Config.s_bHWID);
    return(MC3XXX_RETCODE_SUCCESS);
}

/*****************************************
 *** SET RESOLUTION
 *****************************************/
void mc3xxx_set_resolution(const MC3XXX_RESOLUTION_ENUM resolution)
{
    uint8_t cmd;

#if (MCUBE_DRIVER == MCUBE_DRIVER_PRECOMPILED)
    if (resolution == MC3XXX_RESOLUTION_HIGH) {
        cmd = MC3XXX_CMD_OUTCFG_HIGH;
    }

    mc3xxx_write_register(MC3XXX_OUTCFG, cmd);
#elif (MCUBE_DRIVER == MCUBE_DRIVER_UNIFIED)
    cmd = 0x0F;

    if (IS_MCFM12() || IS_MCFM3X()) {
        if (MC3XXX_RESOLUTION_HIGH == resolution) {
            cmd = 0x25;
        }
    }

    mc3xxx_write_register(0x20, cmd);
#endif // MCUBE_DRIVER ?
}

/*****************************************
 *** CONVERT RAW TO MPSS
 *****************************************/
float mc3xxx_raw2mpss(const MC3XXX_RESOLUTION_ENUM resolution, const int16_t raw)
{
#if (MCUBE_DRIVER == MCUBE_DRIVER_UNIFIED)
    #define MC3XXX_RAW2MPSS_DIV_HIGH        1024.0f
#endif // (MCUBE_DRIVER == MCUBE_DRIVER_UNIFIED)

    float mpss = 0;

    if (resolution == MC3XXX_RESOLUTION_HIGH) {  // 8g, 14 bits
        mpss = (((float)raw)/MC3XXX_RAW2MPSS_DIV_HIGH*MC3XXX_STD_G);
    }

    return mpss;
}

/*****************************************
 *** CONTROL SENSOR
 *****************************************/
void mc3xxx_control(const MC3XXX_CTRL_ENUM ctrl)
{
    uint8_t value = 0;

    if (s_Config.s_CTRL == ctrl) {
        return;
    }

#if (MCUBE_DRIVER == MCUBE_DRIVER_PRECOMPILED)
    if (s_Config.IPP == MC3XXX_IPP_OPEN_DRAIN) {
        value |= MC3XXX_MAKECMD(MC3XXX_MODE_IPP_OPEN_DRAIN, MC3XXX_MODE_IPP, MC3XXX_MODE_BASE);
    } else if (s_Config.IPP == MC3XXX_IPP_PUSH_PULL) {
        value |= MC3XXX_MAKECMD(MC3XXX_MODE_IPP_PUSH_PULL, MC3XXX_MODE_IPP, MC3XXX_MODE_BASE);
    }

    if (s_Config.IAH == MC3XXX_IAH_ACTIVE_LOW) {
        value |= MC3XXX_MAKECMD(MC3XXX_MODE_IAH_ACTIVE_LOW, MC3XXX_MODE_IAH, MC3XXX_MODE_BASE);
    } else if (s_Config.IAH == MC3XXX_IAH_ACTIVE_HIGH) {
        value |= MC3XXX_MAKECMD(MC3XXX_MODE_IAH_ACTIVE_HIGH, MC3XXX_MODE_IAH, MC3XXX_MODE_BASE);
    }

    if (ctrl == MC3XXX_CTRL_STANDBY) {
        value |= MC3XXX_MAKECMD(MC3XXX_MODE_OPCON_STANDBY, MC3XXX_MODE_OPCON, MC3XXX_MODE_BASE);
    } else if (ctrl == MC3XXX_CTRL_WAKE) {
        value |= MC3XXX_MAKECMD(MC3XXX_MODE_OPCON_WAKE, MC3XXX_MODE_OPCON, MC3XXX_MODE_BASE);
    }

    mc3xxx_write_register(MC3XXX_MODE, value);
#elif (MCUBE_DRIVER == MCUBE_DRIVER_UNIFIED)
    if (s_Config.IPP == MC3XXX_IPP_PUSH_PULL) {
        value |= 0x40;
    }

    if (s_Config.IAH == MC3XXX_IAH_ACTIVE_HIGH) {
        value |= 0x80;
    }

    if (ctrl == MC3XXX_CTRL_STANDBY) {
        if (IS_MCFM12() || IS_MCFM3X()) {
            value |= 0x00;
        } else {
            value |= 0x03;
        }
    } else if (ctrl == MC3XXX_CTRL_WAKE) {
        value |= 0x01;
    }

    mc3xxx_write_register(0x07, value);
#endif // MCUBE_DRIVER ?

    if (ctrl == MC3XXX_CTRL_WAKE) {
        MC3XXX_DelayMS(MC3XXX_WAKEUP_DELAY);
    } else if (ctrl == MC3XXX_CTRL_STANDBY) {
        MC3XXX_DelayMS(MC3XXX_STANDBY_DELAY);
    }

    s_Config.s_CTRL = ctrl;
}

/*****************************************
 *** SETUP SENSOR
 *****************************************/
MC3XXX_RETCODE mc3xxx_setup(const mc3xxx_config_t config)
{
    MC3XXX_RETCODE ret;

    s_Config.IAH = config.IAH;
    s_Config.IPP = config.IPP;
    s_Config.resolution = config.resolution;

    ret = mc3xxx_get_pcode_and_hwid(&(s_Config.s_bPCODE), &(s_Config.s_bHWID));
    if (MC3XXX_IS_FAILED(ret)) {
        return(ret);
    }

    mc3xxx_set_resolution(s_Config.resolution);

    mc3xxx_dump();

    return MC3XXX_RETCODE_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////
/// DRIVER INTERFACES
////////////////////////////////////////////////////////////////////////////////////////

/** 
 ***************************************************************************************
 * @brief Initialize sensor
 * 
 * @param[in]   config  Sensor configuration
 *
 * @return      Senor return code 
 ***************************************************************************************
 */
MC3XXX_RETCODE mc3xxx_probe(const mc3xxx_config_t config)
{
    mc3xxx_control(MC3XXX_CTRL_STANDBY);    // make sure it is stanby if not set it
    return mc3xxx_setup(config);
}

/** 
 ***************************************************************************************
 * @brief Release sensor
 **************************************************************************************
 */
void mc3xxx_release(void)
{
    mc3xxx_control(MC3XXX_CTRL_STANDBY);
}

/** 
 ***************************************************************************************
 * @brief Reset sensor
 * 
 * @param[in]   config  Sensor configuration
 *
 * @return      Senor return code 
 ***************************************************************************************
 */
MC3XXX_RETCODE mc3xxx_reset(const mc3xxx_config_t config)
{
    mc3xxx_control(MC3XXX_CTRL_STANDBY);
    return mc3xxx_setup(config);
}

/** 
 ***************************************************************************************
 * @brief Power sensor on or off
 * 
 * @param[in]   on        Sensor power on or off
 * 
 ***************************************************************************************
 */
void mc3xxx_power(bool on)
{
    if (on) {
        mc3xxx_control(MC3XXX_CTRL_WAKE);
    } else {
        mc3xxx_control(MC3XXX_CTRL_STANDBY);
    }
}

/** 
 ****************************************************************************************
 * @brief Get sensor outputs
 * 
 * @param[in]       unit        output unit
 * 
 * @return Sensor output values in raw or m/s^2.
 ****************************************************************************************
 */
mc3xxx_output_t mc3xxx_output(const MC3XXX_OUTPUT_UNIT_ENUM unit)
{
    mc3xxx_output_t output = {0};
    int16_t raw[MC3XXX_AXIS_NUM] = {0};
    uint8_t data[6] = {0};
    uint8_t r;

    if (s_Config.resolution == MC3XXX_RESOLUTION_HIGH) {
        r = (uint8_t)mc3xxx_wakeup_and_burst_read(0x0D, &data[0], 6);
        if (r != 6) {
            Error_Printf("Failed to read output data\r\n");
            goto EXIT;
        }

        for (r = 0; r < MC3XXX_AXIS_NUM; ++r) {
            raw[r] = (int16_t)(((uint16_t)data[r*2]) | (((uint16_t)data[r*2+1]) << 8));
            if (unit == MC3XXX_OUTPUT_UNIT_RAW) {
                output.raw[r] = raw[r];
            } else if (unit == MC3XXX_OUTPUT_UNIT_MPSS) {
                output.mpss[r] = mc3xxx_raw2mpss(MC3XXX_RESOLUTION_HIGH, raw[r]);
            }
        }
    }

EXIT:
    return output;
}

/** 
 ****************************************************************************************
 * @brief Burst read data via sensor I2C interface
 * 
 * @param[in]       address     Address
 * @param[in,out]   buf         Data buffer
 * @param[in]       size        Data buffer size
 * 
 * @return Bytes that are actually read.
 ****************************************************************************************
 */
uint32_t mc3xxx_burst_read(const uint32_t address, uint8_t* buf, const uint32_t size)
{
    uint32_t ret;

    I2C_Init();

    MC3XXX_DelayMS(MC3XXX_I2C_INIT_DELAY);

    ret = I2C_ReadData(buf, address, size);

    I2C_Release();

    return ret;
}

uint32_t mcube_burst_read(const uint32_t address, uint8_t* buf, const uint32_t size)
{
    return mc3xxx_burst_read(address, buf, size);
}

#endif    // END of #if (USE_SMDEVICE == SMDEVICE_MC3XXX)

