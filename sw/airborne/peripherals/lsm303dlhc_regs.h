/*
 * Copyright (C) 2012-2013 The Acoslab Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file peripherals/mpu60x0_regs.h
 *
 * Register and address definitions for MPU-6000 and MPU-6050.
 */

#ifndef LSM303DLHC_REGS_H
#define LSM303DLHC_REGS_H

// Magnetometer
#define LSM303DLHC_REG_MAGN_CONF_A     0x00
#define LSM303DLHC_REG_MAGN_CONF_B     0x01
#define LSM303DLHC_REG_MAGN_MODE       0x02
#define LSM303DLHC_REG_MAGN_XOUT_H     0x03
#define LSM303DLHC_REG_MAGN_XOUT_L     0x04
#define LSM303DLHC_REG_MAGN_YOUT_H     0x05
#define LSM303DLHC_REG_MAGN_YOUT_L     0x06
#define LSM303DLHC_REG_MAGN_ZOUT_H     0x07
#define LSM303DLHC_REG_MAGN_ZOUT_L     0x08
#define LSM303DLHC_REG_MAGN_STATUS     0x09
#define LSM303DLHC_REG_MAGN_IDTF_A     0x0A
#define LSM303DLHC_REG_MAGN_IDTF_B     0x0B
#define LSM303DLHC_REG_MAGN_IDTF_C     0x0C

//Acelerometer
#define LSM303DLHC_REG_CRTL_REG1       0x20
#define LSM303DLHC_REG_CRTL_REG2       0x21
#define LSM303DLHC_REG_CRTL_REG1       0x22
#define LSM303DLHC_REG_CRTL_REG2       0x23
#define LSM303DLHC_REG_CRTL_REG1       0x24
#define LSM303DLHC_REG_CRTL_REG2       0x25 
#define LSM303DLHC_REG_REFERENCE       0x26
#define LSM303DLHC_REG_STATUS          0x27
#define LSM303DLHC_REG_ACCEL_XOUT_L    0x28
#define LSM303DLHC_REG_ACCEL_XOUT_H    0x29
#define LSM303DLHC_REG_ACCEL_YOUT_L    0x2A
#define LSM303DLHC_REG_ACCEL_YOUT_H    0x2B
#define LSM303DLHC_REG_ACCEL_ZOUT_L    0x2C
#define LSM303DLHC_REG_ACCEL_ZOUT_H    0x2D
#define LSM303DLHC_REG_FIFO_CTRL       0x2E
#define LSM303DLHC_REG_FIFO_SRC        0x2F
#define LSM303DLHC_REG_INT1_CFG        0x30
#define LSM303DLHC_REG_INT1_SOURCE     0x31
#define LSM303DLHC_REG_INT1_THS        0x32
#define LSM303DLHC_REG_INT1_DURATION   0x33
#define LSM303DLHC_REG_INT2_CFG        0x34
#define LSM303DLHC_REG_INT2_SOURCE     0x35
#define LSM303DLHC_REG_INT2_THS        0x36
#define LSM303DLHC_REG_INT2_DURATION   0x37
#define LSM303DLHC_REG_CLICK_CFG       0x38
#define LSM303DLHC_REG_CLICK_SRC       0x39
#define LSM303DLHC_REG_CLICK_THS       0x3A
#define LSM303DLHC_REG_TIME_LIMIT      0x3B
#define LSM303DLHC_REG_TIME_LATENCY    0x3C
#define LSM303DLHC_REG_TIME_WINDOW     0x3B

//Temperature
#define LSM303DLHC_REG_TEMP_OUT_H      0x31
#define LSM303DLHC_REG_TEMP_OUT_L      0x32






