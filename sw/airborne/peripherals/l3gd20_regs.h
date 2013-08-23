/*
 * Copyright (C) 2013 Eduardo Lavratti  <agressiva@hotmail.com>
 * Modified by The Arcoslab Team 2013
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
 * @file peripherals/l3gd20_regs.h
 * Register defs for L3GD20 gyros.
 */

#ifndef L3GD20_REGS_H
#define L3GD20_REGS_H

/* default I2C address */
#define L3GD20_ADDR            		0xD4
#define L3GD20_ADDR_ALT        		0xD6

/*Reserved*/
/*0x00-0x0E*/

/* Registers */

#define L3GD20_RNW 			(1 << 7) /* Write when zero */
#define L3GD20_MNS			(1 << 6) /* Multiple reads when 1 */

/*Reserved 0x00-0x0E*/

#define L3GD20_WHO_AM_I 		0x0F

/*Reserved 0x10-0x1F*/

#define L3GD20_REG_CTRL_REG1 		0x20
#define L3GD20_REG_CTRL_REG1_PD 	(1 << 3)
#define L3GD20_REG_CTRL_REG1_XEN 	(1 << 1)
#define L3GD20_REG_CTRL_REG1_YEN 	(1 << 0)
#define L3GD20_REG_CTRL_REG1_ZEN 	(1 << 2)
#define L3GD20_REG_CTRL_REG1_BW_SHIFT 	4
#define L3GD20_REG_CTRL_REG2 		0x21
#define L3GD20_REG_CTRL_REG3 		0x22
#define L3GD20_REG_CTRL_REG4 		0x23
#define L3GD20_REG_CTRL_REG4_FS_SHIFT 	4
#define L3GD20_REG_CTRL_REG5 		0x24
#define L3GD20_REG_REFERENCE 		0x25
#define L3GD20_REG_OUT_TEMP 		0x26
#define L3GD20_REG_STATUS_REG 		0x27

#define L3GD20_REG_OUT_X_L 		0x28
#define L3GD20_REG_OUT_X_H 		0x29
#define L3GD20_REG_OUT_Y_L 		0x2A
#define L3GD20_REG_OUT_Y_H 		0x2B
#define L3GD20_REG_OUT_Z_L 		0x2C
#define L3GD20_REG_OUT_Z_H 		0x2D

#define L3GD20_REG_FIFO_CTRL_REG 	0x2E
#define L3GD20_REG_FIFO_SRC_REG 	0x2F

#define L3GD20_REG_INT1_CFG 		0x30
#define L3GD20_REG_INT1_SRC 		0x31
#define L3GD20_REG_INT1_THS_XH 		0x32
#define L3GD20_REG_INT1_THS_XL 		0x33
#define L3GD20_REG_INT1_THS_YH 		0x34
#define L3GD20_REG_INT1_THS_YL 		0x35
#define L3GD20_REG_INT1_THS_ZH 		0x36
#define L3GD20_REG_INT1_THS_ZL 		0x37
#define L3GD20_REG_INT1_DURATION 	0x38

/** Output Data Rate Options */
enum L3GD20_DR {
  L3GD20_DR_100Hz = 0x0,
  L3GD20_DR_200Hz = 0x1,
  L3GD20_DR_400Hz = 0x2,
  L3GD20_DR_800Hz = 0x3
};
/** Digital Low Pass Filter Options */
enum L3GD20_DLPF {
  L3GD20_DLPF_1 = 0x0,
  L3GD20_DLPF_2 = 0x1,
  L3GD20_DLPF_3 = 0x2,
  L3GD20_DLPF_4 = 0x3
};

#endif /* L3GD20_REGS_H */
