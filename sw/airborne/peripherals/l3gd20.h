/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
 *               2013 Eduardo Lavratti <agressiva@hotmail.com>
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
 *
 */

/**
 * @file peripherals/l3gd20.h
 *
 * Driver for the gyro L3GD20 From ST.
 */
#ifndef L3GD20_H
#define L3GD20_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Address and register definitions */
#include "peripherals/l3gd20_regs.h"


/// Default Output rate 800hz
#define L3GD20_DEFAULT_DR L3GD20_DR_800Hz
/// Default digital lowpass filter 35hz
#define L3GD20_DEFAULT_DLPF L3GD20_DLPF_2


/* Default conf */
#define L3GD20_DEFAULT_CTRL_REG1 0x8f // 400hz ODR, 20hz filter, run!
#define L3GD20_DEFAULT_CTRL_REG5 0x02 // low pass filter enable

struct L3gd20Config {
  uint8_t ctrl_reg1;     ///<
  uint8_t ctrl_reg5;     ///<
};

/** config status states */
enum L3gd20ConfStatus {
  L3G_CONF_UNINIT,
  L3G_CONF_REG1,
  L3G_CONF_REG5,
  L3G_CONF_DONE
};

struct L3gd20 {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  bool_t initialized;                 ///< config done flag
  enum L3gd20ConfStatus init_status; ///< init status
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int32Rates rates;          ///< data as angular rates in gyro coordinate system
    int32_t value[3];                 ///< data values accessible by channel index
  } data;
  struct L3gd20Config config;
};

// Functions
extern void l3gd20_init(struct L3gd20 *itg, struct spi_periph *spi_p, uint8_t spi_address);
extern void l3gd20_set_default_config(struct L3gd20Config *conf);
extern void l3gd20_start_configure(struct L3gd20 *l3g);
extern void l3gd20_read(struct L3gd20 *l3g);
extern void l3gd20_event(struct L3gd20 *l3g);

/// convenience function: read or start configuration if not already initialized
static inline void l3gd20_periodic(struct L3gd20 *l3g) {
  if (l3g->initialized)
    l3gd20_read(l3g);
  else
    l3gd20_start_configure(l3g);
}

#endif // /*L3GD20_H*/
