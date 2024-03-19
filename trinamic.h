/*
  motors/trinamic.h - Trinamic stepper driver plugin

  Part of grblHAL

  Copyright (c) 2018-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _TRINAMIC_H_
#define _TRINAMIC_H_

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if TRINAMIC_ENABLE

#ifdef TRINAMIC_R_SENSE
#define R_SENSE TRINAMIC_R_SENSE
#endif

#if TRINAMIC_ENABLE == 2130
#ifndef R_SENSE
#define R_SENSE 110
#endif
#include "../trinamic/tmc2130hal.h"
#endif
#if TRINAMIC_ENABLE == 2209
#ifndef R_SENSE
#define R_SENSE 110
#endif
#ifdef TMC_STEALTHCHOP
#undef TMC_STEALTHCHOP
#endif
#define TMC_STEALTHCHOP 0 // not supported
#include "../trinamic/tmc2209hal.h"
#endif
#if TRINAMIC_ENABLE == 2660
#ifndef R_SENSE
#define R_SENSE 50
#endif
#include "../trinamic/tmc2660hal.h"
#endif
#if TRINAMIC_ENABLE == 5160
#ifndef R_SENSE
#define R_SENSE 75
#endif
#include "../trinamic/tmc5160hal.h"
#endif

#ifndef TMC_POLL_STALLED
#if TRINAMIC_I2C
#define TMC_POLL_STALLED            1
#else
#define TMC_POLL_STALLED            0
#endif
#endif

#ifndef PWM_THRESHOLD_VELOCITY
#define PWM_THRESHOLD_VELOCITY      0 // mm/min - 0 to disable, should be set > homing seek rate when enabled (use M913 to set at run time)
#endif

#ifndef TMC_STEALTHCHOP
#define TMC_STEALTHCHOP             1    // 0 = CoolStep, 1 = StealthChop
#endif

#if TRINAMIC_ENABLE == 2209
#define TMC_STALLGUARD              4 // Do not change!
#else
#define TMC_STALLGUARD              2 // Do not change!
#endif

#ifndef TRINAMIC_POLL_STATUS
#define TRINAMIC_POLL_STATUS        0
#endif
#ifndef TRINAMIC_DYNAMIC_CURRENT
#define TRINAMIC_DYNAMIC_CURRENT    0
#endif

//#define TMC_HOMING_ACCELERATION 50.0f // NOT tested... Reduce acceleration during homing to avoid falsely triggering DIAG output

// The following parameters will default to driver specific values.
//

//#define TMC_DRVCONF 0x?    // consult the driver datasheet to determine the value to use.
//#define TMC_COOLCONF_SEMIN   5  // Range: 0 - 15, 0 = CoolStep off
//#define TMC_COOLCONF_SEMAX   2  // Range: 0 - 15
//#define TMC_COOLCONF_SEDN    1  // Range: 0 - 3
//#define TMC_COOLCONF_SEUP    1  // Range: 0 - 3
//#define TMC_COOLCONF_SEIMIN  0  // 0 or 1
//#define TMC_CHOPCONF_HSTRT   1  // Range: 1 - 8
//#define TMC_CHOPCONF_HEND    2  // Range: -3 - 12
//#define TMC_CHOPCONF_TBL     1  // 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
//#define TMC_CHOPCONF_TOFF    5  // Range: 1 - 15
//#define TMC_CHOPCONF_RDNTF   0  // 0 = fixed, 1 = random
//#define TMC_CHOPCONF_CHM     0  // 0 = Spreadcycle, 1 = constant off time
//#define TMC_CHOPCONF_HDEC    0  // TMC2260 only
//#define TMC_CHOPCONF_TFD     0  // Range: 0 - 15
//#define TMC_CHOPCONF_INTPOL  1  // 0 or 1

//

// General
#if TRINAMIC_MIXED_DRIVERS
#define TMC_X_ENABLE 0
#else
#define TMC_X_ENABLE 1 // Do not change
#endif
#define TMC_X_MONITOR 1
#define TMC_X_MICROSTEPS 16
#define TMC_X_R_SENSE R_SENSE // mOhm

#ifndef TMC_X_CURRENT
#define TMC_X_CURRENT DEFAULT_X_CURRENT // mA RMS
#endif

#define TMC_X_HOLD_CURRENT_PCT 50
#define TMC_X_HOMING_SEEK_SGT 22
#define TMC_X_HOMING_FEED_SGT 22
#define TMC_X_STEALTHCHOP TMC_STEALTHCHOP


#if TRINAMIC_MIXED_DRIVERS
#define TMC_Y_ENABLE 0
#else
#define TMC_Y_ENABLE 1 // Do not change
#endif
#define TMC_Y_MONITOR 1
#define TMC_Y_MICROSTEPS 16
#define TMC_Y_R_SENSE R_SENSE // mOhm

#ifndef TMC_Y_CURRENT
#define TMC_Y_CURRENT DEFAULT_Y_CURRENT // mA RMS
#endif

#define TMC_Y_HOLD_CURRENT_PCT 50
#define TMC_Y_HOMING_SEEK_SGT 22
#define TMC_Y_HOMING_FEED_SGT 22
#define TMC_Y_STEALTHCHOP TMC_STEALTHCHOP

#if TRINAMIC_MIXED_DRIVERS
#define TMC_Z_ENABLE 0
#else
#define TMC_Z_ENABLE 1 // Do not change
#endif
#define TMC_Z_MONITOR 1
#define TMC_Z_MICROSTEPS 16
#define TMC_Z_R_SENSE R_SENSE // mOhm

#ifndef TMC_Z_CURRENT
#define TMC_Z_CURRENT DEFAULT_Z_CURRENT // mA RMS
#endif

#define TMC_Z_HOLD_CURRENT_PCT 50
#define TMC_Z_HOMING_SEEK_SGT 22
#define TMC_Z_HOMING_FEED_SGT 22
#define TMC_Z_STEALTHCHOP TMC_STEALTHCHOP

#ifdef A_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_A_ENABLE 0
#else
#define TMC_A_ENABLE 1 // Do not change
#endif
#define TMC_A_MONITOR 1
#define TMC_A_MICROSTEPS 16
#define TMC_A_R_SENSE R_SENSE // mOhm

#ifndef TMC_A_CURRENT
#define TMC_A_CURRENT DEFAULT_A_CURRENT // mA RMS
#endif

#define TMC_A_HOLD_CURRENT_PCT 50
#define TMC_A_HOMING_SEEK_SGT 22
#define TMC_A_HOMING_FEED_SGT 22
#define TMC_A_STEALTHCHOP TMC_STEALTHCHOP

#endif

#ifdef B_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_B_ENABLE 0
#else
#define TMC_B_ENABLE 1 // Do not change
#endif
#define TMC_B_MONITOR 1
#define TMC_B_MICROSTEPS 16
#define TMC_B_R_SENSE R_SENSE // mOhm

#ifndef TMC_B_CURRENT
#define TMC_B_CURRENT DEFAULT_B_CURRENT // mA RMS
#endif

#define TMC_B_HOLD_CURRENT_PCT 50
#define TMC_B_HOMING_SEEK_SGT 22
#define TMC_B_HOMING_FEED_SGT 22
#define TMC_B_STEALTHCHOP TMC_STEALTHCHOP

#endif

#ifdef C_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_C_ENABLE 0
#else
#define TMC_C_ENABLE 1 // Do not change
#endif
#define TMC_C_MONITOR 1
#define TMC_C_MICROSTEPS 16
#define TMC_C_R_SENSE R_SENSE // mOhm

#ifndef TMC_C_CURRENT
#define TMC_C_CURRENT DEFAULT_C_CURRENT // mA RMS
#endif

#define TMC_C_HOLD_CURRENT_PCT 50
#define TMC_C_HOMING_SEEK_SGT 22
#define TMC_C_HOMING_FEED_SGT 22
#define TMC_C_STEALTHCHOP TMC_STEALTHCHOP

#endif

#ifdef U_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_U_ENABLE 0
#else
#define TMC_U_ENABLE 1 // Do not change
#endif
#define TMC_U_MONITOR 1
#define TMC_U_MICROSTEPS 16
#define TMC_U_R_SENSE R_SENSE // mOhm

#ifndef TMC_U_CURRENT
#define TMC_U_CURRENT DEFAULT_U_CURRENT // mA RMS
#endif

#define TMC_U_HOLD_CURRENT_PCT 50
#define TMC_U_HOMING_SEEK_SGT 22
#define TMC_U_HOMING_FEED_SGT 22
#define TMC_U_STEALTHCHOP TMC_STEALTHCHOP

#endif

#ifdef V_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_V_ENABLE 0
#else
#define TMC_V_ENABLE 1 // Do not change
#endif
#define TMC_V_MONITOR 1
#define TMC_V_MICROSTEPS 16
#define TMC_V_R_SENSE R_SENSE // mOhm

#ifndef TMC_V_CURRENT
#define TMC_V_CURRENT DEFAULT_V_CURRENT // mA RMS
#endif

#define TMC_V_HOLD_CURRENT_PCT 50
#define TMC_V_HOMING_SEEK_SGT 22
#define TMC_V_HOMING_FEED_SGT 22
#define TMC_V_STEALTHCHOP TMC_STEALTHCHOP

#endif

//#define TRINAMIC_EXTENDED_SETTINGS 1
//

typedef struct {
    uint16_t current; // mA
    uint8_t hold_current_pct;
    uint16_t r_sense; // mOhm
    uint16_t microsteps;
    trinamic_mode_t mode;
    float homing_seek_rate;
    float homing_feed_rate;
    int16_t homing_seek_sensitivity;
    int16_t homing_feed_sensitivity;
} motor_settings_t;

typedef struct {
    axes_signals_t driver_enable;
    axes_signals_t homing_enable;
    motor_settings_t driver[N_AXIS];
#ifdef TRINAMIC_EXTENDED_SETTINGS
    trinamic_cfg_t cfg_params;
#endif
} trinamic_settings_t;

typedef struct {
    uint8_t address;            // slave address, for UART Single Wire Interface drivers - can be overridden by driver interface
    motor_settings_t *settings; // for info only, do not modify
} trinamic_driver_config_t;

typedef void (*trinamic_on_drivers_init_ptr)(uint8_t n_motors, axes_signals_t enabled);
typedef void (*trinamic_on_driver_preinit_ptr)(motor_map_t motor, trinamic_driver_config_t *config);
typedef void (*trinamic_on_driver_postinit_ptr)(motor_map_t motor, const tmchal_t *driver);

typedef struct {
    trinamic_on_drivers_init_ptr on_drivers_init;
    trinamic_on_driver_preinit_ptr on_driver_preinit;
    trinamic_on_driver_postinit_ptr on_driver_postinit;
} trinamic_driver_if_t;

bool trinamic_init (void);
void trinamic_fault_handler (void);
void trinamic_warn_handler (void);
void trinamic_if_init (trinamic_driver_if_t *driver);

#endif // TRINAMIC_ENABLE

#endif
