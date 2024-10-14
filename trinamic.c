/*
  motors/trinamic.c - Trinamic stepper driver plugin

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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if TRINAMIC_ENABLE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "trinamic.h"
#if TRINAMIC_I2C
#include "../trinamic/tmc_i2c_interface.h"
#endif

#include "../grbl/nvs_buffer.h"
#include "../grbl/protocol.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#include "../grbl/platform.h"

#ifndef TRINAMIC_POLL_INTERVAL
#define TRINAMIC_POLL_INTERVAL 250
#endif

static bool warning = false, is_homing = false, settings_loaded = false;
static volatile uint_fast16_t diag1_poll = 0;
static char sbuf[65]; // string buffer for reports
static char min_current[5], max_current[5];
static uint_fast8_t n_motors = 0;
static const tmchal_t *stepper[TMC_N_MOTORS_MAX];
static motor_map_t *motor_map;
static axes_signals_t homing = {0}, otpw_triggered = {0}, driver_enabled = {0};
#if TMC_POLL_STALLED
static limits_get_state_ptr limits_get_state = NULL;
#endif
static limits_enable_ptr limits_enable = NULL;
static stepper_pulse_start_ptr hal_stepper_pulse_start = NULL;
static nvs_address_t nvs_address;
static on_realtime_report_ptr on_realtime_report;
static on_report_options_ptr on_report_options;
static driver_setup_ptr driver_setup;
static settings_changed_ptr settings_changed;
static user_mcode_ptrs_t user_mcode;
static trinamic_driver_if_t driver_if = {0};
static trinamic_settings_t trinamic;
static trinamic_cfg_t cfg_cap;
#ifdef TRINAMIC_EXTENDED_SETTINGS
static trinamic_cfg_t *cfg_params = &trinamic.cfg_params;
#else
static trinamic_cfg_t params, *cfg_params = &params;
#endif
#if TRINAMIC_POLL_STATUS
static TMC_drv_status_t status[TMC_N_MOTORS_MAX];
#endif
#if TRINAMIC_DYNAMIC_CURRENT
static uint16_t dynamic_current[TMC_N_MOTORS_MAX], reduced_current[TMC_N_MOTORS_MAX];
static stepper_pulse_start_ptr stepper_block_start = NULL;
#endif

static struct {
    bool raw;
    bool sg_status_enable;
    volatile bool sg_status;
    bool sfilt;
    uint32_t sg_status_motor;
    axes_signals_t sg_status_motormask;
    uint32_t msteps;
} report = {0};

#if TRINAMIC_I2C

static stepper_enable_ptr stepper_enable = NULL;

TMCI2C_enable_dgr_t dgr_enable = {
    .addr.value = TMC_I2CReg_ENABLE
};

TMCI2C_monitor_status_dgr_t dgr_monitor = {
    .addr.value = TMC_I2CReg_MON_STATE
};
#endif

static void write_debug_report (uint_fast8_t axes);

// Wrapper for initializing physical interface
void trinamic_if_init (trinamic_driver_if_t *driver)
{
    memcpy(&driver_if, driver, sizeof(trinamic_driver_if_t));
}

#if TRINAMIC_POLL_STATUS

static void trinamic_status_report (void *data)
{
    uint_fast8_t motor = 0;
    uint_fast16_t axis;

    for(motor = 0; motor < n_motors; motor++) {
        if(stepper[motor]) {
            if(status[motor].stst) {
                axis = stepper[motor]->get_config(motor)->motor.axis;
                hal.stream.write("[MOST:");
                hal.stream.write(uitoa(axis));
                //hal.stream.write("]" ASCII_EOL);
                hal.stream.write("[CURR:");
                hal.stream.write(uitoa((trinamic.driver[axis].current)));
                //hal.stream.write("]" ASCII_EOL);
                //hal.delay_ms(15, NULL);
#if TRINAMIC_DYNAMIC_CURRENT
                hal.stream.write("[STCR:");
                hal.stream.write(uitoa(reduced_current[motor]));
#endif
                hal.stream.write("]" ASCII_EOL);
                //hal.delay_ms(15, NULL);
            }

            /*if(status[motor].stallguard){
                strcpy(sbuf, "SG:M ");
                strcat(sbuf, uitoa(motor));
                report_message(sbuf, Message_Warning);
            }*/

            if(status[motor].otpw){
                strcpy(sbuf, "Over-Temperature Motor: ");
                strcat(sbuf, uitoa(motor));
                report_message(sbuf, Message_Warning);
            }
        }
    }
}

static void trinamic_poll_status (void *data)
{
    static bool error_active = false, error_hold = false;
    static uint32_t error_count = 0;
    static hold_state_t holding_state = Hold_NotHolding;

    uint_fast8_t motor = n_motors;
    uint8_t stall_fault = 0, otpw_fault = 0;
    sys_state_t current_state = state_get();

    task_add_delayed(trinamic_poll_status, NULL, TRINAMIC_POLL_INTERVAL);

    do {
        if(stepper[--motor]) {

            if(current_state == STATE_IDLE)
                status[motor] = stepper[motor]->get_drv_status(motor);
            else if(current_state == STATE_HOLD && holding_state == Hold_Pending && sys.holding_state == Hold_Complete) {
                holding_state == Hold_Complete; // potentially dangerous if spindle is running and is lowering itself due to gravity pull,
                st_go_idle();                   // so disabled for now (see below).
            }

            if(status[motor].otpw) // overtemp?
                otpw_fault |= (1 << motor);
/*            if(status[motor].stallguard) // stalled?
                stall_fault |= (1 << motor); */

#if TRINAMIC_DYNAMIC_CURRENT
            uint_fast8_t axis = motor_map[motor].axis;
            if((current_state == STATE_IDLE || (current_state & (STATE_ALARM|STATE_HOLD))) && dynamic_current[motor] == trinamic.driver[axis].current)
                stepper[motor]->set_current(motor, (dynamic_current[motor] = reduced_current[motor]), trinamic.driver[axis].hold_current_pct);
#endif
        }
    } while(motor);

    if(stall_fault || otpw_fault) {
        if(++error_count > 2 && !error_active) {
            error_active = true;
            if(current_state & (STATE_CYCLE|STATE_HOMING|STATE_JOG)) {
                // holding_state == Hold_Pending;
                grbl.enqueue_realtime_command(CMD_FEED_HOLD);
            }
            task_add_immediate(trinamic_status_report, NULL);
        }
    } else if(error_active) {
        error_active = false;
        error_count = 0;
        if(holding_state != Hold_NotHolding) {
            holding_state = Hold_NotHolding;
            st_wake_up();
        }
    }
}

#endif // TRINAMIC_POLL_STATUS

#if TRINAMIC_DYNAMIC_CURRENT

static void set_current_for_block (void *block)
{
    uint_fast16_t axis = 0;
    uint_fast8_t motor = n_motors;

    do {
        if(stepper[--motor]) {
            axis = motor_map[motor].axis;
            if((((stepper_t *)block)->steps[axis] ? trinamic.driver[axis].current : reduced_current[motor]) != dynamic_current[motor]) {
                dynamic_current[motor] = ((stepper_t *)block)->steps[axis] ? trinamic.driver[axis].current : reduced_current[motor];
                stepper[motor]->set_current(motor, dynamic_current[motor], trinamic.driver[axis].hold_current_pct);
            }
#if TRINAMIC_POLL_STATUS
            status[motor] = stepper[motor]->get_drv_status(motor);
#endif
        }
    } while(motor);
}

static void set_current_for_homing (void)
{
    uint_fast16_t axis = 0;
    uint_fast8_t motor = n_motors;

    do {
        axis = motor_map[--motor].axis;
        if(stepper[motor] && dynamic_current[motor] != trinamic.driver[axis].current)
            stepper[motor]->set_current(motor, (dynamic_current[motor] = trinamic.driver[axis].current), trinamic.driver[axis].hold_current_pct);
    } while(motor);
}

static void dynamic_current_pulse_start (stepper_t *stepper)
{
    if(stepper->new_block && !is_homing)
        task_add_immediate(set_current_for_block, stepper);

    stepper_block_start(stepper);
}

#endif // TRINAMIC_DYNAMIC_CURRENT

static bool trinamic_driver_config (motor_map_t motor, uint8_t seq)
{
    bool ok = false;
    trinamic_driver_config_t cfg = {
        .address = motor.id,
        .settings = &trinamic.driver[motor.axis]
    };

    if(driver_if.on_driver_preinit)
        driver_if.on_driver_preinit(motor, &cfg);

    #if TRINAMIC_ENABLE == 2209
        ok = (stepper[motor.id] = TMC2209_AddMotor(motor, cfg.address, cfg.settings->current, cfg.settings->microsteps, cfg.settings->r_sense)) != NULL;
    #elif TRINAMIC_ENABLE == 2660
        uint8_t retries = 25;
        do {
            if(!(ok = (stepper[motor.id] = TMC2660_AddMotor(motor, cfg.settings->current, cfg.settings->microsteps, cfg.settings->r_sense)) != NULL))
                hal.delay_ms(10, NULL);
        } while(!ok && --retries);

    #elif TRINAMIC_ENABLE == 2130
        ok = (stepper[motor.id] = TMC2130_AddMotor(motor, cfg.settings->current, cfg.settings->microsteps, cfg.settings->r_sense)) != NULL;
    #elif TRINAMIC_ENABLE == 5160
        ok = (stepper[motor.id] = TMC5160_AddMotor(motor, cfg.settings->current, cfg.settings->microsteps, cfg.settings->r_sense)) != NULL;
    #endif

    if(!ok) {
        protocol_enqueue_foreground_task(report_warning, "Could not communicate with stepper driver!");
    //    system_raise_alarm(Alarm_SelftestFailed);
        return false;
    }

    stepper[motor.id]->get_config(motor.id)->motor.seq = seq; //

    driver_enabled.mask |= bit(motor.axis);

    switch(motor.axis) {

        case X_AXIS:
          #if TRINAMIC_I2C && TMC_X_MONITOR
            dgr_enable.reg.monitor.x = TMC_X_MONITOR;
          #endif
            break;

        case Y_AXIS:
          #if TRINAMIC_I2C && TMC_Y_MONITOR
            dgr_enable.reg.monitor.y = TMC_Y_MONITOR;
          #endif
            break;

        case Z_AXIS:
          #if TRINAMIC_I2C && TMC_Z_MONITOR
            dgr_enable.reg.monitor.z = TMC_Z_MONITOR;
          #endif
            break;

#ifdef A_AXIS
        case A_AXIS:
          #if TRINAMIC_I2C && TMC_A_MONITOR
            dgr_enable.reg.monitor.a = TMC_A_MONITOR;
          #endif
            break;
#endif

#ifdef B_AXIS
        case B_AXIS:

          #if TRINAMIC_I2C && TMC_B_MONITOR
            dgr_enable.reg.monitor.b = TMC_B_MONITOR;
          #endif
            break;
#endif

#ifdef C_AXIS
        case C_AXIS:
          #if TRINAMIC_I2C && TMC_C_MONITOR
            dgr_enable.reg.monitor.c = TMC_C_MONITOR;
          #endif
            break;
#endif

#ifdef U_AXIS
        case U_AXIS:
          #if TRINAMIC_I2C && TMC_U_MONITOR
            dgr_enable.reg.monitor.u = TMC_U_MONITOR;
          #endif
            break;
#endif

#ifdef V_AXIS
        case V_AXIS:
          #if TRINAMIC_I2C && TMC_V_MONITOR
            dgr_enable.reg.monitor.v = TMC_V_MONITOR;
          #endif
            break;
#endif
    }

    stepper[motor.id]->sg_filter(motor.id, 1);
    stepper[motor.id]->coolconf(motor.id, cfg_params->coolconf);
    stepper[motor.id]->chopper_timing(motor.id, cfg_params->chopconf);

    if(stepper[motor.id]->stealthChop)
        stepper[motor.id]->stealthChop(motor.id, cfg.settings->mode == TMCMode_StealthChop);
    else if(cfg.settings->mode == TMCMode_StealthChop)
        cfg.settings->mode = TMCMode_CoolStep;

#if PWM_THRESHOLD_VELOCITY > 0
    stepper[motor.id]->set_tpwmthrs(motor.id, (float)PWM_THRESHOLD_VELOCITY / 60.0f, cfg.settings->steps_per_mm);
#endif
    stepper[motor.id]->set_current(motor.id, cfg.settings->current, cfg.settings->hold_current_pct);
    stepper[motor.id]->set_microsteps(motor.id, cfg.settings->microsteps);
#if TRINAMIC_I2C
    tmc_spi_write((trinamic_motor_t){0}, (TMC_spi_datagram_t *)&dgr_enable);
#endif

#if TRINAMIC_DYNAMIC_CURRENT
    reduced_current[motor.id] = cfg.settings->current * cfg.settings->hold_current_pct / 100;;
#endif

    if(driver_if.on_driver_postinit)
        driver_if.on_driver_postinit(motor, stepper[motor.id]);

    return true;
}

#if 1 // Region settings

static void trinamic_drivers_init (axes_signals_t axes)
{
    bool ok = axes.value != 0;
    uint_fast8_t motor = n_motors, n_enabled = 0, seq = 0;

    memset(stepper, 0, sizeof(stepper));

    do {
        if(bit_istrue(axes.mask, bit(motor_map[--motor].axis)))
            seq++;
    } while(motor);

    motor = n_motors;
    *min_current = '\0';
    do {
        if(bit_istrue(axes.mask, bit(motor_map[--motor].axis))) {
            if((ok = trinamic_driver_config(motor_map[motor], --seq))) {
                n_enabled++;
                if(*min_current == '\0') {
                    strcpy(min_current, uitoa(stepper[motor_map[motor].id]->get_current(motor_map[motor].id, TMCCurrent_Min)));
                    strcpy(max_current, uitoa(stepper[motor_map[motor].id]->get_current(motor_map[motor].id, TMCCurrent_Max)));
                }
            }
        }
    } while(ok && motor);

    tmc_motors_set(ok ? n_enabled : 0);

    if(!ok) {
        driver_enabled.mask = 0;
        memset(stepper, 0, sizeof(stepper));
    }
#if TRINAMIC_POLL_STATUS || TRINAMIC_DYNAMIC_CURRENT
    else {
#if TRINAMIC_POLL_STATUS
        task_add_delayed(trinamic_poll_status, NULL, TRINAMIC_POLL_INTERVAL);
#endif
#if TRINAMIC_DYNAMIC_CURRENT
        if(stepper_block_start == NULL) {
            stepper_block_start = hal.stepper.pulse_start;
            hal.stepper.pulse_start = dynamic_current_pulse_start;
        }
#endif
    }
#endif
}

static void trinamic_drivers_setup (void)
{
    if(driver_if.on_drivers_init) {

        uint8_t n_enabled = 0, motor = n_motors;

        do {
            if(bit_istrue(trinamic.driver_enable.mask, bit(motor_map[--motor].axis)))
                n_enabled++;
        } while(motor);

        driver_if.on_drivers_init(n_enabled, trinamic.driver_enable);
    }

    trinamic_drivers_init(trinamic.driver_enable);
}

#if TRINAMIC_MIXED_DRIVERS

static status_code_t set_driver_enable (setting_id_t id, uint_fast16_t value)
{
    if(trinamic.driver_enable.mask != (uint8_t)value) {

        driver_enabled.mask = 0;
        trinamic.driver_enable.mask = (uint8_t)value;

        trinamic_drivers_setup();
    }

    return Status_OK;
}

static uint32_t get_driver_enable (setting_id_t setting)
{
    return trinamic.driver_enable.mask;
}

#endif

// Parse and set driver specific parameters
static status_code_t set_axis_setting (setting_id_t setting, uint_fast16_t value)
{
    uint_fast8_t axis, motor = n_motors;
    status_code_t status = Status_OK;

    switch(settings_get_axis_base(setting, &axis)) {

        case Setting_AxisStepperCurrent:
            trinamic.driver[axis].current = (uint16_t)value;
            do {
                motor--;
                if(stepper[motor] && stepper[motor]->get_config(motor)->motor.axis == axis) {
                    stepper[motor]->set_current(motor, trinamic.driver[axis].current, trinamic.driver[axis].hold_current_pct);
#if TRINAMIC_DYNAMIC_CURRENT
                    reduced_current[motor] = trinamic.driver[axis].current * trinamic.driver[axis].hold_current_pct / 100;
#endif
                }
            } while(motor);
            break;

        case Setting_AxisExtended1: // Hold current percentage
            if(value > 100)
                value = 100;
            trinamic.driver[axis].hold_current_pct = (uint16_t)value;
            do {
                motor--;
                if(stepper[motor] && stepper[motor]->get_config(motor)->motor.axis == axis) {
                    stepper[motor]->set_current(motor, trinamic.driver[axis].current, trinamic.driver[axis].hold_current_pct);
#if TRINAMIC_DYNAMIC_CURRENT
                    reduced_current[motor] = trinamic.driver[axis].current * trinamic.driver[axis].hold_current_pct / 100;
#endif
                }
            } while(motor);
            break;

        case Setting_AxisMicroSteps:
            do {
                motor--;
                if(stepper[motor] && stepper[motor]->get_config(motor)->motor.axis == axis) {
                    if(stepper[motor]->microsteps_isvalid(motor, (uint16_t)value)) {
                        trinamic.driver[axis].microsteps = value;
                        stepper[motor]->set_microsteps(motor, trinamic.driver[axis].microsteps);
                        if(report.sg_status_motormask.mask & bit(axis))
                            report.msteps = trinamic.driver[axis].microsteps;
                    } else {
                        status = Status_InvalidStatement;
                        break;
                    }
                }
            } while(motor);
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static uint32_t get_axis_setting (setting_id_t setting)
{
    uint32_t value = 0;
    uint_fast8_t idx;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisStepperCurrent:
            value = trinamic.driver[idx].current;
            break;

        case Setting_AxisExtended1: // Hold current percentage
            value = trinamic.driver[idx].hold_current_pct;
            break;

        case Setting_AxisMicroSteps:
            value = trinamic.driver[idx].microsteps;
            break;

        default: // for stopping compiler warning
            break;
    }

    return value;
}

// Parse and set driver specific parameters
static status_code_t set_axis_setting_float (setting_id_t setting, float value)
{
    status_code_t status = Status_OK;

    uint_fast8_t idx;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisHomingFeedRate:
            trinamic.driver[idx].homing_feed_rate = value;
            break;

        case Setting_AxisHomingSeekRate:
            trinamic.driver[idx].homing_seek_rate = value;
            break;

        case Setting_AxisExtended0:
            trinamic.driver[idx].homing_seek_sensitivity = (int16_t)value;
            break;

        case Setting_AxisExtended2:
            trinamic.driver[idx].homing_feed_sensitivity = (int16_t)value;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static float get_axis_setting_float (setting_id_t setting)
{
    float value = 0.0f;

    uint_fast8_t idx;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisHomingFeedRate:
            value = trinamic.driver[idx].homing_feed_rate;
            break;

        case Setting_AxisHomingSeekRate:
            value = trinamic.driver[idx].homing_seek_rate;
            break;

        case Setting_AxisExtended0:
            value = (float)trinamic.driver[idx].homing_seek_sensitivity;
            break;

        case Setting_AxisExtended2:
            value = (float)trinamic.driver[idx].homing_feed_sensitivity;
            break;

        default: // for stopping compiler warning
            break;
    }

    return value;
}

#ifdef TRINAMIC_EXTENDED_SETTINGS

static status_code_t set_extended (setting_id_t id, uint_fast16_t value)
{
    switch(id) {

        case Setting_Stepper1:
            trinamic.cfg_params.chopconf.toff = value;
            break;

        case Setting_Stepper2:
            trinamic.cfg_params.chopconf.tbl = value;
            break;

        case Setting_Stepper3:
            trinamic.cfg_params.chopconf.chm = value;
            break;

        case Setting_Stepper4:
            trinamic.cfg_params.chopconf.hstrt = value - 1;
            break;

        case Setting_Stepper6:
            trinamic.cfg_params.chopconf.hdec = value;
            break;

        case Setting_Stepper7:
            trinamic.cfg_params.chopconf.rndtf = value;
            break;

        case Setting_Stepper8:
            trinamic.cfg_params.chopconf.tbl = value; //
            break;

        case Setting_Stepper9:
            trinamic.cfg_params.coolconf.semin = value;
            break;

        case Setting_Stepper10:
            trinamic.cfg_params.coolconf.seup = value;
            break;

        case Setting_Stepper11:
            trinamic.cfg_params.coolconf.semax = value;
            break;

        case Setting_Stepper12:
            trinamic.cfg_params.coolconf.seimin = value;
            break;

        case Setting_Stepper13:
            trinamic.cfg_params.chopconf.tbl = value;
            break;

        default:
            break;
    }

    uint_fast8_t motor = n_motors;
    do {
        if(stepper[--motor]) {
            stepper[motor]->coolconf(motor, cfg_params->coolconf);
            stepper[motor]->chopper_timing(motor, cfg_params->chopconf);
        }
    } while(motor);

    return Status_OK;
}

static uint32_t get_extended (setting_id_t setting)
{
    uint32_t value;

    switch(setting) {

        case Setting_Stepper1:
            value = trinamic.cfg_params.chopconf.toff;
            break;

        case Setting_Stepper2:
            value = trinamic.cfg_params.chopconf.tbl;
            break;

        case Setting_Stepper3:
            value = trinamic.cfg_params.chopconf.chm;
            break;

        case Setting_Stepper4:
            value = trinamic.cfg_params.chopconf.hstrt + 1;
            break;

        case Setting_Stepper6:
            value = trinamic.cfg_params.chopconf.hdec;
            break;

        case Setting_Stepper7:
            value = trinamic.cfg_params.chopconf.rndtf;
            break;

        case Setting_Stepper8:
            value = trinamic.cfg_params.chopconf.tbl; //
            break;

        case Setting_Stepper9:
            value = trinamic.cfg_params.coolconf.semin;
            break;

        case Setting_Stepper10:
            value = trinamic.cfg_params.coolconf.seup;
            break;

        case Setting_Stepper11:
            value = trinamic.cfg_params.coolconf.semax;
            break;

        case Setting_Stepper12:
            value = trinamic.cfg_params.coolconf.seimin;
            break;

        case Setting_Stepper13:
            value = trinamic.cfg_params.chopconf.tbl;
            break;

        default:
            break;
    }

    return value;
}

static status_code_t set_extended_float (setting_id_t id, float value)
{
    switch(id) {

        case Setting_Stepper5:
            if(isintf(value))
                trinamic.cfg_params.chopconf.hend = (int8_t)(value) + 3;
            else
                return Status_BadNumberFormat;
            break;

        default:
            break;
    }

    uint_fast8_t motor = n_motors;
    do {
        if(stepper[--motor])
            stepper[motor]->chopper_timing(motor, cfg_params->chopconf);
    } while(motor);

    return Status_OK;
}

static float get_extended_float (setting_id_t setting)
{
    uint32_t value;

    switch(setting) {

        case Setting_Stepper5:
            value = (float)trinamic.cfg_params.chopconf.hend - 3.0f;
            break;

        default:
            break;
    }

    return value;
}

static bool is_extended_available (const setting_detail_t *setting)
{
    bool ok = false;

    switch(setting->id) {

        case Setting_Stepper1:
            ok = !!cfg_cap.chopconf.toff;
            break;

        case Setting_Stepper2:
            ok = !!cfg_cap.chopconf.tbl;
            break;

        case Setting_Stepper3:
            ok = !!cfg_cap.chopconf.chm;
            break;

        case Setting_Stepper4:
            ok = !!cfg_cap.chopconf.hstrt;
            break;

        case Setting_Stepper5:
            ok = !!cfg_cap.chopconf.hend;
            break;

        case Setting_Stepper6:
            ok = !!cfg_cap.chopconf.hdec;
            break;

        case Setting_Stepper7:
            ok = !!cfg_cap.chopconf.rndtf;
            break;

        case Setting_Stepper8:
            ok = !!cfg_cap.chopconf.tbl; //
            break;

        case Setting_Stepper9:
            ok = !!cfg_cap.coolconf.semin;
            break;

        case Setting_Stepper10:
            ok = !!cfg_cap.coolconf.seup;
            break;

        case Setting_Stepper11:
            ok = !!cfg_cap.coolconf.semax;
            break;

        case Setting_Stepper12:
            ok = !!cfg_cap.coolconf.seimin;
            break;

        case Setting_Stepper13:
            ok = !!cfg_cap.chopconf.tbl;
            break;

        default:
            break;
    }

    return ok;
}

#endif

#define AXIS_OPTS { .subgroups = On, .increment = 1 }

static const setting_detail_t trinamic_settings[] = {
#if TRINAMIC_MIXED_DRIVERS
    { Setting_TrinamicDriver, Group_MotorDriver, "Trinamic driver", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_NonCoreFn, set_driver_enable, get_driver_enable, NULL },
#endif
    { Setting_TrinamicHoming, Group_MotorDriver, "Sensorless homing", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_NonCore, &trinamic.homing_enable.mask, NULL, NULL },
    { Setting_AxisStepperCurrent, Group_Axis0, "-axis motor current", "mA", Format_Integer, "###0", min_current, max_current, Setting_NonCoreFn, set_axis_setting, get_axis_setting, NULL, AXIS_OPTS },
    { Setting_AxisMicroSteps, Group_Axis0, "-axis microsteps", "steps", Format_Integer, "###0", NULL, NULL, Setting_NonCoreFn, set_axis_setting, get_axis_setting, NULL, AXIS_OPTS },
    { Setting_AxisHomingFeedRate, Group_Axis0, "-axis homing locate feed rate", "mm/min", Format_Decimal, "###0", NULL, NULL, Setting_NonCoreFn, set_axis_setting_float, get_axis_setting_float, NULL, AXIS_OPTS },
    { Setting_AxisHomingSeekRate, Group_Axis0, "-axis homing search seek rate", "mm/min", Format_Decimal, "###0", NULL, NULL, Setting_NonCoreFn, set_axis_setting_float, get_axis_setting_float, NULL, AXIS_OPTS },
#if TMC_STALLGUARD == 4
    { Setting_AxisExtended0, Group_Axis0, "-axis StallGuard4 fast threshold", NULL, Format_Decimal, "##0", "0", "255", Setting_NonCoreFn, set_axis_setting_float, get_axis_setting_float, NULL, AXIS_OPTS },
#else
    { Setting_AxisExtended0, Group_Axis0, "-axis StallGuard2 fast threshold", NULL, Format_Decimal, "-##0", "-64", "63", Setting_NonCoreFn, set_axis_setting_float, get_axis_setting_float, NULL, AXIS_OPTS },
#endif
    { Setting_AxisExtended1, Group_Axis0, "-axis hold current", "%", Format_Int8, "##0", "5", "100", Setting_NonCoreFn, set_axis_setting, get_axis_setting, NULL, AXIS_OPTS },
#if TMC_STALLGUARD == 4
    { Setting_AxisExtended2, Group_Axis0, "-axis StallGuard4 slow threshold", NULL, Format_Decimal, "##0", "0", "255", Setting_NonCoreFn, set_axis_setting_float, get_axis_setting_float, NULL, AXIS_OPTS },
#else
    { Setting_AxisExtended2, Group_Axis0, "-axis stallGuard2 slow threshold", NULL, Format_Decimal, "-##0", "-64", "63", Setting_NonCoreFn, set_axis_setting_float, get_axis_setting_float, NULL, AXIS_OPTS },
#endif
#ifdef TRINAMIC_EXTENDED_SETTINGS
    { Setting_Stepper1, Group_MotorDriver, "Chopper toff", NULL, Format_Int8, "#0", "1", "15", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper2, Group_MotorDriver, "Chopper tbl", NULL, Format_Int8, "0", "0", "3", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper3, Group_MotorDriver, "Chopper mode", NULL, Format_RadioButtons, "Spreadcycle,Constant toff", NULL, NULL, Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper4, Group_MotorDriver, "Chopper hstrt", NULL, Format_Int8, "0", "1", "8", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper5, Group_MotorDriver, "Chopper hend", NULL, Format_Decimal, "-#0", "-3", "12", Setting_NonCoreFn, &set_extended_float, &get_extended_float, is_extended_available },
    { Setting_Stepper6, Group_MotorDriver, "Chopper hdec", NULL, Format_Int8, "0", "0", "3", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper7, Group_MotorDriver, "Chopper random TOFF", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
//??    { Setting_Stepper8, Group_MotorDriver, "THRESH", NULL, Format_Int8, "###0", NULL, NULL, Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available }, // == Setting_AxisExtended0 & Setting_AxisExtended1?
    { Setting_Stepper9, Group_MotorDriver, "CoolStep semin", NULL, Format_Int8, "#0", "0", "15", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper10, Group_MotorDriver, "CoolStep seup", NULL, Format_Int8, "0", "0", "3", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper11, Group_MotorDriver, "CoolStep semax", NULL, Format_Int8, "#0", "0", "15", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper12, Group_MotorDriver, "CoolStep sedn", NULL, Format_Int8, "0", "0", "3", Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper13, Group_MotorDriver, "CoolStep seimin", NULL, Format_RadioButtons, "0.5 x CS,.25 x CS", NULL, NULL, Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
    { Setting_Stepper14, Group_MotorDriver, "drvconf_reg", NULL, Format_Int8, "###0", NULL, NULL, Setting_NonCoreFn, &set_extended, &get_extended, is_extended_available },
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t trinamic_settings_descr[] = {
#if TRINAMIC_MIXED_DRIVERS
    { Setting_TrinamicDriver, "Enable SPI or UART controlled Trinamic drivers for axes." },
#endif
    { Setting_TrinamicHoming, "Enable sensorless homing for axes. Requires SPI or UART controlled Trinamic drivers." },
    { Setting_AxisStepperCurrent, "Motor current in mA (RMS)." },
    { Setting_AxisMicroSteps, "Microsteps per fullstep." },
    { Setting_AxisExtended0, "StallGuard threshold for fast (seek) homing phase." },
    { Setting_AxisExtended1, "Motor current at standstill as a percentage of full current.\\n"
                             "NOTE: if grblHAL is configured to disable motors on standstill this setting has no use."
    },
    { Setting_AxisExtended2, "StallGuard threshold for slow (feed) homing phase." },
    { Setting_AxisHomingFeedRate, "Feed rate to slowly engage limit switch to determine its location accurately.\\n"
                                  "NOTE: only used for axes with Trinamic driver enabled, others use the $24 setting."
    },
    { Setting_AxisHomingSeekRate, "Seek rate to quickly find the limit switch before the slower locating phase.\\n"
                                  "NOTE: only used for axes with Trinamic driver enabled, others use the $25 setting."
    },
#ifdef TRINAMIC_EXTENDED_SETTINGS
    { Setting_Stepper1, "Off time. Duration of slow decay phase as a multiple of system clock periods: NCLK= 24 + (32 x TOFF). This will limit the maximum chopper frequency (0-15).\\n"
                         "0: MOSFETs shut off, driver disabled.\\n"
                         "1: Use with TBL of minimum 24 clocks." },
    { Setting_Stepper2, "Blanking time interval in system clock periods (0-3 = 16,24,36,54). Needs to cover the switching event and the duration of the ringing on the sense resistor." },
    { Setting_Stepper3, "Chopper mode. Affects HDEC, HEND, and HSTRT parameters.\\n"
                         "0: Standard mode (SpreadCycle).\\n"
                         "1: Constant TOFF with fast decay time. Fast decay is after on time. Fast decay time is also terminated when the negative nominal current is reached." },
    { Setting_Stepper4, "CHM=0: Hysteresis start, offset from HEND (1-8). To be effective, HEND+HSTRT must be ≤15.\\n"
                        "CHM=1: Fast decay time. Three least-significant bits of the duration of the fast decay phase. The MSB is HDEC0. Fast decay time is a multiple of system clock periods: NCLK= 32 x (HDEC0+HSTRT)."},
    { Setting_Stepper5, "Can be either negative, zero, or positive, -3 to 12.\\n"
                        "CHM=0: Hysteresis end (low). Sets the hysteresis end value after a number of decrements, used for the hysteresis chopper and controlled by HDEC. HSTRT+HEND must be less than 16. 1/512 adds to the current setting.\\n"
                        "CHM=1: Sine wave offset. A positive offset corrects for zero crossing error. 1/512 adds to the absolute value of each sine wave entry." },
    { Setting_Stepper6, "CHM=0: Hysteresis decrement interval period in system clock periods. Determines the slope of the hysteresis during on time from fast to very slow (0-3 = 16,32,48,64).\\n"
                         "CHM=1: Fast decay mode." },
    { Setting_Stepper7, "Change from fixed to randomized TOFF times, by dNCLK= -24 to +6 clocks." },
//    { Setting_Stepper8, "StallGuard threshold." },
    { Setting_Stepper9, "Lower CoolStep threshold. If the SG value falls below SEMIN x 32, the coil current scaling factor is increased (0-15).\\n"
                         "0: CoolStep disabled."},
    { Setting_Stepper10, "Number of increments of the coil current each time SG is sampled below the lower threshold (0-3 = 1,2,4,8)." },
    { Setting_Stepper11, "Upper CoolStep threshold offset from lower threshold. If SG is sampled above (SEMIN+SEMAX+1)x32 enough times, the coil current scaling factor is decremented (0-15)." },
    { Setting_Stepper12, "Number of times SG must be sampled above the upper threshold before the coil current is decremented (0-3 = 32,8,2,1)." },
    { Setting_Stepper13, "Minimum CoolStep current as a factor of the set motor current\\n"
                          "0: 1/2, 1: 1/4" },
    { Setting_Stepper14, "DRVCONF register." },
#endif
};

#endif

static void trinamic_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&trinamic, sizeof(trinamic_settings_t), true);
}

static void trinamic_settings_get_defaults (bool cap_only)
{
    const trinamic_cfg_params_t *params;

#if TRINAMIC_ENABLE == 2209
    params = TMC2209_GetConfigDefaults();
#elif TRINAMIC_ENABLE == 2660
    params = TMC2660_GetConfigDefaults();
#elif TRINAMIC_ENABLE == 2130
    params = TMC2130_GetConfigDefaults();
#elif TRINAMIC_ENABLE == 5160
    params = TMC5160_GetConfigDefaults();
#endif

    memcpy(&cfg_cap, &params->dflt, sizeof(trinamic_cfg_t));

    if(!cap_only)
        memcpy(cfg_params, &params->dflt, sizeof(trinamic_cfg_t));

#ifdef TMC_DRVCONF
    cfg_params->drvconf = TMC_DRVCONF;
#endif
#ifdef TMC_COOLCONF_SEMIN
    cfg_params->coolconf.semin = TMC_COOLCONF_SEMIN & cfg_cap.coolconf.semin;
#endif
#ifdef TMC_COOLCONF_SEMAX
    cfg_params->coolconf.semax = TMC_COOLCONF_SEMAX & cfg_cap.coolconf.semax;
#endif
#ifdef TMC_COOLCONF_SEDN
    cfg_params->coolconf.sedn = TMC_COOLCONF_SEDN & cfg_cap.coolconf.sedn;
#endif
#ifdef TMC_COOLCONF_SEUP
    cfg_params->coolconf.seup = TMC_COOLCONF_SEUP & cfg_cap.coolconf.seup;
#endif
#ifdef TMC_COOLCONF_SEIMIN
    cfg_params->coolconf.seimin = TMC_COOLCONF_SEIMIN & cfg_cap.coolconf.seimin;
#endif

#ifdef TMC_CHOPCONF_HSTRT
    cfg_params->chopconf.hstrt = (TMC_CHOPCONF_HSTRT - 1) & cfg_cap.chopconf.hstrt;
#endif
#ifdef TMC_CHOPCONF_HEND
    cfg_params->chopconf.hend = (TMC_CHOPCONF_HEND + 3) & cfg_cap.chopconf.hend;
#endif
#ifdef TMC_CHOPCONF_TBL
    cfg_params->chopconf.tbl = TMC_CHOPCONF_TBL & cfg_cap.chopconf.tbl;
#endif
#ifdef TMC_CHOPCONF_TOFF
    cfg_params->chopconf.toff = TMC_CHOPCONF_TOFF & cfg_cap.chopconf.toff;
#endif
#ifdef TMC_CHOPCONF_RDNTF
    cfg_params->chopconf.rndtf = TMC_CHOPCONF_RDNTF & cfg_cap.chopconf.rndtf;
#endif
#ifdef TMC_CHOPCONF_CHM
    cfg_params->chopconf.chm = TMC_CHOPCONF_CHM & cfg_cap.chopconf.chm;
#endif
#ifdef TMC_CHOPCONF_HDEC
    cfg_params->chopconf.hdec = TMC_CHOPCONF_HDEC & cfg_cap.chopconf.hdec;
#endif
#ifdef TMC_CHOPCONF_TFD
    cfg_params->chopconf.tfd = TMC_CHOPCONF_TFD & cfg_cap.chopconf.tfd;
#endif
#ifdef TMC_CHOPCONF_INTPOL
    cfg_params->chopconf.intpol = TMC_CHOPCONF_INTPOL & cfg_cap.chopconf.intpol;
#endif
}

// Initialize default EEPROM settings
static void trinamic_settings_restore (void)
{
    uint_fast8_t idx = N_AXIS;

    trinamic.driver_enable.mask = driver_enabled.mask = 0;
    trinamic.homing_enable.mask = 0;

    do {

        switch(--idx) {

            case X_AXIS:
#if TMC_X_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.x = TMC_X_ENABLE;
                trinamic.driver[idx].current = TMC_X_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_X_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_X_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_X_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_X_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_X_HOMING_FEED_SGT;
                break;

            case Y_AXIS:
#if TMC_Y_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.y = TMC_Y_ENABLE;
                trinamic.driver[idx].current = TMC_Y_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_Y_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_Y_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_Y_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_Y_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_Y_HOMING_FEED_SGT;
                break;

            case Z_AXIS:
#if TMC_Z_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.z = TMC_Z_ENABLE;
                trinamic.driver[idx].current = TMC_Z_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_Z_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_Z_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_Z_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_Z_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_Z_HOMING_FEED_SGT;
                break;

#ifdef A_AXIS
            case A_AXIS:
#if TMC_A_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.a = TMC_A_ENABLE;
                trinamic.driver[idx].current = TMC_A_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_A_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_A_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_A_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_A_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_A_HOMING_FEED_SGT;
                break;
#endif

#ifdef B_AXIS
            case B_AXIS:
#if TMC_B_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.b = TMC_B_ENABLE;
                trinamic.driver[idx].current = TMC_B_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_B_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_B_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_B_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_B_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_B_HOMING_FEED_SGT;
                break;
#endif

#ifdef C_AXIS
            case C_AXIS:
#if TMC_C_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.c = TMC_C_ENABLE;
                trinamic.driver[idx].current = TMC_C_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_C_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_C_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_C_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_C_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_C_HOMING_FEED_SGT;
                break;
#endif
#ifdef U_AXIS
            case U_AXIS:
#if TMC_U_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.u = TMC_U_ENABLE;
                trinamic.driver[idx].current = TMC_U_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_U_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_U_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_U_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_U_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_U_HOMING_FEED_SGT;
                break;
#endif
#ifdef V_AXIS
            case V_AXIS:
#if TMC_V_STEALTHCHOP
                trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                trinamic.driver_enable.v = TMC_V_ENABLE;
                trinamic.driver[idx].current = TMC_V_CURRENT;
                trinamic.driver[idx].hold_current_pct = TMC_V_HOLD_CURRENT_PCT;
                trinamic.driver[idx].microsteps = TMC_V_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_V_R_SENSE;
                trinamic.driver[idx].homing_seek_sensitivity = TMC_V_HOMING_SEEK_SGT;
                trinamic.driver[idx].homing_feed_sensitivity = TMC_V_HOMING_FEED_SGT;
                break;
#endif
        }

        trinamic.driver[idx].homing_seek_rate = DEFAULT_HOMING_SEEK_RATE;
        trinamic.driver[idx].homing_feed_rate = DEFAULT_HOMING_FEED_RATE;

    } while(idx);


    trinamic_settings_get_defaults(false);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&trinamic, sizeof(trinamic_settings_t), true);

    if(settings_loaded)
        trinamic_drivers_setup();
}

static void trinamic_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&trinamic, nvs_address, sizeof(trinamic_settings_t), true) != NVS_TransferResult_OK)
        trinamic_settings_restore();
    else {

        uint_fast8_t idx = N_AXIS;

#ifdef TRINAMIC_EXTENDED_SETTINGS
        trinamic_settings_get_defaults(true);
#else
        trinamic_settings_get_defaults(false);
#endif

        do {
#if TMC_STALLGUARD == 4
            if(trinamic.driver[--idx].homing_seek_sensitivity < 0)
                trinamic.driver[idx].homing_seek_sensitivity = 0;
            if(trinamic.driver[idx].homing_feed_sensitivity  < 0)
                trinamic.driver[idx].homing_feed_sensitivity = 0;
#else
            if(trinamic.driver[--idx].homing_seek_sensitivity > 64)
                trinamic.driver[idx].homing_seek_sensitivity = 0;
            if(trinamic.driver[idx].homing_feed_sensitivity  > 64)
                trinamic.driver[idx].homing_feed_sensitivity = 0;
#endif
// Until $-setting is added set from mode from defines
            switch(idx) {
                case X_AXIS:
#if TMC_X_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_X_R_SENSE;
                    break;
                case Y_AXIS:
#if TMC_Y_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_Y_R_SENSE;
                    break;
                case Z_AXIS:
#if TMC_Z_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_Z_R_SENSE;
                    break;
#ifdef A_AXIS
                case A_AXIS:
#if TMC_A_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_A_R_SENSE;
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
#if TMC_B_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_B_R_SENSE;
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
#if TMC_C_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_C_R_SENSE;
                    break;
#endif
#ifdef U_AXIS
                case U_AXIS:
#if TMC_U_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_U_R_SENSE;
                    break;
#endif
#ifdef V_AXIS
                case V_AXIS:
#if TMC_V_STEALTHCHOP
                    trinamic.driver[idx].mode = TMCMode_StealthChop;
#else
                    trinamic.driver[idx].mode = TMCMode_CoolStep;
#endif
                    trinamic.driver[idx].r_sense = TMC_V_R_SENSE;
                    break;
#endif
            }
//
        } while(idx);
    }

#if !TRINAMIC_MIXED_DRIVERS
    trinamic.driver_enable.mask = AXES_BITMASK;
#endif

    settings_loaded = true;
}

static void on_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    static bool init_ok = false;
    static float steps_per_mm[N_AXIS];

    uint_fast8_t idx = N_AXIS;

    settings_changed(settings, changed);

    if(init_ok) {
        do {
            idx--;
            if(steps_per_mm[idx] != settings->axis[idx].steps_per_mm) {
                steps_per_mm[idx] = settings->axis[idx].steps_per_mm;
#if PWM_THRESHOLD_VELOCITY > 0
                uint8_t motor = n_motors;
                do {
                    motor--;
                    if(bit_istrue(driver_enabled.mask, bit(idx)) && idx == motor_map[motor].axis && stepper[motor]->set_tpwmthrs)
                        stepper[motor]->set_tpwmthrs(motor, (float)PWM_THRESHOLD_VELOCITY / 60.0f, steps_per_mm[idx]);
                } while(motor);
#endif
            }
        } while(idx);
    } else {
        init_ok = true;
        do {
            idx--;
            steps_per_mm[idx] = settings->axis[idx].steps_per_mm;
        } while(idx);
    }

#if TRINAMIC_DYNAMIC_CURRENT
    if(hal.stepper.pulse_start != dynamic_current_pulse_start) {
        stepper_block_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = dynamic_current_pulse_start;
    }
#endif
}

static setting_details_t settings_details = {
    .settings = trinamic_settings,
    .n_settings = sizeof(trinamic_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = trinamic_settings_descr,
    .n_descriptions = sizeof(trinamic_settings_descr) / sizeof(setting_descr_t),
#endif
    .load = trinamic_settings_load,
    .save = trinamic_settings_save,
    .restore = trinamic_settings_restore
};

#endif // End region settings


// Add warning info to next realtime report when warning flag set by drivers
static void trinamic_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(warning) {
        warning = false;
#if TRINAMIC_I2C
        TMC_spi_status_t status = tmc_spi_read((trinamic_motor_t){0}, (TMC_spi_datagram_t *)&dgr_monitor);
        otpw_triggered.mask |= dgr_monitor.reg.otpw.mask;
        sprintf(sbuf, "|TMCMON:%d:%d:%d:%d:%d", status, dgr_monitor.reg.ot.mask, dgr_monitor.reg.otpw.mask, dgr_monitor.reg.otpw_cnt.mask, dgr_monitor.reg.error.mask);
        stream_write(sbuf);
#endif
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

// Return pointer to end of string
static char *append (char *s)
{
    while(*s) s++;

    return s;
}

// Write CRLF terminated string to current stream
static void write_line (char *s)
{
    strcat(s, ASCII_EOL);
    hal.stream.write(s);
}

//
static void report_sg_status (void *data)
{
    hal.stream.write("[SG:");
    hal.stream.write(uitoa(stepper[report.sg_status_motor]->get_sg_result(report.sg_status_motor)));
    hal.stream.write("]" ASCII_EOL);
}

static void stepper_pulse_start (stepper_t *motors)
{
    static uint32_t step_count = 0;

    hal_stepper_pulse_start(motors);

    if(motors->step_outbits.mask & report.sg_status_motormask.mask) {
        uint32_t ms = hal.get_elapsed_ticks();
        if(ms - step_count >= 20) {
            step_count = ms;
            protocol_enqueue_foreground_task(report_sg_status, NULL);
        }
/*        step_count++;
        if(step_count >= report.msteps * 4) {
            step_count = 0;
            protocol_enqueue_foreground_task(report_sg_status, NULL);
        } */
    }
}

static char *get_axisname (motor_map_t motor)
{
    static char axisname[3] = {0};

    axisname[0] = axis_letter[motor.axis][0];
    axisname[1] = motor.id == motor.axis ? '\0' : '2';

    return axisname;
}

#if TRINAMIC_DEV

static uint_fast8_t bit_count (uint32_t n)
{
    uint_fast8_t count = 0;

    while (n) {
        n &= (n - 1);
        count++;
    }

    return count;
}

static axes_signals_t get_axes (parameter_words_t words)
{
    axes_signals_t axes = {0};

    axes.x = words.x;
    axes.y = words.y;
    axes.z = words.z;
#ifdef A_AXIS
    axes.a = words.a;
#endif
#ifdef B_AXIS
    axes.b = words.b;
#endif
#ifdef C_AXIS
    axes.c = words.c;
#endif
#ifdef U_AXIS
    axes.u = words.u;
#endif
#ifdef V_AXIS
    axes.v = words.v;
#endif
    axes.mask &= driver_enabled.mask;

    return axes;
}

#endif

// Validate M-code axis parameters
// Sets value to NAN (Not A Number) and returns false if driver not installed
static bool check_params (parser_block_t *gc_block)
{
    static const parameter_words_t wordmap[] = {
       { .x = On },
       { .y = On },
       { .z = On }
#if N_AXIS > 3
     , { .a = On },
       { .b = On },
       { .c = On }
#endif
#if N_AXIS > 6
     , { .u = On },
       { .v = On },
#endif
    };

    uint_fast8_t n_found = 0, n_ok = 0, idx = N_AXIS;

    do {
        idx--;
        if(gc_block->words.mask & wordmap[idx].mask) {
            n_found++;
            if(bit_istrue(driver_enabled.mask, bit(idx)) && !isnanf(gc_block->values.xyz[idx])) {
                n_ok++;
                gc_block->words.mask &= ~wordmap[idx].mask;
            }
        } else
            gc_block->values.xyz[idx] = NAN;
    } while(idx);

    return n_ok > 0 && n_ok == n_found;
}

// Check if given M-code is handled here
static user_mcode_type_t trinamic_MCodeCheck (user_mcode_t mcode)
{
#if TRINAMIC_DEV
    if(mcode == Trinamic_ReadRegister || mcode == Trinamic_WriteRegister)
        return UserMCode_Normal;
#endif

    return (driver_enabled.mask && (mcode == Trinamic_StepperCurrent || mcode == Trinamic_ReportPrewarnFlags ||
                                     mcode == Trinamic_ClearPrewarnFlags || mcode == Trinamic_HybridThreshold ||
                                      mcode == Trinamic_HomingSensitivity))
             ? UserMCode_Normal
             : (mcode == Trinamic_DebugReport
                 ? UserMCode_NoValueWords
                 : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported));
}

// Validate driver specific M-code parameters
static status_code_t trinamic_MCodeValidate (parser_block_t *gc_block)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

#if TRINAMIC_DEV

        case Trinamic_ReadRegister:
        case Trinamic_WriteRegister:
            {
                axes_signals_t axes = get_axes(gc_block->words);
                if(axes.mask == 0)
                    state = Status_GcodeNoAxisWords;
                else if(bit_count(axes.mask) > 1)
                    state = Status_ValueWordConflict;
                else {
                    if(gc_block->words.i && !(gc_block->values.ijk[0] == 0.0f || gc_block->values.ijk[0] == 1.0f))
                        state = Status_GcodeValueOutOfRange;
                    if(gc_block->words.r && (gc_block->user_mcode == Trinamic_ReadRegister || gc_block->words.o)) {
                        if(!isintf(gc_block->values.r))
                            state = Status_BadNumberFormat;
                        else {
                            if(stepper[0]->get_register_addr(0, (uint8_t)gc_block->values.r))
                                state = Status_OK;
                            else
                                state = Status_GcodeValueOutOfRange;
                            gc_block->words.r = gc_block->words.o = Off;
                        }
                    }
                }
            }
            break;

#endif

        case Trinamic_DebugReport:
            state = Status_OK;

            if(gc_block->words.h && gc_block->values.h > 1)
                state = Status_BadNumberFormat;

            if(gc_block->words.q && isnanf(gc_block->values.q))
                state = Status_BadNumberFormat;

            if(gc_block->words.s && isnanf(gc_block->values.s))
                state = Status_BadNumberFormat;

            gc_block->words.h = gc_block->words.i = gc_block->words.q = gc_block->words.s =
             gc_block->words.x = gc_block->words.y = gc_block->words.z = Off;

#ifdef A_AXIS
            gc_block->words.a = Off;
#endif
#ifdef B_AXIS
            gc_block->words.b = Off;
#endif
#ifdef C_AXIS
            gc_block->words.c = Off;
#endif
//            gc_block->user_mcode_sync = true;
            break;

        case Trinamic_StepperCurrent:
            if(check_params(gc_block)) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
                if(!gc_block->words.q)
                    gc_block->values.q = NAN;
                else // TODO: add range check?
                    gc_block->words.q = Off;
            }
            break;

        case Trinamic_ReportPrewarnFlags:
        case Trinamic_ClearPrewarnFlags:
            state = Status_OK;
            break;

        case Trinamic_HybridThreshold:
            if(check_params(gc_block)) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
            }
            break;

        case Trinamic_HomingSensitivity:
            if(check_params(gc_block)) {
                uint_fast8_t idx = N_AXIS;
                state = gc_block->words.i && (isnanf(gc_block->values.ijk[0]) || gc_block->values.ijk[0] != 1.0f) ? Status_BadNumberFormat : Status_OK;
                gc_block->words.i = Off;
                if(state == Status_OK) do {
                    idx--;
#if TMC_STALLGUARD == 4
                    if(!isnanf(gc_block->values.xyz[idx]) && (gc_block->values.xyz[idx] < 0.0f || gc_block->values.xyz[idx] > 255.0f))
                        state = Status_BadNumberFormat;
#else
                    if(!isnanf(gc_block->values.xyz[idx]) && (gc_block->values.xyz[idx] < -64.0f || gc_block->values.xyz[idx] > 63.0f))
                        state = Status_BadNumberFormat;
#endif
                } while(idx && state == Status_OK);
            }
            break;

        case Trinamic_ChopperTiming:
            if(check_params(gc_block)) {
                state = Status_OK;
                if(gc_block->words.o && (gc_block->values.o < 1 || gc_block->values.ijk[0] > 15))
                    state = Status_BadNumberFormat;
                if(gc_block->words.p && (!isintf(gc_block->values.p) || gc_block->values.p < -3.0f || gc_block->values.p > 12.0f))
                    state = Status_BadNumberFormat;
                if(gc_block->words.s && (!isintf(gc_block->values.s) || gc_block->values.s < 1.0f || gc_block->values.ijk[0] > 8.0f))
                    state = Status_BadNumberFormat;
                gc_block->words.o = gc_block->words.p = gc_block->words.s = Off;
            }
            break;

    default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

// Execute driver specific M-code
static void trinamic_MCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;
    uint_fast8_t motor = n_motors;

    switch(gc_block->user_mcode) {

#if TRINAMIC_DEV

        case Trinamic_ReadRegister:
            {
                uint32_t value;
                uint_fast8_t axis, index = gc_block->words.i ? (uint8_t)gc_block->values.ijk[0] : 0;
                axes_signals_t axes = get_axes(gc_block->words);
                do {
                    axis = motor_map[--motor].axis;
                    if(bit_istrue(axes.mask, bit(axis)) && index ? axis != motor_map[motor].id : axis == motor_map[motor].id) {
                        stepper[motor]->read_register(motor, (uint8_t)gc_block->values.r, &value);
                        sprintf(sbuf, "[TMCREG:%d:%lx]" ASCII_EOL, (uint8_t)gc_block->values.r, value);
                        hal.stream.write(sbuf);
                    }
                } while(motor);
            }
            break;

        case Trinamic_WriteRegister:
            {
                // NOTE: gcode parser has to be changed to allow reading full range uint32_t value into the O word!
                uint_fast8_t axis, index = gc_block->words.i ? (uint8_t)gc_block->values.ijk[0] : 0;
                axes_signals_t axes = get_axes(gc_block->words);
                do {
                    axis = motor_map[--motor].axis;
                    if(bit_istrue(axes.mask, bit(axis)) && index ? axis != motor_map[motor].id : axis == motor_map[motor].id) {
                        stepper[motor]->write_register(motor, (uint8_t)gc_block->values.r, gc_block->values.o);
                    }
                } while(motor);
            }
            break;

#endif

        case Trinamic_DebugReport:
            {
                if(driver_enabled.mask != trinamic.driver_enable.mask) {
                    if(gc_block->words.i)
                        trinamic_drivers_init(trinamic.driver_enable);
                    else
                        protocol_enqueue_foreground_task(report_warning, "Could not communicate with stepper driver!");
                    return;
                }

                if(!trinamic.driver_enable.mask) {
                    hal.stream.write("TMC driver(s) not enabled, enable with $338 setting." ASCII_EOL);
                    return;
                }

                axes_signals_t axes = {0};
                bool write_report = !(gc_block->words.i || gc_block->words.s || gc_block->words.h || gc_block->words.q);

                if(!write_report) {

                    if(gc_block->words.i)
                        trinamic_drivers_init(driver_enabled);

                    if(gc_block->words.h)
                        report.sfilt = gc_block->values.h != 0.0f;

                    if(gc_block->words.q)
                        report.raw = gc_block->values.q != 0.0f;

                    if(gc_block->words.s)
                        report.sg_status_enable = gc_block->values.s != 0.0f;
                }

                axes.x = gc_block->words.x;
                axes.y = gc_block->words.y;
                axes.z = gc_block->words.z;
    #ifdef A_AXIS
                axes.a = gc_block->words.a;
    #endif
    #ifdef B_AXIS
                axes.b = gc_block->words.b;
    #endif
    #ifdef C_AXIS
                axes.c = gc_block->words.c;
    #endif
    #ifdef U_AXIS
                axes.u = gc_block->words.u;
    #endif
    #ifdef V_AXIS
                axes.v = gc_block->words.v;
    #endif
                axes.mask &= driver_enabled.mask;

                if(!write_report) {

                    uint_fast16_t axis;

                    do {
                        motor--;
                        if(stepper[motor] && (axis = motor_map[motor].axis) == report.sg_status_motor) {
                            if(trinamic.driver[axis].mode == TMCMode_StealthChop)
                                stepper[motor]->stealthchop_enable(motor);
                            else if(trinamic.driver[axis].mode == TMCMode_CoolStep)
                                stepper[motor]->coolstep_enable(motor);
                        }
                    } while(motor);

                    if(axes.mask) {
                        uint_fast16_t mask = axes.mask;
                        axis = 0;
                        while(mask) {
                            if(mask & 0x01) {
                                report.sg_status_motor = axis;
                                break;
                            }
                            axis++;
                            mask >>= 1;
                        }
                    }

                    if(report.sg_status_enable && stepper[report.sg_status_motor]) {

                        report.sg_status_motormask.mask = 1 << report.sg_status_motor;
                        report.msteps = trinamic.driver[report.sg_status_motor].microsteps;
                        if(hal_stepper_pulse_start == NULL) {
                            hal_stepper_pulse_start = hal.stepper.pulse_start;
                            hal.stepper.pulse_start = stepper_pulse_start;
                        }

                        motor = n_motors;
                        do {
                            if((axis = motor_map[--motor].axis) == report.sg_status_motor) {
                                stepper[motor]->stallguard_enable(motor, settings.homing.feed_rate, settings.axis[axis].steps_per_mm, trinamic.driver[motor_map[motor].axis].homing_seek_sensitivity);
                                stepper[motor]->sg_filter(motor, report.sfilt);
                                if(stepper[motor]->set_thigh_raw) // TODO: TMC2209 and TMC2260 do not have this...
                                    stepper[motor]->set_thigh_raw(motor, 0);
                            }
                        } while(motor);
                    } else if(hal_stepper_pulse_start != NULL) {
                        hal.stepper.pulse_start = hal_stepper_pulse_start;
                        hal_stepper_pulse_start = NULL;
                    }

                } else
                    write_debug_report(axes.mask ? axes.mask : driver_enabled.mask);
            }
            break;

        case Trinamic_StepperCurrent:
            do {
                if(!isnanf(gc_block->values.xyz[motor_map[--motor].axis]))
                    stepper[motor]->set_current(motor, (uint16_t)gc_block->values.xyz[motor_map[motor].axis],
                                                      isnanf(gc_block->values.q) ? trinamic.driver[motor_map[motor].axis].hold_current_pct : (uint8_t)gc_block->values.q);
            } while(motor);
            break;

        case Trinamic_ReportPrewarnFlags:
            {
                TMC_drv_status_t status;
                strcpy(sbuf, "[TMCPREWARN:");
                for(motor = 0; motor < n_motors; motor++) {
                    if(bit_istrue(driver_enabled.mask, bit(motor_map[motor].axis))) {
                        status = stepper[motor]->get_drv_status(motor);
                        strcat(sbuf, "|");
                        strcat(sbuf, get_axisname(motor_map[motor]));
                        strcat(sbuf, ":");
                        if(status.driver_error)
                            strcat(sbuf, "E");
                        else if(status.ot)
                            strcat(sbuf, "O");
                        else if(status.otpw)
                            strcat(sbuf, "W");
                    }
                }
                hal.stream.write(sbuf);
                hal.stream.write("]" ASCII_EOL);
            }
            break;

        case Trinamic_ClearPrewarnFlags:
            otpw_triggered.mask = 0;
            break;

        case Trinamic_HybridThreshold:
            {
                uint_fast8_t axis;
                do {
                    axis = motor_map[--motor].axis;
                    if(!isnanf(gc_block->values.xyz[axis]) && stepper[motor]->set_tpwmthrs) // mm/min
                        stepper[motor]->set_tpwmthrs(motor, gc_block->values.xyz[axis] / 60.0f, settings.axis[axis].steps_per_mm);
                } while(motor);
            }
            break;

        case Trinamic_HomingSensitivity:
            {
                uint_fast8_t axis;
                do {
                    axis = motor_map[--motor].axis;
                    if(!isnanf(gc_block->values.xyz[axis])) {
                        trinamic.driver[axis].homing_seek_sensitivity = (int16_t)gc_block->values.xyz[axis];
                        stepper[motor]->sg_filter(motor, report.sfilt);
                        stepper[motor]->sg_stall_value(motor, trinamic.driver[axis].homing_seek_sensitivity);
                    }
                } while(motor);
            }
            break;

        case Trinamic_ChopperTiming:
            {
                uint_fast8_t axis;
                trinamic_chopconf_t timing;

                timing.value = cfg_params->chopconf.value; // TDODO: keep per axis config?

                if(gc_block->words.o)
                    timing.toff = (uint8_t)gc_block->values.o;

                if(gc_block->words.p)
                    timing.hend = (int8_t)gc_block->values.p + 3;

                if(gc_block->words.s)
                    timing.hstrt = (uint8_t)gc_block->values.s;

                do {
                    axis = motor_map[--motor].axis;
                    if(!isnanf(gc_block->values.xyz[axis])) {
                        stepper[motor]->chopper_timing(motor, timing);
                    }
                } while(motor);
            }
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}


#if TRINAMIC_I2C

static void trinamic_stepper_enable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;

    dgr_enable.reg.enable.mask = enable.mask & driver_enabled.mask;

    tmc_spi_write((trinamic_motor_t){0}, (TMC_spi_datagram_t *)&dgr_enable);
}

#endif

#if TMC_POLL_STALLED

// hal.limits.get_state is redirected here when homing
static limit_signals_t trinamic_limits (void)
{
    limit_signals_t signals = limits_get_state(); // read from switches first

    signals.min.mask &= ~homing.mask;
    signals.min2.mask &= ~homing.mask;

    if(hal.clear_bits_atomic(&diag1_poll, 0)) {
        // TODO: read I2C bridge status register instead of polling drivers when using I2C comms
        uint_fast8_t motor = n_motors;
        do {
            if(bit_istrue(homing.mask, bit(motor_map[--motor].axis))) {
                if(stepper[motor]->get_drv_status(motor).stallguard) {
                    if(motor == motor_map[motor].axis)
                        bit_true(signals.min.mask, motor_map[motor].axis);
                    else
                        bit_true(signals.min2.mask, motor_map[motor].axis);
                }
            }
        } while(motor);
    }

    return signals;
}

#endif

// Configure sensorless homing for enabled axes
static void trinamic_on_homing (axes_signals_t axes, float feedrate, homing_mode_t mode)
{
    uint_fast8_t motor = n_motors, axis;

    axes.mask &= (driver_enabled.mask & trinamic.homing_enable.mask);

    if(axes.mask) do {

        axis = motor_map[--motor].axis;

        if(bit_istrue(axes.mask, bit(axis))) switch(mode) {

            case HomingMode_Seek:
                stepper[motor]->stallguard_enable(motor, feedrate, settings.axis[axis].steps_per_mm, trinamic.driver[axis].homing_seek_sensitivity);
                break;

            case HomingMode_Locate:
                stepper[motor]->stallguard_enable(motor, feedrate, settings.axis[axis].steps_per_mm, trinamic.driver[axis].homing_feed_sensitivity);
                break;

            default: // HomingMode_Pulloff
                if(trinamic.driver[axis].mode == TMCMode_StealthChop)
                    stepper[motor]->stealthchop_enable(motor);
                else if(trinamic.driver[axis].mode == TMCMode_CoolStep)
                    stepper[motor]->coolstep_enable(motor);
                break;
        }
    } while(motor);
}

// Get homing rate for the homing cycle.
// NOTE: if more than one axis is homed in the cycle all axes has to be configured with
//       the same feedrates or the cycle will be skipped.
static float trinamic_get_homing_rate (axes_signals_t axes, homing_mode_t mode)
{
    axes.mask &= (driver_enabled.mask & trinamic.homing_enable.mask);

    if(!axes.mask /*?? || mode == HomingMode_Pulloff*/)
        return mode == HomingMode_Locate ? settings.homing.feed_rate : settings.homing.seek_rate;

    uint_fast8_t motor = n_motors, axis;
    float feed_rate = 0.0f, seek_rate = 0.0f;

    do {
        axis = motor_map[--motor].axis;
        if(bit_istrue(axes.mask, bit(axis))) {

            float feed_rate_cfg = trinamic.driver[axis].homing_feed_rate,
                  seek_rate_cfg = trinamic.driver[axis].homing_seek_rate;

            if(feed_rate == 0.0f) {
                feed_rate = feed_rate_cfg;
                seek_rate = seek_rate_cfg;
            } else if(!(feed_rate == feed_rate_cfg && seek_rate == seek_rate_cfg)) {
                feed_rate = seek_rate = 0.0f;
                break;
            }
        } else {
            if(feed_rate == 0.0f) {
                feed_rate = settings.homing.feed_rate;
                seek_rate = settings.homing.seek_rate;
            } else if(!(feed_rate == settings.homing.feed_rate && seek_rate == settings.homing.seek_rate)) {
                feed_rate = seek_rate = 0.0f;
                break;
            }
        }
    } while(motor);

    return mode == HomingMode_Locate ? feed_rate : seek_rate;
}

// Enable/disable sensorless homing
static void trinamic_homing (bool on, axes_signals_t homing_cycle)
{
#ifdef TMC_HOMING_ACCELERATION
    static float accel[N_AXIS];
#endif

    if(limits_enable)
        limits_enable(on, homing_cycle);

    homing.mask = driver_enabled.mask & trinamic.homing_enable.mask;

    is_homing = homing_cycle.mask != 0;

#if TRINAMIC_DYNAMIC_CURRENT
    if(is_homing)
        set_current_for_homing();
#endif

    if(is_homing && homing.mask) {

        grbl.on_homing_rate_set = trinamic_on_homing;

#ifdef TMC_HOMING_ACCELERATION
        uint_fast8_t axis = 0, axes = homing.mask;
        while(axes) {
            if(axes & 1 && accel[axis] == 0.0f) {
                accel[axis] = settings.axis[axis].acceleration / (60.0f * 60.0f);
                settings_override_acceleration(axis, min(TMC_HOMING_ACCELERATION, accel[axis]));
            }
            axes >>= 1;
            axis++;
        }
#endif
#if TMC_POLL_STALLED
        if(limits_get_state == NULL) {
            limits_get_state = hal.limits.get_state;
            hal.limits.get_state = trinamic_limits;
        }
        diag1_poll = 0;
#endif
    } else {

        uint_fast8_t motor = n_motors, axis;

        do {
            axis = motor_map[--motor].axis;
            if(bit_istrue(driver_enabled.mask, bit(axis))) {
                if(trinamic.driver[axis].mode == TMCMode_StealthChop)
                    stepper[motor]->stealthchop_enable(motor);
                else if(trinamic.driver[axis].mode == TMCMode_CoolStep)
                    stepper[motor]->coolstep_enable(motor);
#ifdef TMC_HOMING_ACCELERATION
                if(accel[axis] > 0.0f) {
                    settings_override_acceleration(axis, accel[axis]);
                    accel[axis] = 0.0f;
                }
#endif
            }
        } while(motor);
#if TMC_POLL_STALLED
        if(limits_get_state != NULL) {
            hal.limits.get_state = limits_get_state;
            limits_get_state = NULL;
        }
#endif
    }
}

// Write Marlin style driver debug report to output stream (M122)
// NOTE: this output is not in a parse friendly format for grbl senders
static void write_debug_report (uint_fast8_t axes)
{
    typedef struct {
        TMC_chopconf_t chopconf;
        TMC_drv_status_t drv_status;
        uint16_t current;
        TMC_ihold_irun_t ihold_irun;
    } debug_report_t;

    uint_fast8_t motor = n_motors;
    bool has_gscaler = false;
    debug_report_t debug_report[6];

    hal.stream.write("[TRINAMIC]" ASCII_EOL);

    do {
        if(bit_istrue(axes, bit(motor_map[--motor].axis))) {
            debug_report[motor].drv_status = stepper[motor]->get_drv_status(motor);
            debug_report[motor].chopconf = stepper[motor]->get_chopconf(motor);
            debug_report[motor].current = stepper[motor]->get_current(motor, TMCCurrent_Actual);
            debug_report[motor].ihold_irun =  stepper[motor]->get_ihold_irun(motor);
//            TMC5160_ReadRegister(&stepper[motor], (TMC5160_datagram_t *)&stepper[motor]->pwm_scale);
            if(debug_report[motor].drv_status.otpw)
                otpw_triggered.mask |= bit(motor);
            has_gscaler |= !!stepper[motor]->get_global_scaler;
        }
    } while(motor);

    if(report.raw) {

    } else {

        sprintf(sbuf, "%-15s", "");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", get_axisname(motor_map[motor]));
        }

        write_line(sbuf);
        sprintf(sbuf, "%-15s", "Driver");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", stepper[motor]->name);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Set current");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", stepper[motor]->get_config(motor)->current);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "RMS current");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", debug_report[motor].current);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Peak current");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8" UINT32SFMT, (uint32_t)((float)debug_report[motor].current * sqrtf(2)));
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Run current");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%5d/31", debug_report[motor].ihold_irun.irun);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Hold current");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%5d/31", debug_report[motor].ihold_irun.ihold);
        }
        write_line(sbuf);

        if(has_gscaler) {
            sprintf(sbuf, "%-15s", "Global scaler");
            for(motor = 0; motor < n_motors; motor++) {
                if(bit_istrue(axes, bit(motor)) && stepper[motor]->get_global_scaler)
                    sprintf(append(sbuf), "%4d/256", stepper[motor]->get_global_scaler(motor));
            }
            write_line(sbuf);
        }

        sprintf(sbuf, "%-15s", "CS actual");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%5d/31", debug_report[motor].drv_status.cs_actual);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "PWM scale");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)) && stepper[motor]->pwm_scale)
                sprintf(append(sbuf), "%8d", stepper[motor]->pwm_scale(motor));
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "vsense");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis))) {
                if(stepper[motor]->vsense)
                    sprintf(append(sbuf), "%8s", stepper[motor]->vsense(motor) ? "1=0.180" : "0=0.325");
                else
                    sprintf(append(sbuf), "%8s", "N/A");
            }
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "stealthChop");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", stepper[motor]->get_en_pwm_mode && stepper[motor]->get_en_pwm_mode(motor) ? "true" : "false");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "msteps");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", 1 << (8 - debug_report[motor].chopconf.mres));
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "tstep");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis))) {
                if(stepper[motor]->get_tstep)
                    sprintf(append(sbuf), "%8" UINT32SFMT, stepper[motor]->get_tstep(motor));
                else
                    sprintf(append(sbuf), "%8s", "-");
            }
        }
        write_line(sbuf);

        hal.stream.write("pwm" ASCII_EOL);

        sprintf(sbuf, "%-15s", "threshold");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis))) {
                if(stepper[motor]->get_tpwmthrs_raw)
                    sprintf(append(sbuf), "%8" UINT32SFMT, stepper[motor]->get_tpwmthrs_raw(motor));
                else
                    sprintf(append(sbuf), "%8s", "-");
            }
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "[mm/s]");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis))) {
                if(stepper[motor]->get_tpwmthrs)
                    sprintf(append(sbuf), "%8" UINT32SFMT, (uint32_t)stepper[motor]->get_tpwmthrs(motor, settings.axis[motor_map[motor].axis].steps_per_mm));
                else
                    sprintf(append(sbuf), "%8s", "-");
            }
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "OT prewarn");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.otpw ? "true" : "false");
        }
        write_line(sbuf);

        hal.stream.write("OT prewarn has" ASCII_EOL);
        sprintf(sbuf, "%-15s", "been triggered");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", bit_istrue(otpw_triggered.mask, bit(motor_map[motor].axis)) ? "true" : "false");
        }
        write_line(sbuf);

/*
    #if HAS_TMC220x
      TMC_REPORT("pwm scale sum",     TMC_PWM_SCALE_SUM);
      TMC_REPORT("pwm scale auto",    TMC_PWM_SCALE_AUTO);
      TMC_REPORT("pwm offset auto",   TMC_PWM_OFS_AUTO);
      TMC_REPORT("pwm grad auto",     TMC_PWM_GRAD_AUTO);
    #endif
        case TMC_PWM_SCALE_SUM: SERIAL_PRINT(st.pwm_scale_sum(), DEC); break;
        case TMC_PWM_SCALE_AUTO: SERIAL_PRINT(st.pwm_scale_auto(), DEC); break;
        case TMC_PWM_OFS_AUTO: SERIAL_PRINT(st.pwm_ofs_auto(), DEC); break;
        case TMC_PWM_GRAD_AUTO: SERIAL_PRINT(st.pwm_grad_auto(), DEC); break;

        sprintf(sbuf, "%-15s", "pwm autoscale");

        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", stepper[motor]->pwmconf.reg.pwm_autoscale);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "pwm ampl");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", stepper[motor]->pwmconf.reg.pwm_ampl);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "pwm grad");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", stepper[motor]->pwmconf.reg.pwm_grad);
        }
        write_line(sbuf);
*/

        sprintf(sbuf, "%-15s", "off time");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", debug_report[motor].chopconf.toff);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "blank time");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", debug_report[motor].chopconf.tbl);
        }
        write_line(sbuf);

        hal.stream.write("hysteresis" ASCII_EOL);

        sprintf(sbuf, "%-15s", "-end");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", (int)debug_report[motor].chopconf.hend - 3);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "-start");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", debug_report[motor].chopconf.hstrt + 1);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Stallguard thrs");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", stepper[motor]->get_sg_stall_value(motor));
        }
        write_line(sbuf);

        hal.stream.write("DRIVER STATUS:" ASCII_EOL);

        sprintf(sbuf, "%-15s", "stallguard");
        write_line(sbuf);
        sprintf(sbuf, "%-15s", "sg_result");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8d", debug_report[motor].drv_status.sg_result);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "fsactive");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.fsactive ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "stst");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.stst ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "olb");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.olb ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "ola");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.ola ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "s2gb");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.s2gb ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "s2ga");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.s2ga ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "otpw");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.otpw ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "ot");
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis)))
                sprintf(append(sbuf), "%8s", debug_report[motor].drv_status.ot ? "*" : "");
        }
        write_line(sbuf);

        hal.stream.write("STATUS REGISTERS:" ASCII_EOL);
        for(motor = 0; motor < n_motors; motor++) {
            if(bit_istrue(axes, bit(motor_map[motor].axis))) {
                uint32_t reg = stepper[motor]->get_drv_status_raw(motor);
                sprintf(sbuf, " %s = 0x%02X:%02X:%02X:%02X", get_axisname(motor_map[motor]), (uint8_t)(reg >> 24), (uint8_t)((reg >> 16) & 0xFF), (uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF));
                write_line(sbuf);
            }
        }
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
    	report_plugin("Trinamic", "0.17");
    else if(driver_enabled.mask) {
        hal.stream.write(",TMC=");
        hal.stream.write(uitoa(driver_enabled.mask));
    }
}

static void count_motors (motor_map_t motor)
{
    n_motors++;
}

static void assign_motors (motor_map_t motor)
{
    motor_map[n_motors++].value = motor.value;
}

static bool on_driver_setup (settings_t *settings)
{
    bool ok;

    if((ok = driver_setup(settings))) {
        hal.delay_ms(100, NULL); // Allow time for drivers to boot
        trinamic_drivers_setup();
    }

    return ok;
}

bool trinamic_init (void)
{
    if(hal.stepper.motor_iterator) {
        hal.stepper.motor_iterator(count_motors);
        if((motor_map = malloc(n_motors * sizeof(motor_map_t)))) {
            n_motors = 0;
            hal.stepper.motor_iterator(assign_motors);
        }
    } else {
        motor_map = malloc(N_AXIS * sizeof(motor_map_t));
        if(motor_map) {
            uint_fast8_t idx;
    //        n_motors = N_AXIS;
            for(idx = 0; idx < N_AXIS; idx++) {
                motor_map[idx].id = idx;
                motor_map[idx].axis = idx;
            }
            n_motors = idx;
        }
    }

    if(motor_map && (nvs_address = nvs_alloc(sizeof(trinamic_settings_t)))) {

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

        grbl.user_mcode.check = trinamic_MCodeCheck;
        grbl.user_mcode.validate = trinamic_MCodeValidate;
        grbl.user_mcode.execute = trinamic_MCodeExecute;

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = trinamic_realtime_report;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_setup = hal.driver_setup;
        hal.driver_setup = on_driver_setup;

        settings_changed = hal.settings_changed;
        hal.settings_changed = on_settings_changed;

        limits_enable = hal.limits.enable;
        hal.limits.enable = trinamic_homing;

        hal.homing.get_feedrate = trinamic_get_homing_rate;

        settings_register(&settings_details);

#if TRINAMIC_I2C
        stepper_enable = hal.stepper.enable;
        hal.stepper.enable = trinamic_stepper_enable;
#endif
    }

    return nvs_address != 0;
}

// Interrupt handler for DIAG1 signal(s)
void trinamic_fault_handler (void)
{
    if(is_homing)
        diag1_poll = 1;
    else {
        limit_signals_t limits = {0};
        limits.min.mask = AXES_BITMASK;
        hal.limits.interrupt_callback(limits);
    }
}

#if TRINAMIC_I2C
// Interrupt handler for warning event from I2C bridge
// Sets flag to add realtime report message
void trinamic_warn_handler (void)
{
    warning = true;
}
#endif

#endif
