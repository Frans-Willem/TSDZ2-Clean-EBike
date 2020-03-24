/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho and EndlessCadence, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "ebike_app.h"
#include <stdint.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "main.h"
#include "adc.h"
#include "utils.h"
#include "motor.h"
#include "pwm.h"
#include "brake.h"
#include "eeprom.h"
#include "config.h"

#define STATE_NO_PEDALLING                0
#define STATE_PEDALLING                   2

#define BOOST_STATE_BOOST_DISABLED        0
#define BOOST_STATE_BOOST                 1
#define BOOST_STATE_FADE                  2
#define BOOST_STATE_BOOST_WAIT_TO_RESTART 3

uint8_t ui8_adc_battery_max_current       = ADC_BATTERY_CURRENT_MAX;
uint8_t ui8_target_battery_max_power_x10  = ADC_BATTERY_CURRENT_MAX;

volatile struct_configuration_variables m_configuration_variables;

// variables for various system functions
volatile uint8_t ui8_system_state = NO_ERROR;
volatile uint16_t ui16_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile enum pedaling_direction_t enm_g_pedaling_direction = 0;
uint8_t   ui8_pas_cadence_rpm = 0;
uint16_t  ui16_pedal_torque_x10;
uint16_t  ui16_pedal_power_x10;
uint8_t   ui8_pedal_human_power = 0;
uint16_t  ui16_adc_motor_temperatured_accumulated = 0;
uint8_t   ui8_m_adc_battery_target_current;
uint8_t ui8_tstr_state_machine = STATE_NO_PEDALLING;
static uint8_t ui8_m_motor_enabled = 1;
static uint8_t ui8_m_brake_is_set = 0;
volatile uint8_t  ui8_throttle = 0;
volatile uint8_t  ui8_torque_sensor = 0;
volatile uint8_t  ui8_torque_sensor_raw = 0;
volatile uint8_t  ui8_g_adc_torque_sensor_min_value;
volatile uint8_t  ui8_g_adc_torque_sensor_max_value;
volatile uint8_t  ui8_adc_battery_current_offset;
volatile uint8_t  ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP;
volatile uint8_t  ui8_adc_target_battery_max_current;
uint8_t           ui8_adc_battery_current_max;
volatile uint16_t ui16_current_ramp_up_inverse_step;


// variables for walk assist
uint8_t ui8_m_walk_assist_target_duty_cycle = 0;


// variables for cruise function
static uint8_t    ui8_cruise_target_PWM = 0;
static uint8_t    ui8_initialize_cruise_PID = 1;
uint16_t   ui16_received_target_wheel_speed_x10 = 0;
static uint16_t   ui16_target_wheel_speed_x10 = 0;


// variables for wheel speed
volatile uint16_t   ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
uint8_t             ui8_wheel_speed_max = 0;
float               f_wheel_speed_x10;
uint16_t     ui16_wheel_speed_x10;
uint16_t     ui16_wheel_speed_for_vlcd = 0;
volatile uint32_t   ui32_wheel_speed_sensor_tick_counter = 0;



static void communications_controller (void);

// system functions
static void ebike_control_motor (void);
static void ebike_app_set_target_adc_battery_max_current (uint8_t ui8_value);

static void check_system(void);
static void throttle_read (void);
static void read_pas_cadence (void);
static void torque_sensor_read (void);
static void calc_pedal_force_and_torque (void);
static void calc_wheel_speed (void);
static void calc_motor_temperature (void);

static void apply_speed_limit (uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint8_t *ui8_target_current);
static void apply_temperature_limiting (uint8_t *ui8_target_current);
static void apply_walk_assist(uint8_t *ui8_p_adc_target_current);
static void apply_cruise (uint8_t *ui8_target_current);
static void apply_throttle(uint8_t ui8_throttle_value, uint8_t *ui8_target_current);


// BOOST
uint8_t   ui8_startup_boost_enable = 0;
uint8_t   ui8_startup_boost_fade_enable = 0;
uint8_t   ui8_m_startup_boost_state_machine = 0;
uint8_t   ui8_startup_boost_no_torque = 0;
uint8_t   ui8_startup_boost_timer = 0;
uint8_t   ui8_startup_boost_fade_steps = 0;
uint16_t  ui16_startup_boost_fade_variable_x256;
uint16_t  ui16_startup_boost_fade_variable_step_amount_x256;
static void     boost_run_statemachine (void);
static uint8_t  apply_boost (uint8_t ui8_pas_cadence, uint8_t ui8_max_current_boost_state, uint8_t *ui8_target_current);
static void     apply_boost_fade_out (uint8_t *ui8_target_current);



void ebike_app_init (void)
{
  // init variables with the stored value on EEPROM
  eeprom_init_variables ();
  ebike_app_set_battery_max_current (ADC_BATTERY_CURRENT_MAX);
  
  initializeConfigurationVariables();
}


void ebike_app_controller (void)
{
  throttle_read();
  torque_sensor_read();
  read_pas_cadence();
  calc_pedal_force_and_torque();
  calc_wheel_speed();
  calc_motor_temperature();
  ebike_control_motor();
  communications_controller();
  check_system();
}


static void ebike_control_motor (void)
{
  static uint32_t ui32_temp;
  uint8_t ui8_tmp_pas_cadence_rpm;
  uint32_t ui32_adc_max_battery_current_x4;
  uint8_t ui8_adc_max_battery_current = 0;
  uint8_t ui8_adc_max_battery_power_current = 0;
  uint8_t ui8_boost_enabled_and_applied = 0;

  uint16_t ui16_battery_voltage_filtered = calc_filtered_battery_voltage ();

  uint8_t ui8_adc_max_battery_current_boost_state = 0;

  // the ui8_m_brake_is_set is updated here only and used all over ebike_control_motor()
  ui8_m_brake_is_set = brake_is_set();

  // make sure this vars are reset to avoid repetion code on next elses
  ui8_adc_max_battery_current_boost_state = 0;
  ui8_m_adc_battery_target_current = 0;
  ui8_adc_max_battery_power_current = 0;

  // controller works with no less than 15V so calculate the target current only for higher voltages
  if(ui16_battery_voltage_filtered > 15)
  {
    // 1.6 = 1 / 0.625 (each adc step for current)
    // 25 * 1.6 = 40
    // 40 * 4 = 160
    if(m_configuration_variables.ui8_startup_motor_power_boost_assist_level > 0)
    {
      ui32_temp = (uint32_t) ui16_pedal_torque_x10 * (uint32_t) m_configuration_variables.ui8_startup_motor_power_boost_assist_level;
      ui32_temp /= 10;

      // 1.6 = 1 / 0.625 (each adc step for current)
      // 1.6 * 8 = ~13
      ui32_temp = (ui32_temp * 13) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_current_boost_state = ui32_temp >> 3;
      ui8_limit_max(&ui8_adc_max_battery_current_boost_state, 255);
    }
    else
    {
      // nothing
    }

    if(m_configuration_variables.ui8_assist_level_factor_x10 > 0)
    {
      if(m_configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation == 0)
      {
        ui32_temp = (uint32_t) ui16_pedal_power_x10 * (uint32_t) m_configuration_variables.ui8_assist_level_factor_x10;
        ui32_temp /= 100;
      }
      else
      {
        if(ui8_pas_cadence_rpm)
        {
          ui32_temp = (uint32_t) ui16_pedal_power_x10 * (uint32_t) m_configuration_variables.ui8_assist_level_factor_x10;
          ui32_temp /= 100;
        }
        else
        {
          ui32_temp = (uint32_t) ui16_pedal_torque_x10 * (uint32_t) m_configuration_variables.ui8_assist_level_factor_x10;
          ui32_temp /= 100;
        }
      }

      // 1.6 = 1 / 0.625 (each adc step for current)
      // 1.6 * 8 = ~13
      ui32_temp = (ui32_temp * 13) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_current = ui32_temp >> 3;
      ui8_limit_max(&ui8_adc_max_battery_current, 255);
      ui8_m_adc_battery_target_current = ui8_adc_max_battery_current;
    }
    else
    {
      // nothing
    }

    if(m_configuration_variables.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
    {
      ui32_adc_max_battery_current_x4 = (((uint32_t) m_configuration_variables.ui8_target_battery_max_power_div25) * 160) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_power_current = ui32_adc_max_battery_current_x4 >> 2;
    }
    else
    {
      // nothing
    }
  }
  else
  {
    // nothing
  }

  ui8_tmp_pas_cadence_rpm = ui8_pas_cadence_rpm;
  // let's cheat next value, only to cheat apply_boost()
  if(m_configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation)
  {
    if(ui8_pas_cadence_rpm < 10) { ui8_tmp_pas_cadence_rpm = 10; }
  }

  // apply boost and boost fade out
  if(m_configuration_variables.ui8_startup_motor_power_boost_feature_enabled)
  {
    boost_run_statemachine();
    ui8_boost_enabled_and_applied = apply_boost(ui8_tmp_pas_cadence_rpm, ui8_adc_max_battery_current_boost_state, &ui8_m_adc_battery_target_current);
    apply_boost_fade_out(&ui8_m_adc_battery_target_current);
  }


  // throttle
  apply_throttle(ui8_throttle, &ui8_m_adc_battery_target_current);


  // walk assist or cruise
  if(m_configuration_variables.ui8_walk_assist)
  {
    // enable walk assist or cruise function depending on speed
    if(ui16_wheel_speed_x10 < WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10) // if current speed is less than threshold speed, enable walk assist
    {
      // enable walk assist
      apply_walk_assist(&ui8_m_adc_battery_target_current);
    }
    else // if current speed is more or equal the threshold speed, enable cruise function
    {
      // enable cruise function
      apply_cruise(&ui8_m_adc_battery_target_current);
    }
  }
  else
  {
    // set flag to initialize cruise PID if user later activates function
    ui8_initialize_cruise_PID = 1;
  }
  
  // speed limit
  apply_speed_limit(ui16_wheel_speed_x10, m_configuration_variables.ui8_wheel_max_speed, &ui8_m_adc_battery_target_current);


  // max power
  if(m_configuration_variables.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
  {
    // limit the current to max value defined by user on LCD max power, if:
    // - user defined to make that limitation
    // - we are not on boost or fade state
    if((m_configuration_variables.ui8_startup_motor_power_boost_limit_to_max_power == 1) || (!((ui8_boost_enabled_and_applied == 1) || (ui8_startup_boost_fade_enable == 1))))
    {
      // now let's limit the target battery current to battery max current (use min value of both)
      ui8_m_adc_battery_target_current = ui8_min(ui8_m_adc_battery_target_current, ui8_adc_max_battery_power_current);
    }
  }


  // motor over temperature protection
  apply_temperature_limiting(&ui8_m_adc_battery_target_current);


  // let's force our target current to 0 if brake is set or if there are errors
  if(ui8_m_brake_is_set || ui8_system_state != NO_ERROR) { ui8_m_adc_battery_target_current = 0; }

  // check to see if we should enable the motor
  if(ui8_m_motor_enabled == 0 &&
    (ui16_motor_get_motor_speed_erps() == 0) && // we can only enable if motor is stopped other way something bad can happen due to high currents/regen or something like that
     ui8_m_adc_battery_target_current)
  {
    ui8_m_motor_enabled = 1;
    ui8_g_duty_cycle = 0;
    motor_enable_pwm();
  }

  // check to see if we should disable the motor
  if(ui8_m_motor_enabled &&
      ui16_motor_get_motor_speed_erps() == 0 &&
      ui8_m_adc_battery_target_current == 0 &&
      ui8_g_duty_cycle == 0)
  {
    ui8_m_motor_enabled = 0;
    motor_disable_pwm();
  }

  // apply the target current if motor is enable and if not, reset the duty_cycle controller
  if(ui8_m_motor_enabled)
  {
    // finally set the target battery current to the battery current controller
    ebike_app_set_target_adc_battery_max_current(ui8_m_adc_battery_target_current);
  }
  else
  {
    ebike_app_set_target_adc_battery_max_current(0);
    ui8_g_duty_cycle = 0;
  }

  // set motor PWM target
  if(m_configuration_variables.ui8_walk_assist && ui8_m_brake_is_set == 0 && ui8_m_motor_enabled)
  {
    if(ui16_wheel_speed_x10 < WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10)
    {
      motor_set_pwm_duty_cycle_target(ui8_m_walk_assist_target_duty_cycle);
    }
    else
    {
      motor_set_pwm_duty_cycle_target(ui8_cruise_target_PWM);
    }
  }
  else if(ui8_m_adc_battery_target_current)
  {
    motor_set_pwm_duty_cycle_target(255);
  }
  else
  {
    motor_set_pwm_duty_cycle_target(0);
  }
}


static void communications_controller (void)
{
  uart_receive_package ();
  uart_send_package ();
}


// each 1 unit = 0.625 amps
static void ebike_app_set_target_adc_battery_max_current (uint8_t ui8_value)
{
  // limit max number of amps
  if (ui8_value > ui8_adc_battery_current_max) { ui8_value = ui8_adc_battery_current_max; }

  ui8_adc_target_battery_max_current = ui8_adc_battery_current_offset + ui8_value;
}


// in amps
void ebike_app_set_battery_max_current(uint8_t ui8_value)
{
  // each 1 unit = 0.625 amps (0.625 * 256 = 160)
  ui8_adc_battery_current_max = ((((uint16_t) ui8_value) << 8) / 160);

  if (ui8_adc_battery_current_max > ADC_BATTERY_CURRENT_MAX) { ui8_adc_battery_current_max = ADC_BATTERY_CURRENT_MAX; }
}


static void calc_pedal_force_and_torque(void)
{
  uint16_t ui16_pedal_torque_x100;

  // calculate torque on pedals
  ui16_pedal_torque_x100 = (uint16_t) ui8_torque_sensor * (uint16_t) ADC_STEP_PEDAL_TORQUE_X100;
  ui16_pedal_torque_x10 = ui16_pedal_torque_x100 / 10;

  // calculate power on pedals
  // formula for angular velocity in degrees: power  =  force  *  rotations per second  *  2  *  pi
  // formula for angular velocity in degrees: power  =  (force  *  rotations per minute  *  2  *  pi) / 60

  // ui16_pedal_power_x10 = (ui16_pedal_torque_x100 * ui8_pas_cadence_rpm  *  2  *  pi) / (60 * 10)
  // (2 * pi) / (60 * 10) = 0.010466667
  // 1 / 0.010466667 = 96
  // ui16_pedal_power_x10 = (ui16_pedal_torque_x100 * ui8_pas_cadence_rpm) / 96

  // NOTE
  /*
  Users did report that pedal human power is about 2x more.

  @casainho had the idea to evaluate the torque sensor peak signal (measuring peak signal every pedal rotation) as being a sinewave and so the average would be:

  > [Average value = 0.637 × maximum or peak value, Vpk](https://www.electronics-tutorials.ws/accircuits/average-voltage.html)

  For a quick hack, we can just reduce actual value to 0.637.

  */
  ui16_pedal_power_x10 = (uint16_t) (((uint32_t) ui16_pedal_torque_x100 * (uint32_t) ui8_pas_cadence_rpm) / (uint32_t) 96);
}


static void calc_wheel_speed(void)
{
  // calc wheel speed in km/h
  if (ui16_wheel_speed_sensor_pwm_cycles_ticks < WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS)
  {
    f_wheel_speed_x10 = ((float) PWM_CYCLES_SECOND) / ((float) ui16_wheel_speed_sensor_pwm_cycles_ticks); // rps
    f_wheel_speed_x10 *= m_configuration_variables.ui16_wheel_perimeter; // millimeters per second
    f_wheel_speed_x10 *= 0.036; // ((3600 / (1000 * 1000)) * 10) kms per hour * 10
    ui16_wheel_speed_x10 = (uint16_t) f_wheel_speed_x10;

    // the speed is a 16 bit integer that contains the time for one wheel rotation measured
    // in ~2.04 ms units
    // https://github.com/hurzhurz/tsdz2/blob/master/serial-communication.md
    // pwm period = 64 us -> 2.04 ms / 64 us = 32
    ui16_wheel_speed_for_vlcd = ui16_wheel_speed_sensor_pwm_cycles_ticks / 32;
  }
  else
  {
    ui16_wheel_speed_x10 = 0;
    ui16_wheel_speed_for_vlcd = 0;
  }
}


static void calc_motor_temperature(void)
{
  uint16_t ui16_adc_motor_temperatured_filtered_10b;

  // low pass filter to avoid possible fast spikes/noise
  ui16_adc_motor_temperatured_accumulated -= ui16_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;
  ui16_adc_motor_temperatured_accumulated += ui16_adc_read_throttle_10b ();
  ui16_adc_motor_temperatured_filtered_10b = ui16_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;

  m_configuration_variables.ui16_motor_temperature_x2 = (uint16_t) ((float) ui16_adc_motor_temperatured_filtered_10b / 1.024);
  m_configuration_variables.ui8_motor_temperature = (uint8_t) (m_configuration_variables.ui16_motor_temperature_x2 >> 1);
}


uint16_t calc_filtered_battery_voltage (void)
{
  uint16_t ui16_batt_voltage_filtered = (uint16_t) motor_get_adc_battery_voltage_filtered_10b () * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  return (ui16_batt_voltage_filtered >> 9);
}


static void apply_speed_limit(uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint8_t *ui8_target_current)
{
  *ui8_target_current = (uint8_t) (map ((uint32_t) ui16_speed_x10,
                                        (uint32_t) ((ui8_max_speed * 10) - 20),
                                        (uint32_t) ((ui8_max_speed * 10) + 20),
                                        (uint32_t) *ui8_target_current,
                                        (uint32_t) 0));
}


static void apply_throttle(uint8_t ui8_throttle_value, uint8_t *ui8_target_current)
{
  // apply throttle if it is enabled and the motor temperature limit function is not enabled instead
  if (m_configuration_variables.ui8_temperature_limit_feature_enabled == 2)
  {
    // overide ui8_adc_battery_current_max with throttle value only when user is using throttle
    if(ui8_throttle_value)
    {
      uint8_t ui8_temp = (uint8_t) (map ((uint32_t) ui8_throttle_value,
                                         (uint32_t) 0,
                                         (uint32_t) 255,
                                         (uint32_t) 0,
                                         (uint32_t) ui8_adc_battery_current_max));

      // set target current
      *ui8_target_current = ui8_temp;
    }
  }
}


static void apply_temperature_limiting(uint8_t *ui8_target_current)
{
  // apply motor temperature protection if it is enabled and the throttle function is not enabled instead
  if (m_configuration_variables.ui8_temperature_limit_feature_enabled == 1)
  {
    // min temperature value can't be equal or higher than max temperature value...
    if (m_configuration_variables.ui8_motor_temperature_min_value_to_limit >= m_configuration_variables.ui8_motor_temperature_max_value_to_limit)
    {
      *ui8_target_current = 0;
      m_configuration_variables.ui8_temperature_current_limiting_value = 0;
    }
    else
    {
      // reduce motor current if over temperature
      *ui8_target_current = 
        (uint8_t) (map ((uint32_t) m_configuration_variables.ui16_motor_temperature_x2,
                        (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                        (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                        (uint32_t) *ui8_target_current,
                        (uint32_t) 0));

      // get a value linear to the current limitation, just to show to user
      m_configuration_variables.ui8_temperature_current_limiting_value = 
        (uint8_t) (map ((uint32_t) m_configuration_variables.ui16_motor_temperature_x2,
                        (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                        (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                        (uint32_t) 255,
                        (uint32_t) 0));
    }
  }
  else
  {
    // keep ui8_temperature_current_limiting_value = 255 because 255 means no current limiting happening, otherwise temperature symbol on display will be blinking
    m_configuration_variables.ui8_temperature_current_limiting_value = 255;
  }
}

static void apply_walk_assist(uint8_t *ui8_p_adc_target_current)
{
  // use max current as the limit of current
  *ui8_p_adc_target_current = ui8_adc_battery_current_max;

  // check so that walk assist level factor is not too large (too powerful), if it is -> limit the value
  if(m_configuration_variables.ui8_assist_level_factor_x10 > 100)
  {
    // limit and set walk assist PWM to some safe value, not too powerful 
    ui8_m_walk_assist_target_duty_cycle = 100;
  }
  else
  {
    // set walk assist PWM to the target duty cycle from user defined value on display (walk assist level factor)
    ui8_m_walk_assist_target_duty_cycle = m_configuration_variables.ui8_assist_level_factor_x10;
  }
}

static void apply_cruise(uint8_t *ui8_target_current)
{
  static int16_t i16_error;
  static int16_t i16_last_error;
  static int16_t i16_integral;
  static int16_t i16_derivative;
  static int16_t i16_control_output;
  
  // set target current to max current
  *ui8_target_current = ui8_adc_battery_current_max;
  
  // initialize cruise PID controller
  if (ui8_initialize_cruise_PID)
  {
    // reset flag to save current speed to maintain (for cruise function)
    ui8_initialize_cruise_PID = 0;
    
    // reset PID variables
    i16_error = 0;          // error should be 0 when cruise function starts
    i16_last_error = 0;     // last error should be 0 when cruise function starts 
    i16_integral = 250;     // integral can start at around 250 when cruise function starts ( 250 = around 64 target PWM = around 8 km/h depending on gear and bike )
    i16_derivative = 0;     // derivative should be 0 when cruise function starts 
    i16_control_output = 0; // control signal/output should be 0 when cruise function starts
    
    // check what target wheel speed to use (received or current)
    if (ui16_received_target_wheel_speed_x10 > 0)
    {
      // set received target wheel speed to target wheel speed
      ui16_target_wheel_speed_x10 = ui16_received_target_wheel_speed_x10;
    }
    else
    {
      // set current wheel speed to maintain
      ui16_target_wheel_speed_x10 = ui16_wheel_speed_x10;
    }
  }
  
  // calculate error
  i16_error = (ui16_target_wheel_speed_x10 - ui16_wheel_speed_x10);
  
  // calculate integral
  i16_integral = i16_integral + i16_error;
  
  // limit integral
  if (i16_integral > CRUISE_PID_INTEGRAL_LIMIT)
  {
    i16_integral = CRUISE_PID_INTEGRAL_LIMIT; 
  }
  else if (i16_integral < 0)
  {
    i16_integral = 0;
  }
  
  // calculate derivative
  i16_derivative = i16_error - i16_last_error;

  // save error to last error
  i16_last_error = i16_error;
  
  // calculate control output ( output =  P I D )
  i16_control_output = (CRUISE_PID_KP * i16_error) + (CRUISE_PID_KI * i16_integral) + (CRUISE_PID_KD * i16_derivative);
  
  // limit control output to just positive values
  if (i16_control_output < 0) { i16_control_output = 0; }
  
  // limit control output to the maximum value
  if (i16_control_output > 1000) { i16_control_output = 1000; }
  
  // map the control output to an appropriate target PWM value
  ui8_cruise_target_PWM = (uint8_t) (map ((uint32_t) (i16_control_output),
                                          (uint32_t) 0,     // minimum control output from PID
                                          (uint32_t) 1000,  // maximum control output from PID
                                          (uint32_t) 0,     // minimum target PWM
                                          (uint32_t) 255)); // maximum target PWM
}


static void boost_run_statemachine(void)
{
  if(m_configuration_variables.ui8_startup_motor_power_boost_time > 0)
  {
    switch(ui8_m_startup_boost_state_machine)
    {
      // ebike is stopped, wait for throttle signal to startup boost
      case BOOST_STATE_BOOST_DISABLED:
        if(ui8_torque_sensor > 12 &&
            (ui8_m_brake_is_set == 0))
        {
          ui8_startup_boost_enable = 1;
          ui8_startup_boost_timer = m_configuration_variables.ui8_startup_motor_power_boost_time;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST;
        }
      break;

      case BOOST_STATE_BOOST:
        // braking means reseting
        if(ui8_m_brake_is_set)
        {
          ui8_startup_boost_enable = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        // end boost if
        if(ui8_torque_sensor < 12)
        {
          ui8_startup_boost_enable = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }

        // decrement timer
        if(ui8_startup_boost_timer > 0) { ui8_startup_boost_timer--; }

        // end boost and start fade if
        if(ui8_startup_boost_timer == 0)
        {
          ui8_m_startup_boost_state_machine = BOOST_STATE_FADE;
          ui8_startup_boost_enable = 0;

          // setup variables for fade
          ui8_startup_boost_fade_steps = m_configuration_variables.ui8_startup_motor_power_boost_fade_time;
          ui16_startup_boost_fade_variable_x256 = ((uint16_t) ui8_m_adc_battery_target_current << 8);
          ui16_startup_boost_fade_variable_step_amount_x256 = (ui16_startup_boost_fade_variable_x256 / ((uint16_t) ui8_startup_boost_fade_steps));
          ui8_startup_boost_fade_enable = 1;
        }
      break;

      case BOOST_STATE_FADE:
        // braking means reseting
        if(ui8_m_brake_is_set)
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        if(ui8_startup_boost_fade_steps > 0) { ui8_startup_boost_fade_steps--; }

        // disable fade if
        if(ui8_torque_sensor < 12 ||
            ui8_startup_boost_fade_steps == 0)
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }
      break;

      // restart when user is not pressing the pedals AND/OR wheel speed = 0
      case BOOST_STATE_BOOST_WAIT_TO_RESTART:
        // wheel speed must be 0 as also torque sensor
        if((m_configuration_variables.ui8_startup_motor_power_boost_state & 1) == 0)
        {
          if(ui16_wheel_speed_x10 == 0 &&
              ui8_torque_sensor < 12)
          {
            ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
        // torque sensor must be 0
        if((m_configuration_variables.ui8_startup_motor_power_boost_state & 1) > 0)
        {
          if(ui8_torque_sensor < 12 ||
              ui8_pas_cadence_rpm == 0)
          {
            ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
      break;

      default:
      break;
    }
  }
}


static uint8_t apply_boost (uint8_t ui8_pas_cadence, uint8_t ui8_max_current_boost_state, uint8_t *ui8_target_current)
{
  uint8_t ui8_boost_enable = ui8_startup_boost_enable && m_configuration_variables.ui8_assist_level_factor_x10 && ui8_pas_cadence > 0 ? 1 : 0;

  if (ui8_boost_enable)
  {
    *ui8_target_current = ui8_max_current_boost_state;
  }

  return ui8_boost_enable;
}


static void apply_boost_fade_out(uint8_t *ui8_target_current)
{
  if (ui8_startup_boost_fade_enable)
  {
    // here we try to converge to the regular value, ramping down or up step by step
    uint16_t ui16_adc_battery_target_current_x256 = ((uint16_t) ui8_m_adc_battery_target_current) << 8;
    if (ui16_startup_boost_fade_variable_x256 > ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 -= ui16_startup_boost_fade_variable_step_amount_x256;
    }
    else if (ui16_startup_boost_fade_variable_x256 < ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 += ui16_startup_boost_fade_variable_step_amount_x256;
    }

    *ui8_target_current = (uint8_t) (ui16_startup_boost_fade_variable_x256 >> 8);
  }
}


static void read_pas_cadence(void)
{
  // cadence in RPM =  60 / (ui16_pas_timer2_ticks * PAS_NUMBER_MAGNETS * 0.000064)
  if((ui16_pas_pwm_cycles_ticks >= ((uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS)) ||
      (enm_g_pedaling_direction != pedaling_direction_forward)) // if not rotating pedals forward
  { 
    ui8_pas_cadence_rpm = 0; 
  }
  else
  {
    ui8_pas_cadence_rpm = (uint8_t) (60 / (((float) ui16_pas_pwm_cycles_ticks) * ((float) PAS_NUMBER_MAGNETS) * 0.000064));
  }
}


static void torque_sensor_read(void)
{
  uint8_t ui8_adc_torque_sensor = UI8_ADC_TORQUE_SENSOR;

  // remove the offset
  // make sure readed value is higher than the offset
  if(ui8_adc_torque_sensor >= ui8_g_adc_torque_sensor_min_value)
  {
    ui8_torque_sensor_raw = ui8_adc_torque_sensor - ui8_g_adc_torque_sensor_min_value;
  }
  // offset is higher, something is wrong so just keep ui8_torque_sensor_raw at 0 value
  else
  {
    ui8_torque_sensor_raw = 0;
  }

  // next state machine is used to filter out the torque sensor signal
  // when user is resting on the pedals
  switch(ui8_tstr_state_machine)
  {
    // ebike is stopped
    case STATE_NO_PEDALLING:
    if(ui8_torque_sensor_raw > 0 && ui16_wheel_speed_x10)
    {
      ui8_tstr_state_machine = STATE_PEDALLING;
    }
    break;

    // wait on this state and reset when ebike stops
    case STATE_PEDALLING:
    if(ui16_wheel_speed_x10 == 0 && ui8_torque_sensor_raw == 0)
    {
      ui8_tstr_state_machine = STATE_NO_PEDALLING;
    }
    break;

    default:
    break;
  }

  // bike is moving but user doesn't pedal, disable torque sensor signal because user can be resting the feet on the pedals
  if(ui8_tstr_state_machine == STATE_PEDALLING && ui8_pas_cadence_rpm == 0)
  {
    ui8_torque_sensor = 0;
  }
  else
  {
    ui8_torque_sensor = ui8_torque_sensor_raw;
  }
}


static void throttle_read (void)
{
  // map value from 0 up to 255
  ui8_throttle =  (uint8_t) (map (UI8_ADC_THROTTLE,
                                 (uint8_t) ADC_THROTTLE_MIN_VALUE,
                                 (uint8_t) ADC_THROTTLE_MAX_VALUE,
                                 (uint8_t) 0,
                                 (uint8_t) 255));
}

struct_configuration_variables* get_configuration_variables (void)
{
  return &m_configuration_variables;
}


void check_system()
{
  #define MOTOR_BLOCKED_COUNTER_THRESHOLD             30    // 30  =>  3 seconds
  #define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X5  8     // 8  =>  (8 * 0.826) / 5 = 1.3216 ampere  =>  (X) units = ((X * 0.826) / 5) ampere
  #define MOTOR_BLOCKED_ERPS_THRESHOLD                10    // 10 ERPS
  #define MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD       100   // 100  =>  10 seconds
  
  static uint8_t ui8_motor_blocked_counter;
  static uint8_t ui8_motor_blocked_reset_counter;

  // if the motor blocked error is enabled start resetting it
  if (ui8_system_state == ERROR_MOTOR_BLOCKED)
  {
    // increment motor blocked reset counter with 100 milliseconds
    ui8_motor_blocked_reset_counter++;
    
    // check if the counter has counted to the set threshold for reset
    if (ui8_motor_blocked_reset_counter > MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD)
    {
      // reset motor blocked error code
      ui8_system_state = NO_ERROR;
      
      // reset the counter that clears the motor blocked error
      ui8_motor_blocked_reset_counter = 0;
    }
  }
  else
  {
    // if battery current (x5) is over the current threshold (x5) and the motor ERPS is below threshold start setting motor blocked error code
    if ((motor_get_adc_battery_current_filtered_10b() > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X5) && (ui16_motor_get_motor_speed_erps() < MOTOR_BLOCKED_ERPS_THRESHOLD))
    {
      // increment motor blocked counter with 100 milliseconds
      ui8_motor_blocked_counter++;
      
      // check if motor is blocked for more than some safe threshold
      if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD)
      {
        // set motor blocked error code
        ui8_system_state = ERROR_MOTOR_BLOCKED;
        
        // reset motor blocked counter as the error code is set
        ui8_motor_blocked_counter = 0;
      }
    }
    else
    {
      // current is below the threshold and/or motor ERPS is above the threshold so reset the counter
      ui8_motor_blocked_counter = 0;
    }
  }
}
