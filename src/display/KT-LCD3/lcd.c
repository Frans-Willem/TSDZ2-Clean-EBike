/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <string.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_iwdg.h"
#include "gpio.h"
#include "timers.h"
#include "ht162.h"
#include "lcd.h"
#include "adc.h"
#include "buttons.h"
#include "main.h"
#include "config.h"
#include "eeprom.h"
#include "pins.h"
#include "uart.h"


uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

uint8_t ui8_lcd_field_offset[] = {
    ASSIST_LEVEL_DIGIT_OFFSET,
    ODOMETER_DIGIT_OFFSET,
    TEMPERATURE_DIGIT_OFFSET,
    WHEEL_SPEED_OFFSET,
    BATTERY_POWER_DIGIT_OFFSET,
    SECOND_DIGIT_OFFSET,
    MINUTE_DIGIT_OFFSET,
    0
};

uint8_t ui8_lcd_digit_mask[] = {
    NUMBER_0_MASK,
    NUMBER_1_MASK,
    NUMBER_2_MASK,
    NUMBER_3_MASK,
    NUMBER_4_MASK,
    NUMBER_5_MASK,
    NUMBER_6_MASK,
    NUMBER_7_MASK,
    NUMBER_8_MASK,
    NUMBER_9_MASK
};

uint8_t ui8_lcd_digit_mask_inverted[] = {
    NUMBER_0_MASK_INVERTED,
    NUMBER_1_MASK_INVERTED,
    NUMBER_2_MASK_INVERTED,
    NUMBER_3_MASK_INVERTED,
    NUMBER_4_MASK_INVERTED,
    NUMBER_5_MASK_INVERTED,
    NUMBER_6_MASK_INVERTED,
    NUMBER_7_MASK_INVERTED,
    NUMBER_8_MASK_INVERTED,
    NUMBER_9_MASK_INVERTED
};

typedef struct _var_number
{
  void *p_var_number;
  uint8_t ui8_size;
  uint8_t ui8_decimal_digit;
  uint32_t ui32_max_value;
  uint32_t ui32_min_value;
  uint32_t ui32_increment_step;
  uint8_t ui8_odometer_field;
} var_number_t;

static struct_motor_controller_data motor_controller_data;
static struct_configuration_variables configuration_variables;


// global system variables
static uint16_t   ui16_battery_voltage_filtered_x10;
static uint16_t   ui16_battery_current_filtered_x5;
static uint16_t   ui16_battery_power_filtered_x50;
static uint16_t   ui16_battery_power_filtered;
static volatile uint32_t ui32_wh_sum_x5 = 0;
static uint32_t   ui32_wh_x10 = 0;
static uint8_t    ui8_config_wh_x10_offset;
static uint16_t   ui16_battery_soc_watts_hour;
static uint16_t   ui16_battery_voltage_soc_x10;
static uint16_t   ui16_pedal_torque_filtered;
static uint16_t   ui16_pedal_power_filtered;
static uint8_t    ui8_pedal_cadence_filtered;
static uint8_t    ui8_lights_state = 0;
static uint8_t    ui8_street_mode_enabled = 0;
static volatile uint16_t ui16_timer3_counter = 0;


// menu variables
static uint8_t    ui8_lcd_menu = MAIN_MENU;
static uint8_t    ui8_lcd_menu_config_submenu_state = 0;
static uint8_t    ui8_lcd_menu_flash_state;
static uint8_t    ui8_lcd_menu_flash_state_temperature;
static uint8_t    ui8_lcd_menu_config_submenu_number = 0;
static uint8_t    ui8_lcd_menu_config_submenu_active = 0;
static uint8_t    ui8_lcd_menu_config_submenu_change_variable_enabled = 0;
static uint8_t    ui8_odometer_sub_field_state;
uint8_t           ui8_start_odometer_show_field_number = 0;
uint8_t           ui8_odometer_show_field_number_counter = 1;
uint8_t           ui8_odometer_show_field_number = 0;


// time measurement variables
static volatile uint8_t ui8_second = 0;
static uint16_t   ui16_seconds_since_power_on = 0;
static uint8_t    ui8_second_TM = 0;
static uint16_t   ui16_minute_TM = 0;


// energy data variables
static uint16_t   ui16_average_energy_consumption_since_power_on_x10 = 0;
static uint32_t   ui32_wh_since_power_on_x10 = 0;
static uint16_t   ui16_estimated_range_since_power_on_x10 = 0;


// wheel measurement variables
static uint8_t    ui8_average_measured_wheel_speed_x10 = 0;
static uint8_t    ui8_max_measured_wheel_speed_x10 = 0;


// system functions
void low_pass_filter_battery_voltage_current_power (void);
void assist_level_state (void);
void brake (void);
void odometer (void);
void wheel_speed (void);
void temperature (void);
void battery_soc (void);
void calc_distance (void);
void calc_battery_soc (void);
void low_pass_filter_pedal_torque_and_power (void);
static void low_pass_filter_pedal_cadence (void);
void lights_state (void);
void walk_assist_state (void);
void street_mode (void);
void time_measurement (void);
void energy_data (void);


// menu functions
void lcd_execute_main_screen (void);
void lcd_execute_menu_config (void);
void lcd_execute_menu_config_power (void);
void lcd_execute_menu_config_submenu_basic_config (void);
void lcd_execute_menu_config_submenu_battery (void);
void lcd_execute_menu_config_submenu_assist_level (void);
void lcd_execute_menu_config_submenu_walk_assist (void);
void lcd_execute_menu_config_submenu_cruise (void);
void lcd_execute_menu_config_main_screen_setup (void);
void lcd_execute_menu_config_submenu_motor_startup_power_boost (void);
void lcd_execute_menu_config_submenu_motor_temperature (void);
void lcd_execute_menu_config_submenu_street_mode (void);
void lcd_execute_menu_config_submenu_technical (void);
void update_menu_flashing_state (void);
void submenu_state_controller(uint8_t ui8_state_max_number);
void advance_on_subfield (uint8_t* ui8_p_state, uint8_t ui8_state_max_number);
void odometer_increase_field_state (void);
uint8_t reset_variable_check (void);


// LCD functions
void lcd_power_off (uint8_t SaveToEEPROM);
void lcd_update (void);
void lcd_clear (void);
void lcd_set_frame_buffer (void);
void lcd_set_backlight_intensity (uint8_t ui8_intensity);
void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options);
void lcd_configurations_print_number(var_number_t* p_lcd_var_number);
void power(void);
void power_off_timer (void);


// LCD symbol functions
void lcd_enable_w_symbol (uint8_t ui8_state);
void lcd_enable_vol_symbol (uint8_t ui8_state);
void lcd_enable_km_symbol (uint8_t ui8_state);
void lcd_enable_mil_symbol (uint8_t ui8_state);
void lcd_enable_kmh_symbol (uint8_t ui8_state);
void lcd_enable_mph_symbol (uint8_t ui8_state);
void lcd_enable_odo_symbol (uint8_t ui8_state);
void lcd_enable_avs_symbol (uint8_t ui8_state);
void lcd_enable_mxs_symbol (uint8_t ui8_state);
void lcd_enable_walk_symbol (uint8_t ui8_state);
void lcd_enable_kmh_symbol (uint8_t ui8_state);
void lcd_enable_dst_symbol (uint8_t ui8_state);
void lcd_enable_tm_symbol (uint8_t ui8_state);
void lcd_enable_ttm_symbol (uint8_t ui8_state);
void lcd_enable_colon_symbol (uint8_t ui8_state);
void lcd_enable_motor_symbol (uint8_t ui8_state);
void lcd_enable_brake_symbol (uint8_t ui8_state);
void lcd_enable_assist_symbol (uint8_t ui8_state);
void lcd_enable_lights_symbol (uint8_t ui8_state);
void lcd_enable_cruise_symbol (uint8_t ui8_state);
void lcd_enable_temperature_1_symbol (uint8_t ui8_state);
void lcd_enable_odometer_point_symbol (uint8_t ui8_state);
void lcd_enable_battery_power_1_symbol (uint8_t ui8_state);
void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state);
void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state);
void lcd_enable_temperature_degrees_symbol (uint8_t ui8_state);
void lcd_enable_temperature_farneight_symbol (uint8_t ui8_state);


// happens every 1 ms
void TIM3_UPD_OVF_BRK_IRQHandler(void) __interrupt(TIM3_UPD_OVF_BRK_IRQHANDLER)
{
  ui16_timer3_counter++;
  
  static uint8_t ui8_100ms_timmer_counter;
  static uint16_t ui16_second_counter;
  
  // calculate watt-hours every 100 ms
  if (++ui8_100ms_timmer_counter >= 100)
  {
    // reset counter
    ui8_100ms_timmer_counter = 0;
    
    // measure consumed watt-hours
    ui32_wh_sum_x5 += ui16_battery_power_filtered_x50 / 10;
  }
  
  // increment second for time measurement 
  if (++ui16_second_counter >= 1000)
  {
    // reset counter
    ui16_second_counter = 0;
    
    // increment second
    ui8_second++;
  }
  
  // clear Interrupt Pending bit
  TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
}


uint16_t get_timer3_counter(void)
{
  return ui16_timer3_counter;
}


void lcd_clock (void)
{
  // clear the screen
  lcd_clear ();

  // return here until the first communication package is received from the motor controller
  if (uart_received_first_package () == 0) { return; }

  // LCD menus 
  switch (ui8_lcd_menu)
  {
    case MAIN_MENU: 
      lcd_execute_main_screen ();
    break;

    case POWER_MENU:
      lcd_execute_menu_config_power ();
    break;
    
    case CONFIGURATION_MENU:
      lcd_execute_menu_config ();
    break;
    
    default:
      ui8_lcd_menu = MAIN_MENU;
    break;
  }
  
  update_menu_flashing_state ();
  low_pass_filter_battery_voltage_current_power ();
  low_pass_filter_pedal_cadence ();
  low_pass_filter_pedal_torque_and_power ();
  calc_battery_soc ();
  calc_distance ();
  lcd_update ();
  power_off_timer ();
}


void lcd_execute_main_screen (void)
{
  temperature ();
  odometer ();
  wheel_speed ();
  walk_assist_state ();
  street_mode ();
  power ();
  battery_soc ();
  lights_state ();
  brake ();
  time_measurement ();
  energy_data ();
  assist_level_state ();
  
  // enter configuration menu if...
  if (UP_DOWN_LONG_CLICK)
  {
    ui8_lcd_menu = CONFIGURATION_MENU;
  }
  
  // enter power menu if...
  if (ONOFF_UP_LONG_CLICK && configuration_variables.ui8_main_screen_power_menu_enabled && !ui8_street_mode_enabled)
  {
    ui8_lcd_menu = POWER_MENU;
  }
  
  // power off if...
  if (ONOFF_LONG_CLICK) 
  {
    lcd_power_off (1); 
  }
}


void lcd_execute_menu_config (void)
{
  if (ui8_lcd_menu_config_submenu_active)
  {
    switch (ui8_lcd_menu_config_submenu_number)
    {
      case 0:
        lcd_execute_menu_config_submenu_basic_config ();
      break;

      case 1:
        lcd_execute_menu_config_submenu_battery ();
      break;

      case 2:
        lcd_execute_menu_config_submenu_assist_level ();
      break;
      
      case 3:
        lcd_execute_menu_config_submenu_walk_assist ();
      break;
      
      case 4:
        lcd_execute_menu_config_submenu_cruise ();
      break;
      
      case 5:
        lcd_execute_menu_config_main_screen_setup ();
      break;

      case 6:
        lcd_execute_menu_config_submenu_motor_startup_power_boost ();
      break;

      case 7:
        lcd_execute_menu_config_submenu_motor_temperature ();        
      break;

      case 8:
        lcd_execute_menu_config_submenu_street_mode ();
      break;

      case 9:
        lcd_execute_menu_config_submenu_technical ();
      break;  

      default:
        ui8_lcd_menu_config_submenu_number = 0;
      break;
    }
  }
  else
  {
    // advance on submenu if...
    if (UP_CLICK)
    {
      if (ui8_lcd_menu_config_submenu_number < 9) { ++ui8_lcd_menu_config_submenu_number; } 
      else { ui8_lcd_menu_config_submenu_number = 0; }
    }

    // recede on submenu if...
    if (DOWN_CLICK)
    {
      if (ui8_lcd_menu_config_submenu_number > 0) { --ui8_lcd_menu_config_submenu_number; } 
      else { ui8_lcd_menu_config_submenu_number = 9; }
    }
  
    // enter submenu if...
    if (ONOFF_CLICK)
    {
      ui8_lcd_menu_config_submenu_active = 1;
      
      ui8_config_wh_x10_offset = 1;
    }
    
    // leave config menu if...
    if (ONOFF_LONG_CLICK)
    {
      // save the updated variables on EEPROM
      eeprom_write_variables ();
      
      // switch to main menu
      ui8_lcd_menu = MAIN_MENU;
    }
    
    // print submenu number only half of the time
    if (ui8_lcd_menu_flash_state)
    {
      lcd_print(ui8_lcd_menu_config_submenu_number, WHEEL_SPEED_FIELD, 0);
    }
  }
}


void lcd_execute_menu_config_submenu_basic_config(void)
{
  var_number_t lcd_var_number;
  uint32_t ui32_temp;
  static uint8_t ui8_reset_to_defaults_counter;
  
  switch(ui8_lcd_menu_config_submenu_state)
  {
    // menu to choose units type
    case 0:

      lcd_var_number.p_var_number = &configuration_variables.ui8_units_type;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // clear previous number written on ODOMETER_FIELD
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ODOMETER_FIELD] - 1] &= NUMBERS_MASK;
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        if (configuration_variables.ui8_units_type == 1)
        {
          lcd_enable_mil_symbol(1);
          lcd_enable_mph_symbol(1);
          lcd_enable_temperature_farneight_symbol(1);
        }
        else
        {
          lcd_enable_km_symbol(1);
          lcd_enable_kmh_symbol(1);
          lcd_enable_temperature_degrees_symbol(1);
        }
      }

    break;
    
    // menu to choose max wheel speed
    case 1:
      
      // display max wheel speed in either imperial or metric units
      if (configuration_variables.ui8_units_type)
      {
        // imperial
        lcd_var_number.p_var_number = &configuration_variables.ui8_wheel_max_speed_imperial;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 62; // needs to be 1.6 times smaller than metric max value
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
        
        lcd_enable_mph_symbol (1);
        
        // convert max wheel speed in imperial units to metric units and save to ui8_wheel_max_speed
        configuration_variables.ui8_wheel_max_speed = (uint8_t) (((float) configuration_variables.ui8_wheel_max_speed_imperial) * 1.6);
      }
      else
      {
        // metric  
        lcd_var_number.p_var_number = &configuration_variables.ui8_wheel_max_speed;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 99; // needs to be smaller than 100 or else value > digits on display
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;        
        lcd_configurations_print_number(&lcd_var_number);
        
        lcd_enable_kmh_symbol (1);
      }
      
    break;
    
    // menu to choose wheel perimeter in millimeters
    case 2:
    
      lcd_var_number.p_var_number = &configuration_variables.ui16_wheel_perimeter;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 3000;
      lcd_var_number.ui32_min_value = 750;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    
    break;
    
    // set odometer
    case 3:

      if (configuration_variables.ui8_units_type)
      {
        // imperial
        ui32_temp = (uint32_t) (((float) configuration_variables.ui32_odometer_x10) / 1.6);

        lcd_var_number.p_var_number = &ui32_temp;
        lcd_var_number.ui8_size = 32;
        lcd_var_number.ui8_decimal_digit = 1;
        lcd_var_number.ui32_max_value = 4294967295; // needs to be 1.6 times smaller than metric max value
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 35;
        lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
        
        // convert imperial distance back to metric and save
        configuration_variables.ui32_odometer_x10 = (uint16_t) (((float) ui32_temp) * 1.6);
        
        lcd_enable_odo_symbol(1);
        lcd_enable_mil_symbol(1);
      }
      else
      {
        // metric
        lcd_var_number.p_var_number = &configuration_variables.ui32_odometer_x10;
        lcd_var_number.ui8_size = 32;
        lcd_var_number.ui8_decimal_digit = 1;
        lcd_var_number.ui32_max_value = 4294967295;
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 35;
        lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
        lcd_configurations_print_number(&lcd_var_number);

        lcd_enable_odo_symbol(1);
        lcd_enable_km_symbol(1);
      }
      
      // set backlight brightness after user has configured settings, looks nicer this way
      if (ui8_lights_state == 0) { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_off_brightness); }
      else { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_on_brightness); }
      
    break;
    
    // backlight off brightness
    case 4:
    
      ui32_temp = configuration_variables.ui8_lcd_backlight_off_brightness * 5;
      
      lcd_var_number.p_var_number = &ui32_temp;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 5;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // convert percentage value
      configuration_variables.ui8_lcd_backlight_off_brightness = ui32_temp / 5;
      
      // show user the chosen backlight brightness, looks nicer this way
      lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_off_brightness);
      
    break;

    // backlight on brightness
    case 5:
    
      ui32_temp = configuration_variables.ui8_lcd_backlight_on_brightness * 5;
      
      lcd_var_number.p_var_number = &ui32_temp;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 5;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // convert percentage value
      configuration_variables.ui8_lcd_backlight_on_brightness = ui32_temp / 5;
      
      // show user the chosen backlight brightness, looks nicer this way
      lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_on_brightness);
      
    break;

    // auto power off
    case 6:
    
      lcd_var_number.p_var_number = &configuration_variables.ui8_lcd_power_off_time_minutes;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 255;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // set backlight brightness after user has configured settings, looks nicer this way
      if (ui8_lights_state == 0) { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_off_brightness); }
      else { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_on_brightness); }
      
    break;

    // reset to defaults
    case 7:
    
      lcd_var_number.p_var_number = &ui8_reset_to_defaults_counter;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 10;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);

      if (ui8_reset_to_defaults_counter > 9)
      {
        // erase saved EEPROM values (all values will be set to defaults)
        eeprom_erase_key_value ();

        // Turn off LCD
        lcd_power_off (0);
      }
      
    break;
  }
  
  submenu_state_controller(7); // 7 sub menus
  
  if (ui8_lcd_menu_config_submenu_state > 1 && (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled))
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
}


void lcd_execute_menu_config_submenu_battery (void)
{
  var_number_t lcd_var_number;
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    // battery max current
    case 0:
      lcd_var_number.p_var_number = &configuration_variables.ui8_battery_max_current;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery low voltage cut-off
    case 1:
      lcd_var_number.p_var_number = &configuration_variables.ui16_battery_low_voltage_cut_off_x10;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 630;
      lcd_var_number.ui32_min_value = 160;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery number of cells in series
    case 2:
      lcd_var_number.p_var_number = &configuration_variables.ui8_battery_cells_number;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 15;
      lcd_var_number.ui32_min_value = 7;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery internal resistance
    case 3:
      lcd_var_number.p_var_number = &configuration_variables.ui16_battery_pack_resistance_x1000;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1000;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery voltage SOC
    case 4:
      lcd_print (ui16_battery_voltage_soc_x10, ODOMETER_FIELD, 1);
      lcd_enable_vol_symbol(1);
    break;
    
    // menu to enable/disable show of numeric watt-hour value and type of representation
    case 5:
      lcd_var_number.p_var_number = &configuration_variables.ui8_battery_SOC_function_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 2;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // menu to set battery_voltage_reset_wh_counter
    case 6:
      lcd_var_number.p_var_number = &configuration_variables.ui16_battery_voltage_reset_wh_counter_x10;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 630;
      lcd_var_number.ui32_min_value = 160;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // menu to set battery capacity in watt-hours
    case 7:
      lcd_var_number.p_var_number = &configuration_variables.ui32_wh_x10_100_percent;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 100000;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 100;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // menu to set current watt hour value
    case 8:
      // on the very first time, use current value of ui32_wh_x10
      if (ui8_config_wh_x10_offset)
      {
        ui8_config_wh_x10_offset = 0;
        configuration_variables.ui32_wh_x10_offset = ui32_wh_x10;
      }
      
      // keep reseting these values
      ui32_wh_sum_x5 = 0;
      ui32_wh_x10 = 0;

      lcd_var_number.p_var_number = &configuration_variables.ui32_wh_x10_offset;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 100000;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 100;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
  }
  
  submenu_state_controller(8); // 8 sub menus
  
  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
}


void lcd_execute_menu_config_submenu_assist_level (void)
{
  var_number_t lcd_var_number;

  // number of assist levels: 1 to 9
  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_number_of_assist_levels;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 1;
    lcd_var_number.ui32_max_value = 9;
    lcd_var_number.ui32_min_value = 1;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ASSIST_LEVEL_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol (1);
    }
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
  }
  else // value of each assist level factor
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_assist_level_factor[(ui8_lcd_menu_config_submenu_state - 1)];
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 1;
    lcd_var_number.ui32_max_value = 255;
    lcd_var_number.ui32_min_value = 1;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol (1);
      lcd_print(ui8_lcd_menu_config_submenu_state, ASSIST_LEVEL_FIELD, 1);
    }
  }
  
  submenu_state_controller(configuration_variables.ui8_number_of_assist_levels);
}


void lcd_execute_menu_config_submenu_walk_assist (void)
{
  var_number_t lcd_var_number;

  // enable/disable walk assist function
  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_walk_assist_function_enabled;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (configuration_variables.ui8_walk_assist_function_enabled && (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled))
    {
      lcd_enable_walk_symbol (1);
    }
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
  }
  else // value of each walk assist power value
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_walk_assist_level_factor[(ui8_lcd_menu_config_submenu_state - 1)];
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 100;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    lcd_enable_walk_symbol (1);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol (1);
      lcd_print(ui8_lcd_menu_config_submenu_state - 1, ASSIST_LEVEL_FIELD, 1);
    }
  }
  
  submenu_state_controller(configuration_variables.ui8_number_of_assist_levels + 1);
}


void lcd_execute_menu_config_submenu_cruise (void)
{
  var_number_t lcd_var_number;
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    // cruise function enable/disable
    case 0:
    
      lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      if (configuration_variables.ui8_cruise_function_enabled && (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled))
      {
        lcd_enable_cruise_symbol (1);
      }
      
      if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
      }
      
    break;
    
    // enable/disable target speed for cruise 
    case 1:
    
      lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_set_target_speed_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_cruise_symbol (1);
      
      if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
      }
      
    break;
    
    // set cruise target speed
    case 2:
      
      // set target cruise speed in either imperial or metric units
      if (configuration_variables.ui8_units_type)
      {
        // imperial
        lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_target_speed_mph;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 62; // needs to be 1.6 times smaller than metric max value
        lcd_var_number.ui32_min_value = 6;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
      
        lcd_enable_mph_symbol (1);
        lcd_enable_cruise_symbol (1);
        
        // convert imperial to metric and save 
        configuration_variables.ui8_cruise_function_target_speed_kph = (uint8_t) (((float) configuration_variables.ui8_cruise_function_target_speed_mph) * 1.6);
      }
      else
      {
        // metric
        lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_target_speed_kph;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 99; // needs to be smaller than 100 or else value > digits on display
        lcd_var_number.ui32_min_value = 9;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
        
        lcd_enable_kmh_symbol (1);
        lcd_enable_cruise_symbol (1);
      }
      
    break;
  
    // show cruise function target speed enable/disable
    case 3:
    
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_cruise_function_set_target_speed;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_cruise_symbol (1);
      
      if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
      { 
        lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
      }
      
    break;
  }
  
  submenu_state_controller(3);
}


void lcd_execute_menu_config_main_screen_setup (void)
{
  var_number_t lcd_var_number;

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // enable/disable show of distance data in odometer field
    case 0:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_distance_data_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break; 
    
    // enable/disable show of battery SOC data in odometer field
    case 1:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_battery_SOC_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of battery voltage or current in odometer field
    case 2:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_battery_state_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of pedal data in odometer field
    case 3:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_pedal_data_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of energy data in odometer field
    case 4:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_energy_data_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of time measurement in odometer field
    case 5:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_time_measurement_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of wheel speed in odometer field
    case 6:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_wheel_speed_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // enable/disable show of motor temperature in odometer field
    case 7:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_motor_temperature_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // enable/disable show of cruise function set target speed
    case 8:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_cruise_function_set_target_speed;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // enable/disable quick set power menu
    case 9:
      lcd_var_number.p_var_number = &configuration_variables.ui8_main_screen_power_menu_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // temperature field setup
    case 10:
      lcd_var_number.p_var_number = &configuration_variables.ui8_temperature_field_state;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 6;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
  }
  
  submenu_state_controller(10);

  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
}


void lcd_execute_menu_config_submenu_motor_startup_power_boost (void)
{
  var_number_t lcd_var_number;
  uint8_t ui8_temp;

  // feature enable or disable
  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_startup_motor_power_boost_feature_enabled;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
  }
  // enabled on startup when wheel speed is zero or always when cadence is zero
  else if (ui8_lcd_menu_config_submenu_state == 1)
  {
    ui8_temp = configuration_variables.ui8_startup_motor_power_boost_state & 1;
    lcd_var_number.p_var_number = &ui8_temp;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);

    if(ui8_temp) { configuration_variables.ui8_startup_motor_power_boost_state |= 1; }
    else { configuration_variables.ui8_startup_motor_power_boost_state &= ~1; }
  }
  // limit to max power
  else if (ui8_lcd_menu_config_submenu_state == 2)
  {
    ui8_temp = (configuration_variables.ui8_startup_motor_power_boost_state & 2) >> 1;
    lcd_var_number.p_var_number = &ui8_temp;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);

    if(ui8_temp) { configuration_variables.ui8_startup_motor_power_boost_state |= 2; }
    else { configuration_variables.ui8_startup_motor_power_boost_state &= ~2; }
  }
  // startup motor power boost time
  else if (ui8_lcd_menu_config_submenu_state == 3)
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_startup_motor_power_boost_time;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 1;
    lcd_var_number.ui32_max_value = 255;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
  }
  // startup motor power boost fade time
  else if (ui8_lcd_menu_config_submenu_state == 4)
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_startup_motor_power_boost_fade_time;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 1;
    lcd_var_number.ui32_max_value = 255;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
  }
  // value of each assist level factor for power boost
  else
  {
    lcd_var_number.p_var_number = &configuration_variables.ui8_startup_motor_power_boost_factor[(ui8_lcd_menu_config_submenu_state - 5)];
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 1;
    lcd_var_number.ui32_max_value = 255;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol(1);
      lcd_print(ui8_lcd_menu_config_submenu_state - 4, ASSIST_LEVEL_FIELD, 1);
    }
  }

  submenu_state_controller(configuration_variables.ui8_number_of_assist_levels + 4);
  
  if (ui8_lcd_menu_config_submenu_state < 5 && (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled))
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
}


void lcd_execute_menu_config_submenu_motor_temperature (void)
{
  var_number_t lcd_var_number;
  uint16_t ui16_temp;

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // motor voltage type
    case 0:
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_type;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 3;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // motor power limit
    case 1:
      ui16_temp = ((uint16_t) configuration_variables.ui8_target_max_battery_power_div25) * 25;
      lcd_var_number.p_var_number = &ui16_temp;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 1; // needs to be for BATTERY_POWER_FIELD
      lcd_var_number.ui32_max_value = 1900;
      lcd_var_number.ui32_min_value = 0;

      if (configuration_variables.ui8_target_max_battery_power_div25 < 10)
      {
        lcd_var_number.ui32_increment_step = 25;
      }
      else
      {
        lcd_var_number.ui32_increment_step = 50;
      }

      lcd_var_number.ui8_odometer_field = BATTERY_POWER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      configuration_variables.ui8_target_max_battery_power_div25 = (uint8_t) (ui16_temp / 25);
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_enable_w_symbol (1);
        lcd_enable_motor_symbol (1);
      }

    break;
  
    // ramp up, amps per second
    case 2:
      lcd_var_number.p_var_number = &configuration_variables.ui8_ramp_up_amps_per_second_x10;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 4;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // motor assistance startup without pedal rotation
    case 3:
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // limit motor temperature or throttle enable/disable 
    case 4:
      lcd_var_number.p_var_number = &configuration_variables.ui8_temperature_limit_feature_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 2;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // motor temperature limit min
    case 5:
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_temperature_min_value_to_limit;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 110;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_temperature_degrees_symbol (1);

    break;

    // motor temperature limit max
    case 6:
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_temperature_max_value_to_limit;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 110;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_temperature_degrees_symbol (1);
    
    break;
  }

  submenu_state_controller(6);

  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
}


void lcd_execute_menu_config_submenu_street_mode (void)
{
  var_number_t lcd_var_number;
  uint16_t ui16_temp;

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // enable/disable street mode
    case 0:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_function_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // enable street mode on system startup
    case 1:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_enabled_on_startup;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // street mode speed limit
    case 2:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_speed_limit;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 99;
      lcd_var_number.ui32_min_value = 1;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_kmh_symbol (1);
    break;

    // enable/disable street mode power limit
    case 3:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_power_limit_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // street mode power limit
    case 4:
      ui16_temp = ((uint16_t) configuration_variables.ui8_street_mode_power_limit_div25) * 25;
      
      lcd_var_number.p_var_number = &ui16_temp;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1900;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 25;
      lcd_var_number.ui8_odometer_field = BATTERY_POWER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      configuration_variables.ui8_street_mode_power_limit_div25 = (uint8_t) (ui16_temp / 25);
      
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_enable_w_symbol (1);
        lcd_enable_motor_symbol (1);
      }
      
    break;
    
    // enable/disable throttle during street mode
    case 5:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_throttle_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
  }
  
  submenu_state_controller(5);

  if (ui8_lcd_menu_config_submenu_state != 2 && (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled))
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
}


void lcd_execute_menu_config_submenu_technical (void)
{
  switch (ui8_lcd_menu_config_submenu_state)
  {
    case 0:
      lcd_print(motor_controller_data.ui8_adc_throttle, ODOMETER_FIELD, 0);
    break;

    case 1:
      lcd_print(motor_controller_data.ui8_throttle, ODOMETER_FIELD, 0);
    break;

    case 2:
      lcd_print(motor_controller_data.ui8_adc_pedal_torque_sensor, ODOMETER_FIELD, 0);
    break;

    case 3:
      lcd_print(motor_controller_data.ui8_pedal_torque_sensor, ODOMETER_FIELD, 0);
    break;

    case 4:
      lcd_print(motor_controller_data.ui8_pedal_cadence, ODOMETER_FIELD, 0);
    break;

    case 5:
      lcd_print(motor_controller_data.ui8_pedal_human_power, ODOMETER_FIELD, 0);
    break;

    case 6:
      lcd_print(motor_controller_data.ui8_duty_cycle, ODOMETER_FIELD, 0);
    break;

    case 7:
      lcd_print(motor_controller_data.ui16_motor_speed_erps, ODOMETER_FIELD, 0);
    break;

    case 8:
      lcd_print(motor_controller_data.ui8_foc_angle, ODOMETER_FIELD, 0);
    break;
  }

  submenu_state_controller(8);

  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
}


void lcd_execute_menu_config_power (void)
{
  var_number_t lcd_var_number;
  uint16_t ui16_temp;
  
  // enable change of variables
  ui8_lcd_menu_config_submenu_change_variable_enabled = 1;

  ui16_temp = ((uint16_t) configuration_variables.ui8_target_max_battery_power_div25) * 25;
  lcd_var_number.p_var_number = &ui16_temp;
  lcd_var_number.ui8_size = 16;
  lcd_var_number.ui8_decimal_digit = 1; // needs to be for BATTERY_POWER_FIELD
  lcd_var_number.ui32_max_value = 1900;
  lcd_var_number.ui32_min_value = 0;

  if (configuration_variables.ui8_target_max_battery_power_div25 < 10)
  {
    lcd_var_number.ui32_increment_step = 25;
  }
  else
  {
    lcd_var_number.ui32_increment_step = 50;
  }

  lcd_var_number.ui8_odometer_field = BATTERY_POWER_FIELD;
  lcd_configurations_print_number(&lcd_var_number);
  configuration_variables.ui8_target_max_battery_power_div25 = (uint8_t) (ui16_temp / 25);
  
  lcd_enable_w_symbol (1);
  lcd_enable_motor_symbol (1);
  
  // leave config power menu with a long onoff click
  if (ONOFF_LONG_CLICK)
  {
    // disable change of variables
    ui8_lcd_menu_config_submenu_change_variable_enabled = 0;

    // save the updated variables to EEPROM
    eeprom_write_variables ();
    
    // change to main menu
    ui8_lcd_menu = MAIN_MENU;
  }
}


void power_off_timer (void)
{
  static uint16_t ui16_seconds_since_power_on_offset;
  
  // check if automatic power off management is configured to any value
  if (configuration_variables.ui8_lcd_power_off_time_minutes != 0)
  {
    // check if there is system activity
    if ((motor_controller_data.ui16_wheel_speed_x10 > 0) ||     // wheel speed > 0
        (motor_controller_data.ui8_battery_current_x5 > 0) ||   // battery current > 0
        (motor_controller_data.ui8_braking) ||                  // braking
        (buttons_get_events ()))                                // any button active
    {
      // reset offset
      ui16_seconds_since_power_on_offset = ui16_seconds_since_power_on;
    }
    else 
    {
      // check if system has been inactive over or equal to the configured threshold time
      if (ui16_seconds_since_power_on - ui16_seconds_since_power_on_offset >= (configuration_variables.ui8_lcd_power_off_time_minutes * 60))
      {
        // power off system and save variables to EEPROM
        lcd_power_off (1);
      }
    }
  }
}


void temperature (void)
{
  // if motor current is being limited due to temperature, force showing temperature!!
  if (motor_controller_data.ui8_temperature_current_limiting_value < 255)
  {
    if (ui8_lcd_menu_flash_state_temperature)
    {
      lcd_print(motor_controller_data.ui8_motor_temperature, TEMPERATURE_FIELD, 1);
      lcd_enable_temperature_degrees_symbol (1);
    }
  }
  else
  {
    switch (configuration_variables.ui8_temperature_field_state)
    {
      // show motor temperature
      case 1:
        // if function is enabled -> display motor temperature
        if (configuration_variables.ui8_temperature_limit_feature_enabled == 1)
        {
          lcd_print (motor_controller_data.ui8_motor_temperature, TEMPERATURE_FIELD, 1);
          lcd_enable_temperature_degrees_symbol (1);
        }
      break;
      
      // show battery state of charge watt-hours
      case 2:
        lcd_print (ui16_battery_soc_watts_hour, TEMPERATURE_FIELD, 1);
      break;
      
      // battery voltage
      case 3:
        lcd_print (ui16_battery_voltage_filtered_x10/10, TEMPERATURE_FIELD, 1);
      break;
      
      // battery current
      case 4:
        lcd_print (ui16_battery_current_filtered_x5/5, TEMPERATURE_FIELD, 1);
      break;
      
      // pedal cadence
      case 5:
        lcd_print (ui8_pedal_cadence_filtered, TEMPERATURE_FIELD, 1);
      break;
      
      // average wheel speed since power on
      case 6:
        // check in what unit of measurement to display average wheel speed
        if (configuration_variables.ui8_units_type)
        {
          // imperial
          lcd_print (((float) ui8_average_measured_wheel_speed_x10/16), TEMPERATURE_FIELD, 1);
        }
        else
        {
          // metric
          lcd_print (ui8_average_measured_wheel_speed_x10/10, TEMPERATURE_FIELD, 1);
        }
      break;
      
      // show nothing
      default:
      break;
    }
  }
}


void time_measurement (void)
{
  // update time
  ui16_seconds_since_power_on += ui8_second;
  ui8_second_TM += ui8_second;
  configuration_variables.ui8_total_second_TTM += ui8_second;
  
  // reset elapsed seconds
  ui8_second = 0;
  
  // time measurement since power on
  
    // check if overflow
    if (ui8_second_TM >= 60)
    {
      // reset second
      ui8_second_TM = 0;
      
      // increment minute
      ui16_minute_TM++;
    }
  
  
  // total time measurement since last reset (TTM)
    
    // check if overflow
    if (configuration_variables.ui8_total_second_TTM >= 60)
    {
      // reset second
      configuration_variables.ui8_total_second_TTM = 0;
      
      // increment minute
      configuration_variables.ui8_total_minute_TTM++;
      
      // check if overflow
      if (configuration_variables.ui8_total_minute_TTM >= 60)
      {
        // reset minute
        configuration_variables.ui8_total_minute_TTM = 0;
        
        // increment hour
        configuration_variables.ui16_total_hour_TTM++;
      }
    }
    
  // display either TM or TTM
  if (configuration_variables.ui8_time_measurement_field_state)
  {
    lcd_enable_colon_symbol(1);
    lcd_enable_tm_symbol(1);
    lcd_print(ui8_second_TM, TIME_SECOND_FIELD, 0);
    lcd_print(ui16_minute_TM, TIME_MINUTE_FIELD, 0);
  }
  else
  {
    lcd_enable_colon_symbol(1);
    lcd_enable_ttm_symbol(1);
    lcd_print(configuration_variables.ui8_total_minute_TTM, TIME_SECOND_FIELD, 0);
    lcd_print(configuration_variables.ui16_total_hour_TTM, TIME_MINUTE_FIELD, 0);
  }
}


void energy_data (void)
{
  static uint8_t ui8_executed_on_startup;
  
  // reset watt-hour value if battery voltage is over threshold set from user, only executed once during startup
  if (!ui8_executed_on_startup)
  {
    ui8_executed_on_startup = 1;
    
    if (((uint32_t) motor_controller_data.ui16_adc_battery_voltage * ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000) > ((uint32_t) configuration_variables.ui16_battery_voltage_reset_wh_counter_x10 * 1000))
    {
      configuration_variables.ui32_wh_x10_offset = 0;
    }
  }

  // calculate watt-hours since power on
  ui32_wh_since_power_on_x10 = ui32_wh_sum_x5 / 18000; //  wh_sum_x5  /  (3600 * 5)
  
  // calculate watt-hours since last full charge
  ui32_wh_x10 = configuration_variables.ui32_wh_x10_offset + ui32_wh_since_power_on_x10;
  
  // calculate average watt-hour consumption per distance traveled, since power on. Check to avoid zero division
  if (configuration_variables.ui16_distance_since_power_on_x10 == 0)
  {
    ui16_average_energy_consumption_since_power_on_x10 = 0;
  }
  else
  {
    // divide watt-hours with distance since power on and save in variable
    ui16_average_energy_consumption_since_power_on_x10 = (ui32_wh_since_power_on_x10 * 10) / configuration_variables.ui16_distance_since_power_on_x10; // multiply numerator with 10 to retain decimal 
  }
  
  // calculate estimated range since power on by dividing watt-hours remaining with average consumption. Check to avoid zero division
  if (ui16_average_energy_consumption_since_power_on_x10 == 0)
  {
    ui16_estimated_range_since_power_on_x10 = 10000;
  }
  else if (ui32_wh_x10 > configuration_variables.ui32_wh_x10_100_percent)
  {
    ui16_estimated_range_since_power_on_x10 = 0;
  }
  else
  {
    ui16_estimated_range_since_power_on_x10 = ((configuration_variables.ui32_wh_x10_100_percent - ui32_wh_x10) * 10) / ui16_average_energy_consumption_since_power_on_x10; // multiply numerator with 10 to retain decimal 
  }
  
  // limit estimated range depending on unit of measurement, looks nicer this way
  if (configuration_variables.ui8_units_type)
  {
    // imperial units
    if (ui16_estimated_range_since_power_on_x10 > 9599) { ui16_estimated_range_since_power_on_x10 = 9599; }
  }
  else
  {
    // metric units
    if (ui16_estimated_range_since_power_on_x10 > 9999) { ui16_estimated_range_since_power_on_x10 = 9999; }
  }
}


void battery_soc (void)
{
  static uint8_t ui8_timmer_counter;
  static uint8_t ui8_battery_state_of_charge;
  uint8_t ui8_battery_cells_number_x10;

  // update battery level value every 100 ms -> 10 times per second. This helps to filter the fast changing values
  if (++ui8_timmer_counter >= 10)
  {
    ui8_timmer_counter = 0;

    // to keep same scale as voltage of x10
    ui8_battery_cells_number_x10 = configuration_variables.ui8_battery_cells_number * 10;

    if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_83))) { ui8_battery_state_of_charge = 4; } // 4 bars | full
    else if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_50))) { ui8_battery_state_of_charge = 3; } // 3 bars
    else if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_17))) { ui8_battery_state_of_charge = 2; } // 2 bars
    else if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_0))) { ui8_battery_state_of_charge = 1; } // 1 bar
    else { ui8_battery_state_of_charge = 0; } // flashing
  }

  /*
    ui8_lcd_frame_buffer[23] |= 16;  // empty
    ui8_lcd_frame_buffer[23] |= 128; // bar number 1
    ui8_lcd_frame_buffer[23] |= 1;   // bar number 2
    ui8_lcd_frame_buffer[23] |= 64;  // bar number 3
    ui8_lcd_frame_buffer[23] |= 32;  // bar number 4
    */

  // first clean battery symbols
  ui8_lcd_frame_buffer[23] &= ~241;

  switch (ui8_battery_state_of_charge)
  {
    case 0:
      // empty, so flash the empty battery symbol
      if (ui8_lcd_menu_flash_state)
      {
        ui8_lcd_frame_buffer[23] |= 16;
      }
    break;

    case 1:
      ui8_lcd_frame_buffer[23] |= 144;
    break;

    case 2:
      ui8_lcd_frame_buffer[23] |= 145;
    break;

    case 3:
      ui8_lcd_frame_buffer[23] |= 209;
    break;

    case 4:
      ui8_lcd_frame_buffer[23] |= 241;
    break;
  }
}


void calc_battery_soc (void)
{
  uint16_t ui16_fluctuate_battery_voltage_x10;
  uint32_t ui32_temp;
  static uint8_t ui8_update_counter;

  if (++ui8_update_counter > 10) // update battery level value every 100 ms -> 10. This helps to filter the fast changing values
  {
    // reset counter
    ui8_update_counter = 0;
    
    // calculate battery voltage that takes into consideration current and internal battery pack resistance
    ui16_fluctuate_battery_voltage_x10 = (uint16_t) ((((uint32_t) configuration_variables.ui16_battery_pack_resistance_x1000) * ((uint32_t) ui16_battery_current_filtered_x5)) / ((uint32_t) 500));
    
    // add voltage value
    ui16_battery_voltage_soc_x10 = ui16_battery_voltage_filtered_x10 + ui16_fluctuate_battery_voltage_x10;
  }

  // calculate battery SOC percentage
  ui32_temp = ui32_wh_x10 * 100;
  
  if (configuration_variables.ui32_wh_x10_100_percent > 0)
  {
    ui32_temp /= configuration_variables.ui32_wh_x10_100_percent;
  }
  else
  {
    ui32_temp = 0;
  }

  if (configuration_variables.ui8_battery_SOC_function_enabled == 1) // SOC from 100 to 0 percent (remaining capacity in percent)
  {
    // limit percentage to 100
    if (ui32_temp > 100)
    {
      ui32_temp = 100;
    }
    
    // calculate and set remaining percentage
    ui16_battery_soc_watts_hour = 100 - ui32_temp;
  }
  else if (configuration_variables.ui8_battery_SOC_function_enabled == 2) // SOC from 0 to 100 percent (consumed capacity in percent)
  {
    // set consumed percentage
    ui16_battery_soc_watts_hour = ui32_temp;
  }
}


void power(void)
{
  lcd_print(ui16_battery_power_filtered, BATTERY_POWER_FIELD, 0);
  lcd_enable_motor_symbol(1);
  lcd_enable_w_symbol(1);
}


void assist_level_state (void)
{
  if (UP_CLICK)
  {
    // increment assist level and check if is out of bounds
    if (++configuration_variables.ui8_assist_level > configuration_variables.ui8_number_of_assist_levels)
    {
      // set assist level to max
      configuration_variables.ui8_assist_level = configuration_variables.ui8_number_of_assist_levels; 
    }
  }
  
  if (DOWN_CLICK)
  {
    // check if assist level is out of bounds
    if (configuration_variables.ui8_assist_level > 0)
    {
      // decrement assist level
      --configuration_variables.ui8_assist_level;
    }
  }

  // display assist level
  lcd_print (configuration_variables.ui8_assist_level, ASSIST_LEVEL_FIELD, 1);

  // if street mode is disabled display "assist" symbol
  if (!ui8_street_mode_enabled)
  {
    lcd_enable_assist_symbol (1);
  }
}


void lights_state (void)
{
  if (UP_LONG_CLICK)
  {
    // toggle light state and display backlight
    ui8_lights_state = !ui8_lights_state;
    motor_controller_data.ui8_lights = ui8_lights_state;
  }
  
  // set backlight brightness
  if (ui8_lights_state == 0) { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_off_brightness); }
  else { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_on_brightness); }
  
  // enable light symbol on display
  lcd_enable_lights_symbol(ui8_lights_state);
}


void walk_assist_state (void)
{
  static uint8_t ui8_walk_assist_activated;
  static uint8_t ui8_cruise_activated;
  static uint8_t ui8_long_hold_down_button;
  #define WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10 80 // 8.0 km/h
  
  if (DOWN_LONG_CLICK)
  {
    ui8_long_hold_down_button = 1;
  }
  
  if (ui8_long_hold_down_button)
  {
    // if down button is pressed
    if (buttons_get_down_state ())
    {
      // enable walk assist or cruise function depending on speed and if function is enabled, also check if the other function was activetad first
      if (motor_controller_data.ui16_wheel_speed_x10 < WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10 && configuration_variables.ui8_walk_assist_function_enabled && ui8_cruise_activated == 0)
      {
        // enable walk assist function because: the wheel speed is less than threshold speed, the function is enabled and cruise was not activated first
        lcd_enable_walk_symbol (1);
        motor_controller_data.ui8_walk_assist_level = 1;
        
        // set flag to indicate that walk assist was activated first during this button event
        ui8_walk_assist_activated = 1;
      }
      else if (motor_controller_data.ui16_wheel_speed_x10 > WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10 && configuration_variables.ui8_cruise_function_enabled && ui8_walk_assist_activated == 0)
      {
        // enable cruise function because: the wheel speed is more than threshold speed, the function is enabled and walk assist was not activated first
        lcd_enable_cruise_symbol (1);
        motor_controller_data.ui8_walk_assist_level = 1;
        
        // set flag to indicate that cruise was activated first during this button event
        ui8_cruise_activated = 1;
      }
      else
      {
        // disable walk assist or cruise function
        motor_controller_data.ui8_walk_assist_level = 0;
      }
    }
    else // button not longer pressed
    {
      // disable walk assist or cruise function
      motor_controller_data.ui8_walk_assist_level = 0;
      
      // reset flags for walk assist and cruise activated
      ui8_walk_assist_activated = 0;
      ui8_cruise_activated = 0;
      
      // reset button event flag
      ui8_long_hold_down_button = 0;
    }
  }
  else
  {
    // disable walk assist or cruise function
    motor_controller_data.ui8_walk_assist_level = 0;
    
    // reset flags for walk assist and cruise activated
    ui8_walk_assist_activated = 0;
    ui8_cruise_activated = 0;
  }
}


void street_mode (void)
{
  static uint8_t ui8_street_mode_assist_symbol_state;
  static uint8_t ui8_street_mode_assist_symbol_state_counter;
  static uint8_t ui8_executed_on_startup;
  
  if (configuration_variables.ui8_street_mode_function_enabled) 
  {
    // enable street mode if user has enabled street mode on startup
    if (!ui8_executed_on_startup)
    {
      ui8_executed_on_startup = 1;
      
      if (configuration_variables.ui8_street_mode_enabled_on_startup) 
      {
        ui8_street_mode_enabled = 1;
        motor_controller_data.ui8_street_mode_enabled = 1;
      }
    }
    
    if (ONOFF_DOWN_LONG_CLICK)
    {
      ui8_street_mode_enabled = !ui8_street_mode_enabled;
      
      motor_controller_data.ui8_street_mode_enabled = ui8_street_mode_enabled;
    }
    
    if (ui8_street_mode_enabled) 
    {
      if (++ui8_street_mode_assist_symbol_state_counter > 45)
      {
        ui8_street_mode_assist_symbol_state_counter = 0;
        
        ui8_street_mode_assist_symbol_state = !ui8_street_mode_assist_symbol_state;
      }

      lcd_enable_assist_symbol (ui8_street_mode_assist_symbol_state);
    }
  }
}


void brake (void)
{
  lcd_enable_brake_symbol (motor_controller_data.ui8_braking);
}


void odometer_increase_field_state (void)
{
  // increment odometer field state and check if out of bounds
  if (++configuration_variables.ui8_odometer_field_state > 8) // case 0 -> case 8, 9 odometer field states
  {
    // reset odometer field state
    configuration_variables.ui8_odometer_field_state = 0;
  }
}


void odometer_start_show_field_number (void)
{
  ui8_start_odometer_show_field_number = 1;
  ui8_odometer_show_field_number_counter = 0;
}


uint8_t reset_variable_check (void)
{
  static uint8_t ui8_odometer_reset_distance_counter_state;
  static uint16_t ui16_odometer_reset_distance_counter;
  
  if (DOWN_CLICK_LONG_CLICK)
  {
    // start counting to reset variable
    ui8_odometer_reset_distance_counter_state = 1;
    
    // reset counter for variable reset
    ui16_odometer_reset_distance_counter = 0;
  }
  
  if (ui8_odometer_reset_distance_counter_state)
  {
    if (buttons_get_down_state ())
    {
      // count time, after limit, reset everything
      if (++ui16_odometer_reset_distance_counter > 300)
      {
        // reset counter state
        ui8_odometer_reset_distance_counter_state = 0;
        
        // reset variable
        return 1;
      }
      
      // check if flash state for variable flashing
      if (ui8_lcd_menu_flash_state)
      {
        // do not display variable
        return 2;
      }
    }
    else // user is not pressing the down button anymore
    {
      ui8_odometer_reset_distance_counter_state = 0;
    }
  }
  
  // display variable as usual
  return 0;
}


void odometer (void)
{
  if (ONOFF_CLICK)
  {
    // increment odometer field state
    odometer_increase_field_state ();
    
    // show field number
    odometer_start_show_field_number ();
  }

  // if there are errors, show the error number on odometer field instead of any other information
  if (motor_controller_data.ui8_error_states != NO_ERROR)
  {
    if (ui8_lcd_menu_flash_state)
    {
      lcd_print(motor_controller_data.ui8_error_states, ODOMETER_FIELD, 0);
    }
  }
  else
  {
    switch (configuration_variables.ui8_odometer_field_state)
    {
      // trip distance 
      case 0:
        
        // check if user has disabled to show distance data in the odometer field
        if (configuration_variables.ui8_show_distance_data_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_0, 2);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_0;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_0)
        {
          // trip distance
          case 0:
          
            switch (reset_variable_check ())
            {
              // display trip distance
              case 0:
                if (configuration_variables.ui8_units_type)
                {
                  // imperial units
                  lcd_print (((float) configuration_variables.ui32_trip_x10 / 1.6), ODOMETER_FIELD, 1);
                  lcd_enable_mil_symbol (1);
                }
                else
                {
                  // metric units
                  lcd_print (configuration_variables.ui32_trip_x10, ODOMETER_FIELD, 1);
                  lcd_enable_km_symbol (1);
                }
              break;
              
              // reset trip distance
              case 1:
                configuration_variables.ui32_trip_x10 = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;

          // distance since power on
          case 1:
          
            // display distance since power on in either imperial or metric units
            if (configuration_variables.ui8_units_type)
            {
              // imperial units
              lcd_print(((float) configuration_variables.ui16_distance_since_power_on_x10 / 1.6), ODOMETER_FIELD, 1);
              // lcd_enable_dst_symbol (1); TODO: this fails, the symbol just work when we set it to 1 AND speed field number is equal or higher than 10.0. Seems the 3rd digit at left is needed.
              lcd_enable_mil_symbol (1);
            }
            else
            {
              // metric units
              lcd_print((uint32_t) configuration_variables.ui16_distance_since_power_on_x10, ODOMETER_FIELD, 1);
              // lcd_enable_dst_symbol (1); TODO: this fails, the symbol just work when we set it to 1 AND speed field number is equal or higher than 10.0. Seems the 3rd digit at left is needed.
              lcd_enable_km_symbol (1);
            }
            
          break;

          // odometer
          case 2:
          
            switch (reset_variable_check ())
            {
              // display odometer distance
              case 0:
                if (configuration_variables.ui8_units_type)
                {
                  // imperial units
                  lcd_print(((float) configuration_variables.ui32_odometer_x10 / 1.6), ODOMETER_FIELD, 1);
                  lcd_enable_odo_symbol (1);
                  lcd_enable_mil_symbol (1);
                }
                else
                {
                  // metric units
                  lcd_print(configuration_variables.ui32_odometer_x10, ODOMETER_FIELD, 1);
                  lcd_enable_odo_symbol (1);
                  lcd_enable_km_symbol (1);
                }
              break;
              
              // reset odometer distance
              case 1:
                configuration_variables.ui32_odometer_x10 = 0;
              break;
              
              // display nothing
              default:
              break;
            }
          
          break;
        }

      break; // end of distance

      // battery SOC
      case 1:
      
        // check if user has disabled to show battery state of charge in the odometer field
        if (configuration_variables.ui8_battery_SOC_function_enabled == 0 || configuration_variables.ui8_show_battery_SOC_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }

        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_1, 1);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_1;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_1)
        {
          // battery SOC in percentage
          case 0:
            lcd_print(ui16_battery_soc_watts_hour, ODOMETER_FIELD, 0);
          break;

          // consumed watt-hours
          case 1:
            lcd_print(ui32_wh_x10, ODOMETER_FIELD, 1);
          break;
        }
      
      break; // end of battery SOC

      // battery state
      case 2:
      
        // check if user has disabled to show battery voltage or current in the odometer field
        if (configuration_variables.ui8_show_battery_state_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
      
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_2, 1);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_2;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_2)
        {
          // voltage value
          case 0:
            lcd_print(ui16_battery_voltage_filtered_x10, ODOMETER_FIELD, 1);
            lcd_enable_vol_symbol (1);
          break;

          // current value
          case 1:
            lcd_print(ui16_battery_current_filtered_x5 << 1, ODOMETER_FIELD, 1);
          break;
        }
      
      break; // end of battery state

      // pedal data
      case 3:
      
        // check if user has disabled to show pedal data in the odometer field
        if (configuration_variables.ui8_show_pedal_data_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_3, 2);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_3;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_3)
        {
          // pedal power
          case 0:
            lcd_print (ui16_pedal_power_filtered, ODOMETER_FIELD, 0);
          break;

          // pedal cadence
          case 1:
            lcd_print (ui8_pedal_cadence_filtered, ODOMETER_FIELD, 0);
          break;

          // pedal torque
          case 2:
            lcd_print (ui16_pedal_torque_filtered, ODOMETER_FIELD, 0);
          break;
        }
        
      break; // end of pedals

      // energy data
      case 4:
      
        // check if user has disabled to show energy data in the odometer field
        if (configuration_variables.ui8_show_energy_data_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_4, 1);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_4;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_4)
        {
          case 0:
            
            // display energy consumption in either imperial or metric units
            if (configuration_variables.ui8_units_type)
            {
              // imperial units
              lcd_print((float) ui16_average_energy_consumption_since_power_on_x10 * 1.6, ODOMETER_FIELD, 1);
            }
            else
            {
              // metric units
              lcd_print(ui16_average_energy_consumption_since_power_on_x10, ODOMETER_FIELD, 1);
            }
          
          break;

          case 1:
            
            // check if user has disabled battery capacity function
            if (configuration_variables.ui8_battery_SOC_function_enabled == 0)
            {
              // function not enabled, go to next sub menu
              configuration_variables.ui8_odometer_sub_field_state_4 = 0;
              
              break;
            }
          
            // display estimated range since power on in either imperial or metric units
            if (configuration_variables.ui8_units_type)
            {
              // imperial units
              lcd_print((float) ui16_estimated_range_since_power_on_x10 / 1.6, ODOMETER_FIELD, 1);
              lcd_enable_mil_symbol (1);
            }
            else
            {
              // metric units
              lcd_print(ui16_estimated_range_since_power_on_x10, ODOMETER_FIELD, 1);
              lcd_enable_km_symbol (1);
            }
            
          break;
        }
        
      break; // end of energy data
      
      // time measurement
      case 5:
      
        // check if user has disabled to show time measurement in the odometer field
        if (configuration_variables.ui8_show_time_measurement_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_5, 1);

        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_5;

        switch (configuration_variables.ui8_odometer_sub_field_state_5)
        {
          // time measurement since power on (TM)
          case 0:
            
            // display time measurement since power on (TM) in time measurement field
            configuration_variables.ui8_time_measurement_field_state = 1;
            
            switch (reset_variable_check ())
            {
              // display total minutes since power on (TM)
              case 0:
                lcd_print(ui16_minute_TM, ODOMETER_FIELD, 0);
              break;
              
              // reset time measurement since power on (TM)
              case 1:
                ui8_second_TM = 0;
                ui16_minute_TM = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;

          // time measurement since last reset (TTM)
          case 1:
            
            // display total time measurement since last reset (TTM) in time measurement field
            configuration_variables.ui8_time_measurement_field_state = 0;
            
            switch (reset_variable_check ())
            {
              // display total hours passed from TTM
              case 0:
                lcd_print(configuration_variables.ui16_total_hour_TTM, ODOMETER_FIELD, 0);
              break;
              
              // reset total time measurement since last reset (TTM)
              case 1:
                configuration_variables.ui8_total_second_TTM = 0;
                configuration_variables.ui8_total_minute_TTM = 0;
                configuration_variables.ui16_total_hour_TTM = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;
        }
        
      break; // end of time measurement
    
      case 6: // wheel speed
      
        // check if user has disabled to show wheel speed in the odometer field
        if (configuration_variables.ui8_show_wheel_speed_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_6, 2);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_6;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_6)
        {
          // wheel speed
          case 0:
          
            // set wheel speed field state
            configuration_variables.ui8_wheel_speed_field_state = 0;
            
            // display wheel speed in either imperial or metric units in the odometer field
            if (configuration_variables.ui8_units_type)
            {
              // imperial
              lcd_print(((float) motor_controller_data.ui16_wheel_speed_x10 / 1.6), ODOMETER_FIELD, 1);
            }
            else
            {
              // metric
              lcd_print(motor_controller_data.ui16_wheel_speed_x10, ODOMETER_FIELD, 1);
            }
          break;
          
          // average wheel speed since power on
          case 1:
          
            // set wheel speed field state
            configuration_variables.ui8_wheel_speed_field_state = 1;
            
            // enable average speed symbol
            lcd_enable_avs_symbol (1);
            
            // display average wheel speed in either imperial or metric units in the odometer field
            if (configuration_variables.ui8_units_type)
            {
              // imperial
              lcd_print(((float) ui8_average_measured_wheel_speed_x10 / 1.6), ODOMETER_FIELD, 1);
            }
            else
            {
              // metric
              lcd_print(ui8_average_measured_wheel_speed_x10, ODOMETER_FIELD, 1);
            }
          break;
          
          // maximum measured wheel speed since power on
          case 2:
          
            // set wheel speed field state
            configuration_variables.ui8_wheel_speed_field_state = 2;
            
            // enable max speed symbol
            lcd_enable_mxs_symbol (1);

            switch (reset_variable_check ())
            {
              // display max measured wheel speed in either imperial or metric units in the odometer field
              case 0:
                if (configuration_variables.ui8_units_type)
                {
                  // imperial
                  lcd_print(((float) ui8_max_measured_wheel_speed_x10 / 1.6), ODOMETER_FIELD, 1);
                }
                else
                {
                  // metric
                  lcd_print(ui8_max_measured_wheel_speed_x10, ODOMETER_FIELD, 1);
                }
              break;
              
              // reset maximum measured wheel speed since power on
              case 1:
                ui8_max_measured_wheel_speed_x10 = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;
        }
        
      break; // end of wheel speed
      
      // motor temperature
      case 7:
      
        // check if user has enabled temperature limit function and enabled to show the value in the odometer field
        if (configuration_variables.ui8_temperature_limit_feature_enabled == 0 || configuration_variables.ui8_show_motor_temperature_odometer_field == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
        
        lcd_print (motor_controller_data.ui8_motor_temperature, ODOMETER_FIELD, 0);

      break; // end of motor temperature
      
      // cruise
      case 8:
        
        // check if user has enabled the set target speed feature and also enabled to show cruise set target speed in the odometer field
        if (configuration_variables.ui8_cruise_function_set_target_speed_enabled == 0 || configuration_variables.ui8_show_cruise_function_set_target_speed == 0)
        {
          // increment odometer field state
          odometer_increase_field_state ();
          
          break;
        }
        
        // display cruise target speed in either imperial or metric units
        if (configuration_variables.ui8_units_type)
        {
          // imperial
          lcd_print ((configuration_variables.ui8_cruise_function_target_speed_mph * 10), ODOMETER_FIELD, 1);
        }
        else
        {
          // metric
          lcd_print ((configuration_variables.ui8_cruise_function_target_speed_kph * 10), ODOMETER_FIELD, 1);
        }
        
      break; // end of cruise
    }

    if (ui8_start_odometer_show_field_number)
    {
      // limit field number flashing after a certain time
      if (++ui8_odometer_show_field_number_counter > 200) // 200 -> 2.0 seconds
      {
        // reset counter
        ui8_odometer_show_field_number_counter = 0;
        
        // disable the flashing
        ui8_start_odometer_show_field_number = 0;
      }

    
      if (ui8_lcd_menu_flash_state)
      {
        ui8_odometer_show_field_number = ((configuration_variables.ui8_odometer_field_state + 1) * 10); // add units for show (x10)
        ui8_odometer_show_field_number += ui8_odometer_sub_field_state;

        lcd_print (ui8_odometer_show_field_number, WHEEL_SPEED_FIELD, 1);
      }
    }
  }
}


void wheel_speed (void)
{
  // check if wheel speed is higher than maximum measured wheel speed
  if (motor_controller_data.ui16_wheel_speed_x10 > ui8_max_measured_wheel_speed_x10)
  {
    // wheel speed is higher than maximum measured wheel speed so update variable
    ui8_max_measured_wheel_speed_x10 = motor_controller_data.ui16_wheel_speed_x10;
  }
  
  // calculate average wheel speed since power on in km/s
  float average_measured_wheel_speed_x10_temp = (float) configuration_variables.ui16_distance_since_power_on_x10 / (float) ui16_seconds_since_power_on;
  
  // convert to km/h
  average_measured_wheel_speed_x10_temp = average_measured_wheel_speed_x10_temp * 3600;
  
  // set calculated value to average wheel speed variable
  ui8_average_measured_wheel_speed_x10 = (uint8_t) average_measured_wheel_speed_x10_temp;
  
  
  // show wheel speed only when we should not start show odometer field number
  if (ui8_start_odometer_show_field_number == 0)
  {
    switch (configuration_variables.ui8_wheel_speed_field_state)
    {
      // display wheel speed
      case 0:
      
        if (configuration_variables.ui8_units_type)
        {
          lcd_print(((float) motor_controller_data.ui16_wheel_speed_x10 / 1.6), WHEEL_SPEED_FIELD, 1);
          lcd_enable_mph_symbol (1);
        }
        else
        {
          lcd_print(motor_controller_data.ui16_wheel_speed_x10, WHEEL_SPEED_FIELD, 1);
          lcd_enable_kmh_symbol (1);
        }
        
      break;
      
      // display average wheel speed since power on
      case 1:
      
        if (configuration_variables.ui8_units_type)
        {
          lcd_print(((float) ui8_average_measured_wheel_speed_x10 / 1.6), WHEEL_SPEED_FIELD, 1);
          lcd_enable_mph_symbol (1);
        }
        else
        {
          lcd_print(ui8_average_measured_wheel_speed_x10, WHEEL_SPEED_FIELD, 1);
          lcd_enable_kmh_symbol (1);
        }
        
      break;
      
      // display maximum measured wheel speed since power on
      case 2:
      
        if (configuration_variables.ui8_units_type)
        {
          lcd_print(((float) ui8_max_measured_wheel_speed_x10 / 1.6), WHEEL_SPEED_FIELD, 1);
          lcd_enable_mph_symbol (1);
        }
        else
        {
          lcd_print(ui8_max_measured_wheel_speed_x10, WHEEL_SPEED_FIELD, 1);
          lcd_enable_kmh_symbol (1);
        }
        
      break;
    }
  }
}


void lcd_clear (void)
{
  memset(ui8_lcd_frame_buffer, 0, LCD_FRAME_BUFFER_SIZE);
}

void lcd_set_frame_buffer (void)
{
  memset(ui8_lcd_frame_buffer, 255, LCD_FRAME_BUFFER_SIZE);
}

void lcd_update (void)
{
  ht1622_send_frame_buffer (ui8_lcd_frame_buffer);
}


void lcd_print(uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options)
{
/*   #define DECIMAL   1

  switch (ui8_lcd_field)
  {
    case ASSIST_LEVEL_FIELD:
    
      // first delete the field
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field]] &= NUMBERS_MASK;
      
      // extract first digit
      uint8_t ui8_digit = ui32_number % 10;
      
      // print digit in field
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field]] &= ui8_lcd_digit_mask[NUMBERS_MASK];   

    break;
    
    case ODOMETER_FIELD:
    
      // first delete the old number in the field and then print the new number, digit by digit
      for (uint8_t ui8_counter = 0; ui8_counter < 5; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if ((ui8_options == DECIMAL) && (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else if (ui8_counter > 1 && ui32_number == 0) // print empty (NUMBERS_MASK) if digit is zero
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
        }
        
        // shift number so next digit is prepared to be printed
        ui32_number >> 1;
      }
      
      // enable decimal point
      if (ui8_options == DECIMAL) { lcd_enable_odometer_point_symbol(1); }
      else { lcd_enable_odometer_point_symbol(0); }
      
    break;
    
    case TEMPERATURE_FIELD:
    
      for (uint8_t ui8_counter = 0; ui8_counter < 2; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if ((ui8_options == DECIMAL) && (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else if ((ui8_counter > 0) && (ui32_number == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
        }

        ui32_number >> 1;
      }
  
      // enable only the "1 symbol" if temperature is > 99
      if (ui32_number > 99) { lcd_enable_temperature_1_symbol(1); }
      else { lcd_enable_temperature_1_symbol(0); }
    
    break;
    
    case WHEEL_SPEED_FIELD:
  
      for (uint8_t ui8_counter = 0; ui8_counter < 3; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if ((ui8_options == DECIMAL) && (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else if (ui8_counter > 1 && ui32_number == 0) // print only first 2 zeros
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
        
        ui32_number >> 1;
      }
      
      // enable decimal point
      if (ui8_options == DECIMAL) { lcd_enable_wheel_speed_point_symbol(1); }
      else { lcd_enable_wheel_speed_point_symbol(0); }
      
    break;
    
    case BATTERY_POWER_FIELD:
    
      for (uint8_t ui8_counter = 0; ui8_counter < 3; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if (ui8_counter > 0 && ui32_number == 0) // print only first zero
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }

        ui32_number >> 1;
      }
      
      // enable the "1 symbol" if power is > 999
      if (ui32_number >= 1000) { lcd_enable_battery_power_1_symbol(1); }
      else { lcd_enable_battery_power_1_symbol(0); }
    
    break;
    
    case TIME_SECOND_FIELD:
    
      for (uint8_t ui8_counter = 0; ui8_counter < 2; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;
        
        ui8_lcd_frame_buffer[SECOND_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];

        ui32_number >> 1;
      }
      
    break;
    
    case TIME_MINUTE_FIELD:
    
      // first delete the old number in the field and then print the new number, digit by digit
      for (uint8_t ui8_counter = 0; ui8_counter < 3; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;
        
        if (ui8_counter > 0 && ui32_number == 0)
        {
          ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
        
        ui32_number >> 1;
      }
    
    break;
    
    default:
      // out of bounds
    break;
  } */
  
  uint8_t ui8_counter;
  uint8_t ui8_digit;
  
  // multiply the value by 10 to not show decimal digit if ...
  if( (ui8_options == 0) && (ui8_lcd_field != ASSIST_LEVEL_FIELD) && (ui8_lcd_field != BATTERY_POWER_FIELD) && (ui8_lcd_field != TIME_SECOND_FIELD) && (ui8_lcd_field != TIME_MINUTE_FIELD) )
  {
    ui32_number *= 10;
  }
  
  // first delete the field
  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    if (ui8_lcd_field == ASSIST_LEVEL_FIELD || ui8_lcd_field == ODOMETER_FIELD || ui8_lcd_field == TEMPERATURE_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;
    }

    // because the LCD mask/layout is different on some field, like numbers would be inverted
    if (ui8_lcd_field == WHEEL_SPEED_FIELD || ui8_lcd_field == BATTERY_POWER_FIELD || ui8_lcd_field == TIME_SECOND_FIELD || ui8_lcd_field == TIME_MINUTE_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;
    }

    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TEMPERATURE_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TIME_SECOND_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == TIME_MINUTE_FIELD) break;
  }

  // enable only the "1" if power is >= 1000
  if (ui8_lcd_field == BATTERY_POWER_FIELD)
  {
    if (ui32_number >= 1000) { lcd_enable_battery_power_1_symbol (1); }
    else { lcd_enable_battery_power_1_symbol (0); }
  }

  // enable only the "1" if temperature is >= 100
  if (ui8_lcd_field == TEMPERATURE_FIELD)
  {
    if (ui32_number >= 100) { lcd_enable_temperature_1_symbol (1); }
    else { lcd_enable_temperature_1_symbol (0); }
  }

  // do not show the point symbol if number*10 is integer
  if (ui8_options == 0)
  {
    if (ui8_lcd_field == ODOMETER_FIELD) { lcd_enable_odometer_point_symbol (0); }
    else if (ui8_lcd_field == WHEEL_SPEED_FIELD) { lcd_enable_wheel_speed_point_symbol (0); }
  }
  else
  {
    if (ui8_lcd_field == ODOMETER_FIELD) { lcd_enable_odometer_point_symbol (1); }
    else if (ui8_lcd_field == WHEEL_SPEED_FIELD) { lcd_enable_wheel_speed_point_symbol (1); }
  }

  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    ui8_digit = ui32_number % 10;

    if (ui8_lcd_field == ASSIST_LEVEL_FIELD || ui8_lcd_field == ODOMETER_FIELD || ui8_lcd_field == TEMPERATURE_FIELD)
    {

      if ((ui8_options == 0) && (ui8_counter == 0))
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      // print empty (NUMBERS_MASK) when ui32_number = 0
      else if ((ui8_counter > 1 && ui32_number == 0) ||
          // TEMPERATURE_FIELD: print 1 zero only when value is less than 10
          (ui8_lcd_field == TEMPERATURE_FIELD && ui8_counter > 0 && ui32_number == 0))
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      else
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
      }
    }

    // because the LCD mask/layout is different on some field, like numbers would be inverted
    if (ui8_lcd_field == WHEEL_SPEED_FIELD || ui8_lcd_field == BATTERY_POWER_FIELD)
    {
      if (ui8_lcd_field == WHEEL_SPEED_FIELD)
      {
        if ((ui8_options == 0) && (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        // print only first 2 zeros
        else if (ui8_counter > 1 && ui32_number == 0)
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
      }

      if (ui8_lcd_field == BATTERY_POWER_FIELD)
      {
        // print only first zero
        if (ui8_counter > 0 && ui32_number == 0)
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
      }
    }
    
    if (ui8_lcd_field == TIME_SECOND_FIELD)
    {
      ui8_lcd_frame_buffer[SECOND_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
    }
    
    if (ui8_lcd_field == TIME_MINUTE_FIELD)
    {
      if (ui8_counter > 0 && ui32_number == 0)
      {
        ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      else
      {
        ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
      }
    }
    
    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TEMPERATURE_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TIME_SECOND_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == TIME_MINUTE_FIELD) break;

    ui32_number /= 10;
  }
}


void lcd_enable_w_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 128;
  else
    ui8_lcd_frame_buffer[9] &= ~128;
}

void lcd_enable_odometer_point_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[6] |= 8;
  else
    ui8_lcd_frame_buffer[6] &= ~8;
}

void lcd_enable_brake_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[23] |= 4;
  else
    ui8_lcd_frame_buffer[23] &= ~4;
}

void lcd_enable_lights_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[23] |= 2;
  else
    ui8_lcd_frame_buffer[23] &= ~2;
}

void lcd_enable_cruise_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[0] |= 16;
  else
    ui8_lcd_frame_buffer[0] &= ~16;
}

void lcd_enable_assist_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[1] |= 8;
  else
    ui8_lcd_frame_buffer[1] &= ~8;
}

void lcd_enable_vol_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[2] |= 8;
  else
    ui8_lcd_frame_buffer[2] &= ~8;
}

void lcd_enable_odo_symbol(uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[3] |= 8;
  else
    ui8_lcd_frame_buffer[3] &= ~8;
}

void lcd_enable_km_symbol(uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[4] |= 8;
  else
    ui8_lcd_frame_buffer[4] &= ~8;
}

void lcd_enable_mil_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[5] |= 8;
  else
    ui8_lcd_frame_buffer[5] &= ~8;
}

void lcd_enable_temperature_1_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[7] |= 8;
  else
    ui8_lcd_frame_buffer[7] &= ~8;
}

void lcd_enable_battery_power_1_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[12] |= 8;
  else
    ui8_lcd_frame_buffer[12] &= ~8;
}

void lcd_enable_temperature_minus_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[8] |= 8;
  else
    ui8_lcd_frame_buffer[8] &= ~8;
}

void lcd_enable_temperature_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 16;
  else
    ui8_lcd_frame_buffer[9] &= ~16;
}

void lcd_enable_temperature_farneight_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 32;
  else
    ui8_lcd_frame_buffer[9] &= ~32;
}

void lcd_enable_farneight_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 1;
  else
    ui8_lcd_frame_buffer[9] &= ~1;
}

void lcd_enable_motor_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 2;
  else
    ui8_lcd_frame_buffer[9] &= ~2;
}

void lcd_enable_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 64;
  else
    ui8_lcd_frame_buffer[9] &= ~64;
}

void lcd_enable_kmh_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 1;
  else
    ui8_lcd_frame_buffer[13] &= ~1;
}

void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 8;
  else
    ui8_lcd_frame_buffer[13] &= ~8;
}

void lcd_enable_avs_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 16;
  else
    ui8_lcd_frame_buffer[13] &= ~16;
}

void lcd_enable_mxs_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 32;
  else
    ui8_lcd_frame_buffer[13] &= ~32;
}

void lcd_enable_walk_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 64;
  else
    ui8_lcd_frame_buffer[13] &= ~64;
}

void lcd_enable_mph_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 128;
  else
    ui8_lcd_frame_buffer[13] &= ~128;
}

void lcd_enable_dst_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[16] |= 8;
  else
    ui8_lcd_frame_buffer[16] &= ~8;
}

void lcd_enable_tm_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[17] |= 16;
  else
    ui8_lcd_frame_buffer[17] &= ~16;
}

void lcd_enable_ttm_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[17] |= 32;
  else
    ui8_lcd_frame_buffer[17] &= ~32;
}

void lcd_enable_colon_symbol (uint8_t ui8_state)
{
   if (ui8_state)
    ui8_lcd_frame_buffer[23] |= 8;
  else
    ui8_lcd_frame_buffer[23] &= ~8; 
}


void low_pass_filter_battery_voltage_current_power (void)
{
  static uint32_t ui32_battery_voltage_accumulated_x10000;
  static uint16_t ui16_battery_current_accumulated_x5;
  
  // low pass filter battery voltage
  ui32_battery_voltage_accumulated_x10000 -= ui32_battery_voltage_accumulated_x10000 >> BATTERY_VOLTAGE_FILTER_COEFFICIENT;
  ui32_battery_voltage_accumulated_x10000 += (uint32_t) motor_controller_data.ui16_adc_battery_voltage * ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;
  ui16_battery_voltage_filtered_x10 = ((uint32_t) (ui32_battery_voltage_accumulated_x10000 >> BATTERY_VOLTAGE_FILTER_COEFFICIENT)) / 1000;

  // low pass filter battery current
  ui16_battery_current_accumulated_x5 -= ui16_battery_current_accumulated_x5 >> BATTERY_CURRENT_FILTER_COEFFICIENT;
  ui16_battery_current_accumulated_x5 += (uint16_t) motor_controller_data.ui8_battery_current_x5;
  ui16_battery_current_filtered_x5 = ui16_battery_current_accumulated_x5 >> BATTERY_CURRENT_FILTER_COEFFICIENT;

  // battery power
  ui16_battery_power_filtered_x50 = ui16_battery_current_filtered_x5 * ui16_battery_voltage_filtered_x10;
  ui16_battery_power_filtered = ui16_battery_power_filtered_x50 / 50;

  // loose resolution under 200 W
  if (ui16_battery_power_filtered < 200)
  {
    ui16_battery_power_filtered /= 10;
    ui16_battery_power_filtered *= 10;
  }
  // loose resolution under 400 W
  else if (ui16_battery_power_filtered < 400)
  {
    ui16_battery_power_filtered /= 20;
    ui16_battery_power_filtered *= 20;
  }
  // loose resolution all other values
  else
  {
    ui16_battery_power_filtered /= 25;
    ui16_battery_power_filtered *= 25;
  }
}


void low_pass_filter_pedal_torque_and_power (void)
{
  static uint32_t ui32_pedal_torque_accumulated;
  static uint32_t ui32_pedal_power_accumulated;
  static uint8_t ui8_update_counter;

  if (++ui8_update_counter > 10) // update every 100 ms -> 10. This helps to filter the fast changing values
  {
    // reset counter
    ui8_update_counter = 0;
    
    // low pass filter for pedal torque display
    ui32_pedal_torque_accumulated -= ui32_pedal_torque_accumulated >> PEDAL_TORQUE_FILTER_COEFFICIENT;
    ui32_pedal_torque_accumulated += (uint32_t) motor_controller_data.ui16_pedal_torque_x10 / 10;
    ui16_pedal_torque_filtered = ((uint32_t) (ui32_pedal_torque_accumulated >> PEDAL_TORQUE_FILTER_COEFFICIENT));

    if (ui16_pedal_torque_filtered > 200)
    {
      ui16_pedal_torque_filtered /= 20;
      ui16_pedal_torque_filtered *= 20;
    }
    else if (ui16_pedal_torque_filtered > 100)
    {
      ui16_pedal_torque_filtered /= 10;
      ui16_pedal_torque_filtered *= 10;
    }
    else
    {
      // do nothing to orginal values
    }

    // low pass filter for pedal power display
    ui32_pedal_power_accumulated -= ui32_pedal_power_accumulated >> PEDAL_POWER_FILTER_COEFFICIENT;
    ui32_pedal_power_accumulated += (uint32_t) motor_controller_data.ui16_pedal_power_x10 / 10;
    ui16_pedal_power_filtered = ((uint32_t) (ui32_pedal_power_accumulated >> PEDAL_POWER_FILTER_COEFFICIENT));

    if (ui16_pedal_power_filtered > 500)
    {
      ui16_pedal_power_filtered /= 25;
      ui16_pedal_power_filtered *= 25;
    }
    else if (ui16_pedal_power_filtered > 200)
    {
      ui16_pedal_power_filtered /= 20;
      ui16_pedal_power_filtered *= 20;
    }
    else if (ui16_pedal_power_filtered > 10)
    {
      ui16_pedal_power_filtered /= 10;
      ui16_pedal_power_filtered *= 10;
    }
    else
    {
      ui16_pedal_power_filtered = 0; // no point to show less than 10 W
    }
  }
}


static void low_pass_filter_pedal_cadence (void)
{
  static uint16_t ui16_pedal_cadence_accumulated;
  static uint8_t ui8_update_counter;

  if (++ui8_update_counter > 50) // update every 500 ms -> 50. This helps to filter the fast changing values
  {
    // reset counter
    ui8_update_counter = 0;
    
    // low pass filter
    ui16_pedal_cadence_accumulated -= (ui16_pedal_cadence_accumulated >> PEDAL_CADENCE_FILTER_COEFFICIENT);
    ui16_pedal_cadence_accumulated += (uint16_t) motor_controller_data.ui8_pedal_cadence;

    // consider the filtered value only for medium and high values of the unfiltered value
    if (motor_controller_data.ui8_pedal_cadence > 20)
    {
      ui8_pedal_cadence_filtered = (uint8_t) (ui16_pedal_cadence_accumulated >> PEDAL_CADENCE_FILTER_COEFFICIENT);
    }
    else
    {
      ui8_pedal_cadence_filtered = motor_controller_data.ui8_pedal_cadence;
    }
  }
}


void calc_distance (void)
{
  uint32_t ui32_temp;
  
  // calculate how many revolutions since last reset and convert to distance traveled
  ui32_temp = (motor_controller_data.ui32_wheel_speed_sensor_tick_counter - motor_controller_data.ui32_wheel_speed_sensor_tick_counter_offset) * ((uint32_t) configuration_variables.ui16_wheel_perimeter);
  
  // if traveled distance is more than 100 meters update all distance variables and reset
  if (ui32_temp >= 100000) // 100000 -> 100000 mm -> 0.1 km
  {
    // update all distance variables
    configuration_variables.ui16_distance_since_power_on_x10 += 1;
    configuration_variables.ui32_odometer_x10 += 1;
    configuration_variables.ui32_trip_x10 += 1;
   
    // reset the always incrementing value (up to motor controller power reset) by setting the offset to current value
    motor_controller_data.ui32_wheel_speed_sensor_tick_counter_offset = motor_controller_data.ui32_wheel_speed_sensor_tick_counter;
  }
}


struct_configuration_variables* get_configuration_variables (void)
{
  return &configuration_variables;
}


struct_motor_controller_data* lcd_get_motor_controller_data (void)
{
  return &motor_controller_data;
}


void lcd_init (void)
{
  ht1622_init ();
  lcd_set_frame_buffer ();
  lcd_update();

  // init variables with the stored value on EEPROM
  eeprom_init_variables ();
}


void lcd_set_backlight_intensity (uint8_t ui8_intensity)
{
  if (ui8_intensity == 0)
  {
    TIM1_CCxCmd (TIM1_CHANNEL_4, DISABLE);
  }
  else if (ui8_intensity <= 20)
  {
    TIM1_SetCompare4 ((uint16_t) ui8_intensity);
    TIM1_CCxCmd (TIM1_CHANNEL_4, ENABLE);
  }
}


void update_menu_flashing_state(void)
{
  static uint8_t ui8_lcd_menu_flash_counter;
  static uint16_t ui16_lcd_menu_flash_counter_temperature;
  
  #define LCD_MENU_FLASH_THRESHOLD              25
  #define LCD_TEMPERATURE_FLASH_OFF_THRESHOLD   10  // 10 -> 100 ms 

  // flash on menus
  if (++ui8_lcd_menu_flash_counter > LCD_MENU_FLASH_THRESHOLD)
  {
    ui8_lcd_menu_flash_counter = 0;
    
    ui8_lcd_menu_flash_state = !ui8_lcd_menu_flash_state;
  }

  // flash the temperature field when the current is being limited due to motor over temperature
  if (motor_controller_data.ui8_temperature_current_limiting_value < 255) // flash only if current is being limited, i.e. below 255
  {
    if (ui8_lcd_menu_flash_state_temperature == 0)
    {
      if (++ui16_lcd_menu_flash_counter_temperature > LCD_TEMPERATURE_FLASH_OFF_THRESHOLD)
      {
        ui16_lcd_menu_flash_counter_temperature = 0;
        
        ui8_lcd_menu_flash_state_temperature = 1;
      }
    }
    else if (++ui16_lcd_menu_flash_counter_temperature > motor_controller_data.ui8_temperature_current_limiting_value + LCD_TEMPERATURE_FLASH_OFF_THRESHOLD)
    {
      ui16_lcd_menu_flash_counter_temperature = 0;
      
      ui8_lcd_menu_flash_state_temperature = 0;
    }
  }
  else
  {
    ui8_lcd_menu_flash_state_temperature = 1;
  }
}


void submenu_state_controller (uint8_t ui8_state_max_number)
{
  if (ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    // stop changing variables if...
    if (ONOFF_LONG_CLICK)
    {
      ui8_lcd_menu_config_submenu_change_variable_enabled = 0;
    }
  }
  else
  {
    // change variables if onoff click event
    if (ONOFF_CLICK)
    {
      ui8_lcd_menu_config_submenu_change_variable_enabled = 1;
    }
    else
    {
      // advance on submenus
      if (UP_CLICK)
      {
        if (ui8_lcd_menu_config_submenu_state < ui8_state_max_number) { ++ui8_lcd_menu_config_submenu_state; } 
        else { ui8_lcd_menu_config_submenu_state = 0; }
      }

      // recede on submenus
      if (DOWN_CLICK)
      {
        if (ui8_lcd_menu_config_submenu_state > 0) { --ui8_lcd_menu_config_submenu_state; } 
        else { ui8_lcd_menu_config_submenu_state = ui8_state_max_number; }
      }
      
      // leave config menu
      if (ONOFF_LONG_CLICK)
      {
        ui8_lcd_menu_config_submenu_active = 0;
        ui8_lcd_menu_config_submenu_state = 0;
        
        // set backlight brightness after user has configured settings, looks nicer this way
        if (ui8_lights_state == 0) { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_off_brightness); }
        else { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_on_brightness); }
      }
    }
  }
}


void advance_on_subfield (uint8_t* ui8_p_state, uint8_t ui8_state_max_number)
{
  if (UP_CLICK_LONG_CLICK)
  {
    if (*ui8_p_state < ui8_state_max_number) { ++*ui8_p_state; } 
    else { *ui8_p_state = 0; }
    
    odometer_start_show_field_number ();
  }
}


void lcd_power_off (uint8_t SaveToEEPROM)
{
  if (SaveToEEPROM)
  {
    // add consumed watt-hours to watt-hours variable 
    configuration_variables.ui32_wh_x10_offset = ui32_wh_x10;
    
    // save variables to EEPROM
    eeprom_write_variables ();
  }

  // clear LCD so it is clear to user what is happening
  lcd_clear ();
  lcd_update ();

  // now disable the power to all the system
  GPIO_WriteLow(LCD3_ONOFF_POWER__PORT, LCD3_ONOFF_POWER__PIN);

  // block here
  while (1) ;
}


void lcd_configurations_print_number(var_number_t* p_lcd_var_number)
{
  static uint8_t    ui8_long_click_started;
  static uint8_t    ui8_long_click_counter;
  
  uint8_t *ui8_p_var = 0;
  uint16_t *ui16_p_var = 0;
  uint32_t *ui32_p_var = 0;
  uint32_t ui32_value = 0;
  uint8_t ui8_long_click_trigger = 0;

  if(p_lcd_var_number->ui8_size == 8)
  {
    ui8_p_var = ((uint8_t *) p_lcd_var_number->p_var_number);
  }
  else if(p_lcd_var_number->ui8_size == 16)
  {
    ui16_p_var = ((uint16_t *) p_lcd_var_number->p_var_number);
  }
  else if(p_lcd_var_number->ui8_size == 32)
  {
    ui32_p_var = ((uint32_t *) p_lcd_var_number->p_var_number);
  }
  
  if (ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    // if LONG CLICK, keep track of long click so variable is increased automatically 10x every second
    if (UP_LONG_CLICK  || DOWN_LONG_CLICK)
    {
      ui8_long_click_started = 1;
    }

    // trigger at every 100 ms if UP/DOWN LONG CLICK
    if((ui8_long_click_started == 1) && (buttons_get_up_state() || buttons_get_down_state()))
    {
      if(++ui8_long_click_counter >= 10)
      {
        ui8_long_click_counter = 0;
        ui8_long_click_trigger = 1;
      }
    }
    else
    {
      ui8_long_click_started = 0;
      ui8_long_click_counter = 0;
    }

    // increase
    if (UP_CLICK || (buttons_get_up_state() && ui8_long_click_trigger))
    {
      if(p_lcd_var_number->ui8_size == 8)
      {
        if((*ui8_p_var) <= (p_lcd_var_number->ui32_max_value - p_lcd_var_number->ui32_increment_step)) { (*ui8_p_var) += p_lcd_var_number->ui32_increment_step; }
        else { (*ui8_p_var) = (uint8_t) p_lcd_var_number->ui32_max_value; }
      }
      else if(p_lcd_var_number->ui8_size == 16)
      {
        if((*ui16_p_var) <= (p_lcd_var_number->ui32_max_value - p_lcd_var_number->ui32_increment_step)) { (*ui16_p_var) += p_lcd_var_number->ui32_increment_step; }
        else { (*ui16_p_var) = (uint16_t) p_lcd_var_number->ui32_max_value; }
      }
      else if(p_lcd_var_number->ui8_size == 32)
      {
        if((*ui32_p_var) <= (p_lcd_var_number->ui32_max_value - p_lcd_var_number->ui32_increment_step)) { (*ui32_p_var) += p_lcd_var_number->ui32_increment_step; }
        else { (*ui32_p_var) = p_lcd_var_number->ui32_max_value; }
      }
    }

    // decrease
    if (DOWN_CLICK || (buttons_get_down_state() && ui8_long_click_trigger))
    {
      if(p_lcd_var_number->ui8_size == 8)
      {
        if((*ui8_p_var) >= (p_lcd_var_number->ui32_min_value + p_lcd_var_number->ui32_increment_step)) { (*ui8_p_var) -= p_lcd_var_number->ui32_increment_step; }
        else { (*ui8_p_var) = (uint8_t) p_lcd_var_number->ui32_min_value; }
      }
      else if(p_lcd_var_number->ui8_size == 16)
      {
        if((*ui16_p_var) >= (p_lcd_var_number->ui32_min_value + p_lcd_var_number->ui32_increment_step)) { (*ui16_p_var) -= p_lcd_var_number->ui32_increment_step; }
        else { (*ui16_p_var) = (uint16_t) p_lcd_var_number->ui32_min_value; }
      }
      else if(p_lcd_var_number->ui8_size == 32)
      {
        if((*ui32_p_var) >= (p_lcd_var_number->ui32_min_value + p_lcd_var_number->ui32_increment_step)) { (*ui32_p_var) -= p_lcd_var_number->ui32_increment_step; }
        else { (*ui32_p_var) = p_lcd_var_number->ui32_min_value; }
      }
    }
  }
  
  if(p_lcd_var_number->ui8_size == 8)
  {
    ui32_value = (uint32_t) (*ui8_p_var);
  }
  else if(p_lcd_var_number->ui8_size == 16)
  {
    ui32_value = (uint32_t) (*ui16_p_var);
  }
  else if(p_lcd_var_number->ui8_size == 32)
  {
    ui32_value = (*ui32_p_var);
  }

  // draw only at every ui8_lcd_menu_flash_state -- will flash the number on the LCD
  if(ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui32_value, p_lcd_var_number->ui8_odometer_field, p_lcd_var_number->ui8_decimal_digit);
  }
}
