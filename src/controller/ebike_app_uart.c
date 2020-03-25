
#include "stm8s.h"

#include "ebike_app.h"
#include "main.h"
#include "interrupts.h"
#include "utils.h"
#include "lights.h"
#include "motor.h"
#include "eeprom.h"
#include "adc.h"
#include "uart.h"
#include "pins.h"

// UART related definitions and variables

#if VLCD
    #define UART_RX_BUFFER_LEN 7
    #define UART_TX_BUFFER_LEN 9
    
    #define RX_CHECK_CODE         (UART_RX_BUFFER_LEN - 1)															
    #define TX_CHECK_CODE		      (UART_TX_BUFFER_LEN - 1)
    #define TX_STX				        0x43
    #define RX_STX                0x59		

    #define ASSIST_PEDAL_LEVEL0	  0x10
    #define ASSIST_PEDAL_LEVEL1	  0x40
    #define ASSIST_PEDAL_LEVEL2	  0x02
    #define ASSIST_PEDAL_LEVEL3	  0x04
    #define ASSIST_PEDAL_LEVEL4		0x08
    #define WALK_ASSIST           0x20
#elif DEBUG_UART
    #include <stdio.h>
    #include <string.h>
    #define UART_RX_BUFFER_LEN 12
    #define UART_TX_BUFFER_LEN 9
    static uint8_t ui8_uart_debug_data_indicator = 0;
    extern volatile uint16_t ui16_pas_pwm_cycles_ticks;
    extern volatile enum pedaling_direction_t enm_g_pedaling_direction;
#else
    // change this value depending on how many data bytes there is to receive 
    //( Package = one start byte + data bytes + two bytes 16 bit CRC )
    #define UART_NUMBER_DATA_BYTES_TO_RECEIVE 6   
    #define UART_RX_BUFFER_LEN (UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3)
    // change this value depending on how many data bytes there is to send 
    // ( Package = one start byte + data bytes + two bytes 16 bit CRC )
    #define UART_NUMBER_DATA_BYTES_TO_SEND      25 
    #define UART_TX_BUFFER_LEN (UART_NUMBER_DATA_BYTES_TO_SEND + 3)
#endif

static volatile uint8_t ui8_received_package_flag = 0;
//static volatile uint8_t ui8_rx_buffer[UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3];
static volatile uint8_t ui8_rx_buffer[UART_RX_BUFFER_LEN];
static volatile uint8_t ui8_rx_counter = 0;
//static volatile uint8_t ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 3];
static volatile uint8_t ui8_tx_buffer[UART_TX_BUFFER_LEN];
static volatile uint8_t ui8_i;
static volatile uint8_t ui8_byte_received;
static volatile uint8_t ui8_state_machine = 0;
static uint16_t  ui16_crc_rx;
static uint16_t  ui16_crc_tx;
static volatile uint8_t ui8_message_ID = 0;


// external variables
// todo: move extern declarations to ebike_app.h

extern volatile struct_configuration_variables m_configuration_variables;
extern uint16_t     ui16_wheel_speed_x10;
extern uint16_t     ui16_wheel_speed_for_vlcd;
extern volatile uint8_t  ui8_throttle;
extern volatile uint8_t  ui8_torque_sensor;
extern uint8_t   ui8_pedal_human_power;
extern volatile uint8_t ui8_system_state;
extern uint16_t  ui16_pedal_torque_x10;
extern uint16_t  ui16_pedal_power_x10;
extern uint16_t   ui16_received_target_wheel_speed_x10;

// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  
#if DEBUG_UART

  if (UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

    uint8_t ui8_bytereceived = UART2_ReceiveData8 ();

    if(ui8_rx_counter < UART_RX_BUFFER_LEN)
    {
      if(ui8_bytereceived == '\n')
      {
        ui8_rx_buffer[ui8_rx_counter] = 0x0;
        ui8_rx_counter = 0;
        ui8_received_package_flag = 1;
      }
      else if(!ui8_received_package_flag) // wait until buffer is cleared
      {
        ui8_rx_buffer[ui8_rx_counter] = ui8_bytereceived;
        ui8_rx_counter++;
      }
    }
    else
    {
      // overflow -> reset rx_counter
      ui8_rx_counter = 0;
    }
    

    //UART2_SendData8(ui8_bytereceived);
    //printf(ui8_bytereceived);

  }

#else

  if (UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

    ui8_byte_received = UART2_ReceiveData8 ();

    switch (ui8_state_machine)
    {
      case 0:
      if (ui8_byte_received == 0x59) // see if we get start package byte
      {
        ui8_rx_buffer [ui8_rx_counter] = ui8_byte_received;
        ui8_rx_counter++;
        ui8_state_machine = 1;
      }
      else
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
      }
      break;

      case 1:
      ui8_rx_buffer [ui8_rx_counter] = ui8_byte_received;
      
      // increment index for next byte
      ui8_rx_counter++;

      // reset if it is the last byte of the package and index is out of bounds
      //if (ui8_rx_counter >= UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3)
      if (ui8_rx_counter >= UART_RX_BUFFER_LEN)
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
        ui8_received_package_flag = 1; // signal that we have a full package to be processed
        UART2->CR2 &= ~(1 << 5); // disable UART2 receive interrupt
      }
      break;

      default:
      break;
    }
  }
#endif
}

void initializeConfigurationVariables(void)
{
  m_configuration_variables.ui8_assist_level_factor_x10 = DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_X10;
  m_configuration_variables.ui8_battery_max_current = DEFAULT_VALUE_BATTERY_MAX_CURRENT;
  m_configuration_variables.ui8_motor_power_x10 = 0;
  m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = 370; // 37V
  m_configuration_variables.ui16_wheel_perimeter = 2050; // 26'' -> 2050mm
  m_configuration_variables.ui8_lights = 0;
  m_configuration_variables.ui8_walk_assist = 0;
  m_configuration_variables.ui8_wheel_max_speed = DEFAULT_VALUE_WHEEL_MAX_SPEED;
  m_configuration_variables.ui8_motor_type = 0; // how to indicate 48V -> 0 -> motor.c
  m_configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation = 0;
  m_configuration_variables.ui8_target_battery_max_power_div25 = 22; // 22x25 = 550, same as motor_power_x10?
  m_configuration_variables.configuration_variables = 0;
  m_configuration_variables.ui8_startup_motor_power_boost_feature_enabled = 0;
  m_configuration_variables.ui8_startup_motor_power_boost_assist_level = 0;
  m_configuration_variables.ui8_startup_motor_power_boost_state = 0;
  m_configuration_variables.ui8_startup_motor_power_boost_limit_to_max_power = 0;
  m_configuration_variables.ui8_startup_motor_power_boost_time = 0;
  m_configuration_variables.ui8_startup_motor_power_boost_fade_time = 0;
  m_configuration_variables.ui8_temperature_limit_feature_enabled = 0;
  m_configuration_variables.ui8_motor_temperature_min_value_to_limit = 0;
  m_configuration_variables.ui8_motor_temperature_max_value_to_limit = 0;
  m_configuration_variables.ui8_temperature_current_limiting_value = 0;
  m_configuration_variables.ui16_motor_temperature_x2 = 0;
  m_configuration_variables.ui8_motor_temperature = 0;
  m_configuration_variables.ui8_ramp_up_amps_per_second_x10 = DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND_X10;

  /* from original uart_receive_package */
  
  // battery voltage cutoff
  // calc the value in ADC steps and set it up
  uint32_t ui32_temp;
  ui32_temp = ((uint32_t)m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 << 8) / ((uint32_t)ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP_INVERSE_X256);
  ui32_temp /= 10;
  motor_set_adc_battery_voltage_cut_off((uint8_t)ui32_temp);

  // max current
  // set max current from battery
  ebike_app_set_battery_max_current(m_configuration_variables.ui8_battery_max_current);

  // current ramp
  // ramp up, amps per second
  // see NOTE: regarding ramp up in default uart_receive_package
  ui32_temp = ((uint32_t)97656) / ((uint32_t)m_configuration_variables.ui8_ramp_up_amps_per_second_x10);
  ui16_current_ramp_up_inverse_step = (uint16_t)ui32_temp;

  lights_set_state (1);

}


#if VLCD


void uart_receive_package(void)
{
  if (ui8_received_package_flag)
  {
    uint8_t ui8_assist_level;
    uint8_t ui8_rx_check_code = 0x00;
    for (ui8_i = 0; ui8_i < RX_CHECK_CODE; ui8_i++)
    {
      ui8_rx_check_code += ui8_rx_buffer[ui8_i];
    }

    // see if check code is ok...
    if (ui8_rx_check_code == ui8_rx_buffer[RX_CHECK_CODE])
    {
      ui8_assist_level = ui8_rx_buffer[1] & 0x5E; // mask: 01011110

      switch (ui8_assist_level)
      {
      case ASSIST_PEDAL_LEVEL0: // OFF
        m_configuration_variables.ui8_assist_level_factor_x10 = 0;
        break;
      case ASSIST_PEDAL_LEVEL1: // ECO
        m_configuration_variables.ui8_assist_level_factor_x10 = 10;
        break;
      case ASSIST_PEDAL_LEVEL2: // TOUR
        m_configuration_variables.ui8_assist_level_factor_x10 = 20;
        break;
      case ASSIST_PEDAL_LEVEL3: // SPEED
        m_configuration_variables.ui8_assist_level_factor_x10 = 30;
        break;
      case ASSIST_PEDAL_LEVEL4: // TURBO
        m_configuration_variables.ui8_assist_level_factor_x10 = 40;
        break;
      default:
        break;
      }

      if ((ui8_rx_buffer[1]) & (1 << 0) ? 1 : 0)
      {
        m_configuration_variables.ui8_lights = 1;
        lights_set_state(m_configuration_variables.ui8_lights);
      }
      else
      {
        m_configuration_variables.ui8_lights = 0;
        lights_set_state(m_configuration_variables.ui8_lights);
      }
      // walk assist
      if (ui8_rx_buffer[1] & 0x20)
      {
        m_configuration_variables.ui8_walk_assist = 1;
      }
      else
      {
        m_configuration_variables.ui8_walk_assist = 0;
      }
    }

    // signal that we processed the full package
    ui8_received_package_flag = 0;

    // enable UART2 receive interrupt as we are now ready to receive a new package
    UART2->CR2 |= (1 << 5);
  }
}

void uart_send_package(void)
{
  // send the data to the LCD
  // start up byte
  ui8_tx_buffer[0] = TX_STX;

  //uint16_t ui16_temp = motor_get_adc_battery_voltage_filtered_10b();
  //uint8_t ui8_battery_state_of_charge = 0;

  //ui16_temp = ui16_temp / 10;
  uint16_t ui16_temp = calc_filtered_battery_voltage();

  if (ui16_temp > 54)
    ui8_tx_buffer[1] = 0x0C; // 6/6
  else if (ui16_temp >= 51)
    ui8_tx_buffer[1] = 0x0A; // 5/6
  else if (ui16_temp >= 47)
    ui8_tx_buffer[1] = 0x08; // 4/6
  else if (ui16_temp >= 43)
    ui8_tx_buffer[1] = 0x06; // 3/6
  else if (ui16_temp >= 41.6)
    ui8_tx_buffer[1] = 0x04; // 2/6
  else if (ui16_temp >= 38)
    ui8_tx_buffer[1] = 0x02; // 1/6
  else
    ui8_tx_buffer[1] = 0x00; // 1/6

  // working status?
  ui8_tx_buffer[2] = 0x0;
  // reserved
  ui8_tx_buffer[3] = 0x46;
  ui8_tx_buffer[4] = 0x46;
  ui8_tx_buffer[5] = 0; // clear display / fault code?
  // wheel speed
  ui8_tx_buffer[6] = (uint8_t)(ui16_wheel_speed_for_vlcd & 0xFF);
  ui8_tx_buffer[7] = (uint8_t)(ui16_wheel_speed_for_vlcd >> 8);

  // prepare check code of the package
  uint8_t ui8_tx_check_code = 0x00;
  for (ui8_i = 0; ui8_i < TX_CHECK_CODE; ui8_i++)
  {
    ui8_tx_check_code += ui8_tx_buffer[ui8_i];
  }
  ui8_tx_buffer[TX_CHECK_CODE] = ui8_tx_check_code;

  // send the full package to UART
  for (ui8_i = 0; ui8_i < UART_TX_BUFFER_LEN; ui8_i++)
  {
    putchar(ui8_tx_buffer[ui8_i]);
  }
}

#elif DEBUG_UART

void uart_receive_package(void)
{

  if(ui8_received_package_flag)
  {
    //printf("%.12s",ui8_rx_buffer);

    if(strncmp(ui8_rx_buffer, "eco", UART_RX_BUFFER_LEN) == 0)
    {
      printf("eco\n");
      m_configuration_variables.ui8_assist_level_factor_x10 = 10;
    }
    else if (strncmp(ui8_rx_buffer, "tour", UART_RX_BUFFER_LEN) == 0)
    {
      printf("tour\n");
      m_configuration_variables.ui8_assist_level_factor_x10 = 0;
    }
    else if (strncmp(ui8_rx_buffer, "speed", UART_RX_BUFFER_LEN) == 0)
    {
      printf("speed\n");
      m_configuration_variables.ui8_assist_level_factor_x10 = 0;
    }
    else if (strncmp(ui8_rx_buffer, "turbo", UART_RX_BUFFER_LEN) == 0)
    {
      printf("turbo\n");
      m_configuration_variables.ui8_assist_level_factor_x10 = 0;
    }
    else if (strncmp(ui8_rx_buffer, "off", UART_RX_BUFFER_LEN) == 0)
    {
      printf("off\n");
      m_configuration_variables.ui8_assist_level_factor_x10 = 0;
    }
    else if (strncmp(ui8_rx_buffer, "walkassist", UART_RX_BUFFER_LEN) == 0)
    {
      printf("walkassist\n");
    }
    else if (strncmp(ui8_rx_buffer, "light", UART_RX_BUFFER_LEN) == 0)
    {
      printf("light\n");
      if(m_configuration_variables.ui8_lights)
        m_configuration_variables.ui8_lights = 0;
      else
        m_configuration_variables.ui8_lights = 1;
      lights_set_state (m_configuration_variables.ui8_lights);
      
    }
    else if (strncmp(ui8_rx_buffer, "data", UART_RX_BUFFER_LEN) == 0)
    {
      printf("data\n");
      if(ui8_uart_debug_data_indicator) 
        ui8_uart_debug_data_indicator = 0;
      else
        ui8_uart_debug_data_indicator = 1; 
    }
    else
    {
      printf("comand not recognized\n");
    }


    ui8_received_package_flag = 0;

  }
}


void uart_send_package (void)
{
  if(ui8_uart_debug_data_indicator)
  {

    /* pas quadrature signal */
    //uint8_t ui8_pas1_state = 0; 
    //if(PAS1__PORT->IDR & PAS1__PIN)
    //  ui8_pas1_state = 1;
    //uint8_t ui8_pas2_state = 0;
    //if(PAS2__PORT->IDR & PAS2__PIN)
    //  ui8_pas2_state = 1;
    //printf("%d %d\n", ui8_pas1_state, ui8_pas2_state);

    //printf("%d %d\n", ui16_pas_pwm_cycles_ticks, enm_g_pedaling_direction);
    printf("%d %d\n", ui8_pas_cadence_rpm, ui16_pedal_torque_x10);
  }
}

# else

//void initializeConfigurationVariables(void)
//{
  // do nothing
//}


void uart_receive_package(void)
{
  uint32_t ui32_temp;
  
  if (ui8_received_package_flag)
  {
    // validation of the package data
    ui16_crc_rx = 0xffff;
    
    for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_RECEIVE; ui8_i++)
    {
      crc16 (ui8_rx_buffer[ui8_i], &ui16_crc_rx);
    }

    // if CRC is correct read the package (16 bit value and therefore last two bytes)
    if (((((uint16_t) ui8_rx_buffer [UART_NUMBER_DATA_BYTES_TO_RECEIVE + 2]) << 8) + ((uint16_t) ui8_rx_buffer [UART_NUMBER_DATA_BYTES_TO_RECEIVE + 1])) == ui16_crc_rx)
    {
      ui8_message_ID = ui8_rx_buffer [1];

      // assist level
      m_configuration_variables.ui8_assist_level_factor_x10 = ui8_rx_buffer [2];
      
      // lights state
      m_configuration_variables.ui8_lights = (ui8_rx_buffer [3] & (1 << 0)) ? 1: 0;
      
      // set lights
      lights_set_state (m_configuration_variables.ui8_lights);
      
      // walk assist / cruise function 
      m_configuration_variables.ui8_walk_assist = (ui8_rx_buffer [3] & (1 << 1)) ? 1: 0;
      
      // battery max power target
      m_configuration_variables.ui8_target_battery_max_power_div25 = ui8_rx_buffer [4];

      switch (ui8_message_ID)
      {
        case 0:
          // battery low voltage cut-off
          m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) ui8_rx_buffer [6]) << 8) + ((uint16_t) ui8_rx_buffer [5]);
          
          // calc the value in ADC steps and set it up
          ui32_temp = ((uint32_t) m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 << 8) / ((uint32_t) ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP_INVERSE_X256);
          ui32_temp /= 10;
          motor_set_adc_battery_voltage_cut_off ((uint8_t) ui32_temp);
        break;

        case 1:
          // wheel perimeter
          m_configuration_variables.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer [6]) << 8) + ((uint16_t) ui8_rx_buffer [5]);
        break;

        case 2:
          // wheel max speed
          m_configuration_variables.ui8_wheel_max_speed = ui8_rx_buffer [5];
          
          // battery max current
          m_configuration_variables.ui8_battery_max_current = ui8_rx_buffer [6];
          
          // set max current from battery
          ebike_app_set_battery_max_current (m_configuration_variables.ui8_battery_max_current);
        break;

        case 3:
          // type of motor (36 volt, 48 volt or some experimental type)
          m_configuration_variables.ui8_motor_type = ui8_rx_buffer [5];
          
          // startup motor power boost state
          m_configuration_variables.ui8_startup_motor_power_boost_state = (ui8_rx_buffer [6] & 1);
          
          // startup power boost max power limit enabled
          m_configuration_variables.ui8_startup_motor_power_boost_limit_to_max_power = (ui8_rx_buffer [6] & 2) >> 1;
        break;

        case 4:
          // startup motor power boost
          m_configuration_variables.ui8_startup_motor_power_boost_assist_level = ui8_rx_buffer [5];
          
          // startup motor power boost time
          m_configuration_variables.ui8_startup_motor_power_boost_time = ui8_rx_buffer [6];
        break;

        case 5:
          // startup motor power boost fade time
          m_configuration_variables.ui8_startup_motor_power_boost_fade_time = ui8_rx_buffer [5];
          
          // startup motor boost enabled
          m_configuration_variables.ui8_startup_motor_power_boost_feature_enabled = ui8_rx_buffer [6];
        break;

        case 6:
          // motor over temperature min value limit
          m_configuration_variables.ui8_motor_temperature_min_value_to_limit = ui8_rx_buffer [5];
          
          // motor over temperature max value limit
          m_configuration_variables.ui8_motor_temperature_max_value_to_limit = ui8_rx_buffer [6];
        break;
        
        case 7:
          // ramp up, amps per second
          m_configuration_variables.ui8_ramp_up_amps_per_second_x10 = ui8_rx_buffer [5];
          
          // check that value seems correct
          if (m_configuration_variables.ui8_ramp_up_amps_per_second_x10 < 4 || m_configuration_variables.ui8_ramp_up_amps_per_second_x10 > 100)
          {
            // value is not valid, set to default
            m_configuration_variables.ui8_ramp_up_amps_per_second_x10 = DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND_X10;
          }
          
          // calculate current step for ramp up
          ui32_temp = ((uint32_t) 97656) / ((uint32_t) m_configuration_variables.ui8_ramp_up_amps_per_second_x10); // see note below
          ui16_current_ramp_up_inverse_step = (uint16_t) ui32_temp;
          
          /*---------------------------------------------------------
          NOTE: regarding ramp up 

          Example of calculation:
          
          Target ramp up: 5 amps per second

          Every second has 15625 PWM cycles interrupts,
          one ADC battery current step --> 0.625 amps:

          5 / 0.625 = 8 (we need to do 8 steps ramp up per second)

          Therefore:

          15625 / 8 = 1953 (our default value)
          ---------------------------------------------------------*/
          
          // received target speed for cruise
          ui16_received_target_wheel_speed_x10 = (uint16_t) (ui8_rx_buffer [6] * 10);
        break;
        
        case 8:
          // motor temperature limit function or throttle
          m_configuration_variables.ui8_temperature_limit_feature_enabled = ui8_rx_buffer [5];
          
          // motor assistance without pedal rotation enable/disable when startup 
          m_configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation = ui8_rx_buffer [6];
        break;
        
        default:
          // nothing, should display error code
        break;
      }

      // verify if any configuration_variables did change and if so, save all of them in the EEPROM
      eeprom_write_if_values_changed ();

      // signal that we processed the full package
      ui8_received_package_flag = 0;
    }

    // enable UART2 receive interrupt as we are now ready to receive a new package
    UART2->CR2 |= (1 << 5);
  }
}


void uart_send_package(void)
{
  uint16_t ui16_temp;

  // start up byte
  ui8_tx_buffer[0] = 0x43;

  // ADC 10 bits battery voltage
  ui16_temp = motor_get_adc_battery_voltage_filtered_10b();
  ui8_tx_buffer[1] = (ui16_temp & 0xff);
  ui8_tx_buffer[2] = ((uint8_t) (ui16_temp >> 4)) & 0x30;

  // battery current x5
  ui8_tx_buffer[3] = (uint8_t) ((float) motor_get_adc_battery_current_filtered_10b() * 0.826);

  // wheel speed
  ui8_tx_buffer[4] = (uint8_t) (ui16_wheel_speed_x10 & 0xff);
  ui8_tx_buffer[5] = (uint8_t) (ui16_wheel_speed_x10 >> 8);

  // brake state
  if(motor_controller_state_is_set(MOTOR_CONTROLLER_STATE_BRAKE))
  {
    ui8_tx_buffer[6] |= 1;
  }
  else
  {
    ui8_tx_buffer[6] &= ~1;
  }

  // throttle value from ADC
  ui8_tx_buffer[7] = UI8_ADC_THROTTLE;
  
  // adjusted throttle value or temperature limit depending on user setup
  if(m_configuration_variables.ui8_temperature_limit_feature_enabled == 1)
  {
    // temperature value
    ui8_tx_buffer[8] = m_configuration_variables.ui8_motor_temperature;
  }
  else
  {
    // throttle value with offset removed and mapped to 255
    ui8_tx_buffer[8] = ui8_throttle;
  }

  // ADC torque_sensor
  ui8_tx_buffer[9] = UI8_ADC_TORQUE_SENSOR;
  
  // torque sensor value with offset removed
  ui8_tx_buffer[10] = ui8_torque_sensor;
  
  // PAS cadence
  ui8_tx_buffer[11] = ui8_pas_cadence_rpm;
  
  // pedal human power mapped to 255
  ui8_tx_buffer[12] = ui8_pedal_human_power;
  
  // PWM duty_cycle
  ui8_tx_buffer[13] = ui8_g_duty_cycle;
  
  // motor speed in ERPS
  ui16_temp = ui16_motor_get_motor_speed_erps();
  ui8_tx_buffer[14] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[15] = (uint8_t) (ui16_temp >> 8);
  
  // FOC angle
  ui8_tx_buffer[16] = ui8_g_foc_angle;
  
  // system state
  ui8_tx_buffer[17] = ui8_system_state;
  
  // temperature actual limiting value
  ui8_tx_buffer[18] = m_configuration_variables.ui8_temperature_current_limiting_value;
  
  // wheel_speed_sensor_tick_counter
  ui8_tx_buffer[19] = (uint8_t) (ui32_wheel_speed_sensor_tick_counter & 0xff);
  ui8_tx_buffer[20] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 8) & 0xff);
  ui8_tx_buffer[21] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 16) & 0xff);

  // ui16_pedal_torque_x10
  ui8_tx_buffer[22] = (uint8_t) (ui16_pedal_torque_x10 & 0xff);
  ui8_tx_buffer[23] = (uint8_t) (ui16_pedal_torque_x10 >> 8);

  // ui16_pedal_power_x10
  ui8_tx_buffer[24] = (uint8_t) (ui16_pedal_power_x10 & 0xff);
  ui8_tx_buffer[25] = (uint8_t) (ui16_pedal_power_x10 >> 8);

  // prepare crc of the package
  ui16_crc_tx = 0xffff;
  
  for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_SEND; ui8_i++)
  {
    crc16 (ui8_tx_buffer[ui8_i], &ui16_crc_tx);
  }
  
  ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 1] = (uint8_t) (ui16_crc_tx & 0xff);
  ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 2] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

  // send the full package to UART
  for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_SEND + 2; ui8_i++)
  {
    putchar (ui8_tx_buffer[ui8_i]);
  }
}

#endif
