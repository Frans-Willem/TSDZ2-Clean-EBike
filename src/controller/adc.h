/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H
#define _ADC_H

#include "main.h"

#define UI8_ADC_BATTERY_VOLTAGE 			(ADC1->DB6RH) // AIN6
#define UI8_ADC_BATTERY_CURRENT				(ADC1->DB5RH) // AIN5
#define UI8_ADC_THROTTLE 				      (ADC1->DB7RH) // AIN7
#define UI8_ADC_TORQUE_SENSOR         (ADC1->DB4RH) // AIN4

void adc_init (void);
uint16_t ui16_adc_read_battery_current_10b (void);
uint16_t ui16_adc_read_battery_voltage_10b (void);
uint16_t ui16_adc_read_torque_sensor_10b (void);
uint16_t ui16_adc_read_throttle_10b (void);

#endif /* _ADC_H */
