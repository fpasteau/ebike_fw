#include <stdint.h>
#include <config.h>
#include <main.h>
#include "ACAeeprom.h"


uint16_t ui16_aca_experimental_flags = 0;
uint8_t ui8_variableDebugA = 0;
uint8_t ui8_variableDebugB = 0;
uint8_t ui8_variableDebugC = 0;
uint8_t ui8_possible_motor_state = 0;
uint8_t ui8_dynamic_motor_state = 0;
uint8_t ui8_s_hall_angle4_0 = 0;
uint8_t ui8_s_hall_angle6_60 = 42;
uint8_t ui8_s_hall_angle2_120 = 85;
uint8_t ui8_s_hall_angle3_180 = 127;
uint8_t ui8_s_hall_angle1_240 = 170;
uint8_t ui8_s_hall_angle5_300 = 212;
uint16_t ui16_pwm_cycles_second = 0;
uint16_t ui16_time_ticks_for_uart_timeout = 0;
uint8_t ui8_position_correction_value = 127; // in 360/256 degrees
uint8_t uint8_t_hall_order[6];
uint8_t ui8_hall_order_counter = 5;
uint8_t uint8_t_60deg_pwm_cycles[6];
uint16_t ui16_motor_speed_erps = 0;
uint16_t ui16_time_ticks_for_speed_calculation = 0;
uint16_t ui16_time_ticks_for_pas_calculation = 0; //time tics for cadence measurement
uint8_t ui8_s_motor_angle = 214;
uint8_t ui8_s_motor_angle_forward = 214;
uint8_t ui8_s_motor_angle_reverse = 214;
uint8_t ui8_correction_at_angle = 127;
uint8_t ui8_direction = 0;



void global_init(void) {
	int di = 0;
	eeprom_init();

	// convert static defines to volatile vars
	ui16_pwm_cycles_second = PWM_CPS_NORMAL_SPEED;
	ui16_aca_experimental_flags = ACA_EXPERIMENTAL;
	ui8_s_motor_angle = MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT;
	ui8_s_motor_angle_forward = eeprom_read(FORWARD_MOTOR_ANGLE);
	ui8_s_motor_angle_reverse = eeprom_read(REVERSE_MOTOR_ANGLE);
	ui8_s_hall_angle4_0 = ANGLE_4_0;
	ui8_s_hall_angle6_60 = ANGLE_6_60;
	ui8_s_hall_angle2_120 = ANGLE_2_120;
	ui8_s_hall_angle3_180 = ANGLE_3_180;
	ui8_s_hall_angle1_240 = ANGLE_1_240;
	ui8_s_hall_angle5_300 = ANGLE_5_300;
	ui8_correction_at_angle = CORRECTION_AT_ANGLE;
	for (di = 0; di < 6; di++) {
		uint8_t_hall_order[di] = 0;
	}
}