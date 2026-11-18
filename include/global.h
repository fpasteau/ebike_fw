#ifndef GLOBAL_H
#define GLOBAL_H

extern uint16_t ui16_aca_experimental_flags;
extern uint8_t ui8_variableDebugA;
extern uint8_t ui8_variableDebugB;
extern uint8_t ui8_variableDebugC;
extern uint8_t ui8_possible_motor_state;
extern uint8_t ui8_dynamic_motor_state;
extern uint8_t ui8_s_hall_angle4_0;
extern uint8_t ui8_s_hall_angle6_60;
extern uint8_t ui8_s_hall_angle2_120;
extern uint8_t ui8_s_hall_angle3_180;
extern uint8_t ui8_s_hall_angle1_240;
extern uint8_t ui8_s_hall_angle5_300;
extern uint16_t ui16_pwm_cycles_second;
extern uint16_t ui16_time_ticks_for_uart_timeout;
extern uint8_t ui8_position_correction_value; // in 360/256 degrees
extern uint8_t uint8_t_hall_order[6];
extern uint8_t ui8_hall_order_counter;
extern uint8_t uint8_t_60deg_pwm_cycles[6];
extern uint16_t ui16_motor_speed_erps;
extern uint16_t ui16_time_ticks_for_speed_calculation;
extern uint16_t ui16_time_ticks_for_pas_calculation; //time tics for cadence measurement
extern uint8_t ui8_s_motor_angle;
extern uint8_t ui8_correction_at_angle;
extern uint8_t ui8_direction;
extern uint8_t ui8_s_motor_angle_forward;
extern uint8_t ui8_s_motor_angle_reverse;

typedef enum {

	DC_STATIC_ZERO = ((uint16_t) 1),
	AVOID_MOTOR_CYCLES_JITTER = ((uint16_t) 2),
	DISABLE_INTERPOLATION = ((uint16_t) 4),
	DISABLE_60_DEG_INTERPOLATION = ((uint16_t) 8),
	SWITCH_360_DEG_INTERPOLATION = ((uint16_t) 16),
	USE_ALTERNATE_WAVETABLE = ((uint16_t) 32),
	USE_ALTERNATE_WAVETABLE_B = ((uint16_t) 64),
	DUMMY_EXP_ALWAYS_ON = ((uint16_t) 128),
	HIGH_SPEED_MOTOR = ((uint16_t) 256),
	PWM_AUTO_OFF = ((uint16_t) 1024),
			
} ACA_EXPERIMENTAL_FLAGS;

void global_init(void);



#endif