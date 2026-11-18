/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "stm8s_itc.h"
#include "stm8s_gpio.h"
#include "interrupts.h"
#include "stm8s_tim2.h"
#include "motor.h"
#include "main.h"
#include "uart.h"
#include "timers.h"
#include "pwm.h"
#include "config.h"
#include "global.h"
#include "brake.h"

uint16_t ui16_log1 = 0;
uint8_t ui8_slowloop_flag = 0;
uint16_t ui16_log2 = 0;
uint8_t ui8_log = 0;
uint8_t ui8_i = 0; //counter for ... next loop

float float_kv = 0;
float float_R = 0;
uint8_t a = 0; //loop counter

static int16_t i16_deziAmps;

int16_t i16_setpoint = 0;



/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

//With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
//in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
//to place the function prototype in a header file as long as the header file is included in the file that
//contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
//not be in place so the ISR will not be executed when the interrupt occurs.

//Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
//all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// and the solution was to avoid using local variables and define them as global instead

// Brake signal interrupt
//void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER);
// Speed signal interrupt
//void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER);
// PAS signal interrupt
//void EXTI_PORTD_IRQHandler(void) __interrupt(EXTI_PORTD_IRQHANDLER);

// Timer1/PWM period interrupt
//void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER);

// Timer2/slow control loop
//void TIM2_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM2_UPD_OVF_TRG_BRK_IRQHANDLER);

// UART2 receivce handler
//void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void main(void) {
	int16_t cpt = 0;
	int up = 1;
	//set clock at the max 16MHz
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	
	timer2_init();
	uart_init();
	global_init();
	pwm_init();
	adc_init();
	brake_init();

	
	hall_sensor_init();
	enableInterrupts();
	watchdog_init(); //init watchdog after enabling interrupt to have fast loop running already

#if (SVM_TABLE == SVM)
	TIM1_SetCompare1(126 << 1);
	TIM1_SetCompare2(126 << 1);
	TIM1_SetCompare3(126 << 1);
#elif (SVM_TABLE == SINE) || (SVM_TABLE == SINE_SVM)
	TIM1_SetCompare1(126 << 2);
	TIM1_SetCompare2(126 << 2);
	TIM1_SetCompare3(126 << 2);
#endif
	i16_setpoint = 0;

	hall_sensors_read_and_action(); // needed to start the motor
	while (1) {

		uart_send_if_avail();
		if(byte_avail_at_position()!=UART_EMPTY_INDICATOR){
			stateMachine(uart_get_buffered());
		}	
		if (ui8_slowloop_flag) {
			/*if(doCalibration){
				printf("Calibrating...\r\n");
				int motor_angle = 0;
				for(motor_angle = 0 ; motor_angle < 255; motor_angle += 10){
					printf("motor_angle:%d\r\n", motor_angle);
					measureCurrentWithMotorAngle(motor_angle, 255);
				}
				doCalibration = 0;
			} else {*/
				i16_setpoint = getSpeed();
				pwm_set_duty_cycle(i16_setpoint);
				
				/*i16_setpoint = cpt/2;
				pwm_set_duty_cycle(i16_setpoint);
				if(up) cpt ++;
				else cpt --;
				if(abs(cpt) >= 500){
					up = !up;
				}*/
			//}

			ui8_slowloop_flag = 0; //reset flag for slow loop
		}

	}// end of while(1) loop
}


