/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "stm8s.h"
#include "stm8s_uart2.h"
#include "interrupts.h"
//#include "ACAcontrollerState.h"
#include "global.h"
#include "ACAeeprom.h"

uint8_t doCalibration = 0;

static uint8_t ui8_uarx_buffer[UART_RINGBUFFER_SIZE];
static uint8_t ui8_uatx_buffer[UART_RINGBUFFER_SIZE];
static uint8_t ui8_rx_fillpointer = 0;
static uint8_t ui8_tx_fillpointer = 0;
static uint8_t ui8_rx_digestpointer = 0;
static uint8_t ui8_tx_digestpointer = 0;

static uint8_t ui8_rx_packet_start_pos = 0;

void uart_put_buffered(uint8_t c) {
    ui8_uatx_buffer[ui8_tx_fillpointer++] = c;
    if (ui8_tx_fillpointer >= UART_RINGBUFFER_SIZE) {
        ui8_tx_fillpointer = 0;
    }
}

uint8_t uart_get_packet_start_pos(void) {
    return ui8_rx_packet_start_pos;
}

uint8_t byte_avail_at_position(void) {
    if (ui8_rx_fillpointer != ui8_rx_digestpointer) {
        return ui8_rx_digestpointer;
    }
    return UART_EMPTY_INDICATOR;
}

uint8_t uart_get_buffered(void) {
    uint8_t c = ui8_uarx_buffer[ui8_rx_digestpointer++];
    if (ui8_rx_digestpointer >= UART_RINGBUFFER_SIZE) {
        ui8_rx_digestpointer = 0;
    }
    return c;
}


#define SPEED_CMD               0x20
#define CALIBRATION_CMD         0x21
#define SAVE_CALIBRATION_CMD    0x22
#define AUTO_CALIBRATION_CMD    0x23

uint8_t syncWord[4] = {0xDE, 0xAD, 0xBE, 0xEF};
typedef enum {INIT, SPEED_LSB, SPEED_MSB, CHECK, CAL_FORWARD, CAL_REVERSE, END} state_t;
typedef enum {NONE, SPEED, CALIBRATION, SAVE_CALIBRATION, AUTO_CALIBRATION} type_t;


state_t state = END;
type_t type = NONE;
int16_t speed = 0;
int16_t curSpeed = 0;
//uint8_t *p_curSpeed = (uint8_t *)&curSpeed;
uint8_t cal_forward = 0;
uint8_t cal_reverse = 0;
uint8_t sum = 0;

int16_t getSpeed(){
    return speed;
}

int syncId = 0;
uint8_t lookForSync(uint8_t c){
    if(c == syncWord[syncId]){
        syncId++;
        if(syncId >= 4){
            syncId = 0;
            return 1;
        }
    } else {
        syncId = 0;
    }
    return 0;
}


void stateMachine(uint8_t c){
    if(lookForSync(c)){
        state = INIT;
        sum = 0xFF;
    } else {
        sum ^= c;
        switch(state){
            case INIT: 
                if(c == SPEED_CMD){
                    type = SPEED;
                    state = SPEED_LSB;
                    curSpeed = 0;
                    break;
                }
                if(c == CALIBRATION_CMD){
                    type = CALIBRATION;
                    state = CAL_FORWARD;
                    break;
                }
                if(c == SAVE_CALIBRATION_CMD){
                    type = SAVE_CALIBRATION;
                    state = CHECK;
                }
                if(c == AUTO_CALIBRATION_CMD){
                    type = AUTO_CALIBRATION;
                    state = CHECK;
                }
                state = END;
            break;
            case SPEED_LSB:
                curSpeed = (c) & 0xFF;
                state = SPEED_MSB;
            break;
            case SPEED_MSB: 
                curSpeed |= (((int16_t)c)<<8);
                state = CHECK;
            break;
            case CAL_FORWARD:
                cal_forward = c;
                state = CAL_REVERSE;
            break;
            case CAL_REVERSE: 
                cal_reverse = c;
                state = CHECK;
            break;
            case CHECK: 
                if(sum == 0){
                    if(type == SPEED)   {
                        speed = curSpeed;
                        //printf("Speed : %d\n", speed);
                    } else
                    if(type == CALIBRATION){
                        ui8_s_motor_angle_forward = cal_forward;
	                    ui8_s_motor_angle_reverse = cal_reverse;
                    } else
                    if(type == SAVE_CALIBRATION){
                        eeprom_write(FORWARD_MOTOR_ANGLE, cal_forward);
                        eeprom_write(REVERSE_MOTOR_ANGLE, cal_reverse);
                    } else
                    if(type == AUTO_CALIBRATION){
                        doCalibration = 1;
                    }
                }
                state = END;
            break;
            case END: 
            break;
            default: 
            break;
        }
    }
}








// FIXME only needed as long as the other displays do not use new functions
// and instead have their own irq handler

 @svlreg @interrupt void UART2_IRQHandler(void) /*__interrupt(UART2_IRQHANDLER)*/ {

    // if there is something to read, read it into cyclic buffer
    if (UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET) {
        // assume data packet timeout after 20ms / new packet starting
        if (ui16_time_ticks_for_uart_timeout > UART_PACKET_TIMEOUT_TICKS) {
            ui8_rx_packet_start_pos = ui8_rx_fillpointer;
        }
        ui16_time_ticks_for_uart_timeout=0;
        
        ui8_uarx_buffer[ui8_rx_fillpointer++] = UART2_ReceiveData8();
        if (ui8_rx_fillpointer >= UART_RINGBUFFER_SIZE) {
            ui8_rx_fillpointer = 0;
        }
       //
        
    } else {

        // catch errors and process them in a reasonable manner which is always to ignore them :)
        if (UART2_GetITStatus(UART2_IT_IDLE) == SET) {
            UART2_ReceiveData8(); // -> clear!
        }
        if (UART2_GetITStatus(UART2_IT_LBDF) == SET) {
            UART2_ReceiveData8(); // -> clear!
        }
        if (UART2_GetITStatus(UART2_IT_OR) == SET) {
            UART2_ReceiveData8(); // -> clear!
        }
        if (UART2_GetITStatus(UART2_IT_PE) == SET) {
            UART2_ReceiveData8(); // -> clear!
        }
    }
}

void uart_send_if_avail(void) {
    // if there is something to send and we are not already sending
    if ((ui8_tx_fillpointer != ui8_tx_digestpointer)&&(UART2_GetFlagStatus(UART2_FLAG_TXE) != RESET)) {
        UART2_SendData8(ui8_uatx_buffer[ui8_tx_digestpointer++]);
        if (ui8_tx_digestpointer >= UART_RINGBUFFER_SIZE) {
            ui8_tx_digestpointer = 0;
        }
    }
}

void uart_fill_rx_packet_buffer(uint8_t *buffer, uint8_t bufferSize, uint8_t *bufferPos) {
    while (byte_avail_at_position() != UART_EMPTY_INDICATOR) {

        uint8_t uart_packet_start_pos = uart_get_packet_start_pos();
        uint8_t byte_avail_at_pos = byte_avail_at_position();

        if ((*bufferPos == 0) && (uart_packet_start_pos != byte_avail_at_pos)) {
            uart_get_buffered(); //discard incomplete chunks
        } else if (*bufferPos < bufferSize) {
            *(buffer + (*bufferPos)) = uart_get_buffered();
            (*bufferPos)++;
        }
    }
}

void uart_init(void) {
    UART2_DeInit();
    UART2_Init((uint32_t) 9600,
            UART2_WORDLENGTH_8D,
            UART2_STOPBITS_1,
            UART2_PARITY_NO,
            UART2_SYNCMODE_CLOCK_DISABLE,
            UART2_MODE_TXRX_ENABLE);
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
}

/*#if __SDCC_REVISION < 9624

void putchar(char c) {
    //Write a character to the UART2
    UART2_SendData8(c);

    //Loop until the end of transmission
    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
}

void putbyte(uint8_t c) {
    //Write a character to the UART2
    UART2_SendData8(c);

    //Loop until the end of transmission
    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
}
#else*/

char putchar(char c) {
    //Write a character to the UART2
    UART2_SendData8(c);

    //Loop until the end of transmission
    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);

    return ((char) c);
}
//#endif

#if __SDCC_REVISION < 9989
char getchar(void)
#else

int getchar(void)
#endif
{
    uint8_t c = 0;



    /* Loop until the Read data register flag is SET */
    while (UART2_GetFlagStatus(UART2_FLAG_RXNE) == RESET);

    c = UART2_ReceiveData8();

    return (c);
}
