/*
 * comm.h
 *
 *  Created on: Jun 19, 2023
 *      Author: gabri
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "stdint.h"

#define PACKET_SIZE 					8		// UART payload size
#define TIMER_PRESCALER 				4000	// Prescaler for timers
#define INT_MAX 						255		// Maximum size for uint8_t
#define LIGHT_INTENSITY_PWM_MULTIPLIER 	100		// Number of times the light intensity PWM
 	 	 	 	 	 	 	 	 	 	 	 	// 	is faster than the stimulation frequency

void receiveUARTpacket();

#endif /* INC_COMM_H_ */
