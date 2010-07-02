/*
 * global_FreeRTOS.h
 *
 *  Created on: 22/feb/2010
 *      Author: Massimo
 */

#ifndef GLOBAL_FREERTOS_H_
#define GLOBAL_FREERTOS_H_

#include "queue.h"

/* This queue is used for uncoupling the EPL stack execution from hardware interrupts.
 * Interrupts events are added to the queue and managed by a task that executes handlers (rxhandler, txhandler, timer callbacks)
 */
//extern xQueueHandle eventQueue;

#define FRAME_RECEIVED_INTERRUPT 	1
#define FRAME_TRANSMITTED_INTERRUPT 2
#define TIMER1_TIMEOUT_INTERRUPT 	4
#define TIMER2_TIMEOUT_INTERRUPT 	8

#define EVENT_QUEUE_SIZE 			50
#define COMMANDSTOPROCESS_QUEUE_SIZE 3
#define COMMANDSPROCESSED_QUEUE_SIZE 3

/* This struct is the primitive type for eventQueue elements
 * interrupt_type may assume only 4 values (FRAME_RECEIVED_INTERRUPT,FRAME_TRANSMITTED_INTERRUPT,TIMER1_TIMEOUT_INTERRUPT,TIMER2_TIMEOUT_INTERRUPT)*/
typedef struct Event
{
	int interrupt_type;
} eventQueueElement;

#endif /* GLOBAL_FREERTOS_H_ */
