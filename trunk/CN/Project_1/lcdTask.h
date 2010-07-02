/**
 * \file lcdTask.h
 *
 *  Created on: Nov 3, 2009
 *      Author: Stefano Oliveri
 */

#ifndef LCDTASK_H_
#define LCDTASK_H_

#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "queue.h"

typedef struct {
	char msg[21];
}LcdMsg;

/* The queue used to send messages to the LCD task. */
extern xQueueHandle g_lcdQueue;

int lcdInitTask(void *pParams);
int lcdStartTask(void *pParams);

#endif /* LCDTASK_H_ */
