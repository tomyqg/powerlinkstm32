/**
 * \file led_sample_irq.h
 *
 *  Created on: NovledTask 2, 2009
 *      Author: Stefano Oliveri
 */

#ifndef LED_TEST_H_
#define LED_TEST_H_

#include "FreeRTOS.h"
#include "stm32f10x.h"

typedef struct {
	unsigned short toggleFrequency;
} LedInitParams;

int ledInitTask(void *pParams);
void ledStartTask(void *pParams);

#endif /* LED_TEST_H_ */
