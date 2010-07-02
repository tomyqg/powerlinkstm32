/**
 * @file drvTask.h
 *
 *  @date 01/02/2010
 *  @Author Alessio Tognazzolo
 *  @Author	Massimo Trubia
 */
#ifndef DRVTASK_H_
#define DRVTASK_H_



#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "queue.h"

//#include "stm32f10x_exti.h"
//#include "stm32f10x_usart.h"
//#include "stm32_eval.h"

#define configDRVTASK_PRIORITY	2

extern xQueueHandle g_lcdQueue;
extern xQueueHandle eventQueue;


int drvInitTask(void *pParams);
void drvTaskInit(void *pParams);
void ledToggle();
void ledToggle2();
void ledToggle3();


#endif
