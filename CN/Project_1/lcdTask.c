/**
 * \file lcdTask.c
 *
 *  Created on: Nov 3, 2009
 *      Author: Stefano Oliveri
 */

#include "lcdTask.h"
#include "STCode/STM3210c_eval_lcd.h"

/* The length of the queue used to send messages to the LCD task. */
#define LCD_QUEUE_SIZE	( 3 )


/**
 *  The queue used to send messages to the LCD task.
 */
xQueueHandle g_lcdQueue;

static void lcdTaskFunc(void *pParams);

/**
 * Initialize all hardware resource needed by the task.
 *
 * @param pParams not used.
 * @return 0 if success, a task specific error code otherwise.
 */
int lcdInitTask(void *pParams)
{
	// In this demo task the initialization is performed after the scheduler start.
	return 0;
}

/**
 * Initialize all software resources needed by the lcd task.
 *
 * @param pParams not used
 * @return 0 if success, a task specific error code otherwise.
 */
int lcdStartTask(void *pParams)
{
	/* Create the queue used by the LCD task.  Messages for display on the LCD
	are received via this queue. */
	g_lcdQueue = xQueueCreate( LCD_QUEUE_SIZE, sizeof( LcdMsg ) );

	xTaskCreate(lcdTaskFunc, "lcd", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);

	return 0;
}

/**
 * Task control function.
 *
 * @param pParams not used.
 */
static void lcdTaskFunc(void *pParams)
{
	unsigned long ulLine = Line3;
	const unsigned long ulLineHeight = 24;
	static char cMsgBuf[ 30 ];
	LcdMsg msg;

	/* The LCD gatekeeper task as described in the comments at the top of this
	file. */
	//Initialize the LCD.
	STM3210C_LCD_Init();
	// Display a startup message.
	LCD_Clear(White);
	LCD_SetTextColor(Green);
	LCD_DisplayStringLine( Line0, ( unsigned char * ) "  www.FreeRTOS.org" );
	LCD_SetTextColor(Blue);
	LCD_DisplayStringLine( Line1, ( unsigned char * ) "  Project 4 example" );
	LCD_SetTextColor(Black);

	for( ;; )
	{
		/* Wait for a message to arrive to be displayed. */
		xQueueReceive( g_lcdQueue, &msg, portMAX_DELAY );

		/* Clear the current line of text. */
		LCD_ClearLine( ulLine );

		/* Move on to the next line. */
		ulLine += ulLineHeight;
		if( ulLine > Line9 )
		{
			ulLine = Line3;
		}

		//Display the received text
		LCD_DisplayStringLine( ulLine, ( unsigned char * )msg.msg );
	}
}
