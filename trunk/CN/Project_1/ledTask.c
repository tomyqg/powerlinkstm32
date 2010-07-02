/**
 * \file ledTask.c
 *
 *  Created on: Nov 2, 2009
 *      Author: Stefano Oliveri
 */

#include "ledTask.h"
#include "task.h"
#include "semphr.h"

//#include "lcdTask.h"

static void ledToggle();
static void ledTaskFunc();
static void ledFindFactors(uint32_t n, uint16_t *a, uint16_t *b);

/**
 * This resource is used to synchronize the TIM2 IRQ and the iNemo data acquisition task.
 */
static xSemaphoreHandle s_timSemaphore;

/**
 * Initialize all hardware resource needed by the task.
 *
 * @param pParams struct of LedInitParams containing the blinking frequency.
 * @return 0 if success, a task specific error code otherwise.
 */
int ledInitTask(void *pParams)
{
	unsigned short a;
	unsigned short b;
	unsigned long n;
	LedInitParams *params;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	params = (LedInitParams*)pParams;

	/* Enable GPIOD, GPIOE clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOD, ENABLE );

	/* Configure PD13 output push-pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	// Enable timer clocks
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

	// Time base configuration for timer 2 - which generates the interrupts.
	n = (configCPU_CLOCK_HZ / params->toggleFrequency);

	ledFindFactors( n, &a, &b );
	TIM_TimeBaseStructure.TIM_Period = b - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = a - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( TIM2, ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	return 0;
}

/**
 * Initialize all software resource needed by the led task.
 *
 * @param pParams not used
 */
void ledStartTask(void *pParams)
{
	// Create the semaphore used to synchronize  the led  task and the TIM2 interrupt service routine.
	vSemaphoreCreateBinary(s_timSemaphore);
	if (!s_timSemaphore) {
		// Error in resource creation.
		while (1);
	}

	// Create the led task
	xTaskCreate(ledTaskFunc, "led_irq", configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);
}

/**
 * Task control function.
 *
 * @param pParams not used.
 */
static void ledTaskFunc(void *pParams)
{
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);


	for (;;) {
		if ( xSemaphoreTake(s_timSemaphore, portMAX_DELAY) == pdTRUE ) {
			ledToggle();
		}
	}

}

/**
 * Toggle the pin E14.
 */
void ledToggle()
{
	/*static bool b = FALSE;

	GPIO_WriteBit( GPIOD, GPIO_Pin_13, b ? Bit_SET : Bit_RESET  );
	b = !b;
	*/
}

/**
 * This function handles TIM2 global interrupt request.
 */
/*
void TIM2_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(s_timSemaphore, &xHigherPriorityTaskWoken);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
*/
/**
 * Compute two integer value a and b such that n = a * b. It used to
 * setup the timer to generate an IRQ whit a specified frequency n.
 *
 * @param n specifies the wanted frequency
 * @param a specifies the prescaler factor
 * @param b specifies the period factor
 */
void ledFindFactors(uint32_t n, uint16_t *a, uint16_t *b)
{
	/* This function is copied from the ST STR7 library and is
	copyright STMicroelectronics.  Reproduced with permission. */

	uint16_t b0;
	uint16_t a0;
	long err, err_min=n;


	*a = a0 = ((n-1)/0xffff) + 1;
	*b = b0 = n / *a;

	for (; *a < 0xffff-1; (*a)++)
	{
		*b = n / *a;
		err = (long)*a * (long)*b - (long)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (long)*a * (long)*b - (long)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}
