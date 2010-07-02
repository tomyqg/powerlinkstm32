/** @file STM32Timer.h
 *
 *  @date 16/02/2010
 *  @Author Alessio Tognazzolo
 *  @Author	Massimo Trubia
 *
 */
#ifndef STM32TIMER_H_
#define STM32TIMER_H_

#include "../../Include/EplTarget.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "global_FreeRTOS.h"
#include "EplTimer.h"


DWORD PUBLIC EplTgtGetTickCountMs (void);

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
#define TIMER_COUNT                 2
/* The highest available interrupt priority. */
#define timerHIGHEST_PRIORITY			( 0 )
/* Calculates frequency in hertz from period in nanoseconds */
#define CALCULATE_FREQUENCY(ullTimeNs_p) (short)((float)(1/((float)ullTimeNs_p/1000000000)))
//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------
void interruptFindFactors(uint32_t , uint16_t *, uint16_t *);
//---------------------------------------------------------------------------
// local variables
//---------------------------------------------------------------------------

typedef struct
{
	unsigned long long  m_ullTimeNs;
	tEplTimerHdl* m_pTimerHdl;
	TIM_TypeDef* m_TIMx;
	BYTE m_used;
	uint32_t  m_RCC_APB1Periph;
	uint8_t m_TIMx_IRQn;
	tEplTimerkCallback  m_pfnCallback;
	BOOL m_fContinuously; //if FALSE timer one shot, else continuously timer interrupts occur
} tEplTimerHighReskTimerInfo;




//Handle for the eventQueue (where interrupt events are put in the queue)
extern xQueueHandle eventQueue;

#endif
