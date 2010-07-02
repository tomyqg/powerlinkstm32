/****************************************************************************

                $Author:Massimo Trubia - Alessio Tognazzolo $

                $Revision: 1.4 $  $Date: 2010/02/16 11:00:00 $

                $State: Exp $

                Build Environment:
                    eclipse +  gcc for arm

****************************************************************************/
#include "STM32Timer.h"
#include "kernel/EplTimerHighResk.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "global_FreeRTOS.h"
#include "queue.h"

tEplTimerHighReskTimerInfo timers_pool[TIMER_COUNT];



tEplKernel PUBLIC EplTimerHighReskInit(){
	tEplKernel Ret;
	Ret=EplTimerHighReskAddInstance();
	return Ret;
}


tEplKernel PUBLIC EplTimerHighReskAddInstance(){
	EPL_MEMSET(&timers_pool[0], 0, sizeof (tEplTimerHighReskTimerInfo));
	EPL_MEMSET(&timers_pool[1], 0, sizeof (tEplTimerHighReskTimerInfo));
	timers_pool[0].m_TIMx = TIM4;
	timers_pool[0].m_RCC_APB1Periph=RCC_APB1Periph_TIM4;
	timers_pool[0].m_TIMx_IRQn = TIM4_IRQn;
	timers_pool[1].m_TIMx = TIM5;
	timers_pool[1].m_RCC_APB1Periph=RCC_APB1Periph_TIM5;
	timers_pool[1].m_TIMx_IRQn = TIM5_IRQn;

	return kEplSuccessful;
}

tEplKernel PUBLIC EplTimerHighReskDelInstance(){
	TIM_Cmd(timers_pool[0].m_TIMx, DISABLE);
	TIM_Cmd(timers_pool[1].m_TIMx, DISABLE);
	EPL_MEMSET(&timers_pool[0], 0, sizeof (tEplTimerHighReskTimerInfo));
	EPL_MEMSET(&timers_pool[1], 0, sizeof (tEplTimerHighReskTimerInfo));
	return kEplSuccessful;
}

tEplKernel PUBLIC EplTimerHighReskSetTimerNs(tEplTimerHdl*     pTimerHdl_p,
                                    unsigned long long  ullTimeNs_p,
                                    tEplTimerkCallback  pfnCallback_p,
                                    unsigned long       ulArgument_p,
                                    BOOL                fContinuously_p)
{
	return kEplSuccessful;
}

tEplKernel PUBLIC EplTimerHighReskModifyTimerNs(tEplTimerHdl*     pTimerHdl_p,
                                    unsigned long long  ullTimeNs_p,
                                    tEplTimerkCallback  pfnCallback_p,
                                    unsigned long       ulArgument_p,
                                    BOOL                fContinuously_p)
{
	tEplKernel                  Ret = kEplSuccessful;
	unsigned int                uiIndex;
	unsigned int                uiPrescaler;
	tEplTimerHighReskTimerInfo* pTimerInfo;
	WORD                        wCounter;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        // search free timer info structure
        for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++)
        {
            if (timers_pool[uiIndex].m_used == 0)
            {   // free structure found, we set it as m_used
            	timers_pool[uiIndex].m_used=1;
            	break;
            }
        }
        if (uiIndex >= TIMER_COUNT)
        {   // no free structure found
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }

        *pTimerHdl_p=uiIndex+1;
        timers_pool[uiIndex].m_pTimerHdl = pTimerHdl_p;
        timers_pool[uiIndex].m_fContinuously = fContinuously_p;
    }
    else
    {
        uiIndex = (*pTimerHdl_p ) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
    }

    // disable the timer because we must disable the interrupt of the old timer with the same handle
    TIM_Cmd(timers_pool[uiIndex].m_TIMx, DISABLE);

    timers_pool[uiIndex].m_pfnCallback=pfnCallback_p;
    timers_pool[uiIndex].m_ullTimeNs = ullTimeNs_p;

    //initializes and starts the timer
    initializeTimer(timers_pool[uiIndex]);



Exit:
    return Ret;

}

tEplKernel PUBLIC EplTimerHighReskDeleteTimer(tEplTimerHdl*     pTimerHdl_p)
{
	TIM_Cmd(timers_pool[0].m_TIMx, DISABLE);
	TIM_Cmd(timers_pool[1].m_TIMx, DISABLE);
	EPL_MEMSET(&timers_pool[0], 0, sizeof (tEplTimerHighReskTimerInfo));
	EPL_MEMSET(&timers_pool[1], 0, sizeof (tEplTimerHighReskTimerInfo));
	return kEplSuccessful;
}


void initializeTimer(tEplTimerHighReskTimerInfo TIMx_p)
{
	unsigned long ulFrequency;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	unsigned short a;
	unsigned short b;

	/* Enable timer clocks */
	RCC_APB1PeriphClockCmd( TIMx_p.m_RCC_APB1Periph, ENABLE );

	/* Initialise data. */
	TIM_DeInit( TIMx_p.m_TIMx );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

	/* Time base configuration for timer - which generates the interrupts. */
	ulFrequency = configCPU_CLOCK_HZ /  CALCULATE_FREQUENCY(TIMx_p.m_ullTimeNs);
	interruptFindFactors( ulFrequency, &a, &b );
	TIM_TimeBaseStructure.TIM_Period = b - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = a - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(  TIMx_p.m_TIMx, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig(  TIMx_p.m_TIMx, ENABLE );


	/* Enable TIM IT. */
	NVIC_InitStructure.NVIC_IRQChannel = TIMx_p.m_TIMx_IRQn  ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
	TIM_ClearITPendingBit(  TIMx_p.m_TIMx, TIM_IT_Update );
	TIM_ITConfig( TIMx_p.m_TIMx, TIM_IT_Update, ENABLE );

	/* Finally, enable  timer */
	TIM_Cmd(TIMx_p.m_TIMx , ENABLE );
}

void TIM4_IRQHandler( void )
{
	long xHigherPriorityTaskWoken = pdFALSE;
	eventQueueElement event;
    TIM_ClearITPendingBit( TIM4, TIM_IT_Update );
	TIM_Cmd(TIM4,DISABLE);
	event.interrupt_type=TIMER1_TIMEOUT_INTERRUPT;
	xQueueSendFromISR(eventQueue, &event ,&xHigherPriorityTaskWoken);
}

void TIM5_IRQHandler( void )
{
	long xHigherPriorityTaskWoken = pdFALSE;
	eventQueueElement event;
    TIM_ClearITPendingBit( TIM5, TIM_IT_Update );
    TIM_Cmd(TIM5,DISABLE);
	event.interrupt_type=TIMER2_TIMEOUT_INTERRUPT;
	xQueueSendFromISR(eventQueue, &event ,&xHigherPriorityTaskWoken);
}

/*!
 * \brief Compute two integer value a and b such that n = a * b. It used to
 *        setup the timer to generate an IRQ whit a specified frequency n.
 *
 * \param n specifies the wanted frequency
 * \param a specifies the prescaler factor
 * \param b specifies the period factor
 */
void interruptFindFactors(uint32_t n, uint16_t *a, uint16_t *b)
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




DWORD EplTgtGetTickCountMs ()
{
	DWORD	TickCountMs;
	return TickCountMs;
}

