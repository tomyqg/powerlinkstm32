/****************************************************************************


                $RCSfile: EdrvF107.c,v $

                $Author: Massimo Trubia, Alessio Tognazzolo $

                $Revision: 1.0 $  $Date: 2010/02/12 09:50:17 $

                $State: Exp $

                Build Environment:
                Eclipse + gnu gcc arm compiler


****************************************************************************/
#ifndef _EDRVF107_H_
#define _EDRVF107_H_

#include "global.h"
#include "EplInc.h"
#include "edrv.h"
#include "stm32f10x.h"
#include "stm32_eth.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "global_FreeRTOS.h"

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
#define NUM_RX_DESCRIPTORS 4
#define MAX_PACKET_SIZE							1520

/* The total number of buffers to be available.  At most (?) there should be
 one available for each Rx descriptor, one for current use, and one that is
 in the process of being transmitted. */
#define NUM_BUFFERS								( NUM_RX_DESCRIPTORS + 5 )

/* Let the DMA know that a new descriptor has been made available to it. */
#define prvRxDescriptorAvailable()				ETH->DMARPDR = 0

/* The field in the descriptor that is unused by this configuration is used to
 hold the send count.  This is just #defined to a meaningful name. */
#define SendCount Buffer2NextDescAddr


/* Hardware specifics. */
#define RCC_MAC_CLOCK							( 1UL << 14UL )
#define RCC_MAC_TX_CLOCK						( 1UL << 15UL )
#define RCC_MAC_RX_CLOCK						( 1UL << 16UL )
#define MODE_MII								( 1UL << 23UL )
#define PHY_ADDRESS								( 1 )
#define ENET_IRQ_NUM							( 61 )
#define REMAP_MAC_IO							( 1UL << 21UL )

/* Let the DMA know that a new descriptor has been made available to it. */
#define prvRxDescriptorAvailable()				ETH->DMARPDR = 0

/* If no buffers are available, then wait this long before looking again.... */
#define netifBUFFER_WAIT_DELAY					( 10 / portTICK_RATE_MS )
/* ...and don't look more than this many times. */
#define netifBUFFER_WAIT_ATTEMPTS				( 9 )
#define EPL_C_DLL_GENERIC_MULTICAST     0x01111E000007LL
//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// Private structure
typedef struct
{
    void*               m_pIoAddr;      // pointer to register space of Ethernet controller
    BYTE*               m_pbRxBuf;      // pointer to Rx buffer
    BYTE*               m_pbTxBuf;      // pointer to Tx buffer
    BOOL                m_afTxBufUsed[EDRV_MAX_TX_BUFFERS];
    unsigned int        m_uiCurTxDesc;

    tEdrvInitParam      m_InitParam;
    tEdrvTxBuffer*      m_pLastTransmittedTxBuffer;

} tEdrvInstance;


// Handle to global LCD queue
extern xQueueHandle g_lcdQueue;
//Handle for the eventQueue (where interrupt events are put in the queue)
extern xQueueHandle eventQueue;









/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <edrv>                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

/**
 * Configure the IO for Ethernet use.
 */
static void prvSetupEthGPIO(void);

static portBASE_TYPE xEthInitialise(void);

/**
 * Return a pointer to an unused buffer, marking the returned buffer as now
 * in use.
 */
unsigned char *prvGetNextBuffer(void);

void vReturnBuffer(unsigned char *);

unsigned short MACRx_Handler(void);

unsigned char MACTx_Handler(unsigned int,unsigned char *);




#endif
