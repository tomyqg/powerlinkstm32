#include "EdrvF107.h"
#include "EplDllk.h"

volatile ETH_DMADESCTypeDef xTxDescriptor __attribute__((aligned(4)));

static unsigned char ucMACBuffers[NUM_BUFFERS][MAX_PACKET_SIZE] __attribute__((aligned(4)));
//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------
/*! \var Allocate the Rx descriptors used by the DMA. */
static ETH_DMADESCTypeDef xRxDescriptors[NUM_RX_DESCRIPTORS] __attribute__((aligned(4)));
/* Allocate the descriptor used for transmitting.  It might be that better
 performance could be achieved by having more than one Tx descriptor, but
 in this simple case only one is used. */
//static volatile ETH_DMADESCTypeDef xTxDescriptor __attribute__((aligned(4)));
/* Buffers used for receiving and transmitting data. */


/*!\var Each ucBufferInUse index corresponds to a position in the same index in the
 ucMACBuffers array.  If the index contains a 1 then the buffer within
 ucMACBuffers is in use, if it contains a 0 then the buffer is free. */
static unsigned char ucBufferInUse[NUM_BUFFERS] = { 0 };
/*! \var Index to the Rx descriptor to inspect next when looking for a received
 packet. */
static unsigned long s_ulNextDescriptor;
/*! \var this structure contains platform specific information about transmission and receiving buffer pointers */
tEdrvInstance EdrvInstance_l;
#define ETH_CRC_SIZE	 4      // size of Ethernet CRC, i.e. FCS

/*!
 *\brief: function for init of the Ethernet controller
 *
 *\param  pEdrvInitParam_p    pointer to struct including the init-parameters
 *
 *\return     Errorcode           kEplSuccessful
 *                                kEplNoResource
 */
tEplKernel EdrvInit(tEdrvInitParam * pEdrvInitParam_p)
{

	tEplKernel  Ret;
	int iResult;
    Ret = kEplSuccessful;
    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));
    // save the init data
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;

    if(xEthInitialise() == pdPASS){

    	return Ret;
    }

    else return kEplEdrvInitError ;
}


/*!
 *
 *
 *\brief Shutdown the Ethernet controller
 *
 *\param  void
 *
 *return tEplKernel   kEplSuccessful
 *
 */
tEplKernel EdrvShutdown(void)
{


	ETH_PowerDownCmd(ENABLE);
	return kEplSuccessful;
}

/*!
 *\brief Set a multicast entry into the Ethernet controller
 *
 *\param pbMacAddr_p     pointer to multicast entry to set
 *
 *\return Errorcode      kEplSuccessful
 *
 */
tEplKernel EdrvDefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
	/* Multicast MAC address is set up in xEthInitialize. */
	return kEplSuccessful;
}


/*!
 *
 * \brief Reset a multicast entry in the Ethernet controller
 *
 * param  pbMacAddr_p     pointer to multicast entry to reset
 *
 * returns tEplKernel      kEplSuccessful
 *
 */
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{

	ETH_MACAddressPerfectFilterCmd(ETH_MAC_Address1, DISABLE);
	return kEplSuccessful;
}

/*!
 *
 *\brief Register a Tx-Buffer
 *
 *\param  pBuffer_p   pointer to Buffer structure
 *
 *\return tEplKernel  kEplSuccessful
 *                    kEplEdrvNoFreeBufEntry
 *
 */
tEplKernel EdrvAllocTxMsgBuffer       (tEdrvTxBuffer * pBuffer_p)
{
 int i;
	pBuffer_p->m_pbBuffer = prvGetNextBuffer();



	if(pBuffer_p->m_pbBuffer==NULL)
	{
		return kEplDllTxBufNotReady;
	}
	return kEplSuccessful;

}


/*!
* \brief Register a Tx-Buffer
*
* \param pBuffer_p   pointer to Buffer structure
*
* return Errorcode   kEplSuccessful
*
*/
tEplKernel EdrvReleaseTxMsgBuffer     (tEdrvTxBuffer * pBuffer_p)
{
	vReturnBuffer((unsigned char *) pBuffer_p->m_pbBuffer);

    return kEplSuccessful;

}


/*!
 *  \brief immediately starts the transmission of the buffer
 *
 * \param  pBuffer_p   buffer descriptor to transmit
 *
 * \return tEplKernel  kEplSuccessful
 */
tEplKernel EdrvSendTxMsg              (tEdrvTxBuffer * pBuffer_p)
{
	 EdrvInstance_l.m_pLastTransmittedTxBuffer = pBuffer_p;
	 MACTx_Handler((unsigned int)pBuffer_p->m_uiTxMsgLen, pBuffer_p->m_pbBuffer);
	 return kEplSuccessful;
}


/*!
 *\brief starts copying the buffer to the ethernet controller's FIFO
 *
 *\param  pbBuffer_p  buffer descriptor to transmit
 *
 *\return tEplKernel  kEplSuccessful
 *
 */
tEplKernel EdrvTxMsgReady              (tEdrvTxBuffer * pBuffer_p)
{

tEplKernel Ret = kEplSuccessful;
unsigned int uiBufferNumber;


Exit:
    return Ret;
}


/*!
* \brief starts transmission of the ethernet controller's FIFO
*
* \param pbBuffer_p  bufferdescriptor to transmit
*
* \return     tEplKernel  kEplSuccessful
*
*/
tEplKernel EdrvTxMsgStart              (tEdrvTxBuffer * pBuffer_p)
{
	tEplKernel Ret = kEplSuccessful;
    return Ret;
}

/*!
 *\brief Initialize all IO and peripherals required for Ethernet communication.
 *
 *\param void
 *
 *\returns pdSUCCESS if success, pdFAIL to signal some error.
 *
 */
static portBASE_TYPE xEthInitialise(void) {
	static ETH_InitTypeDef xEthInit; /* Static so as not to take up too much stack space. */
	BYTE multicast_universal_address[6];

	portBASE_TYPE xReturn;
	unsigned long ul;

	/* Start with things in a safe known state. */
	ETH_DeInit();
	for (ul = 0; ul < NUM_RX_DESCRIPTORS; ul++) {
		ETH_DMARxDescReceiveITConfig(&(xRxDescriptors[ul]), DISABLE);
	}

	/* Route clock to the peripheral. */
	RCC->AHBENR |= (RCC_MAC_CLOCK | RCC_MAC_TX_CLOCK
			| RCC_MAC_RX_CLOCK);

	/* Use MII mode. */
	AFIO->MAPR &= ~(MODE_MII);

	/* Configure all the GPIO as required for MAC/PHY interfacing. */
	prvSetupEthGPIO();

	/* Reset the peripheral. */
	ETH_SoftwareReset();
	while (ETH_GetSoftwareResetStatus() == SET);

	/* Set the MAC address. */
	ETH_MACAddressConfig(ETH_MAC_Address0,(unsigned char *) EdrvInstance_l.m_InitParam.m_abMyMacAddr);

	/* Set the multicast MAC address */
	AmiSetQword48ToBe(&multicast_universal_address[0], EPL_C_DLL_GENERIC_MULTICAST);
	ETH_MACAddressConfig(ETH_MAC_Address1, multicast_universal_address);
	ETH_MACAddressMaskBytesFilterConfig(ETH_MAC_Address1,ETH_MAC_AddressMask_Byte6);
	ETH_MACAddressPerfectFilterCmd(ETH_MAC_Address1, ENABLE);


	/* Initialise using the whopping big structure.  Code space could be saved
	 *
	 by making this a const struct, however that would mean changes to the
	 structure within the library header files could break the code, so for now
	 just set everything manually at run time. */
	ETH_StructInit(&xEthInit);
	xEthInit.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
	xEthInit.ETH_Watchdog = ETH_Watchdog_Disable;
	xEthInit.ETH_Jabber = ETH_Jabber_Disable;
	xEthInit.ETH_ReceiveOwn = ETH_ReceiveOwn_Disable;
	//xEthInit.ETH_ChecksumOffload = ETH_ChecksumOffload_Disable;
	#ifdef CHECKSUM_BY_HARDWARE
	xEthInit.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
	#else
	xEthInit.ETH_ChecksumOffload = ETH_ChecksumOffload_Disable;
	#endif
	xEthInit.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
	xEthInit.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
	xEthInit.ETH_SourceAddrFilter = ETH_SourceAddrFilter_Disable;
	xEthInit.ETH_PassControlFrames = ETH_PassControlFrames_ForwardPassedAddrFilter;
	xEthInit.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
	// ------------------ DMA ------------------

	xReturn = ETH_Init(&xEthInit, PHY_ADDRESS);

	/* Check a link was established. */
	if (xReturn != pdFAIL) {
		/* Rx and Tx interrupts are used. */
		ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, ENABLE);

		/* Only a single Tx descriptor is used.  For now it is set to use an Rx
		 buffer, but will get updated to point to where ever epl_buffer is
		 pointing prior to its use. */
		ETH_DMATxDescChainInit((void *) &xTxDescriptor, (void *) ucMACBuffers, 1);
		ETH_DMARxDescChainInit(xRxDescriptors, (void *) ucMACBuffers, NUM_RX_DESCRIPTORS);
		for (ul = 0; ul < NUM_RX_DESCRIPTORS; ul++) {
			/* Ensure received data generates an interrupt. */
			ETH_DMARxDescReceiveITConfig(&(xRxDescriptors[ul]), ENABLE);

			/* Fix up the addresses used by the descriptors.
			 The way ETH_DMARxDescChainInit() is not compatible with the buffer
			 declarations in this file. */
			xRxDescriptors[ul].Buffer1Addr
					= (unsigned long) &(ucMACBuffers[ul][0]);

			/* Mark the buffer used by this descriptor as in use. */
			ucBufferInUse[ul] = pdTRUE;
		}

		/* When receiving data, start at the first descriptor. */
		s_ulNextDescriptor = 0;

		//enableInterrupt();

		/* Buffers and descriptors are all set up, now enable the MAC. */
		ETH_Start();

		/* Let the DMA know there are Rx descriptors available. */
		prvRxDescriptorAvailable();


	}
	return xReturn;
}


/*! \brief Configure the IO for Ethernet use.
 *
 * \param  void
 *
 * \return void
 *
 */
static void prvSetupEthGPIO(void) {
	GPIO_InitTypeDef xEthInit;

	/* Remap MAC IO. */
	AFIO->MAPR |= (REMAP_MAC_IO);

	/* Set PA2, PA8, PB5, PB8, PB11, PB12, PB13, PC1 and PC2 for Ethernet
	 interfacing. */
	xEthInit.GPIO_Pin = GPIO_Pin_2;/* | GPIO_Pin_8; This should be set when the 25MHz is generated by MCO. */
	xEthInit.GPIO_Speed = GPIO_Speed_50MHz;
	xEthInit.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &xEthInit);

	xEthInit.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; /*5*/
	GPIO_Init(GPIOB, &xEthInit);

	xEthInit.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOC, &xEthInit);

	/* Configure PA0, PA1, PA3, PB10, PC3, PD8, PD9, PD10, PD11 and PD12 as
	 inputs. */
	xEthInit.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
	xEthInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &xEthInit);

	xEthInit.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &xEthInit);

	xEthInit.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &xEthInit);

	xEthInit.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOD, &xEthInit);
}

/*!
 *  \brief enables interrupt channel for receiving interrupt from Ethernet controller
 *  \param  void
 *  \return     void
*/
void enableInterrupt(){
	NVIC_InitTypeDef xNVICInit;
	/* Switch on the interrupts in the NVIC. */
	xNVICInit.NVIC_IRQChannel = ETH_IRQn;
	xNVICInit.NVIC_IRQChannelPreemptionPriority	= 15;
	xNVICInit.NVIC_IRQChannelSubPriority = 0;
	xNVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&xNVICInit);
}

/*!
 *\brief  disables interrupt channel for stopping receiving interrupt from Ethernet controller
 *
 * \param  void
 *
 * \return void
 *
 */
void disableInterrupt(){

	NVIC_InitTypeDef xNVICInit;
	/* Switch on the interrupts in the NVIC. */
	xNVICInit.NVIC_IRQChannel = ETH_IRQn;
	xNVICInit.NVIC_IRQChannelPreemptionPriority
			= configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	xNVICInit.NVIC_IRQChannelSubPriority = 0;
	xNVICInit.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&xNVICInit);

}

/*!
 *\brief This routine is called by hardware at every interrupt. We are interested only at receiving and sending interrupts.
 *
 *\param void
 *
 *\return void
*/
void vMAC_ISR()
{
	tEplFrameInfo   FrameInfo;
	tEplMsgType     MsgType;
	tEplFrame      *pFrame;

	eventQueueElement event;
	unsigned long ulStatus;
	long xHigherPriorityTaskWoken = pdFALSE;
	event.interrupt_type=0;

	//ledToggle3();
	/* What caused the interrupt? */
	ulStatus = ETH->DMASR;
	/* Clear everything before leaving. */
	ETH->DMASR = ulStatus;

	if (ulStatus & ETH_DMA_IT_R) {
		event.interrupt_type=FRAME_RECEIVED_INTERRUPT;

		xQueueSendFromISR(eventQueue, &event ,&xHigherPriorityTaskWoken);
	}
	else if (ulStatus & ETH_DMA_IT_T) {
		event.interrupt_type=FRAME_TRANSMITTED_INTERRUPT;
		xQueueSendFromISR(eventQueue, &event ,&xHigherPriorityTaskWoken);
	}

	/* If xQueueSendFromISR() unblocked a task, and the unblocked task has
	 a higher priority than the currently executing task, then
	 xHigherPriorityTaskWoken will have been set to pdTRUE and this ISR should
	 return directly to the higher priority unblocked task. */
	 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*!
 *\brief Releases the pucBuffer
 *
 *\param  pucBuffer  is the buffer to free (it is returned to the pool of free buffers used by DMA)
 *
 *\return     void
 */
void vReturnBuffer(unsigned char *pucBuffer) {
	unsigned long ul;

	/* Mark a buffer as free for use. */
	for (ul = 0; ul < NUM_BUFFERS; ul++) {
		if (ucMACBuffers[ul] == pucBuffer) {
			ucBufferInUse[ul] = pdFALSE;
			break;
		}
	}

}

/*!
* \brief Get a free buffer
*
* \param  void
*
* \return unsigned char   a pointer to the free buffer obtained from the pool of  buffers (ucMACBuffers)
*
*/


unsigned char *prvGetNextBuffer(void) {
	portBASE_TYPE x;
	unsigned char *ucReturn = NULL;

	while (ucReturn == NULL) {
		/* Look through the buffers to find one that is not in use by
		 anything else. */
		for (x = 0; x < NUM_BUFFERS; x++) {
			if (ucBufferInUse[x] == pdFALSE) {
				ucBufferInUse[x] = pdTRUE;
				ucReturn = &(ucMACBuffers[x][0]);
				break;
			}
		}
		//xTaskResumeAll();

		/* Was a buffer found? */
		if (ucReturn == NULL) {
			/* Wait then look again. */
			vTaskDelay(netifBUFFER_WAIT_DELAY);
		}
	}

	return ucReturn;
}

/*!
* \brief This handler is used for freeing a descriptor and sending the buffer it contains to the stack
*
* \param  void
*
* \return number of bytes received
*/
unsigned short MACRx_Handler(void) {
	unsigned short usReturn;
	tEdrvRxBuffer RxBuffer; //buffer sent to the DLL
	tEplFrame      *pFrame;
	tEplMsgType     MsgType;

	if ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_ES) != 0) {
		/* Error in Rx.  Discard the frame and give it back to the DMA. */
		xRxDescriptors[s_ulNextDescriptor].Status = ETH_DMARxDesc_OWN;
		prvRxDescriptorAvailable();

		/* No data to return. */
		usReturn = 0UL;

		/* Start from the next descriptor the next time this function is called. */
		s_ulNextDescriptor++;
		if (s_ulNextDescriptor >= NUM_RX_DESCRIPTORS) {
			s_ulNextDescriptor = 0UL;
		}
	} else if ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_OWN)
			== 0) {

		/* Get the received data length	from the top 2 bytes of the Status
		 word and the data itself. */
		usReturn = (unsigned short) ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_FL) >> 16UL);
		RxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
		RxBuffer.m_uiRxMsgLen = (unsigned short) ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_FL) >> 16UL);
		RxBuffer.m_uiRxMsgLen-=ETH_CRC_SIZE;
		RxBuffer.m_pbBuffer =(unsigned char *) (xRxDescriptors[s_ulNextDescriptor].Buffer1Addr);

		EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);
		pFrame = (tEplFrame *) RxBuffer.m_pbBuffer;
		MsgType = (tEplMsgType)AmiGetByteFromLe(&pFrame->m_le_bMessageType);

		if(MsgType==kEplMsgTypeSoa)

		EplApiPostUserEvent(kEplEventSourceSdoAsnd);

		/* Mark the current buffer as free as epl_buffer is going to be set to
		 the buffer that contains the received data. */
		vReturnBuffer((unsigned char *) (xRxDescriptors[s_ulNextDescriptor].Buffer1Addr));



		/* Allocate a new buffer to the descriptor. */
		xRxDescriptors[s_ulNextDescriptor].Buffer1Addr = (unsigned long) prvGetNextBuffer();

		/* Give the descriptor back to the DMA. */
		xRxDescriptors[s_ulNextDescriptor].Status = ETH_DMARxDesc_OWN;
		prvRxDescriptorAvailable();

		/* Start from the next descriptor the next time this function is called. */
		s_ulNextDescriptor++;
		if (s_ulNextDescriptor >= NUM_RX_DESCRIPTORS) {
			s_ulNextDescriptor = 0UL;
		}
	} else {
		/* No received data at all. */
		usReturn = 0UL;
	}
   	return usReturn;
}


/*! \brief This handler is used for acquiring a new descriptor and sending a buffer
*
* \param usDataLen length of packet inside the buffer
* \param buffer    buffer containing the frame to send
*
* \return     1 - Successful   0 - Error
*/


unsigned char MACTx_Handler(unsigned int usDataLen,unsigned char * buffer) {
	unsigned long ulAttempts = 0UL;
	unsigned char res = 1;
	/* Check to see if the Tx descriptor is free.  The check against <2 is to
	 ensure the buffer has been sent twice and in so doing preventing a race
	 condition with the DMA on the ETH_DMATxDesc_OWN bit. */
	while ((xTxDescriptor.Status & ETH_DMATxDesc_OWN) == ETH_DMATxDesc_OWN) {
		/* Wait for the Tx descriptor to become available. */
		vTaskDelay(netifBUFFER_WAIT_DELAY);

		ulAttempts++;
		if (ulAttempts > netifBUFFER_WAIT_ATTEMPTS) {
			/* Something has gone wrong as the Tx descriptor is still in use.
			 Clear it down manually, the data it was sending will probably be
			 lost. */
			xTxDescriptor.Status &= ~ETH_DMATxDesc_OWN;
			vReturnBuffer((unsigned char *) xTxDescriptor.Buffer1Addr);
			res = 0;
			break;
		}
	}


	/* Setup the Tx descriptor for transmission. */
	xTxDescriptor.SendCount = 0;
	xTxDescriptor.Buffer1Addr = (unsigned long) buffer;
	xTxDescriptor.ControlBufferSize = (unsigned long) usDataLen;
	xTxDescriptor.Status = ETH_DMATxDesc_OWN | ETH_DMATxDesc_LS
			| ETH_DMATxDesc_FS | ETH_DMATxDesc_TER | ETH_DMATxDesc_TCH
			| ETH_DMATxDesc_IC;
	ETH->DMASR = ETH_DMASR_TBUS;
	ETH->DMATPDR = 0;

	return res;
}


tEplKernel EdrvChangeFilter(tEdrvFilter*    pFilter_p,
                            unsigned int    uiCount_p,
                            unsigned int    uiEntryChanged_p,
                            unsigned int    uiChangeFlags_p)
{
tEplKernel      Ret = kEplSuccessful;


    return Ret;
}
