/*!
 * \file drvTask.c
 *
 * \date Feb 1, 2010
 * \author Alessio Tognazzolo
 * \author Massimo Trubia
 */

#include "drvTask.h"
#include "edrv.h"
#include "epl.h"
#include "debugging.h"
#include "task.h"
#include "stm32f10x.h"
#include "EplDll.h"
#include "global_FreeRTOS.h"
#include "EdrvF107.h"
#include "EplTimer.h"
#include "STM32Timer.h"
#include "EplObd.h"         // function prototypes of the EplOBD-Modul
#include "user/EplPdou.h"   // function prototype of OD callback function
#include "obdcfg.h"
#include "EplObjDef.h"
#include "STCode/STM3210c_eval_lcd.h"
#include "PushButton.h"
#include "EplApiFreeRTOS.h"
#include "string.h"

extern volatile ETH_DMADESCTypeDef xTxDescriptor __attribute__((aligned(4)));
extern tEdrvInstance EdrvInstance_l;
#define EPL_OBD_INIT_RAM_NAME    EplObdInitRam;
// This function is the entry point for your object dictionary. It is defined
// in OBJDICT.C by define EPL_OBD_INIT_RAM_NAME. Use this function name to define
// this function prototype here. If you want to use more than one Epl
// instances then the function name of each object dictionary has to differ.
tEplKernel PUBLIC EplObdInitRam(tEplObdInitParam MEM* pInitParam_p);
void drvTaskFunc(void *pParams);
void frame_received_handler();
void frame_transmitted_handler();
void timer1_handler();
void timer2_handler();
void inizializza_led();
void TaskLocalObjectReader();
void TaskLocalObjectWriter();

static int handlercount = 0;
static int a = 1;
static int b = 0;
static int c = 1;
extern tEplTimerHighReskTimerInfo timers_pool[TIMER_COUNT];

/*! \var process variable */
BYTE bVarIn1;
/*! \var process variable */
BYTE bVarIn2;
/*! \var process variable */
BYTE bVarOut2;
/*! \var process variable */
BYTE bVarOut1;
BYTE bVarOut1Old_l;
BYTE abDomain_l[3];

typedef struct {
	unsigned short toggleFrequency;
} LedInitParams;

static tEplApiInitParam EplApiInitParam = { 0 };
/*!
 * \brief event callback function called by EPL API layer within
 *             user part (low priority).
 *
 * \param EventType_p 			  	event type
 * \param pEventArg_p       		pointer to union, which describes the event in detail
 * \param pUserArg_p  				user specific argument
 *
 * \return tEplKernel   			error code,
 *                                 kEplSuccessful = no error
 *                                 kEplReject = reject further processing
 *                                 otherwise = post error event to API layer
 */

tEplKernel PUBLIC AppCbEvent(tEplApiEventType EventType_p, // IN: event type (enum)
		tEplApiEventArg* pEventArg_p, // IN: event argument (union)
		void GENERIC* pUserArg_p) {
	static int enable_multitasking = 0;
	extern xQueueHandle ipcCommandsToProcessQueue;
	extern xQueueHandle ipcCommandsProcessedQueue;
	tEplKernel EplRet = kEplSuccessful;
	Command command;
	BYTE value = 12;
	int size = sizeof(value);

	// check if NMT_GS_OFF is reached
	switch (EventType_p) {
	case kEplApiEventUserDef:
		//we control if arrived a Asnd frame
		if (pEventArg_p->m_pUserArg == kEplEventSourceSdoAsnd) {
			//we want be sure we can process commands only when queues are created and CN is operational
			//because we can have a Asnd frame event without be operational.
			if (enable_multitasking != 0) {
				if (ipcCommandsToProcessQueue != 0) {
					if (xQueueReceive(ipcCommandsToProcessQueue,&command,1)
							==pdTRUE) {
						processCommand(&command);
						xQueueSend(ipcCommandsProcessedQueue,&command,1);
						//wake up task who was slept
						vTaskResume(command.handle);
					}
				}
			}
		}
		break;
	case kEplApiEventNmtStateChange: {
		switch (pEventArg_p->m_NmtStateChange.m_NewNmtState) {
		case kEplNmtGsOff: { // NMT state machine was shut down,
			// because of user signal (CTRL-C) or critical EPL stack error
			// -> also shut down EplApiProcess() and main()
			EplRet = kEplShutdown;

			break;
		}

		case kEplNmtGsResetCommunication: {

			break;
		}

		case kEplNmtGsResetConfiguration: {
			unsigned int uiSize;

			// fetch object 0x1006 NMT_CycleLen_U32 from local OD (in little endian byte order)
			// for configuration of remote CN
			uiSize = 4;
			EplRet = EplApiReadObject(NULL, 0, 0x1006, 0x00,
					&EplApiInitParam.m_dwCycleLen, &uiSize, kEplSdoTypeAsnd,
					NULL);
			if (EplRet != kEplSuccessful) { // local OD access failed
				break;
			}

			uiSize = 4;
			EplRet = EplApiReadObject(NULL, 0, 0x1C14, 0x00,
					&EplApiInitParam.m_dwLossOfFrameTolerance, &uiSize,
					kEplSdoTypeAsnd, NULL);
			if (EplRet != kEplSuccessful) { // local OD access failed
				break;
			}

			if (pEventArg_p->m_NmtStateChange.m_OldNmtState
					== kEplNmtCsPreOperational2
					&& pEventArg_p->m_NmtStateChange.m_NmtEvent
							== kEplNmtEventResetConfig) {
				NVIC_SystemReset();

			}

			break;
		}

		case kEplNmtMsPreOperational1: {

			// continue
		}

		case kEplNmtGsInitialising:
		case kEplNmtGsResetApplication:
		case kEplNmtMsNotActive:
		case kEplNmtCsNotActive: {

			break;
		}
		case kEplNmtCsPreOperational1: {
			break;
		}
		case kEplNmtCsOperational: {
			//create the queues
			ipcCommandsToProcessQueue = xQueueCreate(
					COMMANDSTOPROCESS_QUEUE_SIZE, sizeof(Command));
			ipcCommandsProcessedQueue = xQueueCreate(
					COMMANDSPROCESSED_QUEUE_SIZE, sizeof(Command));
			xTaskCreate(TaskLocalObjectReader,"localreader",configMINIMAL_STACK_SIZE, NULL, configDRVTASK_PRIORITY+1 , NULL);
			//xTaskCreate(TaskLocalObjectWriter,"localwriter",configMINIMAL_STACK_SIZE, NULL, configDRVTASK_PRIORITY+1 , NULL);
			//we enable multitasking only when openPOWERLINK is operational to be sure we can access to OD safely
			enable_multitasking = 1;

			break;
		}
		case kEplNmtMsOperational: {
			break;
		}
		default: {
			break;
		}
		}

		break;
	}

	case kEplApiEventCriticalError:
	case kEplApiEventWarning: { // error or warning occured within the stack or the application
		// on error the API layer stops the NMT state machine


		// check additional argument
		switch (pEventArg_p->m_InternalError.m_EventSource) {
		case kEplEventSourceEventk:
		case kEplEventSourceEventu: { // error occured within event processing
			// either in kernel or in user part

			break;
		}

		case kEplEventSourceDllk: { // error occured within the data link layer (e.g. interrupt processing)
			// the DWORD argument contains the DLL state and the NMT event

			break;
		}

		default: {

			break;
		}
		}
		break;
	}

	default:
		break;
	}

	return EplRet;
}

/*!
 *
 *
 * \brief sync event callback function called by event module within
 *        kernel part (high priority).
 *        This function sets the outputs, reads the inputs and runs
 *        the control loop.
 *
 * \param  void
 *
 * \return     tEplKernel        error code,
 *                               kEplSuccessful = no error
 *                               otherwise = post error event to API layer
 *
 */
tEplKernel PUBLIC AppCbSync(void) {

#ifdef CN1
	bVarOut1=0;
#endif
#ifdef CN2

	if (bVarOut2 == 1) {

		LCD_DisplayStringLine(Line0, (unsigned char *) "Wake UP");
	} else if (bVarOut2 == 2) {
		ledToggle2();
		LCD_DisplayStringLine(Line1, (unsigned char *) "Key");

	} else {
		LCD_DisplayStringLine(Line0, (unsigned char *) "          ");
		LCD_DisplayStringLine(Line1, (unsigned char *) "          ");

	}

#endif
	tEplKernel EplRet = kEplSuccessful;

#ifdef CN1

	if(PushButtonGetState(Button_TAMPER)==0)
	bVarIn1=1;
	else if(PushButtonGetState(Button_KEY)==0)
	bVarIn1=2;
	else
	bVarIn1=0;
#endif
#ifdef CN2

	bVarIn2=65;
#endif

	return kEplSuccessful;
}
/*!
 *
 * \brief function that maps logican variables in OD
 *
 * \param  void
 *
 * \return     tEplKernel      error code,
 *                             kEplSuccessful = no error
 *
 *
 */
int variablesMapping() {
	tEplObdSize ObdSize;
	unsigned int uiVarEntries;
	tEplKernel EplRet;
	int iRet;
	// link process variables used by CN to object dictionary
	ObdSize = sizeof(bVarIn1);
	uiVarEntries = 1;
#ifdef CN1
	EplRet = EplApiLinkObject(0x6000, &bVarIn1, &uiVarEntries, &ObdSize, 0x01);
#endif
#ifdef CN2
	EplRet
			= EplApiLinkObject(0x6003, &bVarIn2, &uiVarEntries, &ObdSize,
					0x01);
#endif
	if (EplRet != kEplSuccessful) {
		return EplRet;
	}

	ObdSize = sizeof(bVarOut1);
	uiVarEntries = 1;
#ifdef CN1
	EplRet = EplApiLinkObject(0x6200, &bVarOut1, &uiVarEntries, &ObdSize, 0x01);
#endif
#ifdef CN2
	EplRet = EplApiLinkObject(0x6203, &bVarOut2, &uiVarEntries, &ObdSize,
			0x01);
#endif
	if (EplRet != kEplSuccessful) {
		return EplRet;
	}

	// link a DOMAIN to object 0x6100, but do not exit, if it is missing
	ObdSize = sizeof(abDomain_l);
	uiVarEntries = 1;
	EplRet = EplApiLinkObject(0x6100, &abDomain_l, &uiVarEntries, &ObdSize,
			0x00);
	if (EplRet != kEplSuccessful) {
		return EplRet;
	}

	// reset old process variables
	bVarOut1Old_l = 0;
	return kEplSuccessful;

}

/*!
 *
 * \brief Initialize EPL stack and start NMT machine
 *
 * \param  pParams			user params (at moment not used)
 *
 * \return tEplKernel       error code,
 *                          kEplSuccessful = no error
 *
 */

int drvInitTask(void *pParams) {

	tEplKernel EplRet;
	EplApiInitParam.m_uiNodeId = NODEID;
	// calculate IP address
	EplApiInitParam.m_dwIpAddress = (SUBNET_MASK & IP_ADDR)
			| EplApiInitParam.m_uiNodeId;
	EplApiInitParam.m_fAsyncOnly = ASYNC_MODE;
	EplApiInitParam.m_uiSizeOfStruct = sizeof(EplApiInitParam);
	EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));
	EplApiInitParam.m_abMacAddress[5] = (BYTE) EplApiInitParam.m_uiNodeId;
	EplApiInitParam.m_dwFeatureFlags = DW_FEATURE_FLAGS;
	EplApiInitParam.m_dwCycleLen = DW_CYCLE_LEN;
	EplApiInitParam.m_uiIsochrTxMaxPayload = UI_ISOCHR_TX_MAX_PAYLOAD;
	EplApiInitParam.m_uiIsochrRxMaxPayload = UI_ISOCHR_RX_MAX_PAYLOAD;
	EplApiInitParam.m_dwPresMaxLatency = DW_PRES_MAX_LATENCY;
	EplApiInitParam.m_uiPreqActPayloadLimit = UI_PREQ_ACT_PAYLOAD_LIMIT;
	EplApiInitParam.m_uiPresActPayloadLimit = UI_PRES_ACT_PAYLOAD_LIMIT;
	EplApiInitParam.m_dwAsndMaxLatency = DW_ASND_MAX_LATENCY;
	EplApiInitParam.m_uiMultiplCycleCnt = UI_MULTIPL_CYCLE_CNT;
	EplApiInitParam.m_uiAsyncMtu = UI_ASYNC_MTU;
	EplApiInitParam.m_uiPrescaler = UI_PRESCALER;
	EplApiInitParam.m_dwLossOfFrameTolerance = DW_LOSS_OF_FRAME_TOLERANCE;
	EplApiInitParam.m_dwAsyncSlotTimeout = DW_ASYNC_SLOT_TIMEOUT;
	EplApiInitParam.m_dwWaitSocPreq = DW_WAIT_SOC_PREQ;
	EplApiInitParam.m_dwDeviceType = DW_DEVICE_TYPE;
	EplApiInitParam.m_dwVendorId = DW_VENDOR_ID;
	EplApiInitParam.m_dwProductCode = DW_PRODUCT_CODE;
	EplApiInitParam.m_dwRevisionNumber = DW_REVISION_NUMBER;
	EplApiInitParam.m_dwSerialNumber = DW_SERIAL_NUMBER;
	EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
	EplApiInitParam.m_dwDefaultGateway = DW_DEFAULT_GATEWAY;
	EPL_MEMCPY(EplApiInitParam.m_sHostname, HOSTNAME, sizeof(EplApiInitParam.m_sHostname));

	EplApiInitParam.m_pfnCbEvent = AppCbEvent;
	EplApiInitParam.m_pfnCbSync = AppCbSync;
	EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;

	STM3210C_LCD_Init();
	// Display a startup message.
	LCD_Clear(White);
	LCD_SetTextColor(Blue);

	// initialize POWERLINK stack
	EplRet = EplApiInitialize(&EplApiInitParam);

	if (EplRet != kEplSuccessful) {
		goto Exit;
	}

	if (variablesMapping() != kEplSuccessful) {
		goto Exit;
	}
	EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
	enableInterrupt();
	Exit: return EplRet;
	return 0;
}

/*!
 * \brief Initialize all software resources needed by the drv task.
 *
 * \param pParams not used
 * \return 0 if success, a task specific error code otherwise.
 */
int drvStartTask(void *pParams) {
	xTaskCreate(drvTaskFunc, "epl_drv", configMINIMAL_STACK_SIZE * 4, NULL, configDRVTASK_PRIORITY, NULL);
	return 0;
}

tEplKernel MyCallback(tEplTimerEventArg* pEventArg_p) {
}

tEplKernel MyCallback2(tEplTimerEventArg* pEventArg_p) {
	ledToggle2();
}


/*!
 * \brief controls if a event happened and calls the relavie handler
 *
 * \param  pParams			user params (at moment not used)
 *
 * \return     void
 *
 *
 */
void drvTaskFunc(void *pParams) {
	eventQueueElement event;
	tEdrvInitParam parameter;
	unsigned portBASE_TYPE queuesize;
	drvInitTask(NULL);
	int i = 0;

	unsigned short usReturn = 0UL; //current frame received length
	for (;;) {
		if (eventQueue != 0) {

			if (pdTRUE == xQueueReceive( eventQueue, &event, 100)) {

				//third parameter is dummy because INCLUDE_vTaskSuspend is 1

				if (event.interrupt_type & FRAME_RECEIVED_INTERRUPT) {
					frame_received_handler();

				} else if (event.interrupt_type & FRAME_TRANSMITTED_INTERRUPT) {
					frame_transmitted_handler();
				} else if (event.interrupt_type & TIMER1_TIMEOUT_INTERRUPT) {
					timer1_handler();
				} else if (event.interrupt_type & TIMER2_TIMEOUT_INTERRUPT) {
					timer2_handler();
				}
			}

		}


	}
}

void ledToggle() {


	vLedHandlerSetLED((unsigned portBASE_TYPE) 3, (signed portBASE_TYPE) a);
	if (a == 0)
		a = 1;
	else
		a = 0;

}

void ledToggleFromISR() {
	vLedHandlerSetLEDFromISR((unsigned portBASE_TYPE) 3, (signed portBASE_TYPE) a);
	if (a == 0)
		a = 1;
	else
		a = 0;
}

void ledToggle3FromISR() {
	vLedHandlerSetLEDFromISR((unsigned portBASE_TYPE) 1, (signed portBASE_TYPE) a);
	if (a == 0)
		a = 1;
	else
		a = 0;
}

void ledToggle2() {

	int i;
	vLedHandlerSetLED((unsigned portBASE_TYPE) 2, (signed portBASE_TYPE) b);
	if (b == 0)
		b = 1;
	else
		b = 0;

}

void ledToggle3() {
	int i;
	vLedHandlerSetLED((unsigned portBASE_TYPE) 1, (signed portBASE_TYPE) c);

	if (c == 0)
		c = 1;
	else
		c = 0;

}

void frame_received_handler() {
	unsigned short usReturn;
	//if we have a receive interrupt MACRx_Handler is called in order to process the
	//current frame and after allocate a new buffer for next incoming frames
	usReturn = MACRx_Handler();

}

void frame_transmitted_handler() {

	EdrvInstance_l.m_InitParam.m_pfnTxHandler(
			EdrvInstance_l.m_pLastTransmittedTxBuffer);
	/* The Tx buffer is no longer required. */
	vReturnBuffer((unsigned char *) xTxDescriptor.Buffer1Addr);

}

void timer1_handler() {
	tEplTimerEventArg args;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	args.m_TimerHdl = timers_pool[0].m_pTimerHdl;
	timers_pool[0].m_pfnCallback(&args);

	if (timers_pool[0].m_fContinuously == TRUE)
		initializeTimer(timers_pool[0]);
}

void timer2_handler() {
	tEplTimerEventArg args;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	args.m_TimerHdl = timers_pool[1].m_pTimerHdl;
	timers_pool[1].m_pfnCallback(&args);
	if (timers_pool[1].m_fContinuously == TRUE)
		initializeTimer(timers_pool[1]);
}

/*!
* \brief function to process commands.It reads command and
*				invokes eplApiFreeRTOS
*
* \param  command			 command to process
*
* \return  void
*
* State:
*/
void processCommand(Command *command) {
	rObjectParams* readObjectCommand;
	wObjectParams* writeObjectCommand;
	rLocalObjectParams* readLocalObjectCommand;
	wLocalObjectParams* writeLocalObjectCommand;
	tEplKernel ret;

	switch (command->cmd) {
	case EplApiReadObjectCMD:
		readObjectCommand = (rObjectParams *) command->params;
		ret = EplApiReadObject(readObjectCommand->pSdoComConHdl_m,
				readObjectCommand->uiNodeId_m, readObjectCommand->uiIndex_m,
				readObjectCommand->uiSubindex_m,
				readObjectCommand->pDstData_le_m, readObjectCommand->puiSize_m,
				readObjectCommand->SdoType_m, readObjectCommand->pUserArg_m);
		command->ret = ret;
		break;
	case EplApiWriteObjectCMD:
		writeObjectCommand = (wObjectParams *) command->params;
		ret = EplApiWriteObject(writeObjectCommand->pSdoComConHdl_m,
				writeObjectCommand->uiNodeId_m, writeObjectCommand->uiIndex_m,
				writeObjectCommand->uiSubindex_m,
				writeObjectCommand->pSrcData_le_m,
				writeObjectCommand->uiSize_m, writeObjectCommand->SdoType_m,
				writeObjectCommand->pUserArg_m);
		command->ret = ret;
		break;
	case EplApiReadLocalObjectCMD:
		readLocalObjectCommand = (rLocalObjectParams *) command->params;
		ret = EplApiReadLocalObject(readLocalObjectCommand->uiIndex_m,
				readLocalObjectCommand->uiSubindex_m,
				readLocalObjectCommand->pDstData_m,
				readLocalObjectCommand->puiSize_m);
		command->ret = ret;
		break;
	case EplApiWriteLocalObjectCMD:
		writeLocalObjectCommand = (wLocalObjectParams *) command->params;
		ret = EplApiWriteLocalObject(writeLocalObjectCommand->uiIndex_m,
				writeLocalObjectCommand->uiSubindex_m,
				writeLocalObjectCommand->pSrcData_m,
				writeLocalObjectCommand->uiSize_m);
		command->ret = ret;
		break;
	}
}

//void TaskLocalObjectWriter() {
//	static BYTE value = 1;
//	int size = sizeof(value);
//	tEplKernel ret;
//	for (;;) {
//		vTaskDelay(2500 / portTICK_RATE_MS);
//		ret = EplApiWriteLocalObjectFreeRTOS(0x6003, 0x01, &value, size);
//		LCD_DisplayStringLine(Line5,(unsigned char *) "bVarIn2 value");
//		LCD_DisplayStringLine(Line6, value);
//		value++;
//	}
//}

void TaskLocalObjectReader() {
	int value = 12;
	int size = sizeof(value);
	tEplKernel ret;
	char valore[4];
	for (;;) {
		vTaskDelay(2500 / portTICK_RATE_MS);
		ledToggle();
		#ifdef CN1
		ret = EplApiReadLocalObjectFreeRTOS(0x6000, 0x01, &value, &size);
		itochar(value, valore, 10);
		LCD_DisplayStringLine(Line3,(unsigned char *) "U.T. bVarIn1 value");
		LCD_DisplayStringLine(Line4, &valore);
		#endif
		#ifdef CN2
		ret = EplApiReadLocalObjectFreeRTOS(0x6203, 0x01, &value, &size);
		itochar(value, valore, 10);
		LCD_DisplayStringLine(Line3,(unsigned char *) "U.T. bVarOut2 value");
		LCD_DisplayStringLine(Line4, &valore);
		#endif

		ledToggle();
	}
}

