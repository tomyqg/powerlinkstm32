#include "EplApiFreeRTOS.h"

#include "task.h"
tEplKernel PUBLIC EplApiReadLocalObjectFreeRTOS(
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pDstData_p,
            unsigned int*     puiSize_p
            ){
	rLocalObjectParams params;
	Command command;

	tEplKernel          Ret = kEplSuccessful;
	xTaskHandle currentHandle;
	params.uiIndex_m=uiIndex_p;
	params.uiSubindex_m=uiSubindex_p;
	params.pDstData_m=pDstData_p;
	params.puiSize_m=puiSize_p;
	currentHandle=xTaskGetCurrentTaskHandle();
	command.handle=currentHandle;
	command.cmd=EplApiReadLocalObjectCMD;
	command.params=&params;
	xQueueSend(ipcCommandsToProcessQueue,&command,1);
	vTaskSuspend((xTaskHandle)command.handle);
	xQueueReceive(ipcCommandsProcessedQueue,&command,1);
	Ret=command.ret;
    EPL_MEMCPY(pDstData_p,((rLocalObjectParams *)command.params)->pDstData_m,sizeof(((rLocalObjectParams *)command.params)->pDstData_m));
    EPL_MEMCPY(puiSize_p,((rLocalObjectParams *)command.params)->puiSize_m,sizeof(((rLocalObjectParams *)command.params)->puiSize_m));
	return Ret;
}

tEplKernel PUBLIC EplApiWriteLocalObjectFreeRTOS(
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pSrcData_p,
            unsigned int      uiSize_p
            ){

		wLocalObjectParams params;
		Command command;
		tEplKernel          Ret = kEplSuccessful;
		xTaskHandle currentHandle;
		params.uiIndex_m=uiIndex_p;
		params.uiSubindex_m=uiSubindex_p;
		params.pSrcData_m=pSrcData_p;
		params.uiSize_m=uiSize_p;
		currentHandle=xTaskGetCurrentTaskHandle();
		command.handle=currentHandle;

		command.cmd=EplApiWriteLocalObjectCMD;
		command.params=&params;
		xQueueSend(ipcCommandsToProcessQueue,&command,1);
		vTaskSuspend(command.handle);
		Ret=command.ret;
		xQueueReceive(ipcCommandsProcessedQueue,&command,1);
	    EPL_MEMCPY(pSrcData_p,((wLocalObjectParams *)command.params)->pSrcData_m,sizeof(((wLocalObjectParams *)command.params)->pSrcData_m));
		return Ret;
}

tEplKernel PUBLIC EplApiReadObjectFreeRTOS(
            tEplSdoComConHdl* pSdoComConHdl_p,
            unsigned int      uiNodeId_p,
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pDstData_le_p,
            unsigned int*     puiSize_p,
            tEplSdoType       SdoType_p,
            void*             pUserArg_p
            ){
	Command command;
	rObjectParams params;
	tEplKernel          Ret = kEplSuccessful;
	xTaskHandle currentHandle;
	params.pSdoComConHdl_m=pSdoComConHdl_p;
	params.uiNodeId_m=uiNodeId_p;
	params.uiIndex_m=uiIndex_p;
	params.uiSubindex_m=uiSubindex_p;
	params.pDstData_le_m=pDstData_le_p;
	params.puiSize_m=puiSize_p;
	params.SdoType_m=SdoType_p;
	params.pUserArg_m=pUserArg_p;
	currentHandle=xTaskGetCurrentTaskHandle();
	command.handle=currentHandle;
	command.cmd=EplApiReadObjectCMD;
	command.params=&params;
	xQueueSend(ipcCommandsToProcessQueue,&command,1);
	vTaskSuspend(command.handle);
	xQueueReceive(ipcCommandsProcessedQueue,&command,1);
	EPL_MEMCPY(pDstData_le_p,((rObjectParams *)command.params)->pDstData_le_m,sizeof(((rObjectParams *)command.params)->pDstData_le_m));
	EPL_MEMCPY(puiSize_p,((rObjectParams *)command.params)->puiSize_m,sizeof(((rObjectParams *)command.params)->puiSize_m));
    EPL_MEMCPY(pUserArg_p,((rObjectParams *)command.params)->pUserArg_m,sizeof(((rObjectParams *)command.params)->pUserArg_m));

    return Ret;
}

tEplKernel PUBLIC EplApiWriteObjectFreeRTOS(
            tEplSdoComConHdl* pSdoComConHdl_p,
            unsigned int      uiNodeId_p,
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pSrcData_le_p,
            unsigned int      uiSize_p,
            tEplSdoType       SdoType_p,
            void*             pUserArg_p
            ){
	Command command;
	wObjectParams params;
	tEplKernel          Ret = kEplSuccessful;
	xTaskHandle currentHandle;
	params.pSdoComConHdl_m=pSdoComConHdl_p;
	params.uiNodeId_m=uiNodeId_p;
	params.uiIndex_m=uiIndex_p;
	params.uiSubindex_m=uiSubindex_p;
	params.pSrcData_le_m=pSrcData_le_p;
	params.uiSize_m=uiSize_p;
	params.SdoType_m=SdoType_p;
	params.pUserArg_m=pUserArg_p;
	currentHandle=xTaskGetCurrentTaskHandle();
	command.handle=currentHandle;
	command.cmd=EplApiWriteObjectCMD;
	command.params=&params;
	xQueueSend(ipcCommandsToProcessQueue,&command,1);
	vTaskSuspend(command.handle);
	xQueueReceive(ipcCommandsProcessedQueue,&command,1);
	EPL_MEMCPY(pSrcData_le_p,((wObjectParams *)command.params)->pSrcData_le_m,sizeof(((wObjectParams *)command.params)->pSrcData_le_m));
	EPL_MEMCPY(pUserArg_p,((wObjectParams *)command.params)->pUserArg_m,sizeof(((wObjectParams *)command.params)->pUserArg_m));

	return Ret;
}
