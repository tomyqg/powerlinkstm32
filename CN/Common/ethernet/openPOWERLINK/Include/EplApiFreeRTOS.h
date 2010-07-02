/*
 * EplApiFreeRTOS.h
 *
 *  Created on: 27/apr/2010
 *      Author: alessio
 */

#ifndef EPLAPIFREERTOS_H_
#define EPLAPIFREERTOS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Epl.h"

extern xQueueHandle ipcCommandsToProcessQueue;
extern xQueueHandle ipcCommandsProcessedQueue;

#define WAIT_DELAY_ON_PROCESSEDCOMMANDS_QUEUE					( 100 / portTICK_RATE_MS )
typedef enum
{
	EplApiReadObjectCMD = 0x01,
	EplApiWriteObjectCMD = 0x02,
	EplApiReadLocalObjectCMD = 0x03,
	EplApiWriteLocalObjectCMD = 0x04,
}idCommand;

typedef struct{
    tEplSdoComConHdl* pSdoComConHdl_m;
    unsigned int      uiNodeId_m;
    unsigned int      uiIndex_m;
    unsigned int      uiSubindex_m;
    void*             pDstData_le_m;
    unsigned int*     puiSize_m;
    tEplSdoType       SdoType_m;
    void*             pUserArg_m;
}rObjectParams;

typedef struct{
	tEplSdoComConHdl* pSdoComConHdl_m;
	unsigned int      uiNodeId_m;
	unsigned int      uiIndex_m;
	unsigned int      uiSubindex_m;
	void*             pSrcData_le_m;
	unsigned int      uiSize_m;
	tEplSdoType       SdoType_m;
	void*             pUserArg_m;
}wObjectParams;

typedef struct{
	 unsigned int      uiIndex_m;
	 unsigned int      uiSubindex_m;
	 void*             pDstData_m;
	 unsigned int*     puiSize_m;
}rLocalObjectParams;

typedef struct{
	unsigned int      uiIndex_m;
	unsigned int      uiSubindex_m;
	void*             pSrcData_m;
	unsigned int      uiSize_m;
}wLocalObjectParams;


typedef struct{
	xTaskHandle handle;
	idCommand cmd;
	tEplKernel ret;
	void* params;

}Command;

tEplKernel PUBLIC EplApiReadLocalObjectFreeRTOS(
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pDstData_p,
            unsigned int*     puiSize_p
);

tEplKernel PUBLIC EplApiWriteLocalObjectFreeRTOS(
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pSrcData_p,
            unsigned int      uiSize_p
            );

tEplKernel PUBLIC EplApiReadObjectFreeRTOS(
            tEplSdoComConHdl* pSdoComConHdl_p,
            unsigned int      uiNodeId_p,
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pDstData_le_p,
            unsigned int*     puiSize_p,
            tEplSdoType       SdoType_p,
            void*             pUserArg_p
            );

tEplKernel PUBLIC EplApiWriteObjectFreeRTOS(
            tEplSdoComConHdl* pSdoComConHdl_p,
            unsigned int      uiNodeId_p,
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pSrcData_le_p,
            unsigned int      uiSize_p,
            tEplSdoType       SdoType_p,
            void*             pUserArg_p
            );

#endif /* EPLAPIFREERTOS_H_ */
