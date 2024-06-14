/*
* main.c - contains program main
*
* Copyright (c) 2012 emtas GmbH
*-------------------------------------------------------------------
* $Id: main.c 4467 2013-09-03 15:27:46Z  $
*
*
*-------------------------------------------------------------------
*
*
*/

/********************************************************************/
/**
* \file
* \brief main routine
*
*/

/* header of standard C - libraries
---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* header of project specific types
---------------------------------------------------------------------------*/
#include <gen_define.h>

#include <co_canopen.h>

#include <codrv_cpu.h>

/* constant definitions
---------------------------------------------------------------------------*/

/* local defined data types
---------------------------------------------------------------------------*/

/* list of external used functions, if not in headers
--------------------------------------------------------------------------*/
void codrvHardwareInit(void);

/* list of global defined functions
---------------------------------------------------------------------------*/
int init_printf(void);
void exitProgram(int error);

/* list of local defined functions
---------------------------------------------------------------------------*/
static RET_T nmtInd(BOOL_T	execute, CO_NMT_STATE_T newState);
static void hbState(UNSIGNED8	nodeId, CO_ERRCTRL_T state,
		CO_NMT_STATE_T	nmtState);
static RET_T sdoServerReadInd(BOOL_T execute, UNSIGNED8	sdoNr, UNSIGNED16 index,
		UNSIGNED8	subIndex);
static RET_T sdoServerCheckWriteInd(BOOL_T execute, UNSIGNED8 node,
		UNSIGNED16 index, UNSIGNED8	subIndex, const UNSIGNED8 *pData);
static RET_T sdoServerWriteInd(BOOL_T execute, UNSIGNED8 sdoNr,
		UNSIGNED16 index, UNSIGNED8	subIndex);
static void pdoInd(UNSIGNED16);
static void pdoRecEvent(UNSIGNED16);
static void canInd(CO_CAN_STATE_T);
static void commInd(CO_COMM_STATE_EVENT_T);
static void ledGreenInd(BOOL_T);
static void ledRedInd(BOOL_T);


/* external variables
---------------------------------------------------------------------------*/

/* global variables
---------------------------------------------------------------------------*/
UNSIGNED16	test;

/* local defined variables
---------------------------------------------------------------------------*/


/***************************************************************************/
/**
* \brief main entry
*
* \param
*	nothing
* \results
*	nothing
*/
int main(
	void
  )
{
UNSIGNED8	emcyData[5] = { 1, 2, 3, 4, 5 };

	codrvHardwareInit();
	/* init_printf(); */


	if (codrvCanInit(250) != RET_OK)  {
		exitProgram(1);
	}
    if (coCanOpenStackInit(NULL) != RET_OK)  {
		printf("error init library\n");
		exitProgram(1);
	}

	if (codrvTimerSetup(CO_TIMER_INTERVAL) != RET_OK)  {
		exitProgram(2);
	}

	/* register event functions */
	if (coEventRegister_NMT(nmtInd) != RET_OK)  {
		exitProgram(3);
	}
	if (coEventRegister_ERRCTRL(hbState) != RET_OK)  {
		exitProgram(4);
	}
	if (coEventRegister_SDO_SERVER_READ(sdoServerReadInd) != RET_OK)  {
		exitProgram(5);
	}
	if (coEventRegister_SDO_SERVER_CHECK_WRITE(sdoServerCheckWriteInd) != RET_OK)  {
		exitProgram(6);
	}
	if (coEventRegister_SDO_SERVER_WRITE(sdoServerWriteInd) != RET_OK)  {
		exitProgram(7);
	}
	if (coEventRegister_PDO(pdoInd) != RET_OK)  {
		exitProgram(8);
	}
	if (coEventRegister_PDO_REC_EVENT(pdoRecEvent) != RET_OK)  {
		exitProgram(9);
	}
	if (coEventRegister_LED_GREEN(ledGreenInd) != RET_OK)  {
		exitProgram(10);
	}
	if (coEventRegister_LED_RED(ledRedInd) != RET_OK)  {
		exitProgram(11);
	}
	if (coEventRegister_CAN_STATE(canInd) != RET_OK)  {
		exitProgram(12);
	}
	if (coEventRegister_COMM_EVENT(commInd) != RET_OK)  {
		exitProgram(13);
	}

	if (codrvCanEnable() != RET_OK)  {
		exitProgram(14);
	}

	/* write emcy */
	if (coEmcyWriteReq(0x1234, &emcyData[0]) != RET_OK)  {
		exitProgram(15);
	}

	/* Enable global Interrupts and higher priority real-time debug events: */
	EnableInterrupts();
	EINT;   /* Enable Global interrupt INTM */
	ERTM;   /* Enable Global realtime interrupt DBGM */

	while (1)  {
    	coCommTask();

	}
}


/*********************************************************************/
static RET_T nmtInd(
		BOOL_T	execute,
		CO_NMT_STATE_T	newState
	)
{
	printf("nmtInd: New Nmt state %d - execute %d\n", newState, execute);

	return(RET_OK);
}


/*********************************************************************/
static void pdoInd(
		UNSIGNED16	pdoNr
	)
{
	printf("pdoInd: pdo %d received\n", pdoNr);
}


/*********************************************************************/
static void pdoRecEvent(
		UNSIGNED16	pdoNr
	)
{
	printf("pdoRecEvent: pdo %d time out\n", pdoNr);
}


/*********************************************************************/
static void hbState(
		UNSIGNED8	nodeId,
		CO_ERRCTRL_T state,
		CO_NMT_STATE_T	nmtState
	)
{
	printf("hbInd: HB Event %d node %d nmtState: %d\n", state, nodeId, nmtState);

    return;
}


/*********************************************************************/
static RET_T sdoServerReadInd(
		BOOL_T		execute,
		UNSIGNED8	sdoNr,
		UNSIGNED16	index,
		UNSIGNED8	subIndex
	)
{
	printf("sdo server read ind: exec: %d, sdoNr %d, index %x:%d\n",
		execute, sdoNr, index, subIndex);

   // return(RET_INVALID_PARAMETER);
	return(RET_OK);
}


/*********************************************************************/
static RET_T sdoServerCheckWriteInd(
		BOOL_T		execute,
		UNSIGNED8	sdoNr,
		UNSIGNED16	index,
		UNSIGNED8	subIndex,
		const UNSIGNED8	*pData
	)
{
	printf("sdo server check write ind: exec: %d, sdoNr %d, index %x:%d\n",
		execute, sdoNr, index, subIndex);

   // return(RET_INVALID_PARAMETER);
	return(RET_OK);
}


/*********************************************************************/
static RET_T sdoServerWriteInd(
		BOOL_T		execute,
		UNSIGNED8	sdoNr,
		UNSIGNED16	index,
		UNSIGNED8	subIndex
	)
{
	printf("sdo server write ind: exec: %d, sdoNr %d, index %x:%d\n",
		execute, sdoNr, index, subIndex);

   // return(RET_INVALID_PARAMETER);
	return(RET_OK);
}


/*********************************************************************/
static void canInd(
	CO_CAN_STATE_T	canState
	)
{
	switch (canState)  {
		case CO_CAN_STATE_BUS_OFF:
			printf("CAN: Bus Off\n");
			break;
		case CO_CAN_STATE_BUS_ON:
			printf("CAN: Bus On\n");
			break;
		case CO_CAN_STATE_PASSIVE:
			printf("CAN: Passive\n");
			break;
		default:
			printf("canInd %d\n", (int)canState);
			break;
	}
}


/*********************************************************************/
static void commInd(
		CO_COMM_STATE_EVENT_T	commEvent
	)
{
	switch (commEvent)  {
		case CO_COMM_STATE_EVENT_CAN_OVERRUN:
			printf("COMM-Event CAN Overrun\n");
			break;
		case CO_COMM_STATE_EVENT_REC_QUEUE_FULL:
			printf("COMM-Event Rec Queue Full\n");
			break;
		case CO_COMM_STATE_EVENT_REC_QUEUE_OVERFLOW:
			printf("COMM-Event Rec Queue Overflow\n");
			break;
		case CO_COMM_STATE_EVENT_REC_QUEUE_EMPTY:
			printf("COMM-Event Rec Queue empty\n");
			break;
		case CO_COMM_STATE_EVENT_TR_QUEUE_FULL:
			printf("COMM-Event Tr Queue Full\n");
			break;
		case CO_COMM_STATE_EVENT_TR_QUEUE_OVERFLOW:
			printf("COMM-Event Tr Queue Overflow\n");
			break;
		case CO_COMM_STATE_EVENT_TR_QUEUE_EMPTY:
			printf("COMM-Event Tr Queue Empty\n");
			break;
		default:
			printf("commInd %d\n", (int)commEvent);
			break;
	}

}

/*********************************************************************/
static void ledGreenInd(
		BOOL_T	on
	)
{
	printf("GREEN: %d\n", on);
}


/*********************************************************************/
static void ledRedInd(
		BOOL_T	on
	)
{
	printf("RED: %d\n", on);
}

/*********************************************************************/
void exitProgram(int error)
{

	printf("Error %d\n", error);
	while(1);
}


