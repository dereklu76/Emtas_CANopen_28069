/*
* codrv_can_hecc.c - TI CAN driver
*
* Copyright (c) 2013-2016 emtas GmbH
*-------------------------------------------------------------------
* $Id: codrv_can_hecc.c 11615 2015-12-04 16:00:22Z ro $
*
*
*-------------------------------------------------------------------
*
*
*/

/********************************************************************/
/**
* \brief HECC driver
*
* \file codrv_can_hecc.c - TI HECC driver for basic CAN
* \author emtas GmbH
*
* The official small API for Filter usage is contained. But it is not 
* required for a basic CAN driver. 
*
*/

/**
* \define CO_DRV_FILTER
* This setting activate the filter functionality. But note,
* you need a lot of filter to use it. Typical the CAN controller
* are called FullCAN controller.
*
* You have to set this define in gen_define.h!
*/

/**
* \define CO_DRV_GROUP_FILTER
* The group filter mechanism is a additional feature for the general
* filter mechanism. The most filter can set an acceptance mask. A 
* often used mask enable a group for all NodeIds of a specific
* command group, e.g. heartbeat consumer. In this case only one
* filter is required for 128 message identifiers.
*
* You have to set this define in gen_define.h!
*/

/**
* \define POLLING
* Often used driver internal define, e.g. during the development.
* In case this define is set, the driver don't use interrupts.
*
* You have to use it driver internal, only.
*/

/**
* \define CODRV_DEBUG
* Often used driver internal define to activate the printf()
* output for debugging. A completely correct functionality is not
* ensured, if this define is set. Please deactivate it!
*/ 

/**
* \define DEBUG_SEND_TESTMESSAGE
* Often used driver internal #define to send a transmit CAN frame
* during the initialization.
* For measurement purpose the message ID is 0x555 and the data byte
* 0x01..0x8.
* If no other CAN node is connected,
* the CAN controller will send this frame endless.
* This can be used to measure the bit time using an oscilloscope.
* Please deactivate it in production code!
*/


/* header of standard C - libraries
---------------------------------------------------------------------------*/
#include <stddef.h>
#include <stdio.h>

/* header of project specific types
---------------------------------------------------------------------------*/
#include <gen_define.h>

#include <co_datatype.h>
#include <co_drv.h>
#include "codrv_error.h"
#include <co_commtask.h>

#include "codrv_cpu.h"
#include "codrv_can_hecc.h"

/* constant definitions
---------------------------------------------------------------------------*/
//#define POLLING 1

/* no workaround required */
#define CanDoubleWrite(pDst, pSrc) (*(pDst) = *(pSrc))
#define CanDoubleRead(pDst, pSrc)  (*(pDst) = *(pSrc))


/* local defined data types
---------------------------------------------------------------------------*/

/* list of external used functions, if not in headers
---------------------------------------------------------------------------*/

/* list of global defined functions
---------------------------------------------------------------------------*/

/* list of local defined functions
---------------------------------------------------------------------------*/
static RET_T codrvCanTransmit(CO_CONST CO_CAN_MSG_T * pBuf);
static CO_CONST CODRV_BTR_T * codrvCanGetBtrSettings(UNSIGNED16 bitRate);

/* external variables
---------------------------------------------------------------------------*/

/* global variables
---------------------------------------------------------------------------*/

/* local defined variables
---------------------------------------------------------------------------*/
static BOOL_T canEnabled = CO_FALSE; /**< CAN buson */
static BOOL_T transmissionIsActive = CO_FALSE; /**< TX transmission active */

#ifdef CO_DRV_FILTER
static UNSIGNED16 nextFilterEntry = 1u;
static const UNSIGNED16 maxFilterEntry = 31u;
#endif

/** currently TX message */
static CO_CAN_MSG_T *pTxBuf = NULL;

/** CAN Controller address */
//static volatile UNSIGNED32 * CO_CONST pCan = (void*)0x40006400ul;
static volatile struct MBOX * const pECanaMbox = &ECanaMboxes.MBOX0;
static volatile union CANLAM_REG * const pECanaLam = &ECanaLAMRegs.LAM0;
static volatile struct ECAN_REGS * const pECanaRegs = &ECanaRegs;

#ifdef CODRV_BIT_TABLE_EXTERN
/* use an external bitrate table */
extern CO_CONST CODRV_BTR_T codrvCanBittimingTable[];
#else /* CODRV_BIT_TABLE_EXTERN */

/** can bittiming table */
static CO_CONST CODRV_BTR_T codrvCanBittimingTable[] = {
		/* e.g. 75MHz table (0.5*sysclk) , prescaler 8bit (max 256) */
		{   20u, 250u, 0, 12u, 2u }, /* 86.7% */
		{   50u, 100u, 0, 12u, 2u }, /* 86.7% */
		{  100u,  50u, 0, 12u, 2u }, /* 86.7% */
		{  125u,  40u, 0, 12u, 2u }, /* 86.7% */
		{  250u,  20u, 0, 12u, 2u }, /* 86.7% */
		{  500u,  10u, 0, 12u, 2u}, /* 86.7% */
		{ 1000u,   5u, 0, 12u, 2u}, /* 86.7% */
		{0,0,0,0,0} /* last */
	};
#endif /* CODRV_BIT_TABLE_EXTERN */

/*---------------------------------------------------------------------------*/
//#define DEBUG_SEND_TESTMESSAGE
#ifdef DEBUG_SEND_TESTMESSAGE
static void codrvSendTestMessage(void)
{
	struct ECAN_REGS ECanaShadow;
	struct MBOX MBOXx;





	/* CAN ID 0x555 */
	MBOXx.MSGID.all = 0;
	MBOXx.MSGID.bit.STDMSGID = 0x555;
	CanDoubleWrite(&ECanaMboxes.MBOX0.MSGID.all, &MBOXx.MSGID.all);

	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
	ECanaShadow.CANMD.bit.MD0 = 0; // TX
	ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

	//enable
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0 = 1; // enable
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	/* data length 8 */
	MBOXx.MSGCTRL.all = 0;
	MBOXx.MSGCTRL.bit.DLC = 8;
	CanDoubleWrite(&ECanaMboxes.MBOX0.MSGCTRL.all, &MBOXx.MSGCTRL.all);

	/* data 0x1..0x8 */
	MBOXx.MDL.all = 0x04030201;
	CanDoubleWrite(&ECanaMboxes.MBOX0.MDL.all, &MBOXx.MDL.all);
	MBOXx.MDH.all = 0x08070605;
	CanDoubleWrite(&ECanaMboxes.MBOX0.MDH.all, &MBOXx.MDH.all);

	//receive
#ifdef RX_TEST
	union CANLAM_REG LAMx;

	MBOXx.MSGID.all = 0;
	MBOXx.MSGID.bit.STDMSGID = 0x001;
	MBOXx.MSGID.bit.AME = 1;
	CanDoubleWrite(&ECanaMboxes.MBOX1.MSGID.all, &MBOXx.MSGID.all);

	LAMx.all = 0x1FFFFFFF; // accept all
	CanDoubleWrite(&ECanaLAMRegs.LAM1.all, &LAMx.all);

	ECanaRegs.CANMD.bit.MD1 = 1; // RX

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME1 = 1; // enable
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
#endif


	ECanaShadow.CANTRS.all = 0;//ECanaRegs.CANTRS.all;
	ECanaShadow.CANTRS.bit.TRS0 = 1; // Transmit
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;


	do {
		ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
	} while (ECanaShadow.CANTA.bit.TA0 == 0);
	ECanaRegs.CANTA.all = 1; //reset?


//	printf("RMP %0lx\n", ECanaRegs.CANRMP.all );

}
#endif /* DEBUG_SEND_TESTMESSAGE */
/*---------------------------------------------------------------------------*/



/***************************************************************************/
/**
* \brief codrvCanInit - init CAN controller
*
* This function initializes the CAN controller and configures the bitrate.
* At the end of the function, the CAN controller should be in state disabled.
*
* \return RET_T
* \retval RET_OK
*	initialization was OK
*
*/
RET_T codrvCanInit(
		UNSIGNED16	bitRate		/**< CAN bitrate */
	)
{
RET_T	retVal;
UNSIGNED16 i;

union CANTIOC_REG CANTIOC;
union CANRIOC_REG CANRIOC;
union CANMC_REG   CANMC;
union CANES_REG   CANES;
union CANMIM_REG  CANMIM;

union CANMSGCTRL_REG   MSGCTRL;

	/* init req. variables */
	canEnabled = CO_FALSE;
	transmissionIsActive = CO_FALSE;

#ifdef CO_DRV_FILTER
	/*
	 * 0 .. TX
	 */
	nextFilterEntry = 1u;
#endif

	pTxBuf = NULL;

	/* error states */
	codrvCanErrorInit();

	/* initialize CAN controller, setup timing, pin description, CAN mode ...*/
	EALLOW;

    CANTIOC.all = pECanaRegs->CANTIOC.all;
    CANTIOC.bit.TXFUNC = 1;
    pECanaRegs->CANTIOC.all = CANTIOC.all;


    CANRIOC.all = pECanaRegs->CANRIOC.all;
    CANRIOC.bit.RXFUNC = 1;
    pECanaRegs->CANRIOC.all = CANRIOC.all;


    /* Reset CAN module state machine */
	CANMC.all = pECanaRegs->CANMC.all;
	CANMC.bit.SRES = 1; // software reset
	pECanaRegs->CANMC.all = CANMC.all;

	do {
		CANMC.all = pECanaRegs->CANMC.all;
	} while (CANMC.bit.SRES != 0);

	/* configuration mode (but should be currently in this mode */
	CANMC.all = pECanaRegs->CANMC.all;
	CANMC.bit.CCR = 1;
	pECanaRegs->CANMC.all = CANMC.all;


	do {
		CANES.all = pECanaRegs->CANES.all;
	} while (CANES.bit.CCE == 0);

	CANMC.all = pECanaRegs->CANMC.all;
	CANMC.bit.SCB = 1; //HECC
	CANMC.bit.DBO = 1; //little endian order (header use value 0 order)
	CANMC.bit.ABO = 0; //Auto Buson deactivated for better detection
	CANMC.bit.PDR = 0; // Power down off
	CANMC.bit.SUSP = 1; // CAN runs free during debug
	pECanaRegs->CANMC.all = CANMC.all;

	/* Init mailboxes */
	MSGCTRL.all = 0x00000000ul;

	for (i = 0; i < 32; i++) {
		CanDoubleWrite(&pECanaMbox[i].MSGCTRL.all, &MSGCTRL.all);
	}

    /* Clear all TAn bits */
    pECanaRegs->CANTA.all	= 0xFFFFFFFFul;

    /* Clear all RMPn bits */
   	pECanaRegs->CANRMP.all = 0xFFFFFFFFul;

    /* Clear all interrupt flag bits */
    pECanaRegs->CANGIF0.all = 0xFFFFFFFFul;
    pECanaRegs->CANGIF1.all = 0xFFFFFFFFul;

    /* disable all mailsboxes */
    pECanaRegs->CANME.all = 0ul;		/* disable required before writing the MSGIDs */

	EDIS;

	/* set bitrate */
	retVal = codrvCanSetBitRate(bitRate);

	EALLOW;

#ifdef CO_DRV_FILTER
#  ifdef CO_DRV_GROUP_FILTER
	// init RX Mailbox - receive HB and EMCY (81..FF, 701..77F)
#    ifdef CO_HB_CONSUMER_CNT

	MSGID.all = 0;
	MSGID.bit.STDMSGID = 0x700;
	MSGID.bit.AME = 1;
	CanDoubleWrite(&pECanaMbox[nextFilterEntry].MSGID.all, &MSGID.all);
	LAMx.all = 0x7Ful << 18; // accept some
	LAMx.bit.LAMI = 0;
	CanDoubleWrite(&pECanaLam[nextFilterEntry].all, &LAMx.all);


	CANMD.all = pECanaRegs->CANMD.all;
	CANMD.all |= 1ul << nextFilterEntry; // RX
	pECanaRegs->CANMD.all = CANMD.all;


	CANME.all = pECanaRegs->CANME.all;
	CANME.all |= 1ul << nextFilterEntry; // enable
	pECanaRegs->CANME.all = CANME.all;


	CANMIM.all = pECanaRegs->CANMIM.all;
	CANMIM.all |= 1ul << nextFilterEntry; // IRQ
	pECanaRegs->CANMIM.all = CANMIM.all;

	nextFilterEntry ++;
#    endif
#    ifdef CO_EMCY_CONSUMER_CNT

	MSGID.all = 0ul;
	MSGID.bit.STDMSGID = 0x080u;
	MSGID.bit.AME = 1;
	CanDoubleWrite(&pECanaMbox[nextFilterEntry].MSGID.all, &MSGID.all);
	LAMx.all = 0x7Ful << 18; // accept some
	LAMx.bit.LAMI = 0;
	CanDoubleWrite(&pECanaLam[nextFilterEntry].all, &LAMx.all);

	CANMD.all = pECanaRegs->CANMD.all;
	CANMD.all |= 1ul << nextFilterEntry; // RX
	pECanaRegs->CANMD.all = CANMD.all;


	CANME.all = pECanaRegs->CANME.all;
	CANME.all |= 1ul << nextFilterEntry; // enable
	pECanaRegs->CANME.all = CANME.all;

	CANMIM.all = pECanaRegs->CANMIM.all;
	CANMIM.all |= 1ul << nextFilterEntry; // IRQ
	pECanaRegs->CANMIM.all = CANMIM.all;

	nextFilterEntry ++;
#    endif
#  endif
#else
	// init RX Mailbox - receive all
	MSGID.all = 0ul;
	MSGID.bit.STDMSGID = 0x000u;
	MSGID.bit.AME = 1;
	CanDoubleWrite(&pECanaMbox[1].MSGID.all, &MSGID.all);
	LAMx.all = 0x1FFFFFFFul; // accept all
	LAMx.bit.LAMI = 1;
	CanDoubleWrite(&pECanaLam[1].all, &LAMx.all);

	CANMD.all = pECanaRegs->CANMD.all;
	CANMD.bit.MD1 = 1; // RX
	pECanaRegs->CANMD.all = CANMD.all;


	CANME.all = pECanaRegs->CANME.all;
	CANME.bit.ME1 = 1; // enable
	pECanaRegs->CANME.all = CANME.all;
#endif

	pECanaRegs->CANMIL.all = 0ul; // irq line 0 in general

	CANMIM.all = pECanaRegs->CANMIM.all;
	CANMIM.bit.MIM0 = 1;           		// TX
#ifdef CO_DRV_FILTER
#else
	CANMIM.bit.MIM1 = 1;           		// RX
#endif
	pECanaRegs->CANMIM.all = CANMIM.all;

	EDIS;

#ifdef DEBUG_SEND_TESTMESSAGE
	codrvCanEnable();
	codrvSendTestMessage();
#endif /* DEBUG_SEND_TESTMESSAGE */


	return(retVal);
}

/***********************************************************************/
/**
* codrvCanGetBtrSettings - get pointer to the BTR value structure
*
* \internal
*
* \returns
*	pointer to an BTR table entry
*/

static CO_CONST CODRV_BTR_T * codrvCanGetBtrSettings(
		UNSIGNED16 bitRate	/**< required bitrate */
	)
{
CO_CONST CODRV_BTR_T * pBtrEntry = NULL;
UNSIGNED8 i;

	i = 0u;
	while (codrvCanBittimingTable[i].bitRate != 0u) {
		if (codrvCanBittimingTable[i].bitRate == bitRate) {
			pBtrEntry = &codrvCanBittimingTable[i];
			break;
		}
		i++;
	}

	return pBtrEntry;

}


/***********************************************************************/
/**
* \brief codrvCanSetBitRate - set CAN Bitrate
*
* This function sets the CAN Bitrate to the given value.
* Changing the Bitrate is only allowed, if the CAN controller is in reset.
* The state at the start of the function is unknown, 
* so the CAN controller should be switch to state reset.
*
* At the end of the function the CAN controller should be stay in state reset.
*
* \return RET_T
* \retval RET_OK
*	setting of Bitrate was ok
*
*/
RET_T codrvCanSetBitRate(
		UNSIGNED16	bitRate		/**< CAN Bitrate in kbit/s */
	)
{
CO_CONST CODRV_BTR_T * pBtrEntry;

UNSIGNED32 pre;
UNSIGNED32 seg1;
UNSIGNED32 seg2;

union CANMC_REG   CANMC;
union CANES_REG   CANES;
union CANBTC_REG  CANBTC;

	/* stop CAN controller */
	(void)codrvCanDisable();

	/* get bittiming values */
	pBtrEntry = codrvCanGetBtrSettings(bitRate);

	if (pBtrEntry == NULL) {
		/* if Bitrate not supported */
		return(RET_DRV_WRONG_BITRATE); 
	}

	/* many CAN controller use 10bit and our table support this */
	if (pBtrEntry->pre > 256u) {
		/* if Bitrate not supported */
		return(RET_DRV_WRONG_BITRATE);
	}

	/* Configure bit timing parameters */
	EALLOW;
	CANMC.all = pECanaRegs->CANMC.all;
	CANMC.bit.CCR = 1 ;            		// Set CCR = 1
	pECanaRegs->CANMC.all = CANMC.all;

	pre = pBtrEntry->pre; 
	seg1 = pBtrEntry->seg1 + pBtrEntry->prop; 
	seg2 = pBtrEntry->seg2; 

	/* Wait until the CPU has been granted permission to change the configuration registers */
	do {
		CANES.all = pECanaRegs->CANES.all;
	} while(CANES.bit.CCE == 0 );  		// Wait for CCE bit to be set..


	CANBTC.all = 0;
	CANBTC.bit.BRPREG = pre - 1;
	CANBTC.bit.TSEG2REG = seg2 - 1;
	CANBTC.bit.TSEG1REG = seg1 - 1;
	CANBTC.bit.SAM = 1;
	pECanaRegs->CANBTC.all = CANBTC.all;
	//	    ECanaRegs.CANBTC.all = 0x00270059ul; // 250k @150M

	EDIS;


    return(RET_OK);
}


/***********************************************************************/
/**
* \brief codrvCanEnable - enable CAN controller
*
* This function enables the CAN controller.
* At this point the enable bit is set. Typically the CAN controller
* requests 11 recessive bits to go in active mode.
* This will be checked later outside of this function.
*
* \return RET_T
* \retval RET_OK
*	CAN controller, enabled was set
*
*/
RET_T codrvCanEnable(
		void
	)
{
RET_T	retVal = RET_OK;
union CANMC_REG   CANMC;
union CANGIM_REG  CANGIM;

	/* if error is occurred */
	/* retVal = RET_DRV_ERROR; */

	EALLOW;

	/* enable CAN controller */
	CANMC.all = pECanaRegs->CANMC.all;
	CANMC.bit.CCR = 0 ;            		// Set CCR = 0
	pECanaRegs->CANMC.all = CANMC.all;


	/* enable interrupt */
	CANGIM.all = pECanaRegs->CANGIM.all;
	CANGIM.bit.GIL = 0 ;           		// CAN0IRQ
	CANGIM.bit.I0EN = 1;           		// CAN0IRQ
	pECanaRegs->CANGIM.all = CANGIM.all;

	EDIS;

	/* Error active is later checked */
	/* later: canEnabled = CO_TRUE; */

	codrvCanEnableInterrupt();

	return(retVal);
}


/***********************************************************************/
/**
* \brief codrvCanDisable - disable CAN controller
*
* This function disables the CAN controller. Often this function wait 
* that the CAN controller is disabled, because ofter after them
* are functionality implemented, that require this.
* But note, the required time could be the time of one CAN frame.
*
* \return RET_T
* \retval RET_OK
*	CAN controller is set to disable
*
*/
RET_T codrvCanDisable(
		void
	)
{
RET_T	retVal = RET_OK;
union CANMC_REG   CANMC;

	
	/* disable CAN controller */
	EALLOW;
	CANMC.all = pECanaRegs->CANMC.all;
	CANMC.bit.CCR = 1 ;            		// Set CCR = 1
	pECanaRegs->CANMC.all = CANMC.all;
	EDIS;

	canEnabled = CO_FALSE;

	return(retVal);
}

#ifdef CO_DRV_FILTER
/***********************************************************************/
/**
* codrvCanSetFilter - activate and configure the receive filter
*
* Depend of the COB entry's the driver specific filter will 
* be configured. 
*
* 
*
* \retval RET_OK
*	OK
* \retval RET_INVALID_PARAMETER
*	invalid COB reference
* \retval RET_DRV_ERROR
*	filter cannot be set, e.g. no free entry
*
*/
RET_T codrvCanSetFilter(
		CO_CAN_COB_T * pCanCob /**< COB reference */
	)
{
UNSIGNED16 nr;
union CANME_REG   CANME;
union CANMD_REG   CANMD;
union CANMIM_REG  CANMIM;
union CANMSGID_REG     MSGID;
union CANLAM_REG LAMx;

# ifdef CO_DRV_GROUP_FILTER
		if ( ((pCanCob->canId & 0x780) == 0x700)
				|| ((pCanCob->canId & 0x780) == 0x80) )
		{
			/* group filter - no configuration req. */
			return RET_OK;
		}
# endif

	if (pCanCob->enabled == CO_FALSE) {
		nr = pCanCob->canChan;
		if (nr != 0xffffu) {
			CANME.all = pECanaRegs->CANME.all;
			CANME.all &= ~(1ul << nr); // disable
			EALLOW;
			pECanaRegs->CANME.all = CANME.all;
			EDIS;
		}

		return RET_OK;
	}

	nr = pCanCob->canChan;
	if (nr == 0xffffu) {
		if (nextFilterEntry > maxFilterEntry) {
			return RET_DRV_ERROR;
		}
		nr = nextFilterEntry;
		pCanCob->canChan = nr;
		nextFilterEntry ++;
	}

	EALLOW;

	/* init RX Mailbox */
	MSGID.all = 0;
	if (pCanCob->extended == CO_TRUE) {
		MSGID.all = pCanCob->canId & 0x1FFFFFFFul;
		MSGID.bit.IDE = 1;
	} else {
		MSGID.bit.STDMSGID = pCanCob->canId;
	}
	MSGID.bit.AME = 1;
	CanDoubleWrite(&pECanaMbox[nr].MSGID.all, &MSGID.all);

	LAMx.all = 0; // accept only the conf. id
	LAMx.bit.LAMI = 1;
	CanDoubleWrite(&pECanaLam[nr].all, &LAMx.all);

	CANMD.all = pECanaRegs->CANMD.all;
	CANMD.all |= 1ul << nr; // RX
	pECanaRegs->CANMD.all = CANMD.all;


	CANME.all = pECanaRegs->CANME.all;
	CANME.all |= 1ul << nr; // enable
	pECanaRegs->CANME.all = CANME.all;

	CANMIM.all = pECanaRegs->CANMIM.all;
	CANMIM.all |= 1ul << nr;           		// IRQ
	pECanaRegs->CANMIM.all = CANMIM.all;

	EDIS;

	return(RET_OK);
}
#endif /* CO_DRV_FILTER */

/***********************************************************************/
/**
* \brief codrvCanStartTransmission - start can transmission if not active
*
* Transmission of CAN messages should be interrupt driven.
* If a message was sent, the Transmit Interrupt is called
* and the next message can be transmitted.
* To start the transmission of the first message,
* this function is called from the CANopen stack.
*
* The easiest way to implement this function is
* to trigger the transmit interrupt, 
* but only of the transmission is not already active.
*
* \return RET_T
* \retval RET_OK
*	start transmission was successful
*
*/
RET_T codrvCanStartTransmission(
		void
	)
{
union CANGIM_REG  CANGIM;

	/* if can is not enabled, return with error */
	if (canEnabled != CO_TRUE)  {
		return(RET_DRV_ERROR);
	}

	if (transmissionIsActive == CO_FALSE)  {
		/* trigger transmit interrupt */
#ifdef POLLING
		codrvCanTransmitInterrupt();
#else

		/* disable CAN interrupt */
		EALLOW;
		CANGIM.all = pECanaRegs->CANGIM.all;
		CANGIM.bit.I0EN = 0;           		// CAN0IRQ
		pECanaRegs->CANGIM.all = CANGIM.all;
		EDIS;

		/* ?? 4 nops req. ?? */

		/* transmit first message */
		codrvCanTransmitInterrupt(); /* call interrupt handler */

		/* enable CAN interrupt */
		EALLOW;
		CANGIM.all = pECanaRegs->CANGIM.all;
		CANGIM.bit.I0EN = 1;           		// CAN0IRQ
		pECanaRegs->CANGIM.all = CANGIM.all;
		EDIS;

#endif

	}

	return(RET_OK);
}
 

/***********************************************************************/
/**
* \brief codrvCanTransmit - transmit can message
*
* This function writes a new message to the CAN controller and transmits it.
* Normally called from Transmit Interrupt
*
* \return RET_T
* \retval RET_OK
*	Transmission was ok
*
*/
static RET_T codrvCanTransmit(
		CO_CONST CO_CAN_MSG_T * pBuf		/**< pointer to data */
	)
{
RET_T	retVal = RET_OK;
UNSIGNED32 u32tmp;

union CANME_REG   CANME;
union CANMD_REG   CANMD;
union CANTRS_REG  CANTRS;
union CANMSGID_REG     MSGID;
union CANMSGCTRL_REG   MSGCTRL;

/*
	printf("< TX 0x%03lx:%d:%02x%02x%02x%02x%02x%02x%02x%02x\n",
		pBuf->canCob.canId,
		(pBuf->canCob.rtr) ? -(pBuf->len & 0xFFu) : pBuf->len & 0xFFu ,
		pBuf->data[0] & 0xFFu,
		pBuf->data[1] & 0xFFu,
		pBuf->data[2] & 0xFFu,
		pBuf->data[3] & 0xFFu,
		pBuf->data[4] & 0xFFu,
		pBuf->data[5] & 0xFFu,
		pBuf->data[6] & 0xFFu,
		pBuf->data[7] & 0xFFu);
*/


	/* write message to the CAN controller */

	/* disable MB */
	CANME.all = pECanaRegs->CANME.all;
	CANME.bit.ME0 = 0; // disable
	pECanaRegs->CANME.all = CANME.all;

	MSGID.all = 0;
	if (pBuf->canCob.extended == CO_TRUE) {
		MSGID.all = pBuf->canCob.canId;
		MSGID.bit.IDE = 1;
	} else {
		MSGID.bit.STDMSGID = pBuf->canCob.canId;
	}
	CanDoubleWrite(&pECanaMbox[0].MSGID.all, &MSGID.all);

	CANMD.all = pECanaRegs->CANMD.all;
	CANMD.bit.MD0 = 0; // TX
	pECanaRegs->CANMD.all = CANMD.all;

	//enable
	CANME.all = pECanaRegs->CANME.all;
	CANME.bit.ME0 = 1; // enable
	pECanaRegs->CANME.all = CANME.all;

	/* data length x */
	MSGCTRL.all = 0;
	MSGCTRL.bit.DLC = pBuf->len;

	if (pBuf->canCob.rtr == CO_TRUE) {
		MSGCTRL.bit.RTR = 1;
	} else {
		/* data 0x1..0x8 */
		u32tmp = ((UNSIGNED32)pBuf->data[0] & 0xFFu
			|    (UNSIGNED32)(pBuf->data[1] & 0xFFu) << 8
			|    (UNSIGNED32)(pBuf->data[2] & 0xFFu) << 16
			|    (UNSIGNED32)(pBuf->data[3] & 0xFFu) << 24);

		CanDoubleWrite(&pECanaMbox[0].MDL.all, &u32tmp);

		u32tmp = ((UNSIGNED32)pBuf->data[4] & 0xFFu
			|    (UNSIGNED32)(pBuf->data[5] & 0xFFu) << 8
			|    (UNSIGNED32)(pBuf->data[6] & 0xFFu) << 16
			|    (UNSIGNED32)(pBuf->data[7] & 0xFFu) << 24);

		CanDoubleWrite(&pECanaMbox[0].MDH.all, &u32tmp);
	}
	CanDoubleWrite(&pECanaMbox[0].MSGCTRL.all, &MSGCTRL.all);

	/* transmit it */
	transmissionIsActive = CO_TRUE;

	CANTRS.all = 0ul;//ECanaRegs.CANTRS.all;
	CANTRS.bit.TRS0 = 1; // Transmit
	pECanaRegs->CANTRS.all = CANTRS.all;

	return(retVal);
}


/***********************************************************************/
/**
* \brief codrvCanDriverTransmitInterrupt - can driver transmit interrupt
*
* This function is called, after a message was transmitted.
*
* As first, inform stack about message transmission.
* Get the next message from the transmit buffer,
* write it to the CAN controller
* and transmit it.
*
* \return void
*
*/

void codrvCanTransmitInterrupt(
		void
	)
{
union CANTRS_REG  CANTRS;

	CANTRS.all = pECanaRegs->CANTRS.all;
	if (CANTRS.bit.TRS0 == 1) {
		//message is during transmitting
		return;
	}

	pECanaRegs->CANTA.all = 1ul; //reset TXbuffer 0
	while(pECanaRegs->CANTA.all != 0) {
		; /* wait for reset the TA bit */
	}

	transmissionIsActive = CO_FALSE;

	/* inform stack about transmitted message */
	if (pTxBuf != NULL)  {
		coQueueMsgTransmitted(pTxBuf);
		pTxBuf = NULL;
	}

	/* get next message from transmit queue */
	pTxBuf = coQueueGetNextTransmitMessage();
	if (pTxBuf != NULL)  {
		/* and transmit it */
		(void)codrvCanTransmit(pTxBuf);
	}

}


/***********************************************************************/
/**
* \brief codrvCanReceiveInterrupt - can driver receive interrupt
*
* This function is called, if a new message was received.
* As first get the pointer to the receive buffer
* and save the message there.
* Then set the buffer as filled and inform the lib about new data.
*
* \return void
*
*/
void codrvCanReceiveInterrupt(
		void
	)
{
CO_CAN_MSG_T * pRecBuf;
CAN_ERROR_FLAGS_T * pError;
volatile struct MBOX * pRxMbox;

BOOL_T fReread;
UNSIGNED8 lLen;

UNSIGNED16 nr;
UNSIGNED32 mask;

union CANRMP_REG  CANRMP;
union CANRML_REG  CANRML;

union CANMSGID_REG     MSGID;
union CANMSGCTRL_REG   MSGCTRL;


 	CANRMP.all = pECanaRegs->CANRMP.all; //message pending?
	if (CANRMP.all == 0) {
		//no receive message???
		return;
	}

	mask = 1ul << 31;
	for (nr = 31u ; nr > 0; nr-- ) {
		if ((CANRMP.all & mask) != 0) {
			break;
		}
		mask >>= 1;
	}

	/* get receive buffer */
	pRecBuf = coQueueGetReceiveBuffer();
	if (pRecBuf == NULL)  {
		/* error, no buffer available
		 * release all mailboxes -
		 * there is also no free buffer at the next IRQ
		 */
		pECanaRegs->CANRMP.all = CANRMP.all;
		return;
	}

	CANRML.all = pECanaRegs->CANRML.all;
	if ((CANRML.all & mask) != 0) {
		/* Message overflow */
		pError = codrvCanErrorGetFlags();
		pError->canErrorRxOverrun = CO_TRUE;
	}

	pRxMbox = &pECanaMbox[nr];

	do {
		/* reset message pending bit */
		pECanaRegs->CANRMP.all = mask;

		fReread = CO_FALSE;

		/* save message at buffer */

		CanDoubleRead(&MSGID.all, &pRxMbox->MSGID.all);
		if (MSGID.bit.IDE == 0) {
			pRecBuf->canCob.canId = MSGID.bit.STDMSGID;
			pRecBuf->canCob.extended = CO_FALSE;
		} else {
			pRecBuf->canCob.canId = MSGID.all & 0x1FFFFFFFul;
			pRecBuf->canCob.extended = CO_TRUE;
		}

		CanDoubleRead(&MSGCTRL.all, &pRxMbox->MSGCTRL.all);
		lLen = MSGCTRL.bit.DLC;
		pRecBuf->len = lLen;
		if (MSGCTRL.bit.RTR == 1) {
			pRecBuf->canCob.rtr = CO_TRUE;
		} else {
			UNSIGNED8 * pData;
			UNSIGNED32 u32tmp;

			pRecBuf->canCob.rtr = CO_FALSE;

			CanDoubleRead(&u32tmp, &pRxMbox->MDL.all);
			pData = &pRecBuf->data[0];

			*pData ++ = (UNSIGNED8)u32tmp & 0xFFu;
			*pData ++ = (UNSIGNED8)(u32tmp >> 8) & 0xFFu;
			*pData ++ = (UNSIGNED8)(u32tmp >> 16) & 0xFFu;
			*pData ++ = (UNSIGNED8)(u32tmp >> 24) & 0xFFu;

			if (lLen > 4) {
				CanDoubleRead(&u32tmp, &pRxMbox->MDH.all);
				*pData ++ = (UNSIGNED8)u32tmp & 0xFFu;
				*pData ++ = (UNSIGNED8)(u32tmp >> 8) & 0xFFu;
				*pData ++ = (UNSIGNED8)(u32tmp >> 16) & 0xFFu;
				*pData    = (UNSIGNED8)(u32tmp >> 24) & 0xFFu;
			}
		}

		if ((pECanaRegs->CANRMP.all & mask) != 0) {
			/* new pending message - old message was overwritten */
			fReread = CO_TRUE;

			pError = codrvCanErrorGetFlags();
			pError->canErrorRxOverrun = CO_TRUE;
		}

	} while (fReread != CO_FALSE);

	//remove?
	pECanaRegs->CANRMP.all = mask;

/*
	printf("> RX 0x%03lx:%d:%02x%02x%02x%02x%02x%02x%02x%02x\n",
		pRecBuf->canCob.canId,
		(pRecBuf->canCob.rtr)? -pRecBuf->len : pRecBuf->len ,
		pRecBuf->data[0],
		pRecBuf->data[1],
		pRecBuf->data[2],
		pRecBuf->data[3],
		pRecBuf->data[4],
		pRecBuf->data[5],
		pRecBuf->data[6],
		pRecBuf->data[7]);
*/

	/* set buffer filled */
	coQueueReceiveBufferIsFilled();

	/* inform stack about new data */
	coCommTaskSet(CO_COMMTASK_EVENT_MSG_AVAIL);

}


/***********************************************************************/
/**
* \brief codrvCanErrorHandler - local Error handler
*
* This function polls the current state of the CAN controller
* and checks explicitly all situation that are not signalled
* within the interrupts.
*
* To be called outside of interrupts!
* Typically called in codrvCanDriverHandler().
*
* \return void
*
*/

static void codrvCanErrorHandler(void)
{
CAN_ERROR_FLAGS_T * pError;
union CANES_REG CANES;
union CANMC_REG CANMC;
static UNSIGNED16 errorWaitCounter = 0u;

	pError = codrvCanErrorGetFlags();

	CANES.all = pECanaRegs->CANES.all;
	pECanaRegs->CANES.all = CANES.all; /* ack state */

	CANMC.all = pECanaRegs->CANMC.all;

	if (canEnabled == CO_FALSE) {
		pError->canNewState = Error_Offline;

		if (CANES.bit.CCE == 0) {
			/* enable can */
			pError->canNewState = Error_Active;
			canEnabled = CO_TRUE;
		}
	} else if (pError->canOldState == Error_Busoff) {
		if (CANMC.bit.CCR == 0) {
			pError->canNewState = Error_Active;
		} else {
			if (CANES.bit.CCE == 1) {
				CANMC.bit.CCR = 0;
				EALLOW;
				pECanaRegs->CANMC.all = CANMC.all;
				EDIS;
			}
			pError->canNewState = Error_Busoff;
		}

	} else if ((CANES.bit.BO != 0) /*|| (ECanaShadow.CANGIF0.bit.BOIF0 != 0 )*/) {
		/* busoff - problem to see it with auto buson */
		pError->canNewState = Error_Busoff;

		transmissionIsActive = CO_FALSE;

		CANMC.all = pECanaRegs->CANMC.all;
		if (CANMC.bit.ABO == 0) {
			CANMC.bit.CCR = 0;
			EALLOW;
			pECanaRegs->CANMC.all = CANMC.all;
			EDIS;
		}

	} else if ((CANES.bit.EP != 0)  /*|| (ECanaShadow.CANGIF0.bit.EPIF0 != 0 )*/) {
		/* error passive */
		pError->canNewState = Error_Passive;
	} else {
		/* error active */
		pError->canNewState = Error_Active;
	}

	/* correct possible Errors -> CAN has deactivated the transmission */
	if (transmissionIsActive == CO_TRUE) {
		if (pECanaRegs->CANTRS.all == 0) {
			/* e.g. after Busoff */
			errorWaitCounter++;
			if (errorWaitCounter > 20) {
				transmissionIsActive = CO_FALSE;
			}
		} else {
			errorWaitCounter = 0u;
		}
	} else {
		errorWaitCounter = 0u;
	}


	if (canEnabled == CO_TRUE) {
		/* check for stopped transmissions */
		if ((transmissionIsActive == CO_FALSE) && (pTxBuf != NULL)) {
			/* transmission aborted, e.g. busoff, 
		     * discard message -> is done within the tx interrupt
			*/
			(void)codrvCanStartTransmission();
		}
	}

	//printf("ErrorHandler canNewState %d", (int)(pError->canNewState));

}


/***********************************************************************/
/**
* \brief codrvCanDriverHandler - can driver handler
*
* This function is cyclically called from the CANopen stack
* to get the current CAN state
* (BUS_OFF, PASSIVE, ACTIVE).
*
* If a bus off event has occurred,
* this function should try to get to bus on again
* (activate the CAN controller).
*
* \return void
*
*/
void codrvCanDriverHandler(
		void
	)
{
	/* check current state */
	codrvCanErrorHandler();

	/* inform stack about the state changes during two handler calls */
	(void)codrvCanErrorInformStack();

#ifdef POLLING
	/* call interrupts by polling */
	codrvCanReceiveInterrupt();
	codrvCanTransmitInterrupt();
#endif

    return;
}

