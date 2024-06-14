/*
* codrv_cpu_28069.c - contains driver for cpu
*
* Copyright (c) 2012-2016 emtas GmbH
*-------------------------------------------------------------------
* $Id: codrv_cpu_28335.c 6411 2014-07-23 14:06:09Z ro $
*
*
*-------------------------------------------------------------------
*
*
*/

/********************************************************************/
/**
* \brief CPU specific routines
*
* \file codrv_cpu_28069.c
* cpu specific routines
*
* This module contains the cpu specific routines for initialization
* and timer handling.
*
* \author emtas GmbH
*/

/* header of standard C - libraries
---------------------------------------------------------------------------*/
#include <stdio.h>

/* header of project specific types
---------------------------------------------------------------------------*/
#include <gen_define.h>

#include <co_datatype.h>
#include <co_timer.h>

#include "codrv_cpu.h"


/* constant definitions
---------------------------------------------------------------------------*/
#ifndef CAN_IRQ_BEGIN
# define CAN_IRQ_BEGIN()
# define CAN_IRQ_END()
#endif


/* local defined data types
---------------------------------------------------------------------------*/

/* list of external used functions, if not in headers
---------------------------------------------------------------------------*/

/* list of global defined functions
---------------------------------------------------------------------------*/
void InitSystem(void);
void Gpio_select(void);

/* list of local defined functions
---------------------------------------------------------------------------*/
interrupt void codrvTimerISR(void);

static void codrvInitECanaGpio(void);
/* external variables
---------------------------------------------------------------------------*/

/* global variables
---------------------------------------------------------------------------*/

/* local defined variables
---------------------------------------------------------------------------*/

/***************************************************************************/
/**
* \brief codrvHardwareInit - hardware initialization
*
* This function initializes the hardware, incl. clock and CAN hardware.
*/
void codrvHardwareInit(void)
{
	InitSysCtrl(); //init PLL, activate clocks

	DINT; /* disable interrupt */
	IER = 0x0000;
	IFR = 0x0000; /* disable CPU interrupts and reset flags */

	InitPieCtrl();
	InitPieVectTable(); /* copy Flash IRQ table in RAM */

//	InitPeripherals();

	InitCpuTimers();   // only cpu timer0 req.

	codrvHardwareCanInit();
}

/***************************************************************************/
/**
* \brief codrvInitCanHW - CAN related hardware initialization
*
* Within this function you find the CAN only hardware part.
* Goal of it is, that you can have your own hardware initialization
* like codrvHardwareInit(), but you can add our tested CAN 
* initialization.
*
*/
interrupt void codrvCan0Irq(void);
void codrvHardwareCanInit(void)
{
	EALLOW;
	// enable clock (sysclkout/2)
    SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=1;    // eCAN-A

	EDIS;

    // GPIO Pins
    codrvInitECanaGpio();

	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.ECAN0INTA = &codrvCan0Irq;
	EDIS;    // This is needed to disable write to EALLOW protected registers
}

/***************************************************************************/
/**
* \brief codrvCanEnableInterrupt - enable the CAN interrupt
*
*/
void codrvCanEnableInterrupt(void)
{
	/* enable CAN interrupts */

    // Enable ECAN0INT in the PIE: Group 9 interrupt 5
   PieCtrlRegs.PIEIER9.bit.INTx5 = 1;	// eCAN0INT

   // Enable CPU INT9 which is connected to CAN:
   IER |= M_INT9;

   //printf("Enable CAN Interrupt\n");

}

/***************************************************************************/
/**
* \brief codrvCanDisableInterrupt - disable the CAN interrupt
*
*/
void codrvCanDisableInterrupt(void)
{
	/* disable CAN interrupts */

	// Disable ECAN0INT in the PIE: Group 9 interrupt 5
	PieCtrlRegs.PIEIER9.bit.INTx5 = 0;

}

/***************************************************************************/
/**
* \brief codrvCanSetTxInterrupt - set pending bit of the Transmit interrupt
*
* This function set the interrupt pending bit. In case of the NVIC
* enable interrupt and the CAN specific enable TX Interrupt mask
* the CAN interrupt handler is calling.
*
*/
void codrvCanSetTxInterrupt(void)
{
	/* not possible */
}


extern void codrvCanReceiveInterrupt(void);
extern void codrvCanTransmitInterrupt(void);

interrupt void codrvCan0Irq(void)
{
	CAN_IRQ_BEGIN();

	codrvCanReceiveInterrupt();
	codrvCanTransmitInterrupt();

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; //Ack

	CAN_IRQ_END();
}

/***************************************************************************/
/**
* \brief codrvTimerSetup - init and configure the hardware Timer
*
* This function starts a cyclic hardware timer to provide a timing interval
* for the CANopen library.
* Alternativly it can be derived from an other system timer
* with the timer interval given by the function parameter.
*
* \return RET_T
* \retval RET_OK
*	intialization of the timer was ok
*
*/

RET_T codrvTimerSetup(
		UNSIGNED32	timerInterval		/**< timer interval in usec */
	)
{
	/* start hardware timer */

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.TINT0 = &codrvTimerISR;
	EDIS;    // This is needed to disable write to EALLOW protected registers

	// Configure CPU-Timer 0 to interrupt every 1ms:
	// 90MHz CPU Freq, x ms Period (in uSeconds)
	ConfigCpuTimer(&CpuTimer0, 90u, timerInterval);
	StartCpuTimer0();

	// Enable CPU INT1 which is connected to CPU-Timer 0:
	IER |= M_INT1;

	// Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	return(RET_OK);
}


/***************************************************************************/
/**
* \brief codrvTimerISR - Timer interrupt service routine
*
* This function is normally called from timer interrupt
* or from an other system timer.
* It has to call the timer handling function at the library.
*
*
* \return void
*
*/
interrupt void codrvTimerISR(
		void
    )
{
	/* inform stack about new timer event */
	coTimerTick();

	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}


/***************************************************************************/
/**
* \brief InitECanaGpio() from DSP2833x_ECan.c
* \Todo
*
* \return void
*
*/
static void codrvInitECanaGpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected CAN pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;	    // Enable pull-up for GPIO30 (CANRXA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	    // Enable pull-up for GPIO18 (CANRXA)

	GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;	    // Enable pull-up for GPIO31 (CANTXA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	    // Enable pull-up for GPIO19 (CANTXA)

/* Set qualification for selected CAN pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;   // Asynch qual for GPIO18 (CANRXA)


/* Configure eCAN-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAN functional pins.

	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;	// Configure GPIO30 for CANRXA operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;	// Configure GPIO18 for CANRXA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;	// Configure GPIO31 for CANTXA operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;	// Configure GPIO19 for CANTXA operation

    EDIS;
}

