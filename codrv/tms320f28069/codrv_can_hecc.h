/*
* codrv_can_hecc.h
*
* Copyright (c) 2013 emtas GmbH
*-------------------------------------------------------------------
* SVN  $Id: codrv_can_hecc.h 4464 2013-09-03 15:22:55Z  $
*
*
*-------------------------------------------------------------------
*
*
*/

/********************************************************************/
/**
* \file
* \brief header for TI ECAN driver in HECC mode
*
* The TI Headerfiles are required.
*
*/

#ifndef CODRV_CAN_HECC_H
#define CODRV_CAN_HECC_H 1


/* global prototypes, that not in co_drv.h */
void codrvCanReceiveInterrupt(void);
void codrvCanTransmitInterrupt(void);

/* extern required functions */
extern void codrvCanSetTxInterrupt(void);

#endif /* CODRV_CAN_GENERIC_H */

