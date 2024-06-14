/*
* codrv_cpu.h
*
* Copyright (c) 2012-2016 emtas GmbH
*-------------------------------------------------------------------
* SVN  $Date: 2014-04-02 15:03:17 +0200 (Mi, 02 Apr 2014) $
* SVN  $Rev: 5601 $
* SVN  $Author: ro $
*
*
*-------------------------------------------------------------------
*
*
*/

/********************************************************************/
/**
* \file
* \brief CPU driver part for the tms320f28xx driver
*
*
*
*/

#ifndef CODRV_CPU_H
#define CODRV_CPU_H 1

#include "F2806x_Device.h"
#include "F2806x_Examples.h"   // Examples Include File
#include "F2806x_GlobalPrototypes.h"

/* general hardware initialization */
void codrvHardwareInit(void);

/* init CAN related hardware part */
void codrvHardwareCanInit(void);

#endif /* CODRV_CPU_H */
