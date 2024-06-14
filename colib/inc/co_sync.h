/*
* co_sync.h - contains defines for sync services
*
* Copyright (c) 2012-2015 emtas GmbH
*
*-------------------------------------------------------------------
* $Id: co_sync.h 10877 2015-10-02 09:02:38Z boe $
*-------------------------------------------------------------------
*
*
*
*/

/**
* \brief defines for sync services
*
* \file co_sync.h - contains defines for sync services
*/

#ifndef CO_SYNC_H
#define CO_SYNC_H 1

#include <co_datatype.h>


/* datatypes */

/** \brief function pointer to SYNC indication
 * \param syncCounter - actual SYNC counter
 * 
 * \return void
 */
typedef void (* CO_EVENT_SYNC_T)(UNSIGNED8);


/** \brief function pointer to SYNC Finished indication
 * \param syncCounter - actual SYNC counter
 * 
 * \return void
 */
typedef void (* CO_EVENT_SYNC_FINISHED_T)(UNSIGNED8);


EXTERN_DECL RET_T coSyncInit(UNSIGNED32	cobId);
EXTERN_DECL RET_T coEventRegister_SYNC(CO_EVENT_SYNC_T);
EXTERN_DECL RET_T coEventRegister_SYNC_FINISHED(CO_EVENT_SYNC_FINISHED_T);

#endif /* CO_SYNC_H */
