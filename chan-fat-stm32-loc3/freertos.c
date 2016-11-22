/*  FreeRTOS semaphore locking for Chan FAT

Copyright K Sarkies 30 September 2013
*/

#include "integer.h"
#include "ffconf.h"

#if _FS_REENTRANT

#include "FreeRTOS.h"
#include "semphr.h"

/*---------------------------------------------------------------------------*/
/** @brief Create a Sync Object

Create a sync object. For FreeRTOS this is a semaphore to lock out access while
another access is happening to the same volume.

@param[in] BYTE vol: The volume being accessed. An index to an array of semaphores.
@param[in] _SYNC_t* sobj: pointer to an external semaphore array.
@returns true if the semaphore was successfully created.
*/

int ff_cre_syncobj(BYTE vol, _SYNC_t* sobj)
{
    vSemaphoreCreateBinary(sobj[vol]);
    return (sobj[vol] != NULL);
}

/*---------------------------------------------------------------------------*/
/** @brief Grant a Sync Request

Lock the sync object.

@param[in] _SYNC_t sobj: The semaphore.
@returns true if the grant was successful.
*/

int ff_req_grant(_SYNC_t sobj)
{
    return xSemaphoreTake(sobj,_FS_TIMEOUT);
}

/*---------------------------------------------------------------------------*/
/** @brief Grant a Sync Request

Unlock the sync object.

@param[in] _SYNC_t sobj: The semaphore.
*/

void ff_rel_grant(_SYNC_t sobj)
{
    xSemaphoreGive(sobj);
}

/*---------------------------------------------------------------------------*/
/** @brief Delete a Sync Object

A semaphore shouldn't be deleted while in use. Solve this by grabbing it then
releasing. There is no 100% reliable way to do this in FreeRTOS.

@param[in] _SYNC_t sobj: The semaphore.
@returns true if the delete was successful.
*/

int ff_del_syncobj(_SYNC_t sobj)
{
    xSemaphoreTake(sobj,portMAX_DELAY);
    xSemaphoreGive(sobj);
    vSemaphoreDelete(sobj);
    return (sobj == NULL);
}

#endif
