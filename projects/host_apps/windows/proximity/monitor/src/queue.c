/**
 ****************************************************************************************
 *
 * @file queue.c
 *
 * @brief Software for queues and threads.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "queue.h"
#include "console.h"
#include "uart.h"

// Used to stop the tasks.
BOOL StopConsoleTask, StopRxTask;
HANDLE ConsoleQueueSem;    // mutex semaphore to protect console event queue
HANDLE UARTRxQueueSem;     // mutex semaphore to protect uart rx queue
HANDLE Rx232Id, ConsoleTaskId;  // Thread handles
QueueRecord ConsoleQueue, UARTRxQueue; //Queues UARTRx -> Main thread /  Console -> Main thread


void InitTasks(void)
{
    StopConsoleTask = FALSE;
    StopRxTask = FALSE;
    Rx232Id   = (HANDLE) _beginthread(UARTProc, 10000, NULL);
    ConsoleTaskId   = (HANDLE) _beginthread(ConsoleProc, 10000, NULL);
    // Set thread priorities
    SetThreadPriority(Rx232Id, THREAD_PRIORITY_TIME_CRITICAL);
    SetThreadPriority(ConsoleTaskId, THREAD_PRIORITY_TIME_CRITICAL);
    ConsoleQueueSem = CreateMutex( NULL, FALSE, NULL );
    UARTRxQueueSem = CreateMutex( NULL, FALSE, NULL );
}


void EnQueue(QueueRecord *rec,void *vdata)
{
    struct QueueStorage *tmp;

    tmp = (struct QueueStorage *) malloc(sizeof(struct QueueStorage));
    tmp->Next = NULL;
    tmp->Data = vdata;
    if(rec->First == NULL)
    {
        rec->First=tmp;
        rec->Last=tmp;
    }
    else
    {
        rec->Last->Next=tmp;
        rec->Last=tmp;
    }
}


void *DeQueue(QueueRecord *rec)
{
    void *tmp;
    struct QueueStorage *tmpqe;

    if(rec->First == NULL)
    {
        return NULL;
    }
    else
    {
        tmp = rec->First->Data;
        tmpqe = rec->First;
        rec->First = tmpqe->Next;
        free(tmpqe);


        if(rec->First == NULL)
        {
            rec->First = rec->Last = NULL;
        }

    }
    return tmp;
}
