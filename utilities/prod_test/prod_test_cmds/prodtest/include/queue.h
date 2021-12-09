/**
 ****************************************************************************************
 *
 * @file queue.h
 *
 * @brief Header file for queues and threads definitions.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include <conio.h>
#include <process.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <windows.h>
#include <process.h>
#include <stddef.h>     // standard definition


// Queue stuff.
struct QueueStorage {
  struct QueueStorage *Next;
  void *Data;
};

typedef struct {
  struct QueueStorage *First,*Last;
} QueueRecord;


typedef struct {
  unsigned char payload_type;
  unsigned short payload_size;
  unsigned char *payload;
} QueueElement;


// Used to stop the tasks.
extern BOOL StopRxTask;

extern HANDLE UARTRxQueueSem; // mutex semaphore to protect RX queue

extern HANDLE Rx232Id;  // Thread handles

extern QueueRecord UARTRxQueue; // UART Rx queue

extern HANDLE QueueHasAvailableData; // set when the UART Rx queue is not empty

void EnQueue(QueueRecord *rec,void *vdata);
void *DeQueue(QueueRecord *rec);

void InitTasks(void);


#endif //QUEUE_H_