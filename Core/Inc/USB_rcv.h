#ifndef __USB_RCV_H
#define __USB_RCV_H

#include "main.h"
#include "cmsis_os2.h"
struct USB_rcv_t
{
    uint8_t* pBuffer;
    uint32_t *pLength;
};

extern osMessageQueueId_t* pUSB_rcvQueueHandle;

extern void USB_rcvTask(void);

#endif /* __USB_RCV_H */