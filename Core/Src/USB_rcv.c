#include "USB_rcv.h"
#include "usbd_cdc_if.h"

osMessageQueueId_t* pUSB_rcvQueueHandle;
static struct USB_rcv_t USB_rcv_buf;

void USB_rcvTask(void)
{
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(*pUSB_rcvQueueHandle, &USB_rcv_buf, NULL, osWaitForever);
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    /* echo characters */
    CDC_Transmit_FS(USB_rcv_buf.pBuffer, *(USB_rcv_buf.pLength));
  }    
}