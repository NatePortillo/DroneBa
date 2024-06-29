#include <xstatus.h>
#include "xuartps.h"


s32 UART_PS_Init(XUartPs *UartPSInstance, UINTPTR UARTPSAddr);
s32 ESP32_Read(XUartPs *UartPSInstance, u8 *ESP32_RecieveBuffer);