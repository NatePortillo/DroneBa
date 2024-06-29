#include "ESP32Data.h"
#include <xstatus.h>

/**
 * @brief Initializes the UART (Universal Asynchronous Receiver/Transmitter) interface.
 *
 * This function initializes the UART interface on a specified base address and sets
 * the desired baud rate. It performs the following steps:
 * 1. Looks up the configuration for the UART device based on the provided base address.
 * 2. Initializes the UART instance using the retrieved configuration.
 * 3. Performs a self-test on the UART instance to ensure it is functioning correctly.
 * 4. Sets the UART baud rate to 115200.
 *
 * @param UartPSInstance Pointer to the XUartPs instance to be initialized.
 * @param UARTPSAddr The base address of the UART device.
 *
 * @return XST_SUCCESS if the initialization is successful, XST_FAILURE otherwise.
 */
s32 UART_PS_Init(XUartPs *UartPSInstance, UINTPTR UARTPSAddr){
    s32 Status;
    XUartPs_Config *UartConfigPtr;

    UartConfigPtr = XUartPs_LookupConfig(UARTPSAddr);
    if (UartConfigPtr == NULL) {
        return XST_FAILURE;
    }
    Status = XUartPs_CfgInitialize(UartPSInstance, UartConfigPtr, UartConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    Status = XUartPs_SelfTest(UartPSInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    Status = XUartPs_SetBaudRate(UartPSInstance,115200);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/**
 * @brief Reads a data packet from the ESP32 via UART.
 *
 * This function reads a data packet of 13 bytes from the ESP32 via UART and stores the 
 * received data in the provided buffer. It initializes the receive buffer to zero before 
 * receiving the data.
 *
 * @param UartPSInstance Pointer to the XUartPs instance used for UART communication.
 * @param ESP32_RecieveBuffer Pointer to a buffer where the received data will be stored.
 *                            The buffer must be able to hold at least 13 bytes.
 *
 * @return XST_SUCCESS if the data is successfully received, XST_FAILURE otherwise.
 */
s32 ESP32_Read(XUartPs *UartPSInstance, u8 *ESP32_RecieveBuffer){
        static int ReceivedCount = 0;

        for(int i = 0; i < 13; i++){
            ESP32_RecieveBuffer[i] = 0;
        }

        /* Receive data packet from ESP32 */
        while (ReceivedCount < 13) {
            ReceivedCount +=
                XUartPs_Recv(UartPSInstance, &ESP32_RecieveBuffer[ReceivedCount],
                        (13 - ReceivedCount));
        }
    return XST_SUCCESS;
}