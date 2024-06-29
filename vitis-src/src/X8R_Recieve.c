#include "X8R_Recieve.h"

/**
 * @brief Initializes the UART 550 interface with a specified baud rate.
 *
 * This function configures the UART 550 interface with a baud rate of 100k. It sets up
 * the line control register, divisor latch, and FIFO control register.
 * It also enables Received Data Available Interrupts.
 *
 * @return void
 */
void UART550_Init() {
    // Configure BAUD rate to 100k
    Xil_Out32(LINE_CNTRL_REG, 0x80); // Enable divisor latch
    Xil_Out32(DIV_LATCH_LSB, 0x3E);    // Set divisor latch LSB (for 100k BAUD rate)
    Xil_Out32(DIV_LATCH_MSB, 0x00000000);     // Set divisor latch MSB
    Xil_Out32(LINE_CNTRL_REG, 0x1F); // 8 data bits, 2 stop bits, even parity

    Xil_Out32(IER_REG, 0x01); //Enables Received Data Available Interrupts
    // Enable FIFO, reset TX and RX FIFOs
    Xil_Out32(FIFO_CNTRL_REG, 0x1);
}

/**
 * @brief Sets up the interrupt system for the UART 550 interface.
 *
 * This function initializes the interrupt controller and configures the UART 550
 * instance to handle interrupts. It sets up the interrupt handler and initializes
 * the UART 550 interface.
 *
 * @param IntcInstancePtr Pointer to the XScuGic instance to be initialized.
 * @param UART550InstancePtr Pointer to the XUartNs550 instance to be configured.
 * @param BaseAddress The base address of the UART 550 device.
 *
 * @return XST_SUCCESS if the setup is successful, XST_FAILURE otherwise.
 */
int SetupInterruptSystem(XScuGic *IntcInstancePtr, XUartNs550 *UART550InstancePtr, UINTPTR BaseAddress) {
    XUartNs550_Config *UART550Ptr;
    XScuGic_Config *IntcConfig;
    int Status;
    
    // Initialize the interrupt controller driver
    IntcConfig = XScuGic_LookupConfig(XPAR_XSCUGIC_0_BASEADDR);
    if (IntcConfig == NULL) {
        return XST_FAILURE;
    }
    Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }    
    UART550Ptr = XUartNs550_LookupConfig(BaseAddress);
	Status = XUartNs550_Initialize(UART550InstancePtr, BaseAddress);
    Status = XSetupInterruptSystem(UART550InstancePtr, &UARTInterruptHandler,
                    UART550Ptr->IntrId, UART550Ptr->IntrParent,
                    XINTERRUPT_DEFAULT_PRIORITY);

    XUartNs550_SetHandler(UART550InstancePtr, UARTInterruptHandler,
			      UART550InstancePtr);  
    
    UART550_Init();     

    return XST_SUCCESS;
}

/**
 * @brief UART interrupt handler function.
 *
 * This function handles UART interrupts. It updates the total received count
 * when data is received.
 *
 * @param CallBackRef Pointer to the callback reference.
 * @param Event The type of event that has occurred.
 * @param EventData Data associated with the event.
 *
 * @return void
 */
void UARTInterruptHandler(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartNs550 *UartNs550Ptr  = (XUartNs550 *)CallBackRef;
	/*
	 * All of the data has been received.
	 */
	if (Event == XUN_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
	}

}

/**
 * @brief Parses SBUS data from the receive buffer into channel values.
 *
 * This function parses SBUS data from the receive buffer and extracts the channel values.
 * It validates the SBUS frame and decodes the channel data into the provided channels buffer.
 *
 * @param rx_buffer Pointer to the receive buffer containing SBUS data.
 * @param channels_buffer Pointer to the buffer where the parsed channel values will be stored.
 *
 * @return XST_SUCCESS if the data is successfully parsed, XST_FAILURE otherwise.
 */
s32 parseSBUSData(uint8_t *rx_buffer, uint16_t *channels_buffer) {
    if (rx_buffer[0] != SBUS_START_BYTE || rx_buffer[24] != SBUS_END_BYTE) {
        // Invalid SBUS frame
        return XST_FAILURE;
    }
    channels_buffer[0]  = (rx_buffer[1]   | rx_buffer[2] << 8) & 0x07FF;
    channels_buffer[1]  = (rx_buffer[2] >> 3 | rx_buffer[3] << 5) & 0x07FF;
    channels_buffer[2]  = (rx_buffer[3] >> 6 | rx_buffer[4] << 2 | rx_buffer[5] << 10) & 0x07FF;
    channels_buffer[3]  = (rx_buffer[5] >> 1 | rx_buffer[6] << 7) & 0x07FF;
    channels_buffer[4]  = (rx_buffer[6] >> 4 | rx_buffer[7] << 4) & 0x07FF;
    channels_buffer[5]  = (rx_buffer[7] >> 7 | rx_buffer[8] << 1 | rx_buffer[9] << 9) & 0x07FF;
    channels_buffer[6]  = (rx_buffer[9] >> 2 | rx_buffer[10] << 6) & 0x07FF;
    channels_buffer[7]  = (rx_buffer[10] >> 5 | rx_buffer[11] << 3) & 0x07FF;
    channels_buffer[8]  = (rx_buffer[12] | rx_buffer[13] << 8) & 0x07FF;
    channels_buffer[9]  = (rx_buffer[13] >> 3 | rx_buffer[14] << 5) & 0x07FF;
    channels_buffer[10] = (rx_buffer[14] >> 6 | rx_buffer[15] << 2 | rx_buffer[16] << 10) & 0x07FF;
    channels_buffer[11] = (rx_buffer[16] >> 1 | rx_buffer[17] << 7) & 0x07FF;
    channels_buffer[12] = (rx_buffer[17] >> 4 | rx_buffer[18] << 4) & 0x07FF;
    channels_buffer[13] = (rx_buffer[18] >> 7 | rx_buffer[19] << 1 | rx_buffer[20] << 9) & 0x07FF;
    channels_buffer[14] = (rx_buffer[20] >> 2 | rx_buffer[21] << 6) & 0x07FF;
    channels_buffer[15] = (rx_buffer[21] >> 5 | rx_buffer[22] << 3) & 0x07FF;

    // Decode flags
    failsafe = rx_buffer[23] & 0x08;
    lost = rx_buffer[23] & 0x04;

    return XST_SUCCESS;
}

/**
 * @brief Maps SBUS values to a specified range.
 *
 * This function maps SBUS values from the range [172, 1811] to a specified output range
 * [min_output, max_output]. It clips the input SBUS value to ensure it falls within the
 * valid SBUS range before performing the mapping.
 *
 * @param sbus_value The SBUS value to be mapped.
 * @param min_output The minimum value of the output range.
 * @param max_output The maximum value of the output range.
 *
 * @return The mapped value within the specified output range.
 */
float mapSBUSToRange(uint16_t sbus_value, float min_output, float max_output) {
    // Clip the input value to be within the SBUS range
    if (sbus_value < 172 || sbus_value == 195) {
        sbus_value = 172;
    } else if (sbus_value > 1811) {
        sbus_value = 1811;
    }

    // Perform the mapping
    return ((float)(sbus_value - 172) / (1811 - 172)) * (max_output - min_output) + min_output;
}