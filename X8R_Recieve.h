#include "xparameters.h"
#include "xuartns550_l.h"
#include "xuartns550.h"
#include "xscugic.h"
#include "xinterrupt_wrap.h"
#include <stdbool.h>

#define UART_BAUDRATE		10000   /* Baud Rate */
// UART16550 Register Addresses
#define UART16550_BASEADDR XPAR_XUARTNS550_0_BASEADDR
#define LINE_CNTRL_REG (UART16550_BASEADDR + 0x100C)
#define DIV_LATCH_LSB (UART16550_BASEADDR + 0x1000)
#define DIV_LATCH_MSB (UART16550_BASEADDR + 0x1004)
#define FIFO_CNTRL_REG (UART16550_BASEADDR + 0x1008)
#define LINE_STATUS_REG (UART16550_BASEADDR + 0x1014)
#define IER_REG (UART16550_BASEADDR + XUN_IER_OFFSET)
#define IIR_REG (UART16550_BASEADDR + XUN_IIR_OFFSET)
#define RX_BUF_REG (UART16550_BASEADDR + 0x1000)
#define SCRATCH_REG (UART16550_BASEADDR +  0x101C)

#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
#define SBUS_FRAME_LENGTH 25
#define SBUS_NUM_CHANNELS 18

static volatile int TotalReceivedCount;
static bool failsafe = false;
static int  lost = 0;

/*X8R Functions */
s32 parseSBUSData(uint8_t *rx_buffer, uint16_t *channels_buffer);
float mapSBUSToRange(uint16_t sbus_value, float min_output, float max_output);
/*UART Interrupt Set-Up Functions */
void UART550_Init(void);
void UARTInterruptHandler(void *CallBackRef, u32 Event, unsigned int EventData);
int SetupInterruptSystem(XScuGic *IntcInstancePtr, XUartNs550 *UART550InstancePtr, UINTPTR BaseAddress);