#include "xaxidma.h"
#include "xparameters.h"
#include "xdebug.h"
#include "sleep.h"

#ifdef __aarch64__
#include "xil_mmu.h"
#endif

#if defined(XPAR_UARTNS550_0_BASEADDR)
#include "xuartns550_l.h"       /* to use uartns550 */
#endif

#if (!defined(DEBUG))
extern void xil_printf(const char *format, ...);
#endif

/******************** Constant Definitions **********************************/
/*
 * Device hardware build related constants.
 */
#ifndef SDT

#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID

#ifdef XPAR_AXI_7SDDR_0_S_AXI_BASEADDR
#define DDR_BASE_ADDR		XPAR_AXI_7SDDR_0_S_AXI_BASEADDR
#elif defined (XPAR_MIG7SERIES_0_BASEADDR)
#define DDR_BASE_ADDR	XPAR_MIG7SERIES_0_BASEADDR
#elif defined (XPAR_MIG_0_C0_DDR4_MEMORY_MAP_BASEADDR)
#define DDR_BASE_ADDR	XPAR_MIG_0_C0_DDR4_MEMORY_MAP_BASEADDR
#elif defined (XPAR_PSU_DDR_0_S_AXI_BASEADDR)
#define DDR_BASE_ADDR	XPAR_PSU_DDR_0_S_AXI_BASEADDR
#endif

#else

#ifdef XPAR_MEM0_BASEADDRESS
#define DDR_BASE_ADDR		XPAR_MEM0_BASEADDRESS
#endif
#endif

#ifndef DDR_BASE_ADDR
#warning CHECK FOR THE VALID DDR ADDRESS IN XPARAMETERS.H, \
DEFAULT SET TO 0x01000000
#define MEM_BASE_ADDR		0x01000000
#else
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x1000000)
#endif

#define TX_BD_SPACE_BASE	(MEM_BASE_ADDR)
#define TX_BD_SPACE_HIGH	(MEM_BASE_ADDR + 0x00000FFF)
#define RX_BD_SPACE_BASE	(MEM_BASE_ADDR + 0x00001000)
#define RX_BD_SPACE_HIGH	(MEM_BASE_ADDR + 0x00001FFF)
#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00100000)
#define RX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00300000)
#define RX_BUFFER_HIGH		(MEM_BASE_ADDR + 0x004FFFFF)


#define MAX_PKT_LEN		0x20
#define MARK_UNCACHEABLE        0x701

#define TEST_START_VALUE	0xC
#define POLL_TIMEOUT_COUNTER	1000000U

/**************************** Type Definitions *******************************/
u32 *Packet = (u32 *) TX_BUFFER_BASE;

/************************** Function Prototypes ******************************/
int RxSetup(XAxiDma *AxiDmaInstPtr);
int TxSetup(XAxiDma *AxiDmaInstPtr);
int SendDMAPacket(XAxiDma *AxiDmaInstPtr, u8 *Packet, u32 Length);
int CheckData(void);
int CheckDmaResult(XAxiDma *AxiDmaInstPtr);

int RxSetup(XAxiDma *AxiDmaInstPtr)
{
	XAxiDma_BdRing *RxRingPtr;
	int Delay = 0;
	int Coalesce = 1;
	int Status;
	XAxiDma_Bd BdTemplate;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	u32 BdCount;
	u32 FreeBdCount;
	UINTPTR RxBufferPtr;
	int Index;

	RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);

	/* Disable all RX interrupts before RxBD space setup */

	XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	/* Set delay and coalescing */
	XAxiDma_BdRingSetCoalesce(RxRingPtr, Coalesce, Delay);

	/* Setup Rx BD space */
	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
					RX_BD_SPACE_HIGH - RX_BD_SPACE_BASE + 1);

	Status = XAxiDma_BdRingCreate(RxRingPtr, RX_BD_SPACE_BASE,
				      RX_BD_SPACE_BASE,
				      XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);

	if (Status != XST_SUCCESS) {
		xil_printf("RX create BD ring failed %d\r\n", Status);

		return XST_FAILURE;
	}

	/*
	 * Setup an all-zero BD as the template for the Rx channel.
	 */
	XAxiDma_BdClear(&BdTemplate);

	Status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		xil_printf("RX clone BD failed %d\r\n", Status);

		return XST_FAILURE;
	}

	/* Attach buffers to RxBD ring so we are ready to receive packets */

	FreeBdCount = XAxiDma_BdRingGetFreeCnt(RxRingPtr);

	Status = XAxiDma_BdRingAlloc(RxRingPtr, FreeBdCount, &BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("RX alloc BD failed %d\r\n", Status);

		return XST_FAILURE;
	}

	BdCurPtr = BdPtr;
	RxBufferPtr = RX_BUFFER_BASE;
	for (Index = 0; Index < FreeBdCount; Index++) {
		Status = XAxiDma_BdSetBufAddr(BdCurPtr, RxBufferPtr);

		if (Status != XST_SUCCESS) {
			xil_printf("Set buffer addr %x on BD %x failed %d\r\n",
				   (unsigned int)RxBufferPtr,
				   (UINTPTR)BdCurPtr, Status);

			return XST_FAILURE;
		}

		Status = XAxiDma_BdSetLength(BdCurPtr, MAX_PKT_LEN,
					     RxRingPtr->MaxTransferLen);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx set length %d on BD %x failed %d\r\n",
				   MAX_PKT_LEN, (UINTPTR)BdCurPtr, Status);

			return XST_FAILURE;
		}

		/* Receive BDs do not need to set anything for the control
		 * The hardware will set the SOF/EOF bits per stream status
		 */
		XAxiDma_BdSetCtrl(BdCurPtr, 0);
		XAxiDma_BdSetId(BdCurPtr, RxBufferPtr);

		RxBufferPtr += MAX_PKT_LEN;
		BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, BdCurPtr);
	}

	/* Clear the receive buffer, so we can verify data
	 */
	memset((void *)RX_BUFFER_BASE, 0, MAX_PKT_LEN);

	Status = XAxiDma_BdRingToHw(RxRingPtr, FreeBdCount,
				    BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("RX submit hw failed %d\r\n", Status);

		return XST_FAILURE;
	}

	/* Start RX DMA channel */
	Status = XAxiDma_BdRingStart(RxRingPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("RX start hw failed %d\r\n", Status);

		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function sets up the TX channel of a DMA engine to be ready for packet
* transmission
*
* @param	AxiDmaInstPtr is the instance pointer to the DMA engine.
*
* @return	XST_SUCCESS if the setup is successful, XST_FAILURE otherwise.
*
* @note		None.
*
******************************************************************************/
int TxSetup(XAxiDma *AxiDmaInstPtr)
{
	XAxiDma_BdRing *TxRingPtr;
	XAxiDma_Bd BdTemplate;
	int Delay = 0;
	int Coalesce = 1;
	int Status;
	u32 BdCount;

	TxRingPtr = XAxiDma_GetTxRing(AxiDmaInstPtr);

	/* Disable all TX interrupts before TxBD space setup */

	XAxiDma_BdRingIntDisable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	/* Set TX delay and coalesce */
	XAxiDma_BdRingSetCoalesce(TxRingPtr, Coalesce, Delay);

	/* Setup TxBD space  */
	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
					TX_BD_SPACE_HIGH - TX_BD_SPACE_BASE + 1);

	Status = XAxiDma_BdRingCreate(TxRingPtr, TX_BD_SPACE_BASE,
				      TX_BD_SPACE_BASE,
				      XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {
		xil_printf("failed create BD ring in txsetup\r\n");

		return XST_FAILURE;
	}

	/*
	 * We create an all-zero BD as the template.
	 */
	XAxiDma_BdClear(&BdTemplate);

	Status = XAxiDma_BdRingClone(TxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		xil_printf("failed bdring clone in txsetup %d\r\n", Status);

		return XST_FAILURE;
	}

	/* Start the TX channel */
	Status = XAxiDma_BdRingStart(TxRingPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("failed start bdring txsetup %d\r\n", Status);

		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function transmits one packet non-blockingly through the DMA engine.
*
* @param	AxiDmaInstPtr points to the DMA engine instance
*
* @return	- XST_SUCCESS if the DMA accepts the packet successfully,
*		- XST_FAILURE otherwise.
*
* @note     None.
*
******************************************************************************/
int SendDMAPacket(XAxiDma *AxiDmaInstPtr, u8 *Packet, u32 Length)
{
    XAxiDma_BdRing *TxRingPtr;
    XAxiDma_Bd *BdPtr;
    int Status;

    TxRingPtr = XAxiDma_GetTxRing(AxiDmaInstPtr);

    /* Flush the buffers before the DMA transfer, in case the Data Cache
     * is enabled
     */
    Xil_DCacheFlushRange((UINTPTR)Packet, Length);

    /* Allocate a BD */
    Status = XAxiDma_BdRingAlloc(TxRingPtr, 1, &BdPtr);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Set up the BD using the information of the packet to transmit */
    Status = XAxiDma_BdSetBufAddr(BdPtr, (UINTPTR)Packet);
    if (Status != XST_SUCCESS) {
        xil_printf("Tx set buffer addr %x on BD %x failed %d\r\n",
                   (UINTPTR)Packet, (UINTPTR)BdPtr, Status);

        return XST_FAILURE;
    }

    Status = XAxiDma_BdSetLength(BdPtr, Length,
                                 TxRingPtr->MaxTransferLen);
    if (Status != XST_SUCCESS) {
        xil_printf("Tx set length %d on BD %x failed %d\r\n",
                   Length, (UINTPTR)BdPtr, Status);

        return XST_FAILURE;
    }

#ifndef SDT
#if (XPAR_AXIDMA_0_SG_INCLUDE_STSCNTRL_STRM == 1)
    Status = XAxiDma_BdSetAppWord(BdPtr,
                                  XAXIDMA_LAST_APPWORD, Length);

    /* If Set app length failed, it is not fatal
     */
    if (Status != XST_SUCCESS) {
        xil_printf("Set app word failed with %d\r\n", Status);
    }
#endif
#else
    if (TxRingPtr->HasStsCntrlStrm) {
        Status = XAxiDma_BdSetAppWord(BdPtr,
                                      XAXIDMA_LAST_APPWORD, Length);

        /* If Set app length failed, it is not fatal
         */
        if (Status != XST_SUCCESS) {
            xil_printf("Set app word failed with %d\r\n", Status);
        }
    }
#endif

    /* For single packet, both SOF and EOF are to be set
     */
    XAxiDma_BdSetCtrl(BdPtr, XAXIDMA_BD_CTRL_TXEOF_MASK |
                      XAXIDMA_BD_CTRL_TXSOF_MASK);

    XAxiDma_BdSetId(BdPtr, (UINTPTR)Packet);

    /* Give the BD to DMA to kick off the transmission. */
    Status = XAxiDma_BdRingToHw(TxRingPtr, 1, BdPtr);
    if (Status != XST_SUCCESS) {
        xil_printf("to hw failed %d\r\n", Status);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}


/*****************************************************************************/
/*
*
* This function checks data buffer after the DMA transfer is finished.
*
* @param	None
*
* @return	- XST_SUCCESS if validation is successful
*		- XST_FAILURE if validation is failure.
*
* @note		None.
*
******************************************************************************/
int CheckData(void)
{
	u8 *RxPacket;
	RxPacket = (u8 *) RX_BUFFER_BASE;

	/* Invalidate the DestBuffer before receiving the data, in case the
	 * Data Cache is enabled
	 */
	Xil_DCacheInvalidateRange((UINTPTR)RxPacket, 13);

	// Verify if the first byte is valid (as an example) //Start Byte Should be 0xAA and End Byte 0xBB
	if (RxPacket[0] != 0xAA || RxPacket[12] != 0xBB) {
		xil_printf("Data error: Unexpected value in received data\r\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function waits until the DMA transaction is finished, checks data,
* and cleans up.
*
* @param	None
*
* @return	- XST_SUCCESS if DMA transfer is successful and data is correct,
*		- XST_FAILURE if fails.
*
* @note		None.
*
******************************************************************************/
int CheckDmaResult(XAxiDma *AxiDmaInstPtr)
{
	XAxiDma_BdRing *TxRingPtr;
	XAxiDma_BdRing *RxRingPtr;
	XAxiDma_Bd *BdPtr;
	int ProcessedBdCount;
	int FreeBdCount;
	int Status;
	int TimeOut = POLL_TIMEOUT_COUNTER;

	TxRingPtr = XAxiDma_GetTxRing(AxiDmaInstPtr);
	RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);

	/*
	 * Wait until the one BD TX transaction is done or
	 * 1usec * 10^6 iterations of timeout occurs.
	 */
	while (TimeOut) {
		if ((ProcessedBdCount = XAxiDma_BdRingFromHw(TxRingPtr,
					XAXIDMA_ALL_BDS,
					&BdPtr)) != 0) {
			break;
		}
		TimeOut--;
		usleep(1U);
	}

	/* Free all processed TX BDs for future transmission */
	Status = XAxiDma_BdRingFree(TxRingPtr, ProcessedBdCount, BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Failed to free %d tx BDs %d\r\n",
			   ProcessedBdCount, Status);
		return XST_FAILURE;
	}

	TimeOut = POLL_TIMEOUT_COUNTER;

	/*
	 * Wait until the data has been received by the Rx channel or
	 * 1usec * 10^6 iterations of timeout occurs.
	 */
	while (TimeOut) {
		if ((ProcessedBdCount = XAxiDma_BdRingFromHw(RxRingPtr,
					XAXIDMA_ALL_BDS,
					&BdPtr)) != 0) {
			break;
		}
		TimeOut--;
		usleep(1U);
	}

	/* Check received data */
	if (CheckData() != XST_SUCCESS) {

		return XST_FAILURE;
	}

	/* Free all processed RX BDs for future transmission */
	Status = XAxiDma_BdRingFree(RxRingPtr, ProcessedBdCount, BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Failed to free %d rx BDs %d\r\n",
			   ProcessedBdCount, Status);
		return XST_FAILURE;
	}

	/* Return processed BDs to RX channel so we are ready to receive new
	 * packets:
	 *    - Allocate all free RX BDs
	 *    - Pass the BDs to RX channel
	 */
	FreeBdCount = XAxiDma_BdRingGetFreeCnt(RxRingPtr);
	Status = XAxiDma_BdRingAlloc(RxRingPtr, FreeBdCount, &BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("bd alloc failed\r\n");
		return XST_FAILURE;
	}

	Status = XAxiDma_BdRingToHw(RxRingPtr, FreeBdCount, BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Submit %d rx BDs failed %d\r\n", FreeBdCount, Status);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/* DMA INITIALIZATION */
s32 DMA_Init(XAxiDma *AxiDma, UINTPTR DMAPLAddr){
    s32 Status;
    XAxiDma_Config *Config;
    
	Config = XAxiDma_LookupConfig(DMAPLAddr);
	if (!Config) {
		xil_printf("No config found for %d\r\n", DMAPLAddr);
		return XST_FAILURE;
	}
	Status = XAxiDma_CfgInitialize(AxiDma, Config);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}
	if (!XAxiDma_HasSg(AxiDma)) {
		xil_printf("Device configured as Simple mode \r\n");
		return XST_FAILURE;
	}
	Status = TxSetup(AxiDma);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	Status = RxSetup(AxiDma);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
}
