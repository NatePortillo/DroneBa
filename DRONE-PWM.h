#include "xtmrctr.h"
#include <xstatus.h>

#define PWM_PERIOD 20000000 // 20 ms (50 Hz for ESCs)
#define TMRCTR_DEVICE_ID_0 XPAR_XTMRCTR_0_BASEADDR
#define TMRCTR_DEVICE_ID_1 XPAR_XTMRCTR_1_BASEADDR
#define TMRCTR_DEVICE_ID_2 XPAR_XTMRCTR_2_BASEADDR
#define TMRCTR_DEVICE_ID_3 XPAR_XTMRCTR_3_BASEADDR

s32 PWMInit(XTmrCtr *TmrCtrInstancePtr, UINTPTR BaseAddr,u32 Period, u32 PulseWidth);
void PWM_SetDutyCycle(XTmrCtr *TmrCtrInstancePtr, u32 Period, u32 PulseWidth);