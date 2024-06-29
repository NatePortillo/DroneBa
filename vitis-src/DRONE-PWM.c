#include "DRONE-PWM.h"

/**
 * @brief Sets the duty cycle for the PWM (Pulse Width Modulation).
 *
 * This function configures the PWM with the specified period and pulse width, effectively
 * setting the duty cycle. It disables the PWM, reconfigures it with the new parameters, and
 * then re-enables it.
 *
 * @param TmrCtrInstancePtr Pointer to the XTmrCtr instance to be configured.
 * @param Period The total period of the PWM signal.
 * @param PulseWidth The pulse width of the PWM signal, which determines the duty cycle.
 *
 * @return void
 *
 * @note This function assumes that the XTmrCtr instance has already been initialized and configured.
 */
void PWM_SetDutyCycle(XTmrCtr *TmrCtrInstancePtr, u32 Period, u32 PulseWidth) {
    // Configure the PWM with the new duty cycle
    XTmrCtr_PwmDisable(TmrCtrInstancePtr);
    XTmrCtr_PwmConfigure(TmrCtrInstancePtr, Period, PulseWidth);
    XTmrCtr_PwmEnable(TmrCtrInstancePtr);
}

/**
 * @brief Initializes the PWM (Pulse Width Modulation) controller.
 *
 * This function initializes the PWM controller with the specified base address, period, and pulse width.
 * It performs the following steps:
 * 1. Initializes the timer counter instance with the provided base address.
 * 2. Performs a self-test on the timer counter instance to ensure it is functioning correctly.
 * 3. Configures the PWM with the specified period and pulse width.
 * 4. Enables the PWM.
 *
 * @param TmrCtrInstancePtr Pointer to the XTmrCtr instance to be initialized.
 * @param BaseAddr The base address of the PWM controller.
 * @param Period The total period of the PWM signal.
 * @param PulseWidth The pulse width of the PWM signal, which determines the duty cycle.
 *
 * @return XST_SUCCESS if the initialization is successful, XST_FAILURE otherwise.
 *
 * @note This function assumes that the necessary drivers and hardware are properly set up.
 */
s32 PWMInit(XTmrCtr *TmrCtrInstancePtr, UINTPTR BaseAddr,u32 Period, u32 PulseWidth){
	int Status;

    Status = XTmrCtr_Initialize(TmrCtrInstancePtr, BaseAddr);
    if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
    Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    XTmrCtr_PwmConfigure(TmrCtrInstancePtr, Period, PulseWidth);
    XTmrCtr_PwmEnable(TmrCtrInstancePtr);
    return XST_SUCCESS;
}
