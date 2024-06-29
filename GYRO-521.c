#include "GYRO-521.h"
/**
 * @brief Initializes the GY-521 sensor by configuring its registers.
 *
 * This function initializes the GY-521 sensor (also known as the MPU-6050) by configuring 
 * its power management, sample rate, gyroscope, and accelerometer settings. The function 
 * sends the necessary configuration data to the respective registers using the I2C protocol.
 *
 * @param IicInstance Pointer to the IIC instance to be used for communication.
 *
 * @return XST_SUCCESS if initialization is successful, XST_FAILURE otherwise.
 *         The function also prints specific error messages if any configuration step fails.
 *
 * @note The function waits for the I2C bus to be free before sending each configuration 
 *       command. Optionally, a delay or timeout can be added in the while loops to handle
 *       bus busy situations more gracefully.
 */
s32 GY521_Init(XIicPs *IicInstance) {
    s32 Status;
    const uint8_t PWR_MGMT_1[2] = {0x6B, 0x00}; // PWR_MGMT_1 register, Wake-Up of IMU
    const uint8_t SMPLRT_DIV[2] = {0x19, 0x07}; // SMPLRT_DIV register, Wake-Up of IMU  
    const uint8_t GYRO_CONFIG[2] = {0x1B, 0x10}; // Full Scale Range (default +/- 250deg/s)
    const uint8_t ACCEL_CONFIG[2] = {0x1C, 0x10}; // Full Scale Range (default +/- 2g) 
    // Check if the I2C bus is busy
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }
    Status = XIicPs_MasterSendPolled(IicInstance, PWR_MGMT_1, 2, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("PWR_MGMT_1 FAILED\r\n");
        return XST_FAILURE;
    }

    /*
    // Wait for the bus to be free
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }
    Status = XIicPs_MasterSendPolled(IicInstance, SMPLRT_DIV, 2, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("SMPLRT_DIV FAILED\r\n");
        return XST_FAILURE;
    }
    */

    // Wait for the bus to be free
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }
    Status = XIicPs_MasterSendPolled(IicInstance, GYRO_CONFIG, 2, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("SMPLRT_DIV FAILED\r\n");
        return XST_FAILURE;
    }

    // Wait for the bus to be free
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }
    Status = XIicPs_MasterSendPolled(IicInstance, ACCEL_CONFIG, 2, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("SMPLRT_DIV FAILED\r\n");
        return XST_FAILURE;
    }
    
    return XST_SUCCESS;
}
/**
 * @brief Reads accelerometer and gyroscope data from the GY-521 sensor.
 *
 * This function reads the accelerometer and gyroscope data from the GY-521 sensor
 * using the I2C protocol and stores the converted values in the provided data buffer.
 *
 * @param IicInstance Pointer to the IIC instance to be used for communication.
 * @param dataBuffer Pointer to an array of floats where the sensor data will be stored.
 *                   The array must have space for at least 6 floats:
 *                   dataBuffer[0] = Accelerometer X-axis (g)
 *                   dataBuffer[1] = Accelerometer Y-axis (g)
 *                   dataBuffer[2] = Accelerometer Z-axis (g)
 *                   dataBuffer[3] = Gyroscope X-axis (degrees/second)
 *                   dataBuffer[4] = Gyroscope Y-axis (degrees/second)
 *                   dataBuffer[5] = Gyroscope Z-axis (degrees/second)
 *
 * @return XST_SUCCESS if successful, XST_FAILURE otherwise.
 */
s32 GY521_ReadData(XIicPs *IicInstance, float *dataBuffer) {
    s32 Status;
    uint8_t ACCEL_XOUT[6];
    uint8_t ACCEL_REG[1] = {0x3B}; // Starting with register 0x3B (ACCEL_XOUT_H)

    uint8_t GYRO_REG[1] = {0x43}; 
    uint8_t GYRO_XOUT[6];
    // Check if the I2C bus is busy
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }
    Status = XIicPs_MasterSendPolled(IicInstance, ACCEL_REG, 1, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    // Wait for the bus to be free
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }
    Status = XIicPs_MasterRecvPolled(IicInstance, ACCEL_XOUT, 6, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    // Check if the I2C bus is busy
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }

    int16_t accelerometer_x = (ACCEL_XOUT[0] << 8) | ACCEL_XOUT[1];
    int16_t accelerometer_y = (ACCEL_XOUT[2] << 8) | ACCEL_XOUT[3];
    int16_t accelerometer_z = (ACCEL_XOUT[4] << 8) | ACCEL_XOUT[5];
    float Ax = accelerometer_x/16384.0;  // get the float g
    float Ay = accelerometer_y/16384.0;
    float Az = accelerometer_z/16384.0;
    
    Status = XIicPs_MasterSendPolled(IicInstance, GYRO_REG, 1, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    // Wait for the bus to be free
    while (XIicPs_BusIsBusy(IicInstance)) {
        // Optionally, add a delay or timeout
    }
    Status = XIicPs_MasterRecvPolled(IicInstance, GYRO_XOUT, 6, GYRO_521_ADDR);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    int16_t gyro_x = (int16_t)(GYRO_XOUT[0] << 8 | GYRO_XOUT [1]);
    int16_t gyro_y = (int16_t)(GYRO_XOUT[2] << 8 | GYRO_XOUT [3]);
    int16_t gyro_z = (int16_t)(GYRO_XOUT[4] << 8 | GYRO_XOUT [5]);
    float Gx = gyro_x/131.0;
    float Gy = gyro_y/131.0;
    float Gz = gyro_z/131.0;

    //Corrected GYRO values:
    Gx = Gx + 0.56;

    // Store the values in the dataBuffer
    dataBuffer[0] = Ax;
    dataBuffer[1] = Ay;
    dataBuffer[2] = Az;
    dataBuffer[3] = Gx;
    dataBuffer[4] = Gy;
    dataBuffer[5] = Gz;
    
    return XST_SUCCESS;
}

/**
 * @brief Initializes the IIC (I2C) interface.
 *
 * This function initializes the IIC (I2C) interface on a specified base address and sets
 * the desired IIC clock frequency. It performs the following steps:
 * 1. Looks up the configuration for the IIC device based on the provided base address.
 * 2. Initializes the IIC instance using the retrieved configuration.
 * 3. Performs a self-test on the IIC instance to ensure it is functioning correctly.
 * 4. Sets the IIC serial clock rate to the specified frequency.
 *
 * @param IicInstance Pointer to the XIicPs instance to be initialized.
 * @param IICBaseAddr The base address of the IIC device.
 * @param IIC_Frequency The desired IIC serial clock frequency in Hz.
 *
 * @return XST_SUCCESS if the initialization is successful, XST_FAILURE otherwise.
 *
 * @note This function prints an error message if setting the IIC clock rate fails.
 */
s32 IIC_PS_INIT(XIicPs *IicInstance, UINTPTR IICBaseAddr, u32 IIC_Frequency) {  
    s32 Status;
    XIicPs_Config *IICConfigPtr;  /* Pointer to configuration data */
    
    IICConfigPtr = XIicPs_LookupConfig(IICBaseAddr);
    if (IICConfigPtr == NULL) {
        return XST_FAILURE;
    }
    Status = XIicPs_CfgInitialize(IicInstance, IICConfigPtr, IICConfigPtr->BaseAddress); /* Initialize the IIC */
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    Status = XIicPs_SelfTest(IicInstance); /* Self-test for IIC */
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    Status = XIicPs_SetSClk(IicInstance, IIC_Frequency); /* Set the IIC serial clock rate. */
    if (Status != XST_SUCCESS) {
        xil_printf("IIC setClock FAILED \r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}