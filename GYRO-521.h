#include <xstatus.h>
#include "xiicps.h"

/* GYRO MPU Defines */
#define GYRO_521_ADDR            0x68 // I2C address of the MPU-6050

/*GYRO MPU Prototypes */
s32 GY521_Init(XIicPs *IicInstance);
s32 GY521_ReadData(XIicPs *IicInstance, float *dataBuffer);

/*IIC_0 */
s32 IIC_PS_INIT(XIicPs *IicInstance, UINTPTR IICBaseAddr, u32 IIC_Frequency);