/*GYROSCOPE - GY-521 */
#include <xstatus.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_io.h"
/* GYRO-521 Includes */
#include "GYRO-521.h"
/* IIC Include */
#include "xiicps.h"
/* UART Include */
#include "xuartps.h"
#include "ESP32Data.h"
/*BMP-390 includes */
#include "driver_bmp390_basic.h"
/*Timer Motor Control */
#include "xtmrctr.h"
#include "DRONE-PWM.h"
/* PL Processing Include */
#include "xaxidma.h"
#include "dma.h"
/* X8R Reciever */
#include "X8R_Recieve.h"
#include "xuartns550.h"
#include "xuartns550_l.h"
#include "xscugic.h"
#include "xinterrupt_wrap.h"
/* PID */
#include "PID.h"

/*-------------------IIC CONSTANTS----------------------------*/
XIicPs IicInstance; //Instance pointer for the PS IIC_0
XIicPs_Config *IICConfigPtr;
#define IIC_SCLK_RATE		100000
/*-----------------GY-521 CONSTANTS----------------------------*/
static float GY521_DataBuffer[6];
/*-------------------ESP32 CONSTANTS----------------------------*/
XUartPs UartInstance;
u8 ESP32_RecieveBuffer[13];
/*-----------------BMP390 CONSTANTS----------------------------*/
bmp390_interface_t SENSOR_COMMUNICATION_METHOD = BMP390_INTERFACE_IIC;
bmp390_address_t BMP390_SENSOR_ADDRESS = BMP390_ADDRESS_ADO_HIGH;
float temperature_c;
float pressure_pa;
/*-------------------PWM PROCESSING CONSTANTS----------------------------*/
XTmrCtr TimerCounter0; /* The instance of the Tmrctr Device */
XTmrCtr TimerCounter1; /* The instance of the Tmrctr Device */
XTmrCtr TimerCounter2; /* The instance of the Tmrctr Device */
XTmrCtr TimerCounter3; /* The instance of the Tmrctr Device */
/*-------------------PL PROCESSING CONSTANTS----------------------------*/
XAxiDma AxiDma;
/*-------------------X8R CONSTANTS----------------------------*/
volatile uint8_t rx_buffer[SBUS_FRAME_LENGTH];
uint16_t channels[SBUS_NUM_CHANNELS];
/*-------------------- UART550 Interrupt CONSTANTS --------------*/
XScuGic IntcInstance;
XUartNs550 UART550Instance;
/*-------------------- PID Constants ---------------------------*/
PIDController pid_roll;
PIDController pid_pitch;
PIDController pid_yaw;


/*---------MAIN---------*/
int main(void)
{
    s32 Status;
    /* IIC PS Initialization */
    Status = IIC_PS_INIT(&IicInstance, XPAR_XIICPS_0_BASEADDR, IIC_SCLK_RATE);
    if (Status != XST_SUCCESS) {
        xil_printf("IIC Initialization FAILED \r\n");
        return XST_FAILURE;
    }
    /* GYRO-521 Initialization */
    Status = GY521_Init(&IicInstance);
    if (Status != XST_SUCCESS) {
        xil_printf("GY-521 Initialization FAILED \r\n");
        return XST_FAILURE;
    }
    /* UART PS Initialization */
    UART_PS_Init(&UartInstance, XPAR_XUARTPS_1_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("UART Initialization FAILED \r\n");
        return XST_FAILURE;
    }
    /*BMP390 Initialization */
    uint8_t Result_Status = bmp390_basic_init(SENSOR_COMMUNICATION_METHOD, BMP390_SENSOR_ADDRESS);
    if(Result_Status != 0){
        xil_printf("BMP390 Initialization FAILED \r\n");
        return XST_FAILURE;
    }
    /* DMA Initialization */
    Status = DMA_Init(&AxiDma, XPAR_AXI_DMA_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("DMA Initialization FAILED\r\n");
        return XST_FAILURE;
    }
    /*UART550 Initialization */
    Status = SetupInterruptSystem(&IntcInstance, &UART550Instance, UART16550_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("UART Interrupt System Setup FAILED\r\n");
        return XST_FAILURE;
    }
    /*PWM Initialization (4 Hardware PWMs) */
    PWMInit(&TimerCounter0, TMRCTR_DEVICE_ID_0, PWM_PERIOD, 1000000); //109000 is the min. right now
    PWMInit(&TimerCounter1, TMRCTR_DEVICE_ID_1, PWM_PERIOD, 1000000); //109000 is the min. right now
    PWMInit(&TimerCounter2, TMRCTR_DEVICE_ID_2, PWM_PERIOD, 1000000); //109000 is the min. right now
    PWMInit(&TimerCounter3, TMRCTR_DEVICE_ID_3, PWM_PERIOD, 1000000); //109000 is the min. right now

    /*PID Initialization */
    PID_Init(&pid_roll, KPROPORTIONAL, KINTEGRAL, KDERIVITIVE, SET_POINT); // Tune these parameters
    PID_Init(&pid_pitch, KPROPORTIONAL, KINTEGRAL, KDERIVITIVE, SET_POINT);
    PID_Init(&pid_yaw, KPROPORTIONAL, KINTEGRAL, KDERIVITIVE, SET_POINT);

    while(1){
        s32 Status;

        /*
            GYRO Data Buffer Data Packet:
            dataBuffer[0] = Ax;
            dataBuffer[1] = Ay;
            dataBuffer[2] = Az;
            dataBuffer[3] = Gx;
            dataBuffer[4] = Gy;
            dataBuffer[5] = Gz;
        */
        GY521_ReadData(&IicInstance, GY521_DataBuffer);
        float roll = GY521_DataBuffer[3];  // Replace with actual index for roll
        float pitch = GY521_DataBuffer[4]; // Replace with actual index for pitch
        float yaw = GY521_DataBuffer[5];   // Replace with actual index for yaw
        float roll_output = PID_Compute(&pid_roll, roll);
        float pitch_output = PID_Compute(&pid_pitch, pitch);
        float yaw_output = PID_Compute(&pid_yaw, yaw);

        XUartNs550_Recv(&UART550Instance, rx_buffer, 25);
        Status = parseSBUSData(rx_buffer, channels);
        
        if(Status == XST_SUCCESS){
            float throttle = mapSBUSToRange(channels[0], 1000000, 2000000); // Example mapping
            float aileron = mapSBUSToRange(channels[1], 1000000, 2000000); //TODO: Change to different mapping
            float elevator = mapSBUSToRange(channels[2], 1000000, 2000000); //TODO: Change to different mapping
            float rudder = mapSBUSToRange(channels[3], 1000000, 2000000); //TODO: Change to different mapping
            
            /*NON-PID Conditions*/
            //PWM_SetDutyCycle(&TimerCounter0, PWM_PERIOD, throttle );
            //PWM_SetDutyCycle(&TimerCounter1, PWM_PERIOD, throttle );
            //PWM_SetDutyCycle(&TimerCounter2, PWM_PERIOD, throttle );
            //PWM_SetDutyCycle(&TimerCounter3, PWM_PERIOD, throttle );

            // Adjust motor signals based on PID outputs
            PWM_SetDutyCycle(&TimerCounter0, PWM_PERIOD, throttle + roll_output - pitch_output + yaw_output);
            PWM_SetDutyCycle(&TimerCounter1, PWM_PERIOD, throttle - roll_output - pitch_output - yaw_output);
            PWM_SetDutyCycle(&TimerCounter2, PWM_PERIOD, throttle + roll_output + pitch_output - yaw_output);
            PWM_SetDutyCycle(&TimerCounter3, PWM_PERIOD, throttle - roll_output + pitch_output + yaw_output);
        }


        //ESP32_Read(&UartInstance, ESP32_RecieveBuffer);
        bmp390_basic_read(&temperature_c, &pressure_pa);
        pressure_pa = pressure_pa / 100;

        //Status = SendDMAPacket(&AxiDma, ESP32_RecieveBuffer, 13);
        //if (Status != XST_SUCCESS) {
        //    xil_printf("DMA UART transfer to PL failed %d\r\n", Status);
        //}

        //Status = CheckDmaResult(&AxiDma);
        //if (Status != XST_SUCCESS) {
        //    xil_printf("AXI DMA SG Polling Example Failed\r\n");
        //}
    }
}