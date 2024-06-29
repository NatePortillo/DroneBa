# DroneBa
FPGA-Based Drone Flight Controller and Localization
--------------------------------------------------------
STM32-src - Features code for RFID Reader, Servos, and Infrared Sensors. Used to help facilitate docking. FreeRTOS Based.

ESP32-src - Features RX UWB control that communicates to PynqZ1 over UART for closeness data. TX communicates to RX over BT when near a certain distance threshold. 

Vitis(Pynq Z1)-src - Features majority of sensor communication, flight controller functionality, and localization (still under development).

Vivado-src - Features the hardware block design and hardware constrains file.

3D-src - Features the drone and docking station CAD models (docking station under development).

-------------------------------------------------------
Hardware:
  Non-Comprehensive Hardware List:
  1) PynqZ1
  2) ESP32 UWB sensor modules (x2)
  3) NUCLEO-U5 (ARM based board)
  4) MPU-6050 9-Axis GRYO - (IIC-Based)
  5) RFID-RC522 - (IIC-Based)
  6) BMP390 Pressure+Temperature Sensor - (SPI-Based)
  7) VL530X Time-Of-Flight Sensor - (SPI-Based)
  8) X8R Reciever - (Proprietary UART Protocol. Hardware inverter needed)
  9) IR Receiver and Transmitter (x3)
  10) Servos (x3)
  11) Basic Drone Hardware - LiPo 3S 1500mAh Battery, 4 Brushless Motors, Power Distribution Board, 4 ESCs, RC Transmitter
