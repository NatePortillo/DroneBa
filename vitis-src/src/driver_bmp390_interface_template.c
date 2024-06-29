/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_bmp390_interface_template.c
 * @brief     driver bmp390 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-05-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/05/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_bmp390_interface.h"
#include "xiicps.h"
#include "xil_printf.h"
#include <stdarg.h>

#define BMP390_ADDR 0x77
#define CPU_CLOCK_HZ 100000000U // 50 MHz
#define CYCLES_PER_MS (CPU_CLOCK_HZ / 1000U)// Approximate number of iterations for 1 millisecond delay
extern XIicPs IicInstance;
/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t bmp390_interface_iic_init(void)
{
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t bmp390_interface_iic_deinit(void)
{   
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is the iic device write address
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp390_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    int Status;
    uint8_t reg_data = reg;

    // Check if the I2C bus is busy
    while (XIicPs_BusIsBusy(&IicInstance)) {
        // Optionally, add a delay or timeout
    }

    // Send the register address
    Status = XIicPs_MasterSendPolled(&IicInstance, &reg_data, 1, addr);
    if (Status != XST_SUCCESS) {
        return 1; // Return an error code if the send operation fails
    }

    // Wait for the bus to be free
    while (XIicPs_BusIsBusy(&IicInstance)) {
        // Optionally, add a delay or timeout
    }

    // Receive the data
    Status = XIicPs_MasterRecvPolled(&IicInstance, buf, len, addr);
    if (Status != XST_SUCCESS) {
        return 1; // Return an error code if the receive operation fails
    }

    return 0; // Return success if both operations are successful
}


/**
 * @brief     interface iic bus write
 * @param[in] addr is the iic device write address
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp390_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    int Status;
    uint8_t combined_buf[len + 1]; // Create a buffer to hold the register address and the data
    combined_buf[0] = reg;         // Set the first byte to the register address
    memcpy(&combined_buf[1], buf, len); // Copy the data to the buffer after the register address

    // Check if the I2C bus is busy
    while (XIicPs_BusIsBusy(&IicInstance)) {
        // Optionally, add a delay or timeout
    }

    // Send the combined buffer
    Status = XIicPs_MasterSendPolled(&IicInstance, combined_buf, len + 1, addr);
    if (Status != XST_SUCCESS) {
        return 1; // Return an error code if the write operation fails
    }

    return 0; // Return success if the write operation is successful
}


/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t bmp390_interface_spi_init(void)
{
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t bmp390_interface_spi_deinit(void)
{   
    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg is the register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp390_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg is the register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp390_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void bmp390_interface_delay_ms(uint32_t ms)
{
    uint32_t delay_cycles = ms * CYCLES_PER_MS;
    while (delay_cycles > 0) {
        delay_cycles--;
    }
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void bmp390_interface_debug_print(const char *const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    xil_printf(fmt, args);
    va_end(args);
}

/**
 * @brief     interface receive callback
 * @param[in] type is the interrupt type
 * @note      none
 */
void bmp390_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case BMP390_INTERRUPT_STATUS_FIFO_WATERMARK :
        {
            bmp390_interface_debug_print("bmp390: irq fifo watermark.\n");
            
            break;
        }
        case BMP390_INTERRUPT_STATUS_FIFO_FULL :
        {
            bmp390_interface_debug_print("bmp390: irq fifo full.\n");
            
            break;
        }
        case BMP390_INTERRUPT_STATUS_DATA_READY :
        {
            bmp390_interface_debug_print("bmp390: irq data ready.\n");
            
            break;
        }
        default :
        {
            bmp390_interface_debug_print("bmp390: unknown code.\n");
            
            break;
        }
    }
}
