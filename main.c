/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the
* I2C resource as Master using HAL APIs. The I2C master sends the
* command packets to the I2C slave to control an user LED.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "bme280_defs.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY        (1000UL)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)

/* I2C GPIO pins used for clock and data */
#define CYBSP_I2C_SCL P8_0
#define CYBSP_I2C_SDA P8_1


/*******************************************************************************
* Global Variables
*******************************************************************************/
/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;
static cyhal_i2c_t mI2C;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void bme280_intefrace_select(struct bme280_dev *, int8_t);


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*   1. I2C Master sends command packet to the slave
*   2. I2C Master reads the response packet to generate the next command
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cyhal_i2c_cfg_t mI2C_cfg;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize the retarget-io */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                  CY_RETARGET_IO_BAUDRATE);
    /* Retarget-io init failed. Stop program execution */
    handle_error(result);

    /* Free needed GPIO pins */
    result = cyhal_gpio_init(P8_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_OPENDRAINDRIVESHIGH, 1); // 8_1 already init
    cyhal_gpio_free(P8_1);
    result = cyhal_gpio_init(P5_0, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_OPENDRAINDRIVESHIGH, 1); // 8_1 already init
    cyhal_gpio_free(P8_0);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "HAL: I2C Master "
           "****************** \r\n\n");

    /* I2C Master configuration settings */
    printf(">> Configuring I2C Master..... ");
    mI2C_cfg.is_slave = false;
    mI2C_cfg.address = 0;
    mI2C_cfg.frequencyhal_hz = I2C_FREQ;

    /* Init I2C master */
    result = cyhal_i2c_init(&mI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    /* I2C master init failed. Stop program execution */
    handle_error(result);

    /* Configure I2C Master */
    result = cyhal_i2c_configure(&mI2C, &mI2C_cfg);
    /* I2C master configuration failed. Stop program execution */
    handle_error(result);

    printf("Done\r\n\n");

    /* Enable interrupts */
    __enable_irq();

    struct bme280_data comp_data;

    struct bme280_dev dev;
    bme280_intefrace_select(&dev, BME280_I2C_INTF);

    int8_t rslt;
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        printf("Device init failed.\r\n");
    }

    for (;;)
    {
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if (rslt != BME280_OK) {
            printf("Failed to get sensor data.\r\n");
        }
        print_sensor_data(&comp_data);
        /* /1* Give delay between commands *1/ */
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);

    }
}


bme280_delay_us_fptr_t device_delay_us(uint32_t period, void *intf_ptr) {
    /* cyhal function takes milliseconds.
     * 100_000 us = 100 ms */
    cyhal_system_delay_ms(period / 1000);
}

bme280_read_fptr_t device_mem_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {

    dev_addr = *(uint8_t *)intf_ptr;
    return (int8_t)cyhal_i2c_master_mem_read(&mI2C, dev_addr, reg_addr, 1, (uint8_t *)reg_data, (uint8_t *)len, 1);
}

bme280_write_fptr_t device_mem_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {

    dev_addr = *(uint8_t *)intf_ptr;
    return (int8_t)cyhal_i2c_master_mem_write(&mI2C, dev_addr, reg_addr, 1, (uint8_t *)reg_data, (uint8_t *)len, 1);
}

void bme280_intefrace_select(struct bme280_dev *dev, int8_t intf) {

    /* Add code for the case of using SPI interface if needed */
    if (intf == BME280_I2C_INTF) {
        dev_addr = BME280_I2C_ADDR_PRIM;
        dev->intf = BME280_I2C_INTF;
        dev->intf_ptr = &dev_addr;

        dev->read = device_mem_read;
        dev->write = device_mem_write;
        dev->delay_us = device_delay_us;
    } else {
        printf("Not I2C interface is chosen!\nAbort further execution.\r\n");
    }
}

void print_sensor_data(struct bme280_data *data) {
    printf("Temp: %f - Pressure: %f - Humidity: %f\r\n", data->humidity, data->pressure, data->temperature);
}
