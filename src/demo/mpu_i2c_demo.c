#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

double accelX = 0, accelY = 0, accelZ = 0;
double gyroX = 0, gyroY = 0, gyroZ = 0;
static uint8_t MPU6050_Buf_14_uint8[14];

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define CONFIG_ADDR  0x1B

void I2C_Write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t num, uint8_t* data)
{
    uint8_t count = 0;  // Initialize the counter

    // Set the I2C slave address for writing (false indicates a write operation)
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);

    // Load the register address into the data register
    I2CMasterDataPut(I2C0_BASE, reg_addr);

    // Send a start signal to prepare for data transmission (3 indicates a write operation)
    I2CMasterControl(I2C0_BASE, 3);

    // Wait until the I2C bus is not busy
    while (I2CMasterBusy(I2C0_BASE));

    // Loop to write data, excluding the last byte
    for (count = 0; count < num - 1; count++) {
        // Put the data byte into the data register
        I2CMasterDataPut(I2C0_BASE, data[count]);

        // Send the data byte (1 indicates send data)
        I2CMasterControl(I2C0_BASE, 1);

        // Wait until the I2C bus is not busy
        while (I2CMasterBusy(I2C0_BASE));
    }

    // Write the last data byte
    I2CMasterDataPut(I2C0_BASE, data[count]);

    // Send a stop signal to end the transmission (5 indicates send stop)
    I2CMasterControl(I2C0_BASE, 5);

    // Wait until the I2C bus is not busy
    while (I2CMasterBusy(I2C0_BASE));
}

void I2C_Read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t num, uint8_t* data)
{
    uint8_t count = 0;  // Initialize the counter

    // Set the I2C slave address for writing (false indicates a write operation)
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);

    // Load the register address into the data register
    I2CMasterDataPut(I2C0_BASE, reg_addr);

    // Send a start signal to initiate a read operation (7 indicates a read request)
    I2CMasterControl(I2C0_BASE, 7);

    // Wait until the I2C bus is not busy
    while (I2CMasterBusy(I2C0_BASE));

    // Set the I2C slave address for reading (true indicates a read operation)
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);

    // Send a start signal to prepare for reading data (11 indicates a repeated start)
    I2CMasterControl(I2C0_BASE, 11);

    // Wait until the I2C bus is not busy
    while (I2CMasterBusy(I2C0_BASE));

    // Read the first byte of data
    data[0] = I2CMasterDataGet(I2C0_BASE);

    // Loop to read the middle bytes
    for (count = 1; count < num - 1; count++) {
        // Control to read the next byte (9 indicates read and acknowledge)
        I2CMasterControl(I2C0_BASE, 9);

        // Wait until the I2C bus is not busy
        while (I2CMasterBusy(I2C0_BASE));

        // Store the received byte in the data array
        data[count] = I2CMasterDataGet(I2C0_BASE);
    }

    // Send a stop signal to end the transmission (5 indicates send stop)
    I2CMasterControl(I2C0_BASE, 5);

    // Wait until the I2C bus is not busy
    while (I2CMasterBusy(I2C0_BASE));

    // Read the last byte of data without acknowledging (no more data expected)
    data[count] = I2CMasterDataGet(I2C0_BASE);
}

// Initialize I2C
void Initialization(void) {
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.
    // Use the system clock for the I2C0 module.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    /*
     * Configure MPU6050
     */
     // Wake up from sleep & enable temperature reading
    uint8_t PWR_MGMT_1_value = 0x00;
    I2C_Write_bytes(MPU6050_ADDR, PWR_MGMT_1, 1, &PWR_MGMT_1_value);

    // Configure sensors' full scale range using Buffer
    // Read from register -> buffer
    I2C_Read_bytes(MPU6050_ADDR, CONFIG_ADDR, 2, MPU6050_Buf_14_uint8);

    // Set Gyro full scale range (FS_SEL)
    MPU6050_Buf_14_uint8[0] |= 1 << 3;

    // Set Accel full scale range (AFS_SEL)
    MPU6050_Buf_14_uint8[1] |= 1 << 3;

    // Write back from Buffer -> register
    I2C_Write_bytes(MPU6050_ADDR, CONFIG_ADDR, 2, MPU6050_Buf_14_uint8);
}

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    Initialization();

    while (1) {
        // Read raw data from MPU6050 and store in Buffer
        I2C_Read_bytes(104, 59, 14, MPU6050_Buf_14_uint8);

        // Extract data from Buffer and store in the following variables
        accelX = (MPU6050_Buf_14_uint8[0] << 8) | MPU6050_Buf_14_uint8[1];
        accelY = (MPU6050_Buf_14_uint8[2] << 8) | MPU6050_Buf_14_uint8[3];
        accelZ = (MPU6050_Buf_14_uint8[4] << 8) | MPU6050_Buf_14_uint8[5];
        gyroX = (MPU6050_Buf_14_uint8[8] << 8) | MPU6050_Buf_14_uint8[9];
        gyroY = (MPU6050_Buf_14_uint8[10] << 8) | MPU6050_Buf_14_uint8[11];
        gyroZ = (MPU6050_Buf_14_uint8[12] << 8) | MPU6050_Buf_14_uint8[13];

        // Adjust the delay as necessary
        SysCtlDelay(SysCtlClockGet() / 3000);
    }
}
