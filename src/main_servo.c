#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include <math.h>

#define FILTER_WINDOW_SIZE 10
#define GYRO_DEADZONE 0.2f
#define ACCEL_DEADZONE 0.15f

volatile bool g_bMPU6050Done;
tMPU6050 sMPU6050;
tI2CMInstance g_sI2CMSimpleInst;

float g_fYaw = 0.0f;                       // Yaw angle
float g_fPitch = 0.0f;                     // Pitch angle
float g_fRoll = 0.0f;                      // Roll angle
float g_fDeltaTime = 0.01f;                // 10ms sample time
float g_fComplementaryFilterCoeff = 0.96f; // Filter coefficient

float g_yawHistory[FILTER_WINDOW_SIZE] = {0};
float g_pitchHistory[FILTER_WINDOW_SIZE] = {0};
int g_yawFilterIndex = 0;
int g_pitchFilterIndex = 0;

// todo: add reset button for angle reset

//
// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
//
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        g_bMPU6050Done = false;
        return;
    }
    g_bMPU6050Done = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

void Initialization(void)
{
    // enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    // enable GPIO peripheral that contains I2C 0
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

    // clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());
    // Register the interrupt handler for I2C interrupts
    I2CIntRegister(I2C0_BASE, I2CMSimpleIntHandler);

    // Configure the MPU6050
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }
}

void computeAnglesFromAccel(float fAccel[3], float *pfPitch, float *pfRoll)
{
    // Convert accelerometer values to angles
    *pfRoll = atan2f(fAccel[1], fAccel[2]) * 180.0f / M_PI;
    *pfPitch = atan2f(-fAccel[0], sqrtf(fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2])) * 180.0f / M_PI;
}

void InitUART(void)
{
    // Enable UART5 and PORTE peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure GPIO Pins for UART5
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Configure UART5 for 115200 baud, 8N1 operation
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 38400,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}

void sendData(float yawAngle, float pitchAngle)
{
    char data[10]; // Increased buffer size for safety
    
    // Use more precise formatting
    snprintf(data, sizeof(data), "%03.0f%03.0f", yawAngle, pitchAngle);
    
    // Add error checking for UART transmission
    while(UARTBusy(UART5_BASE)) {}  // Wait if UART is busy
    
    char *chp = data;
    while (*chp)
    {
        while(UARTBusy(UART5_BASE)) {}  // Wait if UART is busy
        UARTCharPut(UART5_BASE, *chp++);
    }
    
    while(UARTBusy(UART5_BASE)) {}
    UARTCharPut(UART5_BASE, '\n');
}

float applyDeadZone(float value, float threshold) 
{
    if (fabs(value) < threshold) {
        return 0.0f;
    }
    return value;
}

float applyMovingAverageFilter(float newValue, float* history, int* index) {
    // Update history
    history[*index] = newValue;
    *index = (*index + 1) % FILTER_WINDOW_SIZE;
    
    // Calculate average
    float sum = 0;
    for(int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum += history[i];
    }
    return sum / FILTER_WINDOW_SIZE;
}

int main()
{
    // Set the system clock to use the PLL with a 16 MHz crystal oscillator.
    // The clock is divided by 1 (SYSCTL_SYSDIV_1) and uses an internal oscillator (SYSCTL_OSC_INT).
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    // Initialize UART before using it
    InitUART();

    // Initialize the system (e.g., peripherals, hardware components)
    Initialization();

    // Declare arrays to store accelerometer and gyroscope data
    float fAccel[3], fGyro[3];

    // Reset the MPU6050 sensor by writing to the power management register (PWR_MGMT_1)
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    // Configure the MPU6050 to not be low power mode by writing to the power management register (PWR_MGMT_2)
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    // Main infinite loop to repeatedly read data from the MPU6050
    while (1)
    {
        //
        // Request another reading from the MPU6050 sensor
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }

        // Get accelerometer and gyroscope data
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);

        // Apply dead zone to reduce noise
        for(int i = 0; i < 3; i++) {
            fGyro[i] = applyDeadZone(fGyro[i], GYRO_DEADZONE);
            fAccel[i] = applyDeadZone(fAccel[i], ACCEL_DEADZONE);
        }

        // Calculate angles from accelerometer
        float fAccPitch, fAccRoll;
        computeAnglesFromAccel(fAccel, &fAccPitch, &fAccRoll);

        // Complementary filter with reduced gyro influence when stationary
        g_fPitch = g_fComplementaryFilterCoeff * (g_fPitch + fGyro[0] * g_fDeltaTime) +
                   (1.0f - g_fComplementaryFilterCoeff) * fAccPitch;
        // Yaw can only be calculated from gyro (no gravity reference)
        g_fYaw += 180.0f * (fGyro[2] * g_fDeltaTime);

        // Normalize yaw to 0-180 degrees
        if (g_fYaw > 180.0f)
        {
            g_fYaw = 180.0f;
        }
        else if (g_fYaw < 0.0f)
        {
            g_fYaw = 0.0f;
        }

        // Normalize pitch to 30-90 degrees
        if (g_fPitch > 90.0f)
        {
            g_fPitch = 90.0f;
        }
        else if (g_fPitch < 30.0f)
        {
            g_fPitch = 30.0f;
        }

        // Add filtering:
        g_fYaw = applyMovingAverageFilter(g_fYaw, g_yawHistory, &g_yawFilterIndex);
        g_fPitch = applyMovingAverageFilter(g_fPitch, g_pitchHistory, &g_pitchFilterIndex);

        // Send the computed angles to the servo controller
        sendData(g_fYaw, g_fPitch);

        // Increase delay slightly to reduce sampling rate
        SysCtlDelay(SysCtlClockGet() / (3 * 50)); // Approximately 20ms delay
    }
}