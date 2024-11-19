#include <stdbool.h>
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

void UART5IntHandler(void);
void UART0IntHandler(void);

char charYaw[3], charPitch[3];
int degreeArr[2];
int prevAngle[2];
prevAngle[0] = 0;
prevAngle[1] = 0;

int main()
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
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // register interrupt handler for UART5
    UARTIntRegister(UART5_BASE, UART5IntHandler);
    // enable interrupt for UART5
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
    // enable interrupt for UART5
    IntEnable(INT_UART5);

    // enable UART0 and GPIOA to communicate with PC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // configure PA0 for RX, PA1 for TX
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // set PA0 and PA1 as type UART
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // set UART0 base address, clock and baud rate
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART0_BASE, UART0IntHandler);
    // enable interrupt master control
    IntMasterEnable();

    // UART0 connection indicator
    // UART0 connected if the serial monitor displays `UART0 connected!`
    UARTCharPut(UART0_BASE, 'U');
    UARTCharPut(UART0_BASE, 'A');
    UARTCharPut(UART0_BASE, 'R');
    UARTCharPut(UART0_BASE, 'T');
    UARTCharPut(UART0_BASE, '0');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'C');
    UARTCharPut(UART0_BASE, 'o');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'c');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'd');
    UARTCharPut(UART0_BASE, '!');
    UARTCharPut(UART0_BASE, '\n');

    while (true)
    {
        // yaw_angle = 0;
        // yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        // SysCtlDelay(SysCtlClockGet() / 3);
        // yaw_angle = 90;
        // yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        // SysCtlDelay(SysCtlClockGet() / 3);
        // yaw_angle = 180;
        // yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        // SysCtlDelay(SysCtlClockGet() / 3);
        // yaw_angle = 90;
        // yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        // SysCtlDelay(SysCtlClockGet() / 3);
        // pitch_angle = 60;
        // pitch_duty_cycle = angleToPWMDutyCycle(pitch_angle);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
        // SysCtlDelay(SysCtlClockGet() / 3);
        // pitch_angle = 90;
        // pitch_duty_cycle = angleToPWMDutyCycle(pitch_angle);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
        // SysCtlDelay(SysCtlClockGet() / 3);
        degreeArr[0] = atoi(charYaw);
        degreeArr[1] = atoi(charPitch);
        if (degreeArr[0] != prevAngle[0] || degreeArr[1] != prevAngle[1]) {
            yaw_duty_cycle = angleToPWMDutyCycle(degreeArr[0]);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
            pitch_duty_cycle = angleToPWMDutyCycle(degreeArr[1]);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
            prevAngle[0] = degreeArr[0];
            prevAngle[1] = degreeArr[1];
        }
    }
}

// handler when Tiva receives data from UART0
void UART0IntHandler(void)
{
    // get interrupt status
    uint32_t ui32Status = UARTIntStatus(UART0_BASE, true);
    // clear the interrupt signal
    UARTIntClear(UART0_BASE, ui32Status);
    // receive data from UART0
    while (UARTCharsAvail(UART0_BASE))
    {
        // forward the characters from UART0 to UART5 and back to UART0
        char a = UARTCharGet(UART0_BASE);
        UARTCharPut(UART0_BASE, a);
    }
}

void UART5IntHandler(void)
{
    // get interrupt status
    uint32_t ui32Status = UARTIntStatus(UART5_BASE, true);
    // clear the interrupt signal
    UARTIntClear(UART5_BASE, ui32Status);

    uint32_t charCount = 0;
    // TODO: Test received data, design data receiving logic
    // receive data from UART5
    while (UARTCharsAvail(UART5_BASE))
    {
        char b = UARTCharGet(UART5_BASE);
        if (charCount < 3) {
            charYaw[charCount] = b;
        } else if (charCount < 6) {
            charPitch[charCount - 3] = b;
        }
        charCount++;
    }
}
