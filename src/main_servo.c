// hardware connection:
// servo red wire -> V Bus
// servo brown wire -> GND
// servo (pitch) orange wire -> PD0
// servo (yaw) orange wire -> PD1
// pitch: up-down, yaw: left-right

// slave, receive angular displacement, interrupt-driven
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"

float servo_pwm_freq = 50;
volatile float pitch_angle, yaw_angle, pitch_duty_cycle, yaw_duty_cycle;
int idx = 0;

// determine the duty cycle according to the desired angle
float angleToPWMDutyCycle(float angle)
{
    // angle (duty cycle): 0 (0.5ms/20ms), 90 (1.5ms/20ms), 180 (2.5ms/20ms)
    // angle to pulse width: pulse_width = angle / 90 + 0.5
    // pulse width to duty cycle: duty_cycle = pulse_width / period
    // valid angle range: 0-180
    return (angle / 90 + 0.5) / (1000 / servo_pwm_freq);
}

void UART5IntHandler(void);
void UART0IntHandler(void);

char charYaw[3], charPitch[3];
int degreeArr[3];
int prevAngle[2];
int processLock = 0;

int main()
{
    // set the system clock and the PWM clock
    // system clock frequency : PWM clock frequency = 64 : 1
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // enable module PWM1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlDelay(SysCtlClockGet() / 30); // avoid program overheat & logic issues
    // configure generator 0 of PWM1
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    // calculate the number of PWM instruction cycles in each PWM period
    uint32_t pwm_period = (SysCtlClockGet() / 64 / servo_pwm_freq);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, pwm_period);
    // enable the 0th and 1st outputs of PWM1
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    // PD0 and PD1 to send the signals to the servos
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);

    // enable UART5 and GPIOE to communicate with BLUETOOTH
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // configure PE4 for RX, PE5 for TX
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    // set PORTE pin4 and pin5 as type UART
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // set UART5 base address, clock and baud rate
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

        yaw_duty_cycle = angleToPWMDutyCycle(degreeArr[0]);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        pitch_duty_cycle = angleToPWMDutyCycle(degreeArr[1]);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 1000);
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

    // TODO: Test received data, design data receiving logic
    int isPitch = 0;
    // receive data from UART5
    while (UARTCharsAvail(UART5_BASE))
    {
        char b = UARTCharGet(UART5_BASE);
        degreeArr[0] = b;
        break;
    }
    while (UARTCharsAvail(UART5_BASE))
    {
        char b = UARTCharGet(UART5_BASE);
        degreeArr[1] = b;
        break;
    }
    // clear the interrupt signal
    UARTIntClear(UART5_BASE, ui32Status);
}
