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

// determine the duty cycle according to the desired angle
float angleToPWMDutyCycle(float angle)
{
    // angle (duty cycle): 0 (0.5ms/20ms), 90 (1.5ms/20ms), 180 (2.5ms/20ms)
    // angle to pulse width: pulse_width = angle / 90 + 0.5
    // pulse width to duty cycle: duty_cycle = pulse_width / period
    // valid angle range: 0-180
    return (angle / 90 + 0.5) / (1000 / servo_pwm_freq);
}

char const START_INDICATOR = 254;
char const END_INDICATOR = 255;

void clearArray(int arr[4]) {
    int i = 0;
    while (i < 4) {
        arr[i] = 0;
        i++;
    }
}

int receiveArr[4];
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

    int isValid = 0;
    int idx = 0;
    char b = '\0';

    while (true)
    {
        // empty receive buffer
        clearArray(receiveArr);

        // receive data to buffer
        idx = 0;
        while (UARTCharsAvail(UART5_BASE) || b != END_INDICATOR)
        {
            b = UARTCharGet(UART5_BASE);
        set:
            receiveArr[idx] = b;
            idx = (idx + 1) % 4;
        }

        // check data validity
        if (receiveArr[0] == START_INDICATOR && receiveArr[3] == END_INDICATOR) {
            isValid = 1;
        } else {
            while (UARTCharsAvail(UART5_BASE))
            {
                char temp = UARTCharGet(UART5_BASE);
                if (temp == START_INDICATOR) {
                    idx = 0;
                    clearArray(receiveArr);
                    goto set;
                }
            }
        }

        // write to servo if valid
        if (isValid) {
            yaw_duty_cycle = angleToPWMDutyCycle(receiveArr[1]);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
            pitch_duty_cycle = angleToPWMDutyCycle(receiveArr[2] % 90);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
            isValid = 0;    // reset valid state
        }
        SysCtlDelay(SysCtlClockGet() / 500);
    }
}
