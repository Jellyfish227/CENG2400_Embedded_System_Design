// hardware connection:
// servo red wire -> V Bus
// servo brown wire -> GND
// servo (pitch) orange wire -> PD0
// servo (yaw) orange wire -> PD1
// pitch: up-down, yaw: left-right

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"

float servo_pwm_freq = 50;

// determine the duty cycle according to the desired angle
float angleToPWMDutyCycle(float angle)
{
    // angle (duty cycle): 0 (0.5ms/20ms), 90 (1.5ms/20ms), 180 (2.5ms/20ms)
    // angle to pulse width: pulse_width = angle / 90 + 0.5
    // pulse width to duty cycle: duty_cycle = pulse_width / period
    // valid angle range: 0-180
    return (angle / 90 + 0.5) / (1000 / servo_pwm_freq);
}

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

    float pitch_angle, yaw_angle, pitch_duty_cycle, yaw_duty_cycle;

    while (true)
    {
        yaw_angle = 0;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        yaw_angle = 90;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        yaw_angle = 180;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        yaw_angle = 90;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        pitch_angle = 60;
        pitch_duty_cycle = angleToPWMDutyCycle(pitch_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        pitch_angle = 90;
        pitch_duty_cycle = angleToPWMDutyCycle(pitch_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
    }
}
