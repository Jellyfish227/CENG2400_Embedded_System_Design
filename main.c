#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#define W_DELAY 150
#define RGB_DELAY 1500

volatile uint8_t g_flash_LED = 0;
bool timer1finish = 1;
bool timer0finish = 1;

uint32_t ui32Period;

void Timer1IntHandler(void);
void Timer0IntHandler(void);
void PORTF_IRQHandler(void);

void initialization(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOIntRegister(GPIO_PORTF_BASE, PORTF_IRQHandler);

    // Configure Timer1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
    IntEnable(INT_TIMER1A);
    TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1IntHandler);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    ui32Period = SysCtlClockGet() / 1000;
    TimerLoadSet(TIMER1_BASE, TIMER_A, RGB_DELAY * ui32Period);

    // Configure Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    IntEnable(INT_TIMER0A);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerLoadSet(TIMER0_BASE, TIMER_A, W_DELAY * ui32Period);

    IntMasterEnable();
}

// TODO: Using GPIO Interrupt to replace the function Read_Switches_Timer()
void PORTF_IRQHandler(void)
{
    // Check if the interrupt was triggered by PD4
    if (GPIOIntStatus(GPIO_PORTF_BASE, true) & GPIO_INT_PIN_4)
    {
        // Toggle the LED or perform your action
        if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4))
            g_flash_LED = 0; // Button released
        else
            g_flash_LED = 1; // Button pressed

    }
    // Clear the interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
}

void Timer1IntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    timer1finish = 1;
}

void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    timer0finish = 1;
}

void Flash_Timer(void)
{
    // TODO: Implement this function similar to RGB_Timer()
    static enum {W, W_wait, OFF, OFF_wait} next_state;
    if (g_flash_LED == 1) {
        switch (next_state) {
            case W:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
                timer0finish = 0;
                TimerEnable(TIMER0_BASE, TIMER_A);
                next_state = W_wait;
                break;
            case W_wait:
                if (timer0finish)
                    next_state = OFF;
                break;
            case OFF:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
                timer0finish = 0;
                TimerEnable(TIMER0_BASE, TIMER_A);
                next_state = OFF_wait;
                break;
            case OFF_wait:
                if (timer0finish)
                    next_state = W;
                break;
            default:
                next_state = W;
                break;
        }
    }
}

void RGB_Timer(void)
{
    if (g_flash_LED == 0)
    {
        static enum { R,
                      R_wait,
                      G,
                      G_wait,
                      B,
                      B_wait } next_state;
        switch (next_state)
        {
        case R:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);
            timer1finish = 0;
            TimerEnable(TIMER1_BASE, TIMER_A);
            next_state = R_wait;
            break;
        case R_wait:
            if (timer1finish)
                next_state = G;
            break;
        case G:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2);
            timer1finish = 0;

            TimerEnable(TIMER1_BASE, TIMER_A);
            next_state = G_wait;
            break;
        case G_wait:
            if (timer1finish)
                next_state = B;
            break;
        case B:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);
            timer1finish = 0;
            TimerEnable(TIMER1_BASE, TIMER_A);
            next_state = B_wait;
            break;
        case B_wait:
            if (timer1finish)
                next_state = R;
            break;
        default:
            next_state = R;
            break;
        }
    }
}

int main(void)
{
    initialization();
    g_flash_LED = 0;
    while (1)
    {
        RGB_Timer();
        Flash_Timer();
    }
}
