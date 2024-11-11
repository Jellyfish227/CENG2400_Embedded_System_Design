/*
### UART communication with bluetooth HC-05 and PC ###
UART0 used to communicate with PC
UART5 used to communicate with BLUETOOTH HC-05

Hardware connection:
RXD -> PE5
TXD -> PE4
GND -> GND
VCC -> VBUS

Steps:
1. Enable AT mode and configure the bluetooth module
    1.1 Connect bluetooth-EN to Tiva-VCC to enable AT mode. AT mode is correctly enabled if the LED blinks slowly (around 2-second period)
    1.2 Use the following AT commands: AT+UART? / AT+ROLE? / AT+ADDR? to check the current configuration
    1.3 Set the baud rate (AT+UART) of both the master and the slave 38400
    1.4 Set the role (AT+ROLE) of the master 1, the slave 0
    1.5 Enable the fixed-address mode (AT+CMODE) of the master
    1.6 Bind (AT+BIND) the destination of the master to the slave address (AT+ADDR?)
2. Disconnect bluetooth-EN and Tiva-VCC to disable AT mode
3. Complete, compile and run the code
*/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"


void PORTF_IRQHandler(void);

int main(void) {
    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // TODO: YOUR UART INITIALIZATION PROCEDURE
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

    // enable SW1 of GPIOF for button control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // set interrupt type for SW1 of GPIOF
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_BOTH_EDGES);
    // enable interrupt for SW1 of GPIOF
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
    // register interrupt handler for SW1 of GPIOF
    GPIOIntRegister(GPIO_PORTF_BASE, PORTF_IRQHandler);
    // enable interrupt master control
    IntMasterEnable();
}

void PORTF_IRQHandler(void)
{
    // Check if the interrupt was triggered by PD4
    if (GPIOIntStatus(GPIO_PORTF_BASE, true) & GPIO_INT_PIN_4)
    {
        char a;
        // Toggle the LED or perform your action
        if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4))
            a = 'Y';
        else
            a = 'X'; // Button pressed

        UARTCharPut(UART5_BASE, a);
    }
    // Clear the interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
}