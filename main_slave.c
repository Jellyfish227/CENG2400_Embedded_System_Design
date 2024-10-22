/*
### UART communication with bluetooth HC-05 and PC ###
UART0 used to communicate with PC
UART5 used to communicate with BLUETOOTH HC-05
UART5IntHandler to handle interrupt

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

bool led_on = false;

void UART5IntHandler(void);

int main(void) {
    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // TODO: YOUR UART INITIALIZATION PROCEDURE
    // enable UART5 and GPIOE to communicate with BLUETOOTH
    // configure PE4 for RX, PE5 for TX
    // set PORTE pin4 and pin5 as type UART
    // set UART5 base address, clock and baud rate

    // enable LED of GPIOF for display
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // TODO: YOUR UART INTERRUPT INITIALIZATION PROCEDURE
    // set interrupt

    while (true)
    {
        if (led_on) {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 14);
        }
        else {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
        }
    }
}

// TODO: YOUR UART INTERRUPT HANDLER
// handler when Tiva receives data from UART5
void UART5IntHandler(void)
{
    // get interrupt status
    // clear the interrupt signal
    // receive data from UART5
    // set `led_on` according to the received data
}
