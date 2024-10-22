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

int main(void) {
    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // TODO: YOUR UART INITIALIZATION PROCEDURE
    // enable UART5 and GPIOE to communicate with BLUETOOTH
    // configure PE4 for RX, PE5 for TX
    // set PORTE pin4 and pin5 as type UART
    // set UART5 base address, clock and baud rate

    // enable SW1 of GPIOF for button control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // TODO: YOUR BUTTON CONTROL PROCEDURE
    // you can do
    // polling (i.e., checking button status and send data in a while loop), or
    // interrupt (i.e., writing an interrupt function to check button status and send data)
}
