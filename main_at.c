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

void UART0IntHandler(void);
void UART5IntHandler(void);

int main(void) {
    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

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

    // enable UART5 and GPIOE to communicate with BLUETOOTH
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // configure PE4 for RX, PE5 for TX
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    // set PE4 and PE5 as type UART
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // set UART5 base address, clock and baud rate
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 38400,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // configure interrupts
    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART0_BASE, UART0IntHandler);
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART5_BASE, UART5IntHandler);

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

    while (true) {}
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
        UARTCharPut(UART5_BASE, a);
        UARTCharPut(UART0_BASE, a);
    }
}

// handler when Tiva receives data from UART5
void UART5IntHandler(void)
{
    // get interrupt status
    uint32_t ui32Status = UARTIntStatus(UART5_BASE, true);
    // clear the interrupt signal
    UARTIntClear(UART5_BASE, ui32Status);
    // receive data from UART5
    while (UARTCharsAvail(UART5_BASE))
    {
        // forward the characters from UART5 to UART0
        char b = UARTCharGet(UART5_BASE);
        UARTCharPut(UART0_BASE, b);
    }
}
