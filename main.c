#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_gpio.h"
int32_t ButtonState = 0;
uint32_t g_w_delay = 2000000;
uint32_t g_RGB_delay = 20000000;
uint8_t g_flash_LED = 0;

void read_Switches(void) {
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)) {
        g_flash_LED = 0;
    } else {
        g_flash_LED = 1;
    }
}

void RGB_FSM(void) {
    static enum {ST_RED, ST_GREEN, ST_BLUE, ST_OFF} next_state;
    if (g_flash_LED == 0) {
        switch (next_state) {
            case ST_RED:
                //Control_RGB_LEDs(1, 0, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
                SysCtlDelay(g_RGB_delay);
                next_state = ST_BLUE;
                break;
            case ST_BLUE:
                //Control_RGB_LEDs(0, 1, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_2);
                SysCtlDelay(g_RGB_delay);
                next_state = ST_GREEN;
                break;
            case ST_GREEN:
                //Control_RGB_LEDs(0, 0, 1);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_3);
                SysCtlDelay(g_RGB_delay);
                next_state = ST_RED;
                break;

            default:
                next_state = ST_RED;
                break;
        }
    }
}

void flash_FSM(void) {
    static enum {ST_WHITE, ST_BLACK} next_state;
    if (g_flash_LED == 1) {
        switch (next_state) {
            case ST_WHITE:
                //Control_RGB_LEDs(1, 1, 1);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
                SysCtlDelay(g_w_delay);
                next_state = ST_BLACK;
                break;
            case ST_BLACK:
                //Control_RGB_LEDs(0, 0, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
                SysCtlDelay(g_w_delay);
                next_state = ST_WHITE;
                break;
            default:
                next_state = ST_WHITE;
                break;
        }
    }
}

void flash(void) {
    read_Switches();
    flash_FSM();
    RGB_FSM();

}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
       GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
       //We use pin4 to control SW1
       GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
       //Since the button needs some sort of pull-up, we set pin4 as weak pull-up
       GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

       while (1) {
           flash();
       }
}
