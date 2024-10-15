#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#define RS_PIN GPIO_PIN_5                                                                                            // select pin 5 for RS
#define RW_PIN GPIO_PIN_6                                                                                            // select pin 6 for RS
#define EN_PIN GPIO_PIN_7                                                                                            // select pin 7 for EN
#define DB_PIN GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 // select pins 0~7 for DB
#define ROW GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
#define COL GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6

void flushInput(uint32_t ui32Port, uint8_t ui8Pins);
void delayUs(int n);
void LCD_command(bool rs, bool rw, unsigned char data);
char *message_str1 = "Please Enter:";
char *message_str2 = "                    ";
int flag;
int n;
int cursor_pos = 0xC0;
int inputEnable = 1;
int main(void)
{
begin:
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, DB_PIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, RS_PIN | RW_PIN | EN_PIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ROW);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, COL);
    GPIOPadConfigSet(GPIO_PORTC_BASE, COL, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    /* LCD configuration */
    LCD_command(0, 0, 0x38); // SET FUNCTION: specify 8-bit interface, 2 line display, and 5x7 dots (font)
    LCD_command(0, 0, 0x08); // DISPLAY OFF: set display off
    LCD_command(0, 0, 0x01); // DISPLAY CLEAR: set display clear
    LCD_command(0, 0, 0x06); // ENTRY MODE SET: set cursor move direction as decreasing & display is not shifted
    LCD_command(0, 0, 0x0F); // DISPLAY ON/OFF: set display on, cursor on, & cursor blinking on
    LCD_command(0, 0, 0x80); // SET DD DRAM ADDRESS: set cursor to the first line
    n = 0;
    while (message_str1[n] != '\0')
    {
        LCD_command(1, 0, message_str1[n]); // WRITE DATA: display message_str1[n] on the LCD
        n++;
    }
    LCD_command(0, 0, cursor_pos); // SET DD DRAM ADDRESS: set cursor to the first line

    while (1)
    {

        GPIOPinWrite(GPIO_PORTE_BASE, ROW, 0x1C); // first row
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++); // display behind "Please Enter:"
            LCD_command(1, 0, '1');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_4);
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '2');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_5);
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '3');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_6);
        }

        GPIOPinWrite(GPIO_PORTE_BASE, ROW, 0x1A); // second row
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '4');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_4);
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '5');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_5);
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '6');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_6);
        }

        GPIOPinWrite(GPIO_PORTE_BASE, ROW, 0x16); // third row
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '7');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_4);
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '8');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_5);
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '9');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_6);
        }

        GPIOPinWrite(GPIO_PORTE_BASE, ROW, 0x0E); // forth row
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) // *
        {
            LCD_command(0, 0, 0x01); // DISPLAY CLEAR: set display clear
            cursor_pos = 0xC0;
            inputEnable = 1;
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_5);
            goto begin;
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5) && inputEnable)
        {
            LCD_command(0, 0, cursor_pos++);
            LCD_command(1, 0, '0');
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_5);
        }
        else if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6))
        {
            LCD_command(0, 0, 0b01100); // DISPLAY ON/OFF: set display on, cursor on, & cursor blinking on
            inputEnable = 0;
            flushInput(GPIO_PORTC_BASE, GPIO_PIN_6);
        }

        if (cursor_pos > 0xCF) {
            cursor_pos = 0xC0;
        }
    }
}

void flushInput(uint32_t ui32Port, uint8_t ui8Pins)
{
    /* wait until the key is release to avoid redundant inputs. */
    while (!GPIOPinRead(ui32Port, ui8Pins))
    {
        delayUs(100000);
    }
}

void delayUs(int n)
{
    SysCtlDelay((n * 40) / 3);
}
void LCD_command(bool rs, bool rw, unsigned char data)
{
    if (rs == 0)                                     // L: Command H: Data
        GPIOPinWrite(GPIO_PORTA_BASE, RS_PIN, 0x00); // set RS as L
    else
        GPIOPinWrite(GPIO_PORTA_BASE, RS_PIN, 0x20); // set RS as H
    if (rw == 0)                                     // L: Write mode; H: Read mode
        GPIOPinWrite(GPIO_PORTA_BASE, RW_PIN, 0x00); // set RW as L
    else
        GPIOPinWrite(GPIO_PORTA_BASE, RW_PIN, 0x40); // set RW as H
    delayUs(1);
    GPIOPinWrite(GPIO_PORTA_BASE, EN_PIN, 0x80); // set H to enable signal EN
    GPIOPinWrite(GPIO_PORTB_BASE, DB_PIN, data); // assign DB0~DB7 with "data"
    delayUs(1);
    GPIOPinWrite(GPIO_PORTA_BASE, EN_PIN, 0x00); // set H->L to enable signal EN
    delayUs(1);
    if (rs == 0) // L: Command
    {
        if ((data == 0x01) | (data == 0x02) | (data == 0x03))
            delayUs(1640); // Clear Display & Display/Cursor Home take 1.64ms
        else
            delayUs(40); // all the others commands require only 40us to execute
    }
    else
        delayUs(40); // Data Write takes 40us to execute
}
