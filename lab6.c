#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

uint32_t ui32ADC0Value[4];
volatile uint32_t ui32VolAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;

void ADC0IntHandler(void) {
    //
    // Clear the ADC interrupt flag
    //
    ADCIntClear(ADC0_BASE, 1);
    
    //
    // Get the ADC data
    //
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    
    //
    // Calculate average voltage value
    //
    ui32VolAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2) / 4;
    
    //
    // TODO: Convert the voltage to Celsius and Fahrenheit
    //

}

int main(void) {
    //
    // Set up the system clock
    //
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //
    // The ADC0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    //
    // Enable sample sequence 2 with a timer trigger. Sequence 2
    // will do 4 sampleS when the processor sends a singal to start the
    // conversion. Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3. This example is arbitrarily using sequence 2.
    //
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0-3 on sequence 2. Sample the temperature sensor
    // (ADC_CTL_TS) and configure the interrupt flag (ADC_CTL_IE) to be set
    // when the sample is done. Tell the ADC logic that this is the last
    // conversion on sequence 2 (ADC_CTL_END).  Sequence 2 and Sequence 1 have 4
    // programmable stepS. Sequence 3 has 1 programmable step, and sequence 0 has
    // 8 programmable steps. For more information on the
    // ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);

    //
    // Since sample sequence 2 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 1);
    
    //
    // TODO: Configure Timer to trigger the ADC
    //

    //
    // Register the ADC interrupt handler and enable ADC interrupt
    //
    ADCIntRegister(ADC0_BASE, 1, ADC0IntHandler);
    IntEnable(INT_ADC0SS1);
    ADCIntEnable(ADC0_BASE, 1);

    while(1) {
        //
        // Trigger the ADC conversion.
        // IMPORTANT: if you trigger the ADC conversion using Timer, there is no need to write this function.
        ADCProcessorTrigger(ADC0_BASE, 1);
        //
        SysCtlDelay(SysCtlClockGet() / 10); // 0.1s delay
    }
}
