/*
    MARV Test Bed µC program
    *Main program for Nucleo-F303K8 controlling test bed hardware*

    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include "mbed.h"
#include "stdio.h"

#include "Globals.h"
#include "CANCom.h"
#include <vector>

#include "drivers.h"

/* Macro definitions */
DigitalOut led(LED1);



/* Main function */
int main()
{
    CANCom canRX;
    canRX.setup();

    //read out configuration of CAN master control register
    //uint32_t* CAN_MCR = (uint32_t*)0x40006400;
    //set retry  (ABOM bit in CAN_MCR) to let system reset TEC and REC after bus-off
    //*CAN_MCR |= 1U << 6;


    uint8_t temp_msg = 0;

    //databus.monitor(true);

    while(true) {
        canRX.updateVariables();

        led = ledFlag;

        printf("%i", ledFlag);

        // Is this how we send message? Maybe we think yes
        databus.write(CANMessage(1319, &temp_msg, 1)); 
        
        temp_msg++;
        thread_sleep_for(500);
    }

    return 0;
}