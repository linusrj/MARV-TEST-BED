/*
    MARV Test Bed µC program
    *Main program for Nucleo-F303K8 controlling test bed hardware*

    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include "mbed.h"
#include "stdio.h"

#include "Globals.h"
#include "CANCom.h"

#include "drivers.h"


/* Main function */
int main()
{
    /* Setup */
    CANCom canRX;
    canRX.setup();
    //databus.monitor(true);

    //car_init();
    printf("Car ready..");


    while(true) {
        canRX.updateVariables();

        thread_sleep_for(1);
    }

    return 0;
}