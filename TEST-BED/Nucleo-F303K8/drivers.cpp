/*
    Hardware drivers for MARV Test Bed
    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include "drivers.h"

#include "Globals.h"
#include "Config.h"
#include <mbed.h>
#include <string.h>

#include "Servo.h"


/* Instances */
Servo steering(PB_5);
Servo drive(PA_8);


/* Variables */
float speed = 0.5;
float turn = 0.4;


/* Function declarations */
// Function for test driving hardware with a keyboard using console
void testDrive() {
    drive = speed;
    steering = turn;

    char msg[] = "Press 'u' to increase speed, 'd' to turn decrease\n";
    char *c = new char[1];
    pc.write(msg, sizeof(msg));

    while (1) {
        pc.read(c, sizeof(c));
        if (*c == 'w') {
            speed += 0.01;
            printf("Speed: %f \n", speed);
            drive = speed;
        }
        if (*c == 's') {
            speed -= 0.01;
            printf("Speed: %f \n", speed);
            drive = speed;
        }
        if (*c == 'a') {
            turn += 0.02;
            printf("Turn: %f \n", turn);
            steering = turn;
        }
        if (*c == 'd') {
            turn -= 0.02;
            printf("Turn: %f \n", turn);
            steering = turn;
        }
    }
}