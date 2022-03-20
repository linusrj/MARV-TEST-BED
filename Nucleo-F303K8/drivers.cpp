/*
    Hardware drivers for MARV Test Bed
    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include <mbed.h>
#include "drivers.h"

#include <string.h>

#include "Globals.h"
#include "Config.h"
#include "Servo.h"


/* Instances */
Servo drive(PA_8);
Servo steering(PB_5);


/* Variables */
float speed = 0.5;  // Values in middle of range, 0-1
float angle = 0.5;   // Tuned in car_init()


/* Function declarations */

// We want the car to tell the ACU that it is ready to drive, and a response from the ACU that it knows this
void car_init() {
    // Tune values to stand still with wheels facing forward
    speed = 0.5;
    angle = 0.4;

    // Calibrate servo instances?
    //drive.calibrate();
    //steering.calibrate();

    // Set motor speed and servo angle
    drive = speed;
    steering = angle;


    // Tell ACU that car is ready
    //uint8_t temp_msg = 1;
    //databus.write(CANMessage(69, &temp_msg, 1));

    // Wait for ACU to send message back that it knows car is ready
    //while(no response);
}



// Function for test driving hardware with a keyboard using console
void testDrive() {
    drive = speed;
    steering = angle;

    char *c = new char[1];

    while(1) {
        pc.read(c, sizeof(c));
        if(*c == 'w') {
            speed += 0.01;
            printf("Speed: %f \n", speed);
            drive = speed;
        }
        if(*c == 's') {
            speed -= 0.01;
            printf("Speed: %f \n", speed);
            drive = speed;
        }
        if(*c == 'a') {
            angle += 0.02;
            printf("Turn: %f \n", angle);
            steering = angle;
        }
        if(*c == 'd') {
            angle -= 0.02;
            printf("Turn: %f \n", angle);
            steering = angle;
        }
    }
}