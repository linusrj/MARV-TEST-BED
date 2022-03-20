/*
    Hardware drivers for MARV Test Bed
    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include <mbed.h>
#include "Servo.h"

extern Servo drive;
extern Servo steering;

extern float speed;
extern float angle;

void car_init();
void testDrive();