/*
    Hardware drivers for MARV Test Bed
    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include <mbed.h>
#include <string.h>

#include "Servo.h"

extern Servo steering;
extern Servo drive;

extern float speed;
extern float turn;

void testDrive();