/*
    MARV Test Bed µC program
    *Main program for Nucleo-F303K8 controlling test bed hardware*

    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include "mbed.h"
#include "stdio.h"

#include "Servo.h"


/* Macro definitions */
#define BLINKING_RATE     5ms         // Blinking rate in milliseconds


/* Global variables */
float speed = 0.5;
float midAngle = 0.4;


/* Global instances */
Servo turnServo(PB_5);
Servo DCMotor(PA_8);

static BufferedSerial pc(USBTX, USBRX);


void testRange() {
    float speed = 0.5;
    float turn = 0.4;

    DCMotor = speed;
    turnServo = turn;

    char msg[] = "Press 'u' to increase speed, 'd' to turn decrease\n";
    char *c = new char[1];
    pc.write(msg, sizeof(msg));

    while (1) {
        pc.read(c, sizeof(c));
        if (*c == 'w') {
            speed += 0.01;
            printf("Speed: %f \n", speed);
            DCMotor = speed;
        }
        if (*c == 's') {
            speed -= 0.01;
            printf("Speed: %f \n", speed);
            DCMotor = speed;
        }
        if (*c == 'a') {
            turn += 0.02;
            printf("Turn: %f \n", turn);
            turnServo = turn;
        }
        if (*c == 'd') {
            turn -= 0.02;
            printf("Turn: %f \n", turn);
            turnServo = turn;
        }
    }
}


/* Main function */
int main()
{
    testRange();

    return 0;
}