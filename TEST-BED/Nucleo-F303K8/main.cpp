/*
    MARV Test Bed µC program
    *Main program for Nucleo-F303K8 controlling test bed hardware*

    
    Written by Hugo Björk, Linus Johansson & Joakim Osterman in the spring of 2022
*/

#include "mbed.h"
#include "stdio.h"

#include "Servo.h"


/* Macro definitions */
#define BLINKING_RATE     500ms         // Blinking rate in milliseconds


/* Global variables */
uint32_t speed;
uint32_t angle;
float tempVal = 0.5;
float mid = 0;
float midAngle = 0.4;
float multiplier = 2.5;
float deg = 180 / multiplier;
float servoInc = 0.05;
float speedConv = 0.01;
float a = 0.5;


/* Global instances */
Servo turnServo(PB_5);
Servo DCMotor(PA_8);


/* Main function */
int main()
{
    DigitalOut led(LED1);

    while (true) {
        led = !led;
        ThisThread::sleep_for(BLINKING_RATE);
    }


    return 0;
}



/* Old (maybe) useful stuff from previous project to control motors

turnServo=midAngle;    


speed = (temp.data[0]*16*16+temp.data[1])/100; //Omvandla från hex till decimal

if(speed <= 100){
    tempVal=mid+speed*speedConv;
    DCMotor=tempVal;

case 2:

angle = (temp.data[0]*16*16+temp.data[1])/100; //Omvandla från hex till decimal


if(angle > 0 && angle < 327){
    tempVal = midAngle- angle/deg;
    turnServo=tempVal;
}
else if(angle > 200){
    angle= angle-655;
    tempVal=abs( int(angle));
    tempVal =  midAngle + tempVal/deg;
    turnServo=tempVal;
}
*/