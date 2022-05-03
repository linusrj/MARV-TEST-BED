#include "CANCom.h"
#include "mbed.h"
#include "Types.h"
#include <cstdint>
#include <cstring>
#include <string.h>
#include <Globals.h>
#include "CANConfig.h"

#include "drivers.h"

void canRxIsr();

uint32_t canBufReadIndex;

int counter = 0;


CANCom::CANCom() {
    this->canBufReadIndex=0;
}

void CANCom::setup() {
    databus.attach(&canRxIsr, CAN::RxIrq);                                          
    //CAN bus filter, not used in this application
    //databus.filter(CanID::FILTER_HEARTBEAT, CanID::MASK_MSG, CANFormat::CANStandard, 0);
    //databus.filter(CanID::FILTER_ACU, CanID::MASK_NODE, CANFormat::CANStandard, 1);      
}


void canRxIsr() {
    counter = 0;
    while(databus.read(canBuf[canBufWriteIndex])) {
        canBufWriteIndex++;
        if (canBufWriteIndex == canBufSize) {
            canBufWriteIndex = 0;
        }
    }
}

void CANCom::updateVariables() {
    if(canBufReadIndex != canBufWriteIndex) {  //Check if buffer indices are different which indicates that the position at the read index has been filled.
        switch (canBuf[canBufReadIndex].id) {

            /* ID 33 is for cmdSteering from the ACU, aka REACH */
            case 33:
                //printf("Forward: %d\nAngle: %d\n\n", canBuf[canBufReadIndex].data[1], canBuf[canBufReadIndex].data[5]);
                
                /* We will only be using the forward and angle values, found in the first and fifth elements */
                speed = (float)canBuf[canBufReadIndex].data[1] / 100;
                angle = (float)canBuf[canBufReadIndex].data[5] / 100;
                
                drive = speed;
                steering = angle;
    
                break;


            default: //No message
                break;
        }
        if(++canBufReadIndex == canBufSize) //Increment buffer index and check for 'wrap around'.
            canBufReadIndex = 0;
    }

    if (counter > 300) {
        drive = 0.5;
    }
    else {
        counter++;
    }
}
