#include "CANCom.h"
#include "mbed.h"
#include "Types.h"
#include <cstdint>
#include <cstring>
#include <string.h>
#include <Globals.h>
#include "CANConfig.h"

void canRxIsr();


//### CAN #########################################
uint32_t canBufReadIndex;

//used to merge/split chars in candata and integer value.
union command {                 
    int16_t value;
    char data[2];
} canCommand;

bool canStatusReq=false;
//##################################################

//Constructor
CANCom::CANCom(){
    this->canBufReadIndex=0;
}

void CANCom::setup(){
    databus.attach(&canRxIsr,CAN::RxIrq);                                          
    //CAN bus filter
    //databus.filter(CanID::FILTER_HEARTBEAT, CanID::MASK_MSG, CANFormat::CANStandard, 0);
    //databus.filter(CanID::FILTER_ACU, CanID::MASK_NODE, CANFormat::CANStandard, 1);      
}

void canRxIsr(){
    while(databus.read(canBuf[canBufWriteIndex])) {
        canBufWriteIndex++;
        if (canBufWriteIndex==canBufSize)
            canBufWriteIndex=0;
        }
}

void CANCom::updateVariables(){
    //CAN
    //Decode incoming message
    if (canBufReadIndex != canBufWriteIndex) {  //Check if buffer indices are different which indicates that the position at the read index has been filled.
        switch (canBuf[canBufReadIndex].id) {


            case 50: //Command, read first two chars as a byte.
                canCommand.data[0]=canBuf[canBufReadIndex].data[0];
                canCommand.data[1]=canBuf[canBufReadIndex].data[1];
                break;

            case 51: //Command, read first char as a byte.
                canCommand.data[0]=canBuf[canBufReadIndex].data[0];
                NCU_angle = (uint8_t)canCommand.data[0];
                break;


            default : //No message
                break;
        }
        if(++canBufReadIndex == canBufSize) //Increment buffer index and check for 'wrap around'.
            canBufReadIndex = 0;
    }
}