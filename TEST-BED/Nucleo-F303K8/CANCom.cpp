#include "CANCom.h"
#include "mbed.h"
#include "Types.h"
#include <cstdint>
#include <cstring>
#include <string.h>
#include <Globals.h>
#include "CANConfig.h"

void canRxIsr();

//Variables
uint8_t canScenarioID = 0;
uint8_t canLocation = 0;
uint8_t idAndLocation = 0;
uint8_t scenarioHandlerState = 0;

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
    //can bus filter
    databus.filter(CanID::FILTER_HEARTBEAT, CanID::MASK_MSG, CANFormat::CANStandard, 0);
    databus.filter(CanID::FILTER_ACU, CanID::MASK_NODE, CANFormat::CANStandard, 1);      
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

            case canStateScenario_ID: //Command, read first char as a byte.
                scenarioHandlerState = canBuf[canBufReadIndex].data[0];
                if(!(scenarioHandlerState == STOPPED && scenarioState == REQ_SENT)){
                    scenarioState = (uint8_t)scenarioHandlerState;
                }

                //Send sound signal to buzzer, indicating that the scanrio state is in fault or finished state
                if(scenarioHandlerState == FAULT && !faultStateSignalSent){
                    faultStateSignalSent = true;
                     databus.write(CANMessage(canSoundBuzzer_ID_OCU,&faultStateBuzzer_msg,1));
                     printf("Fault SENT\n");
                }
                else if(scenarioHandlerState == FINISHED && !finishedStateSignalSent){
                    finishedStateSignalSent = true;
                     databus.write(CANMessage(canSoundBuzzer_ID_OCU,&finishedStateBuzzer_msg,1));
                     printf("Finished SENT\n");
                }
                else if(scenarioHandlerState == STOPPED){
                    faultStateSignalSent = false;
                    finishedStateSignalSent = false;
                    //printf("RESET\n");
                }
                break;

            case canSystemTime_ID:
                systemTime = (uint32_t)canBuf[canBufReadIndex].data[3];
                systemTime |= (uint32_t)canBuf[canBufReadIndex].data[2] << 8;
                systemTime |= (uint32_t)canBuf[canBufReadIndex].data[1] << 16;
                systemTime |= (uint32_t)canBuf[canBufReadIndex].data[0] << 24;
                break;

            case canStatusReq_ID_ACU:
                statusUpdate(&UNIT_status[ACU_unit_ID],(int8_t)canBuf[canBufReadIndex].data[0]);
                last_unit_update_time[ACU_unit_ID] = time(NULL);
                break;

            case canStatusReq_ID_RCU:
                statusUpdate(&UNIT_status[RCU_unit_ID],(int8_t)canBuf[canBufReadIndex].data[0]);
                last_unit_update_time[RCU_unit_ID] = time(NULL);
                break;

            case canStatusReq_ID_PDU:
                statusUpdate(&UNIT_status[PDU_unit_ID],(int8_t)canBuf[canBufReadIndex].data[0]);
                last_unit_update_time[PDU_unit_ID] = time(NULL);
                break;

            case canStatusReq_ID_TCU:
                statusUpdate(&UNIT_status[TCU_unit_ID],(int8_t)canBuf[canBufReadIndex].data[0]);
                last_unit_update_time[TCU_unit_ID] = time(NULL);
                break;

            case canStatusReq_ID_NCU:
                statusUpdate(&UNIT_status[NCU_unit_ID],(int8_t)canBuf[canBufReadIndex].data[0]);
                last_unit_update_time[NCU_unit_ID] = time(NULL);
                break;

            case canStatusReq_ID_UCU:
                statusUpdate(&UNIT_status[UCU_unit_ID],(int8_t)canBuf[canBufReadIndex].data[0]);
                last_unit_update_time[UCU_unit_ID] = time(NULL);
                break;

            case canLoggingState_ID:
                loggingState = (int8_t)canBuf[canBufReadIndex].data[0];
                break;

            case canLoggingMarker_ID:
                loggingMarker = (int8_t)canBuf[canBufReadIndex].data[0];
                break;


            case canNotifiMsg_ID: 
                if((canLocation*7 <= notificationMessageLength) && showNotificationPopup == false){
                    canLocation = (uint8_t)canBuf[canBufReadIndex].data[0];
                    memcpy(&notificationMessage[canLocation*7],canBuf[canBufReadIndex].data+1, 7);
                    printf("ID: %d\n",canLocation);
                    //Last pack of data recived, show popup
                    if(canLocation == (notificationMessageLength/7) - 1){
                        showNotificationPopup = true;
                        printf("notification: true\n");
                    }
                }
                break;

            case insStatus_ID:
                insStatus = (int8_t)canBuf[canBufReadIndex].data[0];
                break;

            case canEkfStatus_ID: //Update INS EKF status variable
                ekfStatus = (int8_t)canBuf[canBufReadIndex].data[0];
                break;

            default : //Not a message to OCU
                break;
        }
        if(++canBufReadIndex == canBufSize) //Increment buffer index and check for 'wrap around'.
            canBufReadIndex = 0;
    }
}

void CANCom::statusUpdate(int32_t* UNIT_status_var,uint8_t data){
    if(data >= 0 && data <=2){
        if(!(data == *UNIT_status_var)){
            *UNIT_status_var = data;
            if(data == ER_state){
                unit_state_change_ER = true;
            }
        }
    }
}