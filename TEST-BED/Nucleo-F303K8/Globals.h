#ifndef GLOBALS_H
#define GLOBALS_H

//#include "lib.h" or <lib.h> 
#include "Config.h"
#include "mbed.h"
#include <string.h>

//Unit Variables
extern int32_t NCU_angle;
extern int32_t NCU_offset;
extern float NCU_left_limit;
extern float NCU_right_limit;
extern char set_NCU_offset;
extern char set_NCU_left_limit;
extern char set_NCU_right_limit;
extern uint8_t request_NCU_variables_ID;
extern char request_NCU_variables_msg;

extern BufferedSerial pc;

extern CAN databus;
extern Timer periodicCanMsgTimer;

extern CANMessage canBuf[canBufSize];
extern uint32_t canBufWriteIndex;


extern Timer timeLimitTimer;


extern char ACUreboot_msg;
extern char ACUshutdown_msg;
extern char DBWreboot_msg;
extern char SYSreboot_msg;
extern char SYSshutdown_msg;

extern char stateChange_msg;

extern uint8_t status_req_msg;
extern int32_t UNIT_status[nbrOfOtherUnitsInSystem];
extern time_t last_unit_update_time[nbrOfOtherUnitsInSystem];
extern bool unit_state_change_ER;
extern bool unit_state_change_NC;
extern bool onEnterCanMsgSend;
extern int32_t ekfStatus;
extern int32_t insStatus;


extern char notificationMessage[notificationMessageLength];
extern char notification_msg;
extern bool showNotificationPopup;

extern uint8_t status12V_Auto;

#endif //GLOBALS_H