#ifndef GLOBALS_H
#define GLOBALS_H

//#include "lib.h" or <lib.h> 
#include "Config.h"
#include "mbed.h"
#include <cstdint>
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

extern uint8_t ledFlag;

extern CANMessage canBuf[canBufSize];
extern uint32_t canBufWriteIndex;

extern Timer timeLimitTimer;

#endif //GLOBALS_H