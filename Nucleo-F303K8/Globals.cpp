#include "Globals.h"
#include "Config.h"
#include <mbed.h>
#include <string.h>
#include <vector>

//Settings variables
int32_t NCU_angle=0;
int32_t NCU_offset=0;
float NCU_left_limit=0.85;
float NCU_right_limit=0.25;
char set_NCU_offset=0x1;
char set_NCU_left_limit=0x2;
char set_NCU_right_limit=0x3;
uint8_t request_NCU_variables_ID = 62;
char request_NCU_variables_msg = 0x4;


//USB
BufferedSerial pc(USBTX, USBRX);

//CAN
CAN databus(CANRX, CANTX, 1000000);
Timer periodicCanMsgTimer;

//CAN RX
CANMessage canBuf[canBufSize];
uint32_t canBufWriteIndex=0;

uint8_t ledFlag = 0;