#ifndef CONFIG_H
#define CONFIG_H

//DEBUG
//----------------------------------------------------------------------------
//If defined the display is showed in terminal, preventing other debug messages
//#define DISPLAY_DEBUG
//If defined some simple debug messages are shown
#define SIMPLE_DEBUG
//----------------------------------------------------------------------------

//Units & Status
//----------------------------------------------------------------------------
#define nbrOfOtherUnitsInSystem 6
#define ACU_unit_ID             0
#define RCU_unit_ID             1
#define PDU_unit_ID             2
#define TCU_unit_ID             3
#define NCU_unit_ID             4
#define UCU_unit_ID             5
#define status_timeout          3 //seconds


//PINS
//----------------------------------------------------------------------------
//External mode activated/deativated indicator
#define STM_12_INDIC_pin        PA_1

#define CANRX                   PA_11
#define CANTX                   PA_12


//CAN COMMUNICATION
//----------------------------------------------------------------------------
#define canBufSize              32
#define canStatusReq_ID_ACU     1281 //Message ID 40 ACU
#define canStatusReq_ID_RCU     1282 //Message ID 40 RCU
#define canStatusReq_ID_PDU     1283 //Message ID 40 PDU
#define canStatusReq_ID_TCU     1284 //Message ID 40 TCU
#define canStatusReq_ID_NCU     1285 //Message ID 40 NCU
#define canStatusReq_ID_UCU     1286 //Message ID 40 UCU
#define canStatusReq_ID_OCU     1287 //Message ID 40 OCU
#define canStartScenario_ID     647  //Message ID 20 OCU
#define canStateScenario_ID     673  //Message ID 21 ACU
#define canSoundBuzzer_ID_OCU   711  //Message ID 22 OCU 
#define canACUPower_ID_OCU      583  //Message ID 18 OCU         
#define canPDUPower_ID_OCU      615  //Message ID 19 OCU
#define canLogData_ID           743  //Message ID 23 OCU
#define canLoggingMarker_ID     769  //Message ID 24 ACU
#define canLoggingState_ID      801  //Message ID 25 ACU
#define canSystemTime_ID        737  //Message ID 23 ACU
#define canScenarioConfig_ID    929  //Message ID 29 ACU
#define canNotifiMsg_ID         897  //Message ID 28 ACU
#define canEkfStatus_ID         833  //Message ID 26 ACU
#define canLog2PDU_ID           995  //Message ID 34 PDU
#define insStatus_ID            865  //Message ID 30 ACU
#define status12V_Auto_TEMP_ID  39   //Message ID 1 OCU


//Defines can notification message length
//----------------------------------------------------------------------------
#define notificationMessageLength 70

#endif //CONFIG_H