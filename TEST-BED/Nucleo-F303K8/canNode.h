#ifndef CANNODE
#define CANNODE

#include "mbed.h"
#include <string>
#include <queue>

/*Notes
    Check if a static member function can be defined to keep a list off all nodes (constructors could add themselves to the list?)
*/

class canNode {
    public:
        //static canNode* nodeList[16]; (static list produces compiler errors when trying to assign valus, global varaible used instead)
        canNode(string nodeName, CANMessage* sendBuffer, uint32_t sendBufferLen, uint32_t* sendId, uint32_t sendIdLen, CAN* sendDatabus);
        uint8_t getStatus() {return status;}
        string getNodeName() {return nodeName;}
        int32_t sendFirstMsg();            //sends first message from global CAN send buffer if it matches any of the IDs in sendId array    
    private:
        uint8_t status;
        string nodeName;
        CANMessage* sendBuffer;             //pointer to global buffer for messages to send
        uint32_t sendBufferLen;
        uint32_t* sendId;                   //list of node id:s to send from send buffer
        uint32_t sendIdLen;                 //lenght of sendId list
        CAN* sendDatabus;                   //pointer to databus used for sending messages
    };



#endif