/* 
    MARV Test Bed µC program

    This program is a further development of another bachelors project, original source: https://github.com/AlfredL-A/EENX-15-20-19
*/

/* Libraries */
#include "mbed.h"

#include "stdio.h"
#include <queue>

#include "Servo.h"
#include "canNode.h"


/* Macros */
#define LED_PIN     LED1


//header for RC-Car
DigitalOut          led(LED_PIN);
bool                LED;
bool                brake;
uint32_t            counter = 0;
uint32_t            speed;
uint32_t            angle;
uint32_t            value;
float tempVal = 0.5;
float mid=0;
float midAngle=0.4;
float multiplier=2.5;
float deg=180/multiplier;
float servoInc=0.05;
float speedConv=0.01;
float a=0.5;
Servo turnServo(PB_5);
//Servo brakeServo(PB_6);
Servo DCMotor(PA_8);


//constant variable dec:
uint32_t debugEnable = 0;             //verbose debug output enable
uint32_t canPrintEnableDB1 = 1;       //prints all incoming CAN messages on databus if enabled
uint32_t canPrintEnableDB2 = 0;
const uint32_t bufSizeDB1 = 5;

//global varaible dec:
CANMessage bufferDB1[bufSizeDB1];
CANMessage sendBufferDB1[bufSizeDB1];


std::queue<CANMessage> sendFifoDB1;
std::queue<CANMessage> sendFifoDB2;


//can node IDs - see document Databus 2 CAN messages for defenitions of each ID
uint32_t canIdNCU[] = {1, 33};
uint32_t canIdTCU[] = {0, 32};
uint32_t canIdUCU[] = {2, };

volatile uint32_t writeIndexDB1 = 0;
uint32_t readIndexDB1 = 0;

char bufferPcRx[64];
uint32_t pcRxWriteIndex = 0;
volatile uint32_t commandFlag = 0;
volatile uint32_t testCanFlag = 0;

//function protoypes
void canPrintMsg(CANMessage* msg, Serial* output);
void init();
void initNodes();
void enablePwr();
void disablePwr();
int readBoardTemp();
float readBoardVoltage();
void canRxIsrDB1();
void pcRxIsr();
void parseCommand();

const uint32_t bitrateDB1 = 1000000;

//pin defenitions
//CAN
CAN DB1(PA_11, PA_12,bitrateDB1);                                   // Use for RC-car    CAN Rx pin name, CAN Tx pin name

//USB serial port
Serial              pc(SERIAL_TX, SERIAL_RX);



int main() 
{   
    turnServo=midAngle;    
    init();
    //testkod för att printa CAN
    CANMessage test;
    test.id = 42;
    test.data[0] = 0xFF;
    test.data[1] = 0xAA;
    bufferDB1[0] = test;
    writeIndexDB1++;
    while(1) 
    {
        if(testCanFlag)
            for(int i = 0; i < 10; i++)
                canPrintMsg(&test, &pc);
            testCanFlag=0;
        //check if new msg is available in the DB1 buffer (if write and read index are different)
        if(readIndexDB1 != writeIndexDB1)
        {
                //canPrintMsg(bufferDB1 + readIndexDB1, &pc);
                //reset readIndex if outside bound
                if(readIndexDB1 == bufSizeDB1 - 1)
                {
                    readIndexDB1 = 0;    
                }
        }        
        if(commandFlag)
        {
            int i = 0;
            commandFlag = 0;
            do
            {
                pc.putc(bufferPcRx[i++]);
            } while(bufferPcRx[i] != 0);
            pc.printf("\n\r");
            parseCommand();
        }
    }
}

void moveTurnServo (float a)
{
  turnServo=a;
  //    wait(0.1);
}


void init() 
{
    pc.printf("RC-Car startup\n\r");
    //attach ISR for DB1 and DB2 as callback functions
    DB1.attach(&canRxIsrDB1,CAN::RxIrq);
    //attach ISR callback for serial port pc RX
    pc.attach(&pcRxIsr);
}


// ISR for reading can messages DB1
void canRxIsrDB1()
{
    CANMessage temp;
    DB1.read(temp);
    if (debugEnable)
    {
        pc.printf("Message rec. DB1 \n\r");
    }
    if (canPrintEnableDB1)
    {
        //canPrintMsg(&bufferDB1[prevWriteIndex], &pc);
        //canPrintMsg(&temp, &pc);    
    }
    //pc.printf("Message = %f\r\n",(temp.data[0]*16*16+temp.data[1])/100);
    switch (temp.id){  
        case 1:
            //Signalen som ska till TCU dvs Gas
            //temp.id = 32;
            
            speed = (temp.data[0]*16*16+temp.data[1])/100; //Omvandla från hex till decimal
            
            if(speed <= 100){
                //pc.printf("Speed  = %d ", speed);
                tempVal=mid+speed*speedConv;
                DCMotor=tempVal;
                //pc.printf("|| Signal  = %f\r\n", tempVal);
            }
        
            else
                pc.printf("Invalid speed value \n");
            
            break;
            
        case 2:
            //Signalen som ska till NCU dvs styrvinkel
            //temp.id = 33;
            
            angle = (temp.data[0]*16*16+temp.data[1])/100; //Omvandla från hex till decimal
            //pc.printf("Angle in deci  = %d\r\n", angle);
            
            
            if(angle > 0 && angle < 327){
                pc.printf("Angle  = %d", angle);
                tempVal = midAngle- angle/deg;
                //tempVal=midAngle + servoInc;
                //tempVal=1;
                pc.printf("|| Signal  = %f\r\n", tempVal);
                turnServo=tempVal;
            }
            else if(angle > 200){
                angle= angle-655;
                tempVal=abs( int(angle));
                pc.printf("Angle  = %d", angle);
                //pc.printf("Angle/deg  = %d\r\n", angle/deg);
                tempVal =  midAngle + tempVal/deg;
                //tempVal=midAngle - servoInc;
                //tempVal=0;
                pc.printf("|| Signal  = %f\r\n", tempVal);
                turnServo=tempVal;
            }
            else
                pc.printf("Invalid angle value \n");
            break;
            
        default:
            pc.printf("Invalid message ID recived on DB1");
    }
}


// Function for printing a CANMessage object to a Serial port object (passed to function by pointers)
void canPrintMsg(CANMessage* msg, Serial* output)
{
    if(msg->data[0] || msg->data[1]){
        output->printf("ID %d : ",msg->id);
        for(int i=0; i<msg->len ; i++)
        {
           output->printf("%02x ",msg->data[i]);
        }
        output->printf("\n\r");
        if (debugEnable)
        {
            pc.printf("canPrintMsg called \n\r");
        }
    }
}

void pcRxIsr()
{
    while(pc.readable())
    {
        char inputChar = pc.getc();
        if ( (inputChar == 0x0D) || (inputChar == 0x0A) || (inputChar == '\r' ))
        {
            //null terminate the buffer
            bufferPcRx[pcRxWriteIndex] = 0;
            //reset write index
            pcRxWriteIndex = 0;
            //set command available flag
            commandFlag = 1;
        }
        else
        {
            bufferPcRx[pcRxWriteIndex] = inputChar;
            if(pcRxWriteIndex < 64)
            {
                pcRxWriteIndex++;    
            }    
        }
    }
}



//sscanf paresCommand
//3s Command string - max 3 charachters
//2x hex formatted unsigned in (max one byte)

void parseCommand()
{
    char cmd[3];
    uint32_t d1 = 0, d2 = 0, d3 = 0, d4 = 0, d5 = 0, d6 = 0, d7 = 0, d8 = 0;
    int id = -1; //init to invalid value
    sscanf(bufferPcRx,"%3s%i%2x%2x%2x%2x%2x%2x%2x%2x",cmd, &id, &d1,&d2,&d3,&d4,&d5,&d6,&d7,&d8);
    string cmdString = cmd;
    //command switch statment
    if((cmdString == "DB1")) {
        if((id >= 0) && (id < 2048)) {  //check if id is in valid range
            pc.printf("Write to DB1:");
            CANMessage temp;
            temp.id = id;
            temp.data[0] = d1;
            temp.data[1] = d2;
            temp.data[2] = d3;
            temp.data[3] = d4;
            temp.data[4] = d5;
            temp.data[5] = d6;
            temp.data[6] = d7;
            temp.data[7] = d8;
            canPrintMsg(&temp, &pc);
            sendFifoDB1.push(temp);
            return;
        }
        else
        {
            pc.printf("Invaild ID\n\r");
            pc.printf("ID  = %d\r\n", id);
            return;   
        }
    }
    
    if((cmdString == "PD1")) {
        if(id) {
            canPrintEnableDB1 = 1;
            pc.printf("Can printing enabled on DB1\n\r");
            pc.printf("ID  = %d\r\n", id);
        }
        else {
            canPrintEnableDB1 = 0;
            pc.printf("Can priting disabled on DB1\n\r");    
        }
        return;
    }
    if((cmdString == "TE1")) {
        if(testCanFlag) {
            testCanFlag = 0;
            pc.printf("Stopping CANTest\n\r");
        }
        else {
            testCanFlag = 1;
            pc.printf("Starting CANTest\n\r"); 
        }
        return;
    }
    
    pc.printf("Invalid comamad\n\r");
}
