//put your definition here
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

// Heartbeat message status
#define heartbeat_id = 0x701
#define Initialization = 0x00
#define Pre_Operational = 0x01 
#define Operational; = 0x02

// Charge Command
#define incoming_id = 0x201 
#define stop = 0x00
#define start = 0x01

// Charging Status
#define outgoing_id = 0x181
#define not_charging = 0x00
#define charging = 0x01

// variables and status
bool enable_command;
uint8_t chargingState;
uint8_t networkManagementState;
uint16_t current_reference; 
uint16_t current_feedback; 
uint16_t current_minimum;
uint16_t voltage_reference;
uint16_t voltage_feedback;

// time 
uint32_t time_ms = 0;               //time counter every 1ms 
uint32_t heartbeatTimer;
uint32_t incomingMsgTimer;
uint32_t outgoingMsgTimer = 0;

// CAN struct 
typedef struct 
{
    uint8_t Data[8];
    uint16_t Length;
    uint32_t ID;
} CAN_msg_typedef;

CAN_msg_typedef Can_tx;
CAN_msg_typedef Can_rx;

typedef enum
{
    INITIALIZATION = 0,
    PRE_OPERATIONAL, 
    OPERATIONAL
} NETWORK_STATE

typedef enum
{
    IDLE = 0,
    CONSTANT_CURRENT,
    CONSTANT_VOLTAGE
} CHARGING_STATE

void CAN_write(CAN_msg_typedef *msg);
bool CAN_read(CAN_msg_typedef *msg); //return true if there is received msg

void Initialization(void)
{
    //initialize your variables here
    chargingState = IDLE;
    networkState = INITIALIZATION;
    enable_command = false;
}

void control_routine(void)
{
    //run the control algorithm here
    // 
    // 
    time_ms++; //assume INT frequency is 1kHz, for timing purpose
}


// PI Controller API
void PI_current_control(void)
{
    do
    {
        // runs PI current control to regulate current feedback
    } 
    while (current_feedback != current_reference);
}

void PI_voltage_control(void)
{
     do
    {
        // runs PI voltage control to regulate current feedback
    } 
    while (voltage_feedback != voltage_reference);
}

// Timer
bool IsTimePassed(uint32_t *timerCounter, uint32_t time)
{
    if((time_ms - *timerCounter) >= time)
    {
        *timerCounter = time_ms;
        return true;
    }
    else
    {
        return false;
    }
}

void build_heartbeat_message(uint8_t state)
{
    Can_tx.ID = heartbeat_id; 
    Can_tx.Length = 1; 
    Can_tx.Data[0] = state;
}

void build_outgoing_message(uint8_t status)
{
    Can_tx.ID = outgoing_id;
    Can_tx.Length = 4; 
    Can_tx.Data[0] = (uint8_t)(voltage_feedback);
    Can_tx.Data[1] = (uint8_t)(voltage_feedback >> 8);
    Can_tx.Data[2] = (uint8_t)(current_feedback);
    Can_tx.Data[3] = (uint8_t)(current_feedback >> 8);
    Can_tx.Data[4] = status;
}

void parse_incoming_message()
{
    uint8_t charge_command;
    voltage_reference = (uint16_t)(Can_rx.Data[0]);
    voltage_reference |= (uint16_t)(Can_rx.Data[1] << 8);
    current_reference = (uint16_t)(Can_rx.Data[2]);
    current_reference |= (uint16_t)(Can_rx.Data[3] << 8);
    charge_command = Can_rx.Data[4];
    if (charge_command == start)
    {
        enable_command = true;
    }
    else if (charge_command == stop)
    {
        enable_command = false;
    }
}


void main_state_machine(void)
{
    //run the state transition here
    switch(chargingState)
    {
        case IDLE: 
            if (enable_command == true)
            {
                chargingState = CONSTANT_CURRENT;
            }
            break;
        
        case CONSTANT_CURRENT:
            
            PI_current_control();
            if (voltage_feedback == voltage_reference)
            {
                chargingState = CONSTANT_VOLTAGE;
            }
            break;
        
        case CONSTANT_VOLTAGE:
            PI_voltage_control();
            if(current_feedback == minimum_current)
            {
                chargingState = IDLE;
                enable_command = false; 
            }
            break;
        
        default:
            break;
    }
}

void network_management(void)
{
    // check if 1s has paased
    if(IsTimePassed(&heartbeatTimer, 1000))
    {
        // send heartbeat message
        build_heartbeat_message(networkState);
        CAN_write(Can_tx);
        heartbeatTimer = time_ms;
    }
    
    switch(networkState)
    {
        case INITIALIZATION:
            //wait device boot up and initialization
            // send heartbeat message
            build_heartbeat_message(Initialization);
            CAN_write(Can_tx);
            // start heartbeat message time counter
            heartbeatTimer = time_ms;
            networkState = PRE_OPERATIONAL;
            break;
        
        case PRE_OPERATIONAL:
            // check incoming message
            if(CAN_read(Can_rx) == true)
            {
                parse_incoming_message();
                if(enable_command == true)
                {
                    networkState = OPERATIONAL;
                }
                // start incoming message time counter
                incomingMsgTimer = time_ms;
            }
            break;

        case OPERATIONAL:
            // check outgoing message time counter
            if(outgoingMsgTimer == 0)
            {
                // send first outgoing message
                build_outgoing_message(charging);
                CAN_write(Can_tx);
                outgoingMsgTimer = time_ms;
            }
            else
            {
                if(IsTimePassed(&outgoingMsgTimer, 200))
                {
                    uint8_t chargingStatus;
                    // send outgoing message
                    if (enable_command == true)
                    {
                        chargingStatus = charging;
                    }
                    else
                    {
                        chargingStatus = not_charging;
                    }
                    build_outgoing_message(chargingStatus);
                    CAN_write(Can_tx);
                    outgoingMsgTimer = time_ms;
                }   
            }

            // check incoming message
            if(CAN_read(Can_rx) == true)
            {
                parse_incoming_message();
                if(enable_command == false)
                {
                    outgoingMsgTimer == 0;
                    networkState = PRE_OPERATIONAL;
                }
                // restart incoming meesage time counter
                incomingMsgTimer = time_ms;
            }
            else
            {
                // check if 5s has passed without incoming message
                if(IsTimePassed(&incomingMsgTimer, 5000))
                {
                    incomingMsgTimer = 0;
                    outgoingMsgTimer == 0;
                    networkState = PRE_OPERATIONAL;
                } 
            }
            break;
    }
}

void main(void)
{
    Initialization();
    PieVectTable.EPWM1_INT = &control_routine;

    while(true)
    {
        main_state_machine();
        network_management();
    }
}