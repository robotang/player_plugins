/*
 *  A plugin for interfacing a custom built robot to Player
 *
 *  Copyright (C) 2011, Robert Tang <opensource@robotang.co.nz>
 *
 *  This is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public Licence
 *  along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
 
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include <libplayercore/playercore.h>

extern "C"
{
#include "devbot_protocol.h"
}

#include "devbot_player_position2d.h"
#include "devbot_player_imu.h"
#include "devbot_player_power.h"
#include "devbot_player_joystick.h"

#define SERIAL_RX_BUFFER_LENGTH         (MAX_MESSAGE_LENGTH * (MESSAGE_RX_BUFFER_SIZE + 1))
#define SERIAL_TX_BUFFER_LENGTH         (MAX_MESSAGE_LENGTH * (MESSAGE_TX_BUFFER_SIZE + 1))

//Local variables (need to be read and written by both the devbot_protocol C and devbot C++ modules)
static char serial_rx_data[SERIAL_RX_BUFFER_LENGTH];
static char serial_tx_data[SERIAL_TX_BUFFER_LENGTH];

char *receive_serial(void);
void transmit_serial(char *message);

using namespace std;
class devbot : public ThreadedDriver
{
public:
    devbot(ConfigFile *cf, int section);

    virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data);

private:
    virtual void Main(void);
    virtual int MainSetup(void);
    virtual void MainQuit(void);

    void WriteData(void);

    player_devaddr_t position2d_addr;
    player_devaddr_t imu_addr;
    player_devaddr_t power_addr;
    player_devaddr_t out_opaque_addr;
    player_devaddr_t opaque_addr;
    Device *opaque_dev;
    player_devaddr_t joystick_addr;
    
    bool has_joystick;
    bool has_imu;
    bool has_sonar;
    bool has_power;
    
    player_position2d_geom_t robot_geom;
};

//Initialisation function
Driver* devbot_Init(ConfigFile *cf, int section)
{
    return ((Driver*) (new devbot(cf, section)));
}

//Driver registration function
void devbot_Register(DriverTable *table)
{
    table->AddDriver("devbot", devbot_Init);
}

extern "C"
{
    int player_driver_init(DriverTable *table)
    {
        devbot_Register(table);
        return(0);
    }
}

devbot::devbot(ConfigFile *cf, int section) 
    : ThreadedDriver(cf, section)
{
    if(cf->ReadDeviceAddr(&(this->opaque_addr), section, "requires", PLAYER_OPAQUE_CODE, -1, NULL) != 0)
    {
        PLAYER_ERROR("[devbot]: failed to setup input opaque driver");
        this->SetError(-1);
        return;
    }
    
    //Create position interface
    if(cf->ReadDeviceAddr(&(this->position2d_addr), section, "provides", PLAYER_POSITION2D_CODE, -1, NULL) != 0)
    {
        PLAYER_ERROR("[devbot]: devbot must provide a position2d interface");
        this->SetError(-1);
        return;
    }    
    if(this->AddInterface(this->position2d_addr))
    {
        PLAYER_ERROR("[devbot]: failed to register devbot position2d interface");
        this->SetError(-1);        
        return;
    }
    
    //Create imu interface (optional)
    if(cf->ReadDeviceAddr(&(this->imu_addr), section, "provides", PLAYER_IMU_CODE, -1, NULL) == 0)
    {
        if(this->AddInterface(this->imu_addr))
        {
            PLAYER_ERROR("[devbot]: failed to register devbot imu interface");
            this->SetError(-1);        
            return;
        }
        has_imu = true;
    }   
    else
        has_imu = false;

    
    //Create power interface (optional)
    if(cf->ReadDeviceAddr(&(this->power_addr), section, "provides", PLAYER_POWER_CODE, -1, NULL) == 0)
    {
        if(this->AddInterface(this->power_addr))
        {
            PLAYER_ERROR("[devbot]: failed to register devbot power interface");
            this->SetError(-1);        
            return;
        }
        has_power = true;
    } 
    else   
        has_power = false;
    
    //Create opaque position
    if(cf->ReadDeviceAddr(&(this->out_opaque_addr), section, "provides", PLAYER_OPAQUE_CODE, -1, NULL) != 0)
    {
        PLAYER_ERROR("[devbot]: devbot must provide an output opaque interface");
        this->SetError(-1);
        return;
    }    
    if(this->AddInterface(this->out_opaque_addr))
    {
        PLAYER_ERROR("[devbot]: failed to register devbot output opaque interface");
        this->SetError(-1);
        return;
    }
    
    //Create joystick interface (optional)
    if(cf->ReadDeviceAddr(&(this->joystick_addr), section, "provides", PLAYER_JOYSTICK_CODE, -1, NULL) == 0)
    {
        if(this->AddInterface(this->joystick_addr))
        {
            PLAYER_ERROR("[devbot]: failed to register devbot joystick interface");
            this->SetError(-1);        
            return;
        }
        has_joystick = true;
    }    
    else
        has_joystick = false;
    
    //Read parameters    
    robot_geom.size.sw = cf->ReadTupleFloat(section, "robot_geometry", 0, 0.01);
    robot_geom.size.sl = cf->ReadTupleFloat(section, "robot_geometry", 1, 0.01);
}

int devbot::MainSetup(void)
{
    //Subscribe to the (input) opaque device
    if(!(this->opaque_dev = deviceTable->GetDevice(this->opaque_addr)))
    {
        PLAYER_ERROR("[devbot]: unable to locate suitable (input) opaque device");
        return(-1);
    }
    if(this->opaque_dev->Subscribe(this->InQueue) != 0)
    {
        PLAYER_ERROR("[devbot]: unable to subscribe to (input) opaque device");
        return(-1);
    }
    
    //Setup function handlers for protocol module
    devbot_protocol_init(receive_serial, transmit_serial, true);
    
    //Setup position2d devbot_player module
    devbot_player_position2d_init(this, &position2d_addr);
    devbot_protocol_assign_message_handler(MSG_POSITION, devbot_player_position2d_message_handler);
    
    //Setup imu devbot_player module (optional)
    if(has_imu)
    {
        devbot_player_imu_init(this, &imu_addr);
        devbot_protocol_assign_message_handler(MSG_IMU, devbot_player_imu_message_handler);    
    }
    
    //Setup power devbot_player module (optional)
    if(has_power)
    {
        devbot_player_power_init(this, &power_addr);
        devbot_protocol_assign_message_handler(MSG_POWER, devbot_player_power_message_handler);
    }
    
    //Setup joystick devbot_player module
    if(has_joystick)
    {
        devbot_player_joystick_init(this, &joystick_addr);
        devbot_protocol_assign_message_handler(MSG_JOYSTICK, devbot_player_joystick_message_handler);
    }
    
    //Reset odometry
    DEVBOT_PLAYER_POSITION2D_RESET_ODOMETRY();
    
    return(0);
}

void devbot::MainQuit(void)
{
    this->opaque_dev->Unsubscribe(this->InQueue);
}

void devbot::Main(void) 
{
    while(true)
    {
        pthread_testcancel();
        
        //Process requests
        this->ProcessMessages();

        pthread_testcancel();

        //Update devbot_protocol
        devbot_protocol_update();

        pthread_testcancel();

        //Write data
        this->WriteData();
        
        usleep(100);
    }
    return;
}

int devbot::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
    //Handle (input) opaque messages
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE, this->opaque_addr))
    {
        //Read message
        player_opaque_data_t *recv = reinterpret_cast<player_opaque_data_t *> (data);
        if(recv->data_count)
        {
            char *serial_data = new char[recv->data_count+1];
            int i;
            for(i = 0; i < recv->data_count; i++)
            {
                serial_data[i] = recv->data[i]; //received messages should be null terminated, but may be received incomplete, so copy element by element and add null terminator
            }
            serial_data[i] = '\0';
            //Append this to the serial_rx_data buffer
            strncat(serial_rx_data, serial_data, SERIAL_RX_BUFFER_LENGTH - strlen(serial_rx_data));
            serial_rx_data[SERIAL_RX_BUFFER_LENGTH-1] = '\0'; //if buffer is big enough, this will not over-write data, but is put in for safety
            delete [] serial_data;
        }

        //Transfer this to opaque driver
        this->Publish(this->out_opaque_addr, PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_CMD_DATA, recv);
        
        return(0);
    }
    
    //Handle opaque messages
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_OPAQUE_CMD_DATA, this->out_opaque_addr))
    {
        //Read message
        player_opaque_data_t *recv = reinterpret_cast<player_opaque_data_t *> (data);
        //Transfer to (input) opaque driver        
        this->opaque_dev->PutMsg(this->InQueue, PLAYER_MSGTYPE_CMD, PLAYER_OPAQUE_CMD_DATA, data, 0, NULL);
        
        return(0);
    }
    
    //Handle position2d messages
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL, this->position2d_addr))
    {
        player_position2d_cmd_vel_t *recv = reinterpret_cast<player_position2d_cmd_vel_t *> (data);    
        devbot_player_position2d_set_vel(recv->vel.px, recv->vel.pa);
        return(0);
    }
    
    //Handle position2d odometry requests
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_DATA_STATE, this->position2d_addr))
        {
            player_position2d_data_t odometry_data;
            devbot_player_position2d_get_position(&odometry_data);
        this->Publish(this->position2d_addr, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_DATA_STATE, (void *) &odometry_data, sizeof(odometry_data), NULL);
        return(0);
        }
    
    //Handle geometry requests (for physical size of the robot)
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM, this->position2d_addr))
      {
        this->Publish(this->position2d_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM, (void*) &robot_geom, sizeof(robot_geom), NULL);
        return(0);
    }
    
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_SET_ODOM, this->position2d_addr))
      {
        printf("unimplemented: - PLAYER_POSITION2D_REQ_SET_ODOM\r\n");
        return(0);
    }
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_MOTOR_POWER, this->position2d_addr))
      {
        printf("unimplemented: - PLAYER_POSITION2D_REQ_MOTOR_POWER\r\n");
        this->Publish(this->position2d_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
        return(0);
    }
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_RESET_ODOM, this->position2d_addr))
      {
        printf("unimplemented: - PLAYER_POSITION2D_REQ_RESET_ODOM\r\n");
        return(0);
    }
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_VELOCITY_MODE, this->position2d_addr))
      {
        printf("unimplemented: - PLAYER_POSITION2D_REQ_VELOCITY_MODE\r\n");
        return(0);
    }
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_POSITION_MODE, this->position2d_addr))
      {
        printf("unimplemented: - PLAYER_POSITION2D_REQ_POSITION_MODE\r\n");
        return(0);
    }    

    //Handle imu messages
    else if(has_imu)
    {
        if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_IMU_REQ_RESET_ORIENTATION, this->imu_addr))
        {
            devbot_player_imu_reset();    
            return(0);
        }
    }
    
    return(-1);
}

char *receive_serial(void)
{
    //The serial_rx_data buffer is written to in ProcessMessage() function.
    return serial_rx_data; //The serial_rx_data buffer is 'cleared' in devbot_protocol_update() in devbot_protocol.c 
}

void transmit_serial(char *message)
{
    //Append to the serial_tx_data buffer, this will eventually be sent out
    strncat(serial_tx_data, message, SERIAL_TX_BUFFER_LENGTH - strlen(serial_tx_data));
    serial_tx_data[SERIAL_TX_BUFFER_LENGTH-1] = '\0'; //if buffer is big enough, this will not over-write data, but is put in for safety
}

void devbot::WriteData(void)
{    
    //Write serial_tx_data buffer
    player_opaque_data_t serial_data;
    serial_data.data_count = strlen(serial_tx_data);
    if(serial_data.data_count) //Only publish if there is data to be sent
    {
        serial_data.data = (uint8_t *) serial_tx_data;
        this->opaque_dev->PutMsg(this->InQueue, PLAYER_MSGTYPE_CMD, PLAYER_OPAQUE_CMD_DATA, &serial_data, 0, NULL);
        serial_tx_data[0] = '\0'; //'clear' the serial_tx_data buffer
    }
    
    /*player_opaque_data_t mData;
    mData.data_count = 10;
    mData.data = new uint8_t[10];

    for(int i = 0; i < mData.data_count; i++)
    {
        mData.data[i] = 'A' + i;
    }
    mData.data[mData.data_count-1] = '\0';
    
    this->Publish(this->out_opaque_addr, PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_CMD_DATA, &mData);
    player_opaque_data_t_cleanup(&mData);*/
}

