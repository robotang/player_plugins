/*
 *  A plugin for interfacing gmapping to Player
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

#include "slam_gmapping.h"

//#define DEBUG_GMAPPING

using namespace std;
class gmapping : public ThreadedDriver
{
public:
    gmapping(ConfigFile *cf, int section);
    
    virtual int MainSetup(void);
    virtual void MainQuit(void);
    virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data);    

private:
    virtual void Main(void);
    void RefreshData(void);
    
    int HandleConfigs(QueuePointer& resp_queue, player_msghdr *hdr, void *data);
    int HandleCommands(QueuePointer& resp_queue, player_msghdr *hdr, void *data);
    int HandleData(QueuePointer& resp_queue, player_msghdr *hdr, void *idata);

    //Input position
    player_devaddr_t position_in_addr;
    Device *position_in_device;
    //Output position
    player_devaddr_t position_out_addr;
    int position_out_subscriptions;
    //Laser input
    player_devaddr_t laser_addr;    
    Device *laser_device;
    //Map
    player_devaddr_t map_addr;
    int map_subscriptions;
    player_map_data_t map_data;
    player_map_info_t map_info;
    
    slam_gmapping slam;
};

//Initialisation function
Driver* gmapping_Init(ConfigFile *cf, int section)
{
    return ((Driver*) (new gmapping(cf, section)));
}

//Driver registration function
void gmapping_Register(DriverTable *table)
{
    table->AddDriver("gmapping", gmapping_Init);
}

extern "C"
{
    int player_driver_init(DriverTable *table)
    {
        gmapping_Register(table);
        return(0);
    }
}

gmapping::gmapping(ConfigFile *cf, int section) 
    : ThreadedDriver(cf, section)
{
    //Messages to be passed onto gmapping module
    player_position2d_geom_t initial_pose;
    player_laser_geom_t laser_geom;
    player_laser_config_t laser_config;

    //Read input parameters
    this->position_out_subscriptions = 0;
    this->map_subscriptions = 0;
    
    //Connect to input position2d interface
    if(cf->ReadDeviceAddr(&(this->position_in_addr), section, "requires", PLAYER_POSITION2D_CODE, -1, NULL) == 0)
    {
        if(!(this->position_in_device = deviceTable->GetDevice(this->position_in_addr)))
        {
            PLAYER_ERROR("[gmapping]: unable to locate input position2d device");
            this->SetError(-1);
            return;
        }
        if(this->position_in_device->Subscribe(this->InQueue) != 0)
        {
            PLAYER_ERROR("[gmapping]: unable to subscribe to input position2d device");
            this->SetError(-1);
            return;
        }
        
        //Get initial pose
        if(cf->ReadTupleLength(section, "initial_pose", 0, -99999) != -99999)        
        {
            PLAYER_WARN("[gmapping]: overriding initial pose with one supplied from configuration file");
            initial_pose.pose.px = cf->ReadTupleLength(section, "initial_pose", 0, 0.0);
            initial_pose.pose.py = cf->ReadTupleLength(section, "initial_pose", 1, 0.0);
            initial_pose.pose.pyaw = cf->ReadTupleLength(section, "initial_pose", 2, 0.0);
        }    
        else
        {
            Message* msg;
            if(!(msg = position_in_device->Request(this->InQueue, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM, NULL, 0, NULL, false)))
            {
                PLAYER_ERROR("[gmapping]: failed to get geometry of underlying position device");
                this->SetError(-1);
                return;
            }
            initial_pose = *(player_position2d_geom *) msg->GetPayload();
            delete msg;
        }
    }
    else
    {
        PLAYER_ERROR("[gmapping]: failed to load input position2d interface");
        this->SetError(-1);
        return;
    }
    
    //Create output position2d interface
    if(cf->ReadDeviceAddr(&(this->position_out_addr), section, "provides", PLAYER_POSITION2D_CODE, -1, NULL) == 0)
    {
        if(this->AddInterface(this->position_out_addr))
        {
            PLAYER_ERROR("[gmapping]: failed to register output position2d interface");
            this->SetError(-1);
            return;
        }
    }
    else
    {
        PLAYER_ERROR("[gmapping]: failed to create output position2d interface");
        this->SetError(-1);
        return;
    }    
    
    //Connect to a laser interface
    if(cf->ReadDeviceAddr(&(this->laser_addr), section, "requires", PLAYER_LASER_CODE, -1, NULL) == 0)
    {
        if(!(this->laser_device = deviceTable->GetDevice(this->laser_addr)))
        {
            PLAYER_ERROR("[gmapping]: unable to locate laser device");
            this->SetError(-1);
            return;
        }
        if(this->laser_device->Subscribe(this->InQueue) != 0)
        {
            PLAYER_ERROR("[gmapping]: unable to subscribe to laser device");
            this->SetError(-1);
            return;
        }
        
        //Get laser geometry
        if(cf->ReadTupleLength(section, "laser_geom", 0, -99999) != -99999)        
        {
            PLAYER_WARN("[gmapping]: overriding laser geometry with one supplied from configuration file");
            laser_geom.pose.px = cf->ReadTupleLength(section, "laser_geom", 0, 0.0);
            laser_geom.pose.py = cf->ReadTupleLength(section, "laser_geom", 1, 0.0);
            laser_geom.pose.pyaw = cf->ReadTupleLength(section, "laser_geom", 2, 0.0);
        }    
        else
        {
            Message* msg;
            if(!(msg = laser_device->Request(this->InQueue, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM, NULL, 0, NULL, false)))
            {
                PLAYER_ERROR("[gmapping]: failed to get geometry of laser device");
                this->SetError(-1);
                return;
            }
            laser_geom = *(player_laser_geom_t *) msg->GetPayload();
            delete msg;
        }
        
        //Get laser configuration
        if(cf->ReadTupleLength(section, "laser_config", 0, -99999) != -99999)        
        {
            PLAYER_WARN("[gmapping]: overriding laser configuration with one supplied from configuration file");
            laser_config.min_angle = cf->ReadTupleLength(section, "laser_config", 0, -1.570796);
            laser_config.max_angle = cf->ReadTupleLength(section, "laser_config", 1, 1.570796);
            laser_config.resolution = cf->ReadTupleLength(section, "laser_config", 2, 0.017453);
            laser_config.max_range = cf->ReadTupleLength(section, "laser_config", 3, 8.0);
            laser_config.range_res = cf->ReadTupleLength(section, "laser_config", 4, 1.0);
            laser_config.intensity = cf->ReadTupleLength(section, "laser_config", 5, 1);
            laser_config.scanning_frequency = cf->ReadTupleLength(section, "laser_config", 6, 10);
        }    
        else
        {
            Message* msg;
            if(!(msg = laser_device->Request(this->InQueue, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_CONFIG, NULL, 0, NULL, false)))
            {
                PLAYER_ERROR("[gmapping]: failed to get geometry of laser device");
                this->SetError(-1);
                return;
            }
            laser_config = *(player_laser_config_t *) msg->GetPayload();
            delete msg;
        }
    }
    else
    {
        PLAYER_ERROR("[gmapping]: failed to load laser interface");
        this->SetError(-1);
        return;
    }
    
    //Create map interface
    if(cf->ReadDeviceAddr(&(this->map_addr), section, "provides", PLAYER_MAP_CODE, -1, NULL) == 0)
    {
        if(this->AddInterface(this->map_addr))
        {
            PLAYER_ERROR("[gmapping]: failed to register map interface");
            this->SetError(-1);        
            return;
        }
    }
    else
    {
        PLAYER_ERROR("[gmapping]: failed to create map interface");
        this->SetError(-1);
        return;
    }
    
    //Pass input parameters and initialise gmapping
    this->slam.init(cf, section, &initial_pose, &laser_geom, &laser_config);
}

int gmapping::MainSetup(void)
{
    return(0);
}

void gmapping::MainQuit(void)
{
    this->position_in_device->Unsubscribe(this->InQueue);
    this->laser_device->Unsubscribe(this->InQueue);
}

void gmapping::Main(void) 
{
    while(true)
    {
        pthread_testcancel();        
        
        //Process requests
        if(!this->InQueue->Empty())
            {
            this->ProcessMessages();
        }
        
        pthread_testcancel();
        
        this->RefreshData();

        usleep(10000);
    }
    return;
}

int gmapping::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
    switch(hdr->type)
    {
        case PLAYER_MSGTYPE_REQ:
            return (this->HandleConfigs(resp_queue, hdr, data));
        case PLAYER_MSGTYPE_CMD:
            return (this->HandleCommands(resp_queue, hdr, data));
        case PLAYER_MSGTYPE_DATA:
            return (this->HandleData(resp_queue, hdr, data));
        default:
            return -1;
    }
}

int gmapping::HandleConfigs(QueuePointer& resp_queue, player_msghdr *hdr, void *data)
{
    //Handle request for map meta-data
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_INFO, this->map_addr))
    {
        this->slam.get_map_info(&map_info);
        this->Publish(this->map_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_MAP_REQ_GET_INFO, (void*) &map_info, sizeof(map_info), NULL);
        return 0;
    }
    
    //Handle request for map
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_DATA, this->map_addr))
        {
        player_map_data_t *mapreq = (player_map_data_t*) data;
        size_t mapsize = sizeof(player_map_data_t);
        player_map_data_t *mapresp = (player_map_data_t*)calloc(1, mapsize);
        assert(mapresp);
        
        this->slam.get_map(mapreq, mapresp);
        this->Publish(this->device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_MAP_REQ_GET_DATA, (void*) mapresp);
        delete [] mapresp->data;
        free(mapresp);
        return 0;
    }
    
    //Handle request for position
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_DATA_STATE, this->position_out_addr))
        {
            player_position2d_data_t position_out_data;
            this->slam.get_pose(&position_out_data);
        this->Publish(this->position_out_addr, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_DATA_STATE, (void *) &position_out_data, sizeof(position_out_data), NULL);
        return 0;
        }

    return -1;
}

int gmapping::HandleCommands(QueuePointer& resp_queue, player_msghdr *hdr, void *data)
{
    printf("gmapping: handle commands unimplemented\r\n");
    return -1;
}

//TODO: is it leaking memory here?
int gmapping::HandleData(QueuePointer& resp_queue, player_msghdr *hdr, void *idata)
{
    static player_position2d_data_t last_pose;
    static bool have_pose = false;
    
    //Handle input position data
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, this->position_in_addr))
    {
        have_pose = true;
        player_position2d_data_t *pose = reinterpret_cast<player_position2d_data_t*> (idata);
        last_pose = *pose;
        return 0;
    }
    
    //Handle laser data
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN, this->laser_addr))
    {
        player_laser_data *laser_scan = reinterpret_cast<player_laser_data_t*> (idata);
        if(have_pose)
        {
            #ifdef DEBUG_GMAPPING
            static uint8_t i = 0;
            if(i > 5) { printf("[gmapping input pose]: %f, %f, %f\n", last_pose.pos.px, last_pose.pos.py, last_pose.pos.pa); i = 0; }
            else i++;
            #endif    
            this->slam.update(laser_scan, &last_pose); //TODO: is this ok using last pose? should we request pose instead?
        }
        return 0;
    }
    
    return -1;
}

void gmapping::RefreshData(void)
{
    player_position2d_data_t position_out_data;
    this->slam.get_pose(&position_out_data);
    #ifdef DEBUG_GMAPPING
    static uint8_t i = 0;
    if(i > 10) { printf("[gmapping map pose]: %f, %f, %f\n", position_out_data.pos.px, position_out_data.pos.py, position_out_data.pos.pa); i = 0; }
    else i++;
    #endif
    this->Publish(this->position_out_addr, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, (void *) &position_out_data, sizeof(position_out_data), NULL);
    return;
}

