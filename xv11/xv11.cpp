/*
 *  A plugin for interfacing an xv11 laser scanner to Player
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
#include <stdint.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include <libplayercore/playercore.h>

#include "laser2d_filter.h"

#define XV11_ANGULAR_RESOLUTION             (M_PI/180.0)
#define XV11_RANGE_RESOLUTION               0.01
#define DEFAULT_XV11_MAXRANGE               5.0
#define DEFAULT_XV11_ANGLE_OFFSET_DEGREES    0
#define DEFAULT_XV11_MINANGLE_DEGREES       -90
#define DEFAULT_XV11_MAXANGLE_DEGREES       90
#define DEFAULT_XV11_FILTER                 LASER2D_FILTER_NONE
#define DEFAULT_XV11_FILTER_DEPTH           1

#define XV11_PACKET_LENGTH          22 //(start + index + 2x speed + 4 * 4x data + 2x checksum) - packet with four range measurements
#define MAX_BUFFER_LENGTH           XV11_PACKET_LENGTH

#define XV11_START_BYTE             0xFA
#define XV11_INDEX_OFFSET           0xA0
#define XV11_INDEX_INDEX            1
#define XV11_DATA_START_INDEX       4
#define XV11_CHECKSUM_INDEX         20

#define XV11_RANGES_LENGTH          360
#define RANGES_BUFFER_A             0
#define RANGES_BUFFER_B             1

#define DEGREES_TO_RADIANS(x)       (((x)* M_PI)/180.0)

#define BAD_READING_VALUE           -1.0

using namespace std;
class xv11 : public ThreadedDriver
{
public:
    xv11(ConfigFile *cf, int section);

    virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data);

private:
    virtual void Main(void);
    virtual int MainSetup(void);
    virtual void MainQuit(void);

    void clear_range_buffer(int buffer);
    bool validate_buffer(void);
    void process_buffer(void);

    player_devaddr_t laser_id;
    player_devaddr_t opaque_addr;
    Device *opaque_dev;
    
    player_laser_geom laser_geom;
    player_laser_config_t laser_config;
    double max_range;
    bool invert;
    int angle_offset, min_angle, max_angle, filter_cfg, filter_depth;
    
    uint8_t buffer[MAX_BUFFER_LENGTH];
    int index_;
    
    //Range reading (full 360 degrees)
    float ranges[2][XV11_RANGES_LENGTH];
    int current_ranges_buffer;
    bool send_ranges;
    
    laser2d_filter *filter;
};

//Initialisation function
Driver* xv11_Init(ConfigFile *cf, int section)
{
    return ((Driver*) (new xv11(cf, section)));
}

//Driver registration function
void xv11_Register(DriverTable *table)
{
    table->AddDriver("xv11", xv11_Init);
}

extern "C"
{
    int player_driver_init(DriverTable *table)
    {
        xv11_Register(table);
        return(0);
    }
}

xv11::xv11(ConfigFile *cf, int section) 
    : ThreadedDriver(cf, section)
{
    //Setup (input) opaque interface
    if(cf->ReadDeviceAddr(&(this->opaque_addr), section, "requires", PLAYER_OPAQUE_CODE, -1, NULL) != 0)
    {
        PLAYER_ERROR("failed to setup input opaque driver");
        this->SetError(-1);
        return;
    }
    
    //Setup laser
    if(cf->ReadDeviceAddr(&(this->laser_id), section, "provides", PLAYER_LASER_CODE, -1, NULL) != 0)
    {
        PLAYER_ERROR("failed to setup output laser");
        this->SetError(-1);
        return;
    }    
    else if(this->AddInterface(this->laser_id))
    {
        PLAYER_ERROR("failed to register output laser");
        this->SetError(-1);        
        return;
    }
    
    //Laser geometry
    laser_geom.pose.px = cf->ReadTupleLength(section, "pose", 0, 0.0); //x (m)
    laser_geom.pose.py = cf->ReadTupleLength(section, "pose", 1, 0.0); //y (m)
    laser_geom.pose.pz = cf->ReadTupleLength(section, "pose", 2, 0.0); //z (m)
    laser_geom.pose.proll = cf->ReadTupleLength(section, "pose", 3, 0.0); //roll (rad)
    laser_geom.pose.ppitch = cf->ReadTupleLength(section, "pose", 4, 0.0); //pitch (rad)
    laser_geom.pose.pyaw = cf->ReadTupleLength(section, "pose", 5, 0.0); //yaw (rad)
    laser_geom.size.sw = cf->ReadTupleLength(section, "size", 0, 0.285); //width (m)
    laser_geom.size.sl = cf->ReadTupleLength(section, "size", 1, 0.06); //length (m)
    laser_geom.size.sh = cf->ReadTupleLength(section, "size", 2, 0.04); //height (m)
    
    //Laser parameters
    max_range = cf->ReadFloat(section, "max_range", DEFAULT_XV11_MAXRANGE);
    invert = (cf->ReadInt(section, "invert", 0) != 0) ? true : false;
    angle_offset = cf->ReadInt(section, "angle_offset_degrees", DEFAULT_XV11_ANGLE_OFFSET_DEGREES);
    min_angle = cf->ReadInt(section, "min_angle_degrees", DEFAULT_XV11_MINANGLE_DEGREES);
    max_angle = cf->ReadInt(section, "max_angle_degrees", DEFAULT_XV11_MAXANGLE_DEGREES);
    filter_cfg = cf->ReadInt(section, "filter", DEFAULT_XV11_FILTER);
    filter_depth = cf->ReadInt(section, "filter_depth", DEFAULT_XV11_FILTER_DEPTH);
    
    filter = new laser2d_filter(XV11_RANGES_LENGTH, true, filter_cfg, filter_depth);
    
    index_ = 0;
    current_ranges_buffer = RANGES_BUFFER_A;
    clear_range_buffer(RANGES_BUFFER_A);
    clear_range_buffer(RANGES_BUFFER_B);
    send_ranges = false;
}

int xv11::MainSetup(void)
{
    //Subscribe to the (input) opaque device
    if(!(this->opaque_dev = deviceTable->GetDevice(this->opaque_addr)))
    {
        PLAYER_ERROR("unable to locate suitable (input) opaque device");
        return(-1);
    }
    if(this->opaque_dev->Subscribe(this->InQueue) != 0)
    {
        PLAYER_ERROR("unable to subscribe to (input) opaque device");
        return(-1);
    }
    
    return(0);
}

void xv11::MainQuit(void)
{
    this->opaque_dev->Unsubscribe(this->InQueue);
    if(filter) delete filter;
}

void xv11::Main(void) 
{
    uint32_t id = 1;
    while(true)
    {
        pthread_testcancel();
        
        //Process requests
        this->ProcessMessages();

        pthread_testcancel();

        if(send_ranges)
        {
            int ranges_buffer = (current_ranges_buffer == RANGES_BUFFER_A) ? RANGES_BUFFER_B : RANGES_BUFFER_A;
            
            player_laser_data_t data;
            data.min_angle = DEGREES_TO_RADIANS(min_angle);
            data.max_angle = DEGREES_TO_RADIANS(max_angle);
            data.max_range = max_range;
            data.ranges_count = abs(min_angle) + max_angle; //xv11 has a range for each angle in degrees
            data.ranges = new float[data.ranges_count];
            
            if(invert)
            {
                for(int i = 0, j = max_angle - angle_offset; i < data.ranges_count; i++, j--)
                {
                    if(j <= 0)
                        j += XV11_RANGES_LENGTH;
                
                    data.ranges[i] = ranges[ranges_buffer][j];
                    if(data.ranges[i] < 0.0) data.ranges[i] = 0.0;
                }
            }
            else
            {
                for(int i = 0, j = XV11_RANGES_LENGTH - abs(min_angle) + angle_offset; i < data.ranges_count; i++, j++)
                {
                    if(j >= XV11_RANGES_LENGTH)
                        j -= XV11_RANGES_LENGTH;
                
                    data.ranges[i] = ranges[ranges_buffer][j];
                    if(data.ranges[i] < 0.0) data.ranges[i] = 0.0;
                }
            }
            
            data.resolution = XV11_ANGULAR_RESOLUTION;
            data.intensity_count = data.ranges_count; //not used
            data.intensity = new uint8_t[data.intensity_count]; //not used
            data.id = id++;
            Publish(laser_id, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN, &data);
            player_laser_data_t_cleanup(&data);
        
            send_ranges = false;
        }
        
        usleep(100);
    }
    return;
}

int xv11::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
    //Handle (input) opaque messages
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE, this->opaque_addr))
    {
        //Read message
        player_opaque_data_t *recv = reinterpret_cast<player_opaque_data_t *> (data);
        if(recv->data_count)
        {
            for(int i = 0; i < recv->data_count; i++)
            {
                uint8_t c = (uint8_t) recv->data[i];
                if(c == XV11_START_BYTE)
                {
                    process_buffer();
                    index_ = 0;
                }
                if(index_ < MAX_BUFFER_LENGTH) //Protect from buffer overflow
                    buffer[index_++] = c;
            }    
        }
        
        return(0);
    }
    
    //Respond to laser geometry request
    else if(Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM, this->laser_id))
      {
        player_laser_geom_t geom;
        memset(&geom, 0, sizeof(geom));
        geom = laser_geom;
        Publish(this->laser_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_GET_GEOM, (void*)&geom, sizeof(geom), NULL);
        return(0);
    }    
    
    //Respond to laser config request
    else if(Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_CONFIG, this->laser_id))
    {
        player_laser_config_t config;
        memset(&config, 0, sizeof(config));
        config = laser_config;
        Publish(this->laser_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_LASER_REQ_GET_CONFIG, (void*)&config, sizeof(config), NULL);
        return(0);
    }
    
    return(-1);
}

void xv11::clear_range_buffer(int buffer)
{
    float default_value = (filter_cfg) ? BAD_READING_VALUE : 0.0;
    for(int i = 0; i < XV11_RANGES_LENGTH; i++)
        ranges[buffer][i] = default_value;
}

bool xv11::validate_buffer(void)
{
    int incomming_checksum = (buffer[XV11_CHECKSUM_INDEX + 1] << 8) + buffer[XV11_CHECKSUM_INDEX];    
    
    int chk32 = 0;
    for(int i = 0; i < 10; i++)
        chk32 = (chk32 << 1) + (buffer[2*i + 1] << 8) + buffer[2*i + 0];
    
    int checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum = checksum & 0x7FFF;
    
    return (checksum == incomming_checksum);
}

void xv11::process_buffer(void)
{
    if(!validate_buffer())
        return;
    
    int index = buffer[XV11_INDEX_INDEX] - XV11_INDEX_OFFSET;

    if(index < 0 || index > 90)
        return;

    for(int i = 0; i < 4; i++)
    {
        uint8_t x = buffer[XV11_DATA_START_INDEX + 4*i + 0];
        uint8_t x1 = buffer[XV11_DATA_START_INDEX + 4*i + 1];
        uint8_t x2 = buffer[XV11_DATA_START_INDEX + 4*i + 2];
        uint8_t x3 = buffer[XV11_DATA_START_INDEX + 4*i + 3];
        
        int distance_mm = ((x1 & 0x3F) << 8) + x;
        int quality =  (x3 << 8) + x2;
        
        if(x1 & 0x80) //Low quality reading
            ranges[current_ranges_buffer][4*index + i] = BAD_READING_VALUE;
        else
        {
            if(x1 & 0x40) //Questionable quality reading
                ranges[current_ranges_buffer][4*index + i] = BAD_READING_VALUE;            
            else //Good reading
                ranges[current_ranges_buffer][4*index + i] = distance_mm / 1000.0;
        }
    }
    
    if(index == 0)
    {
        if(filter_cfg)
        {
            filter->process(ranges[current_ranges_buffer]); //changes ranges
        }
        
        current_ranges_buffer = (current_ranges_buffer == RANGES_BUFFER_A) ? RANGES_BUFFER_B : RANGES_BUFFER_A;
        clear_range_buffer(current_ranges_buffer);
        
        send_ranges = true;
    }
}

