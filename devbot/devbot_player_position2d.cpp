/*
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

#include "devbot_player_position2d.h"

//#define DEBUG_DEVBOT_POSITION2D

//Local variables
static player_devaddr_t *devbot_player_position2d_addr; //pointer to position2d_addr
static Driver *devbot_obj;
static player_position2d_data_t position_data;
static bool have_position_data;
void devbot_player_position2d_init(Driver *devbot_obj_pointer, player_devaddr_t *addr)
{
    devbot_obj = devbot_obj_pointer;
    devbot_player_position2d_addr = addr;
    position_data.pos.px = 0.0;
    position_data.pos.py = 0.0;
    position_data.pos.pa = 0.0;
    have_position_data = false;
}

//Message handler of this module for devbot_protocol messages
void devbot_player_position2d_message_handler(message_t m)
{
    static player_pose2d_t last_velocity;
    static bool have_velocity = false;
    
    #ifdef DEBUG_DEVBOT_POSITION2D
    static uint8_t i = 0;
    if(i > 5) { printf("[devbot position message]: %s\n", (char *) m); i = 0; }
    else i++;
    #endif
    
    message_setting_t setting;
    devbot_protocol_read_header(m, &setting);
    switch(MESSAGE_SETTING_TO_ENUM(setting))
    {
        case POSITION_GET_ODOM:
        {
            have_position_data = true;
            float tmp;
            devbot_protocol_read(m, &tmp, MSG_FLOAT); position_data.pos.px = tmp / 100.0; //convert cm to m
            devbot_protocol_read(m, &tmp, MSG_FLOAT); position_data.pos.py = tmp / 100.0; //convert cm to m
            devbot_protocol_read(m, &tmp, MSG_FLOAT); position_data.pos.pa = tmp;
            if(have_velocity)
            {
                position_data.vel = last_velocity;
                position_data.stall = 0; //TODO
            }
            
            devbot_obj->Publish(*devbot_player_position2d_addr, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, (void*)&position_data, sizeof(position_data), NULL);
        } break;
        
        
        case POSITION_GET_VEL:
        {
            //printf("TODO!!!!!!!!!!!!!!!!!!!!!!\r\n");
            /*
            have_velocity = true;
                        
            int16_t left_cmps, right_cmps;
            devbot_protocol_read(m, &left_cmps, MSG_INT16);
            devbot_protocol_read(m, &right_cmps, MSG_INT16);
            printf("\t\tleft_cmps: %f, right_cmps: %f\r\n", left_cmps / 100.0, right_cmps / 100.0);
            
            if(left_cmps == right_cmps)
            {
                last_velocity.px = left_cmps / 100.0;
                last_velocity.py = 0.0; //differential drive robot -> non-holonomic            
                last_velocity.pa = 0.0;
            }
            else
            {
                last_velocity.px = ((left_cmps / 100.0) + (right_cmps / 100.0)) / 2.0;
                last_velocity.py = 0.0; //differential drive robot -> non-holonomic            
                last_velocity.pa = (2.0 / WHEEL_SEPARATION_M) * (last_velocity.px - (left_cmps / 100.0));
            }*/            
        } break;
        
        default:
        {
        
        } break;
    }    
}

bool devbot_player_position2d_have_position_data(void)
{
    return have_position_data;
}

void devbot_player_position2d_get_position(player_position2d_data_t *position)
{
    *position = position_data;
}

void devbot_player_position2d_set_vel(float velocity, float yaw)
{
    //Create a devbot_protocol message
    message_t m;
    devbot_protocol_write_header(m, MSG_POSITION, MESSAGE_SETTING_TO_CHAR(POSITION_SET_VEL));
    velocity = _BIND(MIN_INT16, 100 * velocity, MAX_INT16); //convert from m/s cm/s
    yaw = yaw * 180.0 / M_PI; //convert from rad/s to degrees per second
    yaw = _BIND(MIN_INT16, yaw, MAX_INT16);
    int16_t speed_cmps = (int16_t) velocity;
    int16_t yaw_dps =  (int16_t) yaw;
    devbot_protocol_write(m, &speed_cmps, MSG_INT16);
    devbot_protocol_write(m, &yaw_dps, MSG_INT16);
    
    /*double left_cmps_raw = (yaw == 0.0) ? (velocity * 100) : (velocity - (WHEEL_SEPARATION_M * 0.5* yaw)) * 100; //vl = (r - d/2) * w, r = v / w
    double right_cmps_raw = (yaw == 0.0) ? (velocity * 100) : (velocity + (WHEEL_SEPARATION_M * 0.5* yaw)) * 100; //vr = (r + d/2) * w, r = v / w
    int16_t left_cmps = _BIND(MIN_INT16, (int) left_cmps_raw, MAX_INT16);
    int16_t right_cmps = _BIND(MIN_INT16, (int) right_cmps_raw, MAX_INT16);    
    devbot_protocol_write(m, &left_cmps, MSG_INT16);
    devbot_protocol_write(m, &right_cmps, MSG_INT16);*/
    
    //Add message
    devbot_protocol_add_message(m);
}

void devbot_player_position2d_set_odometry(float x, float y, float yaw)
{
    //Create a devbot_protocol message
    message_t m;
    devbot_protocol_write_header(m, MSG_POSITION, MESSAGE_SETTING_TO_CHAR(POSITION_SET_ODOM));
    int16_t x_ = (int16_t) (x * 100.0), y_ = (int16_t) (y * 100.0), yaw_ = (int16_t) (yaw * 1000.0);    
    devbot_protocol_write(m, &x_, MSG_INT16);
    devbot_protocol_write(m, &y_, MSG_INT16);
    devbot_protocol_write(m, &yaw_, MSG_INT16);
    //Add message
    devbot_protocol_add_message(m);
}

