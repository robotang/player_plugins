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

#include "devbot_player_imu.h"

//#define DEBUG_DEVBOT_IMU

#define GYRO_DPS_SCALE_FACTOR        0.01512
#define ACCEL_1G_SCALE_FACTOR        0.0001678
#define PRY_SCALE_FACTOR        0.0109863
#define PRY_RATE_SCALE_FACTOR        GYRO_DPS_SCALE_FACTOR

//Local variables
static player_devaddr_t *devbot_player_imu_addr; //pointer to imu_addr
static Driver *devbot_obj;

void devbot_player_imu_init(Driver *devbot_obj_pointer, player_devaddr_t *addr)
{
    devbot_obj = devbot_obj_pointer;
    devbot_player_imu_addr = addr;
}

void devbot_player_imu_message_handler(message_t m)
{
    #ifdef DEBUG_DEVBOT_IMU
    printf("imu message: %s\n", (char *) m);
    #endif
    
    message_setting_t setting;
    devbot_protocol_read_header(m, &setting);
    switch(MESSAGE_SETTING_TO_ENUM(setting))
    {
        case IMU_GET_ESTIMATED:
        {
            player_imu_data_state imu_data;
            int16_t tmp;
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.pose.ppitch = tmp * PRY_SCALE_FACTOR; 
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.pose.proll = tmp * PRY_SCALE_FACTOR; 
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.pose.pyaw = tmp * PRY_SCALE_FACTOR;
            devbot_protocol_read(m, &tmp, MSG_INT16); //pitch_rate = tmp * PRY_RATE_SCALE_FACTOR;
            devbot_protocol_read(m, &tmp, MSG_INT16); //roll_rate = tmp * PRY_RATE_SCALE_FACTOR;
            devbot_protocol_read(m, &tmp, MSG_INT16); //yaw_rate = tmp * PRY_RATE_SCALE_FACTOR;
            imu_data.pose.px = 0.0; //TODO
            imu_data.pose.py = 0.0; //TODO
            imu_data.pose.pz = 0.0; //TODO
            
            devbot_obj->Publish(*devbot_player_imu_addr, PLAYER_MSGTYPE_DATA, PLAYER_IMU_DATA_STATE, (void*)&imu_data, sizeof(imu_data), NULL);
        } break;
        
        case IMU_GET_RAW:
        {
            player_imu_data_calib_t  imu_data;
            int16_t tmp;            
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.accel_x = tmp * ACCEL_1G_SCALE_FACTOR; 
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.accel_y = tmp * ACCEL_1G_SCALE_FACTOR;
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.accel_z = tmp * ACCEL_1G_SCALE_FACTOR;
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.gyro_x = tmp * GYRO_DPS_SCALE_FACTOR; 
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.gyro_y = tmp * GYRO_DPS_SCALE_FACTOR;
            devbot_protocol_read(m, &tmp, MSG_INT16); imu_data.gyro_z = tmp * GYRO_DPS_SCALE_FACTOR;
            imu_data.magn_x = 0.0;
            imu_data.magn_y = 0.0;
            imu_data.magn_z = 0.0;
            
            devbot_obj->Publish(*devbot_player_imu_addr, PLAYER_MSGTYPE_DATA, PLAYER_IMU_DATA_CALIB, (void*)&imu_data, sizeof(imu_data), NULL);
        } break;
        
        default:
        {
        
        } break;    
    }
}

void devbot_player_imu_reset(void)
{
    //Create a devbot_protocol message
    message_t m;
    devbot_protocol_write_header(m, MSG_IMU, MESSAGE_SETTING_TO_CHAR(IMU_RESET));

    //Add message
    devbot_protocol_add_message(m);
}

