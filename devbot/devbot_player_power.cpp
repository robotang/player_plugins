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

#include "devbot_player_power.h"

//#define DEBUG_DEVBOT_POWER

//Local variables
static player_devaddr_t *devbot_player_power_addr; //pointer to power_addr
static Driver *devbot_obj;

void devbot_player_power_init(Driver *devbot_obj_pointer, player_devaddr_t *addr)
{
    devbot_obj = devbot_obj_pointer;
    devbot_player_power_addr = addr;
}

void devbot_player_power_message_handler(message_t m)
{
    #ifdef DEBUG_DEVBOT_POWER
    printf("power message: %s\n", (char *) m);
    #endif
    
    message_setting_t setting;
    devbot_protocol_read_header(m, &setting);
    switch(MESSAGE_SETTING_TO_ENUM(setting))
    {
        case POWER_GET:
        {
            player_power_data power_data;
            uint16_t tmp;
            devbot_protocol_read(m, &tmp, MSG_UINT16); power_data.volts = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //current = tmp / 1000.0;
            devbot_protocol_read(m, &tmp, MSG_UINT16); power_data.watts = tmp / 100.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); power_data.joules = tmp / 10.0;
            power_data.valid = (PLAYER_POWER_MASK_VOLTS | PLAYER_POWER_MASK_WATTS | PLAYER_POWER_MASK_JOULES);
            power_data.charging = 0; //TODO
            power_data.percent = 0; //TODO            
            
            devbot_obj->Publish(*devbot_player_power_addr, PLAYER_MSGTYPE_DATA, PLAYER_POWER_DATA_STATE, (void*)&power_data, sizeof(power_data), NULL);
        } break;
        
        case POWER_GET_ALL:
        {
            player_power_data power_data;
            uint16_t tmp;
            devbot_protocol_read(m, &tmp, MSG_UINT16); power_data.volts = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //vcell1 = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //vcell2 = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //vcell3 = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //vcell4 = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //vcell5 = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //vcell6 = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); //current = tmp / 1000.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); power_data.watts = tmp / 100.0; 
            devbot_protocol_read(m, &tmp, MSG_UINT16); power_data.joules = tmp / 10.0;
            power_data.valid = (PLAYER_POWER_MASK_VOLTS | PLAYER_POWER_MASK_WATTS | PLAYER_POWER_MASK_JOULES);
            power_data.charging = 0; //TODO
            power_data.percent = 0; //TODO
            
            devbot_obj->Publish(*devbot_player_power_addr, PLAYER_MSGTYPE_DATA, PLAYER_POWER_DATA_STATE, (void*)&power_data, sizeof(power_data), NULL);
        } break;
        
        default:
        {
        
        } break;    
    }
}

