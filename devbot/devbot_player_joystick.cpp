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

#include "devbot_player_joystick.h"

//#define DEBUG_DEVBOT_JOYSTICK

#define JOYSTICK_X        0
#define JOYSTICK_Y        1

//Local variables
static player_devaddr_t *devbot_player_joystick_addr; //pointer to joystick_addr
static Driver *devbot_obj;

void devbot_player_joystick_init(Driver *devbot_obj_pointer, player_devaddr_t *addr)
{
    devbot_obj = devbot_obj_pointer;
    devbot_player_joystick_addr = addr;
}

void devbot_player_joystick_message_handler(message_t m)
{
    #ifdef DEBUG_DEVBOT_JOYSTICK
    printf("joystick message: %s\n", (char *) m);
    #endif
    
    message_setting_t setting;
    devbot_protocol_read_header(m, &setting);
    switch(MESSAGE_SETTING_TO_ENUM(setting))
    {
        case JOYSTICK_GET:
        {
            player_joystick_data_t joystick_data;
            uint16_t tmp;
            devbot_protocol_read(m, &tmp, MSG_INT8); joystick_data.pos[JOYSTICK_X] = tmp;
            devbot_protocol_read(m, &tmp, MSG_INT8); joystick_data.pos[JOYSTICK_Y] = tmp;
            joystick_data.axes_count = 2;
            
            devbot_obj->Publish(*devbot_player_joystick_addr, PLAYER_MSGTYPE_DATA, PLAYER_JOYSTICK_DATA_STATE, (void*)&joystick_data, sizeof(joystick_data), NULL);
        } break;
        
        default:
        {
        
        } break;    
    }
}

