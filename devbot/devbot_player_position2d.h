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

#ifndef DEVBOT_PLAYER_POSITION2D_H
#define DEVBOT_PLAYER_POSITION2D_H

#include <libplayercore/playercore.h>
extern "C"
{
#include "devbot_protocol.h"
}

#define DEVBOT_PLAYER_POSITION2D_RESET_ODOMETRY()    devbot_player_position2d_set_odometry(0.0, 0.0, 0.0)

void devbot_player_position2d_init(Driver *devbot_obj_pointer, player_devaddr_t *addr);
void devbot_player_position2d_message_handler(message_t m);
bool devbot_player_position2d_have_position_data(void);
void devbot_player_position2d_get_position(player_position2d_data_t *position);
void devbot_player_position2d_set_vel(float velocity, float yaw);
void devbot_player_position2d_set_odometry(float x, float y, float yaw);

#endif
