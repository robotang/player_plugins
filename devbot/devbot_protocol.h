/*
 *  A simple, lightweight, and extensible ASCII based communication protocol for controlling robots via UART
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
 
#ifndef DEVBOT_PROTOCOL_H
#define DEVBOT_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_MESSAGE_LENGTH          60
typedef char message_t[MAX_MESSAGE_LENGTH];
typedef char* message_ptr;

#define MESSAGE_RX_BUFFER_SIZE      5
#define MESSAGE_TX_BUFFER_SIZE      5

#define MSG_NUM                     7
typedef enum {MSG_POSITION, MSG_IMU, MSG_SONAR, MSG_POWER, MSG_JOYSTICK, MSG_DEBUG, MSG_CONFIG} message_packet_t;
typedef enum {MSG_INT8, MSG_UINT8, MSG_INT16, MSG_UINT16, MSG_INT32, MSG_UINT32, MSG_FLOAT, MSG_CHAR, MSG_STRING} message_data_t;
typedef char message_setting_t; //Stored as a char as it allows more settings to be stored with one character (compared to 10 for decimal ASCII)

//Actually use enums for defining each packet type setting, so need to use the following macros to convert
#define MESSAGE_SETTING_TO_CHAR(x)      ((x) + 65)
#define MESSAGE_SETTING_TO_ENUM(c)      ((c) - 65)

//Settings for MSG_POSITION 
enum {POSITION_GET_ODOM, POSITION_SET_ODOM, POSITION_GET_VEL, POSITION_SET_VEL, POSITION_GET_GAINS, POSITION_SET_GAINS, POSITION_CONFIG};
//Settings for MSG_IMU 
enum {IMU_GET_ESTIMATED, IMU_GET_RAW, IMU_CONFIG, IMU_RESET};
//Settings for MSG_SONAR
enum {SONAR_GET};
//Settings for MSG_POWER
enum {POWER_GET, POWER_GET_ALL, POWER_RESET_ENERGY};
//Settings for MSG_JOYSTICK
enum {JOYSTICK_GET};
//Settings for MSG_DEBUG
enum {DEBUG_GET, DEBUG_SET};
//Settings for MSG_CONFIG
enum {CONFIG_GET, CONFIG_SET};

//Various macros
#define _MIN(a,b)                       ((a<b)?(a):(b))
#define _MAX(a,b)                       ((a>b)?(a):(b))
#define _ABS(x)                         ((x>0)?(x):(-x))
#define _BIND(LOWER, val, UPPER)        _MIN((_MAX(val, LOWER)), UPPER)
#define _IS_IN(LOWER,val,UPPER)         ((((val)>LOWER && (val)<UPPER))?(1):(0))

//Various limits of integer types
#define MIN_INT8     -128
#define MAX_INT8     127
#define MAX_UINT8    255
#define MIN_INT16    -32768
#define MAX_INT16    32767
#define MAX_UINT16   65535
#define MIN_INT32    -2147483648
#define MAX_INT32    2147483647
#define MAX_UINT32   4294967295

void devbot_protocol_init(char *(*receive)(void), void (*transmit)(char *), bool bulk_message_process);
void devbot_protocol_assign_message_handler(message_packet_t msg, void (*message_handler)(message_t));
void devbot_protocol_add_message(message_ptr m);
void devbot_protocol_write_header(message_ptr m, message_packet_t message, message_setting_t setting);
void devbot_protocol_read_header(message_ptr m, message_setting_t *setting); //Also strips header away from m
void devbot_protocol_write(message_ptr m, const void *data, message_data_t data_type); //Writes field data into m.
void devbot_protocol_read(message_ptr m, void *data, message_data_t data_type); //Read the field and puts the value into data, and removes this field from m 
void devbot_protocol_update(void);

#endif

