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
 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

//#define MICROCONTROLLER    //Comment this line out if it is on the PC

#ifdef MICROCONTROLLER
    #include "uart.h"
    //#include "rprintf.h"
    //#include "control.h" //For testing
#endif

#include "devbot_protocol.h"
#include "ring.h"

#define ASCIIHEX_TO_UINT(x)         ( ((x) > '9') ? (x-55):(x-48) )    
#define UINT_TO_ASCIIHEX(x)         ( ((x) > 9) ? (x+55):(x+48) )    

#define FIELD_SEPARATOR             ','
#define FIELD_SEPARATOR_STRING      ","

//Local variables
static message_t message_rx_data[MESSAGE_RX_BUFFER_SIZE], message_tx_data[MESSAGE_TX_BUFFER_SIZE]; //Storage of actual data (array of messages (strings))
static ring_t message_rx_buffer, message_tx_buffer; //Buffer abstractions
static void (*devbot_protocol_message_handler[MSG_NUM])(message_t); //Array of message handlers for each message type
#ifndef MICROCONTROLLER
static char *(*devbot_protocol_receive)(void); //Function used to receive incoming characters
#endif
static void (*devbot_protocol_transmit)(char *); //Function used for sending the messages
static bool message_process_bulk;

//Private function prototypes
static uint8_t generate_checksum(const char *string);
static bool validate_message(char *string, uint8_t checksum);
static message_packet_t message_peek(message_t m);
static void message_assemble(unsigned char c);
static void read_field(char *field, void *data, message_data_t data_type);
//static void devbot_protocol_test(void);

/*
 * Public functions
 */

void devbot_protocol_init(char *(*receive)(void), void (*transmit)(char *), bool bulk_message_process)
{
    message_process_bulk = bulk_message_process;
    ring_init(&message_rx_buffer, message_rx_data, sizeof(message_rx_data));
    ring_init(&message_tx_buffer, message_tx_data, sizeof(message_tx_data));
    
    //Set receive and transmit function pointers
    #ifdef MICROCONTROLLER
    uart_set_rx_handler(message_assemble); //uartSetRxHandler(message_assemble);    
    #else
    devbot_protocol_receive = receive;
    #endif
    devbot_protocol_transmit = transmit;
    
    //Clear function pointers
    uint8_t i;
    for(i = 0; i < MSG_NUM; i++) devbot_protocol_message_handler[i] = NULL;
}

void devbot_protocol_assign_message_handler(message_packet_t msg, void (*message_handler)(message_t))
{
    devbot_protocol_message_handler[msg] = message_handler;
}

void devbot_protocol_add_message(message_ptr m)
{
    message_t tmp;
    uint8_t checksum = generate_checksum((const char *) m);
    tmp[0] = '$';
    uint16_t len = strlen(m);
    if(len < MAX_MESSAGE_LENGTH + 6)
    {
        memcpy(&tmp[1], m, len); //i.e. m must be null terminated
        tmp[len + 1] = '*';
        tmp[len + 2] = UINT_TO_ASCIIHEX(checksum / 16);
        tmp[len + 3] = UINT_TO_ASCIIHEX(checksum % 16);
        tmp[len + 4] = '\r';
        tmp[len + 5] = '\n';
        tmp[len + 6] = '\0';
        ring_write_safe(&message_tx_buffer, &tmp, sizeof(message_t));
    }
}

void devbot_protocol_write_header(message_ptr m, message_packet_t message, message_setting_t setting)
{
    uint8_t i = 0;
    //Write message header
    m[i++] = (char) (message + 65); m[i++] = ',';
    m[i++] = setting; m[i++] = '\0';
}

void devbot_protocol_read_header(message_ptr m, message_setting_t *setting)
{
    *setting = m[2];
    uint8_t i, len = strlen(m) - 4;
    //Remove header information, leaving just data in the message
    for(i = 0; i < len; i++)
    {
        m[i] = m[i+4];
    }
    m[i] = '\0';
}

//Example usage: if data is string: devbot_protocol_write(m, data, STRING)
//Otherwise, for all other data types, eg uint16_t: devbot_protocol_write(m, &data, UINT16) 
void devbot_protocol_write(message_ptr m, const void *data, message_data_t data_type)
{
    uint8_t len = strlen(m);
    m[len++] = FIELD_SEPARATOR; m[len] = '\0';
    switch(data_type)
    {
        case MSG_INT8: { snprintf(m + (uint8_t) strlen(m), MAX_MESSAGE_LENGTH, "%d", *(int8_t *)(data)); } break;
        case MSG_UINT8: { snprintf(m + (uint8_t) strlen(m), MAX_MESSAGE_LENGTH, "%d", *(uint8_t *)(data)); } break;
        case MSG_INT16: { snprintf(m + (uint8_t) strlen(m), MAX_MESSAGE_LENGTH, "%d", *(int16_t *)(data)); } break;
        case MSG_UINT16: { snprintf(m + (uint8_t) strlen(m), MAX_MESSAGE_LENGTH, "%d", *(uint16_t *)(data)); } break;
        case MSG_INT32: { snprintf(m + (uint8_t) strlen(m), MAX_MESSAGE_LENGTH, "%d", *(int32_t *)(data)); } break; //Doesnt work on ATmega!
        case MSG_UINT32: { snprintf(m + (uint8_t) strlen(m), MAX_MESSAGE_LENGTH, "%d", *(uint32_t *)(data)); } break; //Doesnt work on ATmega!
        case MSG_FLOAT: { snprintf(m + (uint8_t) strlen(m), MAX_MESSAGE_LENGTH, "%.3f", *(float *)(data)); } break; //Doesnt work on ATmega!
        case MSG_CHAR: { m[len++] = *(char *)(data); m[len++] = '\0'; } break;    
        case MSG_STRING: 
        { 
            uint8_t i, len2;
            char *tmp = (char *) data;            
            len2 = strlen(tmp) + 1; //copy null terminator
            for(i = 0; i < len2; i++) m[len++] = tmp[i];
        } break;

        default:
        {
        
        } break;
    }
}

void devbot_protocol_read(message_t m, void *data, message_data_t data_type)
{
    char tmp[10];
    uint8_t i, j, len = strlen((char *) m);
    //Extract field
    for(i = 0; i < len && m[i] != FIELD_SEPARATOR; i++)
    {
        tmp[i] = m[i];
    }
    tmp[i++] = '\0';
    
    //Read field
    switch(data_type)
    {
        case MSG_INT8: { * (int8_t *) data = atoi(tmp); } break;
        case MSG_UINT8: { * (uint8_t *) data = atoi(tmp); } break;
        case MSG_INT16: { * (uint16_t *) data = atoi(tmp); } break;
        case MSG_UINT16: { * (uint16_t *) data = atoi(tmp); } break;
        case MSG_INT32: { * (int32_t *) data = atoi(tmp); } break; //Doesnt work on ATmega!
        case MSG_UINT32: { * (uint32_t *) data = atoi(tmp); } break; //Doesnt work on ATmega!
        case MSG_FLOAT: { * (float *) data = atof(tmp); } break; //Doesnt work on ATmega!
        case MSG_CHAR: { * (char *) data = tmp[0]; } break;    
        case MSG_STRING: 
        { 
            uint8_t k, len2 = strlen(tmp) + 1; //copy null terminator
            for(k = 0; k < len2; k++) * (char *) (data++) = tmp[k];
        } break;
        default: { ; } break;
    }
    
    //Remove field that was just read, from message
    j = i;
    for(i = 0; i < len - j; i++)
    {
        m[i] = m[i+j];
    }
    m[i] = '\0';
}

void devbot_protocol_update(void)
{
    #ifndef MICROCONTROLLER
    char *rx = devbot_protocol_receive();    
    uint8_t i, len = strlen(rx);
    for(i = 0; i < len; i++)
    {
        //Add new received characters
        message_assemble(rx[i]);
    }
    rx[0] = '\0'; //this is required to 'clear' the rx buffer 
    #endif
    
    //Find out if there is a message to be processed
    do
    {
        if(!ring_empty_p(&message_rx_buffer))
        {
            //Get message
            message_t m;
            ring_read(&message_rx_buffer, m, sizeof(message_t));
            //Find out what message type it is
            message_packet_t msg = message_peek(m);
            //Call the message handler to process the message
            if(devbot_protocol_message_handler[msg] != NULL)
                devbot_protocol_message_handler[msg](m);
        }
        else
            break;
    } while(message_process_bulk);
    
    //Find out if there are messages to be sent
    do
    {
        if(!ring_empty_p(&message_tx_buffer))
        {
            //Get message
            message_t m;
            ring_read(&message_tx_buffer, m, sizeof(message_t));
            //Send message
            devbot_protocol_transmit((char *) m);
        }
        else
            break;
    } while(message_process_bulk);
}

/*
 * Private functions
 */

static uint8_t generate_checksum(const char *string)
{
    uint8_t checksum = 0;
    
    while(*string)
        checksum ^= *string++;
        
    return checksum;
}

static bool validate_message(char *string, uint8_t checksum)
{
    if(generate_checksum(string) == checksum)
        return true;
    else
        return false;
}

static message_packet_t message_peek(message_t m)
{
    return m[0] - 65;
}

static void message_assemble(unsigned char c)
{
    static char buffer[MAX_MESSAGE_LENGTH];
    static uint8_t index = 0;
    static bool message = true;
    static uint8_t checksum = 0;
    static uint8_t i = 0;
    
    if(c == '$')
    {
        message = true; index = 0; checksum = 0;
    }
    else if(c == '*')
    {
        message = false;
        i = 0;
    }
    else if(!message)
    {        
        if(i == 0) { checksum = 16*ASCIIHEX_TO_UINT(c); i++; }
        else if(i == 1) 
        { 
            checksum += ASCIIHEX_TO_UINT(c); i++;
            buffer[index] = '\0';
            bool res = validate_message(buffer, checksum);
            if(res && strlen(buffer) > 0)
            {
                ring_write_safe(&message_rx_buffer, &buffer, sizeof(message_t));
            }
        }        
    } 
    else
    {
        buffer[index++] = c;
        if(index >= MAX_MESSAGE_LENGTH) index = 0; //Should not have to do this, but required for protection to prevent overflow
    }
}

/*static void devbot_protocol_test(void)
{
    #define rprintf printf
    #define rprintfStr uart_puts
    //Read test
    message_t m = "A,C,-23,243,-7983,9990,-99999,876541,67.34,k,hello";
    int8_t a; uint8_t b; int16_t c; uint16_t d; int32_t e; uint32_t f; float g; char h; char i[10];
    message_packet_t msg; message_setting_t setting;
    msg = message_peek(m); devbot_protocol_read_header(m, &setting);
    rprintf("msg=%d,setting=%c,", msg, setting); 
    devbot_protocol_read(m, &a, MSG_INT8); devbot_protocol_read(m, &b, MSG_UINT8); devbot_protocol_read(m, &c, MSG_INT16); 
    devbot_protocol_read(m, &d, MSG_UINT16); devbot_protocol_read(m, &e, MSG_INT32); devbot_protocol_read(m, &f, MSG_UINT32);
    devbot_protocol_read(m, &g, MSG_FLOAT); devbot_protocol_read(m, &h, MSG_CHAR); devbot_protocol_read(m, i, MSG_STRING); 
    rprintf("a=%d,b=%d,c=%d,d=%d,e=%d,f=%d,g=%f,h=%c,i=", a,b,c,d,e,f,g,h); rprintfStr((char *) i);
    //ATmega324 outputs: msg=0,setting=C,a=-23,b=243,c=-7983,d=9990,e=31073,f=0,g=f,h=ý,i=hello
    //pic32 outputs: msg=0,setting=C,a=-23,b=243,c=-7983,d=9990,e=-99999,f=876541,g=67.339996,h=k,i=hello
    
    //Write test
    int8_t a = -23; uint8_t b = 243; int16_t c = -7983; uint16_t d = 9990; int32_t e = -99999;
    uint32_t f = 876541; float g = 67.34; char h = 'k'; char i[] = "hello"; message_t m;
    devbot_protocol_write_header(m, MSG_POSITION, 'C');
    devbot_protocol_write(m, &a, MSG_INT8); devbot_protocol_write(m, &b, MSG_UINT8); devbot_protocol_write(m, &c, MSG_INT16);
    devbot_protocol_write(m, &d, MSG_UINT16); devbot_protocol_write(m, &e, MSG_INT32);
    devbot_protocol_write(m, &f, MSG_UINT32); devbot_protocol_write(m, &g, MSG_FLOAT);
    devbot_protocol_write(m, &h, MSG_CHAR); devbot_protocol_write(m, i, MSG_STRING);
    rprintfStr((char *) m);
    //ATmega324 outputs: A,C,-23,243,-7983,9990,31073,24573,?,k,hello
    //pic32 outputs: A,C,-23,243,-7983,9990,-99999,876541,67.340,k,hello
}*/
