/*
 *  A plugin for interfacing network cameras (using spcaserv) to Player
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
 
#include <linux/videodev.h>
#include <iostream>
#include <stddef.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>

#include <libplayercore/playercore.h>

extern "C"
{
#include "jconfig.h"
#include "avilib.h"
#include "dpsh.h"
#include "utils.h"
#include "tcputils.h"
#include "spcaframe.h"
#include "misc.h"
}

#define SPCA_INCREASE_BRIGHTNESS        'n'
#define SPCA_DECREASE_BRIGHTNESS        'b'
#define SPCA_INCREASE_CONTRAST          'x'
#define SPCA_DECREASE_CONTRAST          'w'

using namespace std;
class spcaclient : public ThreadedDriver
{
public:
    spcaclient(ConfigFile* cf, int section);

    virtual int ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data);

private:
    virtual void Main();
    virtual int MainSetup();
    virtual void MainQuit();
    
    void WriteCameraData(IplImage *img);
    void adjust(char key);
    IplImage *get_frame(void);

    player_devaddr_t outcam_addr;    
    const char *ip_address;
    int port;
    struct frame_t *headerframe;
    struct client_t *messcallback;
    int sock_client;
};

//Initialization function
Driver* spcaclient_Init( ConfigFile* cf, int section)
{
    return ((Driver*) (new spcaclient( cf, section)));
}

//Driver registration function
void spcaclient_Register(DriverTable* table)
{
    table->AddDriver("spcaclient", spcaclient_Init);
}

extern "C"
{
    int player_driver_init(DriverTable* table)
    {
        spcaclient_Register(table);
        return(0);
    }
}

spcaclient::spcaclient( ConfigFile* cf, int section)
    : ThreadedDriver(cf, section)
{
    memset(&(this->outcam_addr), 0, sizeof outcam_addr);

    if(cf->ReadDeviceAddr(&(this->outcam_addr), section, "provides", PLAYER_CAMERA_CODE, -1, NULL))
    {
        PLAYER_ERROR("failed to setup output camera");
        this->SetError(-1);
        return;
    } 
    else if(this->AddInterface(this->outcam_addr))
    {
        PLAYER_ERROR("failed to register output camera");
        this->SetError(-1);
        return;
    }
    
    this->port = cf->ReadInt(section, "port", 7070);
    this->ip_address = cf->ReadString(section, "ip_address", NULL);
    if(this->ip_address == NULL)
    {
        PLAYER_ERROR("no ip address specified");
        this->SetError(-1);
        return;
    }
}

//Set up the device (called by server thread)
int spcaclient::MainSetup()
{    
    this->sock_client = open_clientsock((char*)this->ip_address, this->port);
    this->headerframe = (struct frame_t*)malloc(sizeof(struct frame_t));
    this->messcallback = (struct client_t*)malloc(sizeof(struct client_t));
    init_callbackmessage(this->messcallback);
    
    //Do a dummy read. For some reason if this is not done on 320x240, program segfaults!
    IplImage *dummy = this->get_frame();
    cvReleaseImage(&dummy);
    
    return 0;
}

//Shutdown the device (called by server thread)
void spcaclient::MainQuit()
{
    close(this->sock_client);
    free(this->messcallback);
    free(this->headerframe);
}

void spcaclient::Main()
{
    while(true)
    {
        pthread_testcancel();
        
        //Process requests
        this->ProcessMessages();
        //Get frame 
        IplImage *img = this->get_frame();
        //Write undistorted image (outImage) to output camera
        this->WriteCameraData(img);
        
        cvReleaseImage(&img);
        usleep(100000); //needs to be large!
    }
}

int spcaclient::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{
    //TODO: process messages
    return 0;
}

void spcaclient::WriteCameraData(IplImage *img)
{
    player_camera_data_t * outCameraData;

    if(img == NULL)
        return;

    outCameraData = reinterpret_cast<player_camera_data_t *>(malloc(sizeof(player_camera_data_t)));
    assert(outCameraData);

    outCameraData->width = img->width;
    outCameraData->height = img->height;
    outCameraData->bpp = 24;
    outCameraData->format = PLAYER_CAMERA_FORMAT_RGB888;
    outCameraData->compression = PLAYER_CAMERA_COMPRESS_RAW;
    outCameraData->fdiv = 0;
    outCameraData->image_count = img->imageSize;
    outCameraData->image = NULL;
    if(outCameraData->image_count)
    {
        outCameraData->image = reinterpret_cast<uint8_t *>(malloc(outCameraData->image_count));
        // Copy in the pixels
        memcpy(outCameraData->image, img->imageData, outCameraData->image_count);
    }
    this->Publish(this->outcam_addr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, reinterpret_cast<void *>(outCameraData), 0, NULL, false); //this call should also free outCameraData and outCameraData->image
}

void spcaclient::adjust(char key)
{
    switch(key)
    {
        case SPCA_INCREASE_BRIGHTNESS: messcallback->updobright = 2; break;
        case SPCA_DECREASE_BRIGHTNESS: messcallback->updobright = 1; break;    
        case SPCA_INCREASE_CONTRAST: messcallback->updocontrast = 2; break;
        case SPCA_DECREASE_CONTRAST: messcallback->updocontrast = 1; break;
        default: break;
    }
}

IplImage *spcaclient::get_frame(void)
{
    IplImage *img = NULL;    
    
    unsigned char *buf = NULL;
    unsigned char *picture = NULL;
    
    int jpegsize, width, height;
    
    //Attempt to read a jpeg image
    if((jpegsize = readjpeg(sock_client, &buf, headerframe, messcallback, 0)) < 0)
    {
        cout << "error while reading image from socket\n";
        return img;    
    }
    width = headerframe->w;
    height = headerframe->h;
    
    //Decode jpeg image
    jpeg_decode(&picture,buf,&width,&height);

    //Setup opencv image
    img = cvCreateImage(cvSize(width, height), 8, 3); //TODO: read depth and number of channels
    img->imageData = (char *) picture;
    
    if(buf) free(buf);
    if(picture) free(picture);
        
    return img;
}

