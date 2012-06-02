#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "misc.h"

int readjpeg(int sock, unsigned char **buf,struct frame_t *headerframe,struct client_t *message,int statOn)
{
 	
	int byteread,bytewrite;
	
	bytewrite = write_sock(sock,(unsigned char*)message,sizeof(struct client_t));
	// is sleeping ?
	if ((byteread= read_sock(sock,(unsigned char*)headerframe,sizeof(struct frame_t))) < 0){
	printf("Seem server is gone !! try later \n");
	goto error;
	}

	if(statOn)
		printf (" key %s nb %d width %d height %d times %dms size %d \n",headerframe->header,
		headerframe->nbframe,headerframe->w,headerframe->h,headerframe->deltatimes,headerframe->size);
	if(headerframe->size && !headerframe->wakeup){
	//if(headerframe->size){
			*buf=(unsigned char*) realloc(*buf,headerframe->size);
			if((byteread = read_sock(sock,*buf,headerframe->size)) < 0){
			printf("Seem server is gone !! try later \n");
			goto error;}
		}
		//printf("buf read %d \n",byteread);
	if(headerframe->acknowledge)
			reset_callbackmessage(message);
		usleep(5000);
	return ((headerframe->wakeup)?0:(headerframe->size));
	//return (headerframe->size);
error:
return -1;
}

void init_callbackmessage(struct client_t* callback)
{    char key[4] ={'O','K','\0','\0'} ;
	int x = 128;
	int y = 128;
	unsigned char sleepon=0;
	unsigned char bright=0;
	unsigned char contrast =0;
	unsigned char exposure = 0;
	unsigned char colors = 0;
	unsigned char size = 0;
	unsigned char fps = 0;
	memcpy(callback->message,key,4);
	callback->x = x;
	callback->y = y;
	callback->updobright=bright;
	callback->updocontrast=contrast;
	callback->updoexposure = exposure;
	callback->updocolors = colors;
	callback->sleepon=sleepon;
	callback->updosize = size;
	callback->fps = fps;	
}

void reset_callbackmessage(struct client_t* callback)
{    	
	callback->updobright= 0;
	callback->updocontrast= 0;
	callback->updoexposure = 0;
	callback->updocolors = 0;
	callback->sleepon= 0;
	callback->updosize = 0;
	callback->fps = 0;	
}

