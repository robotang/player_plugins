#ifndef MISC_H
#define MISC_H

#include "spcaframe.h"

int readjpeg(int sock, unsigned char **buf,struct frame_t *headerframe,struct client_t *message,int statOn);
void init_callbackmessage(struct client_t* callback);
void reset_callbackmessage(struct client_t* callback);

#endif

