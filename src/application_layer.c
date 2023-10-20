// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // LinkLayer linkLayer;
    // strcpy(linkLayer.serialPort,serialPort);
    // if(*role == "tx") linkLayer.role = LlTx;
    // else if(*role == "rx") linkLayer.role = LlRx;
    // else printf("Invalid role\n");
    // linkLayer.baudRate = baudRate;
    // linkLayer.nRetransmissions = nTries;
    // linkLayer.timeout = timeout;

    // if(llopen(linkLayer) < 0){
    //     printf("Connection error\n");
    // }

    // if(role == "tx"){

    //     FILE* file = fopen(filename, "rb");
    //     if(file == NULL){
    //         printf("Couldn't read file");
    //     }

    //     // Get size of file

    //     int fileStart = ftell(file);
    //     fseek(file, 0L, SEEK_END);
    //     int fileSize = ftell(file) - fileStart;
    //     fseek(file, fileStart, SEEK_SET);

        
    // }
}
