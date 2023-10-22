// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename){
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort,serialPort);
    if(*role == "tx") linkLayer.role = LlTx;
    else if(*role == "rx") linkLayer.role = LlRx;
    else printf("Invalid role\n");
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;
    if(llopen(linkLayer) < 0){
        printf("Connection error\n");
    }

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

    int bufSize = MAX_PAYLOAD_SIZE -1;
    unsigned char buf[bufSize + 1];

    switch (linkLayer.role){
        case LlRx:{
            int packetSize = -1;
            unsigned char* packet = (unsigned char)malloc(MAX_PAYLOAD_SIZE);
            if((packetSize = llread(packet)) < 0){
                printf("Error reading packet\n");
                exit(-1);
            }

            long int fileSize =0;
            unsigned char fileSizeBytes = packet[2];
            unsigned char fileSizeParse[fileSizeBytes];

            memcpy(fileSizeParse, packet + 3, fileSizeBytes);

            for(int i = 0; i < fileSizeBytes; i++){
                fileSize |= (fileSizeParse[fileSizeBytes - i - 1] << (8 * i));
            }
            

            unsigned char fileNameBytes = packet[3+fileSizeBytes+1];
            unsigned char fileName = (unsigned char)malloc(fileNameBytes);

            memcpy(fileName, packet + 3 + fileSizeBytes + 2, fileNameBytes);

            FILE* fileOut = fopen((char *) filename, "wb+");

            long int readAlready = 0;
            int readingNow;
            while(readAlready < fileSize){
                while((readingNow = llread(packet)) < 0){
                    readAlready += readingNow;
                }
                if(packet[0] != 3){
                    unsigned char *buf = (unsigned char*)malloc(packetSize);
                    memcpy(buf, packet + 4, readingNow -4);

                    fwrite(buf, sizeof(unsigned char), readingNow - 4, fileOut);
                    free(buf);
                }
            }
            fclose(fileOut);
            break;
            }
        case LlTx:{
            FILE* file = fopen(filename, "rb");
            if(file == NULL){
                printf("Couldn't read file");
                exit(-1);
            }

            int prev = ftell(file);
            fseek(file, 0L, SEEK_END);
            long int fileSize = ftell(file) - prev;
            fseek(file, prev, SEEK_SET);

            unsigned int cpSize;
            unsigned char *ctrlPacketStart = getCtrlPacket(2, filename, fileSize);
            if(llwrite(ctrlPacketStart, cpSize) < 0){
                printf("Error sending control packet\n");
                exit(-1);
            }

            bool which = FALSE;
            unsigned char* buf = (unsigned char*)malloc(MAX_PAYLOAD_SIZE);
            long int j = fileSize;

            while(j != 0){
                int readingNow = fread(buf, sizeof(unsigned char), bufSize, file);
                int lenPacket;
                unsigned char* packet = getDataPacket((int)which, buf, MAX_PAYLOAD_SIZE, &lenPacket);

                if(llwrite(packet, lenPacket) < 0){
                    printf("Error sending data packet\n");
                    exit(-1);
                }
                j-= readingNow;
                which = !which;
            }

            unsigned char* endCtrl = getCtrlPacket(3, filename, fileSize);

            if(llwrite(endCtrl, fileSize + 5) < 0){
                printf("Error sending end control packet\n");
                exit(-1);
            }

            llclose(0);
            break;
            }
        default:
            break;
    }
    

}

unsigned char* getCtrlPacket(int which, const char* fileName, long int length){
    const int length1 = (int)ceil(log2f((float)length)/8.0);
    const int length2 = strlen(fileName);
    long int fileSize = 3 + length1 + 2 + length2;

    unsigned char* ctrlPacket = (unsigned char*)malloc(fileSize);

    ctrlPacket[0] = which;
    ctrlPacket[1] = 0;
    ctrlPacket[2] = length1;

    for(int i = 0; i < length1; i++){
        ctrlPacket[2 + length1 - i] = length & 0xFF;
        length >>= 8;
    }

    ctrlPacket[3 + length1] = 1;
    ctrlPacket[4 + length1] = length2;

    memcpy(ctrlPacket + length1 + 4, fileName, length2);

    return ctrlPacket;
}

unsigned char* getDataPacket(int which, unsigned char* data, int dataSize, int *packetLen) {
	*packetLen = dataSize + 4;
	
	unsigned char* packet = (unsigned char*) malloc(packetLen);
	
	packet[0] = 1;
	packet[1] = which;
	packet[2] = dataSize >> 8 & 0xFF;
	packet[3] = dataSize & 0xFF;
	
	memcpy(packet + 4, data, dataSize);
	
	return packet;
}