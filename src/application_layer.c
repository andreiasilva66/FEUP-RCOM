// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#define FILE_SIZE 0
#define FILE_NAME 1
#define DATA_PCKT 1
#define START_PCKT 2
#define END_PCKT 3


int sendCtrlPacket(int which, const char* fileName, long int length){
    int numBits = sizeof(int) * 8 - __builtin_clz(length);
    size_t filelength = (numBits+7)/8.0;
    size_t filename_length = strlen(fileName) + 1;
    long int cp_size = 3 + filelength + 2 + filename_length;

    unsigned char* ctrlPacket = (unsigned char*)malloc(cp_size);

    int i = 0;
    ctrlPacket[i++] = which;
    ctrlPacket[i++] = 0;
    memcpy(ctrlPacket+i, &filelength, sizeof(size_t));
    i += sizeof(size_t);
    ctrlPacket[i++] = 1; 
    memcpy(ctrlPacket + i, fileName, filename_length);
    
    // for(int i = 0; i < filelength; i++){
    //     ctrlPacket[2 + filelength - i] = length & 0xFF;
    //     length >>= 8;
    // }

    // ctrlPacket[3 + filelength] = 1;
    // ctrlPacket[i] = filename_length;

    if(llwrite(ctrlPacket,cp_size) < 0){
        printf("falha ao enviar control packet");
        return -1;
    }

    return 1;
}

int readCtrlPacket(unsigned char control, unsigned char* buf, size_t* file_size, char* filename){
    
    int bufSize;
    if((bufSize = llread(buf)) < 0){
        printf("Error reading control packet\n");
        return -1;
    }

    if(buf[0] != control){
        printf("Invalid control packet\n");
        return -1;
    }

    int i = 1;
    unsigned char type;
    while(i < bufSize){
        type = buf[i++];
        if(type == 0){
            printf("viu o size\n");
            *file_size = buf[i];
            i += sizeof(size_t);
        }
        else if(type == 1){
            printf("viu o name\n");
            *filename = buf[i];
            i += *filename;
        }
        else{
            printf("Invalid control packet type\n");
            return -1;
        }
    }
    return 1;
}


int sendDataPacket(unsigned char* data, int dataSize) {
	size_t packetLen = dataSize + 3;
	
	unsigned char* packet = (unsigned char*) malloc(packetLen);
	
	packet[0] = DATA_PCKT;
	packet[1] = (unsigned char) (dataSize/256);
	packet[2] = (unsigned char) (dataSize % 256);
	
	memcpy(packet + 3, data, dataSize);

    if(llwrite(packet,packetLen) < 0){
        printf("Error sending data packet\n");
        return -1;
    }
	
	return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename){
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort,serialPort);
    if(strcmp(role,"tx") == 0) linkLayer.role = LlTx;
    else if(strcmp(role,"rx") == 0) linkLayer.role = LlRx;
    else printf("Invalid role\n");
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;
    if(llopen(linkLayer) < 0){
        printf("Connection error\n");
    }

    switch (linkLayer.role){
        case LlRx:{
            size_t packetSize;
            char receivedFilename[0xff];
            unsigned char* buf = (unsigned char*)malloc(MAX_PAYLOAD_SIZE);
            printf("antes de ler start packet\n");

            if(readCtrlPacket(START_PCKT, buf, &packetSize, receivedFilename) < 0){
                printf("Error readind control file");
                exit(-1);
            }

            printf("depois do read control packet\n");

            // long int fileSize =0;
            // unsigned char fileSizeBytes = packet[2];
            // unsigned char fileSizeParse[fileSizeBytes];

            // memcpy(fileSizeParse, packet + 3, fileSizeBytes);

            // for(int i = 0; i < fileSizeBytes; i++){
            //     fileSize |= (fileSizeParse[fileSizeBytes - i - 1] << (8 * i));
            // }
            

            // unsigned char fileNameBytes = packet[3+fileSizeBytes+1];
            // unsigned char *fileName = (unsigned char*)malloc(fileNameBytes);

            // memcpy(fileName, packet + 3 + fileSizeBytes + 2, fileNameBytes);

            FILE* fileOut = fopen((char *) filename, "wb+");

            if(fileOut == NULL){
                printf("Error opening file.\n");
                exit(-1);
            }

            printf("abre o file\n");
            free(buf);
            int dataSize;
            while((dataSize = llread(buf)) >= 0){

                if(buf[0] == END_PCKT){
                    printf("recebeu packet final\n");
                    break;
                }
                else{
                    printf("esta a escrever no file\n");
                    fwrite(buf, 1, buf[1] * 256 + buf[2], fileOut);
                }

                //free(buf);
            }
            free(buf);
            fclose(fileOut);
            printf("fecha o file\n");
            printf("antes do llclose\n");
            if(llclose(FALSE) < 0) exit(-1);
            printf("depois do llclose\n");
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
            printf("antes enviar start packet\n");

            if(sendCtrlPacket(START_PCKT, filename, fileSize) == -1){
                printf("Error sending control packet\n");
                exit(-1);
            };

            printf("depois de enviar start packet\n");

            unsigned char* buf = (unsigned char*)malloc(MAX_PAYLOAD_SIZE-3);

            int dataSize;
            while((dataSize = fread(buf, 1, MAX_PAYLOAD_SIZE-3, file)) > 0){
                
                if(sendDataPacket(buf, dataSize)){
                    exit(-1);
                };
            }
            fclose(file);

            printf("depois de enviar todos os data packet\n");

            if(sendCtrlPacket(END_PCKT,filename,fileSize) == -1){
                printf("Error sending control packet\n");
                exit(-1);
            }

            printf("depois de enviar end packet\n");

            if(llclose(0) < 0) exit(-1);
            printf("depois do llclose\n");
            break;
            }
        default:
            break;
    }
    

}