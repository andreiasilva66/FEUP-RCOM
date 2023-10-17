// Link layer protocol implementation

#include "link_layer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#define BIT(n) 1 << n

enum linkState{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
};

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FLAG 0x7E
#define A_TX 0x03
#define A_RC 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_RR(n) ((n << 7) | 0x05)
#define C_REJ(n) ((n << 7) | 0x01)
#define C_DISC 0x0B
#define BAUDRATE B38400


int txStateMachine(enum linkState *state, int fd) {
    
    unsigned char byte;
    
    while(*state != STOP){
        if(read(fd, &byte, 1) > 0){
            switch (*state){
            case START:
                if(byte == FLAG) 
                    *state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if(byte == A_RC) 
                    *state = A_RCV;
                else if(byte != FLAG)
                    *state = START;
                break;
            
            case A_RCV:
                if(byte == C_UA) 
                    *state = C_RCV;
                else if(byte == FLAG)
                    *state = FLAG;
                else
                    *state = START;
                break;

            case C_RCV:
                if(byte == (C_UA ^ A_RC)) 
                    *state = BCC_OK;
                else if(byte == FLAG)
                    *state = FLAG;
                else
                    *state = START;
                break;

            case BCC_OK:
                if(byte == FLAG) 
                    *state = STOP;
                else
                    *state = START;
                break;

            default:
                break;
            }
        }
    }
    return 0;
}


int rcStateMachine(enum linkState *state, int fd) {
    
    unsigned char byte;
    
    while(*state != STOP){
        if(read(fd, &byte, 1) > 0){
            switch (*state){
            case START:
                if(byte == FLAG) 
                    *state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if(byte == A_TX) 
                    *state = A_RCV;
                else if(byte != FLAG)
                    *state = START;
                break;
            
            case A_RCV:
                if(byte == C_SET) 
                    *state = C_RCV;
                else if(byte == FLAG)
                    *state = FLAG;
                else
                    *state = START;
                break;

            case C_RCV:
                if(byte == (C_SET ^ A_TX)) 
                    *state = BCC_OK;
                else if(byte == FLAG)
                    *state = FLAG;
                else
                    *state = START;
                break;

            case BCC_OK:
                if(byte == FLAG) 
                    *state = STOP;
                else
                    *state = START;
                break;

            default:
                break;
            }
        }
    }
    return 0;
}

int connectSerialPort(char serialPort[50]){

    int fd = open(serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0){
        perror(serialPort);
        return -1;
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if(tcgetattr(fd, &oldtio) == -1){
        perror("tcgettattr");
        return -1;
    }

    // Clean the struct
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 5;

    if(tcsetattr(fd, TCSANOW, &newtio) == -1){
        perror("tcsettattr");
        return -1;
    }

    return fd;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    enum linkState state = START;

    int fd = connectSerialPort(connectionParameters.serialPort);
    if(fd < 0)
        return -1;

    switch (connectionParameters.role)
    {
    case LlTx:

        while(connectionParameters.nRetransmissions > 0){
            unsigned char frame[5] = {FLAG, A_TX, C_SET, (A_TX ^ C_SET), FLAG};
            write(fd, frame, 5);
            txStateMachine(&state, fd);
            connectionParameters.nRetransmissions--;
        }
        
        if (state != STOP) return -1;
        break;

    case LlRx:
        rcStateMachine(&state, fd);
        unsigned char frame[5] = {FLAG, A_RC, C_UA, (A_RC ^ C_UA), FLAG};
        write(fd, frame, 5);
        break;
    
    default:
        return -1;
        break;
    }

    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
