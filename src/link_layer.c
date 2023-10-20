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
#include <signal.h>

#define BIT(n) 1 << n

enum linkState{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    READING_DATA,
    FOUND_DATA,
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
#define ESC 0x7D
#define BAUDRATE B38400

int alarmEnabled = FALSE;
int alarmCount = 0;

unsigned char tramaTx = 0;
unsigned char tramaRx = 1;

// Alarm function handler
void alarmHandler(int signal){
    alarmEnabled = TRUE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}
int txFrameCount = 0;
int rcFrameCount = 0;
int nRetransmissions = 0;
int timeout = 0;
int fd;

int txStateMachine(enum linkState *state, int fd) {
    
    unsigned char byte;
    
    while(*state != STOP && alarmCount < 3){
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

    fd = open(serialPort, O_RDWR | O_NOCTTY);

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
int llopen(LinkLayer connectionParameters){
    enum linkState state = START;
    timeout = connectionParameters.timeout;
    int fd = connectSerialPort(connectionParameters.serialPort);
    if(fd < 0)
        return -1;

    switch (connectionParameters.role){
        case LlTx:{
            (void) signal(SIGALRM, alarmHandler);
            while(connectionParameters.nRetransmissions > 0){
                unsigned char frame[5] = {FLAG, A_TX, C_SET, (A_TX ^ C_SET), FLAG};
                write(fd, frame, 5);
                alarm(connectionParameters.timeout);
                alarmEnabled = FALSE;
                txStateMachine(&state, fd);
                connectionParameters.nRetransmissions--;
            }
            if (state != STOP) return -1;
            break;
        }
        case LlRx:{
            rcStateMachine(&state, fd);
            unsigned char frame[5] = {FLAG, A_RC, C_UA, (A_RC ^ C_UA), FLAG};
            write(fd, frame, 5);
            txStateMachine(&state, fd);
            //if (state != STOP) return -1;
            break;
        }
        default:
            return -1;
            break;
        }

    return fd;
}


int getCtrlInfo(){
    unsigned char byte;
    unsigned char c;
    enum linkState state = START;

    while(state != STOP){
        if(read(fd, &byte, 1) > 0){
            switch (state){
                case START:
                    if(byte == FLAG) 
                        state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if(byte == A_RC) 
                        state = A_RCV;
                    else if(byte != FLAG)
                        state = START;
                    break;
                
                case A_RCV:
                    if(byte == C_RR(0) || byte == C_RR(1) || byte == C_REJ(0) || byte == C_REJ(1) || C_DISC) {
                        state = C_RCV;
                        c = byte;
                    }
                    else if(byte == FLAG)
                        state = FLAG;
                    else
                        state = START;
                    break;

                case C_RCV:
                    if(byte == (c ^ A_RC)) 
                        state = BCC_OK;
                    else if(byte == FLAG)
                        state = FLAG;
                    else
                        state = START;
                    break;

                case BCC_OK:
                    if(byte == FLAG) 
                        state = STOP;
                    else
                        state = START;
                    break;

                default:
                    break;
            }
        }
    }
    return c;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    unsigned char frame[bufSize+6];
    frame[0] = FLAG;
    frame[1] = A_TX;
    frame[2] = C_REJ(txFrameCount);
    frame[3] = (A_TX ^ C_REJ(txFrameCount));

    memcpy(frame+4, buf, bufSize);
    unsigned char bcc2 = 0;

    for(int i = 0; i < bufSize; i++){
        bcc2 ^= buf[i];
    }

    frame[bufSize+4] = bcc2;
    frame[bufSize+5] = FLAG;

    int transmissionsDone = 0;

    while( transmissionsDone <= nRetransmissions ){

        write(fd,frame, bufSize+6);
        unsigned char c = getCtrlInfo();

        if ( c == C_RR(0) || c == C_RR(1) ){
            txFrameCount++;
            break;
        }

        transmissionsDone++;
    }

    if( transmissionsDone <= nRetransmissions){
        return 1;
    }
    
    // tem de se fechar antes
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    unsigned char byte, field;
    int i = 0;
    enum linkState state = START;
    while(state != STOP){
        if(read(fd, &byte, 1) > 0){
            switch (state){
            case START:
                if(byte == FLAG) 
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if(byte == A_TX) 
                    state = A_RCV;
                else if(byte != FLAG)
                    state = START;
                break;
            
            case A_RCV:
                if(byte == C_SET){
                    state = C_RCV;
                    field = byte;
                }
                    
                else if(byte == FLAG)
                    state = FLAG;
                else
                    state = START;
                break;

            case C_RCV:
                if(byte == (field ^ A_TX)) 
                    state = READING_DATA;
                else if(byte == FLAG)
                    state = FLAG;
                else
                    state = START;
                break;
            case READING_DATA:
                    if (byte == ESC) state = FOUND_DATA;
                    else if (byte == FLAG){
                        unsigned char bcc2 = packet[i-1];
                        i--;
                        packet[i] = '\0';
                        unsigned char acc = packet[0];

                        for (unsigned int j = 1; j < i; j++)
                            acc ^= packet[j];

                        if (bcc2 == acc){
                            state = STOP;
                            unsigned char frame[5] = {FLAG, A_TX, C_SET, (A_RC ^ C_RR(tramaRx)), FLAG};
                            write(fd, frame, 5);
                            tramaRx = (tramaRx + 1)%2;
                            return i; 
                        }
                        else{
                            printf("Error: retransmition\n");
                            unsigned char frame[5] = {FLAG, A_TX, C_SET, (A_RC ^ C_REJ(tramaRx)), FLAG};
                            write(fd, frame, 5);
                            return -1;
                        };

                    }
                    else{
                        packet[i++] = byte;
                    }
                    break;
            case FOUND_DATA:
                    state = READING_DATA;
                    if (byte == ESC || byte == FLAG) packet[i++] = byte;
                    else{
                        packet[i++] = ESC;
                        packet[i++] = byte;
                    }
                    break;
            default:
                break;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics){
    enum linkState state = START;
    unsigned char byte;
    (void) signal(SIGALRM, alarmHandler);
    while(nRetransmissions != 0 && state != STOP){
        unsigned char FRAME[5] = {FLAG, A_TX, C_DISC, A_TX ^ C_DISC, FLAG};
        write(fd, FRAME, 5);
        alarm(timeout);
        alarmEnabled = FALSE;

        while(alarmEnabled == FALSE && state != STOP){
            if(read(fd, &byte, 1) > 0){
                switch (state){
                case START:
                    if(byte == FLAG) 
                        state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if(byte == A_RC) 
                        state = A_RCV;
                    else if(byte != FLAG)
                        state = START;
                    break;
                
                case A_RCV:
                    if(byte == C_DISC) 
                        state = C_RCV;
                    else if(byte == FLAG)
                        state = FLAG;
                    else
                        state = START;
                    break;

                case C_RCV:
                    if(byte == (C_DISC ^ A_RC)) 
                        state = BCC_OK;
                    else if(byte == FLAG)
                        state = FLAG;
                    else
                        state = START;
                    break;

                case BCC_OK:
                    if(byte == FLAG) 
                        state = STOP;
                    else
                        state = START;
                    break;

                default:
                    break;
                }
            }
        }
    }
    if(state != STOP) return -1;
    unsigned char FRAME[5] = {FLAG, A_TX, C_UA, A_TX ^ C_UA, FLAG};
    write(fd, FRAME, 5);
    return close(fd);
}
