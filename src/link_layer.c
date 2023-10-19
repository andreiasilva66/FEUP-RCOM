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

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}
int txFrameCount = 0;
int rcFrameCount = 0;
int nRetransmissions;
int fd;

int txStateMachine(enum linkState *state) {
    
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
        if(alarmEnabled == FALSE){
            alarm(3);
            alarmEnabled = TRUE;
            sleep(3);
        
        }
    }
    return 0;
}


int rcStateMachine(enum linkState *state) {
    
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

    int fd = connectSerialPort(connectionParameters.serialPort);
    if(fd < 0)
        return -1;

    switch (connectionParameters.role){
        case LlTx:
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

        case LlRx:
            rcStateMachine(&state, fd);
            unsigned char frame[5] = {FLAG, A_RC, C_UA, (A_RC ^ C_UA), FLAG};
            write(fd, frame, 5);
            txStateMachine(&state);
            connectionParameters.nRetransmissions--;
        }
        
        if (state != STOP) return -1;
        break;

    case LlRx:
        rcStateMachine(&state);
        unsigned char frame[5] = {FLAG, A_RC, C_UA, (A_RC ^ C_UA), FLAG};
        write(fd, frame, 5);
        break;
    
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
int llwrite(const unsigned char *buf, int bufSize)
{
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
