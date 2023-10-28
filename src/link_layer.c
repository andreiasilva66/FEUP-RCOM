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
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define C_N0 0x00
#define C_N1 0x40
#define TX 0
#define RX 1 

#define C_RR(n) ((n << 7) | 0x05)
#define C_REJ(n) ((n << 7) | 0x01)
#define C_DISC 0x0B
#define ESC 0x7D
#define BAUDRATE B38400
#define STUFF_XOR 0x20

int alarmEnabled = FALSE;
int alarmCount = 0;

unsigned char tramaTx = 0;
unsigned char tramaRx = 0;

// Alarm function handler
void alarmHandler(int signal){
    alarmEnabled = TRUE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int nRetransmissions = 0;
int timeout = 0;
int fd;
int role;

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
                    *state = FLAG_RCV;
                else
                    *state = START;
                break;

            case C_RCV:
                if(byte == (C_UA ^ A_RC)) 
                    *state = BCC_OK;
                else if(byte == FLAG)
                    *state = FLAG_RCV;
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
    printf("tansmissor recebeu a primeira\n");
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
                    *state = FLAG_RCV;
                else
                    *state = START;
                break;

            case C_RCV:
                if(byte == (C_SET ^ A_TX)) 
                    *state = BCC_OK;
                else if(byte == FLAG)
                    *state = FLAG_RCV;
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
    printf("recetor recebeu a priemria\n");
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
    newtio.c_cc[VTIME] = 5;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if(tcsetattr(fd, TCSANOW, &newtio) == -1){
        perror("tcsettattr");
        return -1;
    }

    return 1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    enum linkState state = START;
    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    if(connectSerialPort(connectionParameters.serialPort) < 0 || fd <0)
        return -1;

    switch (connectionParameters.role){
        case LlTx:{
            role = TX;
            (void) signal(SIGALRM, alarmHandler);
            while(connectionParameters.nRetransmissions > 0 && state != STOP){
                unsigned char frame[5] = {FLAG, A_TX, C_SET, (A_TX ^ C_SET), FLAG};
                write(fd, frame, 5);
                alarm(connectionParameters.timeout);
                alarmEnabled = FALSE;
                txStateMachine(&state);
                connectionParameters.nRetransmissions--;
            }
            //alarm(0);
            if (state != STOP) return -1;
            break;
        }
        case LlRx:{
            role = RX;
            rcStateMachine(&state);
            unsigned char frame[5] = {FLAG, A_RC, C_UA, (A_RC ^ C_UA), FLAG};
            write(fd, frame, 5);
            //if (state != STOP) return -1;
            break;
        }
        default:
            return -1;
            break;
        }

    return 1;
}


int getCtrlInfo(){
    unsigned char byte;
    unsigned char c;
    enum linkState state = START;

    printf("inicio getctrlinfo\n");
    while(state != STOP){
        printf("antes do read\n");
        if(read(fd, &byte, 1) > 0){
            printf("leu %x\n",byte);

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
                    if(byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || C_DISC) {
                        state = C_RCV;
                        c = byte;
                    }
                    else if(byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;

                case C_RCV:
                    if(byte == (c ^ A_RC)) 
                        state = BCC_OK;
                    else if(byte == FLAG)
                        state = FLAG_RCV;
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
    printf("fim getctrlinfo\n");

    return c;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    printf("is writing\n");
    int frameSize = bufSize+6;
    unsigned char* frame = malloc(frameSize);
    frame[0] = FLAG;
    frame[1] = A_TX;
    if(tramaTx % 2 == 0) frame[2] = C_N0;
    else frame[2] = C_N1;
    frame[3] = (A_TX ^ frame[2]);

    unsigned char bcc2 = 0;
    for(int i = 0; i < bufSize; i++){
        printf("%x\n", buf[i]);
        bcc2 ^= buf[i];
    }

    int frameCount = 4;
    for(int i = 0; i < bufSize; i++){
        if(buf[i] == FLAG || buf[i] == ESC){
            frame = realloc(frame, ++frameSize);
            frame[frameCount++] = ESC;
            frame[frameCount++] = buf[i] ^ STUFF_XOR;
        }
        else{
            frame[frameCount++] = buf[i];
        }
    }

    frame[frameCount++] = bcc2;
    frame[frameCount] = FLAG;

    int transmissionsDone = 0;
    int accepted,rejected = 0;

    while( transmissionsDone <= nRetransmissions ){
        alarmEnabled = FALSE;
        alarm(timeout);
        accepted = 0;
        rejected = 0;
        while(alarmEnabled == FALSE && accepted == 0 && rejected == 0){
            write(fd,frame, frameSize);
            unsigned char c = getCtrlInfo();
            if(c == 0){
                continue;
            }
            else if(c == C_REJ0 || c == C_REJ1){
                rejected = 1;
            }
            else if(c == C_RR0 || c == C_RR1){
                accepted = 1;
                tramaTx = (tramaTx+1) % 2;
            }else continue;
        }
        if(accepted) break;
        transmissionsDone++;
    }
    printf("is writen\n");
    if(accepted)return bufSize+6;
    else{
        llclose(fd);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    printf("entrou no read\n");
    unsigned char byte, field;
    int i = 0;
    enum linkState state = START;
    while(state != STOP){
        if(read(fd, &byte, 1) > 0){
            //printf("leu %x\n", byte);

            switch (state){
            case START:
                if(byte == FLAG){ 
                    state = FLAG_RCV;
                    printf("flag\n");}
                break;

            case FLAG_RCV:
                if(byte == A_TX){ 
                    state = A_RCV;
                    printf("a\n");}
                else if(byte != FLAG){
                    state = START;
                    printf("flag\n");}
                break;
            
            case A_RCV:
                if(byte == C_N0 || byte == C_N1){
                    state = C_RCV;
                    field = byte;
                    printf("c\n");
                }
                    
                else if(byte == FLAG){
                    printf("flag\n");
                    state = FLAG_RCV;
                    }
                else{
                    state = START;
                    printf("start\n");
                    }
                break;

            case C_RCV:
                if(byte == (field ^ A_TX)){ 
                    state = READING_DATA;
                    printf("reading\n");}
                else if(byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case READING_DATA:
                    if (byte == ESC) state = FOUND_DATA;
                    else if (byte == FLAG){
                        printf("found a flag\n");

                        unsigned char bcc2 = packet[i-1];
                        i--;
                        packet[i] = '\0';
                        unsigned char acc = packet[0];

                        for (unsigned int j = 1; j < i; j++)
                            acc ^= packet[j];

                        if (bcc2 == acc){
                            printf("state final\n");
                            state = STOP;
                            int c;
                            if(tramaRx % 2 == 0) c = C_RR0;
                            else c = C_RR1;
                            unsigned char frame[5] = {FLAG, A_RC, c, (A_RC ^ c), FLAG};
                            write(fd, frame, 5);
                            tramaRx = (tramaRx + 1)%2;
                            return i; 
                        }
                        else{
                            printf("Error: retransmition\n");
                            int c;
                            if(tramaRx % 2 == 0) c = C_REJ0;
                            else c = C_REJ1; 
                            unsigned char frame[5] = {FLAG, A_RC, c, (A_RC ^ c), FLAG};
                            write(fd, frame, 5);
                            return -1;
                        };

                    }
                    else{
                        printf("%x\n", byte);
                        
                        packet[i++] = byte;
                    }
                    break;
            case FOUND_DATA:
                printf("%x\n", (byte ^ STUFF_XOR));
                state = READING_DATA;
                packet[i++] = byte ^ STUFF_XOR;
                break;

            default:
                break;
            }
        }
    }
    printf("chegou ao final de llread\n");
    return 1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics){
    enum linkState state = START;
    unsigned char byte;
    (void) signal(SIGALRM, alarmHandler);

    if(role == TX){
        int transmissionsDone = 0;
        while(nRetransmissions > transmissionsDone && state != STOP){
            unsigned char FRAME[5] = {FLAG, A_TX, C_DISC, A_TX ^ C_DISC, FLAG};
            write(fd, FRAME, 5);
            printf("escreveu frame do llclose\n");
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
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;

                    case C_RCV:
                        if(byte == (C_DISC ^ A_RC)) 
                            state = BCC_OK;
                        else if(byte == FLAG)
                            state = FLAG_RCV;
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
            if(state != STOP) return -1;
        }
    }
    else{
        while(state != STOP){
            printf("antes do read\n");
                if(read(fd, &byte, 1) > 0){
                    printf("leu isto %x\n", byte);
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
                        if(byte == C_DISC) 
                            state = C_RCV;
                        else if(byte == FLAG)
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;

                    case C_RCV:
                        if(byte == (C_DISC ^ A_TX)) 
                            state = BCC_OK;
                        else if(byte == FLAG)
                            state = FLAG_RCV;
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
        printf("chega ao estado final da ultima no rx\n");
        unsigned char FRAME[5] = {FLAG, A_RC, C_DISC, A_RC ^ C_DISC, FLAG};
        write(fd, FRAME, 5);
    }

    alarm(0);

    if(close(fd) < 0)
        return -1;;

    return 1;
}
