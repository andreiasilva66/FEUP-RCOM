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
#include <time.h>

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
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int nRetransmissions = 0;
int timeout = 0;
int fd;
int role;
int frameI, frameS, frameU, duplicatedFrame, frameRej = 0;
clock_t start_t, end_t = 0;

int txOpenStateMachine(enum linkState *state) {
    
    unsigned char byte;
    int result;
    
    while(*state != STOP){
        if((result = read(fd, &byte, 1)) > 0){
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
        if(result == 0){
            return 0;
        }
    }
    return 1;
}


int rcOpenStateMachine(enum linkState *state) {
    
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
            alarmEnabled = FALSE;
            unsigned char frame[5] = {FLAG, A_TX, C_SET, (A_TX ^ C_SET), FLAG};
            while(connectionParameters.nRetransmissions > 0 && state != STOP){
                if(alarmEnabled == FALSE){
                    write(fd, frame, 5);
                    frameU++;
                    alarmEnabled = TRUE;
                    alarm(connectionParameters.timeout);
                    connectionParameters.nRetransmissions--;
                }
                txOpenStateMachine(&state);
            }
            
            if (state != STOP) return -1;
            frameS++;
            break;
        }
        case LlRx:{
            role = RX;
            rcOpenStateMachine(&state);
            frameU++;
            unsigned char frame[5] = {FLAG, A_RC, C_UA, (A_RC ^ C_UA), FLAG};
            write(fd, frame, 5);
            frameS++;
            break;
        }
        default:
            return -1;
            break;
        }
    start_t = clock();
    return 1;
}


int getCtrlInfo(){
    unsigned char byte;
    unsigned char c;
    enum linkState state = START;
    int result;

    while(state != STOP){
        result = read(fd, &byte, 1); 
        if(result > 0){

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
        else if(result == 0){
            return 0;
        }
        else{
            return -1;
        }
    }

    return c;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    int frameSize = bufSize+6;
    unsigned char* frame = malloc(frameSize);
    frame[0] = FLAG;
    frame[1] = A_TX;
    if(tramaTx % 2 == 0) frame[2] = C_N0;
    else frame[2] = C_N1;
    frame[3] = (A_TX ^ frame[2]);

    unsigned char bcc2 = 0;
    for(int i = 0; i < bufSize; i++){
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
    int accepted = 0;
    alarmEnabled = FALSE;

    while( transmissionsDone <= nRetransmissions ){
        if(alarmEnabled == FALSE){
            write(fd,frame, frameSize);
            frameI++;
            transmissionsDone++;
            alarmEnabled = TRUE;
            alarm(timeout);
        }
        unsigned char c = getCtrlInfo();
        
        if(c == C_RR0 || c == C_RR1){
            accepted = 1;
            frameS++;
            tramaTx = (tramaTx+1) % 2;
            break;
        }
        else if(c == C_REJ0 || c == C_REJ1){
            alarmEnabled = FALSE;
            frameS++;
        }
    }
    free(frame);
    if(accepted){
        return frameSize;
    } 
    else{
        printf("The program exceeded the number of retransmissions\n");
        llclose(0);
        return -1;
    }
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
                if(byte == FLAG){ 
                    state = FLAG_RCV;}
                break;

            case FLAG_RCV:
                if(byte == A_TX){ 
                    state = A_RCV;}
                else if(byte != FLAG){
                    state = START;}
                break;
            
            case A_RCV:
                if(byte == C_N0 || byte == C_N1){
                    state = C_RCV;
                    field = byte;
                }
                    
                else if(byte == FLAG){
                    state = FLAG_RCV;
                    }
                else{
                    state = START;
                    }
                break;

            case C_RCV:
                //int r = rand() % 
                if(byte == (field ^ A_TX)){ 
                    state = READING_DATA;}
                else if(byte == FLAG)
                    state = FLAG_RCV;
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
                            int c;
                            if(tramaRx % 2 == 0) c = C_RR0;
                            else c = C_RR1;
                            unsigned char frame[5] = {FLAG, A_RC, c, (A_RC ^ c), FLAG};
                            write(fd, frame, 5);
                            if((tramaRx % 2 == 0 && field == C_N0) || ( tramaRx % 2 == 1 && field == C_N1))
                                tramaRx = (tramaRx + 1)%2;
                            else{
                                i = 0;
                                duplicatedFrame++;
                            } 
                            frameI++;
                            frameS++;
                            return i;
                        }
                        else{
                            int c;
                            if(tramaRx % 2 == 0) c = C_REJ0;
                            else c = C_REJ1; 
                            unsigned char frame[5] = {FLAG, A_RC, c, (A_RC ^ c), FLAG};
                            write(fd, frame, 5);
                            frameRej++;
                            frameS++;
                            return -1;
                        };

                    }
                    else{
                        packet[i++] = byte;
                    }
                    break;
            case FOUND_DATA:
                state = READING_DATA;
                packet[i++] = byte ^ STUFF_XOR;
                break;

            default:
                break;
            }
        }
    }
    return 1;
}

int txCloseStateMachine(enum linkState *state){
    unsigned char byte;
    int result;
    while(*state != STOP){
        if((result = read(fd, &byte, 1)) > 0){
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
                if(byte == C_DISC) 
                    *state = C_RCV;
                else if(byte == FLAG)
                    *state = FLAG_RCV;
                else
                    *state = START;
                break;

            case C_RCV:
                if(byte == (C_DISC ^ A_RC)) 
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
        else if(result == 0) return 0;
    }
    return 1;
}

void rxStatistics(){
    double time_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
    printf("|==================================================================|\n");
    printf("|                         RECEIVER STATISTICS                      |\n");
    printf("|==================================================================|\n");
    printf("|     INFORMATION FRAMES RECEIVED    |               %i            |\n",frameI);
    printf("|------------------------------------------------------------------|\n");
    printf("|     UNNUMBERED FRAMES RECEIVED     |               %i             |\n",frameU);
    printf("|------------------------------------------------------------------|\n");
    printf("|      SUPERVISION FRAMES SENT       |               %i            |\n",frameS);
    printf("|------------------------------------------------------------------|\n");
    printf("|     DUPLICATED FRAMES RECEIVED     |               %i             |\n",duplicatedFrame);
    printf("|------------------------------------------------------------------|\n");
    printf("|          FRAMES REJECTED           |               %i             |\n",frameRej);
    printf("|------------------------------------------------------------------|\n");
    printf("|         TRANSMITION TIME           |           %f          |\n",time_t);
    printf("|==================================================================|\n");



}


void txStatistics(){
    double time_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
    printf("|==================================================================|\n");
    printf("|                       TRANSMITTER STATISTICS                     |\n");
    printf("|==================================================================|\n");
    printf("|       INFORMATION FRAMES SENT      |               %i            |\n",frameI);
    printf("|------------------------------------------------------------------|\n");
    printf("|       UNNUMBERED FRAMES SENT       |               %i             |\n",frameU);
    printf("|------------------------------------------------------------------|\n");
    printf("|    SUPERVISION FRAMES RECEIVED     |               %i            |\n",frameS);
    printf("|------------------------------------------------------------------|\n");
    printf("|         TRANSMITION TIME           |           %f          |\n",time_t);
    printf("|==================================================================|\n");
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics){
    end_t = clock();
    enum linkState state = START;
    unsigned char byte;
    alarmEnabled = FALSE;

    if(role == TX){
        int transmissionsDone = 0;
        unsigned char frame[5] = {FLAG, A_TX, C_DISC, A_TX ^ C_DISC, FLAG};
        while(nRetransmissions > transmissionsDone){
            if(alarmEnabled == FALSE){
                write(fd, frame, 5);
                transmissionsDone++;
                alarmEnabled = TRUE;
                alarm(timeout);
                frameU++;
            }
            printf("escreveu frame do llclose\n");

            txCloseStateMachine(&state);
            
            if(state == STOP) break;;
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
        frameU++;
        printf("chega ao estado final da ultima no rx\n");
        unsigned char FRAME[5] = {FLAG, A_RC, C_DISC, A_RC ^ C_DISC, FLAG};
        write(fd, FRAME, 5);
    }

    alarm(0);

    if(close(fd) < 0)
        return -1;
    
    if(showStatistics && role == RX){
        rxStatistics();
    }
    else if(showStatistics && role == TX){
        txStatistics();
    }

    return 1;
}
