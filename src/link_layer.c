// Link layer protocol implementation

#include "link_layer.h"

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


int fd;

int txStateMachine(enum linkState *state, LinkLayer *connectionParameters) {
    
    unsigned char byte;
    
    while(state != STOP && connectionParameters->nRetransmissions > 0){
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
        connectionParameters->nRetransmissions--;
    }
}


int rcStateMachine(enum linkState *state) {
    
    unsigned char byte;
    
    while(state != STOP){
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
}



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    enum linkState state = START;

    fd = connection(connectionParameters.serialPort);
    if(fd < 0)
        return -1;

    switch (connectionParameters.role)
    {
    case LlTx:
        txStateMachine(&state, &connectionParameters);
        break;

    case LlRx:
        rcStateMachine(&state);
        break;
    
    default:
        break;
    }

    return 1;
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
