/*
   rtty.h
   Copyright Jon Sowman 2010

   This file is part of the Ferret/librtty project, an Arduino based
   high altitude balloon tracker, and an Arduino library for generating
   an RTTY bitstream.

   CU Spaceflight 2010
   http://www.cuspaceflight.co.uk
*/

#ifndef rtty_h
#define rtty_h

#include "types.h"
#include "emqueue/emqueue.h"

#define ASCII_BITSIZE 7

QUEUE_DECLARE(uint8_t, 255);

typedef enum {
    RTTY_PHASE_START, RTTY_PHASE_SENDING, RTTY_PHASE_STOP
} RTTY_PHASE;

class RTTY {
public:
    RTTY(int pin, int baud, float stopbits, checksum_type ctype, bool reverse,
            bool echo);
    void transmit(char *str);
    void transmit(char data);
    void setBaud(int baud);
    int getBaud();
    void setChecksum(checksum_type ctype);
    checksum_type getChecksum();
protected:
    void _writeStopBit();
    void _writeStartBit();
    void _writeBit(uint8_t data, int bit);
    const float _stopbits;
private:
    unsigned int _crc16(char *string);
    const int _pin;
    int _timestep;
    checksum_type _ctype;
    bool _reverse;
    bool _echo;
};

class AsynchronousRTTY : public RTTY {
public:
    AsynchronousRTTY(int pin, int baud, float stopbits, checksum_type ctype,
            bool reverse, bool echo);
    void transmitInterrupt();
    int bufferSize();
    void transmitAsync(char* str);
private:
    QUEUE_TYPE(uint8_t) _queue;
    // Don't use 'bool' from Arduino/chipKIT, becuase it doesn't work with volatile
    volatile int _queueLock;
    volatile int _transmissionPhase;
    volatile uint8_t _currentByte;
    volatile int _currentBit;
};

#endif
