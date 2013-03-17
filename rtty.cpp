/*
   rtty.cpp
   Copyright Jon Sowman 2010

   This file is part of the Ferret/librtty project, an Arduino based
   high altitude balloon tracker, and an Arduino library for generating
   an RTTY bitstream.

   CU Spaceflight 2010
   http://www.cuspaceflight.co.uk
*/

#include "WProgram.h"
#include "types.h"
#include "rtty.h"

QUEUE_DEFINE(uint8_t);

RTTY::RTTY(int pin, int baud, float stopbits, int asciibits,
        checksum_type ctype, bool reverse, bool echo)
    : _pin(pin), _stopbits(stopbits), _asciibits(asciibits), _ctype(ctype),
        _reverse(reverse), _echo(echo)
{
    // Set the radio TXD pin to output
    pinMode(_pin, OUTPUT);
    _timestep = (int)(500000/baud);
}

void RTTY::_preprocessTransmission(char *str) {
    // First we calculate a checksum if required and append it to the
    // transmit string if so
    unsigned int checksum;
    char checksum_string[6];

    switch(_ctype) {
        case CHECKSUM_CRC16:
            checksum = _crc16(str);
            sprintf(checksum_string, "*%04X", checksum);
            strcat(str, checksum_string);
            break;
        case CHECKSUM_NONE:
            break;
        default:
            break;
    }

    // Then we automatically append a newline
    strcat(str, "\n");
}

void RTTY::transmit(char *str) {
    // Transmit an input string over the radio after appending a checksum
    // if required
    _preprocessTransmission(str);

    // Iterate through the string transmitting byte-by-byte
    int j=0;
    while(str[j] != 0) {
        transmit(str[j]);
        j++;
    }
}

void RTTY::_writeBit(uint8_t data, int bit) {
    if (data & (1 << bit) ) {
        digitalWrite(_pin, _reverse ? LOW : HIGH);
    } else {
        digitalWrite(_pin, _reverse ? HIGH : LOW);
    }
}

void RTTY::_writeStopBit() {
    digitalWrite(_pin, _reverse ? LOW : HIGH);
}

void RTTY::_writeStartBit() {
    digitalWrite(_pin, _reverse ? HIGH : LOW);
}

void RTTY::transmit(char data) {
    // Write a single byte to the radio ensuring it is padded
    // by the correct number of start/stop bits

    _writeStartBit();

    // Calculate the timestep in microseconds
    // We use two smaller delays instead of one larger as delayMicroseconds
    // is not accurate above ~16000uS, and the required delay is 20000uS
    // for 50 baud operation

    // We use delayMicroseconds as it is unaffected by Timer0, unlike delay()
    delayMicroseconds(_timestep);
    delayMicroseconds(_timestep);

    // Write the data byte
    int bit;
    for ( bit=0; bit < _asciibits; bit++ ) {
        _writeBit(data, bit);
        delayMicroseconds(_timestep);
        delayMicroseconds(_timestep);
    }

    if(_echo) {
        Serial.print(data);
    }

    _writeStopBit();
    delayMicroseconds((int)(_timestep * _stopbits));
    delayMicroseconds((int)(_timestep * _stopbits));
}

unsigned int RTTY::_crc16(char *string) {
    // Returns the CRC16_CCITT checksum for the input string

    unsigned int i;
    unsigned int crc;

    // CCITT uses 0xFFFF as the initial CRC value
    crc = 0xFFFF;

    // Iterate through the string updating the checksum byte-by-byte
    for( i=0; i < strlen(string); i++ ) {
        crc = _crc_1021(crc, (uint8_t)(string[i]));
    }

    return crc;
}

// Borrowed from http://www.ccsinfo.com/forum/viewtopic.php?t=24977
unsigned int RTTY::_crc_1021(unsigned int old_crc, uint8_t data) {
  unsigned int crc;
  unsigned int x;

  x = ((old_crc >> 8) ^ data) & 0xff;
  x ^= x >> 4;

  crc = (old_crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
  crc &= 0xffff;
  return crc;
}

void RTTY::setBaud(int baud) {
    // Set the RTTY baud rate to a new one, can be called at any time
    // Calculate the new timestep
    _timestep = (int)(500000/baud);
}

int RTTY::getBaud() {
    // Return the current baud rate
    // We only store the timestep, so re-calculate the baud rate & return
    int baud = (int)(500000/_timestep);
    return baud;
}

void RTTY::setChecksum(checksum_type ctype) {
    // Change the checksum type appended to the transmit string
    _ctype = ctype;
}

checksum_type RTTY::getChecksum() {
    // Return the current checksum setting
    return _ctype;
}

AsynchronousRTTY::AsynchronousRTTY(int pin, int baud, float stopbits,
        int asciibits, checksum_type ctype, bool reverse, bool echo) :
        RTTY(pin, baud, stopbits, asciibits, ctype, reverse, echo),
        _transmissionPhase(RTTY_PHASE_START) {
    QUEUE_INIT(uint8_t, &_queue);
}

void AsynchronousRTTY::transmitInterrupt() {
    switch(_transmissionPhase) {
    case RTTY_PHASE_START: // Grab a char and lets go transmit it.
        if(!_queueLock && !QUEUE_EMPTY(uint8_t, &_queue)) {
            _currentByte = QUEUE_POP(uint8_t, &_queue);
            _currentBit = 0;
            _transmissionPhase = RTTY_PHASE_SENDING;
            _writeStartBit();
        }
        break;
    case RTTY_PHASE_SENDING:
        if(_currentBit < _asciibits) {
            _writeBit(_currentByte, _currentBit);
            ++_currentBit;
        } else {
            _writeStopBit();
            _transmissionPhase = RTTY_PHASE_STOP;
        }
        break;
    case RTTY_PHASE_STOP:
        if(_stopbits == 2) {
            _writeStopBit();
        }
        _transmissionPhase = RTTY_PHASE_START;
        break;
    }
}

int AsynchronousRTTY::bufferSize() {
    return QUEUE_LENGTH(uint8_t, &_queue);
}

void AsynchronousRTTY::transmitAsync(char* data) {
    _preprocessTransmission(data);
    _queueLock = true;
    int i = 0;
    while(data[i] != NULL) {
        QUEUE_PUSH(uint8_t, &_queue, data[i]);
        ++i;
    }
    _queueLock = false;
}
