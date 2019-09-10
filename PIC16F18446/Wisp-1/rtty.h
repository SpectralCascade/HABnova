#ifndef RTTY
#define RTTY
#endif

#ifdef RTTY

// Constants for transmission code follow.
// Baud rate (bits per second).
#define BAUD_RATE 50

// TODO: make constant??
// Half the delay time required to match the baud rate in milliseconds
const unsigned int HALF_BAUD_DELAY = ((1000 / BAUD_RATE) / 2);

//#define HIGH_BAUD_RATE

/// CRC-16 checksum functions
uint16_t crc16_update(char* pData, int length, uint16_t wCrc);
uint16_t crc16(char** data, int segments);

/// Transmit data over RTTY device
void TransmitBit(bool b);
void TransmitChar(char byte);
void TransmitString(char* message);

#endif
