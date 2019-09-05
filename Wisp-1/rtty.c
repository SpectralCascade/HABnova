#include "mcc_generated_files/mcc.h"
#include "rtty.h"
#include <string.h>

#ifdef RTTY

// Modified from https://gist.github.com/tijnkooijmans/10981093
// Takes any number of strings in array form and computes the CRC using the
// CRC16_CCITT_FALSE algorithm with polynomial 0x1021.
uint16_t crc16_update(char* pData, int length, uint16_t wCrc)
{
    uint8_t i;
    while (length--) {
        wCrc ^= *(unsigned char *)pData++ << 8;
        for (i=0; i < 8; i++)
            wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
    }
    return wCrc;
}

uint16_t crc16(char** data, int segments)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < segments; i++)
    {
        crc = crc16_update(data[i], strlen(data[i]), crc);
    }
    return crc & 0xFFFF;
}

void TransmitBit(bool b)
{
	if (b)
	{
		TX_PIN_SetHigh();
	}
	else
	{
		TX_PIN_SetLow();
	}
#ifdef HIGH_BAUD_RATE
    // 100 baud
    __delay_us(HALF_BAUD_DELAY);
#else
    // 50 baud
	__delay_us(HALF_BAUD_DELAY);
	__delay_us(HALF_BAUD_DELAY);
#endif
}

void TransmitByte(char byte)
{
    /// The protocol used for data transmission is a type of UART (similar to RS-232).
    /// Here's the breakdown of the protocol used for transmitting the bytes:
    /// - Order of bits in a byte when transmitting is always LSB first, MSB last.
    /// - We always send a 0 bit (start bit) before the byte.
    /// - We always send two 1 bits after the byte (these are called stop bits).
    /// We can send either 7 or 8 bit characters, but we'll stick to 7-character codes
    /// so we'll actually be transmitting a bit less than a byte of data... pun intended :P

    /// We always send a single LOW bit as our start bit so the receiver software can sync up.
    TransmitBit(0);
    /// ASCII-7 encoding for our string characters.
	for (int i = 0; i < 7; i++)
	{
		TransmitBit((byte >> i) & 1);
	}
	/// Two stop bits to ensure the receiver software can sync up with the next start bit.
	TransmitBit(1);
	TransmitBit(1);
}

void TransmitString(char* message)
{
#ifdef DEBUG_CONSOLE
	printf("Attempting to transmit message:\n%s\n", message);
#endif
	for (int i = 0, counti = strlen(message); i < counti; i++)
	{
		TransmitByte(message[i]);
	}
#ifdef DEBUG_CONSOLE
	printf("\n");
#endif // DEBUG_CONSOLE
}

#endif // RTTY
