#ifndef DEBUG_CONSOLE
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pin_manager.h"
#else
#include "stdbool.h"

#define __delay_us(us)
#define __delay_ms(ms)
#endif
#include <string.h>
#include <stdio.h>

// Baud rate (bits per second).
const int BAUD_RATE = 50;
// Half the delay time required to match the baud rate in MICRO seconds
#define HALF_BAUD_DELAY ((1000 / BAUD_RATE) / 2) * 1000
// How often a message should be sent in seconds
const int MESSAGE_INTERVAL = 1;
// Maximum length of a message, not including appended CRC
#define MAX_MESSAGE_LENGTH 40
// Interval multiplier
const int DELAY_MULT = 1000;

// Creates a CRC16 hash
unsigned short crc16(const char* message, unsigned short polynomial) {
	unsigned int crc;

	crc = 0xFFFF;
	if (strlen(message) == 0)
	{
		return (~crc);
	}
	const unsigned short FRONT_BIT = 0x8000;
	for (unsigned int i = 0, counti = strlen(message); i < counti; i++) {
		char byte = message[i];
		// Move byte to MSB and XOR with the crc
		crc = crc ^ ((unsigned short)(byte << 8));
		// Run through the current byte
		for (int i = 0; i < 8; i++) {
			if ((crc & FRONT_BIT) != 0) {
				crc = ((unsigned short)(crc << 1)) ^ polynomial;
			}
			else {
				// If MSB is not a 1, we can simply skip ahead with XORing
				crc = crc << 1;
			}
		}
	}

	return crc;
}

int transmission_time = 0;

void TransmitBit(bool b)
{
	if (b)
	{
#ifdef DEBUG_CONSOLE
		printf("1");
#else
		TX_PIN_SetHigh();
#endif // DEBUG_CONSOLE
	}
	else
	{
#ifdef DEBUG_CONSOLE
		printf("0");
#else
		TX_PIN_SetLow();
#endif // DEBUG_CONSOLE
	}
	__delay_us(HALF_BAUD_DELAY);
	__delay_us(HALF_BAUD_DELAY);
}

void TransmitByte(char byte)
{
    /// Here's the breakdown of the serial protocol used for transmitting the bytes:
    /// - Order of bits in a byte when transmitting is always LSB first, MSB last.
    /// - We always send a 1 bit (start bit) before the byte.
    /// - We always send two 0 bits after the byte (these are called stop bits).
    /// We can send either 7 or 8 bit characters, but we'll stick to 7-character codes
    /// so we'll actually be transmitting a bit less than a byte of data... pun intended :P
    /// The protocol used is basically RS-232 (apparently).

    /// We always send a single LOW bit as our start bit so the receiver software can sync up.
    TransmitBit(0);
#ifdef DEBUG_CONSOLE
    printf("-");
#endif
    /// ASCII-7 encoding for our string characters.
	for (int i = 0; i < 7; i++)
	{
		TransmitBit((byte >> i) & 1);
	}
#ifdef DEBUG_CONSOLE
    printf("-");
#endif
	/// Two stop bits to ensure the receiver software can sync up with the next start bit.
	TransmitBit(1);
	TransmitBit(1);
#ifdef DEBUG_CONSOLE
	printf(" [%c], ", byte);
#endif // DEBUG_CONSOLE
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

// Appends CRC and a newline character to a string
void AppendCRC(char* data, unsigned short crc)
{
	int len = strlen(data);
	if (len >= MAX_MESSAGE_LENGTH - 1)
	{
		// Ignore data indices greater than max message length
		len = MAX_MESSAGE_LENGTH - 2;
	}
	data[len] = (char)(crc >> 8);
	data[len + 1] = (char)(crc);
	data[len + 2] = '\n';
	data[len + 3] = '\0';
}

#ifdef DEBUG_CONSOLE
int main()
#else
void main(void)
#endif // DEBUG_CONSOLE
{
#ifndef DEBUG_CONSOLE
    SYSTEM_Initialize();
    PIN_MANAGER_Initialize();
#endif // DEBUG_CONSOLE

    char message[MAX_MESSAGE_LENGTH + 3];

    int id = 0;
    while (1)
    {
        sprintf(message, "NOVA TEST TRANSMISSION, ID: %d", id);
        id++;
        AppendCRC(message, crc16(message, 0x1021));
        TransmitString(message);
        // Be aware that this doesn't account for transmission time
        for (int i = 0; i < DELAY_MULT; i++)
        {
            __delay_ms(MESSAGE_INTERVAL);
        }
        transmission_time = 0;
#ifdef DEBUG_CONSOLE
        break;
#endif // DEBUG_CONSOLE
    }
#ifdef DEBUG_CONSOLE
    return 0;
#endif // DEBUG_CONSOLE
}
