#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pin_manager.h"
#include <string.h>
#include <stdio.h>

// Baud rate delay between bit changes
const int BAUD_DELAY = 20;
// How often a message should be sent in seconds
#define MESSAGE_INTERVAL 5
// Maximum length of a message, not including appended CRC
#define MAX_MESSAGE_LENGTH 40
// Multiplier necessary for getting large delay times
#define DELAY_MULT 1000

// Creates a CRC16 hash
unsigned short crc16(const char* message, unsigned short polynomial) {
	unsigned int crc;

	crc = 0xFFFF;

	if (strlen(message) == 0)
	{
		return (~crc);
	}
	char append = 0x00;
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
        TX_PIN_SetHigh();
    }
    else
    {
        TX_PIN_SetLow();
    }
    __delay_ms(BAUD_DELAY);
    transmission_time += BAUD_DELAY;
}

void TransmitByte(char byte)
{
    for (int i = 0; i < 8; i++)
    {
        if (byte & 1)
        {
            TransmitBit(true);
        }
        else
        {
            TransmitBit(false);
        }
        byte = byte >> 1;
    }
}

void TransmitString(char* message)
{
    for (int i = 0, counti = strlen(message); i < counti; i++)
    {
        TransmitByte(message[i]);
    }
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
    data[len] = (char)(crc << 8);
    data[len + 1] = (char)(crc);
    data[len + 2] = '\n';
    data[len + 3] = '\0';
}

void main(void)
{
    SYSTEM_Initialize();
    PIN_MANAGER_Initialize();
    
    char message[MAX_MESSAGE_LENGTH + 3];
    
    int id = 0;
    while (1)
    {
        sprintf(message, "RTTY Transmission Test, Hello World! ID: %d \0", id);
        id++;
        AppendCRC(message, crc16(message, 0x8408));
        TransmitString(message);
        // Set output low
        TransmitBit(false);
        // Be aware that this doesn't account for transmission time
        for (int i = 0; i < DELAY_MULT; i++)
        {
            __delay_ms(MESSAGE_INTERVAL);
        }
        transmission_time = 0;
    }
}
