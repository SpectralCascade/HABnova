#ifndef DEBUG_CONSOLE
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pin_manager.h"

#define millis() TMR1_ReadTimer()

#else
#include "stdbool.h"

#define __delay_us(us)
#define __delay_ms(ms)

unsigned long ticks = 0;

#define millis() (++ticks)

typedef unsigned char uint8_t;

#endif
#include <string.h>
#include <stdio.h>

// I did this to make the code more readable in MPLABx, kinda like #region
// in VS C# but mostly because I'm too lazy to split the project into more files
#define RADIO_TRANSMISSION
#define GPS_MODULE

#ifndef DEBUG_CONSOLE
void TIMER1_Initialize()
{
    // Clear the timer interrupt register
    TMR1IF = 0;
    TMR1IE = 1;
    // Peripheral interrupt enable
    PEIE = 1;
    // Global interrupt enable
    GIE = 1;
    TMR1ON = 1;
    TMR1_StartTimer();
}

#endif // DEBUG_CONSOLE

#ifdef RADIO_TRANSMISSION
// Constants for transmission code follow.

// Baud rate (bits per second).
const int BAUD_RATE = 50;

// Half the delay time required to match the baud rate in MICRO seconds
#define HALF_BAUD_DELAY ((1000 / BAUD_RATE) / 2) * 1000

// How often a message should be sent in seconds
const int MESSAGE_INTERVAL = 5;

#ifndef DEBUG_CONSOLE
// Maximum length of a message, not including appended CRC
#define MAX_MESSAGE_LENGTH 40
#else
#define MAX_MESSAGE_LENGTH 120
#endif

// Interval multiplier
const int DELAY_MULT = 1000;

// Creates a CRC16 hash (CCITT algorithm?)
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
    /// The protocol used for data transmission is a type of UART (similar to RS-232).
    /// Here's the breakdown of the protocol used for transmitting the bytes:
    /// - Order of bits in a byte when transmitting is always LSB first, MSB last.
    /// - We always send a 0 bit (start bit) before the byte.
    /// - We always send two 1 bits after the byte (these are called stop bits).
    /// We can send either 7 or 8 bit characters, but we'll stick to 7-character codes
    /// so we'll actually be transmitting a bit less than a byte of data... pun intended :P

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

#endif // RADIO_TRANSMISSION

#ifdef GPS_MODULE

// UBX command which sets the GPS navigation mode to flight mode.
// See https://www.u-blox.com/en/docs/UBX-13003221 for info on the UBX protocol.
uint8_t setNavFlightMode[] = {
    // Sync characters
    0xB5, 0x62,
    // Message class
    0x06,
    // Message ID
    0x24,
    // Length of message data (2-byte value, little endian)
    0x24, 0x00,
    // The message data
    0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // Checksum
    0x16, 0xDC
};

// Computes the length of a UBX data packet in bytes.
size_t GetLengthUBX(uint8_t* data)
{
#ifdef DEBUG_CONSOLE
    printf("GetLengthUBX outputs %d\n", 8 + (short)((short)data[4] + (short)(data[5] << 8)));
#else
    return (short)(8 + (short)((short)data[4] + (short)(data[5] << 8)));
#endif // DEBUG_CONSOLE
}

// Send commands in the UBX format to the GPS module.
void GPS_SendUBX(uint8_t* data)
{
#ifdef DEBUG_CONSOLE
    printf("Sending UBX command to GPS module:\n");
#endif // DEBUG_CONSOLE
    for (int i = 0, length = GetLengthUBX(data); i < length; i++)
    {
#ifndef DEBUG_CONSOLE
        EUSART_Write(data[i]);
#else
        printf("%c", data[i]);
#endif // DEBUG_CONSOLE
    }
#ifdef DEBUG_CONSOLE
    printf("\n");
#endif
}

// Computes the expected ACK response from the GPS module for the given command
// and checks it against the actual response received from the GPS module.
// ACK means Acknowledgement and NAK means Negative-Acknowledgement.
// https://en.wikipedia.org/wiki/Acknowledgement_(data_networks)
bool GPS_HasAcknowledged(uint8_t* data)
{
#ifdef DEBUG_CONSOLE
    /// Pretend GPS acknowledgement worked
    return true;
#else
    uint8_t ackPacket[10];
    unsigned long startTime = millis();

    // This is the ACK data packet we expect to receive from the GPS module.
    ackPacket[0] = 0xB5;	// header
    ackPacket[1] = 0x62;	// header
    ackPacket[2] = 0x05;	// class    (Ack/Nak messages)
    ackPacket[3] = 0x01;	// id       (Message acknowledged)
    ackPacket[4] = 0x02;	// length
    ackPacket[5] = 0x00;
    ackPacket[6] = data[2];	// ACK class
    ackPacket[7] = data[3];	// ACK id
    ackPacket[8] = 0;		// CK_A
    ackPacket[9] = 0;		// CK_B

    // Calculate the checksum
    for (uint8_t i = 2; i < 8; i++)
    {
        ackPacket[8] = ackPacket[8] + ackPacket[i];
        ackPacket[9] = ackPacket[9] + ackPacket[8];
    }

    // The current byte we're checking.
    uint8_t ackByte;
    // The current byte index of the expected data packet to compare with the
    // current byte.
    uint8_t ackByteID = 0;

    // Read data from the GPS module and check it matches our ACK data packet.
    while (true)
    {

        // Test for success
        if (ackByteID > 9)
        {
            // The data matches our expected data packet,
            // acknowledgement success!
            return true;
        }

        // Timeout if no valid response in 3 seconds
        if (millis() - startTime > 3000)
        {
            return false;
        }

        // Make sure data is available to read
        if (EUSART_is_rx_ready())
        {
            ackByte = EUSART_Read();

            // Check that bytes arrive in sequence as per expected ACK packet
            if (ackByte == ackPacket[ackByteID]) {
                ackByteID++;
            }
            else {
                ackByteID = 0;
                // Reset and look again, invalid data
            }

        }
    }
#endif // DEBUG_CONSOLE
}

bool gps_configured = false;

void SetupGPS()
{
    while (!gps_configured)
    {
#ifndef DEBUG_CONSOLE
        LED_ACK_SetLow();
#endif // DEBUG_CONSOLE
        GPS_SendUBX(setNavFlightMode);
        gps_configured = GPS_HasAcknowledged(setNavFlightMode);
        // Flash the LED on retry, leave it on upon success
#ifndef DEBUG_CONSOLE
        LED_ACK_SetHigh();
        __delay_ms(500);
#endif // DEBUG_CONSOLE
    }
    gps_configured = false;
    // Disable all sentences, we will poll the GPS module as necessary.
    // printf() is rerouted to EUSART so we can use that to send NMEA commands.
    printf("$PUBX,40,GLL,0,0,0,0*5C\r\n");
    printf("$PUBX,40,GGA,0,0,0,0*44\r\n");
    printf("$PUBX,40,VTG,0,0,0,0*5E\r\n");
    printf("$PUBX,40,GSV,0,0,0,0*59\r\n");
    printf("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    printf("$PUBX,40,RMC,0,0,0,0*47\r\n");
}

enum FieldTypesPUBX {
    PUBX_TIME = 2,
    PUBX_LAT,
    PUBX_NS,
    PUBX_LONG,
    PUBX_EW,
    PUBX_ALT,
    PUBX_NAVSTAT,
    // Skip accuracy data estimate
    // Skip accuracy data estimate
    PUBX_SOG = 11,
    PUBX_COG,
    PUBX_VVEL
};

// World time
char gps_time[10] = {'\0'};
// Latitude
char gps_latitude[16] = {'\0'};
// Longitude
char gps_longitude[16] = {'\0'};
// Altitude in metres
char gps_altitude[12] = {'\0'};
// Navigation status
char gps_nav_status[4] = {'\0'};
// Speed over ground in km/h
char gps_speed_over_ground[8] = {'\0'};
// Course over ground in degrees
char gps_course_over_ground[8] = {'\0'};
// Vertical velocity where positive = downwards, negative = upwards in m/s
char gps_vertical_velocity[8] = {'\0'};

#ifdef DEBUG_CONSOLE
int FakeEusartCounter = 0;
const char* fakeData = "$PUBX,00,hhmmss.ss,Latitude,N,Longitude,E,AltRef,NavStat,Hacc,Vacc,SOG,COG,Vvel,ageC,HDOP,VDOP,TDOP,GU,RU,DR,*cs\r\n";
bool EUSART_is_rx_ready()
{
    return true;
}
uint8_t EUSART_Read()
{
    if (FakeEusartCounter >= strlen(fakeData))
    {
        FakeEusartCounter--;
    }
    return fakeData[FakeEusartCounter++];
}
#endif // DEBUG_CONSOLE

// Polls the GPS module and stores received data. Returns false if there is no
// response after 3 seconds.
bool GetNavData()
{
    bool success = false;
#ifndef DEBUG_CONSOLE
    LED_NAV_SetHigh();
#else
    FakeEusartCounter = 0;
#endif // DEBUG_CONSOLE

    unsigned long startTime = millis();

    // The current data index
    int dataIndex = 0;
    // The current data type (e.g. latitude, time, vertical velocity etc.)
    int dataFieldType = 0;

    while (!success)
    {
        char byte;

        // Timeout if no valid response in 3 seconds
        if (millis() - startTime > 3000)
        {
            break;
        }

        // Make sure data is available to read
        if (EUSART_is_rx_ready())
        {
            byte = EUSART_Read();

            bool skip = true;
            switch (byte)
            {
            case ',':
                dataFieldType++;
                dataIndex = 0;
                break;
            case '\n':
                success = dataFieldType > 13;
                break;
            default:
                skip = false;
                break;
            }

            if (!skip)
            {
                switch (dataFieldType)
                {
                case PUBX_TIME:
                    // TODO: check supported separator chars
                    gps_time[dataIndex] = byte;
                    break;
                case PUBX_LAT:
                    gps_latitude[dataIndex + 1] = byte;
                    break;
                case PUBX_NS:
                    // TODO: check the plus sign doesn't break stuff
                    gps_latitude[0] = byte == 'N' ? '+' : '-';
                    break;
                case PUBX_LONG:
                    gps_longitude[dataIndex + 1] = byte;
                    break;
                case PUBX_EW:
                    // TODO: check the plus sign doesn't break stuff
                    gps_longitude[0] = byte == 'E' ? '+' : '-';
                    break;
                case PUBX_ALT:
                    gps_altitude[dataIndex] = byte;
                    break;
                case PUBX_NAVSTAT:
                    gps_nav_status[dataIndex] = byte;
                    break;
                case PUBX_SOG:
                    gps_speed_over_ground[dataIndex] = byte;
                    break;
                case PUBX_COG:
                    gps_course_over_ground[dataIndex] = byte;
                    break;
                case PUBX_VVEL:
                    gps_vertical_velocity[dataIndex] = byte;
                    break;
                default:
                    break;
                }
                dataIndex++;
            }

        }

    }
#ifndef DEBUG_CONSOLE
    if (!success)
    {
        // Flash error indicator
        for (int i = 0; i < 4; i++)
        {
            __delay_ms(250);
            LED_NAV_SetLow();
            __delay_ms(250);
            LED_NAV_SetHigh();
        }
    }
    LED_NAV_SetLow();
#endif // DEBUG_CONSOLE
    return success;
}

#endif // GPS_MODULE

#ifdef DEBUG_CONSOLE
int main()
#else
void main(void)
#endif // DEBUG_CONSOLE
{
#ifndef DEBUG_CONSOLE
    SYSTEM_Initialize();
    TIMER1_Initialize();
#endif // DEBUG_CONSOLE

    SetupGPS();

    char message[MAX_MESSAGE_LENGTH + 3];

    int id = 0;
    while (1)
    {
        GetNavData();
        sprintf(message, "$$HABnova,%d,%s,%s,%s,%s,%s,%s,%s*",
                id, gps_time, gps_latitude, gps_longitude,
                gps_altitude, gps_speed_over_ground, gps_course_over_ground,
                gps_vertical_velocity
        );
        id++;
        AppendCRC(message, crc16(message, 0x1021));
        TransmitString(message);
        for (int i = 0; i < DELAY_MULT; i++)
        {
            __delay_ms(MESSAGE_INTERVAL);
        }
#ifdef DEBUG_CONSOLE
        break;
#endif // DEBUG_CONSOLE
    }
#ifdef DEBUG_CONSOLE
    return 0;
#endif // DEBUG_CONSOLE
}
