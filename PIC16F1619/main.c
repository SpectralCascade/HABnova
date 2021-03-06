#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pin_manager.h"
#include "BME280_driver/bme280.h"
#include "bme280_i2c.h"
#include "timing.h"
#include "rtty.h"

#include <string.h>

// I did this to make the code more readable in MPLABx, kinda like #region
// in VS C# but mostly because I'm too lazy to split the project into more files
#define MISC
#define RADIO_TRANSMISSION
//#define GPS_MODULE
#define UBX
#define BME280_SENSOR

#define CALLSIGN "Wisp-1"

#ifdef MISC

int id = 0;

/// Nulls every character in a string
void ClearString(char* str)
{
    for (int i = 0, counti = strlen(str); i < counti; i++)
    {
        str[i] = '\0';
    }
}

/// Returns true if a source string has a matching target string
bool FindString(char* src, int srcLen, char* target, int targetLen) // 90 words
{
    int matching = 0;
    for (int i = 0; i < srcLen; i++)
    {
        if (src[i] == target[matching])
        {
            matching++;
            if (matching == targetLen)
            {
                return true;
            }
        }
        else
        {
            matching = 0;
        }
    }
    return false;
}

int Insert(char* dest, char src, int index, int destLimit)
{
    char temp = '\0';
    temp = dest[index];
    dest[index] = src;
    if (dest[index + 1] == '\0')
    {
        return index + 1;
    }
    index++;
    while (index < destLimit)
    {
        char current = dest[index];
        dest[index] = temp;
        temp = current;
        index++;
        if (current == '\0')
        {
            dest[index] = current;
            break;
        }
    }
    return index;
}

int InsertString(char* dest, char* src, int index, int destLimit)
{
    for (int i = 0, counti = strlen(src); i < counti && index < destLimit; i++)
    {
        /// Very inefficient and lazy, but saves valuable program memory.
        Insert(dest, src[i], index++, destLimit);
    }
    return index;
}

// For 32-bit integers (max integer value = 2,147,483,647).
#define MAX_INT_DIGITS 10

void ReverseString(char* str)
{
    int counti = strlen(str);
    for (int i = 0, halfway = counti / 2; i < halfway; i++)
    {
        char temp = str[i];
        char* opposite = str + (counti - 1 - i);
        str[i] = *opposite;
        *opposite = temp;
    }
    str[counti] = '\0';
}

void UIntToHexString(uint32_t n, char* dest)
{
    uint32_t i = 0;
    do {
        /// Set the current digit in the string to the ASCII equivalent of 
        /// the remainder of n / 10.
        uint32_t remainder = n % 16;
        // If >= 10, then use letters (ASCII 'A' = 65, so 65 - 10 == 55).
        dest[i++] = remainder + (remainder >= 10 ? 55 : '0');
        /// Shift all the digits to the right to get the next digit.
    } while ((n /= 16) > 0);
    while (i < 4)
    {
        dest[i] = '0';
        i++;
    }
    ReverseString(dest);
}

/// Warning, unsafe! Destination string MUST have >= 4 bytes allocated
/// for a 32-bit hex code.
void IntToHexString(int n, char* dest)
{
    int i = 0;
    do {
        /// Set the current digit in the string to the ASCII equivalent of 
        /// the remainder of n / 10.
        int remainder = n % 16;
        // If >= 10, then use letters (ASCII 'A' = 65, so 65 - 10 == 55).
        dest[i++] = remainder + (remainder >= 10 ? 55 : '0');
        /// Shift all the digits to the right to get the next digit.
    } while ((n /= 16) > 0);
    while (i < 4)
    {
        dest[i] = '0';
        i++;
    }
    ReverseString(dest);
}

/// Warning, unsafe! Destination string MUST have >= 12 bytes allocated
/// to allow maximum 32-bit integer length with a '-' sign.
void IntToString(int n, char* dest)
{
    int i = 0;
    bool sign = n < 0;
    if (sign)
    {
        /// Make the number positive
        n = -n;
    }
    /// Get digits in reverse order
    do {
        /// Set the current digit in the string to the ASCII equivalent of 
        /// the remainder of n / 10.
        dest[i++] = n % 10 + '0';
        /// Shift all the digits to the right to get the next digit.
    } while ((n /= 10) > 0);
    if (sign)
    {
        dest[i++] = '-';
    }
    dest[i] = '\0';
    /// Reverse the string so things are flipped the right way round
    ReverseString(dest);
}

/// Warning, unsafe! Destination string MUST have >= 12 bytes allocated
/// to allow maximum 32-bit integer length with a '-' sign.
void UIntToString(uint32_t n, char* dest)
{
    int i = 0;
    /// Get digits in reverse order
    do {
        /// Set the current digit in the string to the ASCII equivalent of 
        /// the remainder of n / 10.
        dest[i++] = n % 10 + '0';
        /// Shift all the digits to the right to get the next digit.
    } while ((n /= 10) > 0);
    dest[i] = '\0';
    /// Reverse the string so things are flipped the right way round
    ReverseString(dest);
}

/// Modified from https://github.com/vinceallenvince/floatToString/blob/master/floatToString.h
/// on 01/09/2019
void FloatToString(char* dest, float value, int places) {
    int fts_digit;
    float fts_tens = 0.1;
    int fts_tenscount = 0;
    int fts_iterator;
    float fts_tempfloat = value;
    int fts_index = 0;
    
    // Make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
    // If this rounding step isn't here, the value  54.321 prints as 54.3209
    // Rounding term d = 0.5 / pow(10,places)
    float d = 0.5;
    if (value < 0) {
        d *= -1.0;
    }
    // Divide by ten for each decimal place
    for (fts_iterator = 0; fts_iterator < places; fts_iterator++) {
        d /= 10.0;
    }
    // This small addition, combined with truncation will round our values properly
    fts_tempfloat +=  d;

    // First get value tens to be the large power of ten less than value
    if (value < 0) {
        fts_tempfloat *= -1.0;
    }
    while ((fts_tens * 10.0) <= fts_tempfloat) {
        fts_tens *= 10.0;
        fts_tenscount += 1;
    }

    // Write out the negative if needed
    if (value < 0) {
        dest[fts_index++] = '-';
    }

    if (fts_tenscount == 0) {
        dest[fts_index++] = '0';
    }

    for (fts_iterator = 0; fts_iterator < fts_tenscount; fts_iterator++) {
        fts_digit = (int) (fts_tempfloat/fts_tens);
        IntToString(fts_digit, &dest[fts_index++]);
        fts_tempfloat = fts_tempfloat - ((float)fts_digit * fts_tens);
        fts_tens /= 10.0;
    }

    // If no places after decimal, stop now and return
    // Otherwise, write the point and continue on
    if (places > 0){
        dest[fts_index++] = '.';
    }

    // Now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
    for (fts_iterator = 0; fts_iterator < places; fts_iterator++) {
        fts_tempfloat *= 10.0;
        fts_digit = (int) fts_tempfloat;
        IntToString(fts_digit, &dest[fts_index++]);
        // once written, subtract off that digit
        fts_tempfloat = fts_tempfloat - (float) fts_digit;
    }

    dest[fts_index++] = '\0';
}

// Interval multiplier
#define DELAY_MULT 1000

// How often a message should be sent in seconds
#define MESSAGE_INTERVAL 5

#ifndef DEBUG_CONSOLE
// Maximum length of a message, not including appended CRC
#define MAX_MESSAGE_LENGTH 70
#else
#define MAX_MESSAGE_LENGTH 70
#endif

#endif// MISC

// Transmission is split into two messages because the PIC16F1619
// microcontroller has limited contiguous memory.
char message_start[MAX_MESSAGE_LENGTH];
char message_end[MAX_MESSAGE_LENGTH + 3];

/// Array of message block pointers.
char* messages[2] = {message_start, message_end};

void ClearMessages()
{
    ClearString(messages[0]);
    ClearString(messages[1]);
}

/// Large memory footprint
#ifdef GPS_MODULE

void WriteGPS(char* message)
{
    for (int i = 0, counti = strlen(message); i <= counti; i++)
    {
        EUSART_Write((uint8_t)message[i]);
    }
}

#ifdef UBX
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
    return (short)(8 + (short)((short)data[4] + (short)(data[5] << 8)));
}

// Send commands in the UBX format to the GPS module.
void GPS_SendUBX(uint8_t* data)
{
    for (int i = 0, length = GetLengthUBX(data); i < length; i++)
    {
        EUSART_Write(data[i]);
    }
}

// Computes the expected ACK response from the GPS module for the given command
// and checks it against the actual response received from the GPS module.
// ACK means Acknowledgement and NAK means Negative-Acknowledgement.
// https://en.wikipedia.org/wiki/Acknowledgement_(data_networks)
bool GPS_HasAcknowledged(uint8_t* data)
{
    uint8_t ackPacket[10];
    uint32_t startTime = millis();

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
    
    return false;
}

bool gps_configured = false;

#endif// UBX

void SetupGPS() // 281 words
{
    while (!gps_configured)
    {
        LED_ACK_SetLow();
        GPS_SendUBX(setNavFlightMode);
        gps_configured = GPS_HasAcknowledged(setNavFlightMode);
        // Flash the LED on retry, leave it on upon success
        LED_ACK_SetHigh();
        __delay_ms(500);
    }
    gps_configured = false;
    // Disable all sentences, we will poll the GPS module as necessary.
    /*WriteGPS("$PUBX,40,GLL,0,0,0,0*5C\r\n");
    WriteGPS("$PUBX,40,GGA,0,0,0,0*5A\r\n");
    WriteGPS("$PUBX,40,VTG,0,0,0,0*5E\r\n");
    WriteGPS("$PUBX,40,GSV,0,0,0,0*59\r\n");
    WriteGPS("$PUBX,40,GSA,0,0,0,0*4E\r\n");
    WriteGPS("$PUBX,40,RMC,0,0,0,0*47\r\n");*/
}

enum FieldTypesPUBX {
    PUBX_DATA_TYPE = 0,
    PUBX_TIME = 2,
    PUBX_LAT,
    PUBX_NS,
    PUBX_LONG,
    PUBX_EW,
    PUBX_ALT,
    PUBX_NAVSTAT,
    PUBX_HDOP,
    PUBX_VDOP,
    PUBX_SOG,
    PUBX_COG,
    PUBX_VVEL
};

#define LEN_DATA_TYPE   7

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

// Enable this to transmit the first chunk of GPS data received.
//#define TEST_GPS_NAVDATA

bool PollGPS()
{
    bool success = false;
    uint32_t startTime = millis();
    
    // The current data index
    int dataIndex = 0;
    // The current data type (e.g. latitude, time, vertical velocity etc.)
    int dataFieldType = 0;
    
    char byte;
    
    char data_type[LEN_DATA_TYPE];// = {'\0'};
    // More efficient to do this than clear the whole string as we insert anyway.
    data_type[0] = '\0';
    
    bool doParse = false;
    
    ClearMessages();
    
    int index = 0;
    
    int signInsertIndex = -1;
    
    /// Poll the GPS module
    WriteGPS("$PUBX,00*33\r\n");
    
    while (!success)
    {
        // Timeout if no valid response in 3 seconds
        if (millis() - startTime > 3000) /// WARNING: MASSIVE memory usage (54 words!) due to data sizes
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
            case '$':
                doParse = true;
                dataIndex = 0;
                break;
            case ',':
                if (dataFieldType == PUBX_DATA_TYPE)
                {
                    if (strcmp(data_type, "PUBX") != 0)
                    {
                        doParse = false;
                        ClearString(data_type);
                    }
                }
                if (doParse)
                {
                    dataFieldType++;
                    messages[dataFieldType > PUBX_ALT ? 1 : 0][index] = ',';
                    index++;
                }
                dataIndex = 0;
                break;
            case '\n':
                success = dataFieldType > 13;
                break;
            default:
                skip = false;
                break;
            }
            
            if (!skip && doParse)
            {
                switch (dataFieldType)
                {
                case PUBX_DATA_TYPE:
                    Insert(data_type, byte, dataIndex, LEN_DATA_TYPE);
                    break;
                case PUBX_TIME:
                    if (dataIndex < 6)
                    {
                        if (dataIndex == 2 || dataIndex == 4)
                        {
                            messages[0][index] = ':';
                            index++;
                        }
                        messages[0][index] = byte;
                    }
                    break;
                case PUBX_LAT:
                    if (dataIndex == 0)
                    {
                        signInsertIndex = index;
                    }
                    messages[0][index] = byte;
                    break;
                case PUBX_NS:
                    if (byte != 'N')
                    {
                        Insert(messages[0], '-',  signInsertIndex, MAX_MESSAGE_LENGTH);
                    }
                    break;
                case PUBX_LONG:
                    if (dataIndex == 0)
                    {
                        signInsertIndex = index;
                    }
                    messages[0][index] = byte;
                    break;
                case PUBX_EW:
                    if (byte != 'E')
                    {
                        Insert(messages[0], '-', signInsertIndex, MAX_MESSAGE_LENGTH);
                    }
                    break;
                case PUBX_ALT:
                    messages[0][index] = byte;
                    break;
                case PUBX_HDOP:
                    if (dataIndex == 0)
                    {
                        index = 0;
                    }
                case PUBX_VDOP:
                case PUBX_NAVSTAT:
                case PUBX_SOG:
                case PUBX_COG:
                case PUBX_VVEL:
                    messages[1][index] = byte;
                    break;
                default:
                    // Keep index the same
                    index--;
                    break;
                }
                dataIndex++;
                index++;
            }
            
        }
    }
    
    return success;
}

#endif // GPS_MODULE

char checksum[6] = {'\0'};

void main(void)
{
    /// Initialise the microcontroller pins, modules and interrupts
    SYSTEM_Initialize();
    InitTiming();
    
    /// Setup the sensor device
    BME280_Init();
    
    ClearMessages();

#ifdef GPS_MODULE
    SetupGPS();
#endif

    while (1)
    {
        /// Get GPS nav data and parse it into string
#ifdef GPS_MODULE
        if (PollGPS())
#else
        if (true)
#endif
        {
#ifndef TEST_GPS_NAVDATA
            struct bme280_data sensor_data;
            bme280_get_sensor_data(BME280_ALL, &sensor_data, &EnvSensor);
#ifdef RADIO_TRANSMISSION
#ifndef GPS_MODULE
            ClearMessages();
#endif
//#ifdef GPS_MODULE
            char convertedNumber[16];
            convertedNumber[15] = '\0';
            ClearString(convertedNumber);
            /// Temperature
            IntToString(sensor_data.temperature, convertedNumber);
            int index = strlen(messages[1]);
            index = InsertString(messages[1], convertedNumber, index, MAX_MESSAGE_LENGTH);
            index = Insert(messages[1], ',', index, MAX_MESSAGE_LENGTH);
            ClearString(convertedNumber);
            /// Pressure
            UIntToString(sensor_data.pressure, convertedNumber);
            index = InsertString(messages[1], convertedNumber, index, MAX_MESSAGE_LENGTH);    
            index = Insert(messages[1], ',', index, MAX_MESSAGE_LENGTH);
            ClearString(convertedNumber);
            /// Humidity
            UIntToString(sensor_data.humidity, convertedNumber);
            index = InsertString(messages[1], convertedNumber, index, MAX_MESSAGE_LENGTH);
            
            int end = index;

            /// Add the call sign and id.
            index = InsertString(messages[0], CALLSIGN, 0, MAX_MESSAGE_LENGTH);
            index = Insert(messages[0], ',', index, MAX_MESSAGE_LENGTH);
            ClearString(convertedNumber);
            IntToString(id, convertedNumber);
            index = InsertString(messages[0], convertedNumber, index, MAX_MESSAGE_LENGTH);
            Insert(messages[0], ',', index, MAX_MESSAGE_LENGTH);
            
            UIntToHexString(crc16(messages, 2), checksum);
            
            /// Finally, add the 2 start characters and the checksum.
            InsertString(messages[0], "$$", 0, MAX_MESSAGE_LENGTH);     
            
            index = Insert(messages[1], '*', end, MAX_MESSAGE_LENGTH);
            index = InsertString(messages[1], checksum, index, MAX_MESSAGE_LENGTH);
            index = Insert(messages[1], '\n', index, MAX_MESSAGE_LENGTH);
            
            id++;
//#endif
            TX_LED_SetHigh();
            TransmitString(messages[0]);
            TransmitString(messages[1]);
            TX_LED_SetLow();
#endif
#endif
        }
        else
        {
            /// Failed to get GPS data
        }
        
        for (int i = 0; i < DELAY_MULT; i++)
        {
            __delay_ms(MESSAGE_INTERVAL);
        }
    }
}
