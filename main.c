// Creates a CRC16 hash
unsigned short crc16(const char* message, unsigned short polynomial = 0x8408) {
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

int main()
{
    return 0;
}
