
void DebugAlert(unsigned int delay_high, unsigned int delay_low, uint8_t num_flashes);

#define FlashError() DebugAlert(200, 200, 3);
