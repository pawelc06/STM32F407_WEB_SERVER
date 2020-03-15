#include "tm_stm32f4_onewire.h"
#include "tm_stm32f4_ds18b20.h"

#define EXPECTING_SENSORS	1

uint8_t DS_1820_Init(TM_OneWire_t *OneWire1);
uint8_t DS_1820_readTemp(TM_OneWire_t *OneWire1, char *inTemp);
