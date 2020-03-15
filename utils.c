#include "utils.h"

uint8_t DS_1820_Init(TM_OneWire_t *OneWire1) {

	uint8_t devices, i, j, count;
	uint8_t device[EXPECTING_SENSORS][8];

	count = 0;

	TM_OneWire_Init(OneWire1, GPIOD, GPIO_Pin_0);
	devices = TM_OneWire_First(OneWire1);
	while (devices) {

		count++;

		TM_OneWire_GetFullROM(OneWire1, device[count - 1]);

		devices = TM_OneWire_Next(OneWire1);
	}

	if (count > 0) {
		printf("Devices found on 1-wire: %d\n", count);

		for (j = 0; j < count; j++) {
			for (i = 0; i < 8; i++) {
				printf("0x%02X ", device[j][i]);

			}
			printf("\n");
		}
	} else {
		printf("No devices on OneWire.\n");
		return 1;
	}

	for (i = 0; i < count; i++) {

		TM_DS18B20_SetResolution(OneWire1, device[i],
				TM_DS18B20_Resolution_12bits);
	}
	return 0;
}

uint8_t DS_1820_readTemp(TM_OneWire_t *OneWire1, char *inTemp) {
	uint8_t devices, i, j;
	uint8_t device[EXPECTING_SENSORS][8];
	float temps[EXPECTING_SENSORS];

	TM_DS18B20_StartAll(OneWire1);

	while (!TM_DS18B20_AllDone(OneWire1))
		;



		if (TM_DS18B20_Read(OneWire1, OneWire1->ROM_NO, &temps[0])) {

			printf("Temp %d: %2.5f; \n", i, temps[0]);
			sprintf(inTemp, "%2.1f", temps[0]);
		} else {

			printf("Reading error;\n");
			return 1;
		}


	return 0;
}
