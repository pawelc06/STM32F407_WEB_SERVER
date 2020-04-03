/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_ethernet.h"
#include "tm_stm32f4_rtc.h"
#include "tm_stm32f4_watchdog.h"
#include "tm_stm32f4_fatfs.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_id.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
//#include "rf01.h"

#include "rfm69.h"
#include "utils.h"
#include "HTU21D.h"
#include "tm_stm32f4_i2c.h"
#include "mjson.h"

#define ENABLE_DOMOTICZ 1

/* File variable */
FIL fil[ETHERNET_MAX_OPEN_FILES];
/* Fatfs variable */
FATFS fs;

/* RTC data variable */
TM_RTC_t RTC_Data;

/* Set SSI tags for handling variables */
static TM_ETHERNET_SSI_t SSI_Tags[] = { "led1_s", /* Tag 0 = led1 status */

"led2_s", /* Tag 1 = led2 status */
"led3_s", /* Tag 2 = led3 status */
"led4_s", /* Tag 3 = led4 status */
"srv_adr",/* Tag 4 = server address */
"clt_a_c",/* Tag 5 = client all connections */
"clt_s_c",/* Tag 6 = client successfull connections */
"clt_per",/* Tag 7 = client percentage */
"clt_tx", /* Tag 8 = client TX bytes */
"clt_rx", /* Tag 9 = client RX bytes */
"srv_c", /* Tag 10 = server all connections */
"srv_tx", /* Tag 11 = server TX bytes */
"srv_rx", /* Tag 12 = server RX bytes */
"mac_adr",/* Tag 13 = MAC address */
"gateway",/* Tag 14 = gateway */
"netmask",/* Tag 15 = netmask */
"link", /* Tag 16 = link status */
"duplex", /* Tag 17 = duplex status */
"hardware",/* Tag 18 = hardware where code is running */
"rtc_time",/* Tag 19 = current RTC time */
"compiled",/* Tag 20 = compiled date and time */
"outTemp", "outHum", "voltage", "lFrTimestamp", "inTemp", "temp1", "hum1",
		"vbat1", "ts1", "temp2", "hum2", "vbat2", "ts2", "temp3", "hum3",
		"vbat3", "ts3", "temp4", "hum4", "vbat4", "ts4", "temp0", "hum0",
		"vbat0" };

/* LED CGI handler */
const char * LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[],
		char *pcValue[]);
const char * TEMP_CGI_Handler(int iIndex, int iNumParams, char *pcParam[],
		char *pcValue[]);

uint8_t parseFrame(char *frame, char *temp, char *hum);
uint8_t parseFrameV(char *frame, char *temp, char *hum, char *vcc);
uint8_t parseTempAndHumFromJSON(char *inData, char *tempStr,char *humStr);

/* CGI call table, only one CGI used */
TM_ETHERNET_CGI_t CGI_Handlers[] = { { "/ledaction.cgi", LEDS_CGI_Handler }, /* LEDS_CGI_Handler will be called when user connects to "/ledaction.cgi" URL */
{ "/gettemp.cgi", TEMP_CGI_Handler }, /* LEDS_CGI_Handler will be called when user connects to "/ledaction.cgi" URL */
};

char outTemp[16];
char outHum[16];
char temp[10];
char vbat[10];
char lastFrameTimestamp[21];
char lastFrameTimestampWithSpace[21];
char serverSensorsData[21];

char inTemp[9];

char temp0[6];
char temp1[6];
char temp2[6];
char temp3[6];
char temp4[6];
char temp5[6];

char hum0[5];
char hum1[5];
char hum2[5];
char hum3[5];
char hum4[5];
char hum5[5];

char vbat0[5];
char vbat1[5];
char vbat2[5];
char vbat3[5];
char vbat4[5];

char ts1[20];
char ts2[20];
char ts3[20];
char ts4[20];
char ts5[20];

//struct ip_addr ip_targetURL;
uint8_t targetIp1;
uint8_t targetIp2;
uint8_t targetIp3;
uint8_t targetIp4;
char *rxBuffer;
volatile uint8_t data_ready = 0;

volatile unsigned int timestamp;
time_t timeStructure;
volatile unsigned char buf[16];
volatile uint32_t RtcIrqIntCounter = 0;
volatile bool salonPendingMsg = false;
volatile bool openWeatherMapPendingMsg = false;

uint8_t mac_address[6];
uint8_t ip_address[] = { 192, 168, 0, 120 };
uint8_t i;

uint8_t timeapiIpAddr[4];
volatile bool timeapiIpAddrFoundFlag = false;
volatile bool dnsCallbackCalled = false;
volatile bool dhcpAddrSet = false;
volatile bool internetStarted = false;

static char status[10];
static char message[10];
static char countryCode[10];
static char zoneName[20];
static char abbreviation[10];
static char gmtOffset[10];
static char dst[10];
char json_buffer[2048];
char outTempBuf[21];
char urlParamStr[100];

static const struct json_attr_t json_timeserver_msg_attrs[] =
		{ { "status", t_string, .addr.string = status, .len = sizeof(status) },
				{ "message", t_string, .addr.string = message, .len =
						sizeof(message) },
				{ "countryCode", t_string, .addr.string = countryCode, .len =
						sizeof(countryCode) }, { "zoneName", t_string,
						.addr.string = zoneName, .len = sizeof(zoneName) }, {
						"abbreviation", t_string, .addr.string = abbreviation,
						.len = sizeof(abbreviation) }, { "gmtOffset", t_string,
						.addr.string = gmtOffset, .len = sizeof(gmtOffset) },
				{ "dst", t_string, .addr.string = dst, .len = sizeof(dst) }, {
						"timestamp", t_uinteger, .addr.uinteger = &timestamp },
				{ NULL }, };



void updateVbat0(char * vbat0) {
	float vbat0f;

	vbat0f = (float) TM_ADC_ReadVbat(ADC1);
	vbat0f = vbat0f / 1000.0f;
	sprintf(vbat0, "%1.2f", vbat0f);
}

void setRTCAlarmForTimeSynchronization() {
	/* Struct for alarm time */
	TM_RTC_AlarmTime_t AlarmTime;

	/* Set alarm A each day 1 (Monday) in a week */
	/* Alarm will be first triggered 5 seconds later as time is configured for RTC */
	AlarmTime.hours = 00;
	AlarmTime.minutes = 01;
	AlarmTime.seconds = 00;
	AlarmTime.alarmtype = TM_RTC_AlarmType_DayInWeek;
	AlarmTime.day = 7;

	/* Set RTC alarm A, time in binary format */
	TM_RTC_SetAlarm(TM_RTC_Alarm_A, &AlarmTime, TM_RTC_Format_BIN);

}

/* Custom request handler function */
/* Called on alarm A interrupt */
void TM_RTC_AlarmAHandler(void) {
	/* Show user to USART */
	printf("Alarm A triggered!\n");
	TM_ETHERNETCLIENT_Connect("api.timezonedb.com", timeapiIpAddr[0], timeapiIpAddr[1], timeapiIpAddr[2], timeapiIpAddr[3], 80,
			"?zone=Europe/Warsaw&format=json&key=G7BLC6X458B0");

	/* Disable Alarm so it will not trigger next week at the same time */
	//TM_RTC_DisableAlarm(TM_RTC_Alarm_A);
}



void TM_RTC_RequestHandler(void) {//called every 10s

	RtcIrqIntCounter++;
	if (RtcIrqIntCounter % 6 == 0) {
		salonPendingMsg = true;

	}

	if (RtcIrqIntCounter % 30 == 0) {
		openWeatherMapPendingMsg = true;
			/* Read RTC clock */
			TM_RTC_GetDateTime(&RTC_Data, TM_RTC_Format_BIN);

			/* Print current time to USART */
			printf("Current date: %02d:%02d:%02d\n", RTC_Data.hours, RTC_Data.minutes,
			RTC_Data.seconds);
			printf("RTC_RequestHandler invoked every 90 sec\n\r");

	}


}

int main(void) {

	uint8_t ret = 0;
	int status = 0;
	char rx[64];
	int bytesReceived = 0;

	/* OneWire working struct */
	TM_OneWire_t OneWire1;

	//struct fs_file file;
	FIL file;
	FRESULT res;
	float hum0f, temp0f;

	struct ip_addr ip;
	struct ip_addr gtw;
	struct ip_addr netmask1;
	TM_ETHERNET_Result_t connResult;
	//uint8_t get_params[200];

	SysTick_Init();

	/* Initialize system */
	SystemInit();

	/* Initialize I2C, SCL: PB6 and SDA: PB7 with 100kHt serial clock */
	TM_I2C_Init(I2C1, TM_I2C_PinsPack_1, 100000);

	hum0f = readHumidity();
	//temp0f = readTemperature();

	RFM69_GPIO_Init();
	RFM69_reset();

	/* Init USART6, TX: PC6 for debug */
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 115200);

	/* Enable watchdog, 4 seconds before timeout */

	printf("Start\r\n");



	 if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_16s)) {
		 printf("Reset occured because of Watchdog(init)\n");
	 }



	/* Initialize delay */
	TM_DELAY_Init();

	/************ DS18B20 sensor code *********************/
	/* Initialize OneWire on pin PD0 */

	//DS_1820_Init(&OneWire1);
	//DS_1820_readTemp(&OneWire1, inTemp);
	/* Initialize ADC1 */
	TM_ADC_InitADC(ADC1);

	/* Enable vbat channel */
	TM_ADC_EnableVbat();

	updateVbat0(vbat0);

	// init RF module and put it to sleep
	RFM69_init(RF69_868MHZ, 100);

	RFM69_setAESEncryption("sampleEncryptKey", 16);
	// send a packet and let RF module sleep

	RFM69_dumpRegisters();

	char testdata[18];
	float humd = 55.5f;
	float temp = 33.3f;

	char temps[5];
	char humds[5];
	unsigned char payload[24];
	int sentResult;




	//RFM69_sleep();

	// set output power
	RFM69_setPowerDBm(10); // +10 dBm

	// enable CSMA/CA algorithm
	RFM69_setCSMA(true);
	//RFM69_setCSMA(false);

	RFM69_sleep();

	/*******************************************************/

	/* Initialize leds on board */
	TM_DISCO_LedInit();

	/* Initialize button */
	TM_DISCO_ButtonInit();

	/* Display to user */
	printf("Program starting..\n");

	/* Initialize RTC with external clock if not already */
	if (!TM_RTC_Init(TM_RTC_ClockSource_External)) {

	};

	TM_RTC_Interrupts(TM_RTC_Int_10s);
	setRTCAlarmForTimeSynchronization();

	/* Initialize ethernet peripheral */
	/* All parameters NULL, default options for MAC, static IP, gateway and netmask will be used */
	/* They are defined in tm_stm32f4_ethernet.h file */

	IP4_ADDR(&ip, 192, 168, 0, 120);
	IP4_ADDR(&gtw, 192, 168, 0, 1);
	IP4_ADDR(&netmask1, 255, 255, 255, 0);

	/* Set MAC address from unique ID */
	for (i = 0; i < 6; i++) {
		/* Set MAC addr */
		mac_address[i] = TM_ID_GetUnique8(11 - i);
	}

	/* Initialize ethernet peripheral */
	/* Set MAC address, set IP address which will be used in case DHCP can't get IP from router */
	if (TM_ETHERNET_Init(mac_address, ip_address, NULL, NULL)
			== TM_ETHERNET_Result_Ok) {
		/* Successfully initialized */
		TM_DISCO_LedOn(LED_GREEN);
	} else {
		/* Unsuccessfull communication */
		TM_DISCO_LedOn(LED_RED);
	}

	/*
	 if (TM_ETHERNET_Init(NULL, &ip, &gtw, &netmask1) == TM_ETHERNET_Result_Ok) {

	 TM_DISCO_LedOn(LED_GREEN);
	 } else {

	 TM_DISCO_LedOn(LED_RED);
	 }
	 */

	/* Reset watchdog */
	TM_WATCHDOG_Reset();

	/* Initialize ethernet server if you want use it, server port 80 */
	connResult = TM_ETHERNETSERVER_Enable(80);

	/* Set SSI tags, we have 37 SSI tags */
	TM_ETHERNETSERVER_SetSSITags(SSI_Tags, 45);

	/* Set CGI tags, we have 1 CGI handler, for leds only */
	TM_ETHERNETSERVER_SetCGIHandlers(CGI_Handlers, 2);

	if (!TM_RTC_Init(TM_RTC_ClockSource_Internal)) {
		/* RTC was first time initialized */
		/* Do your stuff here */
		/* eg. set default time */
	}

	/* Read RTC clock */
	TM_RTC_GetDateTime(&RTC_Data, TM_RTC_Format_BIN);

	/* Print current time to USART */
	printf("Current date: %02d:%02d:%02d\n", RTC_Data.hours, RTC_Data.minutes,
			RTC_Data.seconds);

	/* Reset watchdog */TM_WATCHDOG_Reset();


	rxBuffer = 0;

	/* Reset watchdog */TM_WATCHDOG_Reset();

	//RFM69_dumpRegisters();

	memset(rx, 0, 64);

	strcpy(outHum, "0.0");
	strcpy(outTemp, "0.0");

	strcpy(hum0, "0.0");
	strcpy(temp0, "0.0");
	strcpy(vbat, "0.0");

	strcpy(hum1, "0.0");
	strcpy(temp1, "0.0");
	strcpy(vbat1, "0.0");

	strcpy(hum2, "0.0");
	strcpy(temp2, "0.0");
	strcpy(vbat2, "0.0");

	strcpy(hum3, "0.0");
	strcpy(temp3, "0.0");
	strcpy(vbat3, "0.0");

	strcpy(hum4, "0.0");
	strcpy(temp4, "0.0");

	strcpy(hum5, "0.0");
	strcpy(temp5, "0.0");

	TM_WATCHDOG_Reset();

	printf("Waiting for internet..\n");
	while(!internetStarted);
	printf("Internet started!\n");

	delay_ms(500);

	timeapiIpAddr[0] = 168;
	timeapiIpAddr[1] = 235;
	timeapiIpAddr[2] = 104;
	timeapiIpAddr[3] = 115;


			TM_ETHERNETCLIENT_Connect("api.timezonedb.com", timeapiIpAddr[0], timeapiIpAddr[1], timeapiIpAddr[2],timeapiIpAddr[3], 80, "?zone=Europe/Warsaw&format=json&key=G7BLC6X458B0");
			printf("####### Connect-->api.timezonedb.com\r\n");


	while (1) {

		/* Update ethernet, call this as fast as possible */
		TM_ETHERNET_Update();

		if (salonPendingMsg) {
			hum0f = readHumidity();
			temp0f = readTemperature();

			//hum0f = 33.5f;
			//temp0f = 28.2f;

			sprintf(hum0, "%2.1f", hum0f);
			sprintf(temp0, "%2.1f", temp0f);
			updateVbat0(vbat0);
			printf("Salon [0] T:%s H:%s\n\r", temp0, hum0);

			if(ENABLE_DOMOTICZ){
			sprintf(urlParamStr,
					"json.htm?type=command&param=udevice&idx=6&nvalue=0&svalue=%s;%s;0&battery=%d",
					temp0, hum0,(uint8_t)(atof(vbat0)*100.0f/3.05f));

			connResult = TM_ETHERNETCLIENT_Connect("domoticz", 192, 168, 0, 35,
					8080, urlParamStr);
			}
			salonPendingMsg = false;
		}

		if(openWeatherMapPendingMsg){
			TM_WATCHDOG_Reset();
			sprintf(urlParamStr,"/data/2.5/weather?q=Warsaw,pl&APPID=03af47dbe80f5630cbe86c62cff0d537&lang=pl&units=metric");

			connResult = TM_ETHERNETCLIENT_Connect("api.openweathermap.org", 37,139,20,5,
					80, urlParamStr);
			if(connResult == TM_ETHERNET_Result_Ok){
				printf("TM_ETHERNETCLIENT_Connect::TM_ETHERNET_Result_Ok\r\n");
			} else {
				printf("TM_ETHERNETCLIENT_Connect::TM_ETHERNET_Result_Error\r\n");
			}

			openWeatherMapPendingMsg = false;
		}

#ifdef USE_IRQ
		bytesReceived = RFM69_receive_non_block(rx, 25);
		if (bytesReceived >= 17) {

			TM_RTC_GetDateTime(&RTC_Data, TM_RTC_Format_BIN);
			//DS_1820_readTemp(&OneWire1, inTemp);

			if (rx[2] == 1) { //outTemp

				printf("[1]\n\r");
				if (parseFrameV(rx + 4, outTemp, outHum, vbat)) {
					sprintf(lastFrameTimestamp, "%02d.%02d.%04d %02d:%02d:%02d",
							RTC_Data.date, RTC_Data.month, RTC_Data.year + 2000,
							RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);


					sprintf(outTempBuf, "T:%s H:%s V:%s", outTemp, outHum, vbat);
					sentResult = RFM69_send(outTempBuf, 20, 7);

					if(ENABLE_DOMOTICZ){
						sprintf(urlParamStr,
								"json.htm?type=command&param=udevice&idx=2&nvalue=0&svalue=%s;%s;0&battery=%d",
								outTemp, outHum,(int)(atof(vbat)*100.0f/3.05f));
						TM_WATCHDOG_Reset();
						connResult = TM_ETHERNETCLIENT_Connect("domoticz", 192, 168,
								0, 35, 8080, urlParamStr);
					}
					printf("Balkon:");

					printf(rx + 4);
					printf("\n\r");

					res = f_mount(&fs, "0:", 1);
					res = f_open(&file, "0:www/log.txt",
							FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

					/* Move to end of the file to append data */
					res = f_lseek(&file, f_size(&file));

					//res = f_printf(&file,"%02d.%02d.%04d,%02d:%02d:%02d,%s,%s\r\n",RTC_Data.date,RTC_Data.month, RTC_Data.year+2000, RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds,temperature,vbat);
					res = f_printf(&file,
							"%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n",
							lastFrameTimestamp, outTemp, outHum, inTemp, vbat,
							ts1, temp1, hum1, ts2, temp2, hum2);
					res = f_close(&file);
					res = f_mount(NULL, "", 1);
					//GPIO_ToggleBits(GPIOB, GPIO_Pin_5);
					strcpy(lastFrameTimestampWithSpace, lastFrameTimestamp);
					strcat(lastFrameTimestampWithSpace, " ");
					//delay_ms(50);
					//GPIO_ToggleBits(GPIOB, GPIO_Pin_5);
					//GPIO_ToggleBits(GPIOB, GPIO_Pin_5);

					sentResult = RFM69_send(lastFrameTimestampWithSpace, 20, 6);
					//GPIO_ToggleBits(GPIOB, GPIO_Pin_5);
					memset(rx, 0, 64);
				}

			}

			if (rx[2] == 2) {
				printf("[2]\n\r");
				if (parseFrameV(rx + 4, temp1, hum1, vbat1)) {
					sprintf(ts1, "%02d.%02d.%04d %02d:%02d:%02d", RTC_Data.date,
							RTC_Data.month, RTC_Data.year + 2000,
							RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);

					printf("Pokoj Kasi:");

					printf(rx + 4);
					memset(rx, 0, 64);
				}
				//delay_ms(50);
				sprintf(serverSensorsData, "T:%s H:%s V:%s", temp0, hum0,
						vbat0);
				sentResult = RFM69_send(serverSensorsData, 20, 5);
				if(ENABLE_DOMOTICZ){
					sprintf(urlParamStr,
						"json.htm?type=command&param=udevice&idx=3&nvalue=0&svalue=%s;%s;0&battery=%d",
						temp1, hum1,(uint8_t)(atof(vbat1)*100.0f/3.05f));

					connResult = TM_ETHERNETCLIENT_Connect("domoticz", 192, 168, 0,
						35, 8080, urlParamStr);
				}

			}

			if (rx[2] == 3) {
				printf("[3]\n\r");
				if (parseFrameV(rx + 4, temp2, hum2, vbat2)) {

					sprintf(ts2, "%02d.%02d.%04d %02d:%02d:%02d", RTC_Data.date,
							RTC_Data.month, RTC_Data.year + 2000,
							RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);
					if(ENABLE_DOMOTICZ){
					sprintf(urlParamStr,
							"json.htm?type=command&param=udevice&idx=4&nvalue=0&svalue=%s;%s;0&battery=%d",
							temp2, hum2,(uint8_t)(atof(vbat2)*100.0f/3.05f));

					connResult = TM_ETHERNETCLIENT_Connect("domoticz", 192, 168,
							0, 35, 8080, urlParamStr);
					}
					printf("Pokoj Basi:");
					printf(rx + 4);
					printf(" *\n\r");
					memset(rx, 0, 64);
				}

			}

			if (rx[2] == 4) {
				printf("[4]\n\r");
				if (parseFrameV(rx + 4, temp3, hum3, vbat3)) {

					sprintf(ts3, "%02d.%02d.%04d %02d:%02d:%02d", RTC_Data.date,
							RTC_Data.month, RTC_Data.year + 2000,
							RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);
					if(ENABLE_DOMOTICZ){
					sprintf(urlParamStr,
							"json.htm?type=command&param=udevice&idx=5&nvalue=0&svalue=%s;%s;0&battery=%d",
							temp3, hum3,(uint8_t)(atof(vbat3)*100.0f/3.05f));

					connResult = TM_ETHERNETCLIENT_Connect("domoticz", 192, 168,
							0, 35, 8080, urlParamStr);
					}
					printf("Sypialnia:");
					printf(rx + 4);
					printf(" *\n\r");
					memset(rx, 0, 64);
				}

			}

		}
#else
		bytesReceived = RFM69_receive(rx, 17);
#endif

		/* Reset watchdog */
		TM_WATCHDOG_Reset();

	}

}

/* Delay 1ms handler */
void TM_DELAY_1msHandler(void) {
	/* Time update for ethernet, 1ms */
	/* Add 1ms time for ethernet */
	TM_ETHERNET_TimeUpdate(1);
}

/* Handle CGI request for LEDS */
const char* LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[],
		char *pcValue[]) {
	uint8_t i;

	/* This function handles request like one below: */
	/* /ledaction.cgi?ledtoggle=1&ledoff=2 */
	/* This will toggle LED 1 and turn off LED 2 */

	/* Callback */
	if (iIndex == 0) {
		/* Go through all params */
		for (i = 0; i < iNumParams; i++) {
			/* If current pair = ledtoggle=someled */
			if (strstr(pcParam[i], "ledtoggle")) {
				/* Switch first character */
				switch (pcValue[i][0]) {
				case '1':
					TM_DISCO_LedToggle(LED_GREEN);
					break;
				case '2':
					TM_DISCO_LedToggle(LED_ORANGE);
					break;
				case '3':
					TM_DISCO_LedToggle(LED_RED);
					break;
				case '4':
					TM_DISCO_LedToggle(LED_BLUE);
					break;
				default:
					break;
				}
			} else if (strstr(pcParam[i], "ledon")) {
				switch (pcValue[i][0]) {
				case '1':
					TM_DISCO_LedOn(LED_GREEN);
					break;
				case '2':
					TM_DISCO_LedOn(LED_ORANGE);
					break;
				case '3':
					TM_DISCO_LedOn(LED_RED);
					break;
				case '4':
					TM_DISCO_LedOn(LED_BLUE);
					break;
				default:
					break;
				}
			} else if (strstr(pcParam[i], "ledoff")) {
				switch (pcValue[i][0]) {
				case '1':
					TM_DISCO_LedOff(LED_GREEN);
					break;
				case '2':
					TM_DISCO_LedOff(LED_ORANGE);
					break;
				case '3':
					TM_DISCO_LedOff(LED_RED);
					break;
				case '4':
					TM_DISCO_LedOff(LED_BLUE);
					break;
				default:
					break;
				}
			}
		}
	}

	/* Return URL to be used after call */
	return "/index.shtml";
}

const char* TEMP_CGI_Handler(int iIndex, int iNumParams, char *pcParam[],
		char *pcValue[]) {
	uint8_t i;

	/* This function handles request like one below: */
	/* /ledaction.cgi?ledtoggle=1&ledoff=2 */
	/* This will toggle LED 1 and turn off LED 2 */

	/* Callback */
	if (iIndex == 0) {
		/* Go through all params */
		for (i = 0; i < iNumParams; i++) {
			/* If current pair = ledtoggle=someled */
			if (strstr(pcParam[i], "ledtoggle")) {
				/* Switch first character */
				switch (pcValue[i][0]) {
				case '1':
					TM_DISCO_LedToggle(LED_GREEN);
					break;
				case '2':
					TM_DISCO_LedToggle(LED_ORANGE);
					break;
				case '3':
					TM_DISCO_LedToggle(LED_RED);
					break;
				case '4':
					TM_DISCO_LedToggle(LED_BLUE);
					break;
				default:
					break;
				}
			} else if (strstr(pcParam[i], "ledon")) {
				switch (pcValue[i][0]) {
				case '1':
					TM_DISCO_LedOn(LED_GREEN);
					break;
				case '2':
					TM_DISCO_LedOn(LED_ORANGE);
					break;
				case '3':
					TM_DISCO_LedOn(LED_RED);
					break;
				case '4':
					TM_DISCO_LedOn(LED_BLUE);
					break;
				default:
					break;
				}
			} else if (strstr(pcParam[i], "ledoff")) {
				switch (pcValue[i][0]) {
				case '1':
					TM_DISCO_LedOff(LED_GREEN);
					break;
				case '2':
					TM_DISCO_LedOff(LED_ORANGE);
					break;
				case '3':
					TM_DISCO_LedOff(LED_RED);
					break;
				case '4':
					TM_DISCO_LedOff(LED_BLUE);
					break;
				default:
					break;
				}
			}
		}
	}

	/* Return URL to be used after call */
	return "/temp2.json";
}

void TM_ETHERNETDNS_FoundCallback(char* host_name, uint8_t ip_addr1,
		uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4) {


	timeapiIpAddr[0] = ip_addr1;
	timeapiIpAddr[1] = ip_addr2;
	timeapiIpAddr[2] = ip_addr3;
	timeapiIpAddr[3] = ip_addr4;
	dnsCallbackCalled = true;


	//printf("####### Found DNS address: %d\.%d\.%d\.%d for host %s\r\n",timeapiIpAddr[0],timeapiIpAddr[1],timeapiIpAddr[2],timeapiIpAddr[3],host_name);

	if(!strcmp("api.timezonedb.com",host_name)){


		timeapiIpAddrFoundFlag = true;
	}



	return;
}

void TM_ETHERNETDNS_ErrorCallback(char* host_name) {
	dnsCallbackCalled = true;
	//timeapiIpAddrFoundFlag = false;
	return;
}

/* SSI server callback, always is called this callback */
uint16_t TM_ETHERNETSERVER_SSICallback(int iIndex, char *pcInsert,
		int iInsertLen) {
	uint8_t status;

	/* Return number of characters written */
	if (iIndex < 4) {
		/* First 4 tags are leds */
		/* Get led status */
		switch (iIndex) {
		case 0:
			/* Green LED */
			status = TM_DISCO_LedIsOn(LED_GREEN);
			break;
		case 1:
			/* Orange LED */
			status = TM_DISCO_LedIsOn(LED_ORANGE);
			break;
		case 2:
			/* Red LED */
			status = TM_DISCO_LedIsOn(LED_RED);
			break;
		case 3:
			/* Blue LED */
			status = TM_DISCO_LedIsOn(LED_BLUE);
			break;

		default:
			return 0;
		}

		/* Set string according to status */
		if (status) {
			/* Led is ON */
			sprintf(pcInsert, "<span class=\"green\">On</span>");
		} else {
			/* Led is OFF */
			sprintf(pcInsert, "<span class=\"red\">Off</span>");
		}
	} else if (iIndex == 4) {
		/* #serv_adr tag is requested */
		sprintf(pcInsert, "%d.%d.%d.%d", TM_ETHERNET_GetLocalIP(0),
				TM_ETHERNET_GetLocalIP(1), TM_ETHERNET_GetLocalIP(2),
				TM_ETHERNET_GetLocalIP(3));
	} else if (iIndex == 5) {
		/* #clt_a_c tag */
		sprintf(pcInsert, "%u", TM_ETHERNETCLIENT_GetConnectionsCount());
	} else if (iIndex == 6) {
		/* #clt_s_c tag */
		sprintf(pcInsert, "%u",
				TM_ETHERNETCLIENT_GetSuccessfullConnectionsCount());
	} else if (iIndex == 7) {
		/* #clt_per tag */
		if (TM_ETHERNETCLIENT_GetConnectionsCount() == 0) {
			strcpy(pcInsert, "0 %");
		} else {
			sprintf(pcInsert, "%f %%",
					(float) TM_ETHERNETCLIENT_GetSuccessfullConnectionsCount()
							/ (float) TM_ETHERNETCLIENT_GetConnectionsCount()
							* 100);
		}
	} else if (iIndex == 8) {
		/* #clt_tx tag */
		sprintf(pcInsert, "%llu", TM_ETHERNETCLIENT_GetTXBytes());
	} else if (iIndex == 9) {
		/* #clt_rx tag */
		sprintf(pcInsert, "%llu", TM_ETHERNETCLIENT_GetRXBytes());
	} else if (iIndex == 10) {
		/* #srv_c tag */
		sprintf(pcInsert, "%u", TM_ETHERNETSERVER_GetConnectionsCount());
	} else if (iIndex == 11) {
		/* #srv_tx tag */
		sprintf(pcInsert, "%llu", TM_ETHERNETSERVER_GetTXBytes());
	} else if (iIndex == 12) {
		/* #srv_rx tag */
		sprintf(pcInsert, "%llu", TM_ETHERNETSERVER_GetRXBytes());
	} else if (iIndex == 13) {
		/* #mac_adr */
		sprintf(pcInsert, "%02X-%02X-%02X-%02X-%02X-%02X",
				TM_ETHERNET_GetMACAddr(0), TM_ETHERNET_GetMACAddr(1),
				TM_ETHERNET_GetMACAddr(2), TM_ETHERNET_GetMACAddr(3),
				TM_ETHERNET_GetMACAddr(4), TM_ETHERNET_GetMACAddr(5));
	} else if (iIndex == 14) {
		/* #gateway */
		sprintf(pcInsert, "%d.%d.%d.%d", TM_ETHERNET_GetGateway(0),
				TM_ETHERNET_GetGateway(1), TM_ETHERNET_GetGateway(2),
				TM_ETHERNET_GetGateway(3));
	} else if (iIndex == 15) {
		/* #netmask */
		sprintf(pcInsert, "%d.%d.%d.%d", TM_ETHERNET_GetNetmask(0),
				TM_ETHERNET_GetNetmask(1), TM_ETHERNET_GetNetmask(2),
				TM_ETHERNET_GetNetmask(3));
	} else if (iIndex == 16) {
		/* #link */
		if (TM_ETHERNET_Is100M()) {
			strcpy(pcInsert, "100Mbit");
		} else {
			strcpy(pcInsert, "10Mbit");
		}
	} else if (iIndex == 17) {
		/* #duplex */
		if (TM_ETHERNET_IsFullDuplex()) {
			strcpy(pcInsert, "Full");
		} else {
			strcpy(pcInsert, "Half");
		}
	} else if (iIndex == 18) {
		/* #hardware */
		strcpy(pcInsert, "STM32F4-Discovery");
	} else if (iIndex == 19) {
		/* #rtc_time */
		TM_RTC_GetDateTime(&RTC_Data, TM_RTC_Format_BIN);
		sprintf(pcInsert, "%02d.%02d.%04d %02d:%02d:%02d", RTC_Data.date,
				RTC_Data.month, RTC_Data.year + 2000, RTC_Data.hours,
				RTC_Data.minutes, RTC_Data.seconds);
		/*
		 sprintf(pcInsert, "%04d-%02d-%02d %02d:%02d:%02d", RTC_Data.year + 2000,
		 RTC_Data.month, RTC_Data.date, RTC_Data.hours, RTC_Data.minutes,
		 RTC_Data.seconds);
		 */
	} else if (iIndex == 20) {
		/* #compiled */
		strcpy(pcInsert, __DATE__ " at " __TIME__);
	} else if (iIndex == 21) {
		/* #temperature */
		strcpy(pcInsert, outTemp);
	} else if (iIndex == 22) {
		/* #outHum */
		strcpy(pcInsert, outHum);
	} else if (iIndex == 23) {
		/* #vbat */
		strcpy(pcInsert, vbat);
	} else if (iIndex == 24) {
		/* #lastFrameTimestamp */
		strcpy(pcInsert, lastFrameTimestamp);
	} else if (iIndex == 25) {
		/* #inTemp */
		strcpy(pcInsert, inTemp);
	} else if (iIndex == 26) {
		/* #temp1 */
		strcpy(pcInsert, temp1);
	} else if (iIndex == 27) {
		/* #hum1 */
		strcpy(pcInsert, hum1);
	} else if (iIndex == 28) {
		/* #vbat11 */
		strcpy(pcInsert, vbat1);
	} else if (iIndex == 29) {
		/* #ts1 */
		strcpy(pcInsert, ts1);
	} else if (iIndex == 30) {
		/* #temp2 */
		strcpy(pcInsert, temp2);
	} else if (iIndex == 31) {
		/* #hum2 */
		strcpy(pcInsert, hum2);
	} else if (iIndex == 32) {
		/* #vbat2 */
		strcpy(pcInsert, vbat2);
	} else if (iIndex == 33) {
		/* #ts2 */
		strcpy(pcInsert, ts2);
	} else if (iIndex == 34) {
		/* #temp3 */
		strcpy(pcInsert, temp3);
	} else if (iIndex == 35) {
		/* #hum3 */
		strcpy(pcInsert, hum3);
	} else if (iIndex == 36) {
		/* #vbat3 */
		strcpy(pcInsert, vbat3);
	} else if (iIndex == 37) {
		/* #ts3 */
		strcpy(pcInsert, ts3);
	} else if (iIndex == 42) {
		/* #temp0 */
		strcpy(pcInsert, temp0);
	} else if (iIndex == 43) {
		/* #hum0 */
		strcpy(pcInsert, hum0);
	} else if (iIndex == 44) {
		/* #vbat0 */
		strcpy(pcInsert, vbat0);
	} else {
		/* No valid tag */
		return 0;
	}

	/* Return number of characters written in buffer */
	return strlen(pcInsert);
}

void TM_ETHERNET_IPIsSetCallback(uint8_t ip_addr1, uint8_t ip_addr2,
		uint8_t ip_addr3, uint8_t ip_addr4, uint8_t dhcp) {
	/* Called when we have valid IP, it might be static or DHCP */
	TM_ETHERNET_Result_t connResult;

	if (dhcp) {
		/* IP set with DHCP */
		printf("######### DHCP");
		printf("IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2,
				ip_addr3, ip_addr4);
		dhcpAddrSet = true;
	} else {
		/* Static IP */
		printf("IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2,
				ip_addr3, ip_addr4);
	}

	/* Print MAC address to user */
	printf("MAC: %02X-%02X-%02X-%02X-%02X-%02X\n", TM_ETHERNET_GetMACAddr(0),
			TM_ETHERNET_GetMACAddr(1), TM_ETHERNET_GetMACAddr(2),
			TM_ETHERNET_GetMACAddr(3), TM_ETHERNET_GetMACAddr(4),
			TM_ETHERNET_GetMACAddr(5));
	/* Print netmask to user */
	printf("Netmask: %d.%d.%d.%d\n", TM_ETHERNET_GetGateway(0),
			TM_ETHERNET_GetGateway(1), TM_ETHERNET_GetGateway(2),
			TM_ETHERNET_GetGateway(3));
	/* Print gateway to user */
	printf("Gateway: %d.%d.%d.%d\n", TM_ETHERNET_GetNetmask(0),
			TM_ETHERNET_GetNetmask(1), TM_ETHERNET_GetNetmask(2),
			TM_ETHERNET_GetNetmask(3));
	/* Print 100M link status, 1 = 100M, 0 = 10M */
	printf("Link 100M: %d\n", TM_ETHERNET.speed_100m);
	/* Print duplex status: 1 = Full, 0 = Half */
	printf("Full duplex: %d\n", TM_ETHERNET.full_duplex);

	internetStarted = true;


}

void TM_ETHERNET_DHCPStartCallback(void) {
	/* Called when has DHCP started with getting IP address */
	printf("DHCP has started with assigning IP address\n");
}

void TM_ETHERNET_LinkIsDownCallback(void) {
	/* This function will be called when ethernet cable will not be plugged */
	/* It will also be called on initialization if connection is not detected */

	/* Print to user */
	printf("Link is down, do you have connected to your modem/router?");
}

void TM_ETHERNET_LinkIsUpCallback(void) {
	/* Cable has been plugged in back, link is detected */
	/* I suggest you that you reboot MCU here */
	/* Do important stuff before */

	/* Print to user */
	printf("Link is up back\n");
}

int TM_ETHERNETSERVER_OpenFileCallback(struct fs_file* file, const char* name) {
	FRESULT fres;
	char buffer[100];

	/* Print which file you will try to open */
	printf("Trying to open file %s\n", name);

	/* Mount card, it will be mounted when needed */
	if ((fres = f_mount(&fs, "", 1)) != FR_OK) {
		printf("Mount error: %d\n", fres);
	} else {
		printf("Mount SUCCESS!");
	}

	/* Format name, I have on subfolder everything on my SD card */
	sprintf((char *) buffer, "/www%s", name);

	/* Try to open */
	fres = f_open(&fil[file->id], buffer,
			FA_OPEN_EXISTING | FA_READ | FA_WRITE);

	/* If not opened OK */
	if (fres != FR_OK) {
		printf("Open %s Failure!", buffer);
		printf("Error Code:%d\r\n", fres);
		/* In case we are only opened file, but we didn't succedded */
		if (*file->opened_files_count == 0) {
			/* Unmount card, for safety reason */
			f_mount(NULL, "", 1);
		}

		/* Return 0, opening error */
		return 0;
	} else {
		printf("Open %s SUCCESS!", buffer);
	}

	/* !IMPORTANT; Set file size */
	file->len = f_size(&fil[file->id]);

	/* Return 1, file opened OK */
	return 1;
}

void TM_ETHERNETCLIENT_ReceiveDataCallback(TM_TCPCLIENT_t* connection,
		uint8_t* buffer, uint16_t buffer_length, uint16_t total_length) {
	uint16_t buf_len = buffer_length;
	time_t timeStructure1;
	struct tm time1;
	TM_RTC_t timeStruct;
	TM_RTC_Result_t res;
	uint8_t res1;

	rxBuffer = buffer;

	int status1;


	if (!strcmp(connection->name, "domoticz")
			&& !getJSONPayload(buffer, json_buffer)) {
		printf("Domoticz::Received Data:\n\r");
		printf(json_buffer);
		return;
	}

	if (!strcmp(connection->name, "domoticz2")
			&& !getJSONPayload(buffer, json_buffer)) {
		printf("Domoticz2::Received Data:\n\r");
		printf(json_buffer);
		return;
	}

	if(!strcmp(connection->name, "api.timezonedb.com")){
			buffer[buffer_length] = 0;
			printf("Received api.timezonedb.com data!\n\r");
			if(!getJSONPayload(buffer, json_buffer)) {

				printf("Received JSON:\n\r");
				printf(json_buffer);

				status1 = json_read_object(json_buffer, json_timeserver_msg_attrs, NULL);
				if (!status1) {
					printf("Parser JSON succesfully!\r\n");
					printf("Timestamp: %d\r\n", timestamp);

					timeStructure1 = (time_t) timestamp;

					time1 = *localtime(&timeStructure1);

					//time1ptr = gmtime(timeStructure1);

					timeStruct.seconds = time1.tm_sec;
					timeStruct.minutes = time1.tm_min;
					timeStruct.hours = time1.tm_hour;

					timeStruct.date = time1.tm_mday;
					timeStruct.month = time1.tm_mon + 1;
					timeStruct.year = time1.tm_year - 100;
					//timeStruct.year = 1920;
					timeStruct.day = time1.tm_wday;

					printf("%i -> %s", timeStructure1, asctime(&time1));

					//TM_RTC_SetDateTimeString("17.04.15.6;20:49:30");
					res = TM_RTC_SetDateTime(&timeStruct, TM_RTC_Format_BIN);

					if (res  == TM_RTC_Result_Ok) {
						printf("Time set succesfully\r\n");

					} else {
						printf("Error during time set\r\n");
					}

					//data_ready = 1;
				} else {
					printf("JSON Read status error:%d\r\n", status1);
					printf("JSON Read eror message:%s\r\n", json_error_string(status1));
				}
			}
		return;


	}

	if(!strcmp(connection->name, "api.openweathermap.org")){
		buffer[buffer_length] = 0;
		if(!getJSONPayload(buffer, json_buffer)) {
			printf("[api.openweathermap.org] Received Data:\n\r");
			printf(json_buffer);
			printf("\n\r");

			res1 = parseTempAndHumFromJSON(json_buffer, temp5,hum5);
			if(!res1){
				printf("[api.openweathermap.org] Temp:%s Hum:%s\n\r",temp5,hum5);
				sprintf(outTempBuf, "T:%s H:%s V:%s", temp5, hum5,vbat);
				RFM69_send(outTempBuf, 20, 7);

				if(ENABLE_DOMOTICZ){

					sprintf(urlParamStr,"json.htm?type=command&param=udevice&idx=1&nvalue=0&svalue=%s;%s;0&battery=%d",temp5,hum5,(uint8_t)(atof(vbat)*100.0f/3.05f));
					TM_ETHERNETCLIENT_Connect("domoticz", 192, 168, 0,35, 8080, urlParamStr);
				}
			}
		}
	}
}

int TM_ETHERNETSERVER_ReadFileCallback(struct fs_file* file, char* buffer,
		int count) {
	uint32_t readed;

	/* print debug */
	printf("Trying to read %d bytes from file %s\n", count, file->file_name);

	/* End of file? */
	if (f_eof(&fil[file->id])) {
		return -1;
	}

	/* Read max block */
	if (count > 65535) {
		count = 65535;
	}

	/* Read data */
	f_read(&fil[file->id], buffer, count, &readed);

	/* Return number of bytes read */
	return readed;
}

void TM_ETHERNETSERVER_CloseFileCallback(struct fs_file* file) {
	/* Close file */
	f_close(&fil[file->id]);

	/* Print to user */
	printf("Closing file %s\n", file->file_name);

	/* Unmount in case there is no opened files anymore */
	if (!*file->opened_files_count) {
		/* Unmount, protect SD card */
		f_mount(NULL, "", 1);
	}
}

/* Client is connected */
uint8_t TM_ETHERNETSERVER_ClientConnectedCallback(struct tcp_pcb *pcb) {
	struct ip_addr ip;
	/* Fill bad IP */
	IP4_ADDR(&ip, 84, 12, 16, 46);

	/* Check IP address */
	if (pcb->remote_ip.addr == ip.addr) {
		/* Print to user */
		printf("User with bad IP was trying to access to website\n");
		/* Disable access, show error page */
		return 0;
	}
	/* Print to user */
	printf("Connection allowed\n");
	/* Allow access to others */
	return 1;
}

void TM_ETHERNETSERVER_ClientDisconnectedCallback(void) {
	/* Print to user */
	printf("Client disconnected\n");
}

void TM_ETHERNETCLIENT_ConnectionClosedCallback(TM_TCPCLIENT_t* connection, uint8_t success) {
    /* We are disconnected, done with connection */

	/*
	if (success) {
        printf("Connection %s was successfully closed. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
    } else {
        printf("Connection %s was closed because of error. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
    }
    */

	return;

}

/* For printf function */
int fputc(int ch, FILE *f) {
	/* Send over usart */
	TM_USART_Putc(USART6, ch);

	/* Return character back */
	return ch;
}

uint8_t parseFrame(char *frame, char *temp, char *hum) {
	if (frame[0] == 'T') {
		strncpy(temp, frame + 2, 4);
		temp[4] = 0;
		strncpy(hum, frame + 9, 4);
		hum[4] = 0;

		return 1;
	} else {
		return 0;
	}
}

uint8_t parseFrameV(char *frame, char *temp, char *hum, char *vcc) {
	char * firstSpacePtr;
	char * humStartPtr;
	char * vccStartPtr;
	if (frame[0] == 'T') {
		firstSpacePtr = strchr(frame, ' '); //first space pointer after temperature data

		/** copy temp **/
		strncpy(temp, frame + 2, firstSpacePtr - (frame + 2));
		temp[firstSpacePtr - (frame + 2)] = 0;

		/** copy hum **/
		humStartPtr = firstSpacePtr + 1;
		firstSpacePtr = strchr(humStartPtr, ' ');
		strncpy(hum, humStartPtr + 2, firstSpacePtr - (humStartPtr + 2));
		hum[firstSpacePtr - humStartPtr] = 0;

		/** copy vcc **/
		vccStartPtr = firstSpacePtr + 1;

		strncpy(vcc, vccStartPtr + 2, 4);
		vcc[4] = 0;

		return 1;

	} else {
		return 0;
	}
}

uint8_t parseTempAndHumFromJSON(char *inData, char *tempStr,char *humStr){
	char *ptrStart;
	char *ptrEnd;
	char *decimalPoint;
	ptrStart = strstr(inData,"\"temp\":");
	//"humidity":

	if(ptrStart){
		ptrStart+=7;
		ptrEnd = strstr(ptrStart,",");
		strncpy(tempStr,ptrStart,ptrEnd-ptrStart);
		tempStr[ptrEnd-ptrStart] = 0;
		if((decimalPoint = strstr(tempStr,"."))){ //if number of decimal places after comma is more than one, set end of string after first decimal digit
			*(decimalPoint+2) = 0;
		}

		ptrStart = strstr(inData,"\"humidity\":");
		if(ptrStart){
				ptrStart+=11;
				ptrEnd = strstr(ptrStart,"}");
				strncpy(humStr,ptrStart,ptrEnd-ptrStart);
				//tempStr[ptrEnd-ptrStart] = 0;
				humStr[ptrEnd-ptrStart] = 0;
				strcat(humStr,".0");
		} else {
			return 1;
		}

		return 0;
	} else {
		return 1;
	}

}

