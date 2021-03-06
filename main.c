/**
 *	Coocox project template for ethernet server.
 *	File to be shown to user is stored on SD card
 *
 *	@author		Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@ide		Coocox
 */
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
#include "tm_stm32f4_onewire.h"
#include "tm_stm32f4_ds18b20.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include "rf01.h"

/* File variable */
FIL fil[ETHERNET_MAX_OPEN_FILES];
/* Fatfs variable */
FATFS fs;

/* RTC data variable */
TM_RTC_t RTC_Data;

/* Set SSI tags for handling variables */
static TM_ETHERNET_SSI_t SSI_Tags[] = {
	"led1_s", /* Tag 0 = led1 status */
	"led2_s", /* Tag 1 = led2 status */
	"led3_s", /* Tag 2 = led3 status */
	"led4_s", /* Tag 3 = led4 status */
	"srv_adr",/* Tag 4 = server address */
	"clt_a_c",/* Tag 5 = client all connections */
	"clt_s_c",/* Tag 6 = client successfull connections */
	"clt_per",/* Tag 7 = client percentage */
	"clt_tx", /* Tag 8 = client TX bytes */
	"clt_rx", /* Tag 9 = client RX bytes */
	"srv_c",  /* Tag 10 = server all connections */
	"srv_tx", /* Tag 11 = server TX bytes */
	"srv_rx", /* Tag 12 = server RX bytes */
	"mac_adr",/* Tag 13 = MAC address */
	"gateway",/* Tag 14 = gateway */
	"netmask",/* Tag 15 = netmask */
	"link",   /* Tag 16 = link status */
	"duplex", /* Tag 17 = duplex status */
	"hardware",/* Tag 18 = hardware where code is running */
	"rtc_time",/* Tag 19 = current RTC time */
	"compiled",/* Tag 20 = compiled date and time */
	"temperature",
	"voltage",
	"lFrTimestamp",
	"inTemp"
};

/* LED CGI handler */
const char * LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char * TEMP_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

/* CGI call table, only one CGI used */
TM_ETHERNET_CGI_t CGI_Handlers[] = {
	{"/ledaction.cgi", LEDS_CGI_Handler}, /* LEDS_CGI_Handler will be called when user connects to "/ledaction.cgi" URL */
	{"/gettemp.cgi", TEMP_CGI_Handler}, /* LEDS_CGI_Handler will be called when user connects to "/ledaction.cgi" URL */
};

char temperature[16];
char temp[10];
char vbat[10];
char lastFrameTimestamp[20];
char inTemp[9];
//struct ip_addr ip_targetURL;
uint8_t targetIp1;
uint8_t targetIp2;
uint8_t targetIp3;
uint8_t targetIp4;
char *rxBuffer;
volatile uint8_t data_ready=0;


volatile unsigned int timestamp;
time_t timeStructure;
volatile unsigned char buf[16];

#define EXPECTING_SENSORS	1

int main(void) {

	uint8_t ret = 0;
	int status = 0;

	/* OneWire working struct */
	TM_OneWire_t OneWire1;

	uint8_t devices, i, j, count, alarm_count;
	uint8_t device[EXPECTING_SENSORS][8];
	uint8_t alarm_device[EXPECTING_SENSORS][8];
	float temps[EXPECTING_SENSORS];

	//struct fs_file file;
	FIL file;
	FRESULT res;

	struct ip_addr ip;
	struct ip_addr gtw;
	struct ip_addr netmask1;
	TM_ETHERNET_Result_t connResult;
	uint8_t get_params[200];

	SysTick_Init();
	printf("Prep All\r\n");
	prepAll();

	/* Initialize system */
	SystemInit();

	/* Init USART6, TX: PC6 for debug */
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 115200);

	/* Enable watchdog, 4 seconds before timeout */




	if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_4s)) {

		printf("Reset occured because of Watchdog(init)\n");
	}


	/* Initialize delay */
	TM_DELAY_Init();

	/************ DS18B20 sensor code *********************/
	/* Initialize OneWire on pin PD0 */

	TM_OneWire_Init(&OneWire1, GPIOD, GPIO_Pin_0);


		count = 0;
		devices = TM_OneWire_First(&OneWire1);
		while (devices) {

			count++;


			TM_OneWire_GetFullROM(&OneWire1, device[count - 1]);


			devices = TM_OneWire_Next(&OneWire1);
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
			printf( "No devices on OneWire.\n");
		}



			for (i = 0; i < count; i++) {

				TM_DS18B20_SetResolution(&OneWire1, device[i], TM_DS18B20_Resolution_12bits);
			}


		TM_DS18B20_StartAll(&OneWire1);


		while (!TM_DS18B20_AllDone(&OneWire1));


		for (i = 0; i < count; i++) {

			if (TM_DS18B20_Read(&OneWire1, device[i], &temps[i])) {

				printf("Temp %d: %2.5f; \n", i, temps[i]);
				sprintf(inTemp,"%2.1f",temps[i]);
			} else {

				printf( "Reading error;\n");
			}
		}

	/*******************************************************/


	/* Initialize leds on board */
	TM_DISCO_LedInit();

	/* Initialize button */
	TM_DISCO_ButtonInit();

	/* Display to user */
	printf("Program starting..\n");

	/* Initialize RTC with internal clock if not already */
	if (!TM_RTC_Init(TM_RTC_ClockSource_External)) {
		/* Set default time for RTC */

		/* Set date and time if RTC is not initialized already */
		//TM_RTC_SetDateTimeString("24.04.16.6;18:28:30");
	};

	/* Initialize ethernet peripheral */
	/* All parameters NULL, default options for MAC, static IP, gateway and netmask will be used */
	/* They are defined in tm_stm32f4_ethernet.h file */

	IP4_ADDR(&ip, 192, 168, 0, 120);
	IP4_ADDR(&gtw, 192, 168, 0, 1);
	IP4_ADDR(&netmask1, 255, 255, 255, 0);


	//if (TM_ETHERNET_Init(NULL, NULL, NULL, NULL) == TM_ETHERNET_Result_Ok) {
	if (TM_ETHERNET_Init(NULL, &ip, &gtw, &netmask1) == TM_ETHERNET_Result_Ok) {
		/* Successfully initialized */
		TM_DISCO_LedOn(LED_GREEN);
	} else {
		/* Unsuccessfull communication */
		TM_DISCO_LedOn(LED_RED);
	}

	/* Reset watchdog */
	TM_WATCHDOG_Reset();

	/* Initialize ethernet server if you want use it, server port 80 */
	TM_ETHERNETSERVER_Enable(80);

	/* Set SSI tags, we have 21 SSI tags */
	TM_ETHERNETSERVER_SetSSITags(SSI_Tags, 25);

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
	printf("Current date: %02d:%02d:%02d\n", RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);

	/* Reset watchdog */
	TM_WATCHDOG_Reset();

	strcpy(get_params,"?zone=Europe/Warsaw&format=json&key=G7BLC6X458B0");

	connResult = TM_ETHERNETDNS_GetHostByName("api.timezonedb.com");




	//connResult = TM_ETHERNETCLIENT_Connect("testconn1",192,168,2,100,80,NULL);
	//connResult = TM_ETHERNETCLIENT_Connect("testconn1",140,211,11,105,80,get_params);

	//NASK: 195.187.242.146

	//168.235.88.213
	printf("Connecting to host..\r\n");
	rxBuffer = 0;

	if(connResult == TM_ETHERNET_Result_Ok){
     //connResult = TM_ETHERNETCLIENT_Connect("api.timezonedb.com",targetIp1,targetIp2,targetIp3,targetIp4,80,get_params);
		connResult = TM_ETHERNETCLIENT_Connect("api.timezonedb.com",168,235,88,213,80,get_params);

		if(connResult == TM_ETHERNET_Result_Ok){
			;
		}
	}

	  res = f_mount(&fs,"0:",1);
	  res = f_open(&file, "0:log/log.txt",FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	  res = f_printf(&file,"Current date: %02d.%02d.%04d,%02d:%02d:%02d\n",RTC_Data.date,RTC_Data.month, RTC_Data.year+2000, RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);
	  //f_printf(file,"test\r\n");
	  res = f_close(&file);
	  res = f_mount(NULL,"",1);


	/* Reset watchdog */
		TM_WATCHDOG_Reset();



	while (1) {



		/* Update ethernet, call this as fast as possible */
			TM_ETHERNET_Update();



			if( !(RF01_status.stat&0x03) )
				  rf01_rxstart();

			if( RF01_status.New )	{

					  ret = rf12_rxfinish(buf);


					  if(ret > 0 && ret < 254) {	// brak b��d�w CRC - odebrana ramka
						  printf("Received correct frame: ");
						  printf(buf);
						  printf("\n\r");


						  /* Start temperature conversion on all devices on one bus */

						  	TM_DS18B20_StartAll(&OneWire1);


						  	while (!TM_DS18B20_AllDone(&OneWire1));


							for (i = 0; i < count; i++) {

								if (TM_DS18B20_Read(&OneWire1, device[i], &temps[i])) {

									printf("Temp %d: %2.5f; \n", i, temps[i]);
									sprintf(inTemp,"%2.1f",temps[i]);

								} else {

									printf( "Reading error;\n");
								}
							}



						  if(handleTempData(buf,temperature,vbat)){
							  printf("Frame has been processed!\n\r");
							  TM_RTC_GetDateTime(&RTC_Data, TM_RTC_Format_BIN);

							  sprintf(lastFrameTimestamp,"%02d.%02d.%04d %02d:%02d:%02d",RTC_Data.date,RTC_Data.month, RTC_Data.year+2000, RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);
							  //printf("Current date: %02d.%02d.%04d %02d:%02d:%02d\n",RTC_Data.date,RTC_Data.month, RTC_Data.year+2000, RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds);
							  puts(lastFrameTimestamp);
							  puts("\r\n");
							  res = f_mount(&fs,"0:",1);
							  res = f_open(&file, "0:www/log.txt",FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

							  /* Move to end of the file to append data */
							        res = f_lseek(&file, f_size(&file));

							  //res = f_printf(&file,"%02d.%02d.%04d,%02d:%02d:%02d,%s,%s\r\n",RTC_Data.date,RTC_Data.month, RTC_Data.year+2000, RTC_Data.hours, RTC_Data.minutes, RTC_Data.seconds,temperature,vbat);
							        res = f_printf(&file,"%s,%s,%s,%s\r\n",lastFrameTimestamp,temperature,inTemp,vbat);
							  res = f_close(&file);
							  res = f_mount(NULL,"",1);
						  } else {
							  printf("Frame error\n\r");
						  }


					  }

			  }


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
const char* LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
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

const char* TEMP_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
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

/* SSI server callback, always is called this callback */
uint16_t TM_ETHERNETSERVER_SSICallback(int iIndex, char *pcInsert, int iInsertLen) {
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
		sprintf(pcInsert, "%d.%d.%d.%d", TM_ETHERNET_GetLocalIP(0), TM_ETHERNET_GetLocalIP(1), TM_ETHERNET_GetLocalIP(2), TM_ETHERNET_GetLocalIP(3));
	} else if (iIndex == 5) {
		/* #clt_a_c tag */
		sprintf(pcInsert, "%u", TM_ETHERNETCLIENT_GetConnectionsCount());
	} else if (iIndex == 6) {
		/* #clt_s_c tag */
		sprintf(pcInsert, "%u", TM_ETHERNETCLIENT_GetSuccessfullConnectionsCount());
	} else if (iIndex == 7) {
		/* #clt_per tag */
		if (TM_ETHERNETCLIENT_GetConnectionsCount() == 0) {
			strcpy(pcInsert, "0 %");
		} else {
			sprintf(pcInsert, "%f %%", (float)TM_ETHERNETCLIENT_GetSuccessfullConnectionsCount() / (float)TM_ETHERNETCLIENT_GetConnectionsCount() * 100);
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
			TM_ETHERNET_GetMACAddr(0),
			TM_ETHERNET_GetMACAddr(1),
			TM_ETHERNET_GetMACAddr(2),
			TM_ETHERNET_GetMACAddr(3),
			TM_ETHERNET_GetMACAddr(4),
			TM_ETHERNET_GetMACAddr(5)
		);
	} else if (iIndex == 14) {
		/* #gateway */
		sprintf(pcInsert, "%d.%d.%d.%d",
			TM_ETHERNET_GetGateway(0),
			TM_ETHERNET_GetGateway(1),
			TM_ETHERNET_GetGateway(2),
			TM_ETHERNET_GetGateway(3)
		);
	} else if (iIndex == 15) {
		/* #netmask */
		sprintf(pcInsert, "%d.%d.%d.%d",
			TM_ETHERNET_GetNetmask(0),
			TM_ETHERNET_GetNetmask(1),
			TM_ETHERNET_GetNetmask(2),
			TM_ETHERNET_GetNetmask(3)
		);
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
		sprintf(pcInsert, "%04d-%02d-%02d %02d:%02d:%02d",
			RTC_Data.year + 2000,
			RTC_Data.month,
			RTC_Data.date,
			RTC_Data.hours,
			RTC_Data.minutes,
			RTC_Data.seconds
		);
	} else if (iIndex == 20) {
		/* #compiled */
		strcpy(pcInsert, __DATE__ " at " __TIME__);
	}
	else if (iIndex == 21) {
			/* #temperature */
			strcpy(pcInsert, temperature);
		}
	else if (iIndex == 22) {
				/* #vbat */
				strcpy(pcInsert, vbat);
			}
	else if (iIndex == 23) {
					/* #lastFrameTimestamp */
					strcpy(pcInsert, lastFrameTimestamp);
				}
	else if (iIndex == 24) {
						/* #inTemp */
						strcpy(pcInsert, inTemp);
					}
	else {
		/* No valid tag */
		return 0;
	}

	/* Return number of characters written in buffer */
	return strlen(pcInsert);
}

void TM_ETHERNET_IPIsSetCallback(uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4, uint8_t dhcp) {
	/* Called when we have valid IP, it might be static or DHCP */

	if (dhcp) {
		/* IP set with DHCP */
		printf("IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
	} else {
		/* Static IP */
		printf("IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
	}

	/* Print MAC address to user */
	printf("MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
		TM_ETHERNET_GetMACAddr(0),
		TM_ETHERNET_GetMACAddr(1),
		TM_ETHERNET_GetMACAddr(2),
		TM_ETHERNET_GetMACAddr(3),
		TM_ETHERNET_GetMACAddr(4),
		TM_ETHERNET_GetMACAddr(5)
	);
	/* Print netmask to user */
	printf("Netmask: %d.%d.%d.%d\n",
		TM_ETHERNET_GetGateway(0),
		TM_ETHERNET_GetGateway(1),
		TM_ETHERNET_GetGateway(2),
		TM_ETHERNET_GetGateway(3)
	);
	/* Print gateway to user */
	printf("Gateway: %d.%d.%d.%d\n",
		TM_ETHERNET_GetNetmask(0),
		TM_ETHERNET_GetNetmask(1),
		TM_ETHERNET_GetNetmask(2),
		TM_ETHERNET_GetNetmask(3)
	);
	/* Print 100M link status, 1 = 100M, 0 = 10M */
	printf("Link 100M: %d\n", TM_ETHERNET.speed_100m);
	/* Print duplex status: 1 = Full, 0 = Half */
	printf("Full duplex: %d\n", TM_ETHERNET.full_duplex);
}

void TM_ETHERNET_DHCPStartCallback(void) {
	/* Called when has DHCP started with getting IP address */
	printf("Trying to get IP address via DHCP\n");
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
	sprintf((char *)buffer, "/www%s", name);



	/* Try to open */
	fres = f_open(&fil[file->id], buffer, FA_OPEN_EXISTING | FA_READ | FA_WRITE);

	/* If not opened OK */
	if (fres != FR_OK) {
		printf("Open %s Failure!",buffer);
		printf("Error Code:%d\r\n",fres);
		/* In case we are only opened file, but we didn't succedded */
		if (*file->opened_files_count == 0) {
			/* Unmount card, for safety reason */
			f_mount(NULL, "", 1);
		}

		/* Return 0, opening error */
		return 0;
	} else {
		printf("Open %s SUCCESS!",buffer);
	}

	/* !IMPORTANT; Set file size */
	file->len = f_size(&fil[file->id]);

	/* Return 1, file opened OK */
	return 1;
}

int TM_ETHERNETSERVER_ReadFileCallback(struct fs_file* file, char* buffer, int count) {
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

/* For printf function */
int fputc(int ch, FILE *f) {
	/* Send over usart */
	TM_USART_Putc(USART6, ch);

	/* Return character back */
	return ch;
}




