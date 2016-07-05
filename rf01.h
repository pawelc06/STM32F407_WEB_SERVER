
#define RF01_UseIRQ 1

#include "stm32f4xx_gpio.h"

#ifndef cbi
#define clear_pin(sfr, bit)     (GPIO_ResetBits(sfr,bit) )
#endif
#ifndef sbi
#define set_pin(sfr, bit)     (GPIO_SetBits(sfr,bit) )
#endif

#ifndef cLED0
#define cLED0()     GPIO_ResetBits(GPIOD, GPIO_Pin_12)
#endif
#ifndef sLED0
#define sLED0()     GPIO_SetBits(GPIOD, GPIO_Pin_12)
#endif

#ifndef cLED1
#define cLED1()     GPIO_ResetBits(GPIOD, GPIO_Pin_13)
#endif
#ifndef sLED1
#define sLED1()     GPIO_SetBits(GPIOD, GPIO_Pin_13)
#endif

extern void rf01_trans(unsigned short wert);
extern void rf01_init(void);
extern void rf01_setbandwidth(unsigned char bandwidth);
extern void rf01_setreceiver(unsigned char gain, unsigned char drssi);
extern void rf01_setfreq(unsigned short freq);
extern void rf01_setbaud(unsigned short baud);
extern void rf01_rxdata(unsigned char *data, unsigned char number);
void prepAll();

#define RF01FREQ(freq)	((freq-430.0)/0.0025)

extern void blinkLED(void);
extern void makePulse(int numberOfPulses);

#define RF_PORT GPIOB

#define SDI		GPIO_Pin_0	// SDI,  -> RF02 13 YELLOW
#define SCK		GPIO_Pin_1	// SCK,  -> RF02 12 BLUE
#define CS		GPIO_Pin_2	// nSEL, -> RF02 11 VIOLET
#define SDO		GPIO_Pin_10	// SDO,  <- RF02 GREEN

typedef union {
	uint8_t stat;
	struct {
		uint8_t Rx:1;
		uint8_t New:1;
	};
} RF01_STAT;


extern volatile RF01_STAT RF01_status;

#if RF01_UseIRQ == 1

#define RFM01_GPIO_PORT            GPIOA                    /*!< Port which IR output is connected */
#define RFM01_GPIO_PORT_CLK        RCC_AHB1Periph_GPIOA      /*!< IR pin GPIO Clock Port */
#define RFM01_GPIO_PIN             GPIO_Pin_0               /*!< Pin which IR is connected */
#define RFM01_EXTI_LINE			EXTI_Line0
#define RFM01_NVIC_IRQn			EXTI1_IRQn
#define RFM01_EXTI_PORT_SOURCE		EXTI_PortSourceGPIOA
#define RFM01_EXTI_PIN_SOURCE		EXTI_PinSource0

#define RF01_DataLength	12		//max length 243
#endif

#if RF01_UseIRQ == 1

//! start odbioru ramki danych
uint8_t rf01_rxstart(void);

//! odczytanie do bufora gdy zostanie poprawnie odebrana
uint8_t rf01_rxfinish( char *data);


//! zatrzymanie wszystkich operacji nadwania i odbioru
void rf01_allstop(void);
#endif

