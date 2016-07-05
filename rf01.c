#include "rf01.h"
#include "stm32f4xx_gpio.h"
#include "delay.h"


/*
#define SDI		5	// SDI,  -> RF02 13
#define SCK		4	// SCK,  -> RF02 12
#define CS		3	// nSEL, -> RF02 11
#define SDO		2	// SDO,  <- RF02
*/






// nFFS: 1-10k Pullup an Vcc !!!

static unsigned char sdrssi, sgain;


void rf01_trans(unsigned short val)
{	unsigned char i;

	clear_pin(RF_PORT, CS);
	for (i=0; i<16; i++)
	{	if (val&32768)
			set_pin(RF_PORT, SDI);
		else
			clear_pin(RF_PORT, SDI);
		set_pin(RF_PORT, SCK);
		val<<=1;


		//delay_us(1);
		delay_200ns();

		clear_pin(RF_PORT, SCK);
	}
	set_pin(RF_PORT, CS);
}

void rf01_init(void)
{
	unsigned char i;
	GPIO_InitTypeDef  GPIO_InitStructure;




/* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = SDI | SCK| CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SDO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

  set_pin(GPIOB,CS);

	for (i=0; i<11; i++)

		/*** !!!!!!!!!!!!!!!!!!!!!!!!! DELAY ******************/
		delay_ms(10);			// wait until POR done

	rf01_trans(0xC2E0);			// AVR CLK: 10MHz
	rf01_trans(0xC42B);			// Data Filter: internal
	rf01_trans(0xCE88);			// FIFO mode
	rf01_trans(0xC6F7);			// AFC settings: autotuning: -10kHz...+7,5kHz
	rf01_trans(0xE000);			// disable wakeuptimer
	rf01_trans(0xCC00);			// disable low duty cycle


	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

#if RF01_UseIRQ == 1
		// jeœli obs³uga na przerwaniach
		RF01_status.Rx 	= 0;
		RF01_status.New = 0;
		RFM01Irq_Init();
#endif

	blinkLED();

}

void rf01_setbandwidth(unsigned char bandwidth)
{
	rf01_trans(0x8970|((bandwidth&7)<<1));
}

void rf01_setreceiver(unsigned char gain, unsigned char drssi)
{
	sdrssi=drssi;
	sgain=gain;
}

void rf01_setfreq(unsigned short freq)
{	if (freq<96)				// 430,2400MHz
		freq=96;
	else if (freq>3903)			// 439,7575MHz
		freq=3903;
	rf01_trans(0xA000|freq);
}

void rf01_setbaud(unsigned short baud)
{
	if (baud<336)
		return;
	if (baud<5400)				// Baudrate= 344827,58621/(R+1)/(1+CS*7)
		rf01_trans(0xC880|((43104/baud)-1));
	else
		rf01_trans(0xC800|((344828UL/baud)-1));

	rf01_trans(0xC806);
}

void rf01_rxdata(unsigned char *data, unsigned char number)
{	unsigned char i,j,c;

	rf01_trans(0xC0C1|((sgain&3)<<4)|((sdrssi&7)<<1));	// RX on
	rf01_trans(0xCE89);			// set FIFO mode
	rf01_trans(0xCE8B);			// enable FIFO
	clear_pin(RF_PORT, SDI); //SDI = LOW
	for (i=0; i<number; i++){
		clear_pin(RF_PORT, CS);
		//while (!(RF_PIN&(1<<SDO))); // wait until data in FIFO
		while (!(GPIO_ReadInputDataBit(RF_PORT,SDO))); // wait until data in FIFO
		for (j=0; j<16; j++)	// read and discard status register
		{	set_pin(RF_PORT, SCK);
			asm("nop");
			clear_pin(RF_PORT, SCK);
		}
		c=0;
		for (j=0; j<8; j++)
		{	c<<=1;
			//if (RF_PIN&(1<<SDO))
		if(GPIO_ReadInputDataBit(RF_PORT,SDO))
				c|=1;
			set_pin(RF_PORT, SCK);

			//delay_us(1);
			delay_200ns();



			clear_pin(RF_PORT, SCK);
		}
		*data++=c;
		set_pin(RF_PORT, CS);
	}
	//blinkLED();
	rf01_trans(0xC0C0|((sgain&3)<<4)|((sdrssi&7)<<1));	// RX off
}

void blinkLED(void){
	for (unsigned char i=0; i<15; i++)

		/*** !!!!!!!!!!!!!!!!!!!!!!!!! DELAY ******************/
		delay_ms(5);
	sLED1();

	for (unsigned char i=0; i<15; i++)
		/*** !!!!!!!!!!!!!!!!!!!!!!!!! DELAY ******************/
		delay_ms(5);
	cLED1();

}

void makePulse(int numberOfPulses){
	if ( numberOfPulses > 0)
	{
	for (unsigned char i=0; i<numberOfPulses; i++)
		{
		/*** !!!!!!!!!!!!!!!!!!!!!!!!! DELAY ******************/
		delay_ms(20);
			sLED0();
			/*** !!!!!!!!!!!!!!!!!!!!!!!!! DELAY ******************/
			delay_ms(20);
			cLED0();			
		}
	/*** !!!!!!!!!!!!!!!!!!!!!!!!! DELAY ******************/
	delay_ms(50);
		}

}

#if RF01_UseIRQ == 1

volatile RF01_STAT RF01_status;
volatile uint8_t RF01_Index = 0;
uint8_t RF01_Data[ RF01_DataLength+10 ];	// +10 nadmiarowo na pozosta³e czêœci ramki

void RFM01Irq_Init(void) {
	EXTI_InitTypeDef   EXTI_InitStructure;
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;

	  /* Enable GPIOA clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PA0 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect EXTI Line0 to PA0 pin */
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	  /* Configure EXTI Line0 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

}

uint8_t rf01_rxstart(void) {

	if(RF01_status.New)
		return(1);			//bufor jeszcze nie pusty

	if(RF01_status.Rx)
		return(3);			//trwa odbieranie


	rf01_trans(0xC0C1|((sgain&3)<<4)|((sdrssi&7)<<1));	// RX on
		rf01_trans(0xCE88);			// set FIFO mode
		rf01_trans(0xCE8B);			// enable FIFO


	clear_pin(RF_PORT, SDI);


	//cbi(RF_PORT, SDO); //SDI input

	RF01_Index = 0;
	RF01_status.Rx = 1;



	return(0);				//wszystko w porz¹dku
}

uint8_t rf12_rxfinish( char *data ) {
	uint16_t crc, crc_chk = 0;
	uint8_t crc8, crc_chk8=0;
	uint8_t i, size = 12;
	if(RF01_status.Rx) return(255);		//odbiór jeszcze nie zakoñczony
	if(!RF01_status.New) return(254);	//jeszcze stare dane w buforze

	if( size > RF01_DataLength ) {
		data[0] = 0;
		RF01_status.New = 0;
		return 0; // b³¹d wielkoœci ramki
	}


	crc_chk8 = calculate_crc(RF01_Data);
	crc8=RF01_Data[size];
	/*
	for(i=0; i<size +1 ; i++)
		crc_chk = crcUpdate(crc_chk, RF01_Data[i]);

	crc = RF01_Data[i++];
	crc |= RF01_Data[i] << 8;
	*/
	crc = 0;

	RF01_status.New = 0;

	if(crc8 != crc_chk8) return(0);		//b³¹d sumy CRC lub rozmiaru ramki
	else {
		for(i=0; i<size-1; i++)
			data[i] = RF01_Data[i];

		data[ size-1 ] = 0;		// zakoñczenie ramki zerem
		return( size-1 );			// rozmiar odebranej ramki w bajtach
	}
}

void EXTI0_IRQHandler(void) {
	unsigned char i,j,c;

  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    /* Toggle LED1 */

	  GPIO_SetBits(GPIOD,GPIO_Pin_12);

	  if(RF01_Index < RF01_DataLength){
	//for (i=0; i<RF01_DataLength; i++){
	  clear_pin(RF_PORT, CS);
	  while (!(GPIO_ReadInputDataBit(RF_PORT,SDO))); // wait until data in FIFO


    		for (j=0; j<16; j++)	// read and discard status register
    		{	set_pin(RF_PORT, SCK);
    			asm("nop");
    			asm("nop");
    			clear_pin(RF_PORT, SCK);
    		}
    		c=0;
    		for (j=0; j<8; j++)
    		{	c<<=1;
    			//if (RF_PIN&(1<<SDO))


    		if(GPIO_ReadInputDataBit(RF_PORT,SDO))
    				c|=1;
    			set_pin(RF_PORT, SCK);

    			//delay_us(1);
    			delay_200ns();



    			clear_pin(RF_PORT, SCK);
    		}
    		RF01_Data[RF01_Index++]=c;
    		set_pin(RF_PORT, CS);

	  //}
	  } else {

		rf01_trans(0xC0C0|((sgain&3)<<4)|((sdrssi&7)<<1));	// RX off

		RF01_status.Rx = 0;
		RF01_status.New = 1;
		return; 				// na pewno bêdzie b³êdna ramka
	  }

	  if(RF01_Index >= 12) {

		//rf12_trans(0x8208); //idle mode
		rf01_trans(0xC0C0|((sgain&3)<<4)|((sdrssi&7)<<1));	// RX off

		RF01_status.Rx = 0;
		RF01_status.New = 1;	// poprawnie zakoñczona ramka
	}

	  //while (!(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)));


    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }

}
#endif

void prepAll(){
	rf01_init();					// ein paar Register setzen (z.B. CLK auf 10MHz)
	rf01_setfreq(RF01FREQ(434));	// Sende/Empfangsfrequenz auf 433,92MHz einstellen
	rf01_setbandwidth(4);			// 4 200kHz Bandbreite
	rf01_setreceiver(2,4);			//2,4 -6dB Verstï¿½rkung, DRSSI threshold: -79dBm
	rf01_setbaud(57600);			// 19200 Baud

}
