#define SPI_CS_PIN GPIO_Pin_0
#define SPI_CLK_PIN GPIO_Pin_10
#define SPI_MISO_PIN GPIO_Pin_2
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_CLK_PORT GPIOB
#define SPI_PORT GPIOC


  uint8_t spi_init();

  void spi_deinit();

  uint16_t spi_transfer(uint16_t data);

 void spi_select();

  void spi_unselect();


 uint8_t spi_initClock();

 void RCC_Config2(void);
 void GPIO_SPI2_Config(void);

 void mySPI_SendData(uint8_t adress, uint8_t data);
 void mySPI_Init(void);



