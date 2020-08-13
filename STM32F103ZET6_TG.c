#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_utils.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_usart.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_system.h>
#include <stm32f1xx_ll_utils.h>
#include <stm32f1xx_ll_spi.h>
#include <stm32f1xx_ll_i2c.h>
#include <stm32f1xx_ll_dma.h>
#include <stm32f1xx_ll_cortex.h>
#include "stdlib.h"

//--------|  COMM LIBS
#include "CL_CONFIG.h"
#include "CL_delay.h"
#include "CL_systemClockUpdate.h"
#include "CL_printMsg.h"
#include "CL_CFA_240x320.h"
#include "font-ascii-12x16.h"


void initLED_D4(void);
//--------|  Prototypes
void F12x16__printMsg(uint16_t x, uint16_t y, char *msg, ...);
void F12x16_DrawString(uint16_t x, uint16_t y, const char* text);
void F12x16_DrawChar(uint16_t x, uint16_t y, char c);
//void show_BMPs_in_root(void);
void SPI_send_pixels(uint8_t len, uint8_t* data);
void LCD_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t r, uint8_t g, uint8_t b);
void Fast_Horizontal_Line(uint16_t x0, uint16_t y, uint16_t x1, uint8_t r, uint8_t g, uint8_t b);
void LCD_Circle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t R, uint16_t G, uint16_t B);
void LCD_Circle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t R, uint16_t G, uint16_t B);
void Put_Pixel(uint16_t x, uint16_t y, uint8_t R, uint8_t G, uint8_t B);
void Fill_LCD(uint8_t R, uint8_t G, uint8_t B);
void Set_LCD_for_write_at_X_Y(uint16_t x, uint16_t y);
void Initialize_LCD(void);
void SPI_sendCommand(uint8_t cmd);
void SPI_sendData(uint8_t data);
void init_spi(void);
void spiSend(uint8_t *data, uint32_t len);
void led0Setup(void);
void tftGPIO_init(void);

int main(void)
{
	CL_setSysClockTo72();
	CL_printMsg_init_Default(false);
	CL_delay_init();


	initLED_D4();
	init_spi();
	tftGPIO_init();
	CLR_RESET;
	CLR_RS;
	SET_CS;

	Initialize_LCD();	
	Fill_LCD(0, 0, 0xff);

	uint32_t num = 0;
	uint32_t num2 = 50;
	uint16_t x;
	uint16_t sub_x;
	uint16_t y;
	uint16_t sub_y;
	
	
	Fill_LCD(0x00, 0x10, 0x10);
	//draw exit button
	for(x = 210 ; x < 240 ; x++)
	  for(y = 0 ; y < 30 ; y++)
	    if(y == x - 210 || y == 240 - x)
	      Put_Pixel(x, y, 0xFF, 0xFF, 0xFF);
	else
	  Put_Pixel(x, y, 0xFF, 0x00, 0x00);
	
	
	
	for (;;)
	{
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_3);	
	
	delayMS(500);
	}
}
void initLED_D4(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_LOW);

}
void init_spi(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	LL_GPIO_InitTypeDef spiPort;
	LL_GPIO_StructInit(&spiPort);
	
	spiPort.OutputType  = LL_GPIO_OUTPUT_PUSHPULL;
	spiPort.Mode		= LL_GPIO_MODE_ALTERNATE;
	spiPort.Pin			= LL_GPIO_PIN_5;
	spiPort.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
	
	LL_GPIO_Init(GPIOA, &spiPort);
	//GPIOA->AFR[0] = 5;
	
	spiPort.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOA, &spiPort);

	// config SPI
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI); 
	

	
	
	mySPI.BaudRate			= LL_SPI_BAUDRATEPRESCALER_DIV2;    
	mySPI.Mode				= LL_SPI_MODE_MASTER;
	mySPI.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
	mySPI.NSS				= LL_SPI_NSS_SOFT;
	mySPI.DataWidth			= LL_SPI_DATAWIDTH_8BIT;
	LL_SPI_Init(SPI1, &mySPI);
	LL_SPI_Enable(SPI1);	
	
	//Clocks
	//RCC->AHBENR |= RCC_AHBENR_DMA1EN; //DMA1 enable
	//RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN; //GPIOA, SPI1, AFIO, enable
	
	//GPIO  A5:CLOCK  |  A7:MOSI
	//GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF7); //rest pins configs
	//GPIOA->CRL |= GPIO_CRL_CNF5_1 | GPIO_CRL_CNF7_1; //alt function
	
	//SPI
	/*
	SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_MSTR  | SPI_CR1_BR_1 ;
	SPI1->CR2 = SPI_CR2_TXDMAEN;  //when this is enabled the DMA transfer happens
	SPI1->CR1 |= SPI_CR1_SPE;*/
	/*
	LL_GPIO_InitTypeDef spiPort;
	LL_GPIO_StructInit(&spiPort);
	
	spiPort.Mode 	= LL_GPIO_MODE_ALTERNATE;
	spiPort.Speed	= LL_GPIO_SPEED_FREQ_HIGH;
	spiPort.Pin			= LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
	spiPort.OutputType  = LL_GPIO_OUTPUT_PUSHPULL;
	
	LL_GPIO_Init(GPIOA, &spiPort);

	spiPort.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOA, &spiPort);

	// config SPI
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI); 	
	mySPI.BaudRate			= LL_SPI_BAUDRATEPRESCALER_DIV8;   //40Mbits/s
	mySPI.Mode				= LL_SPI_MODE_MASTER;
	mySPI.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
	mySPI.NSS				= LL_SPI_NSS_SOFT;
	mySPI.DataWidth			= LL_SPI_DATAWIDTH_8BIT;
	LL_SPI_Init(SPI1, &mySPI);
	LL_SPI_Enable(SPI1);	
	*/
	/*
	//DMA
	DMA1_Channel3->CNDTR = (uint16_t)len;
	DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_CIRC ;
	DMA1_Channel3->CPAR = (uint32_t)(&SPI1->DR);
	DMA1_Channel3->CMAR = (uint32_t)(&myString);
	DMA1_Channel3->CCR |= DMA_CCR_TCIE; 
	
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	*/
}
void spiSend(uint8_t *data, uint32_t len)
{
	
	uint8_t volatile *spidr = ((__IO uint8_t *)&SPI1->DR);
	while (len > 0)
	{
		while (!(SPI1->SR&SPI_SR_TXE)) {
			;
		}
		*spidr = *data++;
		len--;
	}	
	while ((SPI1->SR&SPI_SR_BSY)) {
		;
	}
	
	
}
void tftGPIO_init(void)
{
	/*
 * Pin Definitions / Maping to TFT and Touch
 * 
 * LCD_MOSI		24 -->	PA7 SPI MOSI
 * LCD_SCL		33  --> PA5 SPI CLK
 * LCD_ WRB		32 -->  PB7
 * LCD_CS		34 -->  PB6
 * LCD_RESET	35 -->  PB5
 * 
 * Touch_GND     1/12
 * Touch_Reset	2/11 --> PB4
 * Touch_IRQ	3/10 --> PB3
 * Touch_SDA	4/9  --> PA8 I2C SDA
 * Touch_SCL	5/8  --> PA9 I2C CLK
 * Touch_V+     6/7
 * 
 *
 **/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
	
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Mode			= LL_GPIO_MODE_OUTPUT;
	myGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	myGPIO.Pin			=  LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	myGPIO.Speed		= LL_GPIO_MODE_OUTPUT_50MHz;
	LL_GPIO_Init(GPIOB, &myGPIO);
	
	//still have to add touch pins

}
void initi2c(void)
{
	//------[I2C_2] [ CLOCK :PB10 ] [ DATA : PB11 ]-------
	
	 LL_I2C_InitTypeDef I2C_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	/**I2C2 GPIO Configuration  
	PB10   ------> I2C2_SCL
	PB11   ------> I2C2_SDA 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	/** I2C Initialization 
	*/
	LL_I2C_DisableOwnAddress2(I2C2);
	LL_I2C_DisableGeneralCall(I2C2);
	LL_I2C_EnableClockStretching(I2C2);
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_InitStruct.ClockSpeed = 100000;
	I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C2, &I2C_InitStruct);
	LL_I2C_SetOwnAddress2(I2C2, 0);
	
	LL_I2C_Enable(I2C2);
	CL_printMsg("I2C Setup Complete\n");
	
	
}//----------------------------------------------------------------------------------------