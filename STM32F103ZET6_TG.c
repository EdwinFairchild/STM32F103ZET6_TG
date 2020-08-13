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

	for (;;)
	{
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_3);	
		CL_printMsg("\nCL init success\n");	
		delayMS(100);
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
	//clocks	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN;

	
	//LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	//LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	
	
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Pin       = LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
	myGPIO.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
	myGPIO.Mode      = LL_GPIO_MODE_ALTERNATE;
	
	LL_GPIO_Init(GPIOA, &myGPIO);
	
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI);
	mySPI.Mode				= LL_SPI_MODE_MASTER;
	mySPI.TransferDirection	= LL_SPI_HALF_DUPLEX_TX;
	mySPI.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV8;
	mySPI.NSS				= LL_SPI_NSS_SOFT;
	mySPI.DataWidth			= LL_SPI_DATAWIDTH_8BIT;
	
	LL_SPI_Init(SPI1, &mySPI);
	//	LL_SPI_EnableDMAReq_TX(SPI1);
		LL_SPI_Enable(SPI1);
	
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
