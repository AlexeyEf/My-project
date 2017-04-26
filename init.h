
#include <stm32l1xx.h>
#include <system_stm32l1xx.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_adc.h>
#include <stm32l1xx_dma.h>
#include <stm32l1xx_tim.h>
#include <stm32l1xx_spi.h>


static uint16_t value_adc[10]; //array for adc values

//function of configuration of ports, adc, dma and timer modules
void rcc_init(void);
void gpio_init(void);
void adc_init(void);
void tim1_init(void);
void tim2_init(void);
void dma1_init(void);
void dma2_init(void);
void spi_init(void);
void delay (void);

