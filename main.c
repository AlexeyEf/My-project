
#include "init.h"
  
int main()
{ 
  //call function of configuration of ports, adc, dma and timer modules	
  gpio_init();
  dma1_init();
  adc_init();
  tim1_init(); 
  spi_init();
	
  
  
  while(1);
   
}
