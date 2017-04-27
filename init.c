
#include <init.h>
#include <stm32l1xx_dma.h>

static uint16_t value_adc[10]; //array for adc values

void rcc_init(void)
  {
		RCC->CFGR |= RCC_CFGR_SW_HSI; //use HSI oscillator for system clock
		RCC->CR |= RCC_CR_HSION;  //turn on oscillator HSI for clocking ADC1
    while(!(RCC->CR&RCC_CR_HSIRDY)); //waiting for stabilization of HSI
	}
	
void gpio_init(void)
  {

  GPIO_InitTypeDef port;  //definition of variable of GPIO_InitTypeDef type to access to elements of structure GPIO_InitTypeDef to configure port GPIOC
    
  //clock configuration
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); //enable clock of port C
    
		//Port configuration
		//configuration of port A5 & A12 for SPI outputs SCK & MOSI
  port.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_12;  //change pins
  port.GPIO_Mode=GPIO_Mode_AF;  //this ports use for alternative function (SPI)
  port.GPIO_Speed=GPIO_Speed_40MHz; //set max rate via this pins
  port.GPIO_OType=GPIO_OType_PP;
  //call function to configure GPIOA Pin 4, GPIO_Pin_5 and GPIO_Pin_12
  GPIO_Init(GPIOA,&port);
	
		 //configuration of port B6
  port.GPIO_Pin=GPIO_Pin_6;//change pins
  port.GPIO_Mode=GPIO_Mode_OUT;//this ports use as outputs
  port.GPIO_Speed=GPIO_Speed_40MHz;//set max rate via this pins
  port.GPIO_OType=GPIO_OType_PP;
  //call function to configure GPIOB Pin 6 and Pin 7
  GPIO_Init(GPIOB,&port);
	
	//configuration of port C1 & C2 for master outputs SlaveSelect for transmitting to device 1 & device 2	
	port.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2; //change pins
  port.GPIO_Mode=GPIO_Mode_OUT; //this ports use as outputs
  port.GPIO_Speed=GPIO_Speed_40MHz; //set max rate via this pins
  port.GPIO_OType=GPIO_OType_PP;
  //call function to configure GPIOC Pin 1 & GPIOC Pin 2
  GPIO_Init(GPIOC,&port);
	
  
  //configuration of elements of structure GPIO_InitTypeDef for port C
  port.GPIO_Pin=GPIO_Pin_0;//change pins
  port.GPIO_Mode=GPIO_Mode_AN;//this ports use in analog mode
  port.GPIO_Speed=GPIO_Speed_40MHz;//set max rate via this pins
  port.GPIO_OType=GPIO_OType_PP;
  //call function to configure GPIOC Pin 0
  GPIO_Init(GPIOC,&port);
  
  }


void adc_init(void)
  {
  
  ADC_InitTypeDef adc;  //definition of variable of ADC_InitTypeDef type to access to elements of structure ADC_InitTypeDef to configure adc ADC1
   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); //enable clock of ADC1
  
  //ADC configuration
  //configuration of elements of structure ADC_InitTypeDef
  adc.ADC_Resolution=ADC_Resolution_12b; //ADC Resolution = 12 bits
  adc.ADC_ScanConvMode=DISABLE; //use not scan mode - use only one channel
  adc.ADC_ContinuousConvMode=DISABLE; //use discontinuous mode
  adc.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_Rising; //start adc of rising edge from trigger of timer TIM2
  adc.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T2_TRGO;////start adc of trigger of timer TIM2
  adc.ADC_DataAlign=ADC_DataAlign_Right;//tADC data alignment is righ
  adc.ADC_NbrOfConversion=1; // one conversion
  
  //call function to configure ADC1
  ADC_Init(ADC1, &adc);
  //configuration of regular channels
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_16Cycles); //use channel 10, 1 conversion, sample time is 16 cycles 
  ADC_DiscModeCmd(ADC1, ENABLE);//enable discontinuous mode
  
  //configuration of interrupt
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);// enable interrupt of end of conversion
  NVIC_EnableIRQ(ADC1_IRQn);//enable interrupt from ADC in NVIC controller
  //turn on ADC
  ADC_DMACmd(ADC1, ENABLE);// enable adc request to dma
  ADC_Cmd(ADC1, ENABLE);//start adc
  
  
  }

	
void tim1_init(void)
  {

  TIM_TimeBaseInitTypeDef timer; //definition of variable of TimeBaseInitTypeDef type to access to elements of structure TimeBaseInitTypeDef to configure timer TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //enable clock of basic timer TIM2
      
  //Timer configuration
  //configuration of elements of structure TimeBaseInitTypeDef
  timer.TIM_Prescaler=15999; //prescaller of system clock HSI 14 MHz = 15999
  timer.TIM_Period=1000; // period=1 sec
  //call function to configure timer TIM6
  TIM_TimeBaseInit(TIM2,&timer);
      
  TIM2->CR2 |= TIM_CR2_MMS_1; //enable clock for adc from this timer through TRG0
  TIM_Cmd(TIM2,ENABLE);  //start counting
  
  }
	
	void tim2_init(void)
  {

  TIM_TimeBaseInitTypeDef timer; //definition of variable of TimeBaseInitTypeDef type to access to elements of structure TimeBaseInitTypeDef to configure timer TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE); //enable clock of basic timer TIM2
      
  //Timer configuration
  //configuration of elements of structure TimeBaseInitTypeDef
  timer.TIM_Prescaler=15999; //prescaller of system clock 16 MHz = 16000
  timer.TIM_Period=100; // period=100 ms
  //call function to configure timer TIM6
  TIM_TimeBaseInit(TIM6,&timer);
      
  TIM6->DIER |= TIM_DIER_UIE;  //enable interrupt
  NVIC_EnableIRQ(TIM6_IRQn);    //enable interrupt in NVIC controller
  }
  
	void tim3_init(void)
  {
	
  TIM_TimeBaseInitTypeDef timer; //definition of variable of TimeBaseInitTypeDef type to access to elements of structure TimeBaseInitTypeDef to configure timer TIM6
  
	//clock configuration
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE); //enable clock of basic timer TIM6  
  
  
  //Timer configuration
  //configuration of elements of structure TimeBaseInitTypeDef
  timer.TIM_Prescaler=15999; //prescaller of system clock 16 MHz = 16000
  timer.TIM_Period=100;  //frequency=100 Hz
  //timer.TIM_Period=300;  //frequency=300 Hz
  
  /*if frequency=500 kHz,then
  timer.TIM_Prescaler=224;
  timer.TIM_Period=8;  */
  
  //call function to configure timer TIM6
  TIM_TimeBaseInit(TIM7,&timer);
  
  TIM7->DIER |= TIM_DIER_UIE;  //enable interrupt
  TIM_Cmd(TIM7,ENABLE);  //start counting
  NVIC_EnableIRQ(TIM7_IRQn);    //enable interrupt in NVIC controller
  }
	
void dma1_init(void)
  {
    DMA_InitTypeDef dma; //definition of variable of DMA_InitTypeDef type to access to elements of structure GPIO_InitTypeDef to configure DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//enable clock of DMA
    
    dma.DMA_PeripheralBaseAddr=(uint32_t) (&(ADC1->DR)); //Specifies the peripheral base address of data spi register ADC1->DR for DMA1 Channel1

    dma.DMA_MemoryBaseAddr=(uint32_t) (&(value_adc));      //Specifies the memory base address of first element of array value_adc[] for DMA1 Channel1

    dma.DMA_DIR=DMA_DIR_PeripheralSRC;     //Specifies if the peripheral is destination

    dma.DMA_BufferSize=10;         //Specifies the buffer size=10 of 16 bits words

    dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;      //Specifies the Peripheral address register is not incremented

    dma.DMA_MemoryInc=DMA_MemoryInc_Enable;           //Specifies the memory address register is incremented 

    dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;           // Specifies the ADC data width - 16 bit

    dma.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;     // Specifies the Memory data width - 16 bit

    dma.DMA_Mode=DMA_Mode_Normal;               // Specifies the operation mode of the DMA1 Channel3 is not Circular mode

    dma.DMA_Priority=DMA_Priority_High;          // Specifies the software priority for the DMA1 Channel1 - high priority

    dma.DMA_M2M=DMA_M2M_Disable;         //Specifies if the DMA1 Channel3 will be used in memory-to-memory transfer - disable
      
  DMA_Init( DMA1_Channel1, &dma);
  
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);//enable interrupt if full data was transmitted
  
	DMA_Cmd(DMA1_Channel1, ENABLE);//start dma
  
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);//enable interrupt from DMA in NVIC controller
  }
	
	void spi_init(void)
  {
    SPI_InitTypeDef spi; //definition of variable of SPI_InitTypeDef type to access to elements of structure GPIO_InitTypeDef to configure SPI

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //enable clock of SPI
    
    
		spi.SPI_Direction = SPI_Direction_1Line_Tx;  // specifies SPI direction
		spi.SPI_DataSize = SPI_DataSize_16b;  //specifies size of transmitting data - 16 bits
		spi.SPI_CPOL = SPI_CPOL_Low;  //specifies the serial clock steady state - low state
		spi.SPI_CPHA = SPI_CPHA_1Edge; // specifies the clock active edge for the bit capture - active 1 edge
		spi.SPI_NSS = SPI_NSS_Hard; //specifies whether the NSS signal is managed by hardware or by software - by hardware                                       
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//spi clock frequency=16 MHz/8=2 MHz
		spi.SPI_FirstBit = SPI_FirstBit_MSB; //first bit-MSB
 
		spi.SPI_Mode = SPI_Mode_Master;// change master mode
		SPI_Init(SPI1, &spi);
		
		SPI_SSOutputCmd(SPI1, ENABLE); // configure output NSS as output  
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE); // dma request from spi after end of transmitting 16 bits
    
  }
	
	void dma2_init(void)
  {
    DMA_InitTypeDef dma; //definition of variable of DMA_InitTypeDef type to access to elements of structure GPIO_InitTypeDef to configure DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); //enable clock of DMA
    
    dma.DMA_PeripheralBaseAddr=(uint32_t) (&(SPI1->DR)); //Specifies the peripheral base address of data spi register SPI1->DR for DMA1 Channel3

    dma.DMA_MemoryBaseAddr=(uint32_t) (&(value_adc));     //Specifies the memory base address of first element of array value_adc[] for DMA1 Channel3

    dma.DMA_DIR=DMA_DIR_PeripheralDST;      //Specifies if the peripheral is destination
                                       

    dma.DMA_BufferSize=10;         //Specifies the buffer size=10 of 16 bits words
                                        

    dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;      //Specifies whether the Peripheral address register is incremented or not
                                        

    dma.DMA_MemoryInc=DMA_MemoryInc_Enable;          // Specifies whether the memory address register is incremented or not
                                        

    dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;           // Specifies the Peripheral data width - 16 bit
                                        

    dma.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;     // Specifies the Memory data width - 16 bit
                                        

    dma.DMA_Mode=DMA_Mode_Circular;               // Specifies the operation mode of the DMA1 Channel3 - Circular mode
                                        

    dma.DMA_Priority=DMA_Priority_High;           // Specifies the software priority for the DMA1 Channel3 - high priority
                                        

    dma.DMA_M2M=DMA_M2M_Disable;         //Specifies if the DMA1 Channel3 will be used in memory-to-memory transfer - disable
                                        
      
    DMA_Init( DMA1_Channel3, &dma);
  
    DMA_ITConfig(DMA1_Channel3, DMA_IT_HT|DMA_IT_TC, ENABLE);//enable interrupt if half and full data was transmitted
		
    DMA_Cmd(DMA1_Channel3, ENABLE); //start dma
		
		SPI_Cmd(SPI1, ENABLE);// start SPI
		
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);//enable interrupt from DMA in NVIC controller
  }

void ADC1_IRQHandler(void)
  {    
  
      ADC_ClearITPendingBit(ADC1, ADC_IT_EOC); //Clear IT Pending Bit
  
  }

void DMA1_Channel1_IRQHandler(void)
  {    
    DMA_Cmd(DMA1_Channel1, DISABLE);//stop dma
		DMA_ClearITPendingBit(DMA1_IT_TC1);//Clear IT Pending Bit
		ADC_Cmd(ADC1, DISABLE);//stop adc
		TIM_Cmd(TIM2,DISABLE);  //stop timer
		tim3_init();
		dma2_init();
		tim2_init();
    
  }
	void TIM6_IRQHandler(void)
 {
   TIM_Cmd(TIM6, DISABLE); //stop timer
   TIM_ClearITPendingBit(TIM6, TIM_EventSource_Update);//Clear ITPending Bit
 }
 
 void TIM7_IRQHandler(void)
  {
    TIM_ClearFlag(TIM7,TIM_IT_Update);   //clear interrupt flag
    GPIOB->ODR^=GPIO_Pin_6;   //invert state of pin
  }
	

void DMA1_Channel3_IRQHandler(void) 
  {
		  
      if(DMA_GetITStatus(DMA1_IT_HT3)==SET)  //end of transmitting of first half data to device 1
			  
			{				
				// waiting for end of transmitting
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == SET);
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == RESET);
			
				  //set portC1
				  //end of transmitting to device 1
          GPIO_SetBits(GPIOC, GPIO_Pin_1);
				
					//delay of 100 ms
				  delay();
				
					//start transmitting to device 2
          GPIO_ResetBits(GPIOC, GPIO_Pin_2);   //reset portC2, which use to control transmitting to device 2, to transmit second half data to device 2 
			}
			
			if(DMA_GetITStatus(DMA_IT_TC)==SET)   //end of transmitting of second half data to device 1
				
			{
				// waiting for end of transmitting
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == SET);
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == RESET);
				
				//set portC2 
				//end of transmitting to device 2
				GPIO_SetBits(GPIOC, GPIO_Pin_2);
				
				//delay 100 ms
				  delay();
							
				//start of transmitting to device 1
          GPIO_ResetBits(GPIOC, GPIO_Pin_1);
				
			}
			
				DMA_ClearITPendingBit(DMA_IT_HT|DMA_IT_TC);  //reset ITPendingBits
   
	}
	
	void delay (void)
 {
   TIM_Cmd(TIM6, ENABLE);//start counting of timer TIM6
   while(TIM_GetITStatus(TIM6, TIM_FLAG_Update) == SET);//waiting for interrupt
 }
