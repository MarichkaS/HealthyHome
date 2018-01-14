#include <stm32f0xx.h>
#include <stm32f0xx_pwr.h>
#include <stm32f0xx_rtc.h>
#include <stm32f0xx_exti.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_syscfg.h>
#include <math.h>
#include <stdio.h>

EXTI_InitTypeDef E;
GPIO_InitTypeDef G;
NVIC_InitTypeDef N;
TIM_TimeBaseInitTypeDef T;

uint32_t prescaler_ms;

const char helloString[] = "\nHello!\nStarting data transmission!\n";
volatile int analogSensorsValues[4] = {0};
float Temp, Humidity;

typedef enum{
	INPUT = 0,
	OUTPUT = 1
} GPIO_PinMode;

volatile uint8_t Data = 0;

uint16_t DatArray[41] = {0};
volatile uint8_t DCnt = 0;

// using systick as simple timer for delay
volatile uint32_t MSec = 0;
void SysTick_Handler(void){
	MSec++;
}


// DHT11 reading hanlder
void EXTI0_1_IRQHandler(void){
	static uint8_t DCnt = 0;

	//Check for falling edge
	if(EXTI_GetITStatus(EXTI_Line0) == SET){
		EXTI_ClearITPendingBit(EXTI_Line0);

		//Get current time of last falling edge to
		//current falling edge and store in array
		DatArray[DCnt] = TIM_GetCounter(TIM15);

		//Increment the data counter
		DCnt++;

		//If enough bytes have been received, disable
		//the counter
		if(DCnt == 41){
			Data = 1;
			DCnt = 0;
			TIM_Cmd(TIM15, DISABLE);
		}

		//Reset the counter for next data
		TIM_SetCounter(TIM15, 0);
	}
}

//Convert an array to an unsigned byte/word/dword
//Type = 0 for u8
//Type = 1 for u16
//Type = 2 for u32
uint32_t AToU(uint8_t *D, uint8_t Type){
	uint32_t V = 0;
	uint8_t Cnt = 0;

	for(Cnt = 0; Cnt<(8<<Type); Cnt++){
		V |= ((D[(8<<Type)-Cnt]&1)<<Cnt);
	}

	return V;
}

//Function to set the bus pin to either an input or open
//drain output
void AM_Pin(GPIO_PinMode GP){
	if(GP == INPUT){
		GPIO_StructInit(&G);
		G.GPIO_Pin = GPIO_Pin_0;
		G.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &G);
	}
	else{
		GPIO_StructInit(&G);
		G.GPIO_Pin = GPIO_Pin_0;
		G.GPIO_Mode = GPIO_Mode_OUT;
		G.GPIO_OType = GPIO_OType_OD;
		G.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &G);
	}
}

// USART BLOCK

void USART1_CONFIG(void)
{
	/**USART1 CONFIGURATION. Bluetooth **/
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER = (GPIOA->MODER &
	~(GPIO_MODER_MODER9|GPIO_MODER_MODER10))\
	| (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);

	GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH1 |
	GPIO_AFRH_AFRH2))| (1 << (1 * 4)) | (1 << (2 * 4));

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 0x341;
	USART1->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RE | USART_CR1_UESM;
	USART1->CR3 |= USART_CR3_WUFIE;
	while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART1->ICR |= USART_ICR_TCCF;/* clear TC flag */
	USART1->CR1 |= USART_CR1_RXNEIE; /* enable RC interrupt */
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);

}

void USART1_IRQHandler(void) {
	uint8_t chartoreceive= 0;
	if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)   {
		chartoreceive = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
		USART1_send(helloString);
	}
	else{
		//NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
	}
}

void USART1_send(char stringtosend[])
{
	uint16_t idx = 0;
	while( stringtosend[idx] != '\0')
	{
		USART1->TDR = stringtosend[idx++];
		while(! (USART1->ISR & USART_ISR_TXE) ){}
	}
    while(! (USART1->ISR & USART_ISR_TC) );
    USART1->ICR |= USART_ICR_TCCF;
}

// Analog sensors and ADC block

void ADC1_callibration(void){
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN by setting ADDIS*/
	/* (3) Clear DMAEN */
	/* (4) Launch the calibration by setting ADCAL */
	/* (5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
	ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
	/* For robust implementation, add here time-out management */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{
	/* For robust implementation, add here time-out management */
	}
}

void ADC1_enable_sequences(void) {
	/* (1) Ensure that ADRDY = 0 */
	/* (2) Clear ADRDY */
	/* (3) Enable the ADC */
	/* (4) Wait until ADC ready */
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
	{
	ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
	{
	/* For robust implementation, add here time-out management */
	}
}

void SetClockForADC1(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->CR2 |= RCC_CR2_HSI14ON;
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
}

void setSampleTimeADC1(void){
	ADC1->SMPR |= ADC_SMPR1_SMPR_0 | ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2;
	ADC->CCR |= ADC_CCR_VREFEN;
}

void setChannelsADC1(void){
	ADC1->CHSELR = ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL13
	    	| ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL14;
}

void updateAnalogSensorsValues(void) {
	ADC1->CR |= ADC_CR_ADSTART;
	for (int i=0; i < 4; i++) {
	    	while ((ADC1->ISR & ADC_ISR_EOC) == 0) {}
	    	analogSensorsValues[i] = ADC1->DR;
	}
}

// DHT11 block

void initDHT11(void) {
	//Enable clocks!
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	//Initialize the data pin as an input
	GPIO_StructInit(&G);
	G.GPIO_Pin = GPIO_Pin_0;
	G.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &G);

	//Configure the data pin as an EXTI source
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	//Disable but initialize EXTI line
	E.EXTI_Line = EXTI_Line0;
	E.EXTI_LineCmd = DISABLE;
	E.EXTI_Mode = EXTI_Mode_Interrupt;
	E.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&E);

	//Enable EXTI interrupts
	N.NVIC_IRQChannel = EXTI0_1_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 0;
	NVIC_Init(&N);

	T.TIM_ClockDivision = TIM_CKD_DIV1;
	T.TIM_CounterMode = TIM_CounterMode_Up;
	T.TIM_Period = 0xFFFF;
	T.TIM_Prescaler = SystemCoreClock/1000000-1;
	TIM_TimeBaseInit(TIM15, &T);
	TIM_Cmd(TIM15, DISABLE);
	TIM_SetCounter(TIM15, 0);

	MSec = 0;
	//DHT11 "Stability" delay! Needs to be >1s
	while(MSec<2500);
}

void updateHumidityTemperature(void) {
	//DConv will store the threshold'd bits
	uint8_t DConv[41] = {0}, Cnt;

	//Clear all current values in both converted
	//and measured period arrays.
	memset(DConv, 0, 41*sizeof(uint8_t));
	memset(DatArray, 0, 41*sizeof(uint16_t));

	//Variables to store read and final
	//humidity, temperature
	uint16_t HumR, TempR;

	Data = 0;
	E.EXTI_LineCmd = DISABLE;
	EXTI_Init(&E);
	AM_Pin(OUTPUT);
	TIM_SetCounter(TIM15, 0);
	TIM_Cmd(TIM15, ENABLE);

	GPIO_ResetBits(GPIOA, GPIO_Pin_0);

	//Pull low for 18ms minimum
	while(TIM_GetCounter(TIM15) < 18000);
	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	TIM_Cmd(TIM15, DISABLE);
	TIM_SetCounter(TIM15, 0);

	AM_Pin(INPUT);
	//Wait for pin to go low
	while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));

	//Wait for pin to go high
	while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));

	//Next falling edge will be start of transmission
	E.EXTI_LineCmd = ENABLE;
	EXTI_Init(&E);

	TIM_Cmd(TIM15, ENABLE);

	//Wait for data
	while(!Data);

	//Preclear the threshold bits array
	memset(DConv, 0, 41*sizeof(uint8_t));

	//Do the bit thresholding! Theoretically, a 1 should be
	//360 but for a bit of tolerance, I made it 340
	for(Cnt = 0; Cnt<41; Cnt++){
		if(DatArray[Cnt]<80) DConv[Cnt] = 0;
		else DConv[Cnt] = 1;
	}

	//Clear the period array for next time
	memset(DatArray, 0, 41*sizeof(uint16_t));

	//Convert the read humidity value from the threshold array
	//to an unsigned 16bit unsigned integer and convert to real
	//value, stored in a float.
	HumR = AToU(DConv, 1);
	Humidity = (float)HumR*0.1f;

	//Convert the read temperature from the threshold array to
	//a 16bit unsigned integer. Check to see if MSB is set, which
	//would indicate a negative temperature. If MSB is set, clear
	//MSB and assign a negative temperature to the real temperature
	//storing variable (also a float).

	TempR = AToU(&DConv[16], 1);
	if(TempR&(1<<15)){
		TempR &= (1<<16)-1;
		Temp = -(float)TempR*0.1f;
	}
	else{
		Temp = (float)TempR*0.1f;
	}

	//Wait for atleast 2 seconds for another conversion!
	MSec = 0;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	while(MSec<2000);
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void sendResults(void) {
	char buf[1000];

	sprintf(buf, "mq2 results: %i\n\n", analogSensorsValues[0]);
	USART1_send(buf);

	sprintf(buf, "mq4 results: %i\n\n", analogSensorsValues[1]);
	USART1_send(buf);

	sprintf(buf, "mq7 results: %i\n\n", analogSensorsValues[2]);
	USART1_send(buf);

	sprintf(buf, "lightSensor results: %i\n\n", analogSensorsValues[3]);
	USART1_send(buf);

	sprintf(buf, "dht11 humidity results: %f\n\n", Humidity);
	USART1_send(buf);

	sprintf(buf, "dht11 temperature results: %f\n\n", Temp);
	USART1_send(buf);
}

int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();
	//SystemCoreClockUpdate1();
	//Configure systick...
	SysTick_Config(SystemCoreClock/1000);

	SetClockForADC1();
	ADC1_callibration();
	ADC1_enable_sequences();
	setChannelsADC1();
	setSampleTimeADC1();

	USART1_CONFIG();
	USART1_send(helloString);

	initDHT11();

	DBGMCU->CR |= DBGMCU_CR_DBG_STANDBY | DBGMCU_CR_DBG_STOP; // debug in sleep mode

    while(1)
    {
    	updateAnalogSensorsValues();
    	updateHumidityTemperature();
    	sendResults();
    	PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
    }
}
