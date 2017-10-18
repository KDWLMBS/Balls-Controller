/*
* BallWall.cpp
*
* Created: 19.09.2017 11:35:06
* Authors : ktaufer,mzudrell,bbrüstle
*/

#include <avr/io.h>
#include <avr/interrupt.h>

void execute_steps(void);

#define icrValue 15999

volatile uint16_t absStep;
volatile bool dirUp;
volatile bool enabled;
unsigned char spiValues[4];

int main(void)
{
	/* set outputs */
	DDRB |= (1 << PB5);					// PWM
	DDRA |= (1 << PA1) ;				// INT0 für Raspberry
	DDRF = 0xFF;
	DDRC = 0xFF;
	DDRK = 0xFF;
	DDRH = 0xFF;

	/* activate SPI */
	SPCR |= (1<<SPE);

	/* (startup config) */
	absStep = 250;

	/* initialize Timer */
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);		// FAST PWM Mode 14 einstellen
	TCCR1A |= (1 << COM1A1);					// PWM auf Ausgang OC1A aktivieren (PB5)
	
	ICR1 = (uint16_t)icrValue;
	OCR1A = (uint16_t)(icrValue/2);
	TIMSK1 |= (1 << OCIE0A) | (1 << ICIE1);				//Interrupt bei Erreichen von OCR1A aktivieren
	
	/* activate global interrupts */
	sei();

	TCCR1B |= (1 << CS10);
	
	while (1)
	{
	}
}

uint8_t SPI_SlaveReceive(void){
	/* Wait for reception complete */
	while (!(SPSR & (1<<SPIF)));
	/* Return data register */
	return  SPDR;
}

/* Interrupt bei erreichen des ICR Wertes */
ISR(TIMER1_COMPA_vect) {
	execute_steps();
}

ISR(TIMER1_CAPT_vect) {
	/* callback ready to get values to raspberry pi */
	PORTA |= (1 << PA1);
	for(uint8_t i = 0; i <= 3; i++) {
		spiValues[i] = SPI_SlaveReceive();
	}
	PORTA &= ~(1 << PA1);
}

void execute_steps(void){
	//set PORTF
	PORTF = spiValues[0];

	//set PORTK
	PORTK = spiValues[1];

	//set PORTH
	PORTH = spiValues[2];

	//set PORTC
	PORTC = spiValues[3];	
};