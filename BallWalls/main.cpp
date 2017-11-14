/*
* BallWall.cpp
*
* Created: 19.09.2017 11:35:06
* Authors : ktaufer,mzudrell,bbr�stle
*/

#include <avr/io.h>
#include <avr/interrupt.h>

void execute_steps(void);

#define icrValue 23999

volatile bool dirUp;
volatile bool enabled;
unsigned char spiValues[4];

int main(void)
{
	/* set outputs */
	DDRB |= (1 << PB5);					// PWM
	DDRA |= (1 << PA1) ;				// INT0 f�r Raspberry
	DDRF = 0xFF;
	DDRC = 0xFF;
	DDRK = 0xFF;
	DDRH = 0xFF;

	/* activate SPI */
	SPCR |= (1<<SPE);
	

	/* initialize Timer */
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);		// FAST PWM Mode 14 einstellen
	TCCR1A |= (1 << COM1A1);					// PWM auf Ausgang OC1A aktivieren (PB5)
	
	ICR1 = (uint16_t)icrValue;
	OCR1A = 15999;
	OCR1B = 19999;
	TIMSK1 |= (1 << OCIE1B) | (1 << ICIE1);				//Interrupt
	
	/* activate global interrupts */
	sei();

	TCCR1B |= (1 << CS10);
	
	while (1)
	{
	}
}

/* gets values from SPI */
uint8_t SPI_SlaveReceive(void){
	/* Wait for reception complete */
	while (!(SPSR & (1<<SPIF)));
	/* Return data register */
	return SPDR;
}

/* Interrupt routine (reaching OCR value) */
ISR(TIMER1_COMPB_vect) {
	execute_steps();
}

/* Interrupt  routine (reaching ICR value)*/
ISR(TIMER1_CAPT_vect) {
	/* callback ready to get values to raspberry pi */
	PORTA |= (1 << PA1);
	for(uint8_t i = 0; i <= 3; i++) {
		spiValues[i] = SPI_SlaveReceive();
	}
	PORTA &= ~(1 << PA1);
}

/* writes values of SPI to ports */
void execute_steps(void){

	PORTF = spiValues[0];

	PORTK = spiValues[1];

	PORTH = spiValues[2];

	PORTC = spiValues[3];	
};