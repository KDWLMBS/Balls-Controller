/*
* BallWall.cpp
*
* Created: 19.09.2017 11:35:06
* Authors : Kevin Taufer
*/

#include <avr/io.h>
#include <avr/interrupt.h>

void execute_steps(void);

#define icrValue 64000

volatile bool dirUp;
volatile bool enabled;
unsigned char spiValues[8];

int main(void)
{
	/* set outputs */
	DDRB |= (1 << PB5);					// PWM
	DDRB |= (1 << PB3) ;				// INT0 für Raspberry
		
	DDRF = 0xFF;
	DDRC = 0xFF;
	DDRK = 0xFF;
	DDRH = 0xFF;
	DDRL = 0xFF;
	DDRA = 0xFF;
	DDRD = 0xFF;

	/* activate SPI */
	SPCR |= (1<<SPE);	

	/* initialize Timer */
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);		// FAST PWM Mode 14 einstellen
	TCCR1A |= (1 << COM1A1);					// PWM auf Ausgang OC1A aktivieren (PB5)
	
	ICR1 = (uint16_t)icrValue;
	OCR1A = icrValue/2;
	
	TIMSK1 |= (1 << OCIE1A) | (1 << ICIE1);				//Interrupt
	
	/* activate global interrupts */
	sei();
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
	TCCR1B |= (1 << CS10); // Prescaler 1
		
	while (1)
	{
	}
}

/* gets values from SPI */
uint8_t SPI_SlaveReceive(void){
	/* Wait for reception complete */
	while ((!(SPSR & (1<<SPIF))));
	/* Return data register */
	return SPDR;
}

/* Interrupt routine (reaching OCR value) */
ISR(TIMER1_COMPA_vect) {
	execute_steps();
}

/* Interrupt  routine (reaching ICR value)*/
ISR(TIMER1_CAPT_vect) {
	/* callback ready to get values to raspberry pi */
	PORTB |= (1 << PB3);
	for(uint8_t i = 0; i <= 7; i++) {
		spiValues[i] = SPI_SlaveReceive();
	}
	PORTB &= ~(1 << PB3);	
}

/* writes values of SPI to ports */
void execute_steps(void){

	PORTF = spiValues[0];

	PORTK = spiValues[1];

	PORTL = spiValues[2];
			
	PORTA = spiValues[3];
			
	PORTC = spiValues[4];
						
	// Set PORTD			
	PORTD = (spiValues[5] & 0b00001111);
	PORTD = PORTD | ((0b00010000 & spiValues[5]) << 3);
			
	// Set PORTG		
	PORTG = (spiValues[5] & 0b11100000) >> 5; 					
			
	// Set PORTH
	PORTH = (spiValues[6] & 0b00000011) | ((spiValues[6] & 0b00111100) << 1);
			
	// Set PORTJ
	PORTJ = ((spiValues[6] & 0b11000000) >> 6);
			
	// Set PORTB
	PORTB = (spiValues[7] << 4) | (PORTB & 00001111);
};