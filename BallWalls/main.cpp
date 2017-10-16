/*
* BallWall.cpp
*
* Created: 19.09.2017 11:35:06
* Authors : ktaufer,mzudrell,bbrüstle
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include<stdint.h>

void GetValues(void);
void ExecuteSteps(void);

#define icrValue 15999

volatile uint16_t absStep;
volatile bool dirUp;
volatile bool enabled;
unsigned char spiValues[4];
volatile int spiCounter;

int main(void)
{
	/* Ausgänge definieren */

	DDRB |= (1 << PB5) | (1 << PB4);
	DDRA |= (1 << PA1) ;
	DDRF = 0xFF;
	DDRC = 0xFF;
	DDRK = 0xFF;
	DDRH = 0xFF;

	/* Werte an Ports geben */

	//set PORT values to 1111111
	PORTF = 0xFF;	
	PORTK = 0xFF;	
	PORTH = 0xFF;	
	PORTC = 0xFF;

	/* SPI aktivieren */
	SPCR = (1<<SPE);
	
	/* (startup config) */
	absStep = 250;

	/* Timer initialisieren */
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);		// FAST PWM Mode 14 einstellen
	TCCR1A |= (1 << COM1A1);					// PWM auf Ausgang OC1A aktivieren (PB5)
	
	ICR1 = (uint16_t)icrValue;
	OCR1A = (uint16_t)(icrValue/2);
	TIMSK1 |= (1 << OCIE0A) | (1 << ICIE1);				//Interrupt bei Erreichen von OCR1A aktivieren
	
	sei();

	TCCR1B |= (1 << CS10);
	
	while (1)
	{
	}
}

/* Interrupt bei erreichen des ICR Wertes */
ISR(TIMER1_COMPA_vect) {
	ExecuteSteps();
}

ISR(TIMER1_CAPT_vect) {
/* Port für callback an Raspberry Pi */
PORTA |= (1 << PA1);
	GetValues();
}

void GetValues(void)
{
	/* Callback finish an Raspberry Pi */
	PORTB &= ~(1 << PA1);
}

ISR(SPI_STC_vect){
	
	if(spiCounter > 3) {
		spiCounter = 0;
	}
	spiCounter ++;
	spiValues[spiCounter] = SPDR;
}


void ExecuteSteps(void){

	//set PORTF
	PORTF = spiValues[0];

	//set PORTK
	PORTK = spiValues[1];

	//set PORTH
	PORTH = spiValues[2];

	//set PORTC
	PORTC = spiValues[3];

};