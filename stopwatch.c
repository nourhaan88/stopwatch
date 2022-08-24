/*
 * mini_project_2.c
 *
 *  Created on: ??þ/??þ/????
 *      Author: Nourhan Ehab
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char sec_digit =0;
unsigned char sec_tens =0;
unsigned char min_digit =0;
unsigned char min_tens =0;
unsigned char hrs_digit =0;
unsigned char hrs_tens =0;

void timer1_ctc(){
	TCCR1A |= (1<<FOC1A) ;
	TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS12);
	TCNT1 = 0;
	OCR1A = 1000;
	TIMSK = (1<<OCIE1A);
}

ISR (TIMER1_COMPA_vect){
	sec_digit++;
	if(sec_digit ==10){
		sec_tens++;
		sec_digit=0;
		if(sec_tens==6){
			min_digit++;
			sec_tens=0;
		}

		if(min_digit==10){
			min_tens++;
			min_digit=0;
		}
		if(min_tens==6){
			hrs_digit++;
			min_tens=0;
		}
		if(hrs_digit==10){
			hrs_tens++;
			hrs_digit=0;

		if (hrs_tens==3 && hrs_digit==5){
			sec_digit =0;
				 sec_tens =0;
				 min_digit =0;
				 min_tens =0;
				 hrs_digit =0;
				 hrs_tens =0;
		}
	  }
	}

}


void INT0_init(void){
	MCUCR |= (1<<ISC01);
	MCUCR &= ~(1<<ISC00);
	SREG |= (1<<7);
	DDRD &= ~(1<<2);
	PORTB |= (1<<2);
	GICR |= (1<<INT0);

}


ISR (INT0_vect){
	 sec_digit =0;
	 sec_tens =0;
	 min_digit =0;
	 min_tens =0;
	 hrs_digit =0;
	 hrs_tens =0;
}

void INT1_init(void){
	MCUCR |= (1<<ISC10) | (1<<ISC11);
	SREG |= (1<<7);
	DDRD &= ~(1<<3);
	GICR |= (1<<INT1);

}

ISR (INT1_vect)
{
	TCCR1B &= ~(1<<CS10) & (1<<CS11) & (1<<CS12);

}

void INT2_init(void){
    	MCUCSR &= ~(1<<ISC2);
	    SREG |= (1<<7);
		DDRB &= ~(1<<2);
		PORTB |= (1<<2);
		GICR |= (1<<INT2);

}

ISR (INT2_vect){
	timer1_ctc();
	//TCCR1B |= (1<<CS10) | (1<<CS12);
}




int main(void){
	DDRC |= 0x0F;
	PORTC = 0;
	DDRA |= 0x3F;
	PORTA = 0;
	/////SREG |=(1<<7);
	timer1_ctc();
	INT0_init();
	INT1_init();
	INT2_init();

	while(1){
		PORTA = (PORTA & 0x00) | (1 << 0);
				PORTC = sec_digit;
				_delay_ms(3);

				PORTA = (PORTA & 0x00) | (1 << 1);
				PORTC = sec_tens;
				_delay_ms(3);

				PORTA = (PORTA & 0x00) | (1 << 2);
				PORTC = min_digit;
				_delay_ms(3);

				PORTA = (PORTA & 0x00) | (1 << 3);
				PORTC = min_tens;
				_delay_ms(3);

				PORTA = (PORTA & 0x00) | (1 << 4);
				PORTC = hrs_digit;
				_delay_ms(3);

				PORTA = (PORTA & 0x00) | (1 << 5);
				PORTC = hrs_tens;
				_delay_ms(3);

	}
}
