/*
 * Stefan.c
 *
 * Created: 2013-11-11 13:41:44
 *  Author: Piotrek
 */ 
#define F_CPU 7372800UL
//#define F_CPU 1000000UL
#define u08 unsigned char
#define s08 signed char
#define BAUDRATE 115200
#define BAUD_PRESCALE (((F_CPU/(BAUDRATE*16UL)))-1)

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

volatile u08 wartosci[12], strona = 'L';
volatile u08 stany[12];
volatile signed int wagi[12] = {-155, -37, -31, -23, -15, -6, 6, 15, 23, 31, 39, 155};

void ustaw_porty()
{
	DDRA = 0x00;
	PORTA = 0x00;
	DDRB |= ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB6)|(1<<PB7));
	DDRB &= ~(1<<PB5);	//PB5 - wejœcie START z modu³u
	PORTB |= ((1<<PB2)|(1<<PB3)|(1<<PB4));	//LEDy zgaszone
	PORTB &= ~((1<<PB0)|(1<<PB1));	//multipleks wy³¹czony
	DDRC |= ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC7));	//kierunki dla silników
	DDRC &= ~((1<<PC4)|(1<<PC5)|(1<<PC6));	//przyciski
	PORTC |= ((1<<PC4)|(1<<PC5)|(1<<PC6));	//pullupy dla przycisków
	DDRD |= ((1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7));
	DDRD &= ~(1<<PD2);	//wejœcie STOP z modu³u
}

void ustaw_adc()
{
	ADMUX |= ((1<<REFS0)|(1<<ADLAR)|7);	//VCC jako napiêcie odniesienia, wyrównanie wyniku do lewej, kana³ 7 (bateria)
	ADCSRA |= ((1<<ADEN)|(1<<ADPS1));	//w³¹czenie ADC, dzielnik czêstotliwoœci 4
}
void ustaw_usart()
{
	UCSRB |= (1<<RXEN)|(1<<TXEN);
	UCSRC |= (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1); //8bit
	UBRRL = BAUD_PRESCALE;
	UBRRH = (BAUD_PRESCALE>>8);
}
void ustaw_przerwania()
{
	MCUCR |= ((1<<ISC00)|(1<<ISC01));	//zbocze narastaj¹ce na PD2
	GICR |= (1<<INT0);	//w³¹czenie przerwania
	//sei();	//w³¹czenie obs³ugi przerwañ
}

void zmien_czujniki(u08 naKtore)
{
	if(naKtore == 'L') naKtore = PB1; else naKtore = PB0;
	PORTB &= ~((1<<PB0)|(1<<PB1));
	PORTB |= (1<<naKtore);
}

void ustaw_pwm()
{
	TCCR1A |= ((1<<WGM10)|(1<<COM1A1)|(1<<COM1B1));	//non-inverting, 8-bit fast PWM
	TCCR1B |= ((1<<WGM12)|(1<<CS10));	//28 kHz
	OCR1A = 0;
	OCR1B = 0;
	
	/* czêstotliwoœæ PWM:
	CS12 CS11 CS10 czêstotliwoœæ
   * 0    0    1     28 kHz 
	 0    1    0     3,6 kHz
	 0    1    1     450 Hz
	 1    0    0     112 Hz
	 1    0    1     28 Hz	
	*/
}

void ustaw_timer()
{
	TCCR0 |= (1<<WGM01);	//tryb CTC
	TCCR0 |= ((1<<CS02)|(1<<CS00));	//preskaler 1024
	OCR0 = 72;
	TIMSK |= (1<<OCIE0);	//przerwanie przy doliczeniu
	//czêstotliwoœæ 7372800/1024/72 = 100 Hz
	//odlicza 10ms i wywo³uje przerwanie
	//dla 5ms trzeba ustawiæ OCR0 = 36
}

void wyslij_usart(u08 znak)
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = znak;
}

u08 pomiar(u08 kanal)
{
	ADMUX &= ~((1<<MUX4)|(1<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0));
	ADMUX |= kanal;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADCH;
}

int main(void)
{
	u08 czyADC = 0, czyPWM = 0;
	volatile u08 min = 255, max = 0, granica;
	
	//wy³¹czenie JTAG
	MCUCSR |= (1<<JTD);
	MCUCSR |= (1<<JTD);
	
	ustaw_porty();
	ustaw_adc();
	ustaw_usart();
	ustaw_pwm();
	ustaw_przerwania();
	
	_delay_ms(100);
	while(!(czyADC || czyPWM))
	{
		if(!(PINC & (1<<PC5))) czyADC = 1;
		else if(!(PINC & (1<<PC6))) czyPWM = 1;
	}
	while(PINC & (1<<PC4));	//czekaj na lewy przycisk
	
	if(czyADC)
	{
		/*u08 bateria = pomiar(7);
		char bufor[4];
		itoa(bateria, bufor, 10);
		int j = 0;
		while(bufor[j] != 0)
		{
			wyslij_usart(bufor[j]);
			j++;
		}
		
		//czekanie na sygna³ start, potem wchodzimy do pêtli
	    while(1)
	    {
			zmien_czujniki('L');
			_delay_ms(10);
			for(u08 i = 0; i<6; i++)
			{
				wartosci[5-i] = pomiar(i);
			}
			zmien_czujniki('P');
			_delay_ms(10);
			for(u08 i = 0; i<6; i++)
			{
				wartosci[i+6] = pomiar(i);
			}
			
			wyslij_usart('[');
			for(u08 i = 0; i<12; i++)
			{
				//char bufor[4];
				itoa(wartosci[i], bufor, 10);
				int j = 0;
				while(bufor[j] != 0)
				{
					wyslij_usart(bufor[j]);
					j++;
				}
				if(i<11) wyslij_usart(';');
			}			
			wyslij_usart(']');
			_delay_ms(100);
			
	    }*/
		u08 czujniki = 'L';
		while((PINC & (1<<PC5)) && !(PINB & (1<<PB5)))
		{
			if(czujniki == 'P')
			{
				zmien_czujniki('L');
				czujniki = 'L';
			}
			else
			{
				zmien_czujniki('P');
				czujniki = 'P';
			}
			_delay_ms(10);
			for(u08 i = 0; i<6; i++)
			{
				u08 wartosc;
				wartosc = pomiar(i);
				if(wartosc > max) max = wartosc;
				else if(wartosc < min) min = wartosc;
			}
			zmien_czujniki('P');
			_delay_ms(10);
			for(u08 i = 0; i<6; i++)
			{
				u08 wartosc;
				wartosc = pomiar(i);
				if(wartosc > max) max = wartosc;
				else if(wartosc < min) min = wartosc;
			}
			
		}
		granica = (min+max)/2;
		czyPWM = 1;
		
	}
	if(czyPWM)
	{
		for(u08 i = 0; i<4; i++)
		{
			PORTB ^= (1<<PB3);
			_delay_ms(500);
		}
		//u08 licznik = 0;
		//u08 granica = 160;
		OCR1A = 0;
		OCR1B = 0;
		//u08 kier = 'P';
		
		PORTC &= ~((1<<PC1)|(1<<PC2));
		PORTC |= ((1<<PC0)|(1<<PC3));	//do przodu
		signed int starySygnal = 0;
		
		zmien_czujniki('L');
		_delay_ms(10);
		
		while(!(PIND & (1<<PD2)))
		{
			//START - pomiar na czujnikach (wartoœci 0-255)
			//_delay_ms(10);
			for(u08 i = 0; i<6; i++)
			{
				wartosci[5-i] = pomiar(i);
			}
			zmien_czujniki('P');
			_delay_ms(10);
			for(u08 i = 0; i<6; i++)
			{
				wartosci[i+6] = pomiar(i);
			}
			zmien_czujniki('L');
			//KONIEC - pomiar na czujnikach
			//START - stan czujników po progowaniu
			for(u08 i = 0; i<12; i++)
			{
				if(wartosci[i]>granica) stany[i] = 1; else stany[i]=0;	//1 - linia czarna
			}
			//KONIEC - progowanie
			//START - liczenie wartoœci steruj¹cej w zakresie (-6; 6)
			signed int sygnal = 0, suma = 0;
			for(u08 i = 0; i<12; i++)
			{
				sygnal += wagi[i]*stany[i];
				suma += stany[i];
			}
			sygnal /= suma;
			if((sygnal < 6) && (sygnal > -6))
			{
				/*if(starySygnal > 15)
				{
					sygnal = 70;
				}					
				else if(starySygnal < -15)
				{
					sygnal = -70;
				}	*/
				sygnal = starySygnal;				
			}
			//starySygnal = (sygnal+starySygnal)/2;
			starySygnal = sygnal;
			//KONIEC - wartoœæ steruj¹ca
			//START - ustawianie PWM
			
			signed int silnik1 = 40 - sygnal*0.6;	//prawy
			signed int silnik2 = 40 + sygnal*0.6;	//lewy
			if(silnik1 < 0) silnik1 = 0;
				else if(silnik1 > 150) silnik1 = 150;
			if(silnik2 < 0) silnik2 = 0;
				else if(silnik2 > 150) silnik2 = 150;
			OCR1A = silnik1;
			OCR1B = silnik2;
			//KONIEC - ustawianie PWM
			_delay_ms(10);
			//licznik++;
		}
		OCR1A = 0;
		OCR1B = 0;
		for(;;)
		{
			PORTB ^= ((1<<PB2)|(1<<PB3)|(1<<PB4));
			_delay_ms(500);
		}
	}
	
	return 0;
}

ISR(INT0_vect)
{
	//obs³uga sygna³u STOP

}

ISR(TIMER0_COMP_vect)	//dzia³anie co 10ms
{
	for(u08 i=0; i<6; i++)	//pobranie stanu czujników (0-255)
	{
		if(strona == 'L')
			wartosci[5-i] = pomiar(i);
		else
			wartosci[6+i] = pomiar(i);
	}
	
	if(strona=='L')	//zmiana strony
	{
		strona = 'P';
		zmien_czujniki('P');
	}
	else
	{
		strona = 'L';
		zmien_czujniki('L');
	}
	
	if(strona == 'L')	//jeœli s¹ wczytane obie strony; dzia³ania na sygnale
	{
		
	}
}
