/*
 * Stefan.c
 *
 * Created: 2013-11-11 13:41:44
 *  Author: Piotrek
 */ 
#define F_CPU 7372800UL
#define u08 unsigned char
#define s08 signed char
#define BAUDRATE 115200
#define BAUD_PRESCALE (((F_CPU/(BAUDRATE*16UL)))-1)

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define SILNIK_L OCR1B
#define SILNIK_P OCR1A

#define PWM_MAX 150
#define PWM_PROSTO 40
//#define KP 1

#define TRYB_NIC 0
#define TRYB_ODLICZANIE 1
#define TRYB_JAZDA 2
#define TRYB_STOP 3
#define TRYB_KALIBRACJA 4
#define TRYB_KOMP 5

volatile u08 wartosci[12], stany[12], strona = 'L', tryb = TRYB_NIC, granica, licznikKomp=0, wyslacInfo=0;
volatile signed int starySygnal = 0;
volatile signed int wagi[12] = {-155, -55, -31, -23, -15, -6, 6, 15, 23, 31, 55, 155}; // dalbym jako zminna lokalna w przerwaniu

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
	SILNIK_L = 0;
	SILNIK_P = 0;
	
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
void wylacz_timer()
{
	TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
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
	volatile u08 min = 255, max = 0;
	
	//wy³¹czenie JTAG
	MCUCSR |= (1<<JTD);
	MCUCSR |= (1<<JTD);
	
	ustaw_porty();
	ustaw_adc();
	ustaw_usart();
	ustaw_pwm();
	ustaw_przerwania();
	
	sei();	//w³¹czenie przerwañ
	
	_delay_ms(100);
	while(tryb == TRYB_NIC)	//obs³uga przycisków
	{
		if(!(PINC & (1<<PC5))) tryb = TRYB_KALIBRACJA;
		else if(!(PINC & (1<<PC6)))
		{
			tryb = TRYB_KOMP;
			UCSRB |= (1<<RXCIE);	//w³¹cz przerwanie przy odbiorze z USART
		}			
	}
	while(PINC & (1<<PC4));	//czekaj na lewy przycisk
	if(tryb == TRYB_KOMP) ustaw_timer();
	
	while(1)
	{
		if(tryb == TRYB_KALIBRACJA)
		{
			PORTB &= ~(1<<PB2);	//zapalona lewa dioda
			u08 czujniki = 'L';
			while((PINC & (1<<PC5)) && !(PINB & (1<<PB5)))	//czekaj na œrodkowy przycisk lub sygna³ z pilota
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
			tryb = TRYB_ODLICZANIE;
		}
		if(tryb == TRYB_ODLICZANIE)
		{
			PORTB |= (1<<PB2);	//zgaœ diodê od kalibracji
			for(u08 i = 0; i<4; i++)	//mruganie diod¹ œrodkow¹
			{
				PORTB ^= (1<<PB3);
				_delay_ms(500);
			}
			SILNIK_L = 0;
			SILNIK_P = 0;
		
			PORTC &= ~((1<<PC1)|(1<<PC2));
			PORTC |= ((1<<PC0)|(1<<PC3));	//do przodu
		
			ustaw_timer();
			tryb = TRYB_JAZDA;
		}	
	
		if(tryb == TRYB_STOP)
		{
			wylacz_timer();
			SILNIK_L = 0;
			SILNIK_P = 0;
			PORTB |= (1<<PB4);
			
			while(1)	//mrugaj wszystkimi diodami
			{
				PORTB ^= ((1<<PB2)|(1<<PB3)|(1<<PB4));
				_delay_ms(500);
			}
		}
		if(tryb == TRYB_KOMP)
		{
			PORTB &= ~(1<<PB4);	//zapalenie prawej diody
			if(!(PINC & (1<<PC6)))	//wciœniêcie prawego przycisku
			{
				tryb = TRYB_NIC;
				UCSRB &= ~(1<<RXCIE);	//wy³¹czenie odbioru w przerwaniu
				PORTB |= (1<<PB4);	//zgaszenie prawej diody
				wylacz_timer();
			}
		}
	}	
	
	return 0;
}

ISR(INT0_vect)	//obs³uga sygna³u STOP
{
	if(tryb == TRYB_JAZDA) tryb = TRYB_STOP;
}

ISR(USART_RXC_vect)	//odbiór znaku z USART
{
	char temp;
	temp = UDR;
	
	if(temp == '?') wyslacInfo = 1;
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
	
	if(strona == 'L')	//wczytane obie strony; dzia³ania na sygnale
	{
		if(tryb == TRYB_JAZDA)
		{
			for(u08 i = 0; i<12; i++)	//progowanie
			{
				if(wartosci[i]>granica) stany[i] = 1; else stany[i]=0;	//1 - linia czarna
			}
			
			/*******************************/
			wyslij_usart('[');
			for(u08 i=0; i<12; i++)	//dla ka¿dej wartoœci czujnika
			{
				int temp = wartosci[i];
				char bufor[4];
				itoa(temp, bufor, 10);
				u08 j=0;
				while(bufor[j] != 0)
				{
					wyslij_usart(bufor[j]);	//wyœlij j-ty znak i zwiêksz j
					j++;
				}
				if(i<11) wyslij_usart(',');
			}
			wyslij_usart(']');
			/********************************/
		
			//sygnal - aktualne po³o¿enie linii
			//suma - liczba czujników, które wykry³y liniê
			signed int sygnal = 0, suma = 0;
			for(u08 i = 0; i<12; i++)
			{
				sygnal += wagi[i]*stany[i];
				suma += stany[i];
			}
			sygnal /= suma;
		
			//sygnal *= kp;
		
			//sygna³ bliski 0, czyli linia na œrodku albo *zgubiona*
			if((sygnal < 6) && (sygnal > -6))
			{
				sygnal = starySygnal;
				PORTB &= ~(1<<PB4);	
			}
			else PORTB |= (1<<PB4);	//prawa dioda œwieci przy sygnale bliskim 0
		
			starySygnal = sygnal;
		
			//wstawienie wartoœci na silniki
			signed int naSilnikP = PWM_PROSTO - sygnal*0.6;	//prawy
			signed int naSilnikL = PWM_PROSTO + sygnal*0.6;	//lewy
			
			//sprawdzenie prezkroczenia zakresu sterowania
			if(naSilnikP < 0) naSilnikP = 0;
				else if(naSilnikP > PWM_MAX) naSilnikP = PWM_MAX;
			if(naSilnikL < 0) naSilnikL = 0;
				else if(naSilnikL > PWM_MAX) naSilnikL = PWM_MAX;
			
			//wstawienie obliczonych wartoœci na silniki
			SILNIK_P = naSilnikP;
			SILNIK_L = naSilnikL;
		}
		else if(tryb == TRYB_KOMP)
		{
			//if(licznikKomp == 10)	//co 100ms
			//{
				wyslij_usart('[');
				for(u08 i=0; i<12; i++)	//dla ka¿dej wartoœci czujnika
				{
					int temp = wartosci[i];
					char bufor[4];
					itoa(temp, bufor, 10);
					u08 j=0;
					while(bufor[j] != 0)
					{
						wyslij_usart(bufor[j]);	//wyœlij j-ty znak i zwiêksz j
						j++;
					}
					if(i<11) wyslij_usart(',');
				}
				wyslij_usart(']');
				
			//	licznikKomp = 0;
			//}
			//licznikKomp++;
			
			if(wyslacInfo)	//wys³anie paczki informacji
			{
				wyslacInfo = 0;
				
				wyslij_usart('{');
				char bufor[4];
				int bateria;
				bateria = pomiar(7);
				itoa(bateria, bufor, 10);
				
				u08 j=0;
				while(bufor[j] != 0)
				{
					wyslij_usart(bufor[j]);
					j++;
				}
				
				wyslij_usart('}');
			}
		}			
	}
}