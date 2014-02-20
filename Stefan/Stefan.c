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

#define SILNIK_L OCR1B
#define SILNIK_P OCR1A
#define PWM_MAX 150
#define PWM_PROSTO 40
#define TRYB_NIC 0
#define TRYB_ODLICZANIE 1
#define TRYB_JAZDA 2
#define TRYB_STOP 3
#define TRYB_KALIBRACJA 4

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

volatile u08 wartosci[12], stany[12], strona = 'L', tryb = TRYB_NIC, granica;
volatile signed int wagi[12] = {-155, -37, -31, -23, -15, -6, 6, 15, 23, 31, 39, 155};
volatile signed int starySygnal = 0;

void ustaw_porty()
{
	DDRA = 0x00;
	PORTA = 0x00;
	DDRB |= ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB6)|(1<<PB7));
	DDRB &= ~(1<<PB5);	//PB5 - wej�cie START z modu�u
	PORTB |= ((1<<PB2)|(1<<PB3)|(1<<PB4));	//LEDy zgaszone
	PORTB &= ~((1<<PB0)|(1<<PB1));	//multipleks wy��czony
	DDRC |= ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC7));	//kierunki dla silnik�w
	DDRC &= ~((1<<PC4)|(1<<PC5)|(1<<PC6));	//przyciski
	PORTC |= ((1<<PC4)|(1<<PC5)|(1<<PC6));	//pullupy dla przycisk�w
	DDRD |= ((1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7));
	DDRD &= ~(1<<PD2);	//wej�cie STOP z modu�u
}

void ustaw_adc()
{
	ADMUX |= ((1<<REFS0)|(1<<ADLAR)|7);	//VCC jako napi�cie odniesienia, wyr�wnanie wyniku do lewej, kana� 7 (bateria)
	ADCSRA |= ((1<<ADEN)|(1<<ADPS1));	//w��czenie ADC, dzielnik cz�stotliwo�ci 4
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
	MCUCR |= ((1<<ISC00)|(1<<ISC01));	//zbocze narastaj�ce na PD2
	GICR |= (1<<INT0);	//w��czenie przerwania
	//sei();	//w��czenie obs�ugi przerwa�
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
	
	/* cz�stotliwo�� PWM:
	CS12 CS11 CS10 cz�stotliwo��
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
	//cz�stotliwo�� 7372800/1024/72 = 100 Hz
	//odlicza 10ms i wywo�uje przerwanie
	//dla 5ms trzeba ustawi� OCR0 = 36
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
	
	//wy��czenie JTAG
	MCUCSR |= (1<<JTD);
	MCUCSR |= (1<<JTD);
	
	ustaw_porty();
	ustaw_adc();
	ustaw_usart();
	ustaw_pwm();
	ustaw_przerwania();
	ustaw_timer();
	
	_delay_ms(100);
	while(tryb == TRYB_NIC)	//obs�uga przycisk�w
	{
		if(!(PINC & (1<<PC5))) tryb = TRYB_KALIBRACJA;
		else if(!(PINC & (1<<PC6))) tryb = TRYB_ODLICZANIE;
	}
	while(PINC & (1<<PC4));	//czekaj na lewy przycisk
	
	if(tryb == TRYB_KALIBRACJA)
	{
		PORTB &= ~(1<<PB2);	//zapalona lewa dioda
		u08 czujniki = 'L';
		while((PINC & (1<<PC5)) && !(PINB & (1<<PB5)))	//czekaj na �rodkowy przycisk lub sygna� z pilota
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
		PORTB |= (1<<PB2);	//zga� diod� od kalibracji
		for(u08 i = 0; i<4; i++)	//mruganie diod� �rodkow�
		{
			PORTB ^= (1<<PB3);
			_delay_ms(500);
		}
		SILNIK_L = 0;
		SILNIK_P = 0;
		
		PORTC &= ~((1<<PC1)|(1<<PC2));
		PORTC |= ((1<<PC0)|(1<<PC3));	//do przodu
		
		tryb = TRYB_JAZDA;
	}	
	
	if(tryb == TRYB_STOP)
	{
		for(;;)	//mrugaj wszystkimi diodami
		{
			PORTB ^= ((1<<PB2)|(1<<PB3)|(1<<PB4));
			_delay_ms(500);
		}
	}
	
	return 0;
}

ISR(INT0_vect)	//obs�uga sygna�u STOP
{
	tryb = TRYB_STOP;
}

ISR(TIMER0_COMP_vect)	//dzia�anie co 10ms
{
	for(u08 i=0; i<6; i++)	//pobranie stanu czujnik�w (0-255)
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
	
	if(strona == 'L' && tryb == TRYB_JAZDA)	//wczytane obie strony; dzia�ania na sygnale
	{
		for(u08 i = 0; i<12; i++)	//progowanie
		{
			if(wartosci[i]>granica) stany[i] = 1; else stany[i]=0;	//1 - linia czarna
		}
		
		//sygnal - aktualne po�o�enie linii
		//suma - liczba czujnik�w, kt�re wykry�y lini�
		signed int sygnal = 0, suma = 0;
		for(u08 i = 0; i<12; i++)
		{
			sygnal += wagi[i]*stany[i];
			suma += stany[i];
		}
		sygnal /= suma;
		
		//sygna� bliski 0, czyli linia na �rodku albo *zgubiona*
		if((sygnal < 6) && (sygnal > -6))
		{
			sygnal = starySygnal;
		}
		
		starySygnal = sygnal;
		
		//wstawienie warto�ci na silniki
		signed int naSilnikP = PWM_PROSTO - sygnal*0.6;	//prawy
		signed int naSilnikL = PWM_PROSTO + sygnal*0.6;	//lewy
		if(naSilnikP < 0) naSilnikP = 0;
			else if(naSilnikP > PWM_MAX) naSilnikP = PWM_MAX;
		if(naSilnikL < 0) naSilnikL = 0;
			else if(naSilnikL > PWM_MAX) naSilnikL = PWM_MAX;
			
		SILNIK_P = naSilnikP;
		SILNIK_L = naSilnikL;
	}
}
