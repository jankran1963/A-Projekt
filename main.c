/*
 * main.c
 *
 *  Created on: 9 kwi 2024
 *      Author: Andrzej
 *
 *
 *      ATmega32 jako Master Fosc = 16 MHz
 *      Dzia�a z programem AT328_Slave_UART
 *
 *  1. ................. OK spr. ...........................................................................................
 *  ATmega 32, Master TxD po��czony do  RxD Atmega 328 Slave
 *  Atmega 328 Slave TxD po��czony do RxD ATmega 32, Master
 *  ATmega 32, Master wysy�a  /USART_Transmit('Z')/; do ATmega 328/UNO/, Slave, znak 'Z'.
 *  Atmega 328 Slave odbiera: /dana_Master = UDR0;/ i odsy�a /USART_Transmit(dana_Master);/ ten znak 'Z' do Atmega 32 Master
 *  ATmega 32, Master odbiera go /PORTA = UDR;/ zapisuje go do w�asnego portu PA jako kod ASCII = 90 znaku 'Z'
 *  Po od��czeniu: Atmega 328 Slave TxD i RxD ATmega 32, Master w PA [0000 0000]
 *  Master korzysta z przerwania: /ISR(USART_RXC_vect)/, a Slave z przerwania: /ISR(USART_RX_vect)/
 *  Dzia�a dla r�nych Fosc. Master 16MHz, Slave 12MHz, lub jednakowch 12MHz
 *  .......................................................................................................................
 */




#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>			//wa�ne
#include <avr/interrupt.h>

#define UART_BAUND  9600							       // deklarowana pr�dko�c UART  OK.dla 9600,19200,57600,115200
#define _UBRR ((F_CPU+UART_BAUND*8UL)/(16UL*UART_BAUND)-1)    // oblicza UBRR, z Fosc i BAUND
// BAUND to generator szybko�ci transmisji to dzielnik cz�stotliwo�ci, kt�ry generuje szybko�ci transmisji
// UBRR rejestr ma za zadanie przechowywanie liczby, kt�ra okre�la szybko�� transmisji.
# define Pczerwony (1<<PD6)				// Pczerwony [przycisk czerwony]

uint8_t dana_Slave = 0;					    // zmienna do przechowywania danej odebranej od Slave
uint8_t Dana_PC = 0;						// zmienna do przechowywania danej odebranej z klawiatury komputera PC
uint8_t licznik = 0;
//---------------------------------------- Funkcja ---------------------------------------------------------------------------
 void USART_Init( unsigned int baud )			 //Inicjalizacja Transmisji Szeregowej - Funkcja /unsigned int/
   {
	UBRRH = (unsigned char)(baud>>8);			   // Ustaw szybko�� transmisji (unsigned char)
	UBRRL = (unsigned char)baud;				  // (unsigned char)

	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);		//RXEN Odblokowanie odbiornika,TXEN Odblokowanie nadajnika.
												// RXCIE bit zezwala na generowanie przerwania na pojawienie si� flagi RXC
	UCSRC = (1<<URSEL)|(3<<UCSZ0);			   // URSEL do prze��czania dost�pu mi�dzy rejestrami UBRRH i UCSRC
   }										  // 3<<UCSZ0 8 bit�w

 // ------------------------------------- Funkcja --------------------------------------------------------------------------------

 void USART_Transmit(uint8_t data)      // wysy�anie jednego znaku przez USART  - Funkcja unsigned (char data)
 {
 	while ( !( UCSRA & (1<<UDRE)) )		   //UCSRA � Rejestr kontroli i stanu USART A
 		;

 	UDR = data;			             // UDR - Rejestr danych do transmisji oraz rejestr danych do odbioru USART
 	//UDR = 0;					//nok. n.
 }
 /*
 	Flaga UDRE wskazuje, czy bufor transmisji (UDR) jest gotowy na przyj�cie nowych danych. Je�li UDRE ma warto�� jeden, bufor jest
 	pusty i dlatego gotowy do zapisania. Flaga UDRE mo�e wygenerowa� puste przerwanie rejestru danych.
 	UDRE jest ustawiany po resecie, aby wskaza�, �e nadajnik jest gotowy.
 	*/
 //------------------------------ Funkcja ---------------------
  void USART_puts (char * s)						  // Funkcja do wysy�ania ci�g�w znak�w /litery, cyfry/puts
  {
 	while (* s ) USART_Transmit( *s++);
  }
 //-------------------------------- Funkcja -----------------
 void USART_putlong( int32_t liczba, uint8_t radix)          // Funkcja do wysy�ania liczb /uint32_t/
  {
 	 char buf[17];
 	 ltoa(liczba, buf, radix);
 	 USART_puts(buf);
  }
 //-------------------------------- Funkcja ODBIORU -----------------
  unsigned char USART_Odbior(void)
  {
	  while ( !( UCSRA & (1<<RXC)))
		  ;
	  return UDR;

  }
 // ---------------------------------------------------------------------
//--------------------------- Przerwanie UART -----------------------
 ISR(USART_RXC_vect)					// Przerwanie UART, dla ODBIORU danej/znaku, z klawiatury PC/przez portu szeregowy Rx ATmega32/
   {
/*
 // ............OK.  /Komuputer jako Master i AT32 jako Slave kt�ry odsy�a odebrany znak oraz text, na UART_Terminal/
	 Dana_PC = UDR;			             //OK. UDR Rejestr odbiorczy: odbiera znak z klawiatury PC
	 USART_puts("AT32 Master wysyla   ");
	 USART_Transmit(Dana_PC);      		//OK. wysy�a odebrany znak z klawiatury PC, przez USART: znak jest wy�wietlany w oknie UART_Terminal
	 USART_puts(" \r\n ");				//OK. zako�czenie, nowy wiersz w oknie UART_Terminal
 // ............................................................................................
*/
/*
// ..... OK. np.'s','5','=','M' /Komuputer jako Master i AT32 jako Slave kt�ry odsy�a odebrany znak na UART_Terminal/ ......
	 	 Dana_PC = UDR;
	 	 USART_Transmit(Dana_PC);   // POPRAWNE szeroko�ci stan�w H i L OK.
	 	 PORTA = Dana_PC;
// ............................... identyczne przebiegi na oscyloskopie na RxD i TxD na UART_Terminal prawid�owe znaki
*/
/*
// .................... NOK.np.'w' /Komuputer jako Master i AT32 jako Slave kt�ry odsy�a odebrany znak w UART_Terminal/ ...........
	 USART_Odbior();                           // nok.
	 USART_Transmit(UDR);			           // nok.
	 PORTA = UDR;				               // NOK !!
// ..................odebrany znak w UART_Terminal pojawia si� z op�nieniem o jeden znak, a tym samym jest b��d w PORTA
*/

	 Dana_PC = UDR;
	 PORTA = Dana_PC;

   }


 // -------------------------------------------------------------------------

 int main(void)
{
	 	 	 	 	 	 	 	 	 	 	 //  Wywo�anie funkcji Inicjalizacji
	 USART_Init (_UBRR);					//OK. (_UBRR) Parametr obliczony
	 //USART_Init (103);				     //OK. (103) Parametr z tabeli dla fosc = 16MHz /dane producenta/

	 sei();						            // Globalne odblokowanie Przerwa� OK!

	 //DDRA |= 0;
	PORTA = 0;

	 PORTD |= (1<<PD6); 					// PD6 podci�gni�ty do stanu H [przycisk czerwony]
	 DDRD |= (1<<PD5)|(1<<PD4);				//(1<<PD5) Wyj�cie steruj�ce diod� LED czerwona, (1<<PD4)Wyj�cie steruj�ce diod� LED ��t�
	 PORTD &= ~(1<<PD5)|(1<<PD4);            // pocz�tkowy stan niski L



  while(1)
	{
	       PORTD ^= (1<<PD4);				// inne znaki ni� 'a'
	  	  	 _delay_ms(50);					// Dla testu aby program czym� zaj�� i sprawdzi� dzia�anie przerwania

	  	              if(Dana_PC  == 97)			//PC wysy�a'a'  OK...dla['a' lub 97] potwierdzenie odbioru danej od klawiatura PC
	  	  	 	  		   {
							 PORTD ^= (1<<PD4);
							  _delay_ms(500);

	  	  	 	  		  }

	  if (!(PIND & Pczerwony))		// wci�ni�ty [przycisk czerwony]
        {
		  _delay_ms(100);				               // drgania styku przycisku

		       licznik++;
		     if (licznik >= 250)
		       {
		    	   licznik = 0;
		       }
 // ........................................ w oknie UART_Terminal...................
		     //USART_puts("AT32 Master wysyla   ");     // OK. w oknie UART_Terminal
		 	 //USART_Transmit('v');      		      //OK. wysy�a znak'v',odebrany z klawiatury PC,lub 65 kod ASCII znaku A, przez USART:
		 	 	 	 	 	 	 	 	 	 	    // znak jest wy�wietlany w oknie UART_Terminal
		 	//USART_putlong(61,10);				   // OK. liczba w formacie dziesi�tnym, w oknie UART_Terminal
		     //USART_putlong(licznik,10);		  // OK. w oknie UART_Terminal, je�li AT32 Master pod��czony do PC
// ...................................... Master Slave .............................................

		  //USART_Transmit('=');                  // NOK. wysy�a znak 'Z' przez UART do AT328 Slave, a Slave odsy�a do AT32 Master
		     	 	 	 	 	 	 	 	 	 // kt�ry zapisuje do PORTA jako dan�/kod ASCII, b��dy w PORTA
		  	  	  	  	  	  	  	  	  	  	 // SLAVE b��dnie odsy�a du�e znaki 'Z', 'M'..
 // OK.SLAVE odsy�a ma�e znaki 'a', '=',ale na oscyloskopie stany L i H maj� nieco inne szeroko�ci, ni� to co wysy�a MASTER, w PORTA poprawna dana

		    //USART_Transmit(54);					// NOK. Master - Slave

		   USART_putlong(123,10);						// NOK. Master - Slave
		 	 //USART_puts(" \r\n ");				   //OK. zako�czenie, nowy wiersz w oknie UART_Terminal
		    //USART_putlong(123,10);

		  while(!(PIND & (Pczerwony)))   			// dop�ki wci�ni�ty [przycisk czerwony]
			  {
				  PORTD |= (1<<PD5);  			//LED czerwona �wieci, stan H
			  }
		  PORTD &= ~(1<<PD5);                //LED czerwona wygaszona,stan L /zwolniony [przycisk czerwony]/
        }
	}



}
