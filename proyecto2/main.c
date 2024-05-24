/*
 * proyecto2.c
 *
 * Created: 7/05/2024 22:35:47
 * Author : alane
 */ 
#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "PWM1/PWM1.h"
#include "PWM2/PWM2.h"
#include "ADC/ADC.h"
#include <avr/eeprom.h>
volatile uint8_t bufferRX = 0;
volatile uint8_t clectura=0;
volatile uint8_t valorADC=0;
volatile uint8_t PuertoADC=3;
volatile uint16_t top=0;
volatile uint8_t valor = 0;
volatile uint8_t imodo = 1;
volatile uint8_t iguardar = 0;
volatile uint8_t cguardar = 0;
volatile uint8_t temporaleeprom = 0;
char recibido1;
char recibido2;
char recibido3;
int n1;
int n2;
int n3;
int num;

void GUARDADOEEPROM (void);
void ESCRIBIREEPROM(void);
void MULTIADC (void);
void initUART(int brate, int turbo, int multiprocesador, int sincrono, int
paridad, int bit_stop, int cantidad_caracteres,int UBRR0_valor);
void writeUART(char caracter);
void writeTextUART(char* texto);
char receivedChar(void);

int CharToInt(char num){
	return num - '0';
}

int unir(int n1, int n2, int n3){
	return n1*100+ n2*10 + n3;
}

ISR(PCINT1_vect){
	if (!(PINC & (1 << PC3))) {
		imodo++;
	}
	if (!(PINC & (1 << PC2))) {
		iguardar++;
		cguardar = iguardar;
		GUARDADOEEPROM();
		ESCRIBIREEPROM();
	}
}

ISR(USART_RX_vect){
	bufferRX = UDR0;
}


void init_pines(void){
		// Configurar los pines PC4, PC5 y A6 como entradas y habilitar resistencias de pull-up
		DDRC = 0;
		PORTC = 0xFF;
		
		DDRD = 0xFF;
		
		PCICR |= (1 << PCIE1);
		PCMSK1 |= (1 << PCINT11) | (1 << PCINT10);
		
}

uint16_t reescalar(uint8_t valor, uint8_t max_origen, uint16_t max_destino) {
	// Escalar el valor al rango 0 - max_destino
	uint16_t valor_reescalado = (float)valor / max_origen * max_destino;
	//INICIAR TIMER0 (DUTY CYCLE)
	return valor_reescalado;
}

void MULTIADC (void){
	if (PuertoADC==4) {
		valor = readADC(4);
		uint8_t valor_reescalado = reescalar(valor, 255, 40);
		duty_cycle1A(valor_reescalado);
		PuertoADC++;
		} else if (PuertoADC==5) {
		valor = readADC(5);
		uint8_t valor_reescalado = reescalar(valor, 255, 40);
		duty_cycle1B(valor_reescalado);
		PuertoADC++;
		} else if (PuertoADC==6) {
		valor = readADC(6);
		uint8_t valor_reescalado = reescalar(valor, 255, 40);
		duty_cycle2A(valor_reescalado);
		PuertoADC++;
		} else if (PuertoADC==7) {
		valor = readADC(7);
		uint8_t valor_reescalado = reescalar(valor, 255, 40);
		duty_cycle2B(valor_reescalado);
		PuertoADC++;
		} else {
		PuertoADC = 4;
	}
}
void GUARDADOEEPROM (void){
	uint8_t valor_reescalado;
	if (imodo==1){
	if (cguardar==1)
	{
			valor = readADC(4);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 0,  valor_reescalado);
			valor = readADC(5);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 1,  valor_reescalado);
			valor = readADC(6);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 2,  valor_reescalado);
			valor = readADC(7);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 3,  valor_reescalado);
			cguardar=0;
	}
	if (cguardar==2)
	{
			valor = readADC(4);
			uint16_t valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 4,  valor_reescalado);
			valor = readADC(5);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 5,  valor_reescalado);
			valor = readADC(6);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 6,  valor_reescalado);
			valor = readADC(7);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 7,  valor_reescalado);
			cguardar=0;
	}
	if (cguardar==3)
	{
			valor = readADC(4);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 8, valor_reescalado);
		
			valor = readADC(5);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 9, valor_reescalado);
			valor = readADC(6);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 10,  valor_reescalado);
			valor = readADC(7);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 11,  valor_reescalado);
			cguardar=0;
	}
	if (cguardar==4)
	{
			valor = readADC(4);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 12,  valor_reescalado);
			valor = readADC(5);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 13,  valor_reescalado);
			valor = readADC(6);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 14,  valor_reescalado);
			valor = readADC(7);
			 valor_reescalado = reescalar(valor, 255, 40);
			eeprom_write_byte((uint8_t*) 15,  valor_reescalado);
			cguardar=0;
			iguardar=0;
	}
	}
}
void ESCRIBIREEPROM(void){
	if (imodo==2)
	{
	switch (iguardar){
		case 1:
		
		temporaleeprom = eeprom_read_byte((uint8_t*) 0);
		duty_cycle1A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*)1);
		duty_cycle1B(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 2);
		duty_cycle2A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 3);
		duty_cycle2B(temporaleeprom);
		break;
		
		case 2:
		
		temporaleeprom = eeprom_read_byte((uint8_t*) 4);
		duty_cycle1A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 5);
		duty_cycle1B(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 6);
		duty_cycle2A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 7);
		duty_cycle2B(temporaleeprom);
		break;
		
		case 3:
		
		temporaleeprom = eeprom_read_byte((uint8_t*) 8);
		duty_cycle1A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 9);
		duty_cycle1B(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 10);
		duty_cycle2A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 11);
		duty_cycle2B(temporaleeprom);
		break;
		
		case 4:
		
		temporaleeprom = eeprom_read_byte((uint8_t*) 12);
		duty_cycle1A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 13);
		duty_cycle1B(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 14);
		duty_cycle2A(temporaleeprom);
		temporaleeprom = eeprom_read_byte((uint8_t*) 15);
		duty_cycle2B(temporaleeprom);
		iguardar=0;
		break;
		
	}
	}
}
int main(void) {
	cli();
	CLKPR = CLKPCE;
	CLKPR = CLKPS0;
	init_pines();
	//frecuencia del adc -> 16M/128 = 125kHz
	init_ADC(0,0,128);
	top = 155;
	int preescaler=1024;
	uint8_t valorADC;
	uint8_t valor_reescalado;
	init_PWM1A(0,6,preescaler, top);
	init_PWM1B(0);
	init_PWM2A(0, 3, preescaler);
	init_PWM2B(0);
	sei();
	while (1) {
		switch (imodo){
			//MODO ADC
			case 1:
			PORTD &= ~(1<<DDD2) & ~(1<<DDD4) & ~(1<<DDD5);
			PORTD = (1<<DDD2);
			MULTIADC();
			//FIN MODO ADC
			break;
			//MODO UART/ADAFRUIT
			case 2:
			//MODO EEPROM
			PORTD &= ~(1<<DDD2) & ~(1<<DDD4) & ~(1<<DDD5);
			PORTD |= (1<<DDD5);
			
			//FIN MODO EEPROM
			break;
			//MODO ADAFRUIT
			case 3:
			
			PORTD &= ~(1<<DDD2) & ~(1<<DDD4) & ~(1<<DDD5);
			PORTD |= (1<<DDD4);
			initUART(9600,0,0,0,0,1,8,103);
			_delay_ms(100);
			
			writeUART('1');
			
			recibido1=receivedChar();
			recibido2=receivedChar();
			recibido3=receivedChar();
			
			n1=CharToInt(recibido1);
			n2=CharToInt(recibido2);
			n3=CharToInt(recibido3);
			
			num= unir(n1,n2,n3);
			
			valor_reescalado = reescalar(num, 255, 40);
			duty_cycle1A(valor_reescalado);
			
			writeUART('2');
			recibido1=receivedChar();
			recibido2=receivedChar();
			recibido3=receivedChar();
			
			n1=CharToInt(recibido1);
			n2=CharToInt(recibido2);
			n3=CharToInt(recibido3);
			
			num= unir(n1,n2,n3);
			
			num= unir(n1,n2,n3);
			
			valor_reescalado = reescalar(num, 255, 40);
			duty_cycle1B(valor_reescalado);
			
			writeUART('3');
			recibido1=receivedChar();
			recibido2=receivedChar();
			recibido3=receivedChar();
			
			n1=CharToInt(recibido1);
			n2=CharToInt(recibido2);
			n3=CharToInt(recibido3);
			
			num= unir(n1,n2,n3);
			
			valor_reescalado = reescalar(num, 255, 40);
			duty_cycle2A(valor_reescalado);
			
			writeUART('4');
			recibido1=receivedChar();
			recibido2=receivedChar();
			recibido3=receivedChar();
			
			n1=CharToInt(recibido1);
			n2=CharToInt(recibido2);
			n3=CharToInt(recibido3);
			
			num= unir(n1,n2,n3);
			
			valor_reescalado = reescalar(num, 255, 40);
			duty_cycle2B(valor_reescalado);
			//FIN MODO UART/ADAFRUIT
			break;
			case 4:
			imodo=1;
			break;
		}
	}
	return 0;
}
void initUART(int brate, int turbo, int multiprocesador, int sincrono, int
paridad, int bit_stop, int cantidad_caracteres,int UBRR0_valor){
	//Configuramos los pines TX y RX
	DDRD &= ~(1<<DDD0);
	DDRD |= (1<<DDD1);
	//Configuramos A modo FAST U2X0 = 1
	UCSR0A = 0;
	//Modo turbo
	if (turbo==1){
		UCSR0A |= (1<<U2X0);
	}
	//Habilitar multiprocesador
	if (multiprocesador==1){
		UCSR0A |= (1<<MPCM0);
	}
	//Configuramos el b y habilitamos ISRR RX, Habilitamos RX y TX
	UCSR0B = 0;
	UCSR0B |= (1<<RXCIE0)|(1<<UDRIE0)|(1<<UDRIE0);
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	//Configurar C Frame 8 bits datos, no paridad, 1 bit de stop
	UCSR0C=0;
	if (sincrono==1){
		UCSR0C |= (1<<UMSEL00);
	}
	if (paridad==1){

		UCSR0C |= (1<<UPM01);
		} else if (paridad==2){

		UCSR0C |= (1<<UPM01)|(1<<UPM00);
	}
	if (bit_stop==2){
		UCSR0C |= (1<<USBS0);
	}
	switch (cantidad_caracteres){
		case 5:
		break;
		case 6:
		UCSR0C |= (1<<UCSZ00);
		break;
		case 7:
		UCSR0C |= (1<<UCSZ01);
		break;
		case 8:
		UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
		break;
		case 9:
		UCSR0C |= (1<<UCSZ02)|(1<<UCSZ01)|(1<<UCSZ00);
		break;
	}
	UBRR0=UBRR0_valor;
	
	
}
void writeUART(char caracter){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = caracter;
	
}
void writeTextUART(char* texto){
	uint8_t i;
	for (i = 0; texto[i]!='\0'; i++)
	{
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = texto[i];
	}
}
char receivedChar(void){
	
	while (!(UCSR0A & (1 << RXC0)));
	bufferRX=UDR0;
}