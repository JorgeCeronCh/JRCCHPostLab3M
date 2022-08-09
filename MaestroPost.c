/*
 * File:   MaestroPost.c
 * Author: Jorge Cerón
 *
 * Created on 8 de agosto de 2022, 12:07 PM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h> 
#include <stdio.h>
#include "spi.h"
#include "oscilador.h"
#include "LCD8.h"
#include "tmr0.h"
#define _XTAL_FREQ 1000000
#define RS RC6
#define EN RC7
#define IN_MIN 0
#define IN_MAX 1023 
#define OUT_MIN 0
#define OUT_MAX 500


/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint16_t ADRESH1= 0;
uint16_t ADRESH2= 0;
uint16_t ADRESL1 = 0;
uint16_t ADRESL2 = 0;
uint8_t i = 0;
uint8_t VALOR_DESC = 0;
uint16_t ADREST1 = 0;
uint16_t ADREST2 = 0;
uint16_t MAP1;
uint16_t MAP2;
int UNI;
int DECI;
int UNI2;
int DECI2;
char VALORES1[10];
char VALORES2[10];
uint8_t CONT_TMR0 = 0;
uint8_t MOSTRAR = 0;
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(INTCONbits.T0IF){
        CONT_TMR0++;                        // Incrementar CONT_TMR0 en 1
        if (CONT_TMR0 == 10){               // CONT_TMR0 se repite 10 veces (1 SEG)
            MOSTRAR = 1;                    // Encender/Apagar bandera
            CONT_TMR0 = 0;                  // Limpiar CONT_TMR0
        }
        tmr0_reload();
    }
    return;
}

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint16_t  val, uint8_t  in_min, uint16_t in_max, 
            unsigned short out_min, unsigned short out_max);


void main(void) {
    int_osc_MHz(1);
    setup();
    tmr0_init(255);
    tmr0_reload();
    unsigned int a;
    Lcd8_Init();
    while(1){
        if (MOSTRAR){
            PORTEbits.RE0 = 0;          // SS Activo
            __delay_ms(1); 
            spiWrite(72);               // Habilitar el envío del ANDRESH1 del Slave
            if (i == 0){                // Primer valor que devuelve el Slave es desconocido
                VALOR_DESC = spiRead(); // Se lee este primer valor desconocido
                i = 1;                  // Una vez se lee, los demás envíos son correctos
                __delay_ms(1); 
                PORTEbits.RE0 = 0;      // SS Inactivo
            } else {
                ADRESL2 = spiRead();    // Se muestra el valor enviado por el Slave
                __delay_ms(1); 
                PORTEbits.RE0 = 0;      // SS Inactivo
            }
            __delay_ms(100);

            PORTEbits.RE0 = 0;          // SS Activo
            spiWrite(76);               // Habilitar el envío del ANDRESL1 del Slave
            ADRESH1 = spiRead();        // Se muestra el valor enviado por el Slave
            __delay_ms(1); 
            PORTEbits.RE0 = 1;          //SS Inactivo

            __delay_ms(100);

            PORTEbits.RE0 = 0;          // SS Activo
            spiWrite(65);               // Habilitar el envío del ANDRESL2 del Slave
            ADRESL1 = spiRead();        // Se muestra el valor enviado por el Slave
            __delay_ms(1); 
            PORTEbits.RE0 = 1;          //SS Inactivo

             __delay_ms(100);

            PORTEbits.RE0 = 0;          // SS Activo
            spiWrite(66);               // Habilitar el envío del ANDRESH2 del Slave
            ADRESH2 = spiRead();        // Se muestra el valor enviado por el Slave
            __delay_ms(1); 
            PORTEbits.RE0 = 1;          //SS Inactivo

            __delay_ms(100);

            ADREST1 = ((ADRESH1 << 2) + ADRESL1);
            ADREST2 = ((ADRESH2 << 2) + ADRESL2);
            MAP1 = (uint16_t)(map(ADREST1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX));
            UNI = (MAP1/100);
            DECI = (MAP1 - UNI*100);
            sprintf(VALORES1, "%d.%d V", UNI, DECI);

            MAP2 = (uint16_t)(map(ADREST2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX));
            UNI2 = (MAP2/100);
            DECI2 = (MAP2 - UNI2*100);
            sprintf(VALORES2, "%d.%d V", UNI2, DECI2);
            MOSTRAR = 0;
        }
        Lcd8_Set_Cursor(1,1);
        Lcd8_Write_String("Pot 1:   Pot 2:");
        __delay_ms(100);
        Lcd8_Set_Cursor(2,1);
        Lcd8_Write_String(VALORES1);
        Lcd8_Set_Cursor(2,10);
        Lcd8_Write_String(VALORES2);
        __delay_ms(100);
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){       
    // Configuración de puertos
    ANSEL = 0b00000000;         // I/O digitales
    ANSELH = 0b00000000;        // I/O digitales
    TRISD = 0b00000000;         // PORTD como salida
    TRISEbits.TRISE0 = 0;       // RE0 como salida para SS
    PORTD = 0b00000000;         // Limpiar PORTD
    PORTE = 0b0001;             // Limpiar PORTE
    TRISCbits.TRISC6 = 0; //RC6 salida
    TRISCbits.TRISC7 = 0; //RC7 salida
    PORTCbits.RC6 = 0;
    PORTCbits.RC7 = 0;
    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
    // Configuración de interrupciones
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitar interrupciones de perifericos
    return;
}

unsigned short map(uint16_t x, uint8_t x0, uint16_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}