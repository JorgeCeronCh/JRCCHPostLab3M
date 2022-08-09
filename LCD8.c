/*
 * File:   LCD_8.c
 * Author: Javier Alejandro Pérez Marín
 *
 * Created on 29 de julio de 2022, 09:07 AM
 */

#include <stdint.h>
#include "LCD8.h"

void Lcd8_Port(char a)              // Asignación de puertos
{                          
    PORTD = a;
}

void Lcd8_Cmd(char a)
{
    RS = 0;                         // Se ajusta en modo 8 bits (visualizador)
    PORTD = a;
    EN = 1;
    __delay_ms(4);
    EN = 0;
    
}

void Lcd8_Clear(void) {             // Limpieza
    Lcd8_Cmd(0);
    Lcd8_Cmd(1);
}

void Lcd8_Set_Cursor(char a, char b) // Selección de cursor        
{
    char x;
    if (a == 1) {                   // Si es fila 1, seleccionar columna
                                    // (fila, columna)
        x = 0x80 + b - 1;
        Lcd8_Cmd(x);
    }
    else if (a == 2) {              // Si es fila 2, seleccionar columna 
                                    // (fila, columna)
        x = 0xC0 + b - 1;
        Lcd8_Cmd(x);
    }
}

void Lcd8_Init(void) {

    Lcd8_Cmd(0x00);    // Comienza en 0 
    __delay_ms(20);
    Lcd8_Cmd(0x30);     // Se ajusta en moddo 8 bits
    __delay_ms(5);
    Lcd8_Cmd(0x30);     // Se ajusta en moddo 8 bits
    __delay_ms(10);     
    Lcd8_Cmd(0x30);     // Se ajusta en moddo 8 bits
    __delay_us(100);
    Lcd8_Cmd(0x38);     // 8 bits con 2 lineas (5x8 pixeles)
    Lcd8_Cmd(0x0C);     // Visualizador activado y apagado con cursor y parpadeo apagado
    Lcd8_Cmd(0x01);     // Visualizador borrado 
    Lcd8_Cmd(0x06);     // Modo de entrada en incremento y desplazamiento desactivado
}

void Lcd8_Write_Char(char a) {

    RS = 1;
    PORTD = a;
    EN = 1;
    __delay_us(40);
    EN = 0;

}

void Lcd8_Write_String(char *a) {
    char i;
    for (i = 0; a[i] != '\0'; i++)
        Lcd8_Write_Char(a[i]);
}

void Lcd8_Shift_Right(void) {
    Lcd8_Cmd(0x01);
    Lcd8_Cmd(0x0C);
}

void Lcd8_Shift_Left(void) {
    Lcd8_Cmd(0x01);
    Lcd8_Cmd(0x08);
}
