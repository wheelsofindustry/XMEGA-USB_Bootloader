/* ----------------------------------------------------------------------------
	Graphic display functions

	FILE NAME 	: display.h

	DESCRIPTION	: The purpose of this function is to manage a graphic LCD
			  by providing function for control and display text and graphic

	AUTHOR		: Gabriel Anzziani
    www.gabotronics.com

*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "hardware.h"

typedef struct {
    uint8_t     display_setup[3];
    uint8_t     display_data[DISPLAY_DATA_SIZE];
} Disp_data;

extern Disp_data Disp_send;
extern uint8_t u8CursorX, u8CursorY;

#define lcd_goto(x,y) { u8CursorX=(x); u8CursorY=(y); }

/* EXTERN Function Prototype(s) */
void GLCD_LcdInit(void);
void GLCD_LcdOff(void);
void lcd_putsp(const char *);
void GLCD_Putchar (char);
void printN(uint8_t Data);
void clr_display(void);
void write_display(uint8_t data);
void tiny_printp(uint8_t x, uint8_t y, const char *ptr);
void LcdInstructionWrite (unsigned char);

#endif
