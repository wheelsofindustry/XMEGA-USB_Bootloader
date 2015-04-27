// Hardware specific definitions

#ifndef HARDWARE_H
#define HARDWARE_H

#include "SSD1306.h"

#define MENUPULL 0x18					// Menu button Pull up

#define     LCD_RESET       1           // RESET
#define		LCD_RS		    3           // Data / Instruction
#define     LCD_CS          0           // Chip Select
#define		LCD_CTRL        VPORT0.OUT

// PORT DEFINITIONS
#define LEDGRN  4           // PORTD
#define LEDRED  0           // PORTD

#define ONGRN()     setbit(VPORT3.OUT, LEDGRN)
#define OFFGRN()    clrbit(VPORT3.OUT, LEDGRN)
#define ONRED()     setbit(VPORT3.OUT, LEDRED)
#define OFFRED()    clrbit(VPORT3.OUT, LEDRED)

// Port definitions for Assembly code

#define EXTPIN 0x001e,5 // External trigger pin is VPORT3.5
#define CH1ADC 0x0224   // ADCA CH0.RESL
#define CH2ADC 0x022C   // ADCA CH1.RESL

#endif
