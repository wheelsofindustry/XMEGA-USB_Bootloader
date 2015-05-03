// USB Bootloader for XMEGA AU
// February 2014

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "usb_xmega.h"
#include "Descriptors.h"
#include "main.h"
#include "display.h"

FUSES = {
    .FUSEBYTE1 = 0x00,  /* Watchdog Configuration */
    .FUSEBYTE2 = 0xBD,  /* Reset Configuration */
    .FUSEBYTE4 = 0xF7,  /* Start-up Configuration */
    .FUSEBYTE5 = 0xE9,  /* EESAVE and BOD Level */
};
/*  Fuses:
    WDWP = 8CLK
    WDP = 8CLK
    BOOTRST = BOOTLDR
    TOSCSEL = XTAL
    BODPD = SAMPLED
    RSTDISBL = [ ]
    SUT = 4MS
    WDLOCK = [ ]
    BODACT = CONTINUOUS
    EESAVE = [ ]
    BODLVL = 2V8    */

const char FW_Info[] PROGMEM = "GABOOT! FW 0.01";     // Firmware info, max 15 char + NULL

int main(void) {
    // Clock Settings
    // USB Clock
    OSC.DFLLCTRL = OSC_RC32MCREF_USBSOF_gc; // Configure DFLL for 48MHz, calibrated by USB SOF
    DFLLRC32M.CALB = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSC));
    DFLLRC32M.COMP1 = 0x1B; // Xmega AU manual, p41
    DFLLRC32M.COMP2 = 0xB7;
    DFLLRC32M.CTRL = DFLL_ENABLE_bm;

    OSC.PLLCTRL = 0x10;                         // 2MHz is PLL Source - 16x Factor (32MHz)
    OSC.CTRL = OSC_RC2MEN_bm | OSC_RC32MEN_bm | OSC_PLLEN_bm | OSC_XOSCEN_bm;   // Enable 2MHz and 32MHz oscillators and PLL
    delay_ms(2);                                // Wait for PLL ready
    CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // Switch to PLL clock

    // PORTS CONFIGURATION
    PORTCFG.VPCTRLA = 0x41; // VP1 Map to PORTE, VP0 Map to PORTB
    PORTCFG.VPCTRLB = 0x32; // VP3 Map to PORTD, VP2 Map to PORTC
    // Initial value PORTA.DIR       = 0x00; // CH2, CH1, 1V, K1, K2, K3, K4, REF
    VPORT0.DIR		= 0x0B; // PORTB.DIR RES, AWG, D/C, R/W
    // Initial Value PORTB.OUT       = 0x00; //
    // Initial Value PORTC.DIR       = 0x00; // LOGIC
    VPORT3.DIR		= 0x1F; // PORTD.DIR USB, EXT, GREEN, DAT, TP, CLK, RED
    VPORT3.OUT		= 0x04; // PORT.OUT LCD voltage off
    VPORT1.DIR		= 0x09;	// PORTE.DIR TX, RX, RTS (input), CTS (power external board)
    VPORT1.OUT		= 0x01; // PORTE.OUT Power to external board
    PORTA.PIN1CTRL  = MENUPULL; // Pull up or pull down on pin PA1
    PORTA.PIN2CTRL  = 0x18; // Pull up on pin PA2
    PORTA.PIN3CTRL  = 0x18; // Pull up on pin PA3
    PORTA.PIN4CTRL  = 0x18; // Pull up on pin PA4
    PORTA.PIN5CTRL  = 0x07; // Digital Input Disable on pin PA5
    PORTA.PIN6CTRL  = 0x07; // Digital Input Disable on pin PA6
    PORTA.PIN7CTRL  = 0x07; // Digital Input Disable on pin PA7
    //PORTB.PIN2CTRL	= 0x07; // Input Disable on pin PB2
    PORTD.PIN5CTRL  = 0x01; // Sense rising edge (Freq. counter)
    PORTE.PIN1CTRL  = 0x18; // Pull up on pin PE1 (RTS)

    // Initialize USARTD0 for OLED
    USARTD0.BAUDCTRLA = FBAUD;	        // SPI clock rate for display
    USARTD0.CTRLC     = 0xC0;           // Master SPI mode,
    USARTD0.CTRLB     = 0x08;           // Enable TX
    DMA.CTRL          = 0x80;           // Enable DMA, single buffer, round robin
        
    if( !testbit(PORTA.IN,4) ||         // If K1 is pressed, or
        (pgm_read_word(0) == 0xFFFF) || // Unprogrammed flash, or
        (RST.STATUS == 0)) {            // Coming from application
        
        // Interrupt Configuration
        CCPWrite(&PMIC.CTRL, PMIC_IVSEL_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm);    // Move Interrupt Table to Boot

        sei();      // Enable global interrupts
    
        USB_ResetInterface();
        // Initialize LCD
        GLCD_LcdInit();
        GLCD_setting();
        tiny_printp(0,0,FW_Info);
        tiny_printp(0,1,HW_Info);
        dma_display();

        while (1) { }
    }
}

// From Application Note AVR1003
void CCPWrite( volatile uint8_t * address, uint8_t value ) {
    uint8_t volatile saved_sreg = SREG;
    cli();

#ifdef __ICCAVR__
	asm("movw r30, r16");
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm("ldi  r16,  0xD8 \n"
	    "out  0x34, r16  \n"
#if (__MEMORY_MODEL__ == 1)
	    "st     Z,  r17  \n");
#elif (__MEMORY_MODEL__ == 2)
	    "st     Z,  r18  \n");
#else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
	    "st     Z,  r19  \n");
#endif /* __MEMORY_MODEL__ */

#elif defined __GNUC__
	volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
		);

#endif
	SREG = saved_sreg;
}

// Read out calibration byte.
uint8_t ReadCalibrationByte(uint8_t location) {
    uint8_t result;
    // Load the NVM Command register to read the calibration row.
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte(location);
    /* Clean up NVM Command register. */
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;
    return result;
}

void delay_ms(uint8_t n) {
    for(;n;n--) _delay_ms(1);
}

// Convert lower nibble to hex char
uint8_t hex_to_char(uint8_t hex) {
    if (hex < 10) hex += '0';
    else          hex += 'A' - 10;
    return(hex);
}
