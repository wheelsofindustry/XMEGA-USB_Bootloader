// USB Bootloader for XMEGA AU
// February 2014

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "sp_driver.h"
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

uint8_t *Outgoing;
uint8_t *Incoming;

/*
#define HW_PRODUCT XPROTOWATCH
#define HW_VERSION 1.4

#define REQ_INFO		0xB0    // Returns a BootloaderInfo with info about the device
typedef struct{
    uint8_t magic[4]; // String 0x90 0x90 0xBB 0x01
    uint8_t version;
    uint8_t DEVID0;   // Device/Revision ID from MCU. See XMEGA AU Manual p46
    uint8_t DEVID1;
    uint8_t DEVID2;
    uint8_t REVID;
    uint16_t page_size;  // Page size in bytes
    uint32_t app_section_end; // Byte address of end of flash. Add one for flash size
    uint32_t entry_jmp_pointer; // App code can jump to this pointer to enter the bootloader
    char hw_product[16];
    char hw_version[16];
} BootloaderInfo;

#define REQ_ERASE		0xB1    // Erases the application section
#define REQ_START_WRITE 0xB2    // Sets the write pointer to the page address passed in wIndex.
// Data written to bulk endpoint 1 will be written at this address, and the
// address will be incremented automatically. The transfer is complete when a
// packet of less than 64 bytes is written.
#define REQ_CRC_APP		0xB3   // Return a CRC of the application section
#define REQ_CRC_BOOT	0xB4   // Return a CRC of the boot section
#define REQ_RESET	    0xBF
// After acknowledging this request, the bootloader disables USB and resets
// the microcontroller

#define CONCAT_HIDDEN(a, b, c) a##b##c
#define PINCTRL(n) CONCAT_HIDDEN(PIN, n, CTRL)

#define xstringify(s) stringify(s)
#define stringify(s) #s

/// Flash page number where received data will be written
uint16_t page;

/// Byte offset into flash page of next data to be received
uint16_t pageOffs;

/// Buffer of incoming flash page
uint8_t pageBuf[APP_SECTION_PAGE_SIZE];

void pollEndpoint(void);


void reset_into_bootloader(void) __attribute__ ((noinline));
void reset_into_bootloader(void){
    PMIC.CTRL = 0;
    bootloaderflag = BOOTLOADER_MAGIC;
    CCP = CCP_IOREG_gc;
    RST.CTRL = RST_SWRST_bm;
    while(1);
}

/// Jump target at known address to call from application code to switch to bootloader
extern void enterBootloader(void) __attribute__((used, naked, section(".boot-entry")));
void enterBootloader(void){
    reset_into_bootloader();
}
*/
int main(void){
    // Clock Settings
    // USB Clock
    OSC.DFLLCTRL = OSC_RC32MCREF_USBSOF_gc; // Configure DFLL for 48MHz, calibrated by USB SOF
    DFLLRC32M.CALB = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSC));
    DFLLRC32M.COMP1 = 0x1B; // Xmega AU manual, p41
    DFLLRC32M.COMP2 = 0xB7;
    DFLLRC32M.CTRL = DFLL_ENABLE_bm;
    
    OSC.XOSCCTRL = 0xCB;    // Crystal type 0.4-16 MHz XTAL - 16K CLK Start Up time
    OSC.PLLCTRL = 0xC2;     // XOSC is PLL Source - 2x Factor (32MHz)
    OSC.CTRL = OSC_RC2MEN_bm | OSC_RC32MEN_bm | OSC_XOSCEN_bm;
    _delay_ms(2);
    // Switch to internal 2MHz if crystal fails
    if(!testbit(OSC.STATUS,OSC_XOSCRDY_bp)) {   // Check XT ready flag
        OSC.XOSCCTRL = 0x00;    // Disable external oscillators
        // Not entering, comment to save
        //OSC.PLLCTRL = 0x10;     // 2MHz is PLL Source - 16x Factor (32MHz)
    }
    OSC.CTRL = OSC_RC2MEN_bm | OSC_RC32MEN_bm | OSC_PLLEN_bm | OSC_XOSCEN_bm;
    _delay_ms(2);
    CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // Switch to PLL clock
    // Clock OK!
    OSC.CTRL = OSC_RC32MEN_bm | OSC_PLLEN_bm | OSC_XOSCEN_bm;    // Disable internal 2MHz

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
        tiny_printp(0,0,PSTR("BOOTLOADER v1.0"));
        dma_display();

        while (1) { }
    }
}

/*
/// Pack the ep0 input buffer with a response to REQ_INFO
void fillInfoStruct(void){
    BootloaderInfo *i=(BootloaderInfo*)ep0_buf_in;
    i->magic[0] = 0x90;
    i->magic[1] = 0x90;
    i->magic[2] = 0xBB;
    i->magic[3] = 0x01;
    i->version = 1;
    i->DEVID0 = MCU.DEVID0;
    i->DEVID1 = MCU.DEVID1;
    i->DEVID2 = MCU.DEVID2;
    i->REVID = MCU.REVID;
    i->page_size = APP_SECTION_PAGE_SIZE;
    i->app_section_end = APP_SECTION_END;
    i->entry_jmp_pointer = (uint32_t) (unsigned) &enterBootloader;
    strncpy(i->hw_product, xstringify(HW_PRODUCT), 16);
    strncpy(i->hw_version, xstringify(HW_VERSION), 16);
}

// Event handler for the library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(struct USB_Request_Header* req) {
	uint8_t *p;
	uint8_t i=0;
    RTC.CNT=0;  // Clear screen saver timer
	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_VENDOR) {
		switch(req->bRequest) {
            case REQ_INFO:
				fillInfoStruct();
				USB_ep0_send(sizeof(BootloaderInfo));
				return;
            case REQ_ERASE:
				SP_EraseApplicationSection();
			break;
            case REQ_START_WRITE:
				page = req->wIndex;
				pageOffs = 0;
				USB_ep_start_bank(1, 0, pageBuf, 0);
			break;
            case REQ_CRC_APP:
				*(uint32_t*)ep0_buf_in = SP_ApplicationCRC();
				USB_ep_in_start(0, sizeof(uint32_t));
				return;
            case REQ_CRC_BOOT:
				*(uint32_t*)ep0_buf_in = SP_BootCRC();
				USB_ep_in_start(0, sizeof(uint32_t));
				return;
            case REQ_RESET:
				cli();
				USB_ep_in_start(0, 0);
				USB_ep0_enableOut();
				USB_ep_wait(0x80); // Wait for the status stage to complete
				_delay_ms(10);
				USB.CTRLB &= ~USB_ATTACH_bm; // USB Detach
				_delay_ms(100);
				bootloaderflag = 0;
				CCP = CCP_IOREG_gc;
				RST.CTRL = RST_SWRST_bm;
				while(1) {};
//		    default:    // Unknown request
//    			endpoints[0].out.CTRL |= USB_EP_STALL_bm;
//	    		endpoints[0].in.CTRL |= USB_EP_STALL_bm;
		}
        USB_ep_in_start(0, 0);
	}
}
*/

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
