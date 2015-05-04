// Minimal USB Stack for ATxmega32a4u and related
// http://nonolithlabs.com
// (C) 2011 Kevin Mehall (Nonolith Labs) <km@kevinmehall.net>
//
// Heavily borrows from LUFA
// Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)
//
// Licensed under the terms of the GNU GPLv3+

#include <util/delay.h>
#include <avr/wdt.h>
#include "main.h"
#include "hardware.h"
#include "sp_driver.h"
#include "usb_xmega.h"

uint8_t		page_buffer[APP_SECTION_PAGE_SIZE];
uint16_t	page_ptr = 0;

// Bulk transfer buffers
uint8_t Outgoing[APP_SECTION_PAGE_SIZE];
uint8_t Incoming[APP_SECTION_PAGE_SIZE];

// Volatile?
uint8_t ep0_buf_in[USB_EP0SIZE];
uint8_t ep0_buf_out[USB_EP0SIZE];
// XMega manual: The pointer to the endpoint configuration table must be aligned to a 16-bit word
// Manually defined in asmutil.S // USB_EP_pair_t endpoints[USB_MAXEP+1];

volatile uint8_t USB_DeviceState;
volatile uint8_t USB_Device_ConfigurationNumber;
static inline void EVENT_USB_Device_ControlRequest(struct USB_Request_Header* req);

static inline void USB_handleSetAddress(USB_Request_Header_t* req) {
	uint8_t    DeviceAddress = (req -> wValue & 0x7F);
	endpoints[0].out.STATUS &= ~(USB_EP_SETUP_bm | USB_EP_BUSNACK0_bm);
	USB_ep0_in_start(0);
	while (!(endpoints[0].in.STATUS & USB_EP_TRNCOMPL0_bm)); // wait for status stage to complete
	USB.ADDR = DeviceAddress;
	USB_DeviceState = (DeviceAddress) ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
}

#if !defined(NO_INTERNAL_SERIAL) && (USE_INTERNAL_SERIAL != NO_DESCRIPTOR)
inline void USB_Device_GetSerialString(uint16_t* const UnicodeString) __attribute__ ((nonnull (1)));
inline void USB_Device_GetSerialString(uint16_t* const UnicodeString) {
	//uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
	uint8_t SigReadAddress = INTERNAL_SERIAL_START_ADDRESS;
	//GlobalInterruptDisable();

	for (uint8_t SerialCharNum = 0; SerialCharNum < (INTERNAL_SERIAL_LENGTH_BITS / 4); SerialCharNum++) {
		uint8_t SerialByte;

		NVM.CMD    = NVM_CMD_READ_CALIB_ROW_gc;
		SerialByte = pgm_read_byte(SigReadAddress);

		if (SerialCharNum & 0x01) {
			SerialByte >>= 4;
			SigReadAddress++;
		}
		SerialByte &= 0x0F;
		UnicodeString[SerialCharNum] = ((SerialByte >= 10) ?
		(('A' - 10) + SerialByte) : ('0' + SerialByte));
	}
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	//SetGlobalInterruptMask(CurrentGlobalInt);
}

static inline void USB_Device_GetInternalSerialDescriptor(void) {
	struct {
		USB_Descriptor_Header_t Header;
		uint16_t                UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
	}* SignatureDescriptor = (void*) ep0_buf_in;

	SignatureDescriptor->Header.Type = DTYPE_String;
	SignatureDescriptor->Header.Size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);

	USB_Device_GetSerialString(SignatureDescriptor->UnicodeString);
	USB_ep0_in_start(sizeof(*SignatureDescriptor));
}
#endif

static inline void USB_handleGetDescriptor(USB_Request_Header_t* req) {
	const void* DescriptorPointer;
	uint16_t  DescriptorSize;

	#if !defined(NO_INTERNAL_SERIAL) && (USE_INTERNAL_SERIAL != NO_DESCRIPTOR)
	if (req->wValue == ((DTYPE_String << 8) | USE_INTERNAL_SERIAL)){
		USB_Device_GetInternalSerialDescriptor();
	}
	#endif

	if ((DescriptorSize = CALLBACK_USB_GetDescriptor(req->wValue, req->wIndex, &DescriptorPointer))){
		if (DescriptorSize > req->wLength) DescriptorSize=req->wLength;
		USB_ep0_send_progmem(DescriptorPointer, DescriptorSize);
	}
	/* Error -> Stall endpoint */
	//return false;
}

static inline void USB_handleSetConfiguration(USB_Request_Header_t* req) {
	USB_ep0_in_start(0);
	USB_Device_ConfigurationNumber = (uint8_t)(req -> wValue);

	if (USB_Device_ConfigurationNumber) USB_DeviceState = DEVICE_STATE_Configured;
	else							    USB_DeviceState = (USB.ADDR) ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;
}

void USB_ResetInterface(void) {
	// USB D- and D+ Calibration registers
	USB.CAL0 = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL0) );
	USB.CAL1 = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL1) );

	CLK.USBCTRL = CLK_USBSRC_RC32M_gc | CLK_USBSEN_bm;
	USB.EPPTR = (uint16_t)endpoints;
	USB.ADDR = 0;

	// Configure Endpoint 0 for CONTROL transfers
	endpoints[0].out.STATUS = 0;
	endpoints[0].out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_EP0SIZE);
	endpoints[0].out.DATAPTR = (uint16_t)ep0_buf_out;
	endpoints[0].in.STATUS = USB_EP_BUSNACK0_bm;
	endpoints[0].in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_EP0SIZE);
	endpoints[0].in.DATAPTR = (uint16_t)ep0_buf_in;
	// Configure Endpoint 1 for BULK transfers, Multipacket enabled
	endpoints[1].in.STATUS = USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm;
	endpoints[1].in.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_MULTIPKT_bm | USB_EP_size_to_gc(64);
	endpoints[1].in.DATAPTR = (uint16_t)Incoming;
	endpoints[1].out.STATUS = 0; // Accept new data
	endpoints[1].out.CTRL = USB_EP_TYPE_BULK_gc | USB_EP_MULTIPKT_bm | USB_EP_size_to_gc(64);
	endpoints[1].out.DATAPTR = (uint16_t)Outgoing;
	endpoints[1].out.CNT = 0;
	endpoints[1].out.AUXDATA = APP_SECTION_PAGE_SIZE;           // Match USB transfer with flash page size

	USB.CTRLA		= USB_ENABLE_bm | USB_SPEED_bm | USB_MAXEP; // Enable USB at Full Speed, USB_MAXEP endpoints
	USB.INTCTRLA	= USB_BUSEVIE_bm | USB_INTLVL1_bm;			// Enable interrupt for Suspend, Resume or Reset Bus events
	USB.INTCTRLB	= USB_TRNIE_bm | USB_SETUPIE_bm;			// Enable Transaction Complete Interrupts
	USB.CTRLB	    = USB_ATTACH_bm;							// Attaches the device to the USB lines
}

void USB_ep0_send_progmem(const uint8_t* addr, uint8_t size) {
	uint8_t *buf = ep0_buf_in;
	uint8_t remaining = size;
	while (remaining--) {
		*buf++ = pgm_read_byte(addr++);
	}
	USB_ep0_in_start(size);
}

/* USB bus event interrupt includes :
 * - USB line events SOF, reset, suspend, resume, wakeup
 * - endpoint control errors underflow, overflow, stall  */
ISR(USB_BUSEVENT_vect) {
	if (USB.STATUS & USB_BUSRST_bm) {   // A reset condition has been detected
		USB.CTRLA=0;					// Disable USB
		USB_ResetInterface();
	}
	USB.INTFLAGSACLR=0xFF;
}

/* USB transfer complete interrupt includes events about endpoint transfer on all endpoints. */
ISR(USB_TRNCOMPL_vect) {
    ONRED();    // Turn on red LED
	if (endpoints[0].out.STATUS & USB_EP_SETUP_bm) {	// Setup transaction complete
	    USB_Request_Header_t* req = (void *) ep0_buf_out;
	    if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_STANDARD) {
		    switch (req->bRequest) {    // Check for standard requests
			    case REQ_GetStatus:
			        ep0_buf_in[0] = 0;	// No remote wakeup, no self power
			        ep0_buf_in[1] = 0;
			        USB_ep0_in_start(2);
			    break;
			    case REQ_ClearFeature:
			    case REQ_SetFeature:    USB_ep0_in_start(0);    		    break;
			    case REQ_SetAddress:    USB_handleSetAddress(req);          break;
			    case REQ_GetDescriptor: USB_handleGetDescriptor(req);       break;
			    case REQ_GetConfiguration:
			        ep0_buf_in[0] = USB_Device_ConfigurationNumber;
			        USB_ep0_in_start(1);
			    break;
			    case REQ_SetConfiguration: USB_handleSetConfiguration(req); break;
		    }
		} else EVENT_USB_Device_ControlRequest(req);  // Vendor defined request
		endpoints[0].out.STATUS &= ~(USB_EP_SETUP_bm | USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
	} else if(endpoints[0].out.STATUS & USB_EP_TRNCOMPL0_bm) {	// OUT transaction complete on endpoint 0
		endpoints[0].out.STATUS &= ~(USB_EP_TRNCOMPL0_bm | USB_EP_BUSNACK0_bm);
	} else if(endpoints[1].out.STATUS & USB_EP_TRNCOMPL0_bm) {  // OUT transaction complete on endpoint 1
    	endpoints[1].out.CNT = 0;
	    endpoints[1].out.AUXDATA = APP_SECTION_PAGE_SIZE;
		endpoints[1].out.STATUS &= ~(USB_EP_BUSNACK0_bm | USB_EP_BUSNACK1_bm | USB_EP_TRNCOMPL0_bm | USB_EP_TRNCOMPL1_bm);
	}
	USB.FIFORP=0;   // Workaround to clear TRINF flag
	USB.INTFLAGSBCLR=USB_SETUPIF_bm|USB_TRNIF_bm;
	OFFRED();       // Turn off red LED
}

// Event handler for the library USB Control Request reception event.
static inline void EVENT_USB_Device_ControlRequest(struct USB_Request_Header* req) {
	uint8_t *p;
	uint8_t i=0;
	uint16_t	addr;
	addr = *(uint16_t *)(ep0_buf_out+1);
	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_VENDOR) {
		switch(req->bRequest) {
		    case CMD_REQ_INFO:                          // Send device's info
		        memcpy_P(ep0_buf_in, FW_Info, 16);      // Firmware information
		        memcpy_P(&ep0_buf_in[16], HW_Info, 16); // Hardware information
		        ep0_buf_in[32] = MCU.DEVID0,            // XMEGA Device ID
		        ep0_buf_in[33] = MCU.DEVID1,            // XMEGA Device ID
		        ep0_buf_in[34] = MCU.DEVID2,            // XMEGA Device ID
		        ep0_buf_in[35] = MCU.REVID,             // XMEGA Revision ID
		        *(uint16_t *)(&ep0_buf_in[36]) = APP_SECTION_PAGE_SIZE,     // Flash page size
		        *(uint32_t *)(&ep0_buf_in[40]) = APP_SECTION_SIZE,          // Flash size
		        *(uint16_t *)(&ep0_buf_in[42]) = EEPROM_PAGE_SIZE,          // EEPROM page size
		        *(uint32_t *)(&ep0_buf_in[46]) = EEPROM_SIZE;               // EEPROM size
                USB_ep0_in_start(48);   // Sending 48 bytes
		        return;
		    case CMD_RESET_POINTER: // write to RAM page buffer
		        page_ptr = 0;
		    break;
		    case CMD_ERASE_APP_SECTION: // erase entire application section
		        SP_WaitForSPM();
		        SP_EraseApplicationSection();
            break;
		    case CMD_REQ_CRC_APP:   // calculate application and bootloader section CRCs
		        SP_WaitForSPM();
				*(uint32_t*)ep0_buf_in = SP_ApplicationCRC();
				USB_ep0_in_start(sizeof(uint32_t));
				return true;
/*          case REQ_START_WRITE:
				page = req->wIndex;
				pageOffs = 0;
				USB_ep_start_bank(1, 0, pageBuf, 0);
			break;
            case REQ_CRC_BOOT:
				*(uint32_t*)ep0_buf_in = SP_BootCRC();
				USB_ep0_in_start(sizeof(uint32_t));
				return;*/
		    case CMD_READ_FUSES:    // read fuses
		        ep0_buf_in[0] = SP_ReadFuseByte(0);
		        ep0_buf_in[1] = SP_ReadFuseByte(1);
		        ep0_buf_in[2] = SP_ReadFuseByte(2);
		        ep0_buf_in[3] = 0xFF;
		        ep0_buf_in[4] = SP_ReadFuseByte(4);
		        ep0_buf_in[5] = SP_ReadFuseByte(5);
                USB_ep0_in_start(6);   // Sending 48 bytes
		        return;
		    case CMD_WRITE_PAGE:    // write RAM page buffer to application section page
		        if (addr > (APP_SECTION_SIZE / APP_SECTION_PAGE_SIZE)) { // out of range
    		        ep0_buf_in[1] = 0xFF;
    		        ep0_buf_in[2] = 0xFF;
                    USB_ep0_in_start(2);   // Sending 2 bytes
    		        return;
		        }
		        SP_WaitForSPM();
		        SP_LoadFlashPage(page_buffer);
		        SP_WriteApplicationPage(APP_SECTION_START + ((uint32_t)addr * APP_SECTION_PAGE_SIZE));
		    break;
		    case CMD_READ_PAGE: // read application page to RAM buffer and return first 32 bytes
		        if (addr > (APP_SECTION_SIZE / APP_SECTION_PAGE_SIZE)) { // out of range
    		        ep0_buf_in[1] = 0xFF;
    		        ep0_buf_in[2] = 0xFF;
                    USB_ep0_in_start(2);   // Sending 2 bytes
                    return;
		        }
		        else {
    		        memcpy_P(page_buffer, (const void *)((APP_SECTION_START) + (APP_SECTION_PAGE_SIZE * addr)), APP_SECTION_PAGE_SIZE);
    		        memcpy(&ep0_buf_in[3], page_buffer, 32);
    		        page_ptr = 0;
		        }
		    break;
		    case CMD_ERASE_USER_SIG_ROW:    // erase user signature row
		        SP_WaitForSPM();
		        SP_EraseUserSignatureRow();
		    break;
		    case CMD_WRITE_USER_SIG_ROW:    // write RAM buffer to user signature row
		        SP_WaitForSPM();
		        SP_LoadFlashPage(page_buffer);
		        SP_WriteUserSignatureRow();
		    break;
		    case CMD_READ_USER_SIG_ROW: // read user signature row to RAM buffer and return first 32 bytes
		        if (addr > (USER_SIGNATURES_PAGE_SIZE - 32)) {
    		        ep0_buf_in[1] = 0xFF;
    		        ep0_buf_in[2] = 0xFF;
                    USB_ep0_in_start(2);   // Sending 2 bytes
                    return;
		        }
		        else {
    		        memcpy_P(page_buffer, (const void *)(USER_SIGNATURES_SIZE + addr), USER_SIGNATURES_SIZE);
    		        memcpy(&ep0_buf_in[3], page_buffer, 32);
    		        page_ptr = 0;
		        }
	        break;
		    case CMD_READ_SERIAL:
            {
    		    uint8_t	j = 3;
    		    uint8_t b;
    		    for (uint8_t i = 0; i < 6; i++) {
        		    b = SP_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0) + i);
        		    ep0_buf_in[j++] = hex_to_char(b >> 4);
        		    ep0_buf_in[j++] = hex_to_char(b & 0x0F);
    		    }
    		    ep0_buf_in[j++] = '-';
    		    b = SP_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0) + 6);
    		    ep0_buf_in[j++] = hex_to_char(b >> 4);
    		    ep0_buf_in[j++] = hex_to_char(b & 0x0F);
    		    ep0_buf_in[j++] = '-';
    		    for (uint8_t i = 7; i < 11; i++) {
        		    b = SP_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0) + i);
        		    ep0_buf_in[j++] = hex_to_char(b >> 4);
        		    ep0_buf_in[j++] = hex_to_char(b & 0x0F);
    		    }
    		    ep0_buf_in[j] = '\0';
            }                
   		    break;
			case CMD_RESET_DEVICE: // disconnect from USB, reset
    			USB_ep0_in_start(0);
			    USB_ep0_wait_for_complete();
                cli();
                delay_ms(10);
                USB.CTRLB &= ~USB_ATTACH_bm;    // disconnects the device from the USB lines
                delay_ms(100);
                CCPWrite(&RST.CTRL, RST_SWRST_bm);  // Software Reset!                
            break;
		}
        USB_ep0_in_start(0);
	}
}
