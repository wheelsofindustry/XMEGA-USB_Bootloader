// USB Bootloader for XMEGA AU
// February 2014

#include "mygccdef.h"
#include "display.h"

// Global variables, using GPIO for optimized access
#define Display     GPIO7   // Display options
#define Misc        GPIOC   // Miscellaneous bits
#define Key         GPIOF   // Current key pressed

// Display bits     (GPIO7) // Display options
#define grid0       0       // Grid settings (2 bits)
#define grid1       1
#define elastic     2       // Average on successive traces
#define disp_inv    3       // Invert display
#define flip        4       // Flip display
#define persistent  5       // Persistent Display
#define line        6       // Continuous Drawing
#define showset     7       // Show scope settings (time/div volts/div)


uint8_t ReadCalibrationByte(uint8_t location);	// Read out calibration byte.
void CCPWrite( volatile uint8_t * address, uint8_t value );
void calc_fw_crcs(uint32_t *app_crc, uint32_t *boot_crc);
uint8_t hex_to_char(uint8_t hex);

extern const char FW_Info[];     // Firmware information