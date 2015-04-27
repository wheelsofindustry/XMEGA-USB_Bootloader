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

// Misc             (GPIOC) // Miscellaneous bits
#define keyrep      0       // Automatic key repeat
#define negative    1       // Print Negative font
#define bigfont     2       // Small or large font, this bit is normally 0
#define redraw      3       // Redraw screen
#define sacquired   4       // Data has been acquired (for slow sampling)
#define slowacq     5       // Acquired one set of samples a slow sampling rates
#define lowbatt     6       // VCC is below 3.15V
#define autosend    7       // Continuously send data to UART


extern uint8_t *Outgoing;
extern uint8_t *Incoming;

uint8_t ReadCalibrationByte(uint8_t location);	// Read out calibration byte.
void CCPWrite( volatile uint8_t * address, uint8_t value );

