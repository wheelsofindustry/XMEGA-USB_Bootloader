; XMEGA USB Bootloader
;
; Gabotronics
; February 2014
;
; Copyright 2014 Gabriel Anzziani
;
; This program is distributed under the terms of the GNU General Public License
;
; www.gabotronics.com
; email me at: gabriel@gabotronics.com

#include <avr/io.h>

.section .bss
.global endpoints   // Endpoints configuration tables
.align 2            // Align to word
endpoints:  .dc.b 1,2,3,4,5,6,7,8    ; ep0 out
            .dc.b 1,2,3,4,5,6,7,8    ; ep0 in
            .dc.b 1,2,3,4,5,6,7,8    ; ep1 out
            .dc.b 1,2,3,4,5,6,7,8    ; ep1 in