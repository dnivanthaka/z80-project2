	LIST P=PIC18F46K22		;directive to define processor
	#include <p18f46k22.inc>	;processor specific variable definitions
UDATA
	
	DELAY1_TEMP RES 1
	DELAY2_TEMP RES 1
 
CODE	
 
 delay_us:                           ; 40Mhz / 10 = 10Mhz clock = 0.1us (100ns) instruction cycle
    movwf DELAY1_TEMP
    
dm_loop:
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    ;nop
    decfsz DELAY1_TEMP, f
    bra dm_loop
    
    return
    
delay_millis:
    ; delay upto 255 milliseconds
    movwf DELAY2_TEMP
    
dmi_loop:
    movlw .100
    call delay_us
    movlw .100
    call delay_us
    movlw .100
    call delay_us
    movlw .100
    call delay_us
    
    decfsz DELAY2_TEMP
    goto dmi_loop
    
    return
;------------------------------------------------------------------------
GLOBAL delay_us
GLOBAL delay_millis
   
END
    


