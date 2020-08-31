     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>         ; processor specific variable definitions
	
	UDATA
	addressbus_val RES 2
 
	CODE

addressbus_init:
    movlw 0x01
    call addressbusmode_set
    
    return
;--------------------------------------------------------------------
addressbusmode_set:
    bcf STATUS, Z
    xorlw 0x01
    btfss STATUS, Z
    bra _ab_mode_output
    
_ab_mode_input:
    movlw 0xff
    movwf TRISA
    bsf TRISC, 0
    
    return
_ab_mode_output: 
    clrf TRISA
    bcf TRISC, 0
    
    return

;----------------------------------------------------------------    
addressbus_read:
    clrf addressbus_val
    clrf addressbus_val+1
    
    ;low 8 bits    
    movf PORTA, w
    movwf addressbus_val
    
    ;high 1 bit(s)
    btfsc PORTC, 0
    bsf addressbus_val+1, 0
    
    return
;----------------------------------------------------------------    
addressbus_write:
    ;low 8 bits
    movf addressbus_val, w
    movwf LATA
    
    ;high 1 bit(s)
    movf addressbus_val+1, w
    andlw 0x01
    bcf LATC, 0
    btfss STATUS, Z
    bsf LATC, 0
    
    return
;---------------------------------------------------------------    
GLOBAL addressbus_val
GLOBAL addressbus_init
GLOBAL addressbusmode_set
GLOBAL addressbus_read
GLOBAL addressbus_write   
    
	END


