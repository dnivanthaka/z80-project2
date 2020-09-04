     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>         ; processor specific variable definitions
	
	UDATA
	db_temp RES 1
 
	CODE

;----------------------------------------------------------------------
databus_init:
    ; PSP mode disable
    ;bcf TRISE, 4
    ;Settingup pins as input
    movlw 0x01
    call databusmode_set
    
    return
    
;----------------------------------------------------------------------    
databusmode_set:
    bcf STATUS, Z
    xorlw 0x01
    ;btfss STATUS, Z
    bnz _db_mode_output
    
_db_mode_input:
    bsf TRISD, RD0
    bsf TRISD, RD1
    bsf TRISD, RD2
    bsf TRISD, RD3
    bsf TRISD, RD4
    bsf TRISD, RD5
    bsf TRISD, RD6
    bsf TRISD, RD7
    
    return
_db_mode_output:
    bcf TRISD, RD0
    bcf TRISD, RD1
    bcf TRISD, RD2
    bcf TRISD, RD3
    bcf TRISD, RD4
    bcf TRISD, RD5
    bcf TRISD, RD6
    bcf TRISD, RD7
    
    return
;----------------------------------------------------------------------
databus_write:
    movwf db_temp
    
    bcf LATD, LATD0
    bcf LATD, LATD1
    bcf LATD, LATD2
    bcf LATD, LATD3
    bcf LATD, LATD4
    bcf LATD, LATD5
    bcf LATD, LATD6
    bcf LATD, LATD7
    
    btfsc db_temp, 0
    bsf LATD, LATD0
    btfsc db_temp, 1
    bsf LATD, LATD1
    btfsc db_temp, 2
    bsf LATD, LATD2
    btfsc db_temp, 3
    bsf LATD, LATD3
    btfsc db_temp, 4
    bsf LATD, LATD4
    btfsc db_temp, 5
    bsf LATD, LATD5
    btfsc db_temp, 6
    bsf LATD, LATD6
    btfsc db_temp, 7
    bsf LATD, LATD7
    
    return
;----------------------------------------------------------------------
databus_read:
    clrf db_temp
    
    btfsc PORTD, RD0
    bsf db_temp, 0
    btfsc PORTD, RD1
    bsf db_temp, 1
    btfsc PORTD, RD2
    bsf db_temp, 2
    btfsc PORTD, RD3
    bsf db_temp, 3
    btfsc PORTD, RD4
    bsf db_temp, 4
    btfsc PORTD, RD5
    bsf db_temp, 5
    btfsc PORTD, RD6
    bsf db_temp, 6
    btfsc PORTD, RD7
    bsf db_temp, 7
    
    movf db_temp, w
    
    return
;----------------------------------------------------------------------    
databus_clear:
    clrf LATD
    return
;----------------------------------------------------------------------    

    GLOBAL databus_init
    GLOBAL databusmode_set
    GLOBAL databus_write
    GLOBAL databus_read
    GLOBAL databus_clear
    
	END


