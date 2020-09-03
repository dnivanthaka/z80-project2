     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>

#define		SERIAL_2_PORT		PORTA	
#define		SERIAL_2_TRIS		TRISA
#define         SERIAL_2_PIN            3
	
SERIAL_2_HI MACRO
	    bsf     SERIAL_2_PORT, SERIAL_2_PIN
	    ENDM
		    
SERIAL_2_LO MACRO
	    bcf     SERIAL_2_PORT, SERIAL_2_PIN
	    ENDM 	

EXTERN delay_us
	    
    UDATA
	
	tdata   RES 1
	counter RES 2
 
    CODE
;----------------------------------------------------------------------
;Software serial TX, 8N1 format
usart_sw_putchar:                           ;9600bps
        movwf   tdata
	movlw   .8                 ;8N1
	movwf   counter
	
	SERIAL_2_LO                ;start bit
	
	movlw   .55
	call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
;	movlw   .104
;	pagesel delay_us
;	call    delay_us
	
sw_serial_loop
	
	rrncf tdata, f
	btfss STATUS, C
	goto  sw_serial_zero
	goto  sw_serial_one
	
sw_serial_zero
	SERIAL_2_LO
	goto sw_serial_done
	
sw_serial_one
	SERIAL_2_HI

sw_serial_done	
	movlw   .55
	call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
;	movlw   .104
;	pagesel delay_us
;	call    delay_us
	
	decfsz counter, f
	goto sw_serial_loop
	
	;stop bit
	SERIAL_2_HI
	
	movlw   .55
	call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
	;movlw   .250
	;pagesel delay_us
	;call    delay_us
	
;	movlw   .104
;	pagesel delay_us
;	call    delay_us

	return
;----------------------------------------------------------------
usart_sw_init:
    SERIAL_2_HI
    
    bcf     SERIAL_2_TRIS, SERIAL_2_PIN
    return
    END


