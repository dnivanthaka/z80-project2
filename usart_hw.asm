     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>
     
     #define _EI_ bsf INTCON, 7 ; GIE
    #define _DI_ bcf INTCON, 7 ; GIE 
	
    UDATA
	
	temp  RES 1
	temp2 RES 1
	temp3 RES 1
	usart_rxline RES 17 ;last byte contains the length
	rxcount RES 1
 
    CODE
usart_init:
    bcf TRISC, TRISC6            ; tx pin
    bsf TRISC, TRISC7            ; rx pin
    
    clrf TXREG1
    clrf RCREG1
    ;bsf PIR1, TXIF
    
    movlw 0xCF			    ;19200 @ 64Mhz
    ;movlw 0x81                     ; 19200 @ 40Mhz
    ;movlw 0x6A
    ;movlw 0x40                   ; 19200
    ;movlw 0x1F                    ; 19200
    movwf SPBRG1
    
    movlw 0x24
    movwf TXSTA1
    
    bcf TXSTA1, SYNC
    bsf RCSTA1, CREN
    bsf RCSTA1, SPEN
    
    
    ;bsf PIE1, TXIE
    bsf PIE1, RC1IE
    
    return
;--------------------------------------------------------------------    
usart_putchar:
    ;movwf UTXDATA
    ;INTCON, GIE
    
    btfss PIR1, TX1IF
    bra usart_putchar
    
    movwf TXREG1
    return
;--------------------------------------------------------------------    
usart_getchar:
    btfss PIR1, RC1IF
    bra usart_getchar
    
    movf RCREG1, w
    return
    
;--------------------------------------------------------------------
;usart_putstr:
;    movwf	FSR
;    
;outmessage:
;    movf FSR, w
;    incf FSR, f
;    call getmessages
;    xorlw 0
;    btfsc STATUS, Z
;    return
;    call TXPoll
;    goto outmessage
;    return
;--------------------------------------------------------------------    
usart_newline:
    movlw 0x0D
    call usart_putchar
    movlw 0x0A
    call usart_putchar
    return
;--------------------------------------------------------------------
usart_getmessages:
	addwf	PCL, f
	;dt	"Test Message", 0
	;dt	"Test Message2", 0
	dt "-------------------------------------------", 0x0D, 0x0A
	dt "Test Message", 0x0D, 0x0A
	dt "-------------------------------------------", 0
;--------------------------------------------------------------------
usart_hex2ascii_nb:
	movwf	temp
	btfss	temp, 3
	goto	ztn
	xorlw	0x08
	btfsc	STATUS, Z
	goto	ztn
	nop
	movf	temp, w
	xorlw	0x09
	btfsc	STATUS, Z
	goto	ztn
	nop
	movlw	0x07
	addwf	temp, f
ztn:
	movf	temp, w
	addlw	0x30
	call	usart_putchar
	return
;--------------------------------------------------------------------	
usart_hex2ascii:
    ;High byte first
    movwf temp2
    swapf temp2, w
    andlw 0x0f
    call usart_hex2ascii_nb
    movf temp2, w
    andlw 0x0f
    call usart_hex2ascii_nb
    
    return
    
;--------------------------------------------------------------------
usart_readline:
    ;movlw .15
    _DI_
    clrf FSR0H
    clrf rxcount
    movlw low usart_rxline
    movwf FSR0L
    movlw high usart_rxline
    movwf FSR0H
    
readline_loop:    
    call usart_getchar
    movwf temp3
    
    ;movf temp3, w
    xorlw '\n'
    bz readline_done
    
    movf temp3, w
    xorlw '\r'
    bz readline_done
    
    movf temp3, w
    xorlw '\0'
    bz readline_done
    
    ;echo to terminal
    movf temp3, w
    call usart_putchar
    
    ;movf rxcount, w
    ;addwf FSR0L, f
    
    movf temp3, w
    movwf INDF0
    incf rxcount, f
    incf FSR0L, f
    movf rxcount, w
    xorlw .16
    bnz readline_loop
    
readline_done:
    movf rxcount, w
    movwf usart_rxline+16
    _EI_
    
    return
;--------------------------------------------------------------------
;TODO need to add bounds checking
usart_ascii2hex:
    movwf temp3
    movlw '0'
    subwf temp3, w
    ;bz ascii2hex_done
    
ascii2hex_done:    
    return
;--------------------------------------------------------------------
GLOBAL usart_init
GLOBAL usart_putchar
GLOBAL usart_getchar
GLOBAL usart_newline
GLOBAL usart_getmessages
GLOBAL usart_hex2ascii
GLOBAL usart_ascii2hex
GLOBAL usart_readline
GLOBAL usart_rxline
	
    END

