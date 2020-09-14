     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>
UDATA
	
	TXDATA RES 1
 
CODE	
;-------------------------------------------------------------    
ssp_init:
    bsf TRISC, TRISC4            ; SDI
    bcf TRISC, TRISC5            ; SDO
    bcf TRISC, TRISC3            ; SCK
    ;bcf TRISA, TRISA5            ; SS as input
    
    clrf SSP1CON1
    ;clrf SSP1STAT
    clrf SSP1BUF
    bcf PIR1, SSP1IF
    
    movlw 0x03                     ;TMR2/2 drives the clock
    movwf SSP1CON1
    
    ;movlw 0x3F
    ;andwf SSPSTAT, f
    
    ;movlw 0x40
    ;movlw 0
    ;movwf SSPSTAT
    bcf	SSP1STAT, CKE
    bcf	SSP1STAT, SMP
    
    ;bsf SSP1CON1, CKP
    
    ;movlw 0x11
    ;movlw 0x20
    ;movwf SSPCON1
    
;    movlw 0x27
;    movwf SSP1ADD
    
    ;movlw 0x20
    ;iorwf SSPCON1, f
    bsf SSP1CON1, CKP
    ;bsf SSP1CON1, 1 ;FoSC/64
    ;bsf SSP1CON1, 0 ; FoSC/16
    bsf SSP1CON1, SSPEN           ; SSPEN
    
    return
;-------------------------------------------------------------
ssp_write:
    movwf TXDATA
    ;bcf PIR1, SSPIF
    bcf SSP1CON1, WCOL
    ;bcf SSPCON1, SSPOV
    
    ;movf SSPBUF, w
    ;movwf RXDATA
    
    ;bcf T2CON, TMR2ON
    ;clrf TMR2
    
    movf TXDATA, w
    movwf SSP1BUF
    ;BSF T2CON, TMR2ON
    
write_complete:
    btfss PIR1, SSP1IF
    bra write_complete
    
    bcf PIR1, SSP1IF
    
    movf SSP1BUF, w
    
    return
;------------------------------------------------------------
ssp_read:
    bcf PIR1, SSP1IF
    ;bcf SSPCON1, SSPOV
    
    movf SSP1BUF, w
    ;movwf RXDATA
    
    ;bcf T2CON, TMR2ON
    ;clrf TMR2
    
    movlw 0
    movwf SSP1BUF
    ;BSF T2CON, TMR2ON
    
read_complete:
    btfss PIR1, SSP1IF
    bra read_complete
    
    movf SSP1BUF, w
    
    return
;------------------------------------------------------------
    
GLOBAL ssp_init
GLOBAL ssp_write
GLOBAL ssp_read
    
END