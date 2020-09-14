     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>
     
     
;--------------------------------------------------------
ds12887_init:
    movlw DS12887REGB
    movwf ds12887_reg

    call ds12887_read
    movwf ds12887_regB

    bsf ds12887_regB, 2

    movf ds12887_regB, w
    movwf ds12887_reg
    call ds12887_write
    return
    
;--------------------------------------------------------
ds12887_read:
    ;ds12887_reg and ds12887_val should be set
    ;set latch AS pin as high
    bsf SRAM_OE_LAT, SRAM_OE_PIN
    bsf Z80_IOREQ_LAT, Z80_IOREQ_PIN
    
    clrf addressbus_val+1
    bcf addressbus_val+1, 0
    movlw 0x02                   ;address of ds12887
    movwf addressbus_val
    call addressbus_write
    
    nop
    nop
    nop
    
    clrf addressbus_val+1
    bsf addressbus_val+1, 0
    movlw 0x02                   ;address of ds12887
    movwf addressbus_val
    call addressbus_write
    
    nop
    nop
    nop
    nop
    nop
    
    
    clrf addressbus_val+1
    bcf addressbus_val+1, 0
    movlw 0x02                   ;address of ds12887
    movwf addressbus_val
    
    bcf Z80_IOREQ_LAT, Z80_IOREQ_PIN
    
    ;register
    movf ds12887_reg, w
    call databus_write
    call addressbus_write
    
    nop
    nop
    nop
    nop
    nop
    
    bcf SRAM_OE_LAT, SRAM_OE_PIN
    
    movlw 0x01
    call databusmode_set
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop

    call databus_read
    
    bsf SRAM_OE_LAT, SRAM_OE_PIN
    
    bsf Z80_IOREQ_LAT, Z80_IOREQ_PIN
    
    movwf ds12887_val
    ;call usart_hex2ascii
    
    movlw 0x00
    call databusmode_set
    
    movf ds12887_val, w
    
    return
;--------------------------------------------------------------


