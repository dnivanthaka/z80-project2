     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>         ; processor specific variable definitions
    
#define         SD_CS_PIN            1
#define         SD_CS_LAT            LATB
#define         SD_CS_TRIS           TRISB
  
#define         SD_TRIES_MAX         40
#define		SD_VALID             0
#define         SD_TIMEOUT           1
#define         SD_TYPE              2 
     
    EXTERN delay_us
    EXTERN delay_millis 
    
    EXTERN ssp_init
    EXTERN ssp_read
    EXTERN ssp_write
    
    EXTERN usart_init
    EXTERN usart_putchar
    EXTERN usart_putstr
    EXTERN usart_getchar
    EXTERN usart_newline
    EXTERN usart_getmessages
    EXTERN usart_hex2ascii
    EXTERN usart_ascii2hex
    EXTERN usart_readline
    EXTERN usart_rxline
    
    
    UDATA
    
    sd_temp	    RES 1
    sd_data	    RES 6	    ; MSB sd_data+1, LSB sd_data+5
    sd_cmd          RES 1
    sd_tries_count  RES 1
    sd_byte_count   RES 2
    sd_status	    RES 1   
    sd_block_buffer RES 2
     
    CODE
    
sd_init:
    ;ssp_init should be called before calling this
    ;sd card module cs
;    bsf SD_CS_LAT, SD_CS_PIN
;    bcf SD_CS_TRIS, SD_CS_PIN
    
;    bcf SSP1CON1, SSPEN
;    
;    ;setting slow speed spi
;    movlw .32                           ; 250Khz
;    movwf PR2
;    ;    ;Timer2 setup
;    movlw b'00000100'                 ; no prescalar, 1:4 postscalar, 1:16 prescalar
;    movwf T2CON
;    
;    movlw 0x03                     ;TMR2/2 drives the clock
;    movwf SSP1CON1
;    
;    bsf SSP1CON1, CKP
;    
;    bsf SSP1CON1, SSPEN
    
    movlw 0x10
    call delay_millis
    
    clrf sd_status
    clrf sd_tries_count
    
    bsf SD_CS_LAT, SD_CS_PIN
    
    movlw .10
    movwf sd_temp
    
    ;send at least 10 bytes for initialization, min 74 clock pulses
_sd_clk_pulse:
    movlw 0xff
    call ssp_write
    
    decfsz sd_temp, f
    bra _sd_clk_pulse
    
    ;call ssp_read
    
    ;select sd
    bcf SD_CS_LAT, SD_CS_PIN
    
;    call ssp_read
    
    ;send CMD0, SPI mode enable
    movlw 0x00
    movwf sd_cmd
    
    clrf sd_data+3
    clrf sd_data+2
    clrf sd_data+1
    clrf sd_data
    
    call sd_send_command
    ;call write_sd
    
    ;toggle clk wait for response
_sd_wait_ack:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
    incf sd_tries_count, f
    movlw SD_TRIES_MAX
    xorwf sd_tries_count, w
    btfsc STATUS, Z
    goto sd_timeout
    
    movlw 0x01
    xorwf sd_temp, w
    bnz _sd_wait_ack
    
    bsf SD_CS_LAT, SD_CS_PIN
;    
    nop
    clrf sd_tries_count
;    
    bcf SD_CS_LAT, SD_CS_PIN
    
    ;sending CMD8
    movlw 0x08
    movwf sd_cmd
    
    clrf sd_data+3
    clrf sd_data+2
    movlw 0x01
    movwf sd_data+1
    movlw 0xAA
    movwf sd_data                       ;MSB
    
    call sd_send_command
    ;call write_sd
    
_sd_wait_ack2:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
    incf sd_tries_count, f
    movlw SD_TRIES_MAX
    xorwf sd_tries_count, w
    btfsc STATUS, Z
    goto sd_timeout
    
    movlw 0x05                             ; invalid command meaning this is a ver.1 SD
    xorwf sd_temp, w
    bz _sd_ver1_handle
    
    movlw 0x01
    xorwf sd_temp, w
    bnz _sd_wait_ack2
    
    call ssp_read
    call ssp_read
    call ssp_read			    ;mask lower 4bits
    andlw 0x0f
    xorlw 0x01
    bnz _sd_init_error

    call ssp_read
    xorlw 0xAA
    bnz _sd_init_error
    ;call ssp_read
    ;call ssp_read
    ;call ssp_read
    
    ;bsf SD_CS_LAT, SD_CS_PIN
    
    ;nop
    clrf sd_tries_count
    
    ;call ssp_read
    ;bcf SD_CS_LAT, SD_CS_PIN
    ;call ssp_read
_sd_send_cmd55:    
    clrf sd_tries_count
    ;sending acmd41
    movlw 0x55
    movwf sd_cmd
    
    ;movlw   0x40
    clrf sd_data+3
    clrf sd_data+2
    clrf sd_data+1
    clrf sd_data                       ;MSB
    
    call sd_send_command
    ;call write_sd
    
_sd_wait_ack3:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
;    movf sd_temp, w
;    call usart_hex2ascii
    
    movlw 0xff
    xorwf sd_temp, w
    bz _sd_wait_ack3
    
    movlw 0x01
    xorwf sd_temp, w
    bz _sd_send_cmd55
 
clrf sd_tries_count    
    
_sd_send_cmd41:    
    clrf sd_tries_count
    ;sending acmd41
    movlw 0x41
    movwf sd_cmd
    
    movlw   0x40
    movwf sd_data+3
    clrf sd_data+2
    clrf sd_data+1
    clrf sd_data                       ;MSB
    
    call sd_send_command
    ;call write_sd
    
_sd_wait_ack4:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
;    movf sd_temp, w
;    call usart_hex2ascii
    
    movlw 0xff
    xorwf sd_temp, w
    bz _sd_wait_ack4
    
    movlw 0x01
    xorwf sd_temp, w
    bz _sd_send_cmd55
    
;    call ssp_read
;    call ssp_read
;    call ssp_read
;    call ssp_read
    
;        movf sd_temp, w
;    call usart_hex2ascii
    
;    movlw 'D'
;    call usart_putchar
    
;    bsf SD_CS_LAT, SD_CS_PIN
;    
;    ;nop
;    clrf sd_tries_count
;    
;    ;call ssp_read
;    bcf SD_CS_LAT, SD_CS_PIN
    
_sd_send_cmd58:    
    clrf sd_tries_count
    ;sending acmd41
    movlw .58
    movwf sd_cmd
    
    ;movlw   0x40
    clrf sd_data+3
    clrf sd_data+2
    clrf sd_data+1
    clrf sd_data                       ;MSB
    
    call sd_send_command
    ;call write_sd
    
_sd_wait_cmd58:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
    movlw 0xff
    xorwf sd_temp, w
    bz _sd_wait_cmd58
    
    call ssp_read
    movwf sd_temp
    call ssp_read
    call ssp_read
    call ssp_read
    
    movf sd_temp, w
    call usart_hex2ascii
    
    ;call usart_hex2ascii
    
    
    bsf sd_status, SD_VALID
    bsf sd_status, SD_TYPE                 ; v2
    
    ;set high speed spi
    bcf SSP1CON1, SSPEN
    
    movlw 0x01                     ;FoSC/16
    movwf SSP1CON1
    
    bsf SSP1CON1, CKP
    
    bsf SSP1CON1, SSPEN
    
    clrf T2CON                    ;stop tmr2
    clrf PR2
    ;----------
    
    bra init_done
    
_sd_ver1_handle:
    movlw 'V'
    call usart_putchar
    bra init_done
    
sd_timeout:
    movlw 'M'
    call usart_putchar
    
    bsf sd_status, SD_TIMEOUT
    bcf sd_status, SD_VALID
    
    bra init_done
    
_sd_init_error:
    bcf sd_status, SD_VALID
    
    movlw 'E'
    call usart_putchar
    
init_done:
;    movlw 'D'
;    call usart_putchar
    bsf SD_CS_LAT, SD_CS_PIN
    call ssp_read
    
    return
;----------------------------------------------------------
sd_read_block:
    clrf sd_tries_count
    clrf FSR0H
    clrf FSR0L
    
    clrf sd_byte_count			      ; 512 bytes + 2 checksum bytes
    movlw 0x03
    movwf sd_byte_count+1
    
    call usart_newline
    
;    movlw low sd_block_buffer
;    movwf FSR0L
;    
;    movlw high sd_block_buffer
;    movwf FSR0H
    
    ;call ssp_read
    ;select sd
    bcf SD_CS_LAT, SD_CS_PIN
    ;call ssp_read
    ;valid address should be set on sd_data+1 to sd_data+3
_sd_send_cmd17:
    movlw 0x51			;CMD 17
    movwf sd_cmd
    
    ;movlw 0xbe
    ;movlw 0xfe
    ;movlw 0x01
    setf sd_data
    movlw 0x01
    ;movlw 0x90
    ;movlw 0xbe
    movwf sd_data+1
    clrf sd_data+2
    clrf sd_data+3
    clrf sd_data+4
    
    call sd_send_command
    
_sd_wait_rd:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
;    incf sd_tries_count, f
;    movlw .255
;    xorwf sd_tries_count, w
;    bz sd_read_timeout
    
;    movlw .5
;    call delay_millis
;    movf sd_temp, w
;    call usart_hex2ascii
    
;    movlw 0x05
;    xorwf sd_temp, w
;    bz _sd_send_cmd17
    
;    movf sd_temp, w
;    call usart_hex2ascii
    
    movlw 0xfe	
    xorwf sd_temp, w
    bnz _sd_wait_rd
    
;    movlw 0xff
;    call ssp_write
;    call usart_hex2ascii
    
   ; movlw 0x01
;    clrf sd_byte_count			      ; 512 bytes + 2 checksum bytes
;    movlw 0x03
;    movwf sd_byte_count+1
;    
;    call usart_newline 
    
_sd_read_loop:
    ;movlw 0xff
    call ssp_read
    ;movwf sd_temp
    
    ;movf sd_temp, w
    call usart_hex2ascii
    movlw ' '
    call usart_putchar
    
    ;movwf INDF0
    ;incf FSR0, f
    incfsz sd_byte_count, f
    bra _sd_read_loop
    decfsz sd_byte_count+1, f
    bra _sd_read_loop
    
    call ssp_read			    ; 2 checksum bytes
    call usart_hex2ascii
    call ssp_read
    call usart_hex2ascii
    
;    movf sd_temp, w
;    call usart_hex2ascii
    
    call usart_newline
;    
;    ;movlw sd_block_buffer+511
;    call usart_newline
;    movf sd_temp, w
;    call usart_hex2ascii
;    call usart_newline
    
    bra sd_read_done
    
sd_read_timeout:
    movlw 'T'
    call usart_putchar
    
sd_read_done:    
    ;call ssp_read
    ;deselect sd
    bsf SD_CS_LAT, SD_CS_PIN
    ;call ssp_read
    
    return
;----------------------------------------------------------
    
sd_send_command:
   ; call ssp_read
    ; sd_data array should be filled
    ;command
    movlw 0x40
    iorwf sd_cmd, f
    movf sd_cmd, w
    call ssp_write
;    nop
;    nop
;    nop
    
    ;args
    movf sd_data+3, w
    call ssp_write
;    nop
;    nop
;    nop
    movf sd_data+2, w
    call ssp_write
;    nop
;    nop
;    nop
    movf sd_data+1, w
    call ssp_write
;    nop
;    nop
;    nop
    movf sd_data, w
    call ssp_write
;    nop
;    nop
;    nop
    
    ;crc
    call sd_get_crc
    iorlw 0x01
    call ssp_write
;    nop
;    nop
;    nop
    ;call ssp_read
;    call usart_newline
;    movf sd_data, w
;    call usart_hex2ascii
;    movf sd_data+1, w
;    call usart_hex2ascii
;    movf sd_data+2, w
;    call usart_hex2ascii
;    movf sd_data+3, w
;    call usart_hex2ascii
;    movf sd_data+4, w
;    call usart_hex2ascii
;    call sd_get_crc
;    call usart_hex2ascii
;    call usart_newline
    
    
    return
;---------------------------------------------------
sd_get_crc:
    
    movf sd_cmd, w
    xorlw 0x40
    btfsc STATUS, Z
    retlw 0x95
    movf sd_cmd, w
    xorlw 0x48
    btfsc STATUS, Z
    retlw 0x87
    movf sd_cmd, w
    xorlw 0x58
    btfsc STATUS, Z
    retlw 0xff
    movf sd_cmd, w
    xorlw 0x55
    btfsc STATUS, Z
    retlw 0x65
    movf sd_cmd, w
    xorlw 0x41
    btfsc STATUS, Z
    retlw 0x77
    retlw 0x01                      ; default crc
    
    return
;----------------------------------------------------
    ;-------------------------------------------------------------------------------
; write_sd - internal mid-level write a command to SD
;-------------------------------------------------------------------------------
write_sd
    ; test for an ACMD41
    btfsc       sd_cmd, 7
    call        sd_cmd55

    call        ssp_read
    movf        sd_cmd, w
    andlw       0x7F
    call        ssp_write

    ; now the 32 bit data
    movf        sd_data+3, w
    call        ssp_write
    movf        sd_data+2, w
    call        ssp_write
    movf        sd_data+1, w
    call        ssp_write
    movf        sd_data, w
    call        ssp_write

    ; now the checksum, might need to be a real one
    call        sd_get_checksum
    call        ssp_write

    return

; send the special command 55 which preceeds ACMD codes
sd_cmd55
    call        ssp_read
    movlw       0x77
    call        ssp_write
    movlw       0x00
    call        ssp_write
    movlw       0x00
    call        ssp_write
    movlw       0x00
    call        ssp_write
    movlw       0x00
    call        ssp_write
    movlw       0x01
    call        ssp_write

    call        ssp_read
    andlw       0x7e
    btfss       STATUS, Z
    goto        error_exit                      ; if it was not zero an error
                                                ; flag was set
    return
    
error_exit:
    goto $
 sd_get_checksum
    movlw       0x40            ; cmd0
    xorwf       sd_cmd, w
    btfsc       STATUS, Z
    retlw       0x95
    movlw       0x48            ; cmd8
    xorwf       sd_cmd, w
    btfsc       STATUS, Z
    retlw       0x87
    ; neither of those, so don't care
    retlw       0x01   
;--------------------------------------------------------

GLOBAL sd_data    
GLOBAL sd_init
GLOBAL sd_read_block
    
    END


