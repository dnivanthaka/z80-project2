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
    sd_tries_count  RES 1
    sd_byte_count   RES 2
    sd_status	    RES 1
    sd_block_buffer RES 16
     
    CODE
    
sd_init:
    ;ssp_init should be called before calling this
    ;sd card module cs
    bsf SD_CS_LAT, SD_CS_PIN
    bcf SD_CS_TRIS, SD_CS_PIN
    
    bcf SSP1CON1, SSPEN
    
    ;setting slow speed spi
    movlw .32                           ; 250Khz
    movwf PR2
    ;    ;Timer2 setup
    movlw b'00000100'                 ; no prescalar, 1:4 postscalar, 1:16 prescalar
    movwf T2CON
    
    movlw 0x03                     ;TMR2/2 drives the clock
    movwf SSP1CON1
    
    bsf SSP1CON1, CKP
    
    bsf SSP1CON1, SSPEN
    
    movlw .1
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
    
    ;select sd
    bcf SD_CS_LAT, SD_CS_PIN
    
    ;send CMD0, SPI mode enable
    movlw 0x00
    movwf sd_data
    movwf sd_data+1
    movwf sd_data+2
    movwf sd_data+3
    movwf sd_data+4
    
    call sd_send_command
    
    ;toggle clk wait for response
_sd_wait_ack:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
    incf sd_tries_count, f
    movlw SD_TRIES_MAX
    xorwf sd_tries_count, w
    bz sd_timeout
    
    movlw 0x01
    xorwf sd_temp, w
    bnz _sd_wait_ack
    
    bsf SD_CS_LAT, SD_CS_PIN
    
    nop
    clrf sd_tries_count
    
    bcf SD_CS_LAT, SD_CS_PIN
    
    ;sending CMD8
    movlw 0x08
    movwf sd_data
    clrf sd_data+1
    clrf sd_data+2
    movlw 0x01
    movwf sd_data+3
    movlw 0xAA
    movwf sd_data+4
    
    call sd_send_command
    
_sd_wait_ack2:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
    incf sd_tries_count, f
    movlw SD_TRIES_MAX
    xorwf sd_tries_count, w
    bz sd_timeout
    
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
    
    clrf sd_tries_count
    
    ;ACMD41
    movlw 0xE9
    movwf sd_data
    
    movlw 0x40
    movwf sd_data+1
    clrf sd_data+2
    clrf sd_data+3
    clrf sd_data+4
    
    call sd_send_command
    
_sd_wait_ack3:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
    incf sd_tries_count, f
    movlw SD_TRIES_MAX
    xorwf sd_tries_count, w
    bz sd_timeout
    
    movlw 0x01
    xorwf sd_temp, w
    bz _sd_wait_ack3
    
    movlw 'D'
    call usart_putchar
    
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
    
    bra init_done
    
sd_timeout:
    movlw 'T'
    call usart_putchar
    
    bsf sd_status, SD_TIMEOUT
    bcf sd_status, SD_VALID
    
    bra init_done
    
_sd_init_error:
    bcf sd_status, SD_VALID
    
    movlw 'E'
    call usart_putchar
    
init_done:
    bsf SD_CS_LAT, SD_CS_PIN
    
    return
;----------------------------------------------------------
sd_read_block:
    clrf sd_tries_count
    clrf FSR0H
    clrf FSR0L
    
    movlw low sd_block_buffer
    movwf FSR0L
    
    movlw high sd_block_buffer
    movwf FSR0H
    ;select sd
    bcf SD_CS_LAT, SD_CS_PIN
    ;valid address should be set on sd_data+1 to sd_data+3
    movlw 0x51			;CMD 17
    movwf sd_data
    
    call sd_send_command
    
_sd_wait_rd:
    movlw 0xff
    call ssp_write
    movwf sd_temp
    
    incf sd_tries_count, f
    movlw SD_TRIES_MAX
    xorwf sd_tries_count, w
    bz sd_read_timeout
    
    movlw 0xFE
    xorwf sd_temp, w
    bz _sd_wait_rd
    
   ; movlw 0x01
    clrf sd_byte_count			      ; 512 bytes + 2 checksum bytes
    movlw 0x02
    movwf sd_byte_count+1                     ; 
    
_sd_read_loop:
    call ssp_read
    movwf sd_temp
    
    ;movwf INDF0
    ;incf FSR0, f
    incfsz sd_byte_count, f
    bra _sd_read_loop
    decfsz sd_byte_count+1, f
    bra _sd_read_loop
    call ssp_read			    ; 2 checksum bytes
    call ssp_read
    
    ;movlw sd_block_buffer+511
    call usart_newline
    movf sd_temp, w
    call usart_hex2ascii
    call usart_newline
    
    bra sd_read_done
    
sd_read_timeout:
    movlw 'T'
    call usart_putchar
    
sd_read_done:    
    ;deselect sd
    bsf SD_CS_LAT, SD_CS_PIN
    
    return
;----------------------------------------------------------
    
sd_send_command:
    ; sd_data array should be filled
    ;command
    movlw 0x40
    iorwf sd_data, f
    movf sd_data, w
    call ssp_write
    
    ;args
    movf sd_data+1, w
    call ssp_write
    movf sd_data+2, w
    call ssp_write
    movf sd_data+3, w
    call ssp_write
    movf sd_data+4, w
    call ssp_write
    
    ;crc
    call sd_get_crc
    call ssp_write
    
    return
;---------------------------------------------------
sd_get_crc:
    
    movf sd_data, w
    xorlw 0x40
    btfsc STATUS, Z
    retlw 0x95
    movf sd_data, w
    xorlw 0x48
    btfsc STATUS, Z
    retlw 0x87
    retlw 0x01                       ; default crc
    
    return
;----------------------------------------------------

GLOBAL sd_data    
GLOBAL sd_init
GLOBAL sd_read_block
    
    END


