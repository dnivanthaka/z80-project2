;******************************************************************************
;                                                                             *
;   This file is a basic code template for code generation on the             *
;   PIC18F46K22. This file contains the basic code building blocks to build   *
;   upon.                                                                     *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on features    *
;   of the assembler.                                                         *
;                                                                             *
;   Refer to the respective data sheet for additional information on the      *
;   instruction set.                                                          *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:         xxx.asm                                                *
;    Date:                                                                    *
;    File Version:                                                            *
;    Author:                                                                  *
;    Company:                                                                 *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Files required:                                                          *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Notes:                                                                   *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Revision History:                                                        *
;                                                                             *
;******************************************************************************

;------------------------------------------------------------------------------
; PROCESSOR DECLARATION
;------------------------------------------------------------------------------

     LIST      P=PIC18F46K22          ; list directive to define processor
     #INCLUDE <p18f46k22.inc>         ; processor specific variable definitions

;------------------------------------------------------------------------------
;
; CONFIGURATION WORD SETUP
;
; The 'CONFIG' directive is used to embed the configuration word within the 
; .asm file. The lables following the directive are located in the respective 
; .inc file.  See the data sheet for additional information on configuration 
; word settings.
;
;------------------------------------------------------------------------------
;CONFIG  FOSC = INTIO67 
; CONFIG1H
  CONFIG  FOSC = INTIO67        ; Oscillator Selection bits (Internal oscillator block)
  CONFIG  PLLCFG = ON           ; 4X PLL Enable (Oscillator multiplied by 4)
  CONFIG  PRICLKEN = ON        ; Primary clock enable bit (Primary clock can be disabled by software)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = ON            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  PWRTEN = ON          ; Power-up Timer Enable bit (Power up timer disabled)
  CONFIG  BOREN = OFF           ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
  CONFIG  BORV = 190            ; Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

; CONFIG2H
  CONFIG  WDTEN = OFF           ; Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
  CONFIG  WDTPS = 1             ; Watchdog Timer Postscale Select bits (1:1)

; CONFIG3H
  CONFIG  CCP2MX = PORTB3       ; CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
  CONFIG  PBADEN = OFF          ; PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
  CONFIG  CCP3MX = PORTE0       ; P3A/CCP3 Mux bit (P3A/CCP3 input/output is mulitplexed with RE0)
  CONFIG  HFOFST = OFF          ; HFINTOSC Fast Start-up (HFINTOSC output and ready status are delayed by the oscillator stable status)
  CONFIG  T3CMX = PORTB5        ; Timer3 Clock input mux bit (T3CKI is on RB5)
  CONFIG  P2BMX = PORTD2        ; ECCP2 B output mux bit (P2B is on RD2)
  CONFIG  MCLRE = EXTMCLR       ; MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

; CONFIG4L
  CONFIG  STVREN = OFF          ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
  CONFIG  LVP = OFF             ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

; CONFIG5L
  CONFIG  CP0 = OFF             ; Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
  CONFIG  CP1 = OFF             ; Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
  CONFIG  CP2 = OFF             ; Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
  CONFIG  CP3 = OFF             ; Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protection bit (Data EEPROM not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
  CONFIG  WRT2 = OFF            ; Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
  CONFIG  WRT3 = OFF            ; Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#define         SRAM_CS_PIN          2
#define         SRAM_CS_LAT          LATE
#define         SRAM_CS_TRIS         TRISE
 
#define         SRAM_OE_PIN          0
#define         SRAM_OE_LAT          LATE
#define         SRAM_OE_TRIS         TRISE 
 
#define         SRAM_WE_PIN          1
#define         SRAM_WE_LAT          LATE
#define         SRAM_WE_TRIS         TRISE 
  
#define		Z80_RESET_PIN        1
#define		Z80_RESET_LAT        LATC
#define		Z80_RESET_TRIS       TRISC
  
#define		Z80_IOREQ_PIN	     2
#define		Z80_IOREQ_LAT        LATC
#define		Z80_IOREQ_TRIS       TRISC
#define		Z80_IOREQ_PORT       PORTC
  
#define		Z80_WAIT_PIN         0
#define		Z80_WAIT_LAT         LATB
#define		Z80_WAIT_TRIS        TRISB
  
#define		Z80_BUSREQ_PIN       4
#define		Z80_BUSREQ_LAT       LATB
#define		Z80_BUSREQ_TRIS      TRISB
#define		Z80_BUSREQ_PORT      PORTB 
  
#define		Z80_BUSACK_PIN       5
#define		Z80_BUSACK_LAT       LATB
#define		Z80_BUSACK_TRIS      TRISB
#define		Z80_BUSACK_PORT      PORTB
  
#define		Z80_WAITRES_PIN      3
#define		Z80_WAITRES_LAT      LATB
#define		Z80_WAITRES_TRIS     TRISB
#define		Z80_WAITRES_PORT     PORTB  

#define _EI_ bsf INTCON, 7 ; GIE
#define _DI_ bcf INTCON, 7 ; GIE  
  
SRAM_CS_HI MACRO
	    bsf     SRAM_CS_LAT, SRAM_CS_PIN
	    ENDM 
 
SRAM_CS_LO MACRO
	    bcf     SRAM_CS_LAT, SRAM_CS_PIN
	    ENDM 

SRAM_OE_HI MACRO
	    bsf     SRAM_OE_LAT, SRAM_OE_PIN
	    ENDM 
	    
SRAM_OE_LO MACRO
	    bcf     SRAM_OE_LAT, SRAM_OE_PIN
	    ENDM 
	    
SRAM_WE_HI MACRO
	    bsf     SRAM_WE_LAT, SRAM_WE_PIN
	    ENDM 
	    
SRAM_WE_LO MACRO
	    bcf     SRAM_WE_LAT, SRAM_WE_PIN
	    ENDM  
  
EXTERN delay_us
EXTERN delay_millis 
  
EXTERN ssp_init
EXTERN ssp_read
EXTERN ssp_write
	    
EXTERN addressbus_val	    
EXTERN addressbus_init
EXTERN addressbusmode_set
EXTERN addressbus_read
EXTERN addressbus_write
	    
EXTERN databus_init
EXTERN databusmode_set
EXTERN databus_write
EXTERN databus_read
EXTERN databus_clear
	    
EXTERN usart_init
EXTERN usart_putchar
EXTERN usart_getchar
EXTERN usart_newline
EXTERN usart_getmessages
EXTERN usart_hex2ascii
EXTERN usart_readline
EXTERN usart_rxline
	    
EXTERN rom_data
	    
SRAM_WRITE MACRO ADDR, VAL
	    movlw ADDR & 0xff
	    movwf addressbus_val
	    movlw (ADDR >> 8) & 0xff
	    movwf addressbus_val+1

	    movlw VAL
	    call sram_write
	   ENDM
	   
SRAM_READ MACRO ADDR
	    movlw ADDR & 0xff
	    movwf addressbus_val
	    movlw (ADDR >> 8) & 0xff
	    movwf addressbus_val+1

	    call sram_read
	   ENDM

;------------------------------------------------------------------------------
;
; VARIABLE DEFINITIONS
;
; Refer to datasheet for available data memory (RAM) organization
;
;------------------------------------------------------------------------------

; Example of using GPR Uninitialized Data
GPR_VAR        UDATA           
MYVAR1         RES        1      ; User variable linker places
MYVAR2         RES        1      ; User variable linker places
MYVAR3         RES        1      ; User variable linker places

; Example of using Access Uninitialized Data Section
INT_VAR        UDATA_ACS       
W_TEMP         RES        1      ; w register for context saving (ACCESS)
STATUS_TEMP    RES        1      ; status used for context saving 
BSR_TEMP       RES        1      ; bank select used for ISR context saving
       
       UDATA
spi_sram_addr RES 3		; 24bit
spi_sram_temp RES 1
sram_temp     RES 1
   
ds12887_reg   RES 1
ds12887_val   RES 1

serial_status RES 1   
;------------------------------------------------------------------------------
; EEPROM INITIALIZATION
;
; The 18F46K22 has 256 bytes of non-volatile EEPROM starting at 0xF00000
; 
;------------------------------------------------------------------------------

DATAEE    CODE    0xF00000 ; Starting address for EEPROM for 18F4553

    DE    "MCHP"           ; Place 'M' 'C' 'H' 'P' at address 0,1,2,3

;------------------------------------------------------------------------------
; RESET VECTOR
;------------------------------------------------------------------------------

RES_VECT  CODE    0x0000            ; processor reset vector
          GOTO    START             ; go to beginning of program

;------------------------------------------------------------------------------
; HIGH PRIORITY INTERRUPT VECTOR
;------------------------------------------------------------------------------

ISRHV     CODE    0x0008

          ; Run the High Priority Interrupt Service Routine
          GOTO    HIGH_ISR             

;------------------------------------------------------------------------------
; LOW PRIORITY INTERRUPT VECTOR
;------------------------------------------------------------------------------

ISRLV     CODE    0x0018
          
          ; Run the High Priority Interrupt Service Routine
          GOTO    LOW_ISR             

;------------------------------------------------------------------------------
; HIGH PRIORITY INTERRUPT SERVICE ROUTINE
;------------------------------------------------------------------------------

ISRH      CODE                        ; let linker place high ISR routine

HIGH_ISR  

          ; Insert High Priority ISR Here
	  
	  btfsc PIR1, RC1IF
	  bra usart_int
	  
	  movlw 'I'
	  call usart_putchar
	  
	  ;call databus_read
	  ;call usart_hex2ascii
	  
	  ;call addressbus_read
	  ;call usart_hex2ascii
	  
	  ;Reset latch
	  bcf Z80_WAITRES_LAT, Z80_WAITRES_PIN
	  nop
	  nop
	  nop
	  nop
	  bsf Z80_WAITRES_LAT, Z80_WAITRES_PIN
	  nop
	  
	  bcf INTCON, INT0IF
	  
	  bra int_end
	  
usart_int:
	bsf serial_status, 0
	call usart_getchar
    
	bcf PIR1, RC1IF
	  
int_end:
          RETFIE  FAST

;------------------------------------------------------------------------------
; LOW PRIORITY INTERRUPT SERVICE ROUTINE
;------------------------------------------------------------------------------

ISRL      CODE                        ; let linker place low ISR routine

LOW_ISR
          ; Context Saving for Low ISR
          MOVWF   W_TEMP              ; save W register
          MOVFF   STATUS, STATUS_TEMP ; save status register
          MOVFF   BSR, BSR_TEMP       ; save bankselect register

          ; Insert Low Priority ISR Here
	  
	  movlw 'J'
	  call usart_putchar

          ; Context Saving for Low ISR
          MOVFF   BSR_TEMP, BSR       ; restore bankselect register
          MOVF    W_TEMP, W           ; restore W register
          MOVFF   STATUS_TEMP, STATUS ; restore status register
          RETFIE

;------------------------------------------------------------------------------
; MAIN PROGRAM
;------------------------------------------------------------------------------

MAIN_PROG CODE                        ; let linker place main program

START
	  call mcu_init
	  ;call ssp_init
	  call usart_init
    
	  bsf Z80_BUSACK_TRIS, Z80_BUSACK_PIN
	  
	  bsf Z80_BUSREQ_LAT, Z80_BUSREQ_PIN
	  bcf Z80_BUSREQ_TRIS, Z80_BUSREQ_PIN
    
	  ;bsf Z80_WAIT_LAT, Z80_WAIT_PIN
	  bsf Z80_WAIT_TRIS, Z80_WAIT_PIN
	  
	  bsf Z80_IOREQ_TRIS, Z80_IOREQ_PIN

	  bcf Z80_WAITRES_TRIS, Z80_WAITRES_PIN
	  bsf Z80_WAITRES_LAT, Z80_WAITRES_PIN
	  
	  bcf Z80_RESET_TRIS, Z80_RESET_PIN
	  bcf Z80_RESET_LAT, Z80_RESET_PIN
	  
	  call databus_init
	  call addressbus_init
    
	  ;call sram_deselect
	  call sram_init
	  
	  clrf serial_status
	  
	  ;bsf LATB,  RB3
	  ;bcf TRISB, RB3
	  
	  ;init ram
;	  bcf LATB, RB3
;	  movlw 0x01		    ;command, mode register write
;	  call ssp_write
;	  
;	  movlw 0x00
;	  call ssp_write
;	  
;	  movlw 0x02		    ;command, write
;	  call ssp_write
;	  
;	  movlw 0x00		    ;address 24bit
;	  call ssp_write
;	  movlw 0x00
;	  call ssp_write
;	  movlw 0x00
;	  call ssp_write
;	  
;	  movlw 0x56
;	  call ssp_write	    ;data
;	  bsf LATB, RB3

;	  movlw 0x01		    ;byte mode
;	  call spi_sram_mode_write
;	  
;	  movlw 0x00
;	  movwf spi_sram_addr
;	  
;	  movlw 0x00
;	  movwf spi_sram_addr+1
;	  
;	  movlw 0x00
;	  movwf spi_sram_addr+2
;	  
;	  movlw 0x56
;	  call spi_sram_write_data
	  
;	  bcf LATB, RB3
;	  movlw 0x40
;	  call ssp_write	    ; device ID
;	  
;	  movlw 0x00		    ;register, IODIRA
;	  call ssp_write
;	  
;	  movlw 0x00		    ;value
;	  call ssp_write
;	  bsf LATB, RB3
;	  SRAM_WRITE 0x0000, 0x00
;	  SRAM_WRITE 0x0001, 0x76
;	  SRAM_WRITE 0x0002, 0x00
	  
	  
;	  SRAM_WRITE 0x0000, 0x3e
;	  SRAM_WRITE 0x0001, 0x80
;	  SRAM_WRITE 0x0002, 0xd3
;	  SRAM_WRITE 0x0003, 0x00
;	  SRAM_WRITE 0x0004, 0x00
;	  SRAM_WRITE 0x0005, 0x00
;	  SRAM_WRITE 0x0006, 0x00
;	  SRAM_WRITE 0x0007, 0x00
;	  SRAM_WRITE 0x0008, 0x00
;	  SRAM_WRITE 0x0009, 0x00
;	  SRAM_WRITE 0x000A, 0x00
;	  SRAM_WRITE 0x000B, 0x00
;	  SRAM_WRITE 0x000C, 0x00
;	  SRAM_WRITE 0x000D, 0x00
;	  SRAM_WRITE 0x000E, 0x00
;	  SRAM_WRITE 0x000F, 0x00
;	  SRAM_WRITE 0x0010, 0x00
;	  SRAM_WRITE 0x0011, 0x00
;	  SRAM_WRITE 0x0012, 0x00
;	  SRAM_WRITE 0x0013, 0x00
;	  SRAM_WRITE 0x0014, 0xc3
;	  SRAM_WRITE 0x0015, 0x14
;	  SRAM_WRITE 0x0016, 0x00
;	  SRAM_WRITE 0x0017, 0x76
	  
	  call write_rom_to_sram
	  
mem_print_loop:
	  movlw '0'
	  call usart_putchar
	  movlw 'x'
	  call usart_putchar
	  
	  call sram_read
	  call usart_hex2ascii
	  movlw ' '
	  call usart_putchar
	  
	  incfsz addressbus_val, f
	  bra mem_print_loop
	  
	  ;movlw 'X'
	  ;call usart_putchar
	  
	  call release_control
	  call z80_reset
	  
	  bcf INTCON2, INTEDG0
	  bcf INTCON, INT0IF
	  bsf INTCON, INT0IE
    
	  ;disable priority interrupts
	  bcf RCON, IPEN
	  
	  bsf INTCON, 6 ; PEIE
	  bsf INTCON, 7 ; GIE

	  
blink_loop:    
    
	  btfss serial_status, 0
	  bra blink_loop
	  
	  _DI_
	  bcf Z80_BUSREQ_LAT, Z80_BUSREQ_PIN
	  
_wait_busack:
	  btfsc Z80_BUSACK_PORT, Z80_BUSACK_PIN
	  bra _wait_busack
	  
	  call gain_control
	  
	  ;--------------------------------------------
conn_loop:
	  call usart_newline
	  movlw '>'
	  call usart_putchar
	  
	  call usart_readline
	  
	  ;movf usart_rxline, w
	  ;call usart_putchar
	  movf usart_rxline, w
	  ;call usart_newline
	  ;movlw 'X'
	  ;call usart_putchar
	  xorlw 'q'
	  bz conn_end
	  movf usart_rxline+16, w
	  call usart_hex2ascii
	  bra conn_loop
conn_end:
	  bcf serial_status, 0  
	  ;call usart_newline
	  
	  ;clrf addressbus_val
	  ;clrf addressbus_val+1
	  
;mem_print_loop:
;	  movlw '0'
;	  call usart_putchar
;	  movlw 'x'
;	  call usart_putchar
;	  
;	  call sram_read
;	  call usart_hex2ascii
;	  movlw ' '
;	  call usart_putchar
;	  
;	  incfsz addressbus_val, f
;	  bra mem_print_loop
	  
	  movlw 0x00
	  movwf ds12887_reg
	  
	  call ds12887_read
	  call usart_hex2ascii
;	  
;	  movlw .255
;	  call delay_millis
;	  movlw .255
;	  call delay_millis
;	  movlw .255
;	  call delay_millis
;	  
;	  movlw 0x09
;	  movwf ds12887_reg
;	  
;	  movlw 0x63
;	  movwf ds12887_val
;	  
;	  call ds12887_write
	  
;	  movlw 0x00
;	  movwf ds12887_val
;	  
;	  call ds12887_read
;	  
	  movlw .255
	  call delay_millis
	  movlw .255
	  call delay_millis
	  movlw .255
	  call delay_millis
	  movlw .255
	  call delay_millis
;	  movlw .255
;	  call delay_millis
;	  movlw .255
;	  call delay_millis
	  
	  ;----------------------------------------------------
	  
	  call release_control
	  
	  bsf Z80_BUSREQ_LAT, Z80_BUSREQ_PIN
	  
_wait_busack2:
	  btfss Z80_BUSACK_PORT, Z80_BUSACK_PIN
	  bra _wait_busack2
	  
	  _EI_
	  
	  movlw .255
	  call delay_millis
	  movlw .255
	  call delay_millis
	  
	  ;bcf LATB, RB3
	  
	  movlw .255
	  call delay_millis
	  movlw .255
	  call delay_millis
	  

          bra blink_loop                      ; loop program counter
	  
;---------------------------------------------------
	  
mcu_init:
    ;setting high speed oscillator
    movlw b'01110100'
    movwf OSCCON
    
    bsf OSCCON2, PLLRDY
    
    movlw b'11011111'
    movwf OSCTUNE
    
    movlb 0xf			; Set BSR for banked SFRs, Page 137
    
    clrf PORTA
    clrf LATA
    
    clrf PORTB
    clrf LATB
    
    clrf PORTC
    clrf LATC
    
    clrf PORTD
    clrf LATD
    
    clrf ADCON0
    ;movlw 0x0f
    ;movwf ADCON1                 ; disabling AD converter
    clrf ANSELA
    clrf ANSELB
    clrf ANSELC
    clrf ANSELD
    clrf ANSELE
    
    clrf CCP1CON
    clrf CCP2CON                ; disable comparator
    
    bcf CM1CON0, C1ON
    bcf CM2CON0, C2ON
    
    
    
    ;PWM setup
    
;    clrf CCPTMRS0               ; CCP1 PWM uses Timer 2
;    
;    movlw b'00001100'
;    movwf CCP1CON
;    
;    movlw 0x01
;    movwf PR2
;    
;    bcf PWM1CON, P1RSEN
;    
;    movlw 0x01
;    movwf CCPR1L
;    
;    bcf PIR1, TMR2IF
;    
;    ;Timer2 setup
;    movlw b'00000100'                 ; no prescalar
;    movwf T2CON
;    
;    ;bsf PIE1, TMR2IE
;    
;wait_for_tmr2f:
;    btfss PIR1, TMR2IF
;    bra wait_for_tmr2f
    
    ;bcf TRISC, RC2
    
    return
;---------------------------------------------------------------- 
    
spi_sram_write_data:
    movwf spi_sram_temp
    
    bcf LATB, RB3

    movlw 0x02		    ;command, write
    call ssp_write

    movf spi_sram_addr, w		    ;address 24bit
    call ssp_write
    movf spi_sram_addr+1, w
    call ssp_write
    movf spi_sram_addr+2, w
    call ssp_write

    movf spi_sram_temp, w
    call ssp_write	    ;data
    
    bsf LATB, RB3
    
    return
;---------------------------------------------------
    
spi_sram_read_data:
    bcf LATB, RB3

    movlw 0x03		    ;command, read
    call ssp_write

    movf spi_sram_addr, w		    ;address 24bit
    call ssp_write
    movf spi_sram_addr+1, w
    call ssp_write
    movf spi_sram_addr+2, w
    call ssp_write

    movlw 0x00		    ; dummy byte
    call ssp_write	    ;data
    
    bsf LATB, RB3
    
    return
;---------------------------------------------------
    
spi_sram_mode_write:
    movwf spi_sram_temp
    
    bcf LATB, RB3
    
    movlw 0x01		    ;command, mode register write
    call ssp_write

    movf spi_sram_temp, w
    call ssp_write
    
    bsf LATB, RB3
    
    return
;---------------------------------------------------    
sram_read:
    clrf sram_temp
    
    movlw 0x00
    call addressbusmode_set
    
    SRAM_WE_HI
    SRAM_OE_HI
    
    ; write address, should have the addressbus_val and addressbus_val+1 values set
    call addressbus_write
    
    movlw 0x01
    call databusmode_set

    SRAM_CS_LO
    SRAM_OE_LO
    
    ;nop
    ;nop
    call databus_read
    movwf sram_temp
    
    SRAM_OE_HI
    SRAM_CS_HI
    
;    movlw 0x00
;    call databusmode_set
    
    movf sram_temp, w
    
    return
;----------------------------------------------------------------    
sram_write:
    movwf sram_temp
    
    movlw 0x00
    call addressbusmode_set
    
    movlw 0x00
    call databusmode_set
    
    ; write address, should have the addressbus_val and addressbus_val+1 values set
    call addressbus_write
    
    movf sram_temp, w
    
    SRAM_OE_HI
    SRAM_CS_LO
    
    call databus_write
    
    SRAM_WE_LO
    
    nop
    nop
    
    SRAM_WE_HI
    SRAM_CS_HI
    
    return
;----------------------------------------------------------------
sram_deselect:
    SRAM_OE_HI
    SRAM_CS_HI
    SRAM_WE_HI
    nop
    
    return
;----------------------------------------------------------------
sram_init:
    ;sram pins
    
    bsf SRAM_CS_LAT, SRAM_CS_PIN
    bcf SRAM_CS_TRIS, SRAM_CS_PIN
    
    bsf SRAM_OE_LAT, SRAM_OE_PIN
    bcf SRAM_OE_TRIS, SRAM_OE_PIN
    
    bsf SRAM_WE_LAT, SRAM_WE_PIN
    bcf SRAM_WE_TRIS, SRAM_WE_PIN
    
    call sram_deselect
    
    return
;----------------------------------------------------------------
    
release_control:
    ; sram control lines
    bsf SRAM_CS_TRIS, SRAM_CS_PIN
    bsf SRAM_OE_TRIS, SRAM_OE_PIN
    bsf SRAM_WE_TRIS, SRAM_WE_PIN
    bsf Z80_IOREQ_TRIS, Z80_IOREQ_PIN
    
    movlw 0x01
    call addressbusmode_set
    
    movlw 0x01
    call databusmode_set
    
    call sram_deselect
    
    return
;----------------------------------------------------------------
gain_control:
    bsf Z80_IOREQ_LAT, Z80_IOREQ_PIN
    bcf Z80_IOREQ_TRIS, Z80_IOREQ_PIN

    bsf SRAM_CS_LAT, SRAM_CS_PIN
    bcf SRAM_CS_TRIS, SRAM_CS_PIN

    bsf SRAM_OE_LAT, SRAM_OE_PIN
    bcf SRAM_OE_TRIS, SRAM_OE_PIN

    bsf SRAM_WE_LAT, SRAM_WE_PIN
    bcf SRAM_WE_TRIS, SRAM_WE_PIN

    movlw 0x00
    call addressbusmode_set

    movlw 0x00
    call databusmode_set
    
    return
    
z80_reset:
    bcf Z80_RESET_LAT, Z80_RESET_PIN
    bcf Z80_RESET_TRIS, Z80_RESET_PIN
    
    movlw .200
    call delay_millis
    
    bsf Z80_RESET_LAT, Z80_RESET_PIN
    ;bsf Z80_RESET_TRIS, Z80_RESET_PIN
    
wait_busack:
    btfss Z80_BUSACK_PORT, Z80_BUSACK_PIN
    bra  wait_busack    
    
    nop
    
    return
;-------------------------------------------------------
write_rom_to_sram:
    movlw upper rom_data
    movwf TBLPTRU
    movlw high rom_data
    movwf TBLPTRH
    movlw low rom_data
    movwf TBLPTRL
    
    ;flash memory read
    movlw 0x80
    movwf EECON1
    
    clrf addressbus_val
    clrf addressbus_val+1
    
_wr_sram_from_rom:
    TBLRD*+
    nop
    nop
    nop
    movf TABLAT, w
    call sram_write ;call to write sram, even
    incfsz addressbus_val, f
    bra _wr_sram_from_rom
    
    incf addressbus_val+1, f
    
    movf addressbus_val+1, w
    xorlw 0x02
    bnz _wr_sram_from_rom
    return
    
;-------------------------------------------------------
    
ds12887_write:
    ;ds12887_reg and ds12887_val should be set
    ;set latch AS pin as high
    clrf addressbus_val+1
    bsf addressbus_val+1, 0
    movlw 0x02                   ;address of ds12887
    movwf addressbus_val
    call addressbus_write
    
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
    
    bcf SRAM_WE_LAT, SRAM_WE_PIN
    nop
    nop
    nop
    nop
    
    movf ds12887_val, w
    call databus_write
    
    bsf SRAM_WE_LAT, SRAM_WE_PIN
    bsf Z80_IOREQ_LAT, Z80_IOREQ_PIN
    
    return    
;-------------------------------------------------------
    
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
          END


