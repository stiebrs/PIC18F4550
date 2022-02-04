#include <p18cxxx.h>

// on PIC-USB-STK
// 20MHz crystal on OSC1:2
// LEDs 1:4 are on RD0:3
// buttons RB4:7
// pot on RA0/AN0
// power up PICKIT4 then devboard

#define _XTAL_FREQ 2000000

// http://ww1.microchip.com/downloads/en/DeviceDoc/51537a.pdf
// CONFIG1L
// PLL Prescaler Selection bits (Divide by 5 (20 MHz oscillator input))
#pragma config PLLDIV = 5
// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config CPUDIV = OSC1_PLL2
// USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes from the 96 MHz PLL divided by 2)
#pragma config USBDIV = 2

// CONFIG1H
// Oscillator Selection bits (HS oscillator (HS), section 2.2 of DS)
#pragma config FOSC = HSPLL_HS    // should be 20MHz
//#pragma config FOSC = XT_XT    // should be 20MHz
// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config FCMEN = OFF
// Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
#pragma config IESO = OFF

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

void init_timers(void);
void init_uart(uint16_t baudrate);
void init_adc(void);

void Timer0_Isr(void) {
    LATDbits.LATD1 = !LATDbits.LATD1;
}

#define ADC_UNIT_V 5.0f/(float)1024
uint32_t glob = 0;
uint16_t adc_raw = 0;
float adc_voltage = 0.0f;

// need to set compiler to C90 so this would compile
// otherwise "error: expected ';' after top level declarator"
void high_priority interrupt high_isr(void) {
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0x0;  // Clear timer0 overflow interrupt flag
        // add your TMR0 interrupt custom code
        Timer0_Isr();
    }
    if (PIE1bits.RCIE && PIR1bits.RCIF) { // ReCeive Interrupt Flag
        // do receive stuff
    }
    if (PIE1bits.TXIE && PIR1bits.TXIF) { // Transmit Interrupt Flag
          PIE1bits.TXIE = 0;  // Disable TX interrupt
    }
    if (PIE1bits.ADIE && PIR1bits.ADIF) { // ADC done interrupt
          PIR1bits.ADIF = 0;  // Clear interrupt
          // update value
          adc_raw = (ADRESH << 8) | (ADRESL);
          adc_voltage = (float) 5.125 * ((float) adc_raw / (float) 1024);
          // start next update cycle
          ADCON0bits.GO = 1;
    }
}

void main(void) {
    // set pins[3:0] on port D as outputs
    TRISD = 0b11110000;

    init_timers();
    init_uart(9600);
    init_adc();
    LATDbits.LATD0 = 1;
        
    uint32_t i = 0;
    while(1) {
        i++;
        if (i % 10) {
            glob++;
        }
        if (i / 4000) {
            i= 0;
            LATDbits.LATD0 = !LATDbits.LATD0;
        }
    }
    return;
}

void init_timers(void) {
  T0CONbits.T08BIT = 0;     // 16 Bit timer
  T0CONbits.T0CS = 0;       // Internal clock (Fosc / 4)
  T0CONbits.PSA = 0;        // prescaler not assigned
  T0CONbits.TMR0ON = 1;     // Timer0 on
  
  // 1 s for 20MHz clock
  // 20MHz /4 / prescaler 128 /
  T0CONbits.T0PS2 = 1;      // 2/4/8/16/32/64/128/256 prescaler
  T0CONbits.T0PS1 = 1;
  T0CONbits.T0PS0 = 0;
  
  TMR0H = 0x67;             //
  TMR0L = 0x69;
  
  INTCONbits.GIE=1;         // global interrupt enable
  INTCONbits.PEIE=1;        // peripheral interrupt enable
  INTCONbits.TMR0IF = 0x0;  // Clear timer0 overflow interrupt flag
  INTCONbits.TMR0IE = 1;    // enable the timer0 by setting TRM0IE flag
}

#define SPBRG_TARGET(br) ((float)_XTAL_FREQ / (16 * (float)br)) - 1
void init_uart(uint16_t baudrate) {
    TRISC6 = 0;     // TX pin output
    TRISC7 = 1;     // RX pin input
    float temp = SPBRG_TARGET(baudrate);
    SPBRG = (int) temp;
    TXSTA = 0x20;   // TX enable
    RCSTA = 0x90;   // RX enable and serial port enable
    
    PIE1bits.RCIE = 1;  // Enable RX interrupt
    PIE1bits.TXIE = 1;  // Enable TX interrupt
}

void init_adc(void) {
    TRISAbits.RA0 = 1;      /* Set as input port */
    ADCON1 = 0x00;          /* Reset bits */
    ADCON1bits.VCFG1 = 0;   // REF- is GND, not VREF- (AN2)
    ADCON1bits.VCFG0 = 0;   // REF+ is VDD, not VREF+ (AN3)
    // bits [3:0] are channel selection (0:A, 1:D)
    ADCON2 = 0x92;          /* Right Justified, 4Tad and Fosc/32. */
    ADCON2bits.ADFM = 1;    // right justified
    // capture for 20 ADC Clock cycles
    ADCON2bits.ACQT0 = 1;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT1 = 1;
    // FOSC/64 clock source
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 1;
    ADRESH=0;		/* Flush ADC output Register */
    ADRESL=0;
    
    // clear interrupt
    PIR1bits.ADIF = 0;
    // enable interrupt
    PIE1bits.ADIE = 1;
    ADCON0bits.ADON = 1;
    
    ADCON0bits.GO = 1;
}