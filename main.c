/* ------------------------------------------------------------------------
 * PSX In-Game Reset (IGR) for PIC16F18325/26
 * Timed Combo Reset Trigger with LED Feedback
 * Electroanalog (c) 2025
 *
 * Based on original code for "PlayStation 1 Reset Mod" by pyroesp (2019)
 * ------------------------------------------------------------------------
 * This is a derivative work licensed under the GNU General Public 
 * License as published by the Free Software Foundation; either 
 * version 2 of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This version features:
 * - Combo hold logic to prevent unintentional resets
 * - Combo feedback LED driven by asynchronous interrupt
 * - Reset feedback LED blink handled via asynchronous interrupt
 * - Support for PSU-mounted common-anode dual LED (RA5 control only)
 * - Optional support for 2-pin bicolor LED between RA4 and RA5 (dual-color status)
 * ------------------------------------------------------------------------
 * 
 * PIC16F18325/26 Pinout:
 *  1 - VCC = +3.3V
 *  2 - RA5 = COMBO/RESET LED -> Output to [-] cathode sinking control (PSU)
 *  3 - RA4 = COMBO/RESET LED -> Output to [+] anode sourcing control (2-pin bicolor LED)
 *  4 - RA3 = ICSP MCLR/VPP
 *  5 - RC5 = RESET
 *  6 - RC4 = (not used)
 *  7 - RC3 = DEBUG
 *  8 - RC2 = SPI CMD
 *  9 - RC1 = SPI DATA
 * 10 - RC0 = SPI CLK
 * 11 - RA2 = SPI SS
 * 12 - RA1 = ICSP CLK
 * 13 - RA0 = ICSP DAT
 * 14 - VSS = GND
 *
 * For installation instructions, wiring options, and usage details,
 * please refer to the official README at:
 * https://www.electroanalog.com/psx-igr
 */

#include <xc.h>
#include <stdint.h>

// CONFIG1
#pragma config FEXTOSC = OFF     // FEXTOSC External Oscillator mode Selection bits (HS (crystal oscillator) above 4 MHz)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will not cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// BASIC DEFINITIONS
#define _XTAL_FREQ 32000000     // Clock frequency
#pragma anon_unions  			// Enable anonymous unions
#define _BV(b) (1<<(b))         // Create a bit mask by shifting 1 left by 'b' positions
// LED indicator for controller/guncon combo
#define LED_A     LATA4 // LED output ([+] anode-driven | RA4 active high)
#define LED_K     LATA5 // LED output ([-] cathode-driven | RA5 active low)
// Unified control macros for both LED output control
#define LED_BLINK_ON()  do { LED_K = 0; } while(0) // Blink ON (RA5 LOW)
#define LED_BLINK_OFF() do { LED_K = 1; } while(0) // Blink OFF (RA5 HIGH)
#define LED_IDLE()		do { TRISA4 = 0; TRISA5 = 0; LED_A = 0; LED_K = 1; } while(0) // Idle LED ON (RA4 LOW, RA5 HIGH)
#define LED_COMBO()		do { TRISA4 = 0; TRISA5 = 0; LED_A = 1; LED_K = 0; } while(0) // Combo LED ON (RA4 HIGH, RA5 LOW)
#define LED_OFF()		do { TRISA4 = 1; TRISA5 = 1; } while(0) // RA4 and RA5 as inputs (Hi-Z)

/* Debug Configuration */
/* Uncomment the define below to have UART TX debugging on RC3 */
//#define DEBUG
#ifdef DEBUG
    #define REBOOT_DELAY 2 // s
#else
    #define REBOOT_DELAY 12 // s
#endif

// Timing constants
#define TMR0_PRELOAD_H  0xFF	// Timer0 - Preload high byte for 1 ms tick @ 16 MHz / 64
#define TMR0_PRELOAD_L  0x81	// Timer0 - Preload low byte (0xFF81 = 65473 -> 63 ticks = 1 ms)
#define SHORT_DELAY 500 // ms | System Reset
#define LONG_DELAY 2 // s | System Reset
#define COMBO_HOLD 1250 // ms | Ensures intentional reset activation through timed hold
#define RESET_BLINK_COUNT 3 // LED cycle
#define RESET_BLINK_TOTAL_MS 500 // ms
#define RESET_BLINK_DELAY_MS 200 // ms | Delay after blink before returning to idle

// Reset port
#define PS1_LAT_IO LATC // IO port reg used for RESET
#define PS1_TRIS_IO TRISC // IO dir reg used for RESET
#define PS1_RESET 5 // Only IO that outputs a logic 0

// Controller IDs
#define ID_DIG_CTRL 0x5A41 // Digital Pad: SCPH-1010, SCPH-1080
#define ID_ANP_CTRL 0x5A73 // Analog Pad: SCPH-1150, SCPH-1180, SCPH-1200, SCPH-110
#define ID_ANS_CTRL 0x5A53 // Analog Stick: SCPH-1110
#define ID_DS2_CTRL 0x5A79 // DualShock 2: SCPH-10010
#define ID_GUNCON_CTRL 0x5A63 // Light Gun: NPC-103

// PSX Commands
#define CMD_SEL_CTRL_1 0x01 // select controller 1
#define CMD_SEL_MEMC_1 0x81 // select memory card 1
#define CMD_READ_SW 0x42 // read switch status from controller

// PSX Communication Buffer Size
#define PS1_CTRL_BUFF_SIZE 9 // max size of buffer needed for a controller

// Key Combos
#define KEY_COMBO_CTRL 0xFCF6       // select-start-L2-R2   1111 1100 1111 0110
#define KEY_COMBO_GUNCON 0x9FF7     // A-trigger-B          1001 1111 1111 0111
#define KEY_COMBO_XSTATION 0xBCFE   // select-cross-L2-R2   1011 1100 1111 1110

/*
Keys : 
    SELECT, // 0
    L3,
    R3,
    START, // light gun A
    UP,
    RIGHT,
    DOWN,
    LEFT,
    L2,
    R2,
    L1,
    R1,
    TRIANGLE,
    CIRCLE, // light gun trigger
    CROSS, // light gun B
    SQUARE
*/

// PSX Controller Command Union
union PS1_Cmd{
    uint8_t buff[PS1_CTRL_BUFF_SIZE];
    struct{
        uint8_t device_select; // 0x01 or 0x81
        uint8_t command; // 0x42 for read switch
        uint8_t unused[PS1_CTRL_BUFF_SIZE-2]; // always 0 for controller
    };
};

// PSX Controller Data Union
union PS1_Ctrl_Data{
    uint8_t buff[PS1_CTRL_BUFF_SIZE]; // buffer to read data
    struct{
        uint8_t unused; // always 0xFF
        uint16_t id; // 0x5Ayz - y = type; z = # of half word
        uint16_t switches; // Controller switches (key combo stored here)
        union{
            // light gun only (8MHz clock counter since hsync)
            uint16_t x_pos; // if 0x0001 then error, check y_pos
            struct{
                uint8_t adc0; // right joy X
                uint8_t adc1; // right joy Y
            };
        };
        union{
            //light gun only (scanlines since vsync)
            uint16_t y_pos; // if x_pos = 0x0001 && y_pos = 0x000A => not aimed at screen
            struct{
                uint8_t adc2; // left joy X
                uint8_t adc3; // left joy Y
            };
        };
    };
};

// Global variables for PSX communication
volatile union PS1_Cmd cmd;
volatile union PS1_Ctrl_Data data;
volatile uint8_t cmd_cnt, data_cnt;

// Global ms counter updated by Timer0 interrupt
volatile uint16_t t0_ms_count	   = 0;
volatile uint8_t  blink_active     = 0;
volatile uint8_t  blink_count      = 0;
volatile uint16_t blink_timer_ms   = 0;
volatile uint8_t  blink_led_state  = 0;
volatile uint16_t blink_idle_delay = 0;

// SPI interrupt helpers
void clear_spi_flags(void);
void enable_spi_interrupts(void);
void disable_spi_interrupts(void);

// Flush control buffers state prototype
void flush_buffers(void);

// Function prototypes
void reverse_byte(uint8_t *b);
void clear_buff(uint8_t *p, uint8_t s);
void __delay_s(uint8_t s);

// Reset control prototypes
void reset_blink(void);
void reset_combo(uint16_t duration_ms, const char* label);

// Combo processing prototype
uint8_t process_key_combo_reset(void);

#ifdef DEBUG
// UART debug output prototypes
void UART_init(void);
void UART_sendByte(uint8_t c);
void UART_print(uint8_t *str);
void UART_printHex(uint8_t b);
#else
#define UART_init(a)
#define UART_sendByte(a) 
#define UART_print(a) 
#define UART_printHex(a) 
#endif

// ISR
void __interrupt() _sys_int(void) {  
    // Timer0 interrupt - Combo LED
    if (PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        // Timer0 preload
        TMR0H = TMR0_PRELOAD_H;	// Load high byte
        TMR0L = TMR0_PRELOAD_L; // Load low byte
        t0_ms_count++;
        // LED blink handler
    	if (blink_active) {
        	blink_timer_ms++;
        	const uint16_t interval = 
				RESET_BLINK_TOTAL_MS / (RESET_BLINK_COUNT * 2); //Phase duration
        	if (blink_timer_ms >= interval) {
            	blink_timer_ms = 0;		// Reset phase timer
            	blink_led_state ^= 1;	// Toggle state
            	if (blink_led_state) { 	// Count cycles 
                    LED_BLINK_ON();          // RA5 LOW = LED ON (red)
                } else {
                    LED_BLINK_OFF();         // RA5 HIGH = LED OFF
                	blink_count++;
                	if (blink_count >= RESET_BLINK_COUNT) {
                    	blink_active = 0;
                    	blink_idle_delay = RESET_BLINK_DELAY_MS;
                	}
            	}
        	}
    	} else if (blink_idle_delay > 0) {
            LED_OFF(); // LED OFF on delay
    		blink_idle_delay--;
    		if (blink_idle_delay == 0) {
        		LED_IDLE(); // Turn on idle LED after delay
    		}
		}
    }
    // SPI1 interrupt handling...
    if (PIR1bits.SSP1IF){
        if (data_cnt < PS1_CTRL_BUFF_SIZE){
            data.buff[data_cnt] = SSP1BUF;
            data_cnt++;
        }
        PIR1bits.SSP1IF = 0; // clear SPI1 flag
    }
    // SPI2 interrupt handling...
    if (PIR2bits.SSP2IF){
        if (cmd_cnt < PS1_CTRL_BUFF_SIZE){
            cmd.buff[cmd_cnt] = SSP2BUF;
            cmd_cnt++;
        }
        PIR2bits.SSP2IF = 0; // clear SPI2 flag
    }
}

// Main Process
void main(void){
    uint8_t i; // Buffer loop index
    
    // SETUP I/O
    ANSELA = 0; // port A is digital IO
    LATA = 0; // clear latch A
    LATC = 0; // clear latch C
    ANSELC = 0; // port C is digital IO
    
    // RESET Port config
    PS1_LAT_IO &= ~_BV(PS1_RESET); // clear reset output 
    PS1_TRIS_IO |= _BV(PS1_RESET); // set reset pin to input
    
    // Configure outputs for combo/reset LED indicator
    TRISA4 = 0;	// RA4 output ([+] A)
    TRISA5 = 0;	// RA5 output ([-] K)
    
    // Initialize LED to idle (green ON)
    LED_IDLE();
    
    // SPI I/O
    TRISAbits.TRISA2 = 1; // SS set to input
    TRISCbits.TRISC0 = 1; // SCK set to input
    TRISCbits.TRISC1 = 1; // SDI1 set to input
    TRISCbits.TRISC2 = 1; // SDI2 set to input
      
    // SETUP SPI1 & SPI2 I/O - PPS
    SSP1SSPPSbits.SSP1SSPPS = 0x02; // SS1 = RA2 
    SSP2SSPPSbits.SSP2SSPPS = 0x02; // SS2 = RA2 
    
    // PPS SPI Clock input
    SSP1CLKPPSbits.SSP1CLKPPS = 0x10; // SCK1 = RC0
    SSP2CLKPPSbits.SSP2CLKPPS = 0x10; // SCK2 = RC0
    
    // PPS SPI Data input
    SSP1DATPPSbits.SSP1DATPPS = 0x11; // SDI1 = RC1 
    SSP2DATPPSbits.SSP2DATPPS = 0x12; // SDI2 = RC2 
    
    // SETUP SPI1 & SPI2 Module
    SSP1STATbits.SMP = 0; // slave mode SPI1
    SSP1STATbits.CKE = 0; // transmit from idle to active
    SSP1CON1bits.CKP = 1; // clock idle high
    SSP1CON1bits.SSPM = 0x04; // slave mode: clk = sck pin; SS enabled
    SSP1CON3bits.BOEN = 1; // ignore BF flag
    SSP1CON1bits.SSPEN = 1; // enable SPI1
    
    SSP2STATbits.SMP = 0; // slave mode SPI2
    SSP2STATbits.CKE = 0; // transmit from idle to active
    SSP2CON1bits.CKP = 1; // clock idle = high
    SSP2CON1bits.SSPM = 0x04; // slave mode: clk = sck pin; SS enabled
    SSP2CON3bits.BOEN = 1; // ignore BF flag
    SSP2CON1bits.SSPEN = 1; // enable SPI2
    
    // SPI Buffer init
    SSP1BUF = 0xFF;
    SSP2BUF = 0xFF;
    
    // TIMER0 - 16-bit mode, 1ms tick @ 16 MHz
	T0CON1 = 0b01000101;   // Fosc/4 (4 MHz), Prescaler 1:64
	TMR0H = TMR0_PRELOAD_H;	// Load high byte
	TMR0L = TMR0_PRELOAD_L;	// Load low byte
	T0CON0 = 0b10010000;   // Timer0 ON, 16-bit mode
    PIR0bits.TMR0IF = 0;   // Clear Timer0 interrupt flag
    PIE0bits.TMR0IE = 1;   // Enable Timer0 interrupt
       
    // DEBUG
    UART_init();
    UART_print((uint8_t*)"PSX IGR:\n\r");
    
    // SETUP variables and arrays
	flush_buffers(); // Initialize buffers and counters
    
    // delay before enabling interrupts
    __delay_s(REBOOT_DELAY); // wait defined seconds before next reset
    
    // SETUP INTERRUPTS
    clear_spi_flags(); // clear SPI1/SPI2 flag
    enable_spi_interrupts(); // enable MSSP interrupt (SPI1/SPI2)

    // Enable interrupts after all initial configurations   
    INTCONbits.PEIE = 1; // peripheral interrupt enable
    INTCONbits.GIE = 1; // global interrupt enable
   
    // MAIN LOOP
	for(;;){
    	if (data_cnt >= PS1_CTRL_BUFF_SIZE && cmd_cnt >= PS1_CTRL_BUFF_SIZE){
        	// Disable SPI interrupts
			disable_spi_interrupts();
            // Debug: print SPI command/data buffers (bit-reversed) via UART in hex
        	UART_print((uint8_t*)"\n\rCMD: ");
        	for (i = 0; i < PS1_CTRL_BUFF_SIZE; i++){
            	reverse_byte((uint8_t*)&cmd.buff[i]);
            	UART_printHex((uint8_t)cmd.buff[i]);
            	UART_print((uint8_t*)" ");
        	}
        	UART_print((uint8_t*)"\n\rDATA: ");
        	for (i = 0; i < PS1_CTRL_BUFF_SIZE; i++){
            	reverse_byte((uint8_t*)&data.buff[i]);
            	UART_printHex((uint8_t)data.buff[i]);
            	UART_print((uint8_t*)" ");
        	}
        	UART_print((uint8_t*)"\n\rData done converting");      
            // Check first command for device selected
			if (cmd.device_select == CMD_SEL_CTRL_1 && cmd.command == CMD_READ_SW) {
			    // Check if device is one of the supported IDs
			    if (data.id == ID_GUNCON_CTRL ||
			        data.id == ID_DIG_CTRL   ||
			        data.id == ID_ANS_CTRL   ||
			        data.id == ID_ANP_CTRL   ||
			        data.id == ID_DS2_CTRL) {
			        // Process universal reset combo
			        if (process_key_combo_reset()) {
			            // Determine type of reset with specific combo rules
			            if ((data.id == ID_GUNCON_CTRL && data.switches == KEY_COMBO_GUNCON) ||
			                (data.id != ID_GUNCON_CTRL && data.switches == KEY_COMBO_XSTATION)) {
			                reset_combo(LONG_DELAY * 1000, "Long"); // Long reset for Guncon or XStation combo | Convert s -> ms
			            } else if (data.id != ID_GUNCON_CTRL && data.switches == KEY_COMBO_CTRL) {
			                reset_combo(SHORT_DELAY, "Short"); // Short reset for controllers combo
			            }
						flush_buffers(); // Clear buffers and counters after reset
			        }
			    }
			}              
			flush_buffers(); // Clear buffers and counters for next packet
        	clear_spi_flags();	// Clear SPI flags
            enable_spi_interrupts(); // Enable SPI interrupts

    	}
	}
}

/* Reverse byte order 
 * The PSX send data LSb first,
 * but the PIC uses a shift left register
 * so the LSb from the PSX becomes the MSb of the PIC
*/
// Casts to uint8_t added to suppress warnings due to integer promotions
void reverse_byte(uint8_t *b) { 
   *b = (uint8_t)((*b & 0xF0) >> 4 | (*b & 0x0F) << 4);
   *b = (uint8_t)((*b & 0xCC) >> 2 | (*b & 0x33) << 2);
   *b = (uint8_t)((*b & 0xAA) >> 1 | (*b & 0x55) << 1);
}

// Enable SPI interrupt
void enable_spi_interrupts(void) {
    PIE1bits.SSP1IE = 1; // SPI1 int
    PIE2bits.SSP2IE = 1; // SPI2 int
}

// Disable SPI interrupt
void disable_spi_interrupts(void) {
    PIE1bits.SSP1IE = 0; // SPI1 int
    PIE2bits.SSP2IE = 0; // SPI2 int
}

// Clear SPI interrupt flag
void clear_spi_flags(void) {
    PIR1bits.SSP1IF = 0; // SPI1 flag
    PIR2bits.SSP2IF = 0; // SPI2 flag
}

// Clear buffer of size s, aka fill with zero
void clear_buff(uint8_t *p, uint8_t s){
    uint8_t i;
    for (i = 0; i < s; i++)
        p[i] = 0;
}

// Flush data/command buffers and reset counters
void flush_buffers(void) {
    clear_buff((uint8_t*)data.buff, PS1_CTRL_BUFF_SIZE);
    clear_buff((uint8_t*)cmd.buff,  PS1_CTRL_BUFF_SIZE);
    data_cnt = cmd_cnt = 0;
}

// Delay in seconds
void __delay_s(uint8_t s){
    for (; s > 0; s--)
        __delay_ms(1000);
}

/* The following function handles key combo detection using a non-blocking, stateful approach.
 * It monitors whether a known key combination is held for the duration defined by COMBO_HOLD. 
 * If the combo remains active for that time, it returns 1 to signal that a reset should be triggered.
 *
 * Supported combos:
 * - GAMEPAD:   L2 + R2 + Select + Start
 * - XSTATION:  L2 + R2 + Select + X
 * - GUNCON:    A + Trigger + B
 */
uint8_t process_key_combo_reset(void) {
    static uint8_t  last_active  = 0;
    static uint16_t hold_time    = 0;
    uint8_t         triggered    = 0;
    // Validate combo only if it matches the controller type
    uint8_t valid_combo = (
        (data.id == ID_GUNCON_CTRL && data.switches == KEY_COMBO_GUNCON) ||
        (data.id != ID_GUNCON_CTRL && 
        (data.switches == KEY_COMBO_CTRL || data.switches == KEY_COMBO_XSTATION))
    );   
    if (valid_combo) {
        LED_COMBO();       // Turn on combo LED with RA4=HIGH and RA5=LOW for bicolor LED
        if (!last_active) {
            hold_time = 0;
            t0_ms_count = 0;
        }
        hold_time += t0_ms_count;
        t0_ms_count = 0;
        last_active = 1;
		// Hold threshold reached
        if (hold_time >= COMBO_HOLD) {
            triggered   = 1;
            hold_time   = 0;
            last_active = 0;
            LED_OFF(); // LED to off state after reset is triggered
        }
    } else {
        // Clear hold state if no valid controller/combo is active
        hold_time    = 0;
        t0_ms_count  = 0;
        last_active  = 0;
        LED_IDLE();	// LED to idle state when combo is released or invalid
        return 0;
    }
    return triggered; // Return 1 if combo was held long enough, else 0
}

// Blink combo LED to acknowledge that a reset was triggered
void reset_blink(void) {
    blink_active     = 1;
    blink_count      = 0;
    blink_timer_ms   = 0;
    blink_led_state  = 0;
    LED_OFF(); // RA4-RA5 Hi-Z to fully turn off LED
    TRISA4 = 0; // Set RA4 as output for idle/combo
    TRISA5 = 0; // Set RA5 as output for blink
}

// Combo reset handler with controlled duration and LED feedback
void reset_combo(uint16_t duration_ms, const char* label) {
    UART_print((uint8_t*)"\n\r");
    UART_print((uint8_t*)label);
    UART_print((uint8_t*)" Reset successful!\n\r");
    // Activate reset: change RESET pin from input to output, logic low (PORT is already 0)
    PS1_TRIS_IO &= ~_BV(PS1_RESET); 
    // Asynchronous LED blink
    reset_blink(); // Reset feedback
    // Timebase for hold reset
    for (uint16_t i = 0; i < duration_ms; i++) {
        __delay_ms(1);
    }
    // Change RESET back to input
    PS1_TRIS_IO |= _BV(PS1_RESET);
    // Delay to prevent MCU from resetting too early
    __delay_s(REBOOT_DELAY);
}

#ifdef DEBUG
/**********************************************************/
/* UART Debug Functions */

/* Initialize UART with TX on output RC3 */
void UART_init(void){
    // SETUP UART
    ANSELCbits.ANSC3 = 0; // TX set to digital I/O
    LATCbits.LATC3 = 0; // set output 0
    TRISCbits.TRISC3 = 0; // TX set to output
    RC3PPSbits.RC3PPS = 0x14; // TX = RC3

    SP1BRGL = 51; // 9615 baud
    SP1BRGH = 0; // 9615 baud
    
    TX1STAbits.SYNC = 0; // Asynch mode
    RC1STAbits.SPEN = 1; // Serial Port enable bit
    TX1STAbits.BRGH = 0; // low speed
    BAUD1CONbits.BRG16 = 0; // 8 bit baud rate
    BAUD1CONbits.SCKP = 0; // Idle state for TX high level
    CLKRCONbits.CLKRDIV = 0; // Fosc
    CLKRCONbits.CLKRDC = 2; // 50% duty cyle
    CLKRCONbits.CLKREN = 1; // Enable CLK reference
    
    TX1STAbits.TXEN = 1; // Enable transmitter  
}

/* Send byte and wait until transfer finished */
void UART_sendByte(uint8_t c){
    TX1REG = c;
    while(!TX1STAbits.TRMT);
}

/* Send string */
void UART_print(uint8_t *str){
    uint8_t i;
    for (i = 0; str[i] != 0 && i < 255; i++)
        UART_sendByte(str[i]);
}

/* Print hex byte in format 0x%02X */
void UART_printHex(uint8_t b){
    uint8_t low_nibble = b & 0x0F;
    uint8_t high_nibble = (b >> 4) & 0x0F;
    
    UART_print((uint8_t*)"0x");
    UART_sendByte(high_nibble >= 10 ? (high_nibble - 10 + 'A') : high_nibble + '0');
    UART_sendByte(low_nibble >= 10 ? (low_nibble - 10 + 'A') : low_nibble + '0');
}
#endif
