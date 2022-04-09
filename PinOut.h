
/* Mod 2019-05-04 PA0HPG
 * Digital Pin assignments for Teensy 3.2 boards 
 *
 * Teensy 3.2 boards have the following pin conventions :
 * Board layout is a 2x14 pin DIL pin board which is not enough
 * to connect to all pins of the Cortex M4
 * Some pinouts are in the middle of the board or have only
 * solder islands at the back. 
 *
 * Pins 13..23 have digital I/O as well analog input capabilties.
 *           Center : Aref, A10, A11 have pins on center board 
 * The backside has solder islands for Digital I/O D24..D33 +  
 *                                     GND + A12 and A13
 * USB is available on islands VUSB, D+,D- and GND,
 * below the Usb connector and can provide USB via a cable
 *
 * Pin # Function Description     Pin # Function Description
 * ----- -------- -----------     ----- -------- -----------
 *  1    GND                      28    Vin      Input Voltage (5V)
 *  2    RX1      [Serial in ]    27    An_GND   Analog GND
 *  3    TX1      [Serial out]    26    3.3V    
 *  4    D2       Rotary I        25    D23       
 *  5    D3       Rotary Q        24    D22      Paddle left
 *  6    D4       Straight key    23    D21      Audio
 *  7    D5       SPI+SD          22    D20      Audio
 *  8    D6       Rotary Switch   21    D19      TX1 out
 *  9    D7       Audio           20    D18      TX0 out
 * 10    D8       SD              19    D17      I2C
 * 11    D9       Audio           18    D16      I2C
 * 12    D10      SPI             17    D15      Paddle right (dash)
 * 13    D11      Audio           16    D14      SPI SCLKPIN
 * 14    D12      SPI             15    A1       Volume
 *              5 Backside Pins :
 *      -O----O----O-----O-------O----
 * 14   Vbat 3.3V GND Program A14/DAC  15
 *                            Tone out 
 * Pin   A10 : Switch input 1..5 (resistor network with 5 input) 
 * Pin   A14 : DAC output for sidetone
 * Backsideconnections
 * Pin   24  D24  : Led light Command mode
 * Pin   25  D25  : Led light sw1 
 * Pin   26  D26  : Led light sw2
 * Pin   27  D27  : Led light sw3
 * Pin   28  D28  : Led light sw4
 * Pin   29  D29  : Keyboard data A 
 * Pin   30  D30  : Keyboard clock (should be interrupt capable pin)
 * 
 * 
 *    Teensy audio shield connected pinouts, X = connected
 * Pin # X Function Description     Pin # Function Description
 * -----   -------- -----------     ----- -------- -----------
 *  1    GND                      28    Vin
 *  2    RX1                      27    An_GND
 *  3    TX1                      26    3.3V
 *  4    D2                       25    D23
 *  5    D3                       24    D22  
 *  6    D4         X Memory      23    D21        X Audio
 *  7    D5         X SPI SD+Mem  22    D20        X Audio
 *  8    D6                       21    D19
 *  9    D7         X Audio       20    D18
 * 10    D8         X SD card     19    D17        X I2C share
 * 11    D9         X Audio       18    D16        X I2C share
 * 12    D10        X SPI SD+Mem  17    D15
 * 13    D11        X Audio       16    D14
 * 14    D12        X SPI SD+Mem  15    A1         X Volume
 * 
 * 
 * Function	Teensy Pins Used      Shareable
 * --------	----------------      ---------
 * Audio	9,11,13,18,19,22,23   18, 19 (other I2C chips)
 * Volume Pot	15 (A1)	              -
 * SD Card	7, 10, 12, 14	      7, 12, 14 (other SPI chips)
 * Memory Chip	6, 7, 12, 14	      7, 12, 14 (other SPI chips)
 * 
 */

// Digital and analog I/O pin assignments
                      // Teensy 3.2 functionality
#define TX0  0        // Digital 1 and 2 are connected to USB input
#define RX0  1
#define D0   0
#define D1   1
#define D2   2        
#define D3   3        // PWM capable Output
#define D4   4        // PWM capable Output
#define D5   5        // PWM capable Output
#define D6   6        // PWM capable Output
#define D7   7
#define D8   8
#define D9   9        // PWM capable Output
#define D10 10        // PWM capable Output
#define D11 11
#define D12 12
#define D13 13
#define D14 14
#define D15 15
#define D16 16
#define D17 17
#define D18 18
#define D19 19
#define D20 20        // PWM capable Output
#define D21 21        // PWM capable Output
#define D22 22        // PWM capable Output
#define D23 23        // PWM capable Output
#define D24 24
#define D25 25        // PWM capable Output
#define D26 26
#define D27 27
#define D28 28
#define D29 29
#define D30 30
#define D31 31
#define D32 32        // PWM capable Output
#define D33 33
#define D34 34
#define D35 35

/* Analog pinout defines
#define A5   5
#define A6   6
#define A7   7
#define A8   8
#define A9   9
#define A10 10
#define A11 11
#define A12 12
#define A13 13
#define A14 14
#define A15 15
#define A16 16
#define A17 17
#define A18 18
#define A19 19
#define A20 20
*/

#define TRUE  1
#define FALSE 0

/* Pins - Configured for Teensy 3.2 board with additional  Audio shield */

#define paddle_left   D0
#define paddle_right  D21
#define tx_key_line_1 0       // (high = key down/tx on)
#define tx_key_line_2 0
#define tx_key_line_3 0
#define tx_key_line_4 0
#define tx_key_line_5 0
#define tx_key_line_6 0

#define sidetone_line A14       // connect a speaker for sidetone

// Mod. PA0HPG 2016-04-21 For Push Buttons with a led light
// Note : 1.5 K resistors between led lines control the current
// Function : switch_led(button3_led, ON) ;  // Switch led on button3 on

#ifdef FEATURE_BUTTON_LIGHTS
  #define ON                 HIGH
  #define OFF                LOW
  #define command_button_led D24  
  #define rotary_button_led  D24  // == command mode button
  #define cw_led             D24
  #define button1_led        D24
  #define button2_led        D25
  #define button3_led        D26
  #define button4_led        D27
  #define button5_led        D28
#endif // FEATURE_BUTTON_LIGHTS


#ifdef TFT_DISPLAY            // SPI TFT LCD graphical display board 
#define SPI_TFT 1
//-----------------------------------------------------------------------------
// LCD Type ST7735 or QDTech (QDTech display untested)
#define LCD_QDTECH         0  // QDTech 128x160 pixel LCD board using a Samsung S6D02A1 chip.
                              // Else 128x160 pixel LCD board using an ST7735 chip.                              
#define ST7735_BLACKTAB    1  // If ST7735 then select one of these, depending on
#define ST7735_REDTAB      0  // whether the TFTs plastic wrap has a Red, Green or Black
#define ST7735_GREENTAB    0  // Tab when new.  Else, experiment - if the display has wrong
                              // colours or extra 'random' pixels on the top & left.
#define R_B_COLOUR_INVERT  1  // Some 3rd party ST7735 displays invert the colours

// SPI connections for 1.8" TFT display : See also CWReceive.h, moved from there
#define  RST_TFT          1
#define  CS_TFT           2
#define  DC_TFT           3
#define  MOSI_TFT         7   // Alt HW MOSI - Audio shield uses the primary SPI pins
#define  SCLK_TFT        14   // Alt HW SCLK - Audio shield uses the primary SPI pins
#endif

