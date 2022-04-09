/*
 * 
 * PA0HPG Antenna Rotator Controller.
 * Local/Remote antenna rotator controller for Ham-Radio antennas.
 * Set antenna bearing by local control or by means of remote antenna 
 * control software via a USB link. 
 *
 * Author                 : Harm Paas PA0HPG
 * Initial Version V-1.00 : 2021-01-12 Harm Paas PA0HPG 
 * Last Update     V-1.38 : 2022-01-31 Harm Paas PA0HPG 
 *
 * Copyright (C) 2021,2022  Harm Paas, PA0HPG, Zuidlaren, The Netherlands
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Program description
 *
 * TV-type antenna rotators like "Channelmaster" and "Stolle" rotors are able to rotate 
 * a variety of light-weight HF and UHF radio-amateur antennas.
 * In most cases the control of these rotators is done by means of a steering knob.
 * A motor in the control device runs at the same speed as the motor in the rotator 
 * and sets the antenna bearing readout light on the scale of  control device in the same direction.
 * For several reasons the synchronisation between the antenna position on the scale
 * becomes misaligned with the real bearing position, mostly because the two motors do not
 * run at the same speed. 
 * This rotator controller replaces the control unit of these type of antenna rotators and 
 * offers remote computer control via the Yeasu GS232 rotor control protocol.
 * The unit offers local 3-button control as well as remote computer control via a 
 * serial USB line for programs like N1MM, PstRotator, LOG4OM, etc.
 *
 * The Arduino-MEGA2560 based antenna rotator control unit replaces the original control unit
 * and offers several enhancements compared with the original control unit.
 * The control unit not only shows the real antenna bearing on a TFT color display 
 * but can also be connected to a computer via a USB serial link for remote antenna position 
 * control and position display from the antenna on the computer.
 * An azimuthal map projection shows the direct position of the antenna on the world map with 
 * respect to the location of the antenna.
 * New azimuthal map projections can be made by using the utilities in the Additional Software
 * section below.
 *
 * For remote control the Yaesu GS232 compatible command language has been used.
 * The real antenna bearing angle is obtained by adding a position indicator to
 * the motor control unit consisting of a 10 turn potentiometer which is attached
 * to the gearbox of the unit by means of an extra gear. 
 * In case the control cable is broken a warning will be displayed on the control unit.
 * If the antenna becomes misaligned on the mast it is possible to correct this
 * by changing the deviation with respect to the true North. The value will be stored
 * in EEPROM with the new setting. In The Netherlands it is favourable to set the 
 * mechanical endstop of the rotor unit in direction West. Therefore the deviation position
 * of 270 degrees is the default value.
 * For manual control three pushbuttons for Clockwise, CounterClockwise
 * and Stop are used. A short Push will turn the antenna one degree in the 
 * CW or CCW direction. Pressing the CW or CCW button for more than 0.5 seconds will 
 * put the antenna in the autorotate mode until the endstop is reached or by hitting the
 * Stop button before. 
 * The Stop button will set a new deviation value when it is depressed for more than 4 seconds.
 *
 * In the program an averaging (EMA) algoritm of the antenna potentiometer angle value is used to 
 * suppress noise and jitter on the indicated antenna position.
 *
 * Motor Control
 * -------------
 * Because the hardware for controlling the left-right AC motor is done
 * by two relays which turn the rotor at full speed into the left or right direction the position
 * control can only be done with a so called "bang bang" type motor control.
 * A bang-bang controller is a feedback system controller that switches abruptly between 
 * two states. By nature this type of control causes the intended antenna position
 * may differ from the real position, this to avoid oscillations around
 * the new setpoint.
 * In the "dead-band" around the new position will, once arrived, no position
 * update take place.
 * The dead-band region is controlled by a predefined constant.
 *
 * A description of this type of controllers can be found at : 
 * user.ece.cmu.edu/~koopman/ece348/lectures/22_controls_handouts.pdf
 * (sheet 14 and following). 
 * 
 * A more elaborate PID type of control described in the article above can be 
 * implemented with the Arduino AutoPID library. However, PID control can 
 * only be used when a proportional type of motor control is implemented 
 * in the controller hardware. For this type of control a DC rotator motor
 * is much easier to control proportionally than an AC motor type.
 *
 * Pushbutton control
 * ------------------
 * Albert van Dalen's Switch and Button control library has been used 
 * for pushbutton control. For details see www.avdweb.nl : 
 * "Arduino switch and button library with Short/Long Press, Double Click and Beep" 
 * The "Switch" library files have been placed in the source 
 * directory of the program for convenience.
 *
 * Additional software 
 * -------------------
 * Create Azimuthal Equidistant map from QTH locator
 *
 * http://ok2pbq.atesystem.cz/prog/ae_map.php
 * Map needs to be size resampled in a 180x180 pixel map
 * and placed in a color bitmap array.
 *
 * png2c : Create c program bitmap from a png image:
 * https://github.com/oleg-codaio/png2c
 *
 * Color picker : www.barth-dev.de/online/rgb565-color-picker/ 
 *
 * Hardware :
 *
 * 1) Arduino Mega 2560R3
 * 2) HX8537 480x320 pixel 3.2 Inch TFT Display Shield
 * 3) 3 momentary push buttons with LED ring lights.
 * 4) 10 turn 2 KOhm Potentiometer in rotator housing with gear
 * 5) Arduino Dual relay board
 * 6) Rotator transformer from the original rotator controller
 *
 */

#define VERSION "V1.38"

#define MYCALL       "PA0HPG"   // You can place your own Call here
#define MYLOCATOR    "JO33ic"   // You can place your own locator in here


// Language options : Enable one of the three following

#define NL true           // Dutch text display
//#define EN true         // Enable for English text display
//#define DE true         // Enable for German text display

#define WORLDMAP true     // Selects worldmap, disable for a local map

#define MINPOT      95    // Potentiometer value at minimum before mechanical stop
#define MAXPOT     706    // Potmeter value at maximum before mechanical stop

#define DEADBAND     3    // Dead-band window: Antennabearing precision is +/- DEADBAND degrees

//#define DEBUG true     // Normally disabled.

#include <Arduino.h>
#include <avr/pgmspace.h>  // Program space data storage for bitmap array
#include <TimerOne.h>      // ISR library
#include <EEPROM.h>
#include <TFT_HX8357.h>    // ARDUINO HX8537 MEGA2560 TFT Hardware-specific library

#include "PinOut.h" ;      // IO port definitions D0..D* A0.. A*
#include "Switch.h"        // See also : ww.avdweb.nl
#include "Free_Fonts.h"    // Used with the TFT_HX8357 driver

// Display texts
#ifdef NL // Dutch text display
#define POSITION     "AZIMUTH"
#define DEPART       "VERTREK"
#define GOTO         "EINDPUNT"
#define DEVIATION    "DEVIATIE"
#define NORTH        "     Noord"
#define NE           "Noord-Oost"
#define EAST         "      Oost"
#define SE           "Zuid-Oost"
#define SOUTH        "      Zuid"
#define SW           " Zuid-West"
#define WEST         "      West"
#define NW           "Noord-West"
#define N "N"
#define E "O"
#define S "Z"
#define W "W"
#define CABLE        "KABEL"
#endif

#ifdef EN // English text display
#define POSITION     "AZIMUTH"
#define DEPART       "DEPART "
#define GOTO         "ENDPOINT"
#define DEVIATION    "DEVIATION"
#define NORTH        "     North"
#define NE           "North-East"
#define EAST         "      East"
#define SE           "South-East"
#define SOUTH        "     South"
#define SW           "South-West"
#define WEST         "      West"
#define NW           "North-West"
#define N "N"
#define E "E"
#define S "S"
#define W "W"
#define CABLE        "CABLE"
#endif

#ifdef DE // German text display
#define POSITION     "AZIMUTH"
#define DEPART       "ANFANG"
#define GOTO         "ENDZIEL"
#define DEVIATION    "DEVIATION"
#define NORTH        "     Nord"
#define NE           "Nord-Ost"
#define EAST         "      Ost"
#define SE           "Sud-Ost"
#define SOUTH        "      Sud"
#define SW           "Sud-West"
#define WEST         "      West"
#define NW           "Nord-West"
#define N "N"
#define E "O"
#define S "Z"
#define W "W"
#define CABLE        "KABEL"
#endif

#define BLACK     0x0000  // Standard colors
#define BLUE      0x001F
#define RED       0xF800
#define GREEN     0x07E0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
#define WHITE     0xFFFF
#define ORANGE    0xFF00

#define GREY      0x7BEF  // Other color settings
#define DBCOLOR   0x0002  // Dial background color
#define SCREENBG  0x4BA9  // Screen background color 
#define NAMEBG    0x42E9  // Name background
#define LIGHTBLUE 0x07FF  // RGB 0,255,255
#define BLUE1     0x033F  // RGB 0,100,255
#define DARKBLUE  0x2A61  // Compass rose dial color
#define PURPLE    0xF80F  // RGB 255,0,127
#define LTORANGE  0xFC00  // RGB 255,128,0
#define DARKRED   0xA800

#define ON  1
#define OFF 0

#define CENTRE 240
#define LEFT     3
#define RIGHT  480

#define SHORT_PUSH      20 // 20 ms short push button
#define LONG_PUSH     1000 // 1000 ms yields a long push
#define VERYLONG_PUSH 3000 // 3000 ms yields a very long push

#define ACTSHORT  500      // Activate motor for at least ACTSHORT milliseconds

// Digital lines input and output
#define CWmotor    D2    // Motor relay outputs
#define CCWmotor   D3 

// Buttons and  button leds
#define CCWbutton  D4    // Green - Backboard/Left turn
#define CCWled     D5
#define CWbutton   D6    // Red   - Starboard/Right turn
#define CWled      D7
#define STOPbutton D8    // Blue  - Stop  
#define STOPled    D9

#define PotVal     A0    // Potmeter input

#define EEPROM_MagicNumber   42 // A Magic number

#define EEPROM_Magic          0 // EEProm Locations of stored variables set
#define EEPROM_Deviation      4 // Position of the rotor mechanical endstop 
#define EEPROM_Map           12

#define MAXDEGREES 360  // Antenna bearing range is from 0 to 359 degrees

#define  FormatData(x) strcpy_P(dataBuffer, PSTR(x))

enum Direction {CW,CCW,STOP};
enum Direction Dir, OldDir;

Switch CWButton   = Switch(D6); // Internal pullup is default, PULLUP_HIGH 
Switch CCWButton  = Switch(D4); 
Switch StopButton = Switch(D8); 

int Heading    = 0 ; // Endposition where to go to : Set in degrees 0..360
int OldHeading = 0 ; // Former Heading

volatile int AntennaBearing    =  0 ;    // Current antenna bearing 0..360 deg. Follows the Heading.
volatile int OldAntennaBearing = -1 ;    // Former antenna bearing.

volatile int PotmeterPosition  = 0 ;  // True rotor position. Potmeter value somewhere between 0..1023
int PotmeterPosition_Tm1       = 0 ;  // Previous rotor position value.
int PotmeterHeading            = 0 ;  // Potmeter value Heading with deviation taken into account 

int Deviation         = 270;    // Deviation in degrees with respect to the True North.
int LoPot             = MINPOT ;// Minimum value of potmeter before mechanical endstop. DO NOT PASS
int HiPot             = MAXPOT ;// Maximum value of potmeter before mechanical endstop. DO NOT PASS


#define INTERRUPT_TIMER 1000       // 1 ms interrupt = 1000 usec

volatile uint16_t seconds = 0 ; // Timer for one second events
volatile uint16_t msec100 = 0 ; // Timer 100 ms events
volatile uint16_t msec10  = 0 ; // Timer  10 ms events

extern uint8_t BigFont[];
extern uint8_t SmallFont[];
extern uint8_t SevenSegmentFull[];

int Compass_marker[360][4]; // Calculated triangle markers for compass bearing and heading

const int  XC       = 320; // Compass Center
const int  YC       = 160;
const int  dm       = 130; // Radius of compassc circle

struct EEPROMValue {   //EEPROM Data Structure mapping long words on 4 bytes
  union {
    long Value;
    struct {
      unsigned char Byte1;
      unsigned char Byte2;
      unsigned char Byte3;
      unsigned char Byte4;
    }
    __attribute__((packed));
  }
    __attribute__((packed));
}__attribute__((packed));

inline long ReadEEPROMValue(int16_t EEPROMStartAddress);              // Reads the values from EEPROM
                                                                      // Bearing, Deviation, etc
inline void SaveEEPROMValue(int16_t EEPROMStartAddress, long Value);  // Save Value to EEPROM
                                                                      // Starting from the given
                                                                      // Start Address (Long == 4 Bytes)
inline void ResetInputBuffer(void);
inline void DrawHead(int x2, int y2, int x1, int y1, int h, int w, int COLOR);
inline void DisplayUserEntry(int x, int y, String userData); 

static boolean 	StopOnce;

boolean AutoRotate  = false;  // Manual auto rotate CW or CCW until STOP pressed
boolean Running     = false;
boolean NewPosition = false;  // New position set
boolean InPosition  = false;  // if true, running stops

char dataBuffer[60];
char formattedDataBuffer[3];

#ifdef WORLDMAP
#include "Map_JO33ic_180x180.h" ;  // World map
uint8_t *Map = Map_JO33IC_180x180; // Pointer for Display map
boolean Worldmap  = true ;         // World map is default
#else
#include "Europe_180x180.h" ;      // Local map of Europe
uint8_t *Map = Europe_180x180a;    // Pointer for a Local Display map
boolean Worldmap  = false ;
#endif

int dx, dy;                                 // Bearing indicator marker position
int fdx, fdy;                               // Former bearing pos end 
int bearingposdxbegin, bearingposdybegin;   // Bearing marker pos begin
int fbposdxbegin, fbposdybegin;             // Former bearing values used for erase

int posdx,posdy;              // New Heading position set by input
int fposdx, fposdy;           // Former Heading position set
int posdxbegin, posdybegin;   // Heading triangle begin marker of the heading indicator marker
int fposdxbegin, fposdybegin; // Former Heading triangle markerpoint of the heading indicator marker

int dxOuter, dyOuter, dxinner, dyinner;

int CWMotortravel  =  7 ;       // CW direction number of potmeter values rotator travels after power off
int CCWMotortravel =  5 ;       // CCW number of potmeter values


TFT_HX8357 tft = TFT_HX8357();  // Invoke the TFT display

/**** Begin EEPROM read/write routines ****/

// Read and return the stored value specified in the EEPROM Start Address
long ReadEEPROMValue(int16_t EEPROMStartAddress) {
  volatile EEPROMValue eepromVal;
  eepromVal.Byte1 = EEPROM.read(EEPROMStartAddress);
  eepromVal.Byte2 = EEPROM.read(EEPROMStartAddress+1);
  eepromVal.Byte3 = EEPROM.read(EEPROMStartAddress+2);
  eepromVal.Byte4 = EEPROM.read(EEPROMStartAddress+3);
  return eepromVal.Value;
}
/* Store the specified value in the EEPROM  from Start Address
 EEPROM Write will only happens when the stored value and new value are different.
 This will save the number of Writes to the EEPROM.*/
void SaveEEPROMValue(int16_t EEPROMStartAddress, long Value) {
  volatile EEPROMValue eepromVal;
  eepromVal.Value = ReadEEPROMValue(EEPROMStartAddress);
  if(eepromVal.Value != Value) {
    eepromVal.Value = Value;
    EEPROM.write(EEPROMStartAddress,eepromVal.Byte1);
    EEPROM.write(EEPROMStartAddress+1,eepromVal.Byte2);
    EEPROM.write(EEPROMStartAddress+2,eepromVal.Byte3);
    EEPROM.write(EEPROMStartAddress+3,eepromVal.Byte4); 
  }
}

// Setup EEprom : Initialize eeprom values
void Setup_EEPROM() {
  SaveEEPROMValue(EEPROM_Magic, EEPROM_MagicNumber);
  SaveEEPROMValue(EEPROM_Deviation, Deviation);
  SaveEEPROMValue(EEPROM_Map, Map);    // Store Map address in EEPROM
  Serial.println("EEPROM values initialized");
}

// Read EEPROM values
void Read_EEPROM() {
  long Magicnumber ;
  Magicnumber = ReadEEPROMValue (EEPROM_Magic);
  if ( Magicnumber != EEPROM_MagicNumber) {  
    Serial.println("Magic number did not match, setting up EEPROM values initially");
    Setup_EEPROM();
  }

  Serial.print("Reading EEPROM values ... ");
  Deviation   = ReadEEPROMValue(EEPROM_Deviation);
  Serial.println("Done");
}

/********************************************
 *   Basic string variables and functions   *
 ********************************************/
String inputString = "";         // String holding incoming data
boolean stringComplete = false;  // Set at newline -> ready processing

int str_to_int(String input) {
  char charBuf[10];
  input.toCharArray(charBuf, 10);
  int value = atoi(charBuf);
  return value;
}

// PushButton Led control
// All three switches with colored led ring
void Set_CWled(int onoff){
  digitalWrite (CWled, onoff);   // Red Led
}

void Set_CCWled(int onoff){
  digitalWrite (CCWled, onoff);  // Green Led
}

void Set_STOPled(int onoff){
  digitalWrite (STOPled, onoff); // Blue Led
}

// Setup Rotator motor relays
void Setup_Relays (){
  pinMode(CWmotor,   OUTPUT);
  pinMode(CCWmotor,  OUTPUT);
  digitalWrite(CWmotor, HIGH); // Relay signal HIGH == Motor OFF
  digitalWrite(CCWmotor,HIGH);
}

// set turn direction CW ( Right )
void TurnCW() {
  Set_CCWled(OFF);
  Set_STOPled(OFF);
  digitalWrite(CCWmotor,HIGH);
  if ( Running == true ) delay(1000) ; // wait one second to finish travel
  digitalWrite(CWmotor, LOW);   // Power CW motor direction ON
  Set_CWled(ON);
  noInterrupts();   // Do not interfere with new position updates
  delay(ACTSHORT);  // Set motor at least on for a short period
  interrupts();
  StopOnce = true ; // If position reached display motor off command once
}

// set turn direction CCW ( Left )
void TurnCCW() {
  Set_CWled(OFF);
  Set_STOPled(OFF);
  digitalWrite(CWmotor,HIGH);
  if ( Running == true ) delay(1000) ;  // Wait one second
  digitalWrite(CCWmotor, LOW); // Power CCW motor direction ON
  Set_CCWled(ON);
  noInterrupts();
  delay(ACTSHORT);       // Set motor at least for a short period on
  interrupts();
  StopOnce = true ; // If position reached display motor off command once
}

// Motor stop
void Stop_Motor() {
  digitalWrite(CCWmotor,HIGH);
  digitalWrite(CWmotor, HIGH);
  if ( Running == true ) delay(1000) ; // Wait until motor travel finished
  Set_CWled(OFF);
  Set_CCWled(OFF);
  Set_STOPled(ON);
  Running = false; 
}


// Display direction CW CCW or STOP
void Display_Dir(char *Rotation) {
  DrawFrame(RIGHT-94, 16,RIGHT-14 , 48, 10, BLUE,DARKBLUE);    // Scrub frame
  tft.setTextColor(LIGHTBLUE,DARKBLUE);
  tft.drawString(Rotation, RIGHT-90, 20, 4); 
}


/* Initialize rotator */
void Rotator_init(){
  Setup_Relays();     // Setup motor hardware
  Stop_Motor();      // Command stop
  StopOnce = false ;
  return (0); 
}

/* 
 * SetCourse :  
 * Determine CW or CCW motor direction with regard to the the Deviation value.
 * In the rotator software the Deviation parameter is a value  between the position of 
 * the Endstop of the Antenna with respect to the True North.
 * The mechanical endstop of the rotator cannot be passed.
 * In The Netherlands it is useful to set the endstop of the rotor Due West : 270 degrees
 * As the daily HF propagation change starts with the opening of the stations in the East
 * and the MUF will move to the West during daytime towards Canada and the US the shortest path
 * of the beam from East to West will be via the North.
 * We define the 4 quadrants of the compass rose as following : 
 * 0..90 = 1, 90 ..180=2, 180..270=3 and 270..360=4 
 * As the endstop is set by default at 270 Degrees West and cannot be past it also means that
 * reaching quadrant 3 from quadrant 4 can only be done in the CCW direction via North.
 *
 * If needed the Deviation value can be changed and stored in EEPROM.
 * When the antenna has been aligned properly the value does not have to be changed.
 * If it becomes misaligned it is possible to correct the deviation value
 * by means of a command or pressing the STOP button for more than 4 seconds.
 * This avoides climbing into the mast and turn the antenna mounting direction
 * mechanically. After adjusting the deviation value in the rotator deviation setting
 * the position indicator wil point in the proper compass bearing direction again.
 *
 */

 Direction Change_Course( int NewPos) {
   int Course ;
   Direction dir ;

   Course = NewPos - Deviation; // Set Course and convert it into a Potmeter position value

   if ( Course <   0 ) Course = Course + 360 ;
   if ( Course > 360 ) Course = Course - 360 ;

   // Calculate CW or CCW turn with respect to potmeter position
   // Convert Course into potmeter value corercted with the Deviation value.
   PotmeterHeading  = int ( (float)Course/360.0 * ((float) (HiPot - LoPot)) + float(LoPot) ); // New Potmeter position value

   noInterrupts();                  // Begin critical section
   if ( PotmeterPosition < PotmeterHeading) dir = CW ;
   else dir = CCW ;
   interrupts();                    // End critical section

   //Serial.print(" Change_Course: PotmeterPosition "); Serial.print( PotmeterPosition ); // TEST ONLY
   //Serial.print(" PotmeterHeading "); Serial.print( PotmeterHeading );
   //if (Dir == CW )Serial.println(" CW");  
   //else Serial.println(" CCW");  

  return (dir);
}

/*
 * Rotate_To(degrees) : Set new input in degrees and show on display
 *
 * Rotate_To depends on 4 variables for moving into the new
 * position :
 * AntennaBearing : The actual position of the Antenna in degrees
 * NewPos         : The new desired position of the Antenna in degrees
 * Deviation      : The Deviation of the Antenna with respect to the true North.
 *                  The rotor system is capable of dynamically changing
 *                  the deviation should the antenna become misaligned, this avoids 
 *                  an adjustment of the antenna and its rotor on the mast. The default 
 *                  endstop position is 270 degrees(West). 
 *                  Note that passing this point will destroy the position indicator 
 *                  which is connected inside the motor gear.
 *                  The CW or CCW movements are with respect to the Deviation parameter.
 */

void Rotate_To(int NewPos) {

  Dir =  Change_Course(NewPos);
  if( Dir == CW ) rotate_right();  // CW turn , increase AntennaBearing   
  else            rotate_left();

  StopOnce = true ; // Once in position trigger a stop signal
  
  // Display old heading and new heading. Fixed field, 3 numbers wide 
 
  tft.setFreeFont(FSB12);                       // Display the current position set
  tft.setTextColor(LIGHTBLUE,NAMEBG);    
  tft.setTextPadding(tft.textWidth("360", 2)) ; // Calculate text padding width
  
  sprintf(formattedDataBuffer, FormatData("%3d"), Heading);    // Display the current
  tft.drawString(formattedDataBuffer, 84,223, 2);
  sprintf(formattedDataBuffer, FormatData("%3d"), AntennaBearing); // and the former position
  tft.drawString(formattedDataBuffer, 84,203, 2);                  

  Draw_Heading_Marker(Heading, TFT_RED);              // Set marker bar and go to the new position
  //OldHeading = Heading;
  //OldHeading = AntennaBearing;
  
  tft.drawNumber(Deviation,84,240, 2); // Deviation in Red. If Rotor aligned at WEST, value is 270 degrees
  tft.setTextPadding(0) ;              // Set padding off again
}

// Update_Compass : Update 7 Segment display and Antennabearing marker

void Update_Compass(){
  tft.setTextColor(LIGHTBLUE,DARKBLUE); 
  sprintf(formattedDataBuffer, FormatData("%03d"),AntennaBearing);
  tft.drawString(formattedDataBuffer, LEFT+20, 135, 7); //  7 Segment Full size ciphers

  Draw_Bearing_Position (AntennaBearing, TFT_YELLOW);
  
  send_az_position(); // Send position to USB serial line
}

/*
 * rotate_stop : Switch motor off
 */
void rotate_stop() {
  if (InPosition == false) {
    Stop_Motor();
    Display_Dir("STOP ");
    Display_NESW(); 
    //OldAntennaBearing = AntennaBearing;   
    PotmeterHeading  = PotmeterPosition;
    InPosition  = true;
    NewPosition = false ;
    StopOnce    = false ;
    //    DrawInitialScreen(); // Get rid of all debris on compass
    Update_Compass(); 
  }
}

/* Back off.
 * Emergency procedure.
  When passing the minimum or maximum value of the potentiometer crawl back to safe bearing angles
 */
void Back_off( Direction dir) {
  Display_Dir("END  ");
  if ( dir == CW ) {
    Serial.println("Minimum Endstop exceeded, backing off CW "); 

    while ( PotmeterPosition <= LoPot ) {
      TurnCW(); delay (ACTSHORT) ;
    }
  }
  else  {
    Serial.println("Maximum Endstop exceeded, Backing off CCW "); 
    while ( PotmeterPosition >= HiPot ){
      TurnCCW(); delay (ACTSHORT) ;
    }
  }
  PotmeterHeading  = PotmeterPosition;
  rotate_stop(); // Stop when position is in safe region
}

/*
 * DrawFrame() : Draw a canvas frame with rounded corners and color fill
 * Call : DrawFrame(X0,Y0,X1,Y1,Roundcorner, Bordercolor,Fillcolor)
 * Where X0,Y0 = left upper corner, X2,Y2 Right lower corner
 *       Roundedcorner = rounding in pixels ( 5..20 is normal)
 *       Bordercolor and Fillcolor : line and fill colors
 */

void DrawFrame(int X0, int Y0, int X1, int Y1, int Roundedcorner, int Bordercolor, int Fillcolor){
  int Width, Height ;
  Width = X1 - X0 ; Height = Y1 - Y0 ;
  tft.drawRoundRect(X0, Y0, Width, Height, Roundedcorner, Bordercolor);       // Draw frame
  tft.fillRoundRect(X0+1, Y0+1, Width-2, Height-2, Roundedcorner, Fillcolor); // Fill frame
}

// Display Control mode in control frame
void Display_Control(char *Control) {
  DrawFrame(RIGHT-104, 280, RIGHT-10, 315, 10, BLUE, DARKBLUE); // Scrub frame
  tft.setTextColor(LIGHTBLUE,DARKBLUE);
  tft.drawString(Control,  RIGHT-100, 285, 4); 
}


// Pushbutton handling
// Using avdweb Switch library for button control
//
// Callback functions
void CWButtonCallback(void* s){
  int NewPos ;
  if ( s == "short" ) { // short push
    Display_Control("Manual");
    NewPos = Heading + 1 ;
    if (NewPos >= MAXDEGREES) NewPos = NewPos - MAXDEGREES; //  0..359 degrees
    if (NewPos < 0          ) NewPos = NewPos + MAXDEGREES;
    if (NewPos != Heading) {         // If changed, start turning
      Heading = NewPos;
      //OldHeading = AntennaBearing ;
      NewPosition = true ;   // New heading has been set
      Rotate_To(Heading); // Go directly to new position
    }
  }
  if ( s == "long" ) { // Long push
    Stop_Motor();
    AutoRotate = true ;             // rotate CW until Stop
    Display_Control("Auto");
    Dir = CW ;
    Heading=Deviation-3 ;       // Rotate until max Endstop
    //OldHeading = AntennaBearing ;
    NewPosition = true ;   // New heading has been set
    Rotate_To(Heading);
  }
}

void CCWButtonCallback(void* s){
  int NewPos ;
  if ( s == "short" ) { // short push
    Display_Control("Manual");
    NewPos = Heading - 1 ;
    if (NewPos >= MAXDEGREES) NewPos = NewPos - MAXDEGREES; //  0..360 degrees
    if (NewPos < 0          ) NewPos = NewPos + MAXDEGREES;
    if (NewPos != Heading) {         // If changed, start turning
      Heading = NewPos;
      //OldHeading = AntennaBearing ;
      NewPosition = true ;   // New heading has been set
      Rotate_To(Heading); // Go directly to new position
    }
  }
  if ( s == "long" ) { // Long push
    Stop_Motor();
    AutoRotate = true ;
    Display_Control("Auto");
    Dir = CCW ;
    //OldHeading = AntennaBearing ;
    Heading=Deviation+3 ;       // Rotate CCW until minimum Endstop
    NewPosition = true ;   // New heading has been set
    Rotate_To(Heading);
  }
}

void StopButtonCallback(void* s){
  if ( s == "short" ) { // short push
    Display_Control("Manual");
    AutoRotate = false ;// Stop Auto Rotate
    Heading = AntennaBearing ;
    Dir = STOP ;
    rotate_stop();
  }
  if ( s == "long" ) { // Long push. 
    Serial.print("New deviation value : "); Serial.println(AntennaBearing);
    Deviation = AntennaBearing ;	    
    SaveEEPROMValue(EEPROM_Deviation, Deviation);               // Store in EEPROM
    sprintf(formattedDataBuffer, FormatData("%3d"), Deviation); // Display new value
    tft.setTextColor(TFT_RED,NAMEBG);
    tft.drawString(formattedDataBuffer, 84,240, 2);
    Display_Control("Stored");
    delay(2000);
    Display_Control("Manual");
  }
}


// 
// Initialize button hardware and button callback functions
void Setup_Buttons(){

  CWButton.setPushedCallback(&CWButtonCallback, (void*)"short");    // Connect the callback functions
  CCWButton.setPushedCallback(&CCWButtonCallback, (void*)"short");
  StopButton.setPushedCallback(&StopButtonCallback, (void*)"short");

  CWButton.setLongPressCallback(&CWButtonCallback, (void*)"long");     // Connect callback functions
  CCWButton.setLongPressCallback(&CCWButtonCallback, (void*)"long");   // for Long push
  StopButton.setLongPressCallback(&StopButtonCallback, (void*)"long"); // set 3 seconds long push for stop

  CWButton.longPressPeriod = 500;     // 500 ms long push for setting CW direction
  CCWButton.longPressPeriod = 500;    // 500 ms long push for setting CCW direction
  StopButton.longPressPeriod = 4000;  // 4000 ms long push for setting new Deviation value  

  pinMode(CWled,   OUTPUT); // Set Output LED pins
  pinMode(CCWled,  OUTPUT);              
  pinMode(STOPled, OUTPUT);             

}

/* 
 * General interrupt service routine
 * This routine services the position updates of the AD converter,
 * the pushbuttons and the Led control.
 * AD converter sampling rate is 10 HZ. This is fast enough
 * to follow the antenna motor movements accurately.
 *
 * The routine also periodically polls the CCW CW and STOP buttons
 * All buttons are polled at 10 ms using the Switch poll() call
 * Button states are debounced. After a short or long push
 * the callback function belonging to the push buton is called
 * to serve the button.
 */

void Service_Control(void){
  msec10++;
  if ( msec10 > 10 ) { // 10 milliseconds past, button service time
    msec10 = 0;
    CWButton.poll();   // Poll the three buttons,
    CCWButton.poll();  // Callbacks will service the buttons
    StopButton.poll();
  }

  msec100++;
  if ( msec100 > 100 ) { // 100 milliseconds past. Sample AD converter potmeter position
    msec100 = 0 ;
    PotmeterPosition = analogRead(PotVal);  // Read out analog data Pin A0 : 10 bits ( 0V =0 , 5V = 1023)
    PotmeterPosition = mean_Avg(PotmeterPosition, PotmeterPosition_Tm1); // Calculate running average
    PotmeterPosition_Tm1 = PotmeterPosition ;
    Getposition();
    if (AntennaBearing != OldAntennaBearing) {
      Update_Compass();
      Display_NESW();   // Set direction as clear text North, NE, E , etc.
      OldAntennaBearing = AntennaBearing;      
    }
    seconds++;
    if ( seconds > 10 ) { // One second past
      seconds = 0 ;
      if( InPosition == false ) { // Blink stop led on|off
	digitalWrite(STOPled, digitalRead(STOPled) ^1);
      }
      else digitalWrite(STOPled, 1);
    }
  }
}
/*
 * Start the interrupt service routine
 */
 void Setup_Control_ISR(){
  Timer1.initialize(INTERRUPT_TIMER);      // Set the interrupt rate
  Timer1.attachInterrupt(Service_Control); // and connect driver
}

// CW Rotation
void rotate_right() {
  TurnCW();
  Display_Dir("  CW ");
  Running = true ;
}

// CCW rotation
void rotate_left() {
  TurnCCW();
  Display_Dir(" CCW ");
  Running = true;
}

/* EMA : Exponential Mean Average algorithm
 * Calculate running average of the ADC readouts
 * This is a simple but effective way to suppress noise
 * and jitter on the ADC samples from the potentiometer.
 * Alpha is the filter constant used for averaging
 * Alpha > -> older measurements are less in weight.
 * Formula : S1=Y1 ; St=alpha.Yt + (1-alpha).S(t-1)
 */
int mean_Avg( int sensorValue, int sensorValueTm1){
  float Alpha = 0.3 ; // Increase Alpha into a bigger value if not following the rotator speed
  int ema ;
  ema = float(Alpha*sensorValue) + float((1-Alpha)*sensorValueTm1); 
  return(ema);
}

/*
 * Send the Azimuth AntennaBearing position to usb output
 * Two outputs, the decimal potmeter value for calibration purpose and
 * the bearing angle in the Yeasu GS232 protocol 
 */
void send_az_position() {
  int cur_degrees ;
  cur_degrees = int ((float)( PotmeterPosition - LoPot)/(HiPot - LoPot) * 360);  // calculate angle 0..360
  
  // Output potmeter value ( 10 bits decimal between 0 and 1023)
  Serial.print("Potmeter Calibration value: "); Serial.println(PotmeterPosition);

  // Send position in degrees (GS232 format)
  Serial.print("+0");
  if(AntennaBearing < 100) Serial.print("0");
  if(AntennaBearing <  10) Serial.print("0");

  Serial.println(AntennaBearing);                              
}

/********************* Yaesu GS232 rotator command emulation ****************/
/* 
 * This controller uses the same command language for remote control
 * as the Yaesu GS232 rotator.
 * To set the deviation value an extra  command has been added to the command list:
 * DXXX Where XXX is the new deviation value in degrees.
 * Not all the capabilities of the GS232 language can be used, 
 * ie. variable speed and smooth start-stop control has not been implemented.
 * The current implementation only serves light-weight rotators with
 * on-off L/R switched motor types as most of these light rotators use.
 * When using bigger rotators for heavy antennas which deliver 
 * heavy torque forces on the rotator a proportional control
 * with a slow start/stop and breaking at the endpoint is needed.
 * In this case also other power control with regulated output is needed. 
 * Data is sent and received from the Serial USB data output channel.
 */


/*********************************
 *   GS232 command functions   *
 *********************************/

// Custom functions
void prompt() {
  Serial.print("> ");
}
    
void help() {
  Serial.println("--- GS232 ROTATOR COMMAND LIST ---");
  Serial.println("R  CW Rotation");
  Serial.println("L  CCW Rotation");
  Serial.println("S  Azimuth Rotator Stop");
  Serial.println("C  Current azimuth");
  Serial.println("D  Set new Deviation value in degrees : DXXX");
  Serial.println("M  Move to Antenna pos Deg. MXXX");
}

// Setup GS232 Rotator emuluation
void setup_GS232() {
  inputString.reserve(200);
  Rotator_init(); 
  // prompt();
}

// Read command input line
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    if (isalpha(inChar)) inChar = toupper(inChar); // Make input chars uppercase
    inputString += inChar;                         // add to inputString:
    
    if ((inChar == '\r') || (inChar == '\n')) {    // Return or Newline terminates a command.
    if  (inChar == '\r') inputString += '\n' ;     // Replace return by EOL 
      stringComplete = true;
    } 
  }
}
/*
 * Process GS232 compatible Commandline
 * GS232Control is compatible with the Yaesu GS232 rotator command set.
 *
 */

void GS232Control() {
  int NewPos ;
  int in_speed ;
  int Newdeviation ;

  // Command input completed after a cr or newline:
  if (stringComplete) {
    Serial.println(inputString); 
    Display_Control("Remote");
    char Cmd ;
    Cmd = inputString[0];
    switch (Cmd)  {
      case 'R':
        Stop_Motor();
	AutoRotate = true ;             // rotate CW until Stop
	Display_Control("Auto");
	Dir = CW ;
	Heading=Deviation-3 ;       // Rotate until max Endstop
	//OldHeading = AntennaBearing ;
	NewPosition = true ;   // New heading has been set
	Rotate_To(Heading);
	break;
    case 'L':                           // rotate CCW until Stop
        Stop_Motor();
	AutoRotate = true ;
	Display_Control("Auto");
	Dir = CCW ;
	//OldHeading = AntennaBearing ;
	Heading=Deviation+3 ;       // Rotate CCW until minimum Endstop
	NewPosition = true ;   // New heading has been set
	Rotate_To(Heading);
	break;
    case 'A':           // autospeed not implemented
      Serial.println("*** Autospeed control not implemented in this hardware");
      break;
    case 'S':
      AutoRotate = false ;// Stop Auto Rotate
      Heading = AntennaBearing ;
      Dir = STOP ;
      rotate_stop();
      break;
    case 'C':
      send_az_position();
      break;
    case 'H':
      help();
      break;
    case 'M': // Move to pos (degrees)
      NewPos = str_to_int(inputString.substring(1));
	if (NewPos >= MAXDEGREES) NewPos = 0; //  0..359 degrees
	if (NewPos < 0          ) NewPos = 0;
	if (NewPos != Heading) {         // If changed, start turning
	  Heading = NewPos;
	  //OldHeading = AntennaBearing ;
	  NewPosition = true ;   // New heading has been set
	  Rotate_To(Heading); // Go directly to new position
	}
	break;
    case 'X':
	in_speed = str_to_int(inputString.substring(1));
	// az_set_speed(in_speed); // Not implemented
	Serial.println("*** Rotor speed control not Implemented in this hardware");
	break;
    case 'D':
      Newdeviation = str_to_int(inputString.substring(1));
      if ( (Newdeviation >= 0) && ( Newdeviation < MAXDEGREES )) { //  0..360 degrees
	Serial.print("New deviation value : "); Serial.println(Newdeviation);
	Deviation = Newdeviation ;	    
	SaveEEPROMValue(EEPROM_Deviation, Deviation);               // Store in EEPROM
	sprintf(formattedDataBuffer, FormatData("%3d"), Deviation); // Display new value
	tft.setTextColor(TFT_RED,NAMEBG);
	tft.drawString(formattedDataBuffer, 84,240, 2);
	Display_Control("Stored");
	delay(2000);
      }
      else
	Serial.print("Error in new deviation value set : "); Serial.println(Newdeviation);
      break;
          
    default:
      Serial.print("? Illegal Input >");
      break;
    } // end switch

  Serial.print("\r");
  
  // clear the string:
  inputString = "";
  stringComplete = false;
  Display_Control("Remote");
  }
}

/****************** End GS232 emulation *************/

/******************* Begin Graphics functions *******/
 
/* 
 * Draw an azimuthal map compass rose centered around the QTH locator.
 * Done by reading the color pixel map from program memory storage.
 * Note: Arduino Mega2550 array lengths should be less then 32K 
 */

void Draw_AZMap () {
  int index=0;
  int h = 180,w = 180, row, col; 
  int Xstart = XC - 90  ; int Ystart= YC - 90;
  
  if ( Worldmap) Serial.println("World Map") ;
  else           Serial.println("Local Map") ;

  for (row=0; row < (h-1); row++) { 
    for (col=0; col < w; col++) { 
      tft.drawPixel(col+Xstart, row+Ystart, pgm_read_word_far(Map+index));
      index++;
    } 
  }
  for (int deg = 0 ; deg < 360; deg++) Draw_Heading_Marker (deg, DBCOLOR);  // Scrub marker area
}

 
// Set NESW Symbol directions
void DisplayUserEntry(int x, int y, char *userData)  {
  tft.setTextColor(TFT_RED);
  tft.setFreeFont(FSB12);
  tft.drawString(userData,x,y,2);
}

/*
 * Set dial bearing symbols in text and numbers 
 */

void Set_NESW(){   
  DisplayUserEntry((XC-3),(YC-151),N);
  DisplayUserEntry((XC-3),(YC+143),S);
  DisplayUserEntry((XC+145),(YC-7),E); 
  DisplayUserEntry((XC-152),(YC-7),W);
}

void Set_Bearingnumbers(){
  DisplayUserEntry((XC-3),  (YC-117), "0");
  DisplayUserEntry((XC-10), (YC+95),"180");
  DisplayUserEntry((XC+102),(YC-8),  "90");
  DisplayUserEntry((XC-118),(YC-8), "270");
}

/* 
 * Draw markers on compass for N, E S W and texts plus degree numbers
 */
 
 void  Draw_CompassMarkers() {
  for (float i = 0; i <360; i = i + 22.5 ) { // Draw short compass rose markers
    tft.setTextColor(LTORANGE);
    dxOuter = dm * cos((i-90)*PI/180);
    dyOuter = dm * sin((i-90)*PI/180);
    dxinner = dxOuter * 0.97;
    dyinner = dyOuter * 0.97; 
    tft.drawLine(dxOuter+XC,dyOuter+YC,dxinner+XC,dyinner+YC,TFT_MAGENTA); 
  } 
 for (float i = 0; i <360; i = i + 45 ) { // Draw long Compass rose markers 
    tft.setTextColor(LTORANGE);
    dxOuter = dm * cos((i-90)*PI/180);
    dyOuter = dm * sin((i-90)*PI/180);
    dxinner = dxOuter * 0.92;
    dyinner = dyOuter * 0.92;
    tft.drawLine(dxinner+XC,dyinner+YC,dxOuter+XC,dyOuter+YC,TFT_MAGENTA); 
  }

 Set_NESW();            // Set Dial symbols as text
 Set_Bearingnumbers() ; // And place the degree numbers
}

 
/*
 * Draw screen items, compass rose, messages , etc.
 */

void DrawInitialScreen() {
  tft.drawCircle(XC,YC,dm,TFT_GREEN);
  Draw_AZMap();  
  Draw_CompassMarkers() ; 
}

/*
 * Initialize display
 * Start with the inititial TFT display handling.
 * Do basic setup first, after that fill the frame areas for text and compass
 * Then fill frames with texts and compass with markers and map chart.
 *
 */
   
void InitializeDisplay() {
  tft.init() ;   tft.setRotation(1); // Initialize HX8537 TFT display  
  tft.invertDisplay(1);              // These settings may vary between the differen tft disply types
  tft.fillScreen(TFT_BLACK);         // Erase screen

  // Draw screen frames : Draw frames with rounded corner
  // Background Frame first, other frames are overlay of te backround frame 
  //        X0,Y0, X1, Y1,Edg, Brdr,     Fill 
  DrawFrame( 0, 0,480,320, 20, GREY, SCREENBG);   // Background canvas frame is first, others on top

  DrawFrame(LEFT, 10, 156, 72, 10, RED, BLUE);   // Mycall&QTH locator frame name
  tft.setTextColor(TFT_RED);
  tft.drawString(MYCALL,    LEFT+20, 18, 4);      // PA0HPG 
  tft.drawString(MYLOCATOR, LEFT+30, 44, 4);      // JO33ic 

  DrawFrame(LEFT, 80, 156, 319, 10, GREY, NAMEBG);             // Multiple Data display frame
  DrawFrame(LEFT+4, 86, 150, 118, 5, GREY, YELLOW);            // Position display frame
  DrawFrame(LEFT+10, 127, 133, 191, 10, GREY,DARKBLUE);        // 7 Segment Bearing Angle display frame

  DrawFrame(LEFT+4, 196, 150, 258, 5, GREY,DARKBLUE);          // Depart-Goto-Deviation frame
  DrawFrame(76,202,112,220, 5, GREEN,NAMEBG);                  // GOTO data frame
  DrawFrame(76,222,112,240, 5, GREEN,NAMEBG);                  // Depart data frame 

  DrawFrame(LEFT+4, 260, 150, 292, 5, GREY, YELLOW);           // Heading display frame NESW
  DrawFrame(162, 8, 476, 318, 20, BLUE, NAMEBG);               // Compass frame
  DrawFrame(RIGHT-94, 16,RIGHT-14 , 48, 10, BLUE,DARKBLUE);    // CW/CCW/STOP frame

  DrawFrame(RIGHT-104, 280, RIGHT-10, 315, 10, BLUE, DARKBLUE); // Manual/Remote frame

  // Fill data fields
  tft.setTextColor(TFT_BLUE, TFT_YELLOW);    // Draw Position North | East, etc.
  tft.drawString(POSITION, LEFT+18, 90,4);

  // tft.setFreeFont(GLCD);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(DEPART,    LEFT+10, 204,2);
  tft.drawString(GOTO,      LEFT+10, 223,2);
  tft.drawString(DEVIATION, LEFT+10, 240,2);  // Rotor deviation
  
  tft.setFreeFont(FSB12);                     // 12 pts Serif Bold fonts
  tft.setTextColor(TFT_RED);
  tft.drawString("O", LEFT+114, 126, 2);      // Degree symbol current Beam Pos
  tft.drawString("o", 110, 236, 2);           // Degree symbol Deviation


  tft.setTextColor(TFT_WHITE);
  tft.drawString(MYCALL ,LEFT+170, 290,4);
  
  tft.drawCircle(320, 160, 130, TFT_BLUE);  // Draw Compass circle
  tft.fillCircle(320, 160, 130, DARKBLUE);    
  tft.fillCircle(320, 160, 120, DBCOLOR);   // Fill Compass ring

  fdx = (dm *0.9 * cos(0)*PI/180) + XC;                // Calculate initial former bearing endpoint tip
  fdy = (dm *0.9 * sin(0)*PI/180) + YC;    
  fbposdxbegin = (dm *0.7 * cos(0)*PI/180) + XC; // and end  
  fbposdybegin = (dm *0.7 * sin(0)*PI/180) + YC;    

  fposdx = (dm *0.8 * cos(0)*PI/180) + XC;  // Calculate initial former x endpoint heading endpoint
  fposdy = (dm *0.8 * sin(0)*PI/180) + YC;

  DrawInitialScreen();
}


/*
 * Draw position marker
 * The marker is a triangle which travels around the map indicating the position
 * of the antenna bearing or the new position heading in red.
 * x1,y1 : begin point of the marker 
 * x2,y2 : end point of the marker
 *     h : height 
 *     w : width
 * COLOR : color of the marker
 */ 

void Draw_CompassMarker(int x2, int y2, int x1, int y1, int h, int w, int COLOR) {

#define sharpeness 3  // Sets triangle sharpeness. 3 -> 6 yields a sharper triangle

  int dx, dy, x2a, y2a, x3, y3, x4, y4;

  dx = x1 + (w/sharpeness) * (x2 - x1) / h; 
  dy = y1 + (w/sharpeness) * (y2 - y1) / h;
  x2a = x1 - dx;
  y2a = dy - y1;
  x3 = y2a + dx;
  y3 = x2a + dy;
  x4 = dx - y2a;
  y4 = dy - x2a;
  tft.drawTriangle(x2,y2,x3,y3,x4,y4,COLOR);
  tft.fillTriangle(x2,y2,x3,y3,x4,y4,COLOR); 
} 

/* 
 * Create two-dimensional array tables for Bearing and Heading markers
 * tables contain the begin and endpoint of the markers for each compass angle
 * These tables are used to speed up the marker position calculations
 * by doing the sine and cosine calculations at setup only once.
 */
void Create_Marker_Table(){

   for ( int angle = 0; angle <= 359; angle++ ){
     posdx = (dm *0.9 * cos((angle-90)*PI/180)) + XC;      // Calculate new x end position marker 
     posdy = (dm *0.9 * sin((angle-90)*PI/180)) + YC;      // Calculate new y end position   
     posdxbegin = (dm *0.6 * cos((angle-90)*PI/180)) + XC; // Calculate begin x point
     posdybegin = (dm *0.6 * sin((angle-90)*PI/180)) + YC; // Calculate begin y point
     Compass_marker[angle][0] = posdx ; Compass_marker[angle][1] = posdy ;
     Compass_marker[angle][2] = posdxbegin ; Compass_marker[angle][3] = posdybegin ;
   }
}
 
/*
 * Draw the new antenna Heading input marker set by the buttons CW|CCW or External GS232 input command
 * Position is a red marker bar which travels around the globe. If in position, color becomes yellow.
 * On order avoiding a map rewrite the line between the center of the compass has been omitted.
 * posdx, posdy : former dx,dy position of heading marker
 * posdxbegin, posdybegin : begin of marker 
 */

 void Draw_Heading_Marker (int Heading, int COLOR){  

  Draw_CompassMarker(fposdx,fposdy, fposdxbegin,fposdybegin , 60,70, DBCOLOR); // Erase Previous marker

  posdx = Compass_marker[Heading][0] ; // Set end pos marker
  posdy = Compass_marker[Heading][1] ;

  fposdx = posdx; // Save for later marker erase
  fposdy = posdy;
  
  posdxbegin = Compass_marker[Heading][2]; // Set begin point
  posdybegin = Compass_marker[Heading][3];
  
  Draw_CompassMarker(posdx,posdy, posdxbegin,posdybegin , 60,70, COLOR); // Draw new marker position 
  fposdxbegin = posdxbegin;
  fposdybegin = posdybegin;
  
  Set_Bearingnumbers() ; // Restore possible overwritten dial numbers
}
 
/*
 * Draw the antenna bearing marker
 * fdx,fdy : Former dx,dy position
 *  
 */
 void Draw_Bearing_Position (int AntennaBearing, int COLOR){
  
  Draw_CompassMarker(fdx,fdy, fbposdxbegin, fbposdybegin, 40, 40, DBCOLOR); // Erase Previous bearing

  dx = Compass_marker[AntennaBearing][0]; // Set end point marker
  dy = Compass_marker[AntennaBearing][1];

  fdx = dx;  // Save to erase former pos in update
  fdy = dy;

  bearingposdxbegin = Compass_marker[AntennaBearing][2]; // Set begin point marker
  bearingposdybegin = Compass_marker[AntennaBearing][3];

  if ( abs(AntennaBearing - Heading) <= DEADBAND ) Draw_Heading_Marker(Heading, DBCOLOR); // In position: Heading not displayed
  else Draw_Heading_Marker(Heading, TFT_RED); 
  
  Draw_CompassMarker(dx,dy,bearingposdxbegin, bearingposdybegin, 40,40, COLOR);  // Draw Head in new position 
  fbposdxbegin = bearingposdxbegin; // save to erase after update
  fbposdybegin = bearingposdybegin;  
  
  Set_Bearingnumbers() ;       // Restore possible overwritten dial numbers
}
 
/*
 * Get Antennabearing angle from the potentiometer value.
 * The potentiometer is connected to the gearbox of the rotator.
 * A 70 teeth gear is mounted on the 10 turn potentiometer,
 * enabling a high resolution position input of the Antennabearing. 
 * This is much easier to realize than a 360 degrees potentiometer
 * directly mounted on the rotator axis.
 * The 10 turn potentiometer resistance changes from 0 to 2 KOhm and is fed
 * with +5 V DC enabling a maximum voltage change from 0..5V
 * Due to hardware limitations and not willing to damage the potentiometer by 
 * overturning it the number of turns is reduced to a part of the 10 time travel.
 * Calibration should show how many turns the potmeter can make at a 360 degrees
 * rotation of the antenna rotator hardware. This depends on the gear reduction
 * between the gears of the rotator and the 70 teeth potmeter gear.
 * Suppose only 7.9 turns can be made and the potmeter is installed
 * in such a way that 1 turn of the potmeter is at position 0 degrees. 
 * In this case the voltage swing will be from 0.5 .. 4.45 V
 * The resolution of the ADC is 10 bits, so the max resolution is 1023.
 * Only part of this will be used, giving a maximum Antenna Bearing resolution 
 * which still will be always better than 0.5 degree. 
 * The deviation value is the angle between the true North and the antenna position,
 * this for displaying the real azimuthal position on the screen 
 */

/*
 * Get the position angle from the position angle potentiometer
 * Note : Potmeter position is sampled every 100 ms (10 Hz)
 */

void Getposition() {
  if ( PotmeterPosition == 0 ) {  // 0 V potmeter reading:  probably loose cable
    InPosition = true ;
  }
  else {
    if ( PotmeterPosition < LoPot){  // Exceeded MINIMUM position of potmeter 
      Serial.print("Minimum ENDSTOP REACHED : value "); Serial.println(PotmeterPosition); // TEST
      Back_off(CW);
    }
  
    if ( PotmeterPosition > HiPot) { // Exceeded MAX Position of potmeter 
      Serial.print("Maximum ENDSTOP REACHED : value "); Serial.println(PotmeterPosition); // TEST 
      Back_off(CCW);
    }
  }

  AntennaBearing = int ((float)(PotmeterPosition-LoPot)/(HiPot - LoPot) * 360);  // calculate angle 0..360
  AntennaBearing = AntennaBearing + Deviation;                                   // Correct for deviation
  if      ( AntennaBearing >= 360 ) AntennaBearing = AntennaBearing - 360 ; 
  else if ( AntennaBearing <    0 ) AntennaBearing = AntennaBearing + 360 ;
}

// Set direction as clear text North, NE, E , etc.
void Display_NESW() {
  tft.setFreeFont(FSB12);
  tft.setTextColor(TFT_YELLOW,TFT_YELLOW);
  DrawFrame(LEFT+4, 260, 150, 292, 5, GREY, YELLOW);           // Erase heading display frame NESW
  tft.setTextColor(BLUE1,TFT_YELLOW);
  if((AntennaBearing < 22.5)  || (AntennaBearing > 337.5 ))tft.drawString(NORTH , LEFT+7, 265, 4);
  if((AntennaBearing > 22.5)  && (AntennaBearing < 67.5 )) tft.drawString(NE,     LEFT+7, 265, 4);
  if((AntennaBearing > 67.5)  && (AntennaBearing < 112.5 ))tft.drawString(EAST ,  LEFT+7, 265, 4);
  if((AntennaBearing > 112.5) && (AntennaBearing < 157.5 ))tft.drawString(SE,     LEFT+7, 265, 4);
  if((AntennaBearing > 157.5) && (AntennaBearing < 202.5 ))tft.drawString(SOUTH , LEFT+7, 265, 4);
  if((AntennaBearing > 202.5) && (AntennaBearing < 247.5 ))tft.drawString(SW,     LEFT+7, 265, 4);
  if((AntennaBearing > 247.5) && (AntennaBearing < 292.5 ))tft.drawString(WEST ,  LEFT+7, 265, 4);
  if((AntennaBearing > 292.5) && (AntennaBearing < 337.5 ))tft.drawString(NW,     LEFT+7, 265, 4);
}


// Setup. This function is called at startup once.
void setup(){
  
  Serial.begin(9600);

  delay(400);

  Serial.print("PA0HPG Antenna Rotator Control ");
  Serial.println(VERSION);

  Read_EEPROM () ;  
  
  fposdx       = XC ; fposdy       = YC ; // init former marker positions at center compass
  fposdxbegin  = XC ; fposdybegin  = YC ;
  fdx          = XC ; fdy          = YC ;
  fbposdxbegin = XC ; fbposdybegin = YC ;

  Create_Marker_Table(); // Calculate speedup table for markers
  InitializeDisplay();  
  
  PotmeterPosition_Tm1 = analogRead(PotVal);      // Set T-1 potmeter value
  delay (10);
  
  Setup_Buttons();     // Setup button control
  Setup_Control_ISR(); // Activate interrupt services driver

  delay(1000);         // Wait until analog potmeter position is stable

  Getposition();
  PotmeterHeading  = PotmeterPosition;
  InPosition = true ; StopOnce = true ;
  
  Heading  = AntennaBearing;
  OldHeading  = Heading ;
  Dir = STOP ; OldDir  = Dir ;

  setup_GS232() ;     // Initialize GS232 rotator emulation

  noInterrupts();
  Draw_Bearing_Position(AntennaBearing, TFT_YELLOW);
  Update_Compass(); // Set initial position
  interrupts();
  
  Display_NESW();   // Set direction as clear text North, NE, E , etc.
  Rotator_init();
}

/*
 * Relay-driven motor control.
 * Because the motor will not stop turning immediately after power has been 
 * turned off the Motortravel value should be estimated to stop at the intended position. 
 * This depends on the time the motor has come to a full stop after switching off.
 * If a DC motor is used with proportional speed control a calibration of the PWM values
 * should be done for proper startup and Motortravel.
 *
 * CHECK CODE
 */

void Motor_Control () {
  int potmeterpos ;

  noInterrupts();                // critical section
  potmeterpos = PotmeterPosition;
  interrupts();                  // end section

  if ( AutoRotate ) { // Stop before mechanical endstop
    if ( ( potmeterpos <= LoPot ) || ( potmeterpos >= HiPot)){
	 AutoRotate = false ;
	 InPosition = false ;
	 rotate_stop();
    }
  }
  else {
    if (Running == true) {   // Turn in CW or CCW direction
      if ( ( ( potmeterpos < PotmeterHeading ) &&                         // Are we almost there ?
	     (((PotmeterHeading - potmeterpos ) <= CWMotortravel) ))  ||  // Moving CW
	   (((potmeterpos > PotmeterHeading ) &&                          // Moving CCW
	     (potmeterpos - PotmeterHeading ) <= CCWMotortravel)) ) {   
	InPosition = false ;
	rotate_stop();
      }
    }
    else {
      rotate_stop();      // Stop, Running was false
    }
  }
} /* end motorcontrol */

/* 
 * Signal fault in potentiometer position control cable 
 */

void Cable_Fault(){
  InPosition = false ;
  rotate_stop() ; //  Power off motor
  Display_Control(CABLE);
  Serial.println("Probably loose potentiometer control cable, motor switched off");
}


 
/*****         --- Begin of  Main loop ---          *****/
void loop() {  
  
  GS232Control();      // Check whether remote command given via USB and execute
  
  if (NewPosition) {   // Move to the new antenna direction set
    Rotate_To(Heading);
    NewPosition = false ;
  }
  
  Motor_Control();     // Set Motor direction CW or CCW or Stop it. 

  if ( PotmeterPosition != 0 ) 
    InPosition = ( abs(AntennaBearing - Heading) <= DEADBAND ) ;

  if (InPosition ) {  // Arrived. Halt motor and report.
    if ( StopOnce ) { // Give stop command only one time
	StopOnce = false ;
	Display_Dir("STOP ");  
	Stop_Motor();
	if ( PotmeterPosition == 0 ) { // If 0 then most likely control cable loose.
	  Cable_Fault();
	}
    }
  }
  else Rotate_To(Heading); // Drifted away, start motor again and go to Heading
  
} /*** End of main loop ***/ 

/**************************** End of Program ***********************************/
