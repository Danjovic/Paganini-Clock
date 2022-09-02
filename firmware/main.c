/***
 *    $$$$$$$\                                         $$\           $$\       
 *    $$  __$$\                                        \__|          \__|      
 *    $$ |  $$ |$$$$$$\   $$$$$$\   $$$$$$\  $$$$$$$\  $$\ $$$$$$$\  $$\       
 *    $$$$$$$  |\____$$\ $$  __$$\  \____$$\ $$  __$$\ $$ |$$  __$$\ $$ |      
 *    $$  ____/ $$$$$$$ |$$ /  $$ | $$$$$$$ |$$ |  $$ |$$ |$$ |  $$ |$$ |      
 *    $$ |     $$  __$$ |$$ |  $$ |$$  __$$ |$$ |  $$ |$$ |$$ |  $$ |$$ |      
 *    $$ |     \$$$$$$$ |\$$$$$$$ |\$$$$$$$ |$$ |  $$ |$$ |$$ |  $$ |$$ |      
 *    \__|      \_______| \____$$ | \_______|\__|  \__|\__|\__|  \__|\__|      
 *                       $$\   $$ |                                            
 *                       \$$$$$$  |                                            
 *                        \______/                                             
 *     $$$$$$\  $$\                     $$\                                    
 *    $$  __$$\ $$ |                    $$ |                                   
 *    $$ /  \__|$$ | $$$$$$\   $$$$$$$\ $$ |  $$\                              
 *    $$ |      $$ |$$  __$$\ $$  _____|$$ | $$  |                             
 *    $$ |      $$ |$$ /  $$ |$$ /      $$$$$$  /                              
 *    $$ |  $$\ $$ |$$ |  $$ |$$ |      $$  _$$<                               
 *    \$$$$$$  |$$ |\$$$$$$  |\$$$$$$$\ $$ | \$$\                              
 *     \______/ \__| \______/  \_______|\__|  \__|                             
 *                                                                             
 *   
 *    Danjovic 2022 - Beltane 2022
 *    Released under GPL 3.0      
 */

//
// pinout
//                        ATTiny85
//                       +--+__+--+
//    ADC0/BUTTONS  PB5 [|1      8|] Vcc
//                       |        |
//         SCK/RCK  PB3 [|2      7|] PB2  SCL (I2C)
//                       |        |
//   DOUT LED RING  PB4 [|3      6|] PB1  SHIFT DATA OUT
//                       |        |
//                  GND [|4      5|] PB0  SDA (I2C)
//                       +--------+



//    _ _ _                 _        
//   | (_) |__ _ _ __ _ _ _(_)___ ___
//   | | | '_ \ '_/ _` | '_| / -_|_-<
//   |_|_|_.__/_| \__,_|_| |_\___/__/
//                                   

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>
#include "paganini.h"



//       _      __ _      _ _   _             
//    __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
//   / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
//   \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
//   


// 16 segment display stuff
#define CA  // Common Anode
/* #define CC */ // Common Cathode

// /shift register stuff
#define PORT_SH_REG PORTB 
#define bSCK        3
#define bDOUT       1

// I2C stuff
#define PORT_I2C    PORTB
#define DDR_I2C     DDRB
#define PIN_I2C     PINB
#define bSCL        2
#define bSDA        0
#define sendACK     true
#define sendNACK    false
#define RTC_ADDRESS 0x68

// Neopixel ring stuff
#define PORT_NEOLED PORTB
#define bLED        4
#define NUMLEDS     24

// button stuff           
#define bANALOG          5
#define th_BUTTON_NONE 192  // voltage thresholds
#define th_BUTTON_SET   64  // 8 bits, 0xff = 5V 
//#define th_BUTTON_MODE   5  //


#define th_SHORT_PRESS 5    // time thresholds
#define th_LONG_PRESS 65    // in units of 10 ms



//                             
//    _ __  __ _ __ _ _ ___ ___
//   | '  \/ _` / _| '_/ _ (_-<
//   |_|_|_\__,_\__|_| \___/__/
// 
                            
// I2C related
#define sdaHigh()  do { DDR_I2C &= ~(1<<bSDA); PORT_I2C |=  (1<<bSDA); } while (0)
#define sdaLow()   do { DDR_I2C |=  (1<<bSDA); PORT_I2C &= ~(1<<bSDA); } while (0)
#define sdaGet()  ( ( PIN_I2C &   (1<<bSDA) )!=0)
#define sclHigh()  do { DDR_I2C &= ~(1<<bSCL); PORT_I2C |=  (1<<bSCL); } while (0)
#define sclLow()   do { DDR_I2C |=  (1<<bSCL); PORT_I2C &= ~(1<<bSCL); } while (0)
#define I2Cdelay() _delay_us(5) 

// shift register related
#define SCK_LOW()   PORTB &= ~(1<<bSCK)
#define SCK_HIGH()  PORTB |=  (1<<bSCK)
#define DATA_LOW()  PORTB &= ~(1<<bDOUT)
#define DATA_HIGH() PORTB |=  (1<<bDOUT)

  


//               _               _                     
//    __ _  _ __| |_ ___ _ __   | |_ _  _ _ __  ___ ___
//   / _| || (_-<  _/ _ \ '  \  |  _| || | '_ \/ -_|_-<
//   \__|\_,_/__/\__\___/_|_|_|  \__|\_, | .__/\___/__/
//                                   |__/|_|           

typedef struct {
  unsigned units : 4;
  unsigned tens  : 3;  
  unsigned       : 1;  
} t_seconds;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 3;  
  unsigned       : 1;  
} t_minutes;

typedef struct {
  unsigned units   : 4 ;
  unsigned tens    : 2 ;  
  unsigned op12_24 : 1 ; 
  unsigned         : 1 ;  
} t_hours;

typedef struct {
  unsigned dow     : 3 ;  // day of week
  unsigned         : 5 ;  
} t_wkDays;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 2;  
  unsigned       : 2;  
} t_dates;

typedef struct {
  unsigned units   : 4 ;
  unsigned tens    : 1 ;  
  unsigned         : 2 ;  
  unsigned century : 1 ;  
} t_monthsCty;

typedef struct {
  unsigned units : 4;	
  unsigned tens  : 4;
} t_years;

  
typedef struct {
    t_seconds    seconds ;
    t_seconds    minutes ;
    t_hours        hours ;
    t_wkDays       wkDay ;
    t_dates         date ;
    t_monthsCty monthCty ; 
    t_years         year ;
} t_timeAndDate; 
  
typedef union  {
   uint8_t rawdata[7];
   t_timeAndDate datetime;
} t_ds3231records;

typedef struct {
    uint8_t green ;
    uint8_t red ;
    uint8_t blue ;
} t_grb; 

typedef union  {
   t_grb led[NUMLEDS];
   uint8_t raw[ 3*NUMLEDS ]; 
} t_ledFrameBuffer;


typedef enum   {
  BT_NONE,
  BT_SET, 
  BT_MODE,
  BT_BOTH,
  BT_UNKNOWN
} t_buttonStates;


typedef enum   {
  EV_NOCHANGE,
  EV_MODE_PULSE,
  EV_MODE_LONG,
  EV_SET_PULSE,
  EV_SET_LONG,
  EV_RELEASE
} t_buttonEvents;


typedef enum   {
  ST_SHOW_TIME,
  ST_SET_HOUR,
  ST_SET_MINUTE,
  ST_SET_DAY,
  ST_SET_MONTH,
  ST_SET_YEAR
  // ST_SET_HEMISPHERE
} t_operatingStates;


typedef enum {
//   16seg / LED ring
  MD_SEASON_24HOUR, // default
  MD_SEASON_12HOUR,
  MD_ULTRADIAN_SEASON,
  MD_ULTRADIAN_NO_SEASON
} t_displayMode;



typedef enum {
  _JAN = 1,
  _FEB,
  _MAR,
  _APR,
  _MAY,
  _JUN,
  _JUL,
  _AUG,
  _SEP,
  _OCT,
  _NOV,
  _DEC
} t_months; 


//                 _       _                     
//    _ __ _ _ ___| |_ ___| |_ _  _ _ __  ___ ___
//   | '_ \ '_/ _ \  _/ _ \  _| || | '_ \/ -_|_-<
//   | .__/_| \___/\__\___/\__|\_, | .__/\___/__/
//   |_|                       |__/|_|           

void initHW(void);


void clearLeds   ( t_ledFrameBuffer *fbuffer );
void showLeds    ( t_ledFrameBuffer *fbuffer );


void shiftOut16seg ( uint16_t dbuffer ) ; 

t_buttonStates getButton (void);  
t_buttonEvents buttonEvents (void);


bool  readRtc     ( t_ds3231records *t );
bool writeRtc     ( t_ds3231records *t );
void adjustDOW    ( t_ds3231records *t );
bool rtcDataCheck ( t_ds3231records *t );


void I2Cstart( void );
void I2Cstop( void );
bool I2Cwrite(uint8_t d);
uint8_t I2Cread( bool nack);


bool isLeapYear(uint16_t year);
uint16_t dotyFebruary1( uint8_t month, uint8_t day);
uint8_t wday(uint16_t year, uint8_t month, uint8_t day);

void showTime (void);
void showHour (void );
void showMinute (void );
void showDay (void );
void showMonth (void );
void showYear (void );

void advanceDisplayMode (void);
void advanceHour   (uint8_t delta);
void advanceMinute   (uint8_t delta);
void advanceDay   (uint8_t delta);
void advanceMonth   (uint8_t delta);
void advanceYear   (uint8_t delta);

void showTime24HledRing ( uint8_t hour, uint8_t minute );
void showTime12HledRing ( uint8_t hour, uint8_t minute, uint8_t second );
void showTimeUltradian16seg (uint8_t hour, uint8_t minute );
void showUltradianIntervalLedRing ( uint8_t minute, uint8_t second  );

void showSeason16Seg ( uint16_t dayOfYear ); 
void showSeasonLedRing ( uint16_t dayOfYear  );

void runClockEngine( t_buttonEvents btEvent );
void runDisplayEngine( void );
void runDisplayError( void );



//                   _            _      
//    __ ___ _ _  __| |_ __ _ _ _| |_ ___
//   / _/ _ \ ' \(_-<  _/ _` | ' \  _(_-<
//   \__\___/_||_/__/\__\__,_|_||_\__/__/
//                                       

const uint16_t PROGMEM phase[8]    = { _A2, _B , _C , _D1, _D2, _E , _F , _A1 };
const uint16_t PROGMEM seasons[8]  = { _Yul_s0_, _Imb_s0_, _Ost_s0_, _Bel_s0_, _Lit_s0_, _Lam_s0_, _Mab_s0_, _Sam_s0_ };

const uint16_t PROGMEM tensBCD[10]  = { _100_ ,  _10_,  _20_,  _30_,  _40_,  _50_,  _60_,  _70_,  _80_,  _90_ };
const uint16_t PROGMEM unitsBCD[10] = {  _00_ ,  _01_,  _02_,  _03_,  _04_,  _05_,  _06_,  _07_,  _08_,  _09_ };


//const PROGMEM uint16_t minute8[8]   = { _A2, _B,  _C, _D1, _D2, _E,  _F, _A1 }; same as phase[]
const uint16_t PROGMEM hour8[8]     = {  _I, _J, _G2,  _M,  _L, _K, _G1,  _H };


const PROGMEM uint16_t dayThreshold24intervals [24] = {
	 12,  30,  48, 58 ,  73,  89, 101, 120, 140, 150, 165, 181,
	193, 212, 232, 242, 257, 273, 285, 304, 323, 333, 348, 364
};
//
	

const uint8_t PROGMEM seasonColors[24][3] = {  
	{ 0xFF, 0x00, 0x00 } , // RED 
	{ 0xE1, 0x00, 0x1E } , //	
	{ 0xC0, 0x00, 0x3F } , // 	
	{ 0xA2, 0x00, 0x5D } , // 	
	{ 0x81, 0x00, 0x7E } , // 
	{ 0x60, 0x00, 0x9F } , // 
	{ 0x42, 0x00, 0xBD } , // 
	{ 0x21, 0x00, 0xDE } , //  
	{ 0x00, 0x00, 0xFF } , // BLUE 
	{ 0x00, 0x1E, 0xE1 } , //  
	{ 0x00, 0x3F, 0xC0 } , // 
	{ 0x00, 0x60, 0x9F } , // 
	{ 0x00, 0x7E, 0x81 } , // 
	{ 0x00, 0x9F, 0x60 } , // 
	{ 0x00, 0xC0, 0x3F } , // 
	{ 0x00, 0xDE, 0x21 } , // 
	{ 0x00, 0xFF, 0x00 } , // GREEN
	{ 0x21, 0xDE, 0x00 } , // 
	{ 0x3F, 0xC0, 0x00 } , //
	{ 0x60, 0x9F, 0x00 } , //
	{ 0x81, 0x7E, 0x00 } , //
	{ 0x9F, 0x60, 0x00 } , // 
	{ 0xC0, 0x3F, 0x00 } , // 
	{ 0xE1, 0x1E, 0x00 } , // 
};


const PROGMEM uint16_t dayThreshold[72] = {
 //Segment:      all   none  1seg  2seg  3seg  4seg  5seg  6seg  7seg      (next) 
 /* Imbolc  */      0,    1,    6,   12,   18,   24,   30,   36,   42,  /*    48  */
 /* Ostara  */     48,   49,   53,   58,   63,   68,   73,   78,   83,  /*    89  */
 /* Beltane */     89,   90,   95,  101,  108,  114,  120,  127,  133,  /*   140  */
 /* Litha   */    140,  141,  145,  150,  155,  160,  165,  170,  175,  /*   181  */
 /* Lammas  */    181,  182,  187,  193,  200,  206,  212,  219,  225,  /*   232  */
 /* Mabon   */    232,  233,  237,  242,  247,  252,  257,  262,  267,  /*   273  */
 /* Samhain */    273,  274,  279,  285,  291,  298,  304,  310,  316,  /*   323  */
 /* Yule    */    323,  324,  328,  333,  338,  343,  348,  353,  358   /*   364  */
};
//

const PROGMEM uint16_t seasonPattern16segment[72] = {
	_Imbolc_, _Imb_s0_, _Imb_s1_, _Imb_s2_, _Imb_s3_, _Imb_s4_, _Imb_s5_, _Imb_s6_, _Imb_s7_,  
	_Ostara_, _Ost_s0_, _Ost_s1_, _Ost_s2_, _Ost_s3_, _Ost_s4_, _Ost_s5_, _Ost_s6_, _Ost_s7_,  
	_Beltane, _Bel_s0_, _Bel_s1_, _Bel_s2_, _Bel_s3_, _Bel_s4_, _Bel_s5_, _Bel_s6_, _Bel_s7_,  
	_Litha__, _Lit_s0_, _Lit_s1_, _Lit_s2_, _Lit_s3_, _Lit_s4_, _Lit_s5_, _Lit_s6_, _Lit_s7_,  
	_Lammas_, _Lam_s0_, _Lam_s1_, _Lam_s2_, _Lam_s3_, _Lam_s4_, _Lam_s5_, _Lam_s6_, _Lam_s7_,  
	_Mabon__, _Mab_s0_, _Mab_s1_, _Mab_s2_, _Mab_s3_, _Mab_s4_, _Mab_s5_, _Mab_s6_, _Mab_s7_,  
	_Samhain, _Sam_s0_, _Sam_s1_, _Sam_s2_, _Sam_s3_, _Sam_s4_, _Sam_s5_, _Sam_s6_, _Sam_s7_, 
	_Yule___, _Yul_s0_, _Yul_s1_, _Yul_s2_, _Yul_s3_, _Yul_s4_, _Yul_s5_, _Yul_s6_, _Yul_s7_ 
};
//


//                 _      _    _        
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//                                      
t_ds3231records rtc;
t_ledFrameBuffer ledRing;

volatile uint16_t displ16SegBuffer;
volatile uint16_t sequentialBuffer[3];

volatile bool readyToGo = false;

volatile t_displayMode displayMode        = MD_SEASON_24HOUR;  // global
volatile t_operatingStates operatingState = ST_SHOW_TIME; 

uint8_t ledRingBrightness = 32;

bool timeChanged = false;


//               _      
//    _ __  __ _(_)_ _  
//   | '  \/ _` | | ' \'  
//   |_|_|_\__,_|_|_||_|
//                      
int main(void) {
  // initialize hardware
  initHW();
  
  // cleanup LED ring 
  clearLeds ( &ledRing );
  showLeds  ( &ledRing ); // sei() implicit in showLeds()
  
   operatingState = ST_SHOW_TIME;
   timeChanged = false;


  // main loop
    for (;;) {
        // wait for tick  
        while (!readyToGo) {};  // wait
        readyToGo = false;

        if ( readRtc ( &rtc)  ) {
            runClockEngine( buttonEvents () );
            runDisplayEngine();
            
            if (timeChanged) {    // TODO check for error during write to RTC
				writeRtc ( &rtc ); 
                timeChanged = false;
			}           

		} else runDisplayError();

    } // for (;;)    

} // main() 
//


//    _     _                         _      
//   (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
//   | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
//   |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
//                              |_|          

ISR (TIM1_COMPA_vect ) { // occurs every 10ms
    readyToGo = true;
}
//



//     __              _   _             
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//

// ******************************************************************************************************************                                   
//
// Hardware hadling
//

// Initialize I/O and peripherals
void initHW( void ) {

  // Initialize I/O ports
  DDRB = ( 
    (0 <<  bSCL    ) |  
    (0 <<  bSDA    ) |
    (1 <<  bSCK    ) |
    (1 <<  bDOUT   ) |
    (1 <<  bLED    ) |
    (0 <<  bANALOG ) 
  );

 PORTB = ( 
    (1 <<  bSCL    ) |  
    (1 <<  bSDA    ) |
    (1 <<  bSCK    ) |
    (1 <<  bDOUT   ) |
    (0 <<  bLED    ) |
    (0 <<  bANALOG ) 
  );
 

  // Initialize Timer1, Time = 1,0007 ms for 16.5MHz
  TCCR1 = 
  ( 
    (1 << CTC1   ) |  //  CTC mode
    (0 << PWM1A  ) |
    (0 << COM1A1 ) |
    (0 << COM1A0 ) |
    (1 << CS13   ) |  // Prescaler 1024
    (0 << CS12   ) |
    (1 << CS11   ) |
    (1 << CS10   )
  );
 
  OCR1C = 162; // TOP (reload) value 

  TIMSK = 
  ( 
    (1 << OCIE1A ) |  // interrupt on compare match
    (0 << OCIE1B ) |
    (0 << OCIE0A ) |
    (0 << OCIE0B ) |
    (0 << TOIE1  ) |
    (0 << TOIE0  )
  );
  
/*  
  PLLCSR = 
  ( 
    (0 << LSM   ) |  
    (0 << PCKE  ) |
    (1 << PLLE  ) |
    (0 << PLOCK ) 
  );
*/    

  // Ininialize ADC
  ADMUX = 
  ( 
    (0 << REFS1 ) |   // Vcc = Internal voltage reference
    (0 << REFS0 ) |
    (1 << ADLAR ) |   // Left Adjusted, 8 bit value on ADCH
    (0 << REFS2 ) |
    (0 << MUX3  ) |   // Channel = 0 (PB5)
    (0 << MUX2  ) |
    (0 << MUX1  ) |
    (0 << MUX0  )
  );

  ADCSRA = 
  ( 
    (1 << ADEN  ) |   // Enable ADC
    (1 << ADSC  ) |      
    (1 << ADATE ) |   // Auto trigger enable
    (0 << ADIF  ) |
    (0 << ADIE  ) |   
    (1 << ADPS2 ) |   // Prescaler = 128
    (1 << ADPS1 ) |   
    (1 << ADPS0 )
  );    

  ADCSRB = 
  ( 
    (0 << BIN   ) |    
    (0 << ACME  ) |    
    (0 << IPR   ) |     
    (0 << ADTS2 ) |   // Free Running mode
    (0 << ADTS1 ) |   
    (0 << ADTS0 )
  );

}
//


// ******************************************************************************************************************
//
// Soft I2C
//
void I2Cstart() {
  sdaHigh(); I2Cdelay();
  sclHigh(); I2Cdelay(); // sda = 1;  scl = 1;
  sdaLow();  I2Cdelay(); // sda = 0;
  sclLow();
}
// 

void I2Cstop() {
  sdaLow();  I2Cdelay(); // sda = 0;  sda = 0;
  sclHigh(); I2Cdelay(); // scl = 1;  scl = 1;
  sdaHigh();             // sda = 1;
}
//

bool I2Cwrite(uint8_t d) {
  uint8_t i;
  bool nack;
  for (i = 0; i < 8; i++) {
    if (d & 0x80)   // write data bit, msb first
      sdaHigh();
    else sdaLow();
    I2Cdelay(); // give time to settle data
    sclHigh(); I2Cdelay();  sclLow(); // pulse clock
    d = d << 1; // next bit
  }
  // now get the ack
  sdaHigh(); I2Cdelay();  // release data line
  sclHigh(); I2Cdelay();  // release clock line
  nack = sdaGet();  // get nack bit
  sclLow();// clock low
  return nack;
}
//

uint8_t I2Cread(bool nack) {
  uint8_t i, d;

  d = 0;
  sdaHigh();             // release data line and
  sclLow(); I2Cdelay();  // pull down clock line and wait to write a bit 
  for (i = 0; i < 8; i++)  {  
    sclHigh(); I2Cdelay(); // release clock line to read the data
    d = d << 1;
    if (sdaGet() ) d |= 1; // read data bit, msb first
    sclLow(); I2Cdelay();  // pull clock down to allow next bit
  }
  // give ACK / NACK
  if ( nack ) sdaLow(); else sdaHigh();
  
  sclHigh(); I2Cdelay(); // Pulse clock
  sclLow(); I2Cdelay();  // 

  sdaHigh(); // release the data line 
  return d;
}
//


// ******************************************************************************************************************
//
// 16 segment
//
// Shift bits out to 16 segment display
void shiftOut16seg ( uint16_t dbuffer ) {
   uint8_t i;
   uint16_t mask = 1;
   
   //disablePWM(); // disable PWM output
   
   DATA_LOW();  // disable display    
 
   
   // shift in 16 data bits   
   for (i = 0; i < 16 ; i++ ) {   
      SCK_LOW();      
      // write one data bit 
#if defined CA   
      if ( dbuffer & mask ) DATA_LOW(); else  DATA_HIGH();  
#elif defined CC
      if ( dbuffer & mask ) DATA_HIGH(); else  DATA_LOW(); 
#else
#error Must define Common Anode (CA) or Common Cathode (CC)       
#endif           
      mask <<= 1 ; 	//next bit  
      SCK_HIGH();  // shift DP in 
//       _delay_us(1);  // wait for data to settle 	  
      DATA_LOW();  // return data state to its idle	  
 //     SCK_LOW();      // drop clock 
   } 

  // enablePWM(); // return PWM state
}
//


// ******************************************************************************************************************
//
// Neopixel
//
// Send neoled pixel information. borrowed from Adafruit_NeoPixel.cpp
void showLeds  ( t_ledFrameBuffer *fbuffer ) { 
 
  #define pinMask (1<<bLED)
  #define WS2812     
  //#define WS2811   

  volatile uint8_t *port = &PORT_NEOLED;
  volatile uint16_t i    = sizeof(t_ledFrameBuffer); // Loop counter
  volatile uint8_t *ptr  = (void*)fbuffer,           // Pointer to next byte
           b = *ptr++,                               // Current byte value
           hi,                                       // PORT w/output bit set high
           lo;                                       // PORT w/output bit set low
 
  volatile uint8_t next, bit;

  hi = *port | pinMask;
  lo = *port & ~pinMask;
  next = lo;
  bit = 8;
  
  cli();  // disable interrupts to not mess with timing
  
 #if defined ( WS2812 ) // 800KHz

  // WS2811 and WS2812 have different hi/lo duty cycles; this is
  // similar but NOT an exact copy of the prior 400-on-8 code. 
  
  // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
  // ST instructions:         ^   ^        ^       (T=0,5,13) 
 
  asm volatile("head20:"                    "\n\t" // Clk  Pseudocode    (T =  0)
               "st   %a[port],  %[hi]"      "\n\t" // 2    PORT = hi     (T =  2)
               "sbrc %[byte],  7"           "\n\t" // 1-2  if(b & 128)
               "mov  %[next], %[hi]"        "\n\t" // 0-1   next = hi    (T =  4)
               "dec  %[bit]"                "\n\t" // 1    bit--         (T =  5)
               "st   %a[port],  %[next]"    "\n\t" // 2    PORT = next   (T =  7)
               "mov  %[next] ,  %[lo]"      "\n\t" // 1    next = lo     (T =  8)
               "breq nextbyte20"            "\n\t" // 1-2  if(bit == 0) (from dec above)
               "rol  %[byte]"               "\n\t" // 1    b <<= 1       (T = 10)
               "rjmp .+0"                   "\n\t" // 2    nop nop       (T = 12)
               "nop"                        "\n\t" // 1    nop           (T = 13)
               "st   %a[port],  %[lo]"      "\n\t" // 2    PORT = lo     (T = 15)
               "nop"                        "\n\t" // 1    nop           (T = 16)
               "rjmp .+0"                   "\n\t" // 2    nop nop       (T = 18)
               "rjmp head20"                "\n\t" // 2    -> head20 (next bit out)
               "nextbyte20:"                "\n\t" //                    (T = 10)
               "ldi  %[bit]  ,  8"          "\n\t" // 1    bit = 8       (T = 11)
               "ld   %[byte] ,  %a[ptr]+"   "\n\t" // 2    b = *ptr++    (T = 13)
               "st   %a[port], %[lo]"       "\n\t" // 2    PORT = lo     (T = 15)
               "nop"                        "\n\t" // 1    nop           (T = 16)
               "sbiw %[count], 1"           "\n\t" // 2    i--           (T = 18)
               "brne head20"                "\n" // 2    if(i != 0) -> (next byte)
               : [port] "+e"(port), [byte] "+r"(b), [bit] "+r"(bit),
                 [next] "+r"(next), [count] "+w"(i)
               : [ptr] "e"(ptr), [hi] "r"(hi), [lo] "r"(lo));
 
 

#elif defined ( WS2811 ) // 400KHz

  // 40 inst. clocks per bit: HHHHHHHHxxxxxxxxxxxxLLLLLLLLLLLLLLLLLLLL
  // ST instructions:         ^       ^           ^         (T=0,8,20)

  asm volatile("head40:"                         "\n\t" // Clk  Pseudocode    (T =  0)
               "st   %a[port], %[hi]"            "\n\t" // 2    PORT = hi     (T =  2)
               "sbrc %[byte] , 7"                "\n\t" // 1-2  if(b & 128)
               "mov  %[next] , %[hi]"            "\n\t" // 0-1   next = hi    (T =  4)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T =  6)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T =  8)
               "st   %a[port], %[next]"          "\n\t" // 2    PORT = next   (T = 10)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 12)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 14)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 16)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 18)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 20)
               "st   %a[port], %[lo]"            "\n\t" // 2    PORT = lo     (T = 22)
               "nop"                             "\n\t" // 1    nop           (T = 23)
               "mov  %[next] , %[lo]"            "\n\t" // 1    next = lo     (T = 24)
               "dec  %[bit]"                     "\n\t" // 1    bit--         (T = 25)
               "breq nextbyte40"                 "\n\t" // 1-2  if(bit == 0)
               "rol  %[byte]"                    "\n\t" // 1    b <<= 1       (T = 27)
               "nop"                             "\n\t" // 1    nop           (T = 28)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 30)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 32)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 34)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 36)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 38)
               "rjmp head40"                     "\n\t" // 2    -> head40 (next bit out)
               "nextbyte40:"                     "\n\t" //                    (T = 27)
               "ldi  %[bit]  , 8"                "\n\t" // 1    bit = 8       (T = 28)
               "ld   %[byte] , %a[ptr]+"         "\n\t" // 2    b = *ptr++    (T = 30)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 32)
               "st   %a[port], %[lo]"            "\n\t" // 2    PORT = lo     (T = 34)
               "rjmp .+0"                        "\n\t" // 2    nop nop       (T = 36)
               "sbiw %[count], 1"                "\n\t" // 2    i--           (T = 38)
               "brne head40"                    "\n" // 1-2  if(i != 0) -> (next byte)
               : [port] "+e"(port), [byte] "+r"(b), [bit] "+r"(bit),
                 [next] "+r"(next), [count] "+w"(i)
               : [ptr] "e"(ptr), [hi] "r"(hi), [lo] "r"(lo));

#else
#error Unsupported NEOLED model
#endif
  // 
  sei();
} 
//

// Clear LED frame buffer
void clearLeds ( t_ledFrameBuffer *fbuffer ) {
  uint8_t i; 
  
  for (i = 0 ; i < sizeof(t_ledFrameBuffer) ; i++) {
    fbuffer->raw[i] = 0;
  }
}
//


// ******************************************************************************************************************
//
// Button handling
//

// Read Button press
t_buttonStates getButton (void) {
   uint8_t sample;
   
   while ( ! (ADCSRA & (1<<ADIF)) );  // wait for conversion ready
   ADCSRA |= (1<<ADIF);
   
   sample = ADCH;  

   if (sample > th_BUTTON_NONE ) return BT_NONE;
     else if (sample > th_BUTTON_SET) return BT_SET;
        else return BT_MODE;
 
}
//

// Get button events - pulse, long press
t_buttonEvents buttonEvents (void) {
  static uint8_t btModeCount = 0, btSetCount = 0;
  uint8_t btEvent = EV_NOCHANGE;

  uint8_t buttonNow = getButton();

  switch (buttonNow ) {

    case BT_SET:
      if ( btSetCount < th_LONG_PRESS ) {
        btSetCount++;                         // will stop count at threshold
        if ( btSetCount >= th_LONG_PRESS ) { // and generate an unique long press event
          btEvent  = EV_SET_LONG;
        }
      }
      break;

    case BT_MODE:
      if ( btModeCount < th_LONG_PRESS ) {
        btModeCount++ ;                       //  will stop count at threshold
        if ( btModeCount >= th_LONG_PRESS ) {  // and generate an unique long press event
          btEvent  = EV_MODE_LONG;
        }
      }
      break;

    case BT_NONE:
      if ( btSetCount >= th_SHORT_PRESS && btSetCount < th_LONG_PRESS ) {
        btEvent = EV_SET_PULSE;
      }

      if ( btModeCount >= th_SHORT_PRESS && btModeCount < th_LONG_PRESS ) {
        btEvent = EV_MODE_PULSE;
      }

      btSetCount = 0;
      btModeCount = 0;

      break;

    case BT_BOTH:
    case BT_UNKNOWN:
    default:
      btEvent = EV_NOCHANGE;
      break;
  }
  return btEvent;
}
//


// ******************************************************************************************************************
//
// RTC chip handling
//


// write time data on clock Chip using I2C
bool writeRtc ( t_ds3231records *t) {
  uint8_t i;
  I2Cstart();
  if (!I2Cwrite((uint8_t)(RTC_ADDRESS << 1)) ) {  
    I2Cwrite( 0x00 ); // register address, 1st clock register
    for ( i = 0 ; i < 7 ; i++)
      I2Cwrite(t->rawdata[i]);
    I2Cstop();
    return true;
  } else {
    I2Cstop(); I2Cstop();
  } return false;
}
//

// read time data from clock Chip using I2C
bool readRtc ( t_ds3231records *t) {
  uint8_t i;
  I2Cstart();
  if ( !I2Cwrite((uint8_t)(RTC_ADDRESS << 1)) ) {
    I2Cwrite( 0x00); // register address, 1st clock register
    I2Cstart();  // repeated start
    I2Cwrite((uint8_t)(RTC_ADDRESS << 1)  | 1);
    for ( i = 0 ; i < 6 ; i++) {
       t->rawdata[i] = I2Cread ( sendACK );
    } 
    t->rawdata[i] = I2Cread ( sendNACK );       // NACK on last bit
    I2Cstop();
    return true;
  } else {
    I2Cstop(); I2Cstop();
  } return false;
}
//

/* // clear alarm flags 
        // start();
        // i2cwrite((uint8_t)(RTC_ADDRESS<<1))); 
        // i2cwrite(0x0f);
        // i2cwrite(0);            
        // stop();

*/
//


// ******************************************************************************************************************
//
// Date and Time calculation and handling stuff
//

// calculate leap year
bool isLeapYear(uint16_t year) {
    return (  (!(year % 4) && (year % 100)) || !(year % 400)  ) ? true : false;
}
//

// calculate day of the year starting on february 1
uint16_t dotyFebruary1( uint8_t month, uint8_t day) {
    uint16_t doty = 0;
	   
    // offset month from 1..12 (january..december)
    // to 0..11 being (february =0, march = 1, ...december=10, january = 11)
    month = ( month < 2 ) ? month + 11 : month - 1;
   
    doty += day;

    switch (month) {
      case 12: // january -> 31 days
        doty += 31; // add 31 days from december 
      case 11: // december -> 31 days 
        doty += 30; // add 30 days from november          
      case 10: // november -> 30 days  
        doty += 31; // add 31 days from october         
      case 9: // october -> 31 days 
        doty += 30; // add 30 days from september          
      case 8: // september -> 30 days 
        doty += 31; // add 31 days from august      
      case 7: // august -> 31 days 
        doty += 31;// add 31 days from july       
      case 6: // july -> 31 days 
        doty += 30; // add 30 days from june       
      case 5: // june -> 30 days  
        doty += 31; // add 31 days from may       
      case 4: // may -> 31 days 
        doty += 30; // add 30 days from april       
      case 3: // april -> 30 days
         doty += 31; // add 31 days from march
      case 2: // march -> 31 days
         doty += 28; // add 28 days from february	
         // doty += (is_leap_year(year)) ? 1 : 0; 
         // leap year compensation was removed to make all sabbats
         // fall in the same day even in leap year      
    }  
    return  doty-1; // 0..364 days starting at february 1
}
//

// find day of week for a given date
uint8_t wday(uint16_t year, uint8_t month, uint8_t day) {
  uint8_t adjustment, mm, yy;

  adjustment = (14 - month) / 12;
  mm = month + 12 * adjustment - 2;
  yy = year - adjustment;
  return (day + (13 * mm - 1) / 5 +
          yy + yy / 4 - yy / 100 + yy / 400) % 7;
}
//

// advance current hour
void advanceHour   (uint8_t delta) {
  uint8_t d = ((delta
                 + 10 * (uint8_t)rtc.datetime.hours.tens
			     + (uint8_t)rtc.datetime.hours.units )   ) % 24;	
					
	rtc.datetime.hours.tens = ( (uint8_t) d/10 )  & 0b00000011;
	rtc.datetime.hours.units = ( (uint8_t) d%10 ) & 0b00001111;
	
	timeChanged = true; // update rtc data
}
//

// advance current minute
void advanceMinute   (uint8_t delta) {
  uint8_t d = (delta
               + 10 * (uint8_t)rtc.datetime.minutes.tens
			   + (uint8_t)rtc.datetime.minutes.units ) % 60;	
					 
	rtc.datetime.minutes.tens = ( (uint8_t) d/10 )  & 0b00000111;
	rtc.datetime.minutes.units = ( (uint8_t) d%10 ) & 0b00001111;
	
	timeChanged = true; // update rtc data
}
//

// advance current day
void advanceDay   (uint8_t delta) {
  uint8_t f;  // mod factor
  uint8_t mo = 10 * (uint8_t)rtc.datetime.monthCty.tens +
                    (uint8_t)rtc.datetime.monthCty.units;
  uint16_t y; 

  switch (mo) { // calculate mod factor
     case _FEB:
        y = 2000 
		    + 10 * (uint8_t)rtc.datetime.year.tens 
		    + (uint8_t)rtc.datetime.year.units ; 	 
	    f = (isLeapYear(y)) ? 29 : 28 ; 
	    break;		  
  
	 case _JAN:
     case _MAR:
     case _MAY:
     case _JUL:
     case _AUG:
     case _OCT:
     case _DEC:  
	    f = 31; 
	    break;
		
     case _APR:
     case _JUN:
     case _SEP:
     case _NOV:
	 default:
	    f = 30; 
	    break;	
  }					
	
   uint8_t d = ( ( delta -1 // // subtract 1 so day 1 is day 0
                + 10 * (uint8_t)rtc.datetime.date.tens
		        + (uint8_t)rtc.datetime.date.units )  % f )
                +1 ; // // add 1 at the first day of the month is 1 again
					 
	rtc.datetime.date.tens =  ((uint8_t) d/10 )  & 0b00000011;
	rtc.datetime.date.units = ( (uint8_t) d%10 ) & 0b00001111; 
	
	timeChanged = true; // update rtc data
}
//

// advance current month
void advanceMonth   (uint8_t delta) {
	
  uint8_t d = ( (delta  
                 - 1    // subtract 1 so january is 0
                 + 10 * (uint8_t)rtc.datetime.monthCty.tens
			     + (uint8_t)rtc.datetime.monthCty.units )  % 12 ) 
				 + 1;	// add 1 at the end so janyary is 1 again
				 
	rtc.datetime.monthCty.tens = ( (uint8_t) d/10 )  & 0b00000001;
	rtc.datetime.monthCty.units = ( (uint8_t) d%10 ) & 0b00001111;
	
	timeChanged = true; // update rtc data
}
//

// advance current year 
void advanceYear   (uint8_t delta) {
  uint8_t d = (delta
               + 10 * (uint8_t)rtc.datetime.year.tens
			   + (uint8_t)rtc.datetime.year.units ) % 100;	
					
	rtc.datetime.year.tens = ( (uint8_t) d/10 )  & 0b00001111;
	rtc.datetime.year.units = ( (uint8_t) d%10 ) & 0b00001111;
	
	timeChanged = true; // update rtc data
}
//


// Set day of week 
// called whenever time changed, before update rtc
void adjustDOW ( t_ds3231records *t ) {

  uint8_t da = t->datetime.date.tens * 10 +
               t->datetime.date.units;

  uint8_t mo = t->datetime.monthCty.tens * 10 +
               t->datetime.monthCty.units;


  uint16_t yy  = 2000 + t->datetime.year.tens * 10 +
                 t->datetime.year.units;
  
  t->datetime.wkDay.dow = wday( yy , mo, da );
}
//

// check consistency of data read from RTC. 
// protect agains corrupted data
bool rtcDataCheck (  t_ds3231records *t ) {
  uint16_t tmp; //temp
  bool changed = false;

  // check seconds
  tmp = t->datetime.seconds.tens * 10 +
        t->datetime.seconds.units;

  if (tmp >= 60)  {
    changed = true;
    t->datetime.seconds.tens = 0;
    t->datetime.seconds.units = 0;
  }

  // check minutes
  tmp = t->datetime.minutes.tens * 10 +
        t->datetime.minutes.units;

  if (tmp >= 60)  {
    changed = true;
    t->datetime.minutes.tens = 0;
    t->datetime.minutes.units = 0;
  }

  // check hours
  if ( t->datetime.hours.op12_24 ) {
    changed = true;
    t->datetime.hours.op12_24 = 0;

    tmp = ( t->datetime.hours.tens & (1 << 0) ) * 10 +
          t->datetime.hours.units;

    if ( t->datetime.hours.tens & (1 << 1) ) {
      tmp = tmp + 12;
    }

	t->datetime.hours.tens = ( (uint8_t) tmp/10 )  & 0b00000011;
	t->datetime.hours.units = ( (uint8_t) tmp%10 ) & 0b00001111;


  } else {
    tmp = t->datetime.hours.tens * 10 +
          t->datetime.hours.units;
  }

  if (tmp >= 24)  {
    changed = true;
    t->datetime.hours.tens = 0;
    t->datetime.hours.units = 0;
  }


  // check month
  tmp = t->datetime.monthCty.tens * 10 +
        t->datetime.monthCty.units;

  if ( (tmp == 0) || (tmp > 12)) {
    changed = true;
    t->datetime.monthCty.tens = 0;
    t->datetime.monthCty.units = 1;
  }


  // chech day of month
  tmp = t->datetime.date.tens * 10 +        
        t->datetime.date.units;

  if ( (tmp == 0) || (tmp > 31)) {  // TODO make according with month instead of fixed at 31
    changed = true;
    t->datetime.date.tens = 0;
    t->datetime.date.units = 1;
  }
  
  // check year
  tmp = t->datetime.year.tens * 10 +
        t->datetime.year.units;

  if ( tmp > 99 )  {
    changed = true;
    t->datetime.year.tens = 2;
    t->datetime.year.units = 2;
  }

  return changed;
}
//


// ******************************************************************************************************************
//
// Display contents handling
//

// set 3 digits according to the curent hour
void showHour (void ) {
  sequentialBuffer[0] = _HOUR_;
  sequentialBuffer[1] = pgm_read_word (  tensBCD + (uint8_t) rtc.datetime.hours.tens);
  sequentialBuffer[2] = pgm_read_word ( unitsBCD + (uint8_t) rtc.datetime.hours.units); 
}
//

// set 3 digits according to the curent minute
void showMinute (void ) {
  sequentialBuffer[0] = _MINUTE_;
  sequentialBuffer[1] = pgm_read_word (  tensBCD + (uint8_t) rtc.datetime.minutes.tens);
  sequentialBuffer[2] = pgm_read_word ( unitsBCD + (uint8_t) rtc.datetime.minutes.units);
}
//

// set 3 digits according to the curent Day
void showDay (void ) {
  sequentialBuffer[0] = _DAY_;
  sequentialBuffer[1] = pgm_read_word (  tensBCD + (uint8_t) rtc.datetime.date.tens);
  sequentialBuffer[2] = pgm_read_word ( unitsBCD + (uint8_t) rtc.datetime.date.units); 
}
//

// set 3 digits according to the curent Month
void showMonth (void ) {
  uint8_t mo = 10 * (uint8_t)rtc.datetime.monthCty.tens +
                    (uint8_t)rtc.datetime.monthCty.units;
					
  switch (mo) {
	  case _JAN:
	    sequentialBuffer[0] = _J_;
	    sequentialBuffer[1] = _A_;
	    sequentialBuffer[2] = _N_;
	  break;
	  
	  case _FEB:
	    sequentialBuffer[0] = _F_;
	    sequentialBuffer[1] = _E_;
	    sequentialBuffer[2] = _B_;
	  break;
	  
	  case _MAR:
	    sequentialBuffer[0] = _M_;
	    sequentialBuffer[1] = _A_;
	    sequentialBuffer[2] = _R_;
	  break;
	  
	  case _APR:
	    sequentialBuffer[0] = _A_;
	    sequentialBuffer[1] = _P_;
	    sequentialBuffer[2] = _R_;
	  break;
	  
	  case _MAY:
	    sequentialBuffer[0] = _M_;
	    sequentialBuffer[1] = _A_;
	    sequentialBuffer[2] = _Y_;
	  break;
	  
	  case _JUN:
	    sequentialBuffer[0] = _J_;
	    sequentialBuffer[1] = _U_;
	    sequentialBuffer[2] = _N_;
	  break;
	  
	  case _JUL:
	    sequentialBuffer[0] = _J_;
	    sequentialBuffer[1] = _U_;
	    sequentialBuffer[2] = _L_;
	  break;
	  
	  case _AUG:
	    sequentialBuffer[0] = _A_;
	    sequentialBuffer[1] = _U_;
	    sequentialBuffer[2] = _G_;
	  break;
	  
	  case _SEP:
	    sequentialBuffer[0] = _S_;
	    sequentialBuffer[1] = _E_;
	    sequentialBuffer[2] = _P_;
	  break;
	  
	  case _OCT:
	    sequentialBuffer[0] = _O_;
	    sequentialBuffer[1] = _C_;
	    sequentialBuffer[2] = _T_;
	  break;
	  
	  case _NOV:
	    sequentialBuffer[0] = _N_;
	    sequentialBuffer[1] = _O_;
	    sequentialBuffer[2] = _V_;
	  break;
	  
	  case _DEC:
	    sequentialBuffer[0] = _D_;
	    sequentialBuffer[1] = _E_;
	    sequentialBuffer[2] = _C_;
	  break;

	  default:
	    sequentialBuffer[0] = _STAR_;
	    sequentialBuffer[1] = _STAR_;
	    sequentialBuffer[2] = _STAR_;	  
  }					
  //					 
}
//

// set 3 digits according to the curent Year
void showYear (void ) {
  sequentialBuffer[0] = _YEAR_;
  sequentialBuffer[1] = pgm_read_word (  tensBCD + (uint8_t) rtc.datetime.year.tens);
  sequentialBuffer[2] = pgm_read_word ( unitsBCD + (uint8_t) rtc.datetime.year.units); 
}
//



//     ___ _         _     ___           _          
//    / __| |___  __| |__ | __|_ _  __ _(_)_ _  ___ 
//   | (__| / _ \/ _| / / | _|| ' \/ _` | | ' \/ -_)
//    \___|_\___/\__|_\_\ |___|_||_\__, |_|_||_\___|
//                                 |___/            

// 
//  Clock engine stuff
// 

// clock engine
void runClockEngine( t_buttonEvents btEvent ){

  switch ( operatingState ) {
    case ST_SHOW_TIME:    
	     if ( btEvent == EV_MODE_LONG ) advanceDisplayMode();
		 if ( btEvent == EV_SET_LONG ) operatingState = ST_SET_HOUR;
		 showTime();
	break;
	
    case ST_SET_HOUR: 
         showHour();	
		 if ( btEvent == EV_SET_LONG )  operatingState = ST_SHOW_TIME;	
		 if ( btEvent == EV_SET_PULSE ) operatingState = ST_SET_MINUTE;		 
	     if ( btEvent == EV_MODE_PULSE ) advanceHour(1);	
	     if ( btEvent == EV_MODE_LONG )  advanceHour(10);
	break;
	
    case ST_SET_MINUTE: 
         showMinute();	
		 if ( btEvent == EV_SET_LONG )  operatingState = ST_SHOW_TIME;	
		 if ( btEvent == EV_SET_PULSE ) operatingState = ST_SET_DAY;		 
	     if ( btEvent == EV_MODE_PULSE ) advanceMinute(1);	
	     if ( btEvent == EV_MODE_LONG )  advanceMinute(10);	
	break;
	
    case ST_SET_DAY: 
         showDay();	
		 if ( btEvent == EV_SET_LONG )  operatingState = ST_SHOW_TIME;	
		 if ( btEvent == EV_SET_PULSE ) operatingState = ST_SET_MONTH;		 
	     if ( btEvent == EV_MODE_PULSE ) advanceDay(1);	
	     if ( btEvent == EV_MODE_LONG )  advanceDay(10);	
	break;
	
    case ST_SET_MONTH:
	     showMonth();
		 if ( btEvent == EV_SET_LONG )  operatingState = ST_SHOW_TIME;	
		 if ( btEvent == EV_SET_PULSE ) operatingState = ST_SET_YEAR;		 
	     if ( btEvent == EV_MODE_PULSE ) advanceMonth(1);	
	     if ( btEvent == EV_MODE_LONG )  advanceMonth(6);
    break;
	
    case ST_SET_YEAR:
	     showYear();
		 if ( btEvent == EV_SET_LONG )  operatingState = ST_SHOW_TIME;	
		 if ( btEvent == EV_SET_PULSE ) operatingState = ST_SET_HOUR;		 
	     if ( btEvent == EV_MODE_PULSE ) advanceYear(1);	
	     if ( btEvent == EV_MODE_LONG )  advanceYear(10);
	break;

    }
    
/*    
    // check if time changes
    if (timeChanged) {
        timeChanged = false;
        adjustDOW ( &rtc );
        if ( !writeRtc ( &rtc ) )
           return false; 
    }

    return true;
*/    
}
//

// advance display mode
void advanceDisplayMode (void) {
    switch (displayMode) {
       case MD_SEASON_24HOUR:
         displayMode = MD_SEASON_12HOUR;
         break;	 
      case MD_SEASON_12HOUR:
         displayMode = MD_ULTRADIAN_SEASON;
         break;	  
      case MD_ULTRADIAN_SEASON:
         displayMode = MD_ULTRADIAN_NO_SEASON;
         break;
      default:
         displayMode = MD_SEASON_24HOUR;
         break; 
    }
}
//



//    ___  _         _             ___           _          
//   |   \(_)____ __| |__ _ _  _  | __|_ _  __ _(_)_ _  ___ 
//   | |) | (_-< '_ \ / _` | || | | _|| ' \/ _` | | ' \/ -_)
//   |___/|_/__/ .__/_\__,_|\_, | |___|_||_\__, |_|_||_\___|
//             |_|          |__/           |___/            

// Display engine
void runDisplayEngine( void ) {
  static uint8_t sequentialCounter = 0;
  static uint16_t d16buf=0;    
  
  if ( operatingState == ST_SHOW_TIME ) { // Display mode
      d16buf = displ16SegBuffer;
      sequentialCounter = 0; 
      
  } else {  // Setup mode, show buffer sequentially
  
   clearLeds(&ledRing );
   
   switch (sequentialCounter) {
        case 0:
        case 1:
       // case 1: // show first element
            d16buf = sequentialBuffer[0];        
            break;
     
        case 44:  // show second element
            d16buf = sequentialBuffer[1];
            break;   
  
        case 88: // show third element
            d16buf = sequentialBuffer[2];
            break; 
 
        case 40: // blank
        case 84: // blank
        case 128: // blank
            d16buf = 0;    
            break;   
 
    }         
    
    sequentialCounter++; 
    if (sequentialCounter > 192 ) sequentialCounter = 0;   
  
 }
 
  // update displays
  
   // shift out data to 16 segment     
  shiftOut16seg (d16buf);
  
   // show LED ring
  showLeds ( &ledRing ); 
 

}
//

// show Time
void showTime       (void) {
   uint16_t month = 10 * (uint8_t)rtc.datetime.monthCty.tens + (uint8_t)rtc.datetime.monthCty.units ;
   uint16_t day   = 10 * (uint8_t)rtc.datetime.date.tens     + (uint8_t)rtc.datetime.date.units ;
   uint8_t hour   = 10 * (uint8_t)rtc.datetime.hours.tens    + (uint8_t)rtc.datetime.hours.units ;
   uint8_t minute = 10 * (uint8_t)rtc.datetime.minutes.tens  + (uint8_t)rtc.datetime.minutes.units ;
   uint8_t second = 10 * (uint8_t)rtc.datetime.seconds.tens  + (uint8_t)rtc.datetime.seconds.units ;  
   
   uint16_t dayOfYear = dotyFebruary1 ( month, day );
   
   
    switch (displayMode) {
       case MD_SEASON_24HOUR: // 16 segments season, led ring 24 hour
         showSeason16Seg ( dayOfYear ) ;
         showTime24HledRing ( hour, minute );

         break;	 
         
      case MD_SEASON_12HOUR: // 16 segments season, led ring 12 hour + minutes
         showSeason16Seg ( dayOfYear ) ;
         showTime12HledRing ( hour, minute, second );
         break;	  
         
      case MD_ULTRADIAN_SEASON: // 16 segments clock ultradian, led ring season
         showTimeUltradian16seg (hour, minute );	      
         showSeasonLedRing ( dayOfYear );
         break;

      case MD_ULTRADIAN_NO_SEASON: // 16 segments clock ultradian, led ring season
      default:
         showTimeUltradian16seg (hour, minute );
         showUltradianIntervalLedRing ( minute, second );	 
         break; 

         	
    }
}
//



// ******************************************************************************************************************
//
//   Display mode base functions
//

//
// Display on 16 segment 
//

// show season on 16 segment display
void showSeason16Seg ( uint16_t dayOfYear ) { 
  uint8_t th = 71;  // count down to next threshold
  
  while ( dayOfYear < pgm_read_word (dayThreshold + th ) ) {
    th--;
  }
  
  // set correspondent pattern
  displ16SegBuffer =  pgm_read_word( seasonPattern16segment + th ) ;
}
//

// Show time on 16 segment display (ultradian cycle)
void showTimeUltradian16seg (uint8_t hour, uint8_t minute ){
	uint8_t k;
	
	uint8_t inner = ( (hour * 60 + minute) / 90 ) % 8;
    uint8_t outer = ( (hour * 60 + minute) / 10 ) % 9;
	
	displ16SegBuffer  = pgm_read_word( &(hour8[inner]) );// 90 minute interval
	
	while ( outer > 0) { // add 10 minute intervals passed
         k = (outer-1 + inner) % 8;
		 displ16SegBuffer |= pgm_read_word( &(phase[k]) );
        outer--;
      }
   
}


//
// Display on LED Ring
//
// show Season using color table
void showSeasonLedRing ( uint16_t dayOfYear  ){
	uint8_t th = 23;  // count down to next threshold
  while ( dayOfYear < pgm_read_word (dayThreshold24intervals + th ) ) {
    th--;
  }
    // set LED color according to season    
	clearLeds ( &ledRing) ;	
    ledRing.led[ th ].red    = ( pgm_read_byte( &(seasonColors[th][0])) * ledRingBrightness ) >> 8;
    ledRing.led[ th ].green  = ( pgm_read_byte( &(seasonColors[th][1])) * ledRingBrightness ) >> 8;
    ledRing.led[ th ].blue   = ( pgm_read_byte( &(seasonColors[th][2])) * ledRingBrightness ) >> 8;   	  	
}
//

//displays time using single red LED on ring considering 24 hours for full circle
void showTime24HledRing ( uint8_t hour, uint8_t minute ){
	clearLeds ( &ledRing) ;
	ledRing.led[hour].red   = ledRingBrightness ;	
  //  showLeds    ( &ledRing );
}
//

//displays time using red LED on ring for hours, considering 12 hours for full circle
// shows every 2:30 intervals using blue LED to complete 1 hour in a full circle
void showTime12HledRing ( uint8_t hour, uint8_t minute, uint8_t second ){
	uint8_t ih12 = (( hour   * 60 + minute ) / 30  )  % 12;
	uint8_t i2m5 = (( minute * 60 + second ) / 150 )  ;
	clearLeds ( &ledRing) ;
	ledRing.led[ ih12 ].red    = ledRingBrightness ;
	ledRing.led[ i2m5 ].blue   = ledRingBrightness ;
   // showLeds    ( &ledRing );	
}
//

// displays 10 minute ultradian intervals using red LED, considering 24 intervals (of 25 seconds each)
void showUltradianIntervalLedRing ( uint8_t minute, uint8_t second  ){

    uint8_t tenth = ( (minute * 60 + second) / 25 ) % 24; 

    // 	if (tenth == 0)   // uncomment this line for fullfill the ring
 	clearLeds ( &ledRing) ;
 	
	ledRing.led[tenth].red   = ledRingBrightness ;
	showLeds    ( &ledRing );
}
//






//


// ******************************************************************************************************************
//
//   Display error 
//
// RTC fail display routine
void runDisplayError( void ) {
   static uint8_t count=0;
   static uint8_t j = 0;
 
   // circulate led ring
   if ( (count & 3) == 0 ) {
       clearLeds ( &ledRing) ;
       ledRing.led[j].red   = 80 ;
       showLeds ( &ledRing ); 
       if (j<24) j++; else j = 0;
   }
   
   // blink star pattern
   if (count == 0 ) {
      shiftOut16seg ( _STAR_ );
  
   } else if (count == 60 )
         shiftOut16seg ( _BLANK_ );  
        
   if (count < 100 ) count++; else count = 0;
    
}
//




//     __ _      _    
//    / _(_)_ _ (_)___
//   |  _| | ' \| (_-<
//   |_| |_|_||_|_/__/
//                    
