// TC4_diag.ino
//
// TC4 diagnostic utility program to program the eeprom calibration
// and do some low level test of the TC4 board.
//
// Derived from aCatuai.ino and other utility programs
// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2011, MLG Properties, LLC
// All rights reserved.
//
// Contributor:  Jim Gallt
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of 
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list 
//   of conditions and the following disclaimer in the documentation and/or other materials 
//   provided with the distribution.
//
//   Neither the name of the MLG Properties, LLC nor the names of its contributors may be 
//   used to endorse or promote products derived from this software without specific prior 
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------
// TC4_diag Rev.s
// V0.01 Sept. 27,2013 Stan Gardner Initial rev
// V0.02 Sept. 28,2013 Stan Garnder Add ADC test
// V0.03 Oct. 7,2013   Stan Gardner added EEprom dump more printout information
#define BANNER_CAT "TC4_diag V0.03" // version

#if defined(ARDUINO) && ARDUINO >= 100
#define _READ read
#define _WRITE write
#else
#define _READ receive
#define _WRITE send
#endif


// The user.h file contains user-definable compiler options
// It must be located in the same folder as TC4_diag.ino
#include "user.h"

// this library included with the arduino distribution
#include <Wire.h>

#include <avr/pgmspace.h>
#include <alloca.h>

// these "contributed" libraries must be installed in your sketchbook's arduino/libraries folder
#include <thermocouple.h>
#include <cADC.h>
#include <mcEEPROM.h>
// ------------------------ other compile directives
#define MIN_DELAY 300   // ms between ADC samples (tested OK at 270)
#define NCHAN 4  // number of TC input channels
#define DP 1  // decimal places for output on serial port
#define D_MULT 0.001 // multiplier to convert temperatures from int to float
#define MAX_COMMAND 80 // max length of a command string
#define LOOPTIME 1000 // cycle time, in ms

//----------------- user interface ----------------
#define SAMPLES_10 11
#define SAMPLES_100 101
#define A_EEPROM ADDR_BITS

uint8_t channels_displayed = 1;
int calibration_temp = 0; //Temp in Celcius for calibration ref
uint8_t calibration_TC = 1;  //which Tc to use for TC calibration
uint8_t verbose_mode = 0;  //toggle to display extra debug info 

char *input_ptr;
//shadow memory of information to fill eeprom calibration
calBlock blank_fill= {
 "TC4_SHIELD",
  "4.00",  // edit this field to comply with the version of your TC4 board
  1.000, // gain
  0, // uV offset
  0.0, // type T offset temp
  0.0 // type K offset temp
};


// --------------------------------------------------------------
// global variables

// eeprom calibration data structure
calBlock caldata;
// class objects
mcEEPROM eeprom;
cADC adc( A_ADC ); // MCP3424
ambSensor amb( A_AMB ); // MCP9800
filterRC fT[NCHAN]; // filter for displayed/logged ET, BT

int32_t temps[NCHAN]; //  stored temperatures are divided by D_MULT
int32_t ftemps[NCHAN]; // heavily filtered temps
int32_t ftimes[NCHAN]; // filtered sample timestamps
int32_t flast[NCHAN]; // for calculating derivative
int32_t lasttimes[NCHAN]; // for calculating derivative

uint8_t chan_map[NCHAN] = { LOGCHAN1, LOGCHAN2, LOGCHAN3, LOGCHAN4 };

// used in main loop
float timestamp = 0;
boolean first;
uint32_t nextLoop;
float reftime; // reference for measuring elapsed time
boolean standAlone = true; // default is standalone mode

// prototypes
void serialPrintln_P(const prog_char* s);
void serialPrint_P(const prog_char* s);
float calcRise( int32_t T1, int32_t T2, int32_t t1, int32_t t2 );
void logger(void);
void append( char* str, char c );
void resetTimer(void);
void display_cal(void);
void display_cal_block(calBlock *caldata);
void display_menu(void);
int eeprom_dump(int page);
void show_variables(void);
int check_adc(void);
int check_MCP9800(void);
void input_error(void);
void input_accepted(void);
void processCommand(void);  // a newline character has been received, so process the command
void checkSerial(void);  // buffer the input from the serial port
void checkStatus( uint32_t ms ); // this is an active delay loop
void get_samples(void); // this function talks to the amb sensor and ADC via I2C
int check_adc(void);




void show_variables(void){
  serialPrintln_P(PSTR("Program Information"));
  Serial.print(channels_displayed);
  serialPrintln_P(PSTR(" Number of channels  for Temp"));
  Serial.print(calibration_temp);
  serialPrintln_P(PSTR(" Calibration Reference Temperature"));
  Serial.print(calibration_TC);
  serialPrintln_P(PSTR(" Which TC is used for Calibration"));
  Serial.print(verbose_mode);
  serialPrintln_P(PSTR(" Verbose Debug Mode setting"));

}

char command[MAX_COMMAND+1]; // input buffer for commands from the serial port
uint8_t sample_cnt = 0;

// T1, T2 = temperatures x 1000
// t1, t2 = time marks, milliseconds
// ---------------------------------------------------
float calcRise( int32_t T1, int32_t T2, int32_t t1, int32_t t2 ) {
  int32_t dt = t2 - t1;
  if( dt == 0 ) return 0.0;  // fixme -- throw an exception here?
  float dT = (T2 - T1) * D_MULT;
  float dS = dt * 0.001; // convert from milli-seconds to seconds
  return ( dT / dS ) * 60.0; // rise per minute
}

void serialPrint_P(const prog_char* s)
{
   char* p = (char*)alloca(strlen_P(s) + 1);
  strcpy_P(p, s);
  Serial.print(p);
}

void serialPrintln_P(const prog_char* s)
{
  serialPrint_P(s);
  serialPrint_P(PSTR("\n"));
}
//serialPrint_P(PSTR("Hello"));

// ------------------------------------------------------------------
void logger()
{
  int i;
  float t1,t2,t_amb;

  // print timestamp from when samples were taken
  Serial.print( timestamp, DP );

  // print ambient
  serialPrint_P(PSTR(","));
#ifdef CELSIUS
  t_amb = amb.getAmbC();
#else
  t_amb = amb.getAmbF();
#endif
  Serial.print( t_amb, DP );
  // print temperature, rate for each channel
  i = 0;
  if( channels_displayed >= 1 ) {
    serialPrint_P(PSTR(","));
    Serial.print( t1 = D_MULT*temps[i], DP );
    i++;
  };
  
  if( channels_displayed >= 2 ) {
    serialPrint_P(PSTR(","));
    Serial.print( t2 = D_MULT * temps[i], DP );
    i++;
  };
  
  if( channels_displayed >= 3 ) {
    serialPrint_P(PSTR(","));
    Serial.print( D_MULT * temps[i], DP );
    i++;
  };
  
  if( channels_displayed >= 4 ) {
    serialPrint_P(PSTR(","));
    Serial.print( D_MULT * temps[i], DP );
    i++;
  };
  Serial.println();
};

// --------------------------------------------

// -------------------------------------
void append( char* str, char c ) { // reinventing the wheel
  int len = strlen( str );
  str[len] = c;
  str[len+1] = '\0';
}

// ----------------------------
void resetTimer() {
  nextLoop = 10 + millis(); // wait 10 ms and force a sample/log cycle
  reftime = 0.001 * nextLoop; // reset the reference point for timestamp
  return;
}
void display_cal() {
  if( readCalBlock( eeprom, caldata ) ) {
    serialPrintln_P(PSTR(("# EEPROM data read: ")));
    display_cal_block(&caldata);
  }
  else { // if there was a problem with EEPROM read, then use default values
    serialPrintln_P(PSTR("# Failed to read EEPROM.  Using default calibration data. "));
  }   

  return;
}

void display_cal_block(calBlock *caldata) {
    Serial.print("# PCB = ");
    Serial.print( caldata->PCB); serialPrint_P(PSTR("  Version "));
    Serial.println( caldata->version );
    Serial.print("# cal gain ");
    Serial.print( caldata->cal_gain, 6 ); serialPrint_P(PSTR("  cal offset "));
    Serial.println( caldata->cal_offset );
    Serial.print("# K offset ");
    Serial.print( caldata->K_offset, 2 ); serialPrint_P(PSTR("  T offset "));
    Serial.println( caldata->T_offset, 2 );
  return;
}

void display_menu(){
serialPrintln_P(PSTR(""));
  serialPrintln_P(PSTR("a = display cal block"));
  serialPrintln_P(PSTR("b = display cal fill info"));
  serialPrintln_P(PSTR("c = write fill block to eeprom "));
  serialPrintln_P(PSTR("d = change fill PCB, Should start TC4"));
  serialPrintln_P(PSTR("e = change fill Version"));
  serialPrintln_P(PSTR("f = change fill Cal Gain"));
  serialPrintln_P(PSTR("g = change fill Cal offset"));
  serialPrintln_P(PSTR("h = change fill T offset"));
  serialPrintln_P(PSTR("j = change fill K offset"));
  serialPrintln_P(PSTR("k = Set Number of TC channels to display "));
  serialPrintln_P(PSTR("m = read TC(s) up to 1000 times"));
  serialPrintln_P(PSTR("n = test adc"));
  serialPrintln_P(PSTR("q = test MCP9800"));
  serialPrintln_P(PSTR("r = eeprom dump"));
  serialPrintln_P(PSTR("s = Calibration reference temp"));
  serialPrintln_P(PSTR("v = toggle verbose debug mode"));
  serialPrintln_P(PSTR("V = show program variables"));
  serialPrintln_P(PSTR("Enter a Letter to run item"));
  return;
}

int eeprom_dump(int page){
uint8_t a=0,page_lo=0,page_hi=0;
int j=0,i=0;
  page_hi = ((0x80*page & 0xFF00)>>8);
  if(page % 2 == 1)
    page_lo = 0x80;
   else
    page_lo = 0x00; 
  Wire.beginTransmission( A_EEPROM );
  for(j=0;j<8;j++){
    Wire.beginTransmission( A_EEPROM );
    Wire._WRITE( page_hi); //address
    Wire._WRITE( 0x10 * j | page_lo ); // 
    Wire.endTransmission();
    Wire.requestFrom( A_EEPROM, 16 );
    serialPrint_P(PSTR("address 0x"));
      if((page_hi == 0) || (page_hi < 0x10))    
        Serial.print(0,HEX);
    Serial.print(page_hi,HEX);
    if((j == 0) && (page_lo == 0))    
        Serial.print(0,HEX);
    Serial.print((page_lo+j*0x10),HEX);
    for(i=0;i<16;i++){
      serialPrint_P(PSTR(" "));
      a = Wire._READ(); // first data byte
      if((a == 0) || (a < 0x10))    
        Serial.print(0,HEX);
      Serial.print(a,HEX);     
    }
    serialPrint_P(PSTR("\n"));
    Wire.endTransmission();
  }
}

int check_adc(){
// check ADC is ready to process conversions
// Request a conversion, check it started
// wait make sure it is completed in normal delay
// check various bits can be set and cleared
  Wire.beginTransmission( A_ADC );
  Wire._WRITE( 0x2a );
  Wire.endTransmission();
  Wire.requestFrom( A_ADC, 4 );
  uint8_t a = Wire._READ(); // first data byte
  uint8_t b = Wire._READ(); // second data byte
  uint8_t c = Wire._READ(); // 3rd data byte
  uint8_t stat = Wire._READ(); // read the status byte returned from the ADC
//#ifdef DEBUG_I2C     
//  Serial.println(a);  // debug
//  Serial.println(b); // debug
//  Serial.println(c); // debug
//  Serial.println(stat); // debug
//#endif
  if(stat & 0x80){
    serialPrintln_P(PSTR("Error ADC not ready"));
    return 0x01;
  }
  if((stat & 0x7F) != 0x2a){
    serialPrintln_P(PSTR("Error ADC config bits 1 does not match"));
    return (int)0x01;
  }
  Wire.beginTransmission(A_ADC);
  Wire._WRITE( 0xc5 );
  Wire.endTransmission();
  Wire.requestFrom( A_ADC, 4 );
  a = Wire._READ(); // first data byte
  b = Wire._READ(); // second data byte
  c = Wire._READ(); // 3rd data byte
  stat = Wire._READ(); // read the status byte returned from the ADC
  if((stat & 0x80) == 0x00){
    serialPrintln_P(PSTR("Error ADC ready too fast"));
    return (int)0x01;
  }
  if((stat & 0x7F) != 0x45){
//    Serial.println("Error ADC config bits 2 does not match");
    serialPrintln_P(PSTR("Error ADC config bits 2 does not match"));
    return (int)0x01;
  }
  checkStatus(MIN_DELAY); // give the chips time to perform the conversions
  Wire.requestFrom(A_ADC, 4);
  a = Wire._READ(); // first data byte
  b = Wire._READ(); // second data byte
  c = Wire._READ(); // 3rd data byte
  stat = Wire._READ(); // read the status byte returned from the ADC
//#ifdef DEBUG_I2C  
//  Serial.println(a);  // debug
//  Serial.println(b); // debug
//  Serial.println(c); // debug
//  Serial.println(stat); // debug
//#endif
  if(stat & 0x80){
//    Serial.println("Error ADC not ready after delay");
    serialPrintln_P(PSTR("Error ADC not ready after delay"));
    return (int)0x01;
  }
  serialPrintln_P(PSTR("ADC test passed"));
  return (int)0x00; 
}
#define DEBUG_AMB 1
void debug_MCP9800(uint8_t a){
#ifdef DEBUG_AMB
  Serial.println(a,HEX);
#endif
  return;
}

int check_MCP9800(){
// check Temp sensor operation
// write and read programmable registers
uint8_t a=0,b=0,c=0;
  Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x01 ); //address
  Wire._WRITE( 0xaa ); // config data
  Wire.endTransmission();
  Wire.requestFrom( A_AMB, 1 );
  a = Wire._READ(); // first data byte
  debug_MCP9800(a);  // debug
  if(a != 0xaa)
    c++;
  Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x01 ); //address
  Wire._WRITE( 0x55 ); // config data
  Wire.endTransmission();
  Wire.requestFrom( A_AMB, 1 );
  a = Wire._READ(); // first data byte
  debug_MCP9800(a);  // debug
  if(a != 0x55)
    c++;

//hysteresis register
 Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x02 ); //address
  Wire._WRITE( 0x55 ); // config data
  Wire._WRITE( 0x00 ); // config data
  Wire.endTransmission();
  Wire.requestFrom( A_AMB, 2 );
  a = Wire._READ(); // first data byte
  b = Wire._READ(); // first data byte
  debug_MCP9800(a);  // debug
  debug_MCP9800(b);  // debug
  if((a != 0x55) || ((b &0x80) != 0x00))
    c++;

 Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x02 ); //address
  Wire._WRITE( 0xaa ); // config data
  Wire._WRITE( 0x80 ); // config data
  Wire.endTransmission();
  Wire.requestFrom( A_AMB, 2 );
  a = Wire._READ(); // first data byte
  b = Wire._READ(); // first data byte
  debug_MCP9800(a);  // debug
  debug_MCP9800(b);  // debug
  if((a != 0xaa) || ((b & 0x80) != 0x80))
    c++;

  //set back to default
 Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x02 ); //address
  Wire._WRITE( 0x4b ); // config data
  Wire._WRITE( 0x00 ); // config data
  Wire.endTransmission();

//alarm set point register
 Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x03 ); //address
  Wire._WRITE( 0x55 ); // config data
  Wire._WRITE( 0x00 ); // config data
  Wire.endTransmission();
  Wire.requestFrom( A_AMB, 2 );
  a = Wire._READ(); // first data byte
  b = Wire._READ(); // first data byte
  debug_MCP9800(a);  // debug
  debug_MCP9800(b);  // debug
  if((a != 0x55) || ((b &0x80) != 0x00))
    c++;

 Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x03 ); //address
  Wire._WRITE( 0xaa ); // config data
  Wire._WRITE( 0x80 ); // config data
  Wire.endTransmission();
  Wire.requestFrom( A_AMB, 2 );
  a = Wire._READ(); // first data byte
  b = Wire._READ(); // first data byte
  debug_MCP9800(a);  // debug
  debug_MCP9800(b);  // debug
  if((a != 0xaa) || ((b &0x80) != 0x80))
    c++;

  //set back to default
 Wire.beginTransmission( A_AMB );
  Wire._WRITE( 0x03 ); //address
  Wire._WRITE( 0x50 ); // config data
  Wire._WRITE( 0x00 ); // config data
  Wire.endTransmission();

  if(c)
    serialPrintln_P(PSTR("MCP9800 test fail"));
  else
    serialPrintln_P(PSTR("MCP9800 test pass"));
  return 0x00;
}
void input_accepted(void){serialPrintln_P(PSTR("Input Accepted"));}
void input_error(void){serialPrintln_P(PSTR("Error - line too short"));}
// -------------------------------------
void processCommand() {  // a newline character has been received, so process the command
//  char [MAX_COMMAND+1] cmd_buffer ="";
  double temp_f;
  int temp_i;
 switch (command[0]){
   case 'a':
    display_cal();
   break;
   case 'b':
    serialPrintln_P(PSTR("# EEPROM data fill: "));
    display_cal_block(&blank_fill);
    break;
   case 'c':
    eeprom.write( 0, (uint8_t*) &blank_fill, sizeof( blank_fill ) );
    serialPrintln_P(PSTR(""));
    serialPrintln_P(PSTR("# New content "));
    display_cal();
    adc.setCal( caldata.cal_gain, caldata.cal_offset );
    amb.setOffset( caldata.K_offset );
    serialPrintln_P(PSTR(""));
    break;
   case 'd':
      if(strlen(command) >= 3){
        input_ptr = &blank_fill.PCB[0];
        strncpy(input_ptr,command+2,sizeof(blank_fill.PCB));
        input_accepted();
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: d SingleSpace PCBString"));
      }        
    break;
   case 'e':
      if(strlen(command) >= 3){
        input_ptr = &blank_fill.version[0];
        strncpy(input_ptr,command+2,sizeof(blank_fill.version));
        input_accepted();
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: e SingleSpace VersionString"));
      }        
    break;
   case 'f':
      if(strlen(command) >= 3){
        temp_f = atof(command+2);
        blank_fill.cal_gain = (float)temp_f;        
        input_accepted();
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: f SingleSpace FloatingPointValue"));
      }        
    break;
   case 'g':
      if(strlen(command) >= 3){
        temp_i = atoi(command+2);
          blank_fill.cal_offset = (int16_t)temp_i;                
        input_accepted();
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: g SingleSpace IntegerValue"));
      }        
    break;
   case 'h':
      if(strlen(command) >= 3){
        temp_f = atof(command+2);
        blank_fill.T_offset = (float)temp_f;        
        input_accepted();
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: h SingleSpace FloatingPointValue"));
      }        
    break;
   case 'j':
      if(strlen(command) >= 3){
        temp_f = atof(command+2);
        blank_fill.K_offset = (float)temp_f;        
        input_accepted();
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: j SingleSpace FloatingPointValue"));
      }        
    break;
   case 'k':
      if(strlen(command) >= 3){
        temp_i = atoi(command+2);
        if((temp_i > NCHAN) || (temp_i <= 0)){
          serialPrintln_P(PSTR("Error, enter 1,2,3,4 only"));
        }
        else{       
          channels_displayed = (uint8_t)temp_i;        
          input_accepted();
        }
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: k SingleSpace NumberChannelsDisplayedValue"));
      }        
    break;
   case 'm':
      if(strlen(command) >= 3){
        temp_i = atoi(command+2);
        if((temp_i > 1000) || (temp_i <= 0)){
          serialPrintln_P(PSTR("Error, enter 1 TO 1000 only"));
        }
        else{       
          sample_cnt = temp_i+1;
          first = true;
          serialPrint_P(PSTR("# time,ambient,T0"));
          if( channels_displayed >= 2 ) serialPrint_P(PSTR(",T1"));
          if( channels_displayed >= 3 ) serialPrint_P(PSTR(",T2"));
          if( channels_displayed >= 4 ) serialPrint_P(PSTR(",T3"));
          serialPrintln_P(PSTR(""));
          resetTimer();
         }
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: m SingleSpace NumberChannelsDisplayedValue"));
      }        
    break;
   case 'n':   
   check_adc();
   break;
   case 'q':   
   check_MCP9800();
   break;
    case 'r':   
      if(strlen(command) >= 3){
         
        temp_i = atoi(command+2);
        if((temp_i > 511) || (temp_i < 0)){
          serialPrintln_P(PSTR("Error, enter 0 TO 511 only"));
        }
        else{       
           eeprom_dump(temp_i);
        }
      }
      else{
      eeprom_dump(0);
      }
   break;
   case 's':
      if(strlen(command) >= 3){
        temp_i = atoi(command+2);
        if((temp_i > 1000) || (temp_i < 0)){
          serialPrintln_P(PSTR("Error, enter 0 TO 1000 only"));
        }
        else{       
          calibration_temp =temp_i;
          input_accepted();
         }
      }
      else{
        input_error();
        serialPrintln_P(PSTR("Usage: m SingleSpace NumberChannelsDisplayedValue"));
      }        
    break;

  case 'V':
      show_variables();
      break;
  case 'v':
      if(verbose_mode){
        verbose_mode = 0;
        serialPrintln_P(PSTR("Verbose Mode Off"));
      }
      else{
        verbose_mode = 1;
        serialPrintln_P(PSTR("Verbose Mode On"));
      }
      break;
      
  case 'i':
  case 'l':
  case 'o':
    serialPrintln_P(PSTR("Not supported"));
  default:
  display_menu();
  break;
 }
  return;
}

// -------------------------------------
void checkSerial() {  // buffer the input from the serial port
  char c;
  while( Serial.available() > 0 ) {
    c = Serial.read();
    if( ( c == '\n' ) || ( strlen( command ) == MAX_COMMAND ) ) { // check for newline, or buffer overflow
      processCommand();
      strcpy( command, "" ); // empty the buffer
    } // end if
    else if( c != '\r' ) { // ignore CR for compatibility with CR-LF pairs
//      append( command, toupper(c) );
      append( command, c );
    } // end else
  } // end while
}

// ----------------------------------
void checkStatus( uint32_t ms ) { // this is an active delay loop
  uint32_t tod = millis();
  while( millis() < tod + ms ) {
  }
}

// --------------------------------------------------------------------------
void get_samples() // this function talks to the amb sensor and ADC via I2C
{
  int32_t v;
  TC_TYPE tc;
  float tempC;
  
  for( int j = 0; j < channels_displayed; j++ ) { // one-shot conversions on both chips
    adc.nextConversion( chan_map[j] ); // start ADC conversion on channel j
    amb.nextConversion(); // start ambient sensor conversion
    checkStatus( MIN_DELAY ); // give the chips time to perform the conversions
    ftimes[j] = millis(); // record timestamp for RoR calculations
    amb.readSensor(); // retrieve value from ambient temp register
    v = adc.readuV(); // retrieve microvolt sample from MCP3424
    tempC = tc.Temp_C( 0.001 * v, amb.getAmbC() ); // convert to Celsius
#ifdef CELSIUS
    v = round( tempC / D_MULT ); // store results as integers
#else
    v = round( C_TO_F( tempC ) / D_MULT ); // store results as integers
#endif
    temps[j] = fT[j].doFilter( v ); // apply digital filtering for display/logging
  }
}
  
// ------------------------------------------------------------------------
// MAIN
//
void setup()
{
  delay(500);
  Wire.begin(); 
  
  Serial.begin(BAUD);
  delay(500);
  amb.init( AMB_FILTER );  // initialize ambient temp filtering
  serialPrintln_P(PSTR(BANNER_CAT));

  // read calibration and identification data from eeprom
  if( readCalBlock( eeprom, caldata ) ) {
    serialPrintln_P(PSTR(("valid calblock found, using content")));
    adc.setCal( caldata.cal_gain, caldata.cal_offset );
    amb.setOffset( caldata.K_offset );
  }
  else { // if there was a problem with EEPROM read, then use default values
    serialPrintln_P(PSTR(("# Failed to read EEPROM.  Using default calibration data. ")));
    adc.setCal( CAL_GAIN, UV_OFFSET );
    amb.setOffset( AMB_OFFSET );
  }   
  display_menu();

  // write header to serial port

  fT[0].init( BT_FILTER ); // digital filtering on BT
  fT[1].init( ET_FILTER ); // digital filtering on ET

  
  delay( 1800 );
  nextLoop = 2000;
  reftime = 0.001 * nextLoop; // initialize reftime to the time of first sample
  first = true;
}

// -----------------------------------------------------------------
void loop() {
  float idletime;
  uint32_t thisLoop;

  // delay loop to force update on even LOOPTIME boundaries
  while ( millis() < nextLoop ) { // delay until time for next loop
      checkSerial(); // Has a command been received?
   }
  while(sample_cnt){
    sample_cnt--;
    thisLoop = millis(); // actual time marker for this loop
    timestamp = 0.001 * float( thisLoop ) - reftime; // system time, seconds, for this set of samples
    get_samples(); // retrieve values from MCP9800 and MCP3424
    if( first ) // use first samples for RoR base values only
      first = false;
    else {
      logger(); // output results to serial port
    }

    for( int j = 0; j < channels_displayed; j++ ) {
     flast[j] = ftemps[j]; // use previous values for calculating RoR
     lasttimes[j] = ftimes[j];
    }

    idletime = LOOPTIME - ( millis() - thisLoop );
    // arbitrary: complain if we don't have at least 50mS left
    if (idletime < 50 ) {
      serialPrint_P(PSTR("# idle: "));
      Serial.println(idletime);
    }
  }
  nextLoop += LOOPTIME; // time mark for start of next update 
}

