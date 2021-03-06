#ifndef konalib001_H
#define konalib001_H
#include "WProgram.h" //I'm not sure why, but I couldn't use boolean variables in this file unless I have this

///////////
//button wiring
//Select wired to Analog port 0, Minus to Analog port 1, Escape and plus Analog port 2
//
// -------------------------- Settings and Values for Button switches and button wiring
#define SELECT_BUTTON      0      // read Select switch on analog input port 0
#define MINUS_BUTTON       1      // read MINUS switch on analog input port 1
#define PLUS_BUTTON        2      // read ESC and PLUS switches on analog input port 2
#define ESCAPE_BUTTON      3      // read Escape switch on analog input port 3

#define ENTERPIN        11      // read ENTER switch on digital i/o 11
#define ANYKEY         640      // value to decide if anykey is pushed
#define PLUS_VALUE     710      // value to decide if Plus is pushed
#define ESC_VALUE    860       // value to decide if ESC is pushed
#define DEBOUNCE       350      // debounce time
#define SELECT_VALUE     640      // value to decide if LEFT is pushed
#define MINUS_VALUE    640      // value to decide if Minus is pushed
#define FIFTH_VALUE    640      // value to decide if Select is pushed

#define SELECT     0       // set to this value if SELECT is pushed
#define DOWN_MINUS 1       // set to this value if Minus is pushed (was MINUS in 4 button setup)
#define UP_PLUS    2       // set to this value if Plus is pushed (was PLUS in 4 button setup)
#define ESCAPE     3       // set to this value if SELECT is pushed
#define FIFTH     4       // set to this value if RIGHT is pushed (was ESC in 4 button setup)
#define NOBUTTON   5       // set to this value if LEFT is pushed 


// --------------------------RTC settings
#define DS1307_ADDR 0x68  // RTC I2C address

// --------------------------Profile settings
#define NMAX 14 //  max. number of steps in the profile.  Increase this number to add
//                  more steps
#define NAME_SIZE 16 //size of the profile name varibles
#define NO_PROFILES 10  //max number of profiles

#define AUTO_TEMP   1   //roast method automatic using profile, temperature/time method
#define AUTO_ROR    2   //roast method automatic using profile, ror method
#define MANUAL_ROR  3   //roast method manual user controlled, ror method

#define NO_SEGMENTS 2 // currently support 3 roast segments

#define PROFILE_ADDR_PID 200 // address in EEprom to store the PID setup data
#define PROFILE_ADDR_01 5000 // address in EEprom to store the profile data
#define PROFILE_ADDR_RX 5000 // address in EEprom to store the profile data, for data sent from PC

//structure for temp storage of the profiles
struct profile {
  int index;      //0-1
  char name[16];  //2-17 
  char date[16];  //format yyyy/mm/dd hh:mm
  int ror[NMAX];
  int targ_temp[NMAX];
//	  int temp[NMAX];
  int time[NMAX];
  int offset[NMAX];
  int speed[NMAX];
  int delta_temp[NMAX];
  int tbd01[NMAX];
  int tbd02[NMAX];
  int tbd03[NMAX];	
  int maxtemp;
	int profile_method;

};

profile myprofile;  //structure used to read in a profile from eeprom

//profile myprofile_w;  //structure used to write out a profile to eeprom

//structure for storage of PID info in EEprom
struct PID_struc {
  int init_pattern;
  float Pb;
  float I;  
  float D; 
  float PID_factor;
  int starttemp;
  int maxtemp;
  int segment_1;
  int segment_2;
  int seg0_bias;
  int seg1_bias;
  int seg2_bias;
  int seg0_min;
  int seg1_min;
  int seg2_min;
  int startheat;
  };

PID_struc myPID_r;  //structure used to read in PID data

//PID_struc myPID_w;  //structure used to write in PID data


//#ifdef ALPENROST
//---------------------------Port Expander Alpenrost control bits
// bit 3 = flap close, bit 2 = flap open, bit 1 = motor on
#define ALP_MOTOR_ON            (uint8_t) B00000001
#define ALP_FLAP_OPEN           (uint8_t) B00000010
#define ALP_FLAP_OPEN_MOTOR_ON  (uint8_t) B00000011
#define ALP_FLAP_CLOSE          (uint8_t) B00000100
#define ALP_FLAP_CLOSE_MOTOR_ON (uint8_t) B00000101
#define ALL_ON                  (uint8_t) B11111111
//#endif

/*
For information, the Arduino Discrete IO pin assignments
Serial RX			0
Serial TX			1
         			2
  	                3
				    4
 	                5  
 	                6  
				    7
		            8
Heater PWM Out		9
Fan PWM Out			10
 	                11 
				    12
				    13

A0		
A1		
A2		
A3		
A4		I2C
A5		I2C

*/



#endif