//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the Hex Robot.
//  
//    This version of the Configuration file is set up to run on the
//    Lynxmotion BotboardDuino board, which is similiar to the Arduino Duemilanove
//
//    This version of configuration file assumes that the servos will be controlled
//    by a Lynxmotion Servo controller SSC-32 and the user is using a Lynxmotion 
//    PS2 to control the robot.
//
//Date: March 18, 2012
//Programmer: Kurt (aka KurtE)
//
//NEW IN V1.1 (2013-05-17)
//   - Support for Arduino Pro Mini on Bot Board (originally for Basic Atom Pro)
//NEW IN V1.0
//   - First Release
//
//====================================================================

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
//[CONDITIONAL COMPILING] - COMMENT IF NOT WANTED
// Define other optional compnents to be included or not...

//comment if terminal monitor is not required
#define OPT_TERMINAL_MONITOR  

//uncomment the board you want to use
#define __BOTBOARDUINO__    //botboarduino board
//#define __BOTBOARD_ARDUINOPROMINI__  //arduino pro mini on botboard (originally for BasicAtomPro)

//====================================================================
#ifdef OPT_TERMINAL_MONITOR   // turning off terminal monitor will turn these off as well...
#define OPT_SSC_FORWARDER  // only useful if terminal monitor is enabled
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif

#define OPT_GPPLAYER

// Which type of control(s) do you want to compile in
#define DBGSerial         Serial

#if defined(UBRR1H)
#define SSCSerial         Serial1
#else
#endif

#define USEPS2

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// CHR-3
//==================================================================================================================================
#define USE_SSC32
//#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode.

//[SERIAL CONNECTIONS]

// Warning I will undefine some components as the non-megas don't have enough memory...
//#undef OPT_FIND_SERVO_OFFSETS 

#define cSSC_BAUD   38400   //SSC32 BAUD rate

//--------------------------------------------------------------------
//[Botboarduino Pin Numbers]
#ifdef __BOTBOARDUINO__
  #define SOUND_PIN       5   // Botboarduino JR pin number
  #define PS2_DAT         10        
  #define PS2_CMD         11
  #define PS2_SEL         12
  #define PS2_CLK         13
// If we are using a SSC-32 then:
// If were are running on an Arduino Mega we will use one of the hardware serial port, default to Serial1 above.
// If on Non mega, if the IO pins are set to 0, we will overload the hardware Serial port 
// Else we will user SoftwareSerial to talk to the SSC-32
  #define cSSC_OUT       8   //Output pin for Botboard - Input of SSC32 (Yellow)
  #define cSSC_IN        9   //Input pin for Botboard - Output of SSC32 (Blue)
#endif

#ifdef __BOTBOARD_ARDUINOPROMINI__
  #define SOUND_PIN      11   // Bot Board JR pin number (with Arduino Pro Mini plugged)
  #define PS2_DAT        14       
  #define PS2_CMD        15
  #define PS2_SEL        16
  #define PS2_CLK        17
// If we are using a SSC-32 then:
// If were are running on an Arduino Mega we will use one of the hardware serial port, default to Serial1 above.
// If on Non mega, if the IO pins are set to 0, we will overload the hardware Serial port 
// Else we will user SoftwareSerial to talk to the SSC-32
  #define cSSC_OUT       10   //Output pin for Botboard - Input of SSC32 (Yellow)
  #define cSSC_IN         9   //Input pin for Botboard - Output of SSC32 (Blue)
#endif






