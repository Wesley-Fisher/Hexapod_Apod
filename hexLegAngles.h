#ifndef HEX_LEG_ANGLES_H_
#define HEX_LEG_ANGLES_H_



//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1     -560      //Mechanical limits of the Right Rear Leg
#define cRRCoxaMax1     930
#define cRRFemurMin1    -1000
#define cRRFemurMax1    750
#define cRRTibiaMin1    -680
#define cRRTibiaMax1    900
#define cRRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cRMCoxaMin1     -500      //Mechanical limits of the Right Middle Leg
#define cRMCoxaMax1     500
#define cRMFemurMin1    -1000
#define cRMFemurMax1    750
#define cRMTibiaMin1    -680
#define cRMTibiaMax1    900
#define cRMTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRMTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cRFCoxaMin1     -950      //Mechanical limits of the Right Front Leg
#define cRFCoxaMax1     620
#define cRFFemurMin1    -1000
#define cRFFemurMax1    750
#define cRFTibiaMin1    -680
#define cRFTibiaMax1    900
#define cRFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLRCoxaMin1     -930      //Mechanical limits of the Left Rear Leg
#define cLRCoxaMax1     560
#define cLRFemurMin1    -1000
#define cLRFemurMax1    750
#define cLRTibiaMin1    -680
#define cLRTibiaMax1    900
#define cLRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLMCoxaMin1     -500      //Mechanical limits of the Left Middle Leg
#define cLMCoxaMax1     500
#define cLMFemurMin1    -1000
#define cLMFemurMax1    750
#define cLMTibiaMin1    -680
#define cLMTibiaMax1    900
#define cLMTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLMTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLFCoxaMin1     -620      //Mechanical limits of the Left Front Leg
#define cLFCoxaMax1     950
#define cLFFemurMin1    -1000
#define cLFFemurMax1    750
#define cLFTibiaMin1    -680
#define cLFTibiaMax1    900
#define cLFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..


//Min / imax values
const short cCoxaMin1[] PROGMEM = {cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1};
const short cCoxaMax1[] PROGMEM = {cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1};
const short cFemurMin1[] PROGMEM ={cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1};
const short cFemurMax1[] PROGMEM ={cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1};
const short cTibiaMin1[] PROGMEM ={cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1};
const short cTibiaMax1[] PROGMEM = {cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1};

#ifdef c4DOF
const short cTarsMin1[] PROGMEM = {cRRTarsMin1, cRMTarsMin1, cRFTarsMin1, cLRTarsMin1, cLMTarsMin1, cLFTarsMin1};
const short cTarsMax1[] PROGMEM = {cRRTarsMax1, cRMTarsMax1, cRFTarsMax1, cLRTarsMax1, cLMTarsMax1, cLFTarsMax1};
#endif

#define cRRCoxaAngle1   -450   //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    450      //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -450   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    450      //Default Coxa setup angle, decimals = 1


// Servo Horn offsets
#ifdef cRRFemurHornOffset1   // per leg configuration
static const short cFemurHornOffset1[] PROGMEM = {cRRFemurHornOffset1,  cRMFemurHornOffset1,  cRFFemurHornOffset1,  cLRFemurHornOffset1,  cLMFemurHornOffset1,  cLFFemurHornOffset1};
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cFemurHornOffset1
#define cFemurHornOffset1  0
#endif
#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#endif


#endif
