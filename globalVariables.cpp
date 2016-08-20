#include "globalVariables.h"
#include "hexLegAngles.h"

//Default leg angle
const short cCoxaAngle1[] PROGMEM = {cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1};


//[ANGLES]
short           CoxaAngle1[6];    //Actual Angle of the horizontal hip, decimals = 1
short           FemurAngle1[6];   //Actual Angle of the vertical hip, decimals = 1
short           TibiaAngle1[6];   //Actual Angle of the knee, decimals = 1

#ifdef c4DOF
short           TarsAngle1[6];	  //Actual Angle of the knee, decimals = 1
#endif
