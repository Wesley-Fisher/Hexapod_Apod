#ifndef HEXAPOD_GLOBAL_VARIABLES_H_
#define HEXAPOD_GLOBAL_VARIABLES_H_

#include <Arduino.h>

//Default leg angle
extern const short cCoxaAngle1[6];

//[ANGLES]
extern short           CoxaAngle1[6];    //Actual Angle of the horizontal hip, decimals = 1
extern short           FemurAngle1[6];   //Actual Angle of the vertical hip, decimals = 1
extern short           TibiaAngle1[6];   //Actual Angle of the knee, decimals = 1

#ifdef c4DOF
extern short           TarsAngle1[6];	  //Actual Angle of the knee, decimals = 1
#endif


#endif
