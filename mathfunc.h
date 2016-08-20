#ifndef BOTBOARDUINO_TRIG_H_
#define BOTBOARDUINO_TRIG_H_

//GetSinCos / ArcCos
extern short           AngleDeg1;        //Input Angle in degrees, decimals = 1
extern short           sin4;             //Output Sinus of the given Angle, decimals = 4
extern short           cos4;            //Output Cosinus of the given Angle, decimals = 4
extern short           AngleRad4;        //Output Angle in radials, decimals = 4

//GetAtan2
extern short           AtanX;            //Input X
extern short           AtanY;            //Input Y
extern short           Atan4;            //ArcTan2 output
extern short           XYhyp2;            //Output presenting Hypotenuse of X and Y

long GetArcCos(short x);
void GetSinCos(short AngleDeg1);
short GetATan2(short AtanX, short AtanY);

unsigned long isqrt32 (unsigned long n);

#endif
