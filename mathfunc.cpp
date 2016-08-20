#include "mathfunc.h"
#include "Hex_Globals.h"


//GetSinCos / ArcCos
short           AngleDeg1;        //Input Angle in degrees, decimals = 1
short           sin4;             //Output Sinus of the given Angle, decimals = 4
short           cos4;            //Output Cosinus of the given Angle, decimals = 4
short           AngleRad4;        //Output Angle in radials, decimals = 4

//GetAtan2
short           AtanX;            //Input X
short           AtanY;            //Input Y
short           Atan4;            //ArcTan2 output
short           XYhyp2;            //Output presenting Hypotenuse of X and Y



short ApproxSin(short x);
void ApproxCos(short x);

//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4          - Output ARCTAN2(X/Y)
//XYhyp2            - Output presenting Hypotenuse of X and Y
short GetATan2 (short AtanX, short AtanY)
{
    XYhyp2 = isqrt32(((long)AtanX*AtanX*c4DEC) + ((long)AtanY*AtanY*c4DEC));
    GetArcCos (((long)AtanX*(long)c6DEC) /(long) XYhyp2);
    
    if (AtanY < 0)                // removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));  
        Atan4 = -AngleRad4;
    else
        Atan4 = AngleRad4;
    return Atan4;
}  


//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1     - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
void GetSinCos(short x)
{
  float val = PI * x / 1800.0;
  
  //sin4 = 10000 * sin(val);
  cos4 = 10000 * cos(val);
}
 
short ApproxSin(short x)
{
  if (x < 0)
  {
    x *= -1;
  }
  float val = x / 10.0;
  
  sin4 = (short) 10000 * 4 * x * (180 - x) / (4050 - x * (180 - x));
  
  return sin4;
}
void ApproxCos(short x)
{
  if (x > 900)
  {
   cos4 = -1 * ApproxSin(900 - x);
  return; 
  }
  
  cos4 = ApproxSin(x - 900);
  return;
}


//--------------------------------------------------------------------
//(GetARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//x        - Input Cosinus
//AngleRad4     - Output Angle in AngleRad4
long GetArcCos(short x)
{
  float angle = x / 10000.0;
  
  return (long) 100000 * acos(angle);
}  

unsigned long isqrt32 (unsigned long n) 
{
        unsigned long root;
        unsigned long remainder;
        unsigned long  place;

        root = 0;
        remainder = n;
        place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

        while (place > remainder)
        place = place >> 2;
        while (place)
        {
                if (remainder >= root + place)
                {
                        remainder = remainder - root - place;
                        root = root + (place << 1);
                }
                root = root >> 1;
                place = place >> 2;
        }
        return root;
}
