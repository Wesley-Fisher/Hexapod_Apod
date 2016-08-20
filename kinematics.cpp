#include <Arduino.h>
#include "hexPhysicalDimensions.h"
#include "mathfunc.h"
#include "kinematics.h"
#include "Hex_Globals.h"
#include "hexLegAngles.h"
#include "globalVariables.h"

short           PosX;            //Input position of the feet X
short           PosZ;            //Input position of the feet Z
short           PosY;            //Input position of the feet Y
//long          TotalX;            //Total X distance between the center of the body and the feet
//long          TotalZ;            //Total Z distance between the center of the body and the feet
long            BodyFKPosX;        //Output Position X of feet with Rotation
long            BodyFKPosY;        //Output Position Y of feet with Rotation
long            BodyFKPosZ;        //Output Position Z of feet with Rotation
// New with zentas stuff
short           BodyRotOffsetX;    //Input X offset value to adjust centerpoint of rotation
short           BodyRotOffsetY;    //Input Y offset value to adjust centerpoint of rotation
short           BodyRotOffsetZ;    //Input Z offset value to adjust centerpoint of rotation


//[Balance]
long            TotalTransX;
long            TotalTransZ;
long            TotalTransY;
long            TotalYBal1;
long            TotalXBal1;
long            TotalZBal1;

//Leg Inverse Kinematics
long            IKFeetPosX;        //Input position of the Feet X
long            IKFeetPosY;        //Input position of the Feet Y
long            IKFeetPosZ;        //Input Position of the Feet Z
boolean         IKSolution;        //Output true if the solution is possible
boolean         IKSolutionWarning;    //Output true if the solution is NEARLY possible
boolean         IKSolutionError;    //Output true if the solution is NOT possible


//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS) 
//BodyRotX         - Global Input pitch of the body 
//BodyRotY         - Global Input rotation of the body 
//BodyRotZ         - Global Input roll of the body 
//RotationY         - Input Rotation for the gait 
//PosX            - Input position of the feet X 
//PosZ            - Input position of the feet Z 
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//BodyFKPosX         - Output Position X of feet with Rotation 
//BodyFKPosY         - Output Position Y of feet with Rotation 
//BodyFKPosZ         - Output Position Z of feet with Rotation
void BodyFK(short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg)
{
	short            SinA4;          //Sin buffer for BodyRotX calculations
	short            CosA4;          //Cos buffer for BodyRotX calculations
	short            SinB4;          //Sin buffer for BodyRotX calculations
	short            CosB4;          //Cos buffer for BodyRotX calculations
	short            SinG4;          //Sin buffer for BodyRotZ calculations
	short            CosG4;          //Cos buffer for BodyRotZ calculations
	short             CPR_X;            //Final X value for centerpoint of rotation
	short            CPR_Y;            //Final Y value for centerpoint of rotation
	short            CPR_Z;            //Final Z value for centerpoint of rotation

	//Calculating totals from center of the body to the feet 
	CPR_X = (short)pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX + BodyRotOffsetX;
	CPR_Y = PosY + BodyRotOffsetY;         //Define centerpoint for rotation along the Y-axis
	CPR_Z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ + BodyRotOffsetZ;

	//Successive global rotation matrix: 
	//Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate 
	//Sinus Alfa = SinA, cosinus Alfa = cosA. and so on... 

	//First calculate sinus and cosinus for each rotation: 
	GetSinCos(g_InControlState.BodyRot1.x + TotalXBal1);
	SinG4 = sin4;
	CosG4 = cos4;

	GetSinCos(g_InControlState.BodyRot1.z + TotalZBal1);
	SinB4 = sin4;
	CosB4 = cos4;

	GetSinCos(g_InControlState.BodyRot1.y + (RotationY*c1DEC) + TotalYBal1);
	SinA4 = sin4;
	CosA4 = cos4;

	//Calcualtion of rotation matrix: 
	BodyFKPosX = ((long)CPR_X*c2DEC - ((long)CPR_X*c2DEC*CosA4 / c4DEC*CosB4 / c4DEC - (long)CPR_Z*c2DEC*CosB4 / c4DEC*SinA4 / c4DEC
		+ (long)CPR_Y*c2DEC*SinB4 / c4DEC)) / c2DEC;
	BodyFKPosZ = ((long)CPR_Z*c2DEC - ((long)CPR_X*c2DEC*CosG4 / c4DEC*SinA4 / c4DEC + (long)CPR_X*c2DEC*CosA4 / c4DEC*SinB4 / c4DEC*SinG4 / c4DEC
		+ (long)CPR_Z*c2DEC*CosA4 / c4DEC*CosG4 / c4DEC - (long)CPR_Z*c2DEC*SinA4 / c4DEC*SinB4 / c4DEC*SinG4 / c4DEC
		- (long)CPR_Y*c2DEC*CosB4 / c4DEC*SinG4 / c4DEC)) / c2DEC;
	BodyFKPosY = ((long)CPR_Y  *c2DEC - ((long)CPR_X*c2DEC*SinA4 / c4DEC*SinG4 / c4DEC - (long)CPR_X*c2DEC*CosA4 / c4DEC*CosG4 / c4DEC*SinB4 / c4DEC
		+ (long)CPR_Z*c2DEC*CosA4 / c4DEC*SinG4 / c4DEC + (long)CPR_Z*c2DEC*CosG4 / c4DEC*SinA4 / c4DEC*SinB4 / c4DEC
		+ (long)CPR_Y*c2DEC*CosB4 / c4DEC*CosG4 / c4DEC)) / c2DEC;
}



//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError    - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr)
{
	unsigned long    IKSW2;            //Length between Shoulder and Wrist, decimals = 2
	unsigned long    IKA14;            //Angle of the line S>W with respect to the ground in radians, decimals = 4
	unsigned long    IKA24;            //Angle of the line S>W with respect to the femur in radians, decimals = 4
	short            IKFeetPosXZ;    //Diagonal direction from Input X and Z
#ifdef c4DOF
	// these were shorts...
	long            TarsOffsetXZ;    //Vector value \ ;
	long            TarsOffsetY;     //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
	long            TarsToGroundAngle1;    //Angle between tars and ground. Note: the angle are 0 when the tars are perpendicular to the ground
	long            TGA_A_H4;
	long            TGA_B_H3;
#else
#define TarsOffsetXZ 0		// Vector value
#define TarsOffsetY  0		//Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
#endif


	long            Temp1;
	long            Temp2;
	long            T3;

	//Calculate IKCoxaAngle and IKFeetPosXZ
	GetATan2(IKFeetPosX, IKFeetPosZ);
	CoxaAngle1[LegIKLegNr] = (((long)Atan4 * 180) / 3141) + (short)pgm_read_word(&cCoxaAngle1[LegIKLegNr]);

	//Length between the Coxa and tars [foot]
	IKFeetPosXZ = XYhyp2 / c2DEC;
#ifdef c4DOF
	// Some legs may have the 4th DOF and some may not, so handle this here...
	//Calc the TarsToGroundAngle1:
	if ((byte)pgm_read_byte(&cTarsLength[LegIKLegNr])) {    // We allow mix of 3 and 4 DOF legs...
		TarsToGroundAngle1 = -cTarsConst + cTarsMulti*IKFeetPosY + ((long)(IKFeetPosXZ*cTarsFactorA)) / c1DEC - ((long)(IKFeetPosXZ*IKFeetPosY) / (cTarsFactorB));
		if (IKFeetPosY < 0)     //Always compensate TarsToGroundAngle1 when IKFeetPosY it goes below zero
			TarsToGroundAngle1 = TarsToGroundAngle1 - ((long)(IKFeetPosY*cTarsFactorC) / c1DEC);     //TGA base, overall rule
		if (TarsToGroundAngle1 > 400)
			TGA_B_H3 = 200 + (TarsToGroundAngle1 / 2);
		else
			TGA_B_H3 = TarsToGroundAngle1;

		if (TarsToGroundAngle1 > 300)
			TGA_A_H4 = 240 + (TarsToGroundAngle1 / 5);
		else
			TGA_A_H4 = TarsToGroundAngle1;

		if (IKFeetPosY > 0)    //Only compensate the TarsToGroundAngle1 when it exceed 30 deg (A, H4 PEP note)
			TarsToGroundAngle1 = TGA_A_H4;
		else if (((IKFeetPosY <= 0) & (IKFeetPosY > -10))) // linear transition between case H3 and H4 (from PEP: H4-K5*(H3-H4))
			TarsToGroundAngle1 = (TGA_A_H4 - (((long)IKFeetPosY*(TGA_B_H3 - TGA_A_H4)) / c1DEC));
		else                //IKFeetPosY <= -10, Only compensate TGA1 when it exceed 40 deg
			TarsToGroundAngle1 = TGA_B_H3;

		//Calc Tars Offsets:
		GetSinCos(TarsToGroundAngle1);
		TarsOffsetXZ = ((long)sin4*(byte)pgm_read_byte(&cTarsLength[LegIKLegNr])) / c4DEC;
		TarsOffsetY = ((long)cos4*(byte)pgm_read_byte(&cTarsLength[LegIKLegNr])) / c4DEC;
	}
	else {
		TarsOffsetXZ = 0;
		TarsOffsetY = 0;
	}
#endif

	//Using GetAtan2 for solving IKA1 and IKSW
	//IKA14 - Angle between SW line and the ground in radians
	IKA14 = GetATan2(IKFeetPosY - TarsOffsetY, IKFeetPosXZ - (byte)pgm_read_byte(&cCoxaLength[LegIKLegNr]) - TarsOffsetXZ);

	//IKSW2 - Length between femur axis and tars
	IKSW2 = XYhyp2;

	//IKA2 - Angle of the line S>W with respect to the femur in radians
	Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[LegIKLegNr])*(byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) - ((long)(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr])*(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr])))*c4DEC + ((long)IKSW2*IKSW2));
	Temp2 = (long)(2 * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr]))*c2DEC * (unsigned long)IKSW2;
	T3 = Temp1 / (Temp2 / c4DEC);
	IKA24 = GetArcCos(T3);
	//IKFemurAngle
	FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + CFEMURHORNOFFSET1(LegIKLegNr);

	//IKTibiaAngle
	Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[LegIKLegNr])*(byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) + ((long)(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr])*(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr])))*c4DEC - ((long)IKSW2*IKSW2));
	Temp2 = (2 * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr])*(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]));
	GetArcCos(Temp1 / Temp2);
	TibiaAngle1[LegIKLegNr] = -(900 - (long)AngleRad4 * 180 / 3141);

#ifdef c4DOF
	//Tars angle
	if ((byte)pgm_read_byte(&cTarsLength[LegIKLegNr])) {    // We allow mix of 3 and 4 DOF legs...
		TarsAngle1[LegIKLegNr] = (TarsToGroundAngle1 + FemurAngle1[LegIKLegNr] - TibiaAngle1[LegIKLegNr])
			+ CTARSHORNOFFSET1(LegIKLegNr);
	}
#endif

	//Set the Solution quality    
	if (IKSW2 < ((byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) + (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]) - 30)*c2DEC)
		IKSolution = 1;
	else
	{
		if (IKSW2 < ((byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) + (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]))*c2DEC)
			IKSolutionWarning = 1;
		else
			IKSolutionError = 1;
	}
}
